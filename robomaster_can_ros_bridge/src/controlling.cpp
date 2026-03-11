#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/duration.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "robomaster_msgs/srv/led.hpp"
#include "robomaster_msgs/msg/wheel_speed.hpp"
#include "emergency_stop_msgs/srv/emergency_stop.hpp"
#include "std_msgs/msg/bool.hpp"

#include "chassis.hpp"
#include "led.hpp"
#include "can_streambuf.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class RoboMasterControlling : public rclcpp::Node
{
public:
    RoboMasterControlling()
        : Node("robomaster_controlling")
        , last_led_request_{}
        , emerg_led_request_{}
        , emergency_stopped_{false}
        , chassis_workmode_enabled_{false}
        , has_joy_msg_{false}
        , has_auto_msg_{false}
        , joy_was_active_{false}
        , joy_timeout_ms_{200}
        , joy_to_auto_hold_ms_{500}     // Hold still for 2 seconds when switching from joy to auto (unit: milliseconds)
        , linear_zero_threshold_{0.05}   // Linear velocity threshold (m/s)
        , angular_zero_threshold_{0.1}   // Angular velocity threshold (rad/s)
        , _can_streambuf{"can0", 0x201}
        , _can_iostream{&_can_streambuf}
        , _rm_chassis{_can_iostream}
        , _rm_led{_can_iostream}
    {
        // Subscribe to joy_cmd_vel (manual control - higher priority)
        joy_cmd_vel_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
                "joy_cmd_vel",
                rclcpp::SensorDataQoS(),
                std::bind(&RoboMasterControlling::joy_cmd_vel_callback, this, _1)
            );

        // Subscribe to cmd_vel (autonomous control - lower priority)
        speed_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
                "cmd_vel",
                rclcpp::SensorDataQoS(),
                std::bind(&RoboMasterControlling::cmd_vel_callback, this, _1)
            );
        wheel_speed_subscription_ = create_subscription<robomaster_msgs::msg::WheelSpeed>(
                "cmd_wheels",
                rclcpp::SensorDataQoS(),
                std::bind(&RoboMasterControlling::topic_callback_wheel_speed, this, _1)
            );
        timer_heartbeat_ = rclcpp::create_timer(
                this,
                get_clock(),
                rclcpp::Duration(0, 10 * 1e6),
                std::bind(&robomaster::command::chassis::send_heartbeat, &_rm_chassis)
            );
        timer_watchdog_ = rclcpp::create_timer(
                this,
                get_clock(),
                rclcpp::Duration(0, 200 * 1e6),
                std::bind(&RoboMasterControlling::timer_watchdog_callback, this)
            );

        led_service_ = create_service<robomaster_msgs::srv::LED>(
                "led",
                std::bind(&RoboMasterControlling::led_service_callback, this, _1, _2)
            );
        emergency_stop_service_ = create_service<emergency_stop_msgs::srv::EmergencyStop>(
                "emergency_stop",
                std::bind(&RoboMasterControlling::emergency_stop_service_callback, this, _1, _2)
            );
        emergency_stop_subscription_ = create_subscription<std_msgs::msg::Bool>(
                "/emergency_stop",
                rclcpp::QoS(10).reliable(),
                std::bind(&RoboMasterControlling::emergency_stop_topic_callback, this, _1)
            );
        RCLCPP_INFO(this->get_logger(), "Subscribed to global /emergency_stop topic");

        // Initialize time variables with current time to avoid time source mismatch
        last_joy_time_ = this->now();
        joy_release_time_ = this->now();

        emerg_led_request_.r = 255;
        emerg_led_request_.mode = 1;

        last_led_request_.mode = 1;
        last_led_request_.r = 127;
        last_led_request_.g = 127;
        last_led_request_.b = 127;
        update_leds(last_led_request_);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initialized");
    }

private:
    /**
     * Check if a Twist message has any non-zero values
     * Uses separate thresholds for linear and angular velocities
     */
    bool is_non_zero(const geometry_msgs::msg::Twist& msg) const
    {
        return (std::abs(msg.linear.x) > linear_zero_threshold_ ||
                std::abs(msg.linear.y) > linear_zero_threshold_ ||
                std::abs(msg.linear.z) > linear_zero_threshold_ ||
                std::abs(msg.angular.x) > angular_zero_threshold_ ||
                std::abs(msg.angular.y) > angular_zero_threshold_ ||
                std::abs(msg.angular.z) > angular_zero_threshold_);
    }

    /**
     * Send velocity command to the robot
     */
    void send_velocity_command(const geometry_msgs::msg::Twist& msg, const std::string& source)
    {
        if (!emergency_stopped_)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "send_speed from %s", source.c_str());
            // by syy, reverse to unify with ros2 twist.
            _rm_chassis.send_speed(-msg.linear.x, -msg.linear.y, -msg.angular.z * 180.0 / 3.1415);
        }

        reset_watchdog();
    }

    /**
     * Callback for joy_cmd_vel (manual control)
     */
    void joy_cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        last_joy_msg_ = *msg;
        last_joy_time_ = this->now();
        has_joy_msg_ = true;

        bool joy_is_nonzero = is_non_zero(*msg);

        // Detect transition from active joy to inactive (for hold period)
        if (joy_was_active_ && !joy_is_nonzero)
        {
            // Joy just released, record the time for hold period
            joy_release_time_ = this->now();
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Joy released, starting hold period");
        }
        joy_was_active_ = joy_is_nonzero;

        // Check if joy command is non-zero
        if (joy_is_nonzero)
        {
            // Joy is active and non-zero, use it (highest priority)
            send_velocity_command(*msg, "joy");
        }
        else
        {
            // Joy is zero, check if we're in hold period
            auto time_since_release = (this->now() - joy_release_time_).seconds() * 1000.0;
            if (time_since_release < joy_to_auto_hold_ms_)
            {
                // In hold period, send zero velocity
                geometry_msgs::msg::Twist zero_cmd;
                send_velocity_command(zero_cmd, "hold (joy->auto)");
            }
            else if (has_auto_msg_)
            {
                // Hold period passed, fall back to cmd_vel
                send_velocity_command(last_auto_msg_, "cmd_vel (joy=0)");
            }
            else
            {
                // No auto message, send zero
                send_velocity_command(*msg, "joy (zero)");
            }
        }
    }

    /**
     * Callback for cmd_vel (autonomous control)
     */
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        last_auto_msg_ = *msg;
        has_auto_msg_ = true;

        // Check if joy is still active (recent and non-zero)
        bool joy_is_active = false;
        if (has_joy_msg_)
        {
            auto time_since_joy = (this->now() - last_joy_time_).seconds() * 1000.0;
            if (time_since_joy < joy_timeout_ms_)
            {
                joy_is_active = is_non_zero(last_joy_msg_);
            }
        }

        // Check if we're in the hold period after joy release
        bool in_hold_period = false;
        if (joy_was_active_ == false && has_joy_msg_)
        {
            auto time_since_release = (this->now() - joy_release_time_).seconds() * 1000.0;
            if (time_since_release < joy_to_auto_hold_ms_)
            {
                in_hold_period = true;
            }
        }

        if (joy_is_active)
        {
            RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Ignoring cmd_vel, joy is active");
        }
        else if (in_hold_period)
        {
            // In hold period, send zero velocity instead of cmd_vel
            geometry_msgs::msg::Twist zero_cmd;
            send_velocity_command(zero_cmd, "hold (joy->auto)");
        }
        else
        {
            // Joy is not active and hold period passed, use autonomous control
            send_velocity_command(*msg, "cmd_vel");
        }
    }

    void topic_callback_wheel_speed(const robomaster_msgs::msg::WheelSpeed::SharedPtr msg)
    {
        if (!emergency_stopped_)
        {
            _rm_chassis.send_wheel_speed(msg->fr, msg->fl, msg->rl, msg->rr);
        }

        reset_watchdog();
    }

    void led_service_callback(const std::shared_ptr<robomaster_msgs::srv::LED::Request> request,
        std::shared_ptr<robomaster_msgs::srv::LED::Response>      response)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\nr: %d g: %d b: %d",
            request->r, request->g, request->b);

        last_led_request_ = *request;
        update_leds(last_led_request_);
        response->success = true;
    }

    void emergency_stop_service_callback(const std::shared_ptr<emergency_stop_msgs::srv::EmergencyStop::Request> request,
        std::shared_ptr<emergency_stop_msgs::srv::EmergencyStop::Response>      response)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Emergency stop request %d", request->stop);
        handle_emergency_stop(request->stop);
        response->success = true;
    }

    void emergency_stop_topic_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Emergency stop topic received: %s", msg->data ? "STOP" : "RESUME");
        handle_emergency_stop(msg->data);
    }

    void handle_emergency_stop(bool stop)
    {
        emergency_stopped_ = stop;

        if (stop)
        {
            _rm_chassis.send_wheel_speed(0, 0, 0, 0);
            update_leds(emerg_led_request_);
        }
        else
        {
            update_leds(last_led_request_);
        }
    }

    void timer_watchdog_callback(void)
    {
        if (chassis_workmode_enabled_)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Watchdog triggered");
            _rm_chassis.send_workmode(0);
            chassis_workmode_enabled_ = false;
        }

        _rm_chassis.send_wheel_speed(0, 0, 0, 0);
    }

    void update_leds(const robomaster_msgs::srv::LED::Request& request)
    {
        _rm_led.send_led(request.mode, request.r, request.g, request.b, request.speed_up, request.speed_down, 0x3F);
    }

    void reset_watchdog()
    {
        if (!chassis_workmode_enabled_)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Watchdog reset");
            _rm_chassis.send_workmode(1);
            chassis_workmode_enabled_ = true;
        }

        timer_watchdog_->reset();
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr joy_cmd_vel_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr speed_subscription_;
    rclcpp::Subscription<robomaster_msgs::msg::WheelSpeed>::SharedPtr wheel_speed_subscription_;
    rclcpp::Service<robomaster_msgs::srv::LED>::SharedPtr led_service_;
    rclcpp::Service<emergency_stop_msgs::srv::EmergencyStop>::SharedPtr emergency_stop_service_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_subscription_;
    rclcpp::TimerBase::SharedPtr timer_heartbeat_;
    rclcpp::TimerBase::SharedPtr timer_watchdog_;
    robomaster_msgs::srv::LED::Request last_led_request_;
    robomaster_msgs::srv::LED::Request emerg_led_request_;
    bool emergency_stopped_;
    bool chassis_workmode_enabled_;

    // Command velocity multiplexing
    geometry_msgs::msg::Twist last_joy_msg_;
    geometry_msgs::msg::Twist last_auto_msg_;
    rclcpp::Time last_joy_time_;
    rclcpp::Time joy_release_time_;          // Time when joy was released (became zero)
    bool has_joy_msg_;
    bool has_auto_msg_;
    bool joy_was_active_;                    // Track if joy was active in previous message
    double joy_timeout_ms_;
    double joy_to_auto_hold_ms_;             // Hold period when switching from joy to auto
    double linear_zero_threshold_;   // Threshold for linear velocity (m/s)
    double angular_zero_threshold_;  // Threshold for angular velocity (rad/s)

    can_streambuf _can_streambuf;
    std::iostream _can_iostream;
    robomaster::command::chassis _rm_chassis;
    robomaster::command::led _rm_led;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoboMasterControlling>());
    rclcpp::shutdown();
    return 0;
}
