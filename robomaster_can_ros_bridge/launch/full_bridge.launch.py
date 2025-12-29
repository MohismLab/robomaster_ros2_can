#!/usr/bin/env python3
"""
RoboMaster Full Bridge Launch File

This launch file starts both controlling and sensing nodes for complete
RoboMaster CAN bridge functionality.

Controlling Node:
    - Subscribes: /namespace/cmd_vel, /namespace/cmd_wheels
    - Services: /namespace/led, /namespace/emergency_stop
    
Sensing Node:
    - Publishes: /namespace/odom, /namespace/imu, /namespace/battery, etc.

Usage:
    ros2 launch robomaster_can_ros_bridge full_bridge.launch.py
    ros2 launch robomaster_can_ros_bridge full_bridge.launch.py namespace:=rm0
    ros2 launch robomaster_can_ros_bridge full_bridge.launch.py robot_id:=2
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for RoboMaster bridge (controlling + sensing)"""
    
    # Resolve robot_id and namespace from environment variables
    # Priority: ROBOT_NAMESPACE > ROBOT_ID (as rm<ROBOT_ID>)
    robot_id = os.getenv("ROBOT_ID", "rm_0").strip()
    namespace_value = f"{robot_id}"
    # --- Launch Arguments ---
    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=namespace_value,
        description="ROS2 namespace for the nodes"
    )
    
    robot_id_arg = DeclareLaunchArgument(
        "robot_id",
        default_value=robot_id,
        description="Robot ID from environment variable"
    )
    
    can_interface_arg = DeclareLaunchArgument(
        "can_interface",
        default_value="can0",
        description="CAN interface name (e.g., can0, can1)"
    )
    
    xspeed_limit_arg = DeclareLaunchArgument(
        "xspeed_limit",
        default_value="1.0",
        description="Maximum linear speed in X direction (m/s)"
    )
    
    yspeed_limit_arg = DeclareLaunchArgument(
        "yspeed_limit",
        default_value="1.0",
        description="Maximum linear speed in Y direction (m/s)"
    )
    
    angular_speed_limit_arg = DeclareLaunchArgument(
        "angular_speed_limit",
        default_value="5.0",
        description="Maximum angular speed (rad/s)"
    )
    
    # --- Log namespace info ---
    log_namespace = LogInfo(
        msg=["RoboMaster Bridge starting with namespace: ", LaunchConfiguration("namespace")]
    )
    
    # --- Controlling Node ---
    controlling_node = Node(
        package="robomaster_can_ros_bridge",
        executable="controlling",
        namespace=LaunchConfiguration("namespace"),
        name="robomaster_controlling",
        output="screen",
        emulate_tty=True,
        parameters=[
            # Add parameters here if C++ code is updated to use them
            # {"can_interface": LaunchConfiguration("can_interface")},
        ],
    )
    
    # --- Sensing Node ---
    sensing_node = Node(
        package="robomaster_can_ros_bridge",
        executable="sensing",
        namespace=LaunchConfiguration("namespace"),
        name="robomaster_sensing",
        output="screen",
        emulate_tty=True,
        parameters=[
            # Add parameters here if C++ code is updated to use them
        ],
    )

    # # --- Joy Node (reads joystick input) ---
    # joy_node = Node(
    #     package="joy",
    #     executable="joy_node",
    #     name="joy_node",
    #     parameters=[{
    #         "device_id": 0,
    #         "deadzone": 0.05,
    #         "autorepeat_rate": 20.0,
    #     }],
    #     output="screen",
    # )
    
    # # --- Joy Teleop Node (converts joystick to robot commands) ---
    # joy_teleop_node = Node(
    #     package="robomaster_joy",
    #     executable="joy_teleop",
    #     name="joy_teleop",
    #     parameters=[{
    #         "xspeed_limit": LaunchConfiguration("xspeed_limit"),
    #         "yspeed_limit": LaunchConfiguration("yspeed_limit"),
    #         "angular_speed_limit": LaunchConfiguration("angular_speed_limit"),
    #     }],
    #     output="screen",
    # )
    
    return LaunchDescription([
        # Launch arguments
        namespace_arg,
        robot_id_arg,
        can_interface_arg,
        xspeed_limit_arg,
        yspeed_limit_arg,
        angular_speed_limit_arg,
        
        # Logging
        log_namespace,
        
        # Nodes
        controlling_node,
        sensing_node,
        # joy_node,
        # joy_teleop_node,
    ])

