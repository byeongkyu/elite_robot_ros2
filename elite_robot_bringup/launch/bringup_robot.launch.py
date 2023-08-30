from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, LifecycleNode
from launch.event_handlers import OnProcessExit, OnExecutionComplete, OnProcessStart
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals

import os

def generate_launch_description():
    prefix = DeclareLaunchArgument("prefix", default_value="")
    robot_model = DeclareLaunchArgument("robot_model", default_value="cs66")
    robot_ip = DeclareLaunchArgument("robot_ip", default_value="127.0.0.1")
    dashboard_port = DeclareLaunchArgument("dashboard_port", default_value="29999")
    controller_port = DeclareLaunchArgument("controller_port", default_value="30001")
    rtsi_port = DeclareLaunchArgument("rtsi_port", default_value="30004")

    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('elite_robot_description'),
            'urdf/robot.urdf.xacro',
        ]),
        ' is_sim:=', 'false',
        ' prefix:=', LaunchConfiguration('prefix'),
        ' robot_model:=', LaunchConfiguration('robot_model'),
        ' robot_ip:=', LaunchConfiguration('robot_ip'),
        ' dashboard_port:=', LaunchConfiguration('dashboard_port'),
        ' controller_port:=', LaunchConfiguration('controller_port'),
        ' rtsi_port:=', LaunchConfiguration('rtsi_port'),


    ])
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution([
            FindPackageShare('elite_robot_bringup'),
            "config",
            "cs66_controllers.yaml"
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            robot_controllers
            ],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        on_exit=Shutdown(),
    )

    upload_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('elite_robot_description'),
            '/launch/upload_robot.launch.py']
        ),
        launch_arguments = {
            'use_simtime': 'false',

        }.items()
    )

    return LaunchDescription([
        prefix,
        robot_model,
        robot_ip,
        dashboard_port,
        controller_port,
        rtsi_port,
        upload_robot,
        control_node,
    ])

