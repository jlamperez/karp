# Copyright 2022 Jorge Lamperez
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution,
    TextSubstitution)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="karp_description",
            description="Description package with robot URDF/xacro files.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            # default_value="karp.urdf.xacro",
            default_value="karp_system.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value='false',
            description="Start rviz with the launch file",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "joy_vel",
            default_value='cmd_vel',
            description="Joystick velocity command",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "joy_config",
            default_value='ps3',
            description="ps3 controller configuration file",
        )
    )

    joy_config = LaunchConfiguration("joy_config")

    declared_arguments.append(
        DeclareLaunchArgument(
            "joy_dev",
            default_value='/dev/input/js0',
            description="Joystick linux file",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "config_filepath",
            default_value=[TextSubstitution(text=os.path.join(
                get_package_share_directory('teleop_twist_joy'), 'config', '')),
                joy_config, TextSubstitution(text='.config.yaml')],
            description="Configuration file path, by default: \
                '/usr/share/teleop_twist_joy/config/ps3.config.yaml'",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_teleop",
            default_value="true",
            description="Start joystick teleop with the launch file",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value='false',
            description="Use Odrive hardware interface or generic fake hardware",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_velodyne",
            default_value="false",
            description="Starts jvelodyne lidar with the launch file",
        )
    )


    # Initialize Arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")
    launch_rviz = LaunchConfiguration("launch_rviz")
    joy_vel = LaunchConfiguration("joy_vel")
    joy_dev = LaunchConfiguration("joy_dev")
    config_filepath = LaunchConfiguration('config_filepath')
    launch_teleop = LaunchConfiguration("launch_teleop")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    launch_velodyne = LaunchConfiguration("launch_velodyne")

    driver_share_dir_1 = get_package_share_directory('velodyne_driver')
    driver_params_file_1 = os.path.join(driver_share_dir_1, 'config', 'vlp_1.yaml')

    convert_share_dir_1 = get_package_share_directory('velodyne_pointcloud')
    convert_params_file_1 = os.path.join(
        convert_share_dir_1, 'config', 'VLP16-velodyne_convert_node-params.yaml')
    with open(convert_params_file_1, 'r') as f:
        convert_params_1 = yaml.safe_load(f)['velodyne_convert_node']['ros__parameters']
    convert_params_1['calibration'] = os.path.join(convert_share_dir_1, 'params', 'VLP16db.yaml')

    laserscan_share_dir_1 = get_package_share_directory('velodyne_laserscan')
    laserscan_params_file_1 = os.path.join(
        laserscan_share_dir_1, 'config', 'default-velodyne_laserscan_node-params.yaml')

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ",
            "prefix:=",
            prefix,
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", "karp.rviz"]
    )

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("karp_bringup"),
            "config",
            "karp_controllers.yaml",
        ]
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(launch_rviz),
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        remappings=[
            ("/karp_base_controller/cmd_vel_unstamped", "/cmd_vel"),
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["karp_base_controller", "-c", "/controller_manager"],
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    joy_linux = Node(
        package='joy_linux',
        executable='joy_linux_node',
        parameters=[{
            'dev': joy_dev,
            'deadzone': 0.3,
            'autorepeat_rate': 20.0,
        }],
        condition=IfCondition(launch_teleop),
    )

    teleop_twist_joy = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        parameters=[config_filepath],
        remappings={('/cmd_vel', LaunchConfiguration('joy_vel'))},
        condition=IfCondition(launch_teleop),
    )

    velodyne_driver_node_1 = Node(
        package="velodyne_driver",
        executable="velodyne_driver_node",
        name="velodyne_driver_node_1",
        parameters=[driver_params_file_1],
        remappings=[("velodyne_packets", "velodyne_packets_1")],
        condition=IfCondition(launch_velodyne),
    )

    velodyne_convert_node_1 = Node(
        package="velodyne_pointcloud",
        executable="velodyne_convert_node",
        name="velodyne_convert_node_1",
        parameters=[convert_params_1],
        remappings=[
        ("velodyne_packets", "velodyne_packets_1"),
        ("velodyne_points", "velodyne_points_1")],
        condition=IfCondition(launch_velodyne),
    )

    velodyne_laserscan_node_1 = Node(
        package="velodyne_laserscan",
        executable="velodyne_laserscan_node",
        name="velodyne_laserscan_node_1",
        parameters=[laserscan_params_file_1],
        remappings=[
        ("velodyne_points", "velodyne_points_1"),
        ("scan", "scan_1")],
        condition=IfCondition(launch_velodyne),
    )

    nodes = [
        joy_linux,
        teleop_twist_joy,
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        velodyne_driver_node_1,
        velodyne_convert_node_1,
        velodyne_laserscan_node_1,
    ]

    return LaunchDescription(declared_arguments + nodes)