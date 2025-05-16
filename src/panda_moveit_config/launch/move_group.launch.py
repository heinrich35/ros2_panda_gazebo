# Copyright (c) 2023 Franka Robotics GmbH
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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node, SetParameter
# XML includes are provided via the launch_xml package
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

import yaml


def load_yaml(package_name: str, file_path: str):
    """
    Load a YAML file from a ROS2 package (returns dict or None).
    """
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_path, 'r') as f:
            return yaml.safe_load(f)
    except EnvironmentError:
        return None


def generate_launch_description():
    # Declare launch arguments
    declares = [
        DeclareLaunchArgument('robot_ip', default_value='127.0.0.1', description='IP of the Franka robot'),
        DeclareLaunchArgument('use_fake_hardware', default_value='false', description='Use fake hardware interface'),
        DeclareLaunchArgument('fake_sensor_commands', default_value='false', description='Publish fake sensor data'),
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time and interfaces'),
        DeclareLaunchArgument('load_gripper', default_value='true', description='Load the Franka gripper'),
        DeclareLaunchArgument('db', default_value='False', description='Database flag'),
    ]

    # Launch configurations
    robot_ip = LaunchConfiguration('robot_ip')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    fake_sensor_commands = LaunchConfiguration('fake_sensor_commands')
    use_sim_time= LaunchConfiguration('use_sim_time')
    load_gripper = LaunchConfiguration('load_gripper')

    # Robot description (URDF) from xacro
    franka_xacro = os.path.join(
        get_package_share_directory('panda_description'),
        'robots', 'panda_arm.urdf.xacro'
    )
    robot_description_config = Command([
        FindExecutable(name='xacro'), ' ', franka_xacro,
        ' hand:=', load_gripper,
        ' robot_ip:=', robot_ip,
        ' use_fake_hardware:=', use_fake_hardware,
        ' fake_sensor_commands:=', fake_sensor_commands,
        ' use_sim_time:=', use_sim_time,
    ])
    robot_description = {'robot_description': robot_description_config}

    # Semantic description (SRDF)
    franka_semantic = os.path.join(
        get_package_share_directory('panda_moveit_config'),
        'srdf', 'panda_arm.srdf.xacro'
    )
    robot_description_semantic_config = Command([
        FindExecutable(name='xacro'), ' ', franka_semantic,
        ' hand:=', load_gripper
    ])
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_config}

    # Kinematics parameters
    kinematics_yaml = load_yaml('panda_moveit_config', 'config/kinematics.yaml') or {}

    # OMPL planning pipeline parameters
    ompl_yaml = load_yaml('panda_moveit_config', 'config/ompl_planning.yaml') or {}
    # Default to OMPL pipeline
    default_pipeline = {'default_planning_pipeline': 'ompl'}

    # MoveGroup node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_yaml,
            default_pipeline,
        ],
    )

    # Include planning context (loads URDF/SRDF)
    planning_context = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('panda_moveit_config'),
                'launch', 'moveit.launch.py'
            )
        ),
        launch_arguments={
            'load_robot_description': 'true',
            'load_gripper': load_gripper,
            'arm_id': 'panda'
        }.items()
    )

    # Include OMPL planning pipeline
    planning_pipeline_ompl = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('panda_moveit_config'),
                'launch', 'planning_pipeline.launch.xml'
            )
        ),
        launch_arguments={
            'pipeline': 'ompl',
            'arm_id': 'panda'
        }.items()
    )

    ld = LaunchDescription()
    # Add declarations
    for decl in declares:
        ld.add_action(decl)
    # Add XML includes
    ld.add_action(planning_context)
    ld.add_action(planning_pipeline_ompl)
    # Add the MoveGroup node and simulation time
    ld.add_action(move_group_node)
    ld.add_action(SetParameter(name='use_sim_time', value=use_sim_time))

    return ld
