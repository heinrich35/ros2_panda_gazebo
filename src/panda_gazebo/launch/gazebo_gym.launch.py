#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
#
# Gazebo + Panda (ros2_control) + MoveIt² launch file
#  (with velocity & acceleration feed-forward args)
#
import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
    RegisterEventHandler, TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    # ─── launch-time arguments ───
    declare_load_gripper = DeclareLaunchArgument(
        'load_gripper', default_value='true',
        description='Attach the Franka Hand (gripper)',
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulated time from Gazebo (/clock)',
    )
    declare_world_file = DeclareLaunchArgument(
        'world_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('gym_world'), 'worlds', 'gym.sdf'
        ]),
        description='SDF world file (must NOT embed ros2_control)',
    )

    declare_pos_kp = DeclareLaunchArgument('pos_kp', default_value='400.0', description='P gain for position')
    declare_pos_ki = DeclareLaunchArgument('pos_ki', default_value='0.0',   description='I gain for position')
    declare_pos_kd = DeclareLaunchArgument('pos_kd', default_value='40.0',  description='D gain for position')
    declare_pos_max_ie = DeclareLaunchArgument(
        'pos_max_integral_error', default_value='1000.0',
        description='Clamp for I-term')

    declare_ff_vel   = DeclareLaunchArgument('ff_vel',   default_value='1.0', description='Velocity feed-forward gain')
    declare_ff_accel = DeclareLaunchArgument('ff_accel', default_value='0.2', description='Acceleration feed-forward gain')

    load_gripper           = LaunchConfiguration('load_gripper')
    use_sim_time           = LaunchConfiguration('use_sim_time')
    world_file             = LaunchConfiguration('world_file')
    pos_kp                 = LaunchConfiguration('pos_kp')
    pos_ki                 = LaunchConfiguration('pos_ki')
    pos_kd                 = LaunchConfiguration('pos_kd')
    pos_max_integral_error = LaunchConfiguration('pos_max_integral_error')
    ff_vel                 = LaunchConfiguration('ff_vel')
    ff_accel               = LaunchConfiguration('ff_accel')

    #pkg_gym_world = get_package_share_directory('gym_world')
    description_pkg  = 'panda_description'
    description_file = 'robots/panda_arm.urdf.xacro'

    controller_yaml = PathJoinSubstitution([
        FindPackageShare('panda_gazebo'), 'config', 'gazebo_panda_controllers.yaml'
    ])

    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-v4', '-r', world_file],
        output='screen',
    )

    set_speed = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=['gz', 'physics', '-w', 'gym', '-s', '-r', '4.0'],
                output='screen',
            )
        ]
    )

    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([FindPackageShare(description_pkg), description_file]), ' ',
        'arm_id:=panda ',
        'hand:=', load_gripper, ' ',
        'use_sim_time:=', use_sim_time, ' ',
        'simulation_controllers_config_file:=', controller_yaml, ' ',
        'pos_kp:=', pos_kp, ' ',
        'pos_ki:=', pos_ki, ' ',
        'pos_kd:=', pos_kd, ' ',
        'pos_max_integral_error:=', pos_max_integral_error, ' ',
        'ff_vel:=', ff_vel, ' ',
        'ff_accel:=', ff_accel,
    ])

    robot_description = {'robot_description': robot_description_content}

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
        output='screen',
        arguments=['--ros-args', '--log-level', 'error'],
    )


    gz_spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=[
            '-name', 'panda', '-allow_renaming', 'true',
            '-string', robot_description_content,
            '-x', '-0.35', '-y', '0.0', '-z', '0.0',
        ],
    )

    spawner_jsb = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager', '--activate'],
        output='screen',
    )
    spawner_arm = Node(
        package='controller_manager', executable='spawner',
        arguments=['arm_controller', '-c', '/controller_manager', '--activate'],
        output='screen',
    )
    spawner_gripper = Node(
        package='controller_manager', executable='spawner',
        arguments=['panda_gripper', '-c', '/controller_manager', '--activate'],
        output='screen',
    )

    controller_loader = RegisterEventHandler(
        OnProcessExit(
            target_action=gz_spawn,
            on_exit=[TimerAction(period=2.0, actions=[spawner_jsb, spawner_arm, spawner_gripper])],
        )
    )

    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='gz_clock_bridge',
        output='screen', arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('panda_moveit_config'), 'launch', 'moveit.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time, 'load_gripper': load_gripper}.items(),
    )

    initial_pose_pub = Node(
        package='panda_gazebo', executable='initial_pose_pub.py',
        name='initial_pose_pub', output='screen', parameters=[{'use_sim_time': use_sim_time}],
    )
    """
    ik_executor = Node(
        package='panda_gazebo', executable='ik_executor.py',
        name='ik_executor', output='screen', parameters=[{'use_sim_time': use_sim_time}],
    )
        cartesian_keyboard_control = Node(
        package='panda_gazebo', executable='cartesian_keyboard_control.py',
        name='cartesian_keyboard_control', output='screen', parameters=[{'use_sim_time': use_sim_time}],
    )
    
    """
    """
    gym_world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gym_world, 'launch', 'gym_world.launch.py')
        ), launch_arguments={'use_sim_time': use_sim_time}.items(),
    )
    """
    return LaunchDescription([
        declare_load_gripper,
        declare_use_sim_time,
        declare_world_file,
        declare_pos_kp,
        declare_pos_ki,
        declare_pos_kd,
        declare_pos_max_ie,
        declare_ff_vel,
        declare_ff_accel,
        SetParameter(name='use_sim_time', value=use_sim_time),
        gz_sim,
        set_speed,
        robot_state_publisher,
        gz_spawn,
        controller_loader,
        clock_bridge,
        moveit_launch,
        #ik_executor,
        #cartesian_keyboard_control,
        #gym_world_launch,
        initial_pose_pub,
    ])
