#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
#
# Gazebo + Panda (ros2_control) + MoveIt² launch file
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
    # ─────────────────────────── launch-time arguments ───────────────────────────
    declare_load_gripper = DeclareLaunchArgument(
        "load_gripper", default_value="true",
        description="Attach the Franka Hand (gripper)",
    )
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true",
        description="Use simulated time from Gazebo (/clock)",
    )
    declare_world_file = DeclareLaunchArgument(
        "world_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("gym_world"), "worlds", "gym.sdf"]
        ),
        description="SDF world file (must NOT embed ros2_control)",
    )

    #  NEW – PID gain overrides  -------------------------------------------------
    declare_pos_kp = DeclareLaunchArgument(
        "pos_kp", default_value="400.0",
        description="P gain for the joint position PID loop",
    )
    declare_pos_ki = DeclareLaunchArgument(
        "pos_ki", default_value="0.0",
        description="I gain for the joint position PID loop",
    )
    declare_pos_kd = DeclareLaunchArgument(
        "pos_kd", default_value="400.0",
        description="D gain for the joint position PID loop",
    )
    declare_pos_max_ie = DeclareLaunchArgument(
        "pos_max_integral_error", default_value="1000.0",
        description="Clamp value for the integral term of the PID loop",
    )

    load_gripper            = LaunchConfiguration("load_gripper")
    use_sim_time            = LaunchConfiguration("use_sim_time")
    world_file              = LaunchConfiguration("world_file")
    pos_kp                  = LaunchConfiguration("pos_kp")
    pos_ki                  = LaunchConfiguration("pos_ki")
    pos_kd                  = LaunchConfiguration("pos_kd")
    pos_max_integral_error  = LaunchConfiguration("pos_max_integral_error")

    # ─────────────────────────── useful path substitutions ──────────────────────
    description_pkg  = "panda_description"
    description_file = "robots/panda_arm.urdf.xacro"

    controller_yaml = PathJoinSubstitution(
        [FindPackageShare("panda_gazebo"), "config", "gazebo_panda_controllers.yaml"]
    )

    # ───────────────────────────── Gazebo server + GUI ───────────────────────────
    gz_sim = ExecuteProcess(
        cmd=[
            "gz", "sim", "-v4",
            "-r", world_file,

            "--real-time-factor", "10.0",
        ],
        output="screen",
    )


    # ──────────────────────────── robot_description ─────────────────────────────
    robot_description_content = Command([
        FindExecutable(name="xacro"), " ",
        PathJoinSubstitution([FindPackageShare(description_pkg), description_file]), " ",
        # parameters for the URDF/xacro tree -----------------------------------
        "arm_id:=panda ",                       # keep existing parameter order
        "hand:=", load_gripper, " ",
        "use_sim_time:=", use_sim_time, " ",
        "simulation_controllers_config_file:=", controller_yaml, " ",
        #  NEW – PID gains -----------------------------------------------------
        "pos_kp:=", pos_kp, " ",
        "pos_ki:=", pos_ki, " ",
        "pos_kd:=", pos_kd, " ",
        "pos_max_integral_error:=", pos_max_integral_error,
    ])

    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
        output="screen",
    )

    # ───────────────────────────── spawn robot into Gazebo ──────────────────────
    gz_spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name", "panda",
            "-allow_renaming", "true",
            "-string", robot_description_content,
        ],
    )

    # ───────────────────────────── controller spawners ──────────────────────────
    spawner_jsb = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager", "--activate"],
        output="screen",
    )
    spawner_arm = Node(
        package="controller_manager", executable="spawner",
        arguments=["arm_controller", "-c", "/controller_manager", "--activate"],
        output="screen",
    )
    spawner_gripper = Node(
        package="controller_manager", executable="spawner",
        arguments=["panda_gripper", "-c", "/controller_manager", "--activate"],
        output="screen",
    )

    controller_loader = RegisterEventHandler(
        OnProcessExit(
            target_action=gz_spawn,
            on_exit=[TimerAction(
                period=2.0,
                actions=[spawner_jsb, spawner_arm, spawner_gripper],
            )],
        )
    )

    # ─────────────────────────────── /clock bridge ──────────────────────────────
    clock_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        name="gz_clock_bridge",
        output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # ─────────────────────────────── MoveIt² stack ──────────────────────────────
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("panda_moveit_config"),
                "launch", "moveit.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "load_gripper": load_gripper,
        }.items(),
    )

    # ─────────────────────────────── demo IK executor ───────────────────────────
    ik_executor = Node(
        package="panda_gazebo",
        executable="ik_executor.py",
        name="ik_executor",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # ───────────────────────────── assemble LaunchDescription ───────────────────
    return LaunchDescription([
        declare_load_gripper,
        declare_use_sim_time,
        declare_world_file,
        declare_pos_kp,
        declare_pos_ki,
        declare_pos_kd,
        declare_pos_max_ie,

        SetParameter(name="use_sim_time", value=use_sim_time),

        gz_sim,
        robot_state_publisher,
        gz_spawn,
        controller_loader,

        clock_bridge,
        moveit_launch,
        ik_executor,
    ])
