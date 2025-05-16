#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0

import os, yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def load_yaml(pkg, rel_path):
    file_path = os.path.join(get_package_share_directory(pkg), rel_path)
    with open(file_path, "r") as f:
        return yaml.safe_load(f)


def generate_launch_description():
    declare_use_sim = DeclareLaunchArgument(
        "use_sim_time", default_value="true",
        description="Use simulated /clock")
    use_sim_time= LaunchConfiguration("use_sim_time")

    xacro_file = os.path.join(
        get_package_share_directory("panda_description"),
        "robots", "panda_arm.urdf.xacro")

    robot_description = {
        "robot_description": Command([
            FindExecutable(name="xacro"), " ", xacro_file,
            " hand:=true",
            " use_sim_time:=", use_sim_time    # leave the model plug-in in place
        ])
    }

    srdf_file = os.path.join(
        get_package_share_directory("panda_moveit_config"),
        "srdf", "panda_arm.srdf.xacro")

    robot_description_semantic = {
        "robot_description_semantic": Command([
            FindExecutable(name="xacro"), " ", srdf_file,
            " hand:=true"])
    }

    kinematics_yaml    = load_yaml("panda_moveit_config", "config/kinematics.yaml")
    ompl_planning_yaml = load_yaml("panda_moveit_config", "config/ompl_planning.yaml")
    controllers_yaml   = load_yaml("panda_moveit_config", "config/panda_ros_controllers.yaml")

    params = {
        "robot_description_kinematics": kinematics_yaml,
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager":
            "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True
    }

    move_group = Node(
        package="moveit_ros_move_group", executable="move_group",
        parameters=[
            robot_description,
            robot_description_semantic,
            params,
            {"use_sim_time": use_sim_time},
            {"move_group": {
                "planning_plugins": ["ompl_interface/OMPLPlanner"],
                "request_adapters": [
                    "default_planning_request_adapters/CheckForStackedConstraints",
                    "default_planning_request_adapters/CheckStartStateBounds",
                    "default_planning_request_adapters/CheckStartStateCollision",
                    "default_planning_request_adapters/ResolveConstraintFrames",
                    "default_planning_request_adapters/ValidateWorkspaceBounds"],
                "start_state_max_bounds_error": 0.1
            }},
            ompl_planning_yaml or {}
        ],
        output="screen"
    )

    return LaunchDescription([
        declare_use_sim,
        SetParameter(name="use_sim_time", value=True),
        move_group,
    ])
