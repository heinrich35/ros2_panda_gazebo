#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration

class InitialPoseClient(Node):
    def __init__(self):
        super().__init__('initial_pose_client')

        # action name must match <controller_name>/follow_joint_trajectory
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )

        # Joint names + desired positions:
        self.joint_names = [f'panda_joint{i}' for i in range(1,8)]
        self.positions   = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]

        # Timer: try every second until the action server is up
        self._timer = self.create_timer(1.0, self._on_timer)

    def _on_timer(self):
        if not self._action_client.server_is_ready():
            self.get_logger().info('Waiting for FollowJointTrajectory server…')
            return
        self._timer.cancel()
        self._send_initial_goal()

    def _send_initial_goal(self):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names

        pt = JointTrajectoryPoint()
        pt.positions = self.positions
        pt.time_from_start = Duration(sec=1)  # 1-second “move”

        goal.trajectory.points = [pt]

        self.get_logger().info('Sending initial-trajectory goal')
        send_goal = self._action_client.send_goal_async(goal)
        send_goal.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Initial pose goal rejected 😕')
            rclpy.shutdown()
            return

        self.get_logger().info('Initial pose goal accepted 👍')
        result = goal_handle.get_result_async()
        result.add_done_callback(self._on_result)

    def _on_result(self, future):
        self.get_logger().info('Initial trajectory complete 🎉')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = InitialPoseClient()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
