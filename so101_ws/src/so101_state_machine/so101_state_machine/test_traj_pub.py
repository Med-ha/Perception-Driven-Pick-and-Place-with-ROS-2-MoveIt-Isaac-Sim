#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class TestTrajPub(Node):
    def __init__(self):
        super().__init__('test_traj_pub')
        self.pub = self.create_publisher(JointTrajectory, '/planned_arm_trajectory', 10)

    def send_test(self):
        traj = JointTrajectory()
        traj.joint_names = [
            'shoulder_pan',
            'shoulder_lift',
            'elbow_flex',
            'wrist_flex',
            'wrist_roll',
        ]

        p1 = JointTrajectoryPoint()
        p1.positions = [-0.5, 0.2, -0.3, 1.0, -1.7]
        p1.time_from_start = Duration(sec=2)

        p2 = JointTrajectoryPoint()
        p2.positions = [-1.0, 0.2, -0.35, 1.1, -1.8]
        p2.time_from_start = Duration(sec=4)

        traj.points = [p1, p2]
        self.pub.publish(traj)
        self.get_logger().info('Published test trajectory')


def main():
    rclpy.init()
    node = TestTrajPub()
    time.sleep(1.0)
    node.send_test()
    time.sleep(1.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
