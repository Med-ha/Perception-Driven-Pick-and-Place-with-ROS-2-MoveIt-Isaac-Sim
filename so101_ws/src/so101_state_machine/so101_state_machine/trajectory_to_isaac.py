#!/usr/bin/env python3
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory


class TrajectoryToIsaac(Node):
    def __init__(self):
        super().__init__('trajectory_to_isaac')

        self.cmd_pub = self.create_publisher(JointState, '/isaac_joint_command', 10)
        self.traj_sub = self.create_subscription(
            JointTrajectory,
            '/planned_arm_trajectory',
            self.traj_cb,
            10
        )
        self.state_sub = self.create_subscription(
            JointState,
            '/isaac_joint_states',
            self.state_cb,
            10
        )

        self.latest_state = None
        self._busy = False
        self._lock = threading.Lock()

        self.get_logger().info('Waiting for /planned_arm_trajectory')

    def state_cb(self, msg: JointState):
        self.latest_state = msg

    def traj_cb(self, traj: JointTrajectory):
        with self._lock:
            if self._busy:
                self.get_logger().warn('Already executing a trajectory')
                return
            self._busy = True

        t = threading.Thread(target=self.execute_traj, args=(traj,), daemon=True)
        t.start()

    def _current_positions_for(self, joint_names):
        if self.latest_state is None:
            return None
        current_map = dict(zip(self.latest_state.name, self.latest_state.position))
        vals = []
        for n in joint_names:
            if n not in current_map:
                return None
            vals.append(float(current_map[n]))
        return vals

    @staticmethod
    def _max_abs_diff(a, b):
        return max(abs(x - y) for x, y in zip(a, b)) if a and b else 999.0

    def execute_traj(self, traj: JointTrajectory):
        try:
            if not traj.points:
                self.get_logger().warn('Empty trajectory received')
                return

            joint_names = list(traj.joint_names)
            points = list(traj.points)

            current_positions = self._current_positions_for(joint_names)

            # FIX: skip initial point(s) that are already effectively the current state.
            # This avoids the "drop" / snap to a stale first point before real motion starts.
            if current_positions is not None:
                skipped_last_t = 0.0
                while points:
                    first = points[0]
                    diff = self._max_abs_diff(current_positions, list(first.positions))
                    first_t = float(first.time_from_start.sec) + float(first.time_from_start.nanosec) * 1e-9
                    if diff < 0.03:
                        skipped_last_t = first_t
                        points.pop(0)
                    else:
                        break
            else:
                skipped_last_t = 0.0

            if not points:
                self.get_logger().info('Trajectory start already matches current state; nothing to send')
                return

            prev_t = skipped_last_t
            self.get_logger().info(f'Executing {len(points)} trajectory points')

            for pt in points:
                t = float(pt.time_from_start.sec) + float(pt.time_from_start.nanosec) * 1e-9
                sleep_dt = max(0.0, t - prev_t)
                if sleep_dt > 0.0:
                    time.sleep(sleep_dt)
                prev_t = t

                msg = JointState()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.name = joint_names
                msg.position = list(pt.positions)
                msg.velocity = []
                msg.effort = []
                self.cmd_pub.publish(msg)

            self.get_logger().info('Trajectory command stream finished')

        finally:
            with self._lock:
                self._busy = False


def main():
    rclpy.init()
    node = TrajectoryToIsaac()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
