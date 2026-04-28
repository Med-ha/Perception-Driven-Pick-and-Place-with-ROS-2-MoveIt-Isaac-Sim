#!/usr/bin/env python3
import time
import threading

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
from moveit_configs_utils import MoveItConfigsBuilder


class MoveItExecutor(Node):
    def __init__(self):
        super().__init__('moveit_executor')

        self.command_sub = self.create_subscription(
            String, '/executor_command', self.command_cb, 10
        )
        self.status_pub = self.create_publisher(String, '/executor_status', 10)
        self.traj_pub = self.create_publisher(JointTrajectory, '/planned_arm_trajectory', 10)
        self.joint_cmd_pub = self.create_publisher(JointState, '/isaac_joint_command', 10)

        self.state_sub = self.create_subscription(
            JointState, '/isaac_joint_states', self.state_cb, 10
        )

        self.latest_state = None
        self.lock = threading.Lock()
        self.busy = False

        moveit_config = (
            MoveItConfigsBuilder(
                robot_name='so101_new_calib',
                package_name='so101_moveit_config'
            )
            .robot_description_kinematics(file_path='config/kinematics.yaml')
            .trajectory_execution(file_path='config/moveit_controllers.yaml')
            .planning_scene_monitor(
                publish_robot_description=True,
                publish_robot_description_semantic=True,
            )
            .planning_pipelines(pipelines=['ompl'])
            .joint_limits(file_path='config/joint_limits.yaml')
            .to_moveit_configs()
        )

        self.moveit = MoveItPy(
            node_name='moveit_py_executor',
            config_dict=moveit_config.to_dict()
        ) 
        self.arm = self.moveit.get_planning_component('arm')

        self.GRIPPER_OPEN = 1.44
        self.GRIPPER_CLOSE = 0.80

        self.arm_targets = {
            'home': [0.0, 0.0, 0.0, 0.0, 0.0],
            'pregrasp_demo': [-1.00, 0.20, -0.35, 1.10, -1.80],
            'place_demo': [-0.50, 0.20, -0.30, 1.00, -1.70],
        }

        self.get_logger().info('MoveIt executor ready')

    def state_cb(self, msg: JointState):
        self.latest_state = msg

    def publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def command_cb(self, msg: String):
        cmd = msg.data.strip()

        with self.lock:
            if self.busy:
                self.publish_status(f'busy: ignoring {cmd}')
                return
            self.busy = True

        t = threading.Thread(target=self.handle_command, args=(cmd,), daemon=True)
        t.start()

    def handle_command(self, cmd: str):
        try:
            if cmd in self.arm_targets:
                ok = self.plan_and_send_arm(self.arm_targets[cmd], cmd)
                self.publish_status(f'{cmd}: {"success" if ok else "failed"}')
            elif cmd == 'open':
                ok = self.send_gripper(self.GRIPPER_OPEN)
                self.publish_status(f'open: {"success" if ok else "failed"}')
            elif cmd == 'close':
                ok = self.send_gripper(self.GRIPPER_CLOSE)
                self.publish_status(f'close: {"success" if ok else "failed"}')
            else:
                self.publish_status(f'unknown command: {cmd}')
        except Exception as e:
            self.publish_status(f'exception: {e}')
        finally:
            with self.lock:
                self.busy = False

    def plan_and_send_arm(self, target_positions, label='arm_goal'):
        self.arm.set_start_state_to_current_state()

        robot_state = RobotState(self.moveit.get_robot_model())
        robot_state.set_joint_group_positions('arm', list(target_positions))
        self.arm.set_goal_state(robot_state=robot_state)

        plan_result = self.arm.plan()
        if not plan_result:
            self.get_logger().error('planning failed')
            return False

        traj = plan_result.trajectory
        joint_traj = traj.joint_trajectory if hasattr(traj, 'joint_trajectory') else traj

        if len(joint_traj.points) == 0:
            self.get_logger().error('empty trajectory')
            return False

        self.traj_pub.publish(joint_traj)
        self.get_logger().info(f'published trajectory for {label}')

        final_positions = list(joint_traj.points[-1].positions)
        return self.wait_until_close(joint_traj.joint_names, final_positions, timeout=8.0)

    def wait_until_close(self, joint_names, goal_positions, timeout=8.0, tol=0.08):
        start = time.time()
        while time.time() - start < timeout:
            if self.latest_state is None:
                time.sleep(0.05)
                continue

            current = dict(zip(self.latest_state.name, self.latest_state.position))
            diffs = []
            for name, goal in zip(joint_names, goal_positions):
                if name not in current:
                    diffs.append(999.0)
                else:
                    diffs.append(abs(current[name] - goal))

            if diffs and max(diffs) < tol:
                return True

            time.sleep(0.05)

        return False

    def send_gripper(self, gripper_value):
        if self.latest_state is None:
            return False

        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = list(self.latest_state.name)
        out.position = list(self.latest_state.position)
        out.velocity = []
        out.effort = []

        if 'gripper' not in out.name:
            return False

        idx = out.name.index('gripper')
        out.position[idx] = float(gripper_value)
        self.joint_cmd_pub.publish(out)
        time.sleep(1.0)
        return True


def main():
    rclpy.init()
    node = MoveItExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
