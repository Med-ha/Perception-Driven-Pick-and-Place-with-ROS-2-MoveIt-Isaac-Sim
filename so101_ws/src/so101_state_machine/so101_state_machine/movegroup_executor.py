#!/usr/bin/env python3
import time
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import MoveItErrorCodes, Constraints, JointConstraint


class MoveGroupExecutor(Node):
    def __init__(self):
        super().__init__('movegroup_executor')

        self.arm_joint_names = [
            'shoulder_pan',
            'shoulder_lift',
            'elbow_flex',
            'wrist_flex',
            'wrist_roll',
        ]

        # Fixed targets that are allowed to remain fixed
        self.arm_targets = {
            'safe_home':   [-0.2540, -0.1501, 1.5227, -1.0964, -1.6057],
            'above_box':   [0.124, -0.819, 0.665, -0.141, -1.224],
            'drop_in_box': [0.124, -0.819, 1.152, -0.141, -1.224],
            'lift_cup':    [-0.344, -0.634, 1.142, -0.796, -1.326],
        }

        # Known-good side-grasp reference joint targets for the CURRENT grasp style
        self.reference_above_cup_joints = [-0.5861, 0.1134, 1.4290, -1.4810, -1.2239]
        self.reference_grasp_cup_joints = [-0.5963, 0.3700, 0.9000, -1.2250, -1.6074]

        # Reference detected cup pose corresponding to the known-good cup-side joint targets
        self.reference_cup_pose = {
            'x': 0.304,
            'y': 0.073,
            'z': 0.728,
        }

        # Perception-to-joint calibration gains
        self.K_PAN_Y = -2.4
        self.K_LIFT_X = -1.2
        self.K_ELBOW_X = 1.6
        self.K_LIFT_Z = 0.6
        self.K_ELBOW_Z = -0.8

        self.MAX_DELTA_PAN = 0.35
        self.MAX_DELTA_LIFT = 0.30
        self.MAX_DELTA_ELBOW = 0.35

        self.GRIPPER_OPEN = 1.44
        self.GRIPPER_CLOSE = 0.80

        self.status_pub = self.create_publisher(String, '/executor_status', 10)
        self.traj_pub = self.create_publisher(JointTrajectory, '/planned_arm_trajectory', 10)
        self.joint_cmd_pub = self.create_publisher(JointState, '/isaac_joint_command', 10)
        self.attach_pub = self.create_publisher(Bool, '/isaac_attach_cube', 10)

        self.command_sub = self.create_subscription(
            String, '/executor_command', self.command_cb, 10
        )
        self.state_sub = self.create_subscription(
            JointState, '/isaac_joint_states', self.state_cb, 10
        )
        self.cup_pose_sub = self.create_subscription(
            PoseStamped, '/red_cup_pose', self.red_cup_pose_cb, 10
        )

        self.plan_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        while not self.plan_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Still waiting for motion planning service...')

        self.latest_state = None
        self.latest_red_cup_pose = None
        self.last_grasp_joint_target = None

        self.lock = threading.Lock()
        self.busy = False

        self.get_logger().info('MoveGroup executor ready (perception-based targets + MoveIt planning)')

    def publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def state_cb(self, msg: JointState):
        self.latest_state = msg

    def red_cup_pose_cb(self, msg: PoseStamped):
        self.latest_red_cup_pose = msg

    def current_arm_positions(self):
        if self.latest_state is None:
            return None

        name_to_pos = dict(zip(self.latest_state.name, self.latest_state.position))
        vals = []
        for name in self.arm_joint_names:
            if name not in name_to_pos:
                return None
            vals.append(float(name_to_pos[name]))
        return vals

    def filtered_arm_joint_state(self):
        """
        Build a JointState containing only the arm joints in the correct order.
        This is safer for MoveIt than passing the raw Isaac state message through.
        """
        if self.latest_state is None:
            return None

        name_to_pos = dict(zip(self.latest_state.name, self.latest_state.position))
        js = JointState()
        js.header = self.latest_state.header
        js.name = []
        js.position = []

        for name in self.arm_joint_names:
            if name not in name_to_pos:
                self.get_logger().error(f'filtered_arm_joint_state: missing joint {name}')
                return None
            js.name.append(name)
            js.position.append(float(name_to_pos[name]))

        js.velocity = []
        js.effort = []
        return js

    def max_abs_diff(self, a, b):
        if a is None or b is None or len(a) != len(b):
            return 999.0
        return max(abs(x - y) for x, y in zip(a, b))

    def clamp(self, v, lo, hi):
        return max(lo, min(hi, v))

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
            if cmd == 'open':
                ok = self.send_gripper(self.GRIPPER_OPEN)
                self.publish_status(f'open: {"success" if ok else "failed"}')

            elif cmd == 'close':
                ok = self.send_gripper(self.GRIPPER_CLOSE)
                self.publish_status(f'close: {"success" if ok else "failed"}')

            elif cmd == 'attach':
                ok = self.publish_attach('attach')
                self.publish_status(f'attach: {"success" if ok else "failed"}')

            elif cmd == 'detach':
                ok = self.publish_attach('detach')
                self.publish_status(f'detach: {"success" if ok else "failed"}')

            elif cmd == 'safe_home':
                ok = self.send_direct_arm_trajectory(
                    self.arm_targets['safe_home'],
                    'safe_home',
                    duration_sec=2.6
                )
                self.publish_status(f'safe_home: {"success" if ok else "failed"}')

            elif cmd == 'above_box':
                ok = self.send_direct_arm_trajectory(
                    self.arm_targets['above_box'],
                    'above_box',
                    duration_sec=2.6
                )
                self.publish_status(f'above_box: {"success" if ok else "failed"}')

            elif cmd == 'drop_in_box':
                ok = self.send_direct_arm_trajectory(
                    self.arm_targets['drop_in_box'],
                    'drop_in_box',
                    duration_sec=2.6
                )
                self.publish_status(f'drop_in_box: {"success" if ok else "failed"}')

            elif cmd == 'lift_cup':
                ok = self.send_direct_arm_trajectory(
                    self.arm_targets['lift_cup'],
                    'lift_cup',
                    duration_sec=2.6
                )
                self.publish_status(f'lift_cup: {"success" if ok else "failed"}')

            elif cmd == 'plan_to_cup':
                ok = self.move_to_detected_cup_joint_mapped(
                    label='plan_to_cup',
                    reference_joints=self.reference_above_cup_joints,
                    save_as_grasp=False
                )
                self.publish_status(f'plan_to_cup: {"success" if ok else "failed"}')

            elif cmd == 'align_to_grasp':
                ok = self.move_to_detected_cup_joint_mapped(
                    label='align_to_grasp',
                    reference_joints=self.reference_grasp_cup_joints,
                    save_as_grasp=True
                )
                self.publish_status(f'align_to_grasp: {"success" if ok else "failed"}')

            elif cmd == 'lift_object':
                ok = self.lift_from_last_grasp_joint_target()
                self.publish_status(f'lift_object: {"success" if ok else "failed"}')

            else:
                self.publish_status(f'unknown command: {cmd}')

        except Exception as e:
            self.publish_status(f'exception while running {cmd}: {e}')
        finally:
            with self.lock:
                self.busy = False

    def compute_joint_target_from_detected_pose(self, reference_joints, label):
        if self.latest_red_cup_pose is None:
            self.get_logger().error(f'{label}: no /red_cup_pose received yet')
            return None

        raw_x = float(self.latest_red_cup_pose.pose.position.x)
        raw_y = float(self.latest_red_cup_pose.pose.position.y)
        raw_z = float(self.latest_red_cup_pose.pose.position.z)

        dx = raw_x - self.reference_cup_pose['x']
        dy = raw_y - self.reference_cup_pose['y']
        dz = raw_z - self.reference_cup_pose['z']

        delta_pan = self.clamp(self.K_PAN_Y * dy, -self.MAX_DELTA_PAN, self.MAX_DELTA_PAN)
        delta_lift = self.clamp(
            self.K_LIFT_X * dx + self.K_LIFT_Z * dz,
            -self.MAX_DELTA_LIFT,
            self.MAX_DELTA_LIFT
        )
        delta_elbow = self.clamp(
            self.K_ELBOW_X * dx + self.K_ELBOW_Z * dz,
            -self.MAX_DELTA_ELBOW,
            self.MAX_DELTA_ELBOW
        )

        target = list(reference_joints)
        target[0] += delta_pan
        target[1] += delta_lift
        target[2] += delta_elbow

        self.get_logger().info(
            f'{label}: detected cup=({raw_x:.3f}, {raw_y:.3f}, {raw_z:.3f}), '
            f'delta=({dx:.3f}, {dy:.3f}, {dz:.3f}), '
            f'joint_correction=(pan={delta_pan:.3f}, lift={delta_lift:.3f}, elbow={delta_elbow:.3f}), '
            f'target={target}'
        )

        return target

    def plan_joint_target_with_moveit(self, target_positions, label='moveit_plan'):
        start_js = self.filtered_arm_joint_state()
        if start_js is None:
            self.get_logger().error(f'{label}: no usable arm joint state available')
            return None

        req = GetMotionPlan.Request()
        mpr = req.motion_plan_request
        mpr.group_name = 'arm'
        mpr.num_planning_attempts = 8
        mpr.allowed_planning_time = 5.0
        mpr.max_velocity_scaling_factor = 0.25
        mpr.max_acceleration_scaling_factor = 0.25
        mpr.planner_id = "RRTConnectkConfigDefault"
        mpr.start_state.joint_state = start_js

        goal = Constraints()
        for name, value in zip(self.arm_joint_names, target_positions):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(value)
            jc.tolerance_above = 0.05
            jc.tolerance_below = 0.05
            jc.weight = 1.0
            goal.joint_constraints.append(jc)

        mpr.goal_constraints.append(goal)

        future = self.plan_client.call_async(req)

        start_time = time.time()
        while rclpy.ok() and not future.done():
            if time.time() - start_time > 8.0:
                self.get_logger().error(f'{label}: planning timeout')
                return None
            time.sleep(0.05)

        if future.result() is None:
            self.get_logger().error(f'{label}: planning returned no result')
            return None

        res = future.result().motion_plan_response
        if res.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(f'{label}: planning failed with code {res.error_code.val}')
            return None

        traj = res.trajectory.joint_trajectory
        if len(traj.points) == 0:
            self.get_logger().error(f'{label}: empty planned trajectory')
            return None

        self.get_logger().info(f'{label}: MoveIt plan success with {len(traj.points)} points')
        return traj

    def execute_planned_trajectory(self, traj: JointTrajectory, label='execute_plan'):
        current_positions = self.current_arm_positions()
        if current_positions is None:
            self.get_logger().error(f'{label}: no joint state available')
            return False

        self.traj_pub.publish(traj)
        self.get_logger().info(f'Published planned trajectory for {label}')

        final_time = traj.points[-1].time_from_start.sec + traj.points[-1].time_from_start.nanosec / 1e9
        time.sleep(final_time + 1.0)

        final_positions = self.current_arm_positions()
        goal_positions = list(traj.points[-1].positions)

        diff_to_goal = self.max_abs_diff(final_positions, goal_positions)
        diff_from_start = self.max_abs_diff(final_positions, current_positions)

        self.get_logger().info(
            f'{label}: diff_to_goal={diff_to_goal:.4f}, diff_from_start={diff_from_start:.4f}'
        )

        if diff_to_goal < 0.35:
            time.sleep(0.3)
            return True

        if diff_from_start > 0.05:
            self.get_logger().warn(f'{label}: moved but did not fully settle; treating as success')
            time.sleep(0.3)
            return True

        self.get_logger().error(f'{label}: robot did not move enough')
        return False

    def recover_to_safe_home(self, reason: str):
        self.get_logger().warn(f'Recovery to safe_home: {reason}')
        return self.send_direct_arm_trajectory(
            self.arm_targets['safe_home'],
            'safe_home_recovery',
            duration_sec=2.6
        )

    def move_to_detected_cup_joint_mapped(self, label, reference_joints, save_as_grasp=False):
        target_joints = self.compute_joint_target_from_detected_pose(reference_joints, label)
        if target_joints is None:
            self.recover_to_safe_home(f'{label}: missing detected pose')
            return False

        traj = self.plan_joint_target_with_moveit(target_joints, label)
        if traj is None:
            self.recover_to_safe_home(f'{label}: planning failed')
            return False

        ok = self.execute_planned_trajectory(traj, label)

        if ok and save_as_grasp:
            self.last_grasp_joint_target = list(target_joints)

        if not ok:
            self.recover_to_safe_home(f'{label}: execution failed')

        return ok

    def lift_from_last_grasp_joint_target(self):
        if self.last_grasp_joint_target is None:
            self.get_logger().error('lift_object: no last successful grasp joint target available')
            self.recover_to_safe_home('lift_object: missing grasp target')
            return False

        lift_target = list(self.last_grasp_joint_target)
        lift_target[1] -= 0.12
        lift_target[2] -= 0.08

        self.get_logger().info(f'lift_object: target={lift_target}')

        traj = self.plan_joint_target_with_moveit(lift_target, 'lift_object')
        if traj is None:
            self.recover_to_safe_home('lift_object: planning failed')
            return False

        ok = self.execute_planned_trajectory(traj, 'lift_object')

        if not ok:
            self.recover_to_safe_home('lift_object: execution failed')

        return ok

    def send_direct_arm_trajectory(self, target_positions, label='direct_arm', duration_sec=2.0):
        current_positions = self.current_arm_positions()
        if current_positions is None:
            self.get_logger().error(f'{label}: no joint state available')
            return False

        traj = JointTrajectory()
        traj.joint_names = list(self.arm_joint_names)

        p0 = JointTrajectoryPoint()
        p0.positions = current_positions
        p0.time_from_start = Duration(sec=0, nanosec=0)

        p1 = JointTrajectoryPoint()
        p1.positions = [float(x) for x in target_positions]
        sec = int(duration_sec)
        nsec = int((duration_sec - sec) * 1e9)
        p1.time_from_start = Duration(sec=sec, nanosec=nsec)

        traj.points = [p0, p1]

        self.traj_pub.publish(traj)
        self.get_logger().info(f'Published direct trajectory for {label}')

        time.sleep(duration_sec + 0.8)

        final_positions = self.current_arm_positions()
        diff_to_goal = self.max_abs_diff(final_positions, p1.positions)
        diff_from_start = self.max_abs_diff(final_positions, current_positions)

        self.get_logger().info(
            f'{label}: diff_to_goal={diff_to_goal:.4f}, diff_from_start={diff_from_start:.4f}'
        )

        if diff_to_goal < 0.35:
            time.sleep(0.3)
            return True

        if diff_from_start > 0.05:
            self.get_logger().warn(f'{label}: moved but did not fully settle; treating as success')
            time.sleep(0.3)
            return True

        self.get_logger().error(f'{label}: robot did not move enough')
        return False

    def send_gripper(self, gripper_value):
        if self.latest_state is None:
            self.get_logger().error('No Isaac joint state received yet')
            return False

        current_map = dict(zip(self.latest_state.name, self.latest_state.position))

        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = list(self.arm_joint_names) + ['gripper']
        out.position = []

        for joint_name in self.arm_joint_names:
            if joint_name not in current_map:
                self.get_logger().error(f'{joint_name} joint not found')
                return False
            out.position.append(float(current_map[joint_name]))

        if 'gripper' not in current_map:
            self.get_logger().error('gripper joint not found')
            return False

        out.position.append(float(gripper_value))
        out.velocity = []
        out.effort = []

        self.get_logger().info(
            f'send_gripper: arm_hold={out.position[:5]}, gripper={gripper_value:.4f}'
        )

        self.joint_cmd_pub.publish(out)
        time.sleep(1.0)
        return True

    def publish_attach(self, command: str):
        try:
            msg = Bool()
            msg.data = (command == 'attach')
            self.attach_pub.publish(msg)
            self.get_logger().info(f'Published attach bool: {msg.data}')
            time.sleep(0.5)
            return True
        except Exception as e:
            self.get_logger().error(f'Attach publish failed: {e}')
            return False


def main():
    rclpy.init()
    node = MoveGroupExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
