#!/usr/bin/env python3
import time

import py_trees
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped


class BTCoordinator(Node):
    def __init__(self):
        super().__init__("bt_node")

        self.cmd_pub = self.create_publisher(String, "/executor_command", 10)

        self.status_sub = self.create_subscription(
            String, "/executor_status", self.status_cb, 10
        )
        self.cup_sub = self.create_subscription(
            PoseStamped, "/red_cup_pose", self.cup_cb, 10
        )

        self.latest_status = None
        self.latest_status_time = 0.0

        self.latest_cup_pose = None
        self.latest_cup_time = 0.0

        root = py_trees.composites.Sequence(name="PickPlaceRedCup", memory=True)

        root.add_children([
             WaitForRedCup(self, timeout_sec=10.0),
            SendCommandAndWait(self, "safe_home", timeout_sec=10.0),
            SendCommandAndWait(self, "open", timeout_sec=6.0),
            SendCommandAndWait(self, "plan_to_cup", timeout_sec=12.0),
            SendCommandAndWait(self, "align_to_grasp", timeout_sec=12.0),
            SendCommandAndWait(self, "close", timeout_sec=6.0),
            SendCommandAndWait(self, "attach", timeout_sec=6.0),
            SendCommandAndWait(self, "lift_cup", timeout_sec=10.0),
            SendCommandAndWait(self, "above_box", timeout_sec=10.0),
            SendCommandAndWait(self, "drop_in_box", timeout_sec=10.0),
            SendCommandAndWait(self, "open", timeout_sec=6.0),
            SendCommandAndWait(self, "detach", timeout_sec=6.0),
            SendCommandAndWait(self, "above_box", timeout_sec=10.0),
            SendCommandAndWait(self, "safe_home", timeout_sec=10.0),
        ])

        self.tree = py_trees.trees.BehaviourTree(root)
        self.tree.setup(timeout=15)

        self.tick_timer = self.create_timer(0.2, self.tick_tree)

        self.get_logger().info("bt_node ready")

    def status_cb(self, msg: String):
        self.latest_status = msg.data.strip()
        self.latest_status_time = time.time()
        self.get_logger().info(f"status: {self.latest_status}")

    def cup_cb(self, msg: PoseStamped):
        self.latest_cup_pose = msg
        self.latest_cup_time = time.time()

    def send_command(self, command: str):
        msg = String()
        msg.data = command
        self.cmd_pub.publish(msg)
        self.get_logger().info(f"sent command: {command}")

    def tick_tree(self):
        try:
            self.tree.tick()
            root_status = self.tree.root.status
            if root_status == py_trees.common.Status.SUCCESS:
                self.get_logger().info("BT finished successfully")
                self.tick_timer.cancel()
            elif root_status == py_trees.common.Status.FAILURE:
                self.get_logger().error("BT failed")
                self.tick_timer.cancel()
        except Exception as e:
            self.get_logger().error(f"BT tick exception: {e}")
            self.tick_timer.cancel()


class WaitForRedCup(py_trees.behaviour.Behaviour):
    def __init__(self, ros_node: BTCoordinator, timeout_sec: float = 8.0):
        super().__init__(name="WaitForRedCup")
        self.ros_node = ros_node
        self.timeout_sec = timeout_sec
        self.start_time = None

    def initialise(self):
        self.start_time = time.time()

    def update(self):
        if self.ros_node.latest_cup_pose is not None:
            age = time.time() - self.ros_node.latest_cup_time
            if age < 2.0:
                return py_trees.common.Status.SUCCESS

        if time.time() - self.start_time > self.timeout_sec:
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING


class SendCommandAndWait(py_trees.behaviour.Behaviour):
    def __init__(self, ros_node: BTCoordinator, command: str, timeout_sec: float = 8.0):
        super().__init__(name=f"Cmd_{command}")
        self.ros_node = ros_node
        self.command = command
        self.timeout_sec = timeout_sec
        self.start_time = None
        self.sent = False

    def initialise(self):
        self.start_time = time.time()
        self.sent = False

    def update(self):
        if not self.sent:
            # clear old status so we only react to a fresh one
            self.ros_node.latest_status = None
            self.ros_node.latest_status_time = 0.0
            self.ros_node.send_command(self.command)
            self.sent = True
            return py_trees.common.Status.RUNNING

        status = self.ros_node.latest_status
        status_time = self.ros_node.latest_status_time

        # only trust statuses published after this command started
        if status is not None and status_time >= self.start_time:
            ok_text = f"{self.command}: success"
            fail_prefix = f"{self.command}:"

            if status == ok_text:
                return py_trees.common.Status.SUCCESS

            if status.startswith(fail_prefix) and status != ok_text:
                return py_trees.common.Status.FAILURE

        if time.time() - self.start_time > self.timeout_sec:
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING

def main():
    rclpy.init()
    node = BTCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
