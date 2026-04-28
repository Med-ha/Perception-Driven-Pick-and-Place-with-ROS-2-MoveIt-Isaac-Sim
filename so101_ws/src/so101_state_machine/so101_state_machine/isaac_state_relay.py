#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class IsaacStateRelay(Node):
    def __init__(self):
        super().__init__('isaac_state_relay')
        self.sub = self.create_subscription(
            JointState,
            '/isaac_joint_states',
            self.cb,
            10
        )
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.get_logger().info('Relaying /isaac_joint_states -> /joint_states')

    def cb(self, msg: JointState):
        out = JointState()
        out.header = msg.header
        if out.header.frame_id == '':
            out.header.frame_id = 'base_link'
        out.name = list(msg.name)
        out.position = list(msg.position)
        out.velocity = list(msg.velocity)
        out.effort = list(msg.effort)
        self.pub.publish(out)


def main():
    rclpy.init()
    node = IsaacStateRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
