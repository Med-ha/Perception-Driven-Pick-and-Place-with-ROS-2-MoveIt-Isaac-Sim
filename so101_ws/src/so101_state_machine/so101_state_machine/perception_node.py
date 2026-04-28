#!/usr/bin/env python3
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  # noqa: F401


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        self.declare_parameter('rgb_topic', '/rgb')
        self.declare_parameter('depth_topic', '/depth')
        self.declare_parameter('camera_info_topic', '/camera_info')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('min_contour_area', 300.0)

        self.rgb_topic = self.get_parameter('rgb_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.base_frame = self.get_parameter('base_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.min_contour_area = float(self.get_parameter('min_contour_area').value)

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.rgb_img = None
        self.rgb_header = None
        self.depth_img = None
        self.cam_info = None
        self.last_warn_time = 0.0
        self.last_info_time = 0.0

        self.create_subscription(Image, self.rgb_topic, self.rgb_cb, 10)
        self.create_subscription(Image, self.depth_topic, self.depth_cb, 10)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.info_cb, 10)

        self.pose_pub = self.create_publisher(PoseStamped, '/red_cup_pose', 10)
        self.create_timer(0.2, self.process)

        self.get_logger().info(
            f'Perception started: rgb={self.rgb_topic}, depth={self.depth_topic}, '
            f'info={self.camera_info_topic}, camera_frame={self.camera_frame}, base_frame={self.base_frame}'
        )

    def warn_throttled(self, msg, period=2.0):
        now = time.time()
        if now - self.last_warn_time > period:
            self.get_logger().warn(msg)
            self.last_warn_time = now

    def info_throttled(self, msg, period=1.5):
        now = time.time()
        if now - self.last_info_time > period:
            self.get_logger().info(msg)
            self.last_info_time = now

    def rgb_cb(self, msg):
        try:
            self.rgb_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.rgb_header = msg.header
        except Exception as e:
            self.warn_throttled(f'RGB conversion failed: {e}')

    def depth_cb(self, msg):
        try:
            self.depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.warn_throttled(f'Depth conversion failed: {e}')

    def info_cb(self, msg):
        self.cam_info = msg

    def get_depth_meters(self, u, v):
        if self.depth_img is None:
            return None

        h, w = self.depth_img.shape[:2]
        if u < 0 or v < 0 or u >= w or v >= h:
            return None

        u0, u1 = max(0, u - 2), min(w, u + 3)
        v0, v1 = max(0, v - 2), min(h, v + 3)
        patch = self.depth_img[v0:v1, u0:u1]

        if patch.dtype == np.uint16:
            vals = patch[patch > 0].astype(np.float32) / 1000.0
        else:
            patch = patch.astype(np.float32)
            vals = patch[np.isfinite(patch) & (patch > 0.0)]

        if len(vals) == 0:
            return None

        return float(np.median(vals))

    def process(self):
        if self.rgb_img is None or self.depth_img is None or self.cam_info is None:
            return

        hsv = cv2.cvtColor(self.rgb_img, cv2.COLOR_BGR2HSV)

        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return

        contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(contour) < self.min_contour_area:
            return

        M = cv2.moments(contour)
        if M['m00'] == 0:
            return

        u = int(M['m10'] / M['m00'])
        v = int(M['m01'] / M['m00'])

        z_opt = self.get_depth_meters(u, v)
        if z_opt is None:
            self.warn_throttled('No valid depth at red cup centroid')
            return

        fx = float(self.cam_info.k[0])
        fy = float(self.cam_info.k[4])
        cx = float(self.cam_info.k[2])
        cy = float(self.cam_info.k[5])

        # Point from pinhole projection in the camera frame used by Isaac camera topics
        x_cam = (u - cx) * z_opt / fx
        y_cam = (v - cy) * z_opt / fy
        z_cam = z_opt

        pose_cam = PoseStamped()
        pose_cam.header.stamp = self.rgb_header.stamp if self.rgb_header else self.get_clock().now().to_msg()
        pose_cam.header.frame_id = self.camera_frame
        pose_cam.pose.position.x = float(x_cam)
        pose_cam.pose.position.y = float(y_cam)
        pose_cam.pose.position.z = float(z_cam)
        pose_cam.pose.orientation.w = 1.0

        try:
            pose_base = self.tf_buffer.transform(
                pose_cam,
                self.base_frame,
                timeout=Duration(seconds=0.2)
            )

            self.info_throttled(
                f'red cup cam=({x_cam:.3f}, {y_cam:.3f}, {z_cam:.3f}) '
                f'base_link=({pose_base.pose.position.x:.3f}, '
                f'{pose_base.pose.position.y:.3f}, '
                f'{pose_base.pose.position.z:.3f})'
            )

            self.pose_pub.publish(pose_base)

        except Exception as e:
            self.warn_throttled(f'TF to {self.base_frame} failed: {e}')


def main():
    rclpy.init()
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
