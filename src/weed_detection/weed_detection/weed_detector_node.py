import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from cv_bridge import CvBridge
import cv2

from .detect_weeds import detect_weeds, choose_weeds


class WeedDetectorNode(Node):
    def __init__(self):
        super().__init__('weed_detector')

        self.declare_parameter('homography', [0.0] * 9)
        self.declare_parameter('ground_plane_z', 0.0)
        self.declare_parameter('cluster_interval', 20)
        self.declare_parameter('dbscan_eps', 5.0)
        self.declare_parameter('dbscan_min_fraction', 0.8)

        h_flat = self.get_parameter('homography').get_parameter_value().double_array_value
        if len(h_flat) != 9:
            self.get_logger().fatal(f'homography parameter must have 9 elements, got {len(h_flat)}')
            raise ValueError('homography parameter must have 9 elements')
        self.homography = np.array(h_flat).reshape(3, 3)
        self.ground_z = self.get_parameter('ground_plane_z').get_parameter_value().double_value
        self.cluster_interval = self.get_parameter('cluster_interval').get_parameter_value().integer_value
        self.dbscan_eps = self.get_parameter('dbscan_eps').get_parameter_value().double_value
        self.dbscan_min_fraction = self.get_parameter('dbscan_min_fraction').get_parameter_value().double_value

        self.bridge = CvBridge()
        self.frame_count = 0
        self.point_readings: list[np.ndarray] = []

        self.sub = self.create_subscription(Image, 'go_pro/image', self.image_callback, 10)
        self.pub = self.create_publisher(PoseArray, '/detected_weeds', 10)

        self.get_logger().info('Weed detector ready')

    def pixel_to_ground(self, pixels: np.ndarray) -> np.ndarray:
        """Apply homography to (N, 2) pixel coords, returning (N, 3) ground-plane points in cm."""
        reshaped = pixels[:, np.newaxis, :] # opencv needs these differently
        actual_points = cv2.perspectiveTransform(reshaped, self.homography)
        actual_points = actual_points.squeeze(axis=1) # (N, 1, 2) -> (N, 2)
        result = np.column_stack([actual_points[:, 0], actual_points[:, 1], np.full(actual_points.shape[0], self.ground_z)])
        return result

    def image_callback(self, msg: Image):
        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        points = detect_weeds(bgr)
        self.point_readings.append(points)
        self.frame_count += 1

        if self.frame_count % self.cluster_interval != 0:
            return

        clustered = choose_weeds(
            self.point_readings,
            min_fraction=self.dbscan_min_fraction,
            eps=self.dbscan_eps,
        )
        self.point_readings = []

        if clustered.shape[0] == 0:
            return

        ground_points = self.pixel_to_ground(clustered)

        pose_array = PoseArray()
        pose_array.header.stamp = msg.header.stamp
        pose_array.header.frame_id = msg.header.frame_id

        for pt in ground_points:
            pose = Pose()
            pose.position.x = float(pt[0])
            pose.position.y = float(pt[1])
            pose.position.z = float(pt[2])
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)

        self.pub.publish(pose_array)
        self.get_logger().info(f'Published {len(pose_array.poses)} weed detections')


def main(args=None):
    rclpy.init(args=args)
    node = WeedDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
