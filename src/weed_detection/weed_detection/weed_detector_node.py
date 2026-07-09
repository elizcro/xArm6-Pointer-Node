import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose
from cv_bridge import CvBridge
import cv2

from .detect_weeds import detect_weeds, choose_weeds
from .segment_green import segment_and_box_green


class WeedDetectorNode(Node):
    def __init__(self):
        super().__init__('weed_detector')

        self.declare_parameter('ground_plane_z', 0.0)
        self.declare_parameter('camera_height', 0.0)
        self.declare_parameter('camera_angle_deg', 0.0)
        self.declare_parameter('plant_height', 0.0)
        self.declare_parameter('cluster_interval', 20)
        self.declare_parameter('dbscan_eps', 5.0)
        self.declare_parameter('dbscan_min_fraction', 0.8)
        # offset of the center of the camera pinhole from the 0,0 of the arm
        self.declare_parameter('camera_offset_x', 0.0)
        self.declare_parameter('camera_offset_y', 0.0)

        self.camera_angle_degrees = self.get_parameter('camera_angle_deg').get_parameter_value().double_value
        self.camera_height = self.get_parameter('camera_height').get_parameter_value().double_value
        self.ground_z = self.get_parameter('ground_plane_z').get_parameter_value().double_value
        self.plant_height = self.get_parameter('plant_height').get_parameter_value().double_value
        self.cluster_interval = self.get_parameter('cluster_interval').get_parameter_value().integer_value
        self.dbscan_eps = self.get_parameter('dbscan_eps').get_parameter_value().double_value
        self.dbscan_min_fraction = self.get_parameter('dbscan_min_fraction').get_parameter_value().double_value
        self.camera_offset_x = self.get_parameter('camera_offset_x').get_parameter_value().double_value
        self.camera_offset_y = self.get_parameter('camera_offset_y').get_parameter_value().double_value

        self.bridge = CvBridge()
        self.frame_count = 0
        self.point_readings: list[np.ndarray] = []
        
        self.debug_view = True
        self.last_clustered = None
        self.last_ground = None
        self.cam_P = None

        self.sub = self.create_subscription(Image, '/go_pro/image_rect_color', self.image_callback, 10)
        self.sub_info = self.create_subscription(CameraInfo, '/go_pro/camera_info', self.cam_info_callback, 10)
        self.pub = self.create_publisher(PoseArray, '/detected_weeds', 10)

        self.get_logger().info('Weed detector ready')

    def pixel_to_ground(self, pixels: np.ndarray) -> np.ndarray:
        """Map (N, 2) pixel coords to (N, 3) ground-plane points in cm."""
        P = self.cam_P
        if P is None:
            self.get_logger().warn('pixel_to_ground called before camera P (intrinsics) initialized')
            return np.zeros((pixels.shape[0], 3), dtype=np.float64)

        K_rect = P[:, :3].copy()
        # should always be the case because image is rectified, but force in case
        K_rect[[0, 1, 2, 2], [1, 0, 0, 1]] = 0
        K_rect[2, 2] = 1

        h = self.camera_height  # camera height above the ground plane (cm)
        a = np.deg2rad(self.camera_angle_degrees)
        ca, sa = np.cos(a), np.sin(a)

        # H = K_rect @ [r1 | r2 | t], with rotation about the camera x-axis by `a`
        # and t = -R @ C for camera center C at height h.
        H = K_rect @ np.array([[1,   0,     0.0],
                               [0, -sa,  h * ca],
                               [0,  ca,  h * sa]])
        H_inv = np.linalg.inv(H)
        homogeneous_pixels = np.hstack((pixels, np.ones((pixels.shape[0], 1))))

        result = (H_inv @ homogeneous_pixels.T).T
        result[:, 0] /= result[:, 2]
        result[:, 1] /= result[:, 2]
        result[:, 2] = self.ground_z
        return result

    def ground_to_real_plant_intersect(self, ground_points: np.ndarray) -> np.ndarray:
        ground_dist_to_points = np.linalg.norm(ground_points[:, :2], axis=1)
        assert ground_dist_to_points.shape == (ground_points.shape[0],), f"{ground_dist_to_points.shape = }, from {ground_points.shape = }"
    
        point_to_plant_dist = self.plant_height / self.camera_height * ground_dist_to_points
    
        ground_angle = np.arctan2(ground_points[:, 1], ground_points[:, 0])
        assert ground_angle.shape == (ground_points.shape[0],), f"{ground_angle.shape = }, from {ground_points.shape = }"
    
        point_to_plant_dist_x = point_to_plant_dist * np.cos(ground_angle)
        point_to_plant_dist_y = point_to_plant_dist * np.sin(ground_angle)
    
        ground_points[:, 0] -= point_to_plant_dist_x
        ground_points[:, 1] -= point_to_plant_dist_y
        ground_points[:, 2] += self.plant_height
        return ground_points

    def show_debug(self, bgr):
        debug = bgr.copy()
        
        # 1) what counts as "green", plus the per-frame boxes
        try:
            mask, bboxes = segment_and_box_green(bgr)
            m = mask.astype(bool)
            debug[m] = (0.5 * debug[m] + np.array([0, 128, 0])).astype(np.uint8)
            for (x, y, w, h) in bboxes:                 # adjust if your bboxes are x1,y1,x2,y2
                cv2.rectangle(debug, (int(x), int(y)), (int(x + w), int(y + h)), (0, 165, 255), 2)
        except Exception as e:
            self.get_logger().warn(f'debug overlay failed: {e}')

        # 2) the detections that actually got published, labeled with ground (x,y) cm
        if self.last_clustered is not None and self.last_ground is not None:
            for (u, v), g in zip(self.last_clustered, self.last_ground):
                 p = (int(u), int(v))
                 cv2.circle(debug, p, 6, (0, 0, 255), -1)
                 cv2.putText(debug, f'({g[0]:.0f},{g[1]:.0f})', (p[0] + 8, p[1]),
                             cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        cv2.imshow('weed detection', debug)
        cv2.waitKey(1)

    def cam_info_callback(self, msg: CameraInfo):
        if self.cam_P is None:
            self.get_logger().info('Received first CameraInfo; camera intrinsics initialized')
        P = np.array(msg.p, dtype=np.float64)
        P = np.reshape(P, (3, 4))
        self.cam_P = P

    def image_callback(self, msg: Image):
        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        points = detect_weeds(bgr)
        # debug info
        self.get_logger().info(f'detect_weeds returned {len(points)} points: {points}')
        self.point_readings.append(points)
        self.frame_count += 1
        
        if self.debug_view:
            self.show_debug(bgr)

        if self.frame_count % self.cluster_interval != 0:
            return

        clustered = choose_weeds(
            self.point_readings,
            min_fraction=self.dbscan_min_fraction,
            eps=self.dbscan_eps,
            logger=self.get_logger(),
        )
        self.point_readings = []

        if clustered.shape[0] == 0:
            return

        ground_points = self.pixel_to_ground(clustered)
        ground_points = self.ground_to_real_plant_intersect(ground_points)
        # account for camera offset
        ground_points[:, 0] += self.camera_offset_x
        ground_points[:, 1] += self.camera_offset_y
        self.last_clustered = clustered
        self.last_ground = ground_points
        

        self.get_logger().info(f"Clustered: {clustered}, ground_points: {ground_points}")

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
