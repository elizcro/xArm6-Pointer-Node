from collections import deque

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PointStamped
from std_msgs.msg import Bool


class WeedSequencerNode(Node):
    def __init__(self):
        super().__init__('weed_sequencer')

        self.declare_parameter('target_frame', 'link_base')

        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value

        self.queue: deque[PointStamped] = deque()
        self.waiting_for_result = False
        self.batch_accepted = False

        self.sub_weeds = self.create_subscription(
            PoseArray, '/detected_weeds', self.weeds_callback, 10)
        self.sub_result = self.create_subscription(
            Bool, '/pointer/result', self.result_callback, 10)
        self.pub_target = self.create_publisher(
            PointStamped, 'target_point', 10)

        self.get_logger().info('Weed sequencer ready — waiting for first batch')

    def weeds_callback(self, msg: PoseArray):
        if self.batch_accepted:
            self.get_logger().info('Already accepted a batch — ignoring new detections')
            return

        self.batch_accepted = True

        for pose in msg.poses:
            pt = PointStamped()
            pt.header.frame_id = self.target_frame
            pt.point = pose.position
            self.queue.append(pt)

        self.get_logger().info(f'Accepted batch of {len(self.queue)} weed targets')
        self.send_next()

    def result_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info('Arm reached target successfully')
        else:
            self.get_logger().warn('Arm failed/skipped target')

        self.waiting_for_result = False
        self.send_next()

    def send_next(self):
        if len(self.queue) == 0:
            self.get_logger().info('Batch complete — ready for the next batch')
            self.batch_accepted = False   # allow the next detection batch in
            return

        target = self.queue.popleft()
        target.header.stamp = self.get_clock().now().to_msg()
        remaining = len(self.queue)

        self.get_logger().info(
            f'Sending target ({target.point.x:.1f}, {target.point.y:.1f}, {target.point.z:.1f}) cm '
            f'— {remaining} remaining')

        self.pub_target.publish(target)
        self.waiting_for_result = True


def main(args=None):
    rclpy.init(args=args)
    node = WeedSequencerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
