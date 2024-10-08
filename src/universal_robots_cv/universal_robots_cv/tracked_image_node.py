#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError

class TrackedImageNode(Node):
    def __init__(self):
        super().__init__('tracked_image_node')
        self.image_sub = self.create_subscription(
            Image,
            '/camera/tracked_image',
            self.listener_callback,
            10)
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.get_logger().info('Received image')
        except CvBridgeError as e:
            self.get_logger().error('CvBridge Error: {0}'.format(e))
            return
        cv.imshow("Tracked Image", cv_image)
        self.get_logger().info('Displaying image')
        cv.waitKey(1)
        if cv.waitKey(1) & 0xFF == ord('q'):
            cv.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = TrackedImageNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
