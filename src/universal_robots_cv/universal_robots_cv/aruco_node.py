#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
from visualization_msgs.msg import Marker

class ArucoNode(Node):
    def __init__(self):
        super().__init__('aruco_node')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10)
        self.marker_pub = self.create_publisher(Marker, '/object_info/aruco_markers', 10)
        self.image_pub = self.create_publisher(Image, '/camera/aruco_image', 10)

    def listener_callback(self, msg):
        self.get_logger().info('Received image')
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        gray = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY)
        aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50) #Aruco dict may change
        parameters = cv.aruco.DetectorParameters()
        detector = cv.aruco.ArucoDetector(aruco_dict, parameters)
        corners, ids, rejectedImgPoints = detector.detectMarkers(gray)

        if ids is not None:
            for i in range(len(ids)):
                x,y,w,h = cv.boundingRect(corners[i])
                cv.rectangle(cv_image, (x,y), (x+w,y+h), (0,255,0), 2)
                marker_msg = Marker()
                marker_msg.header = msg.header
                marker_msg.ns = 'aruco'
                marker_msg.id = ids[i]
                marker_msg.type = Marker.CUBE
                marker_msg.action = Marker.ADD
                marker_msg.pose.position.x = x + w/2
                marker_msg.pose.position.y = y + h/2
                marker_msg.pose.position.z = 0
                self.marker_pub.publish(marker_msg)
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        pass



def main(args=None):
    rclpy.init(args=args)
    node = ArucoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()