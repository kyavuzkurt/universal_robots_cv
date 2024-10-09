#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
import numpy as np
from cv_bridge import CvBridge
from object_recognition_msgs.msg import ObjectInformation


class PointCloudNode(Node):
    def __init__(self):
        super().__init__('point_cloud_node')

        self.point_cloud_pub = self.create_publisher(PointCloud2, '/point_cloud', 10)
        self.filtered_point_cloud_pub = self.create_publisher(PointCloud2, '/filtered_point_cloud', 10)


        self.get_logger().info('Generating 3D map for COCO dataset')
        self.depth_estimation = self.create_subscription(
            Image,
            '/depth_estimation',
            self.depth_estimation_callback,
            10)



    def depth_estimation_callback(self, msg):
        self.get_logger().info('Depth estimation callback initialized')
        bridge = CvBridge()
        try:
            depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception as e:
            self.get_logger().error(f'Failed to convert depth image: {e}')
            return
        #Placeholder for camera intrinsic parameters
        fx = 525.0  # Focal length in x 
        fy = 525.0  # Focal length in y
        cx = depth_image.shape[1] / 2.0  # Principal point x
        cy = depth_image.shape[0] / 2.0  # Principal point y

        points = []
        for v in range(depth_image.shape[0]):
            for u in range(depth_image.shape[1]):
                z = depth_image[v, u]
                if z == 0:
                    continue  
                x = (u - cx) * z / fx
                y = (v - cy) * z / fy
                points.append([x, y, z])

        point_cloud = np.array(points)
        self.get_logger().info(f'Point cloud generated with {len(point_cloud)} points')
        point_cloud_msg = PointCloud2()
        point_cloud_msg.header = msg.header
        point_cloud_msg.width = len(point_cloud)
        point_cloud_msg.data = point_cloud.astype(np.uint8)
        self.process_point_cloud(point_cloud_msg)

        

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()