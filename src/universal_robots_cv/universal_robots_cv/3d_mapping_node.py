#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from object_recognition_msgs.msg import ObjectInformation
from visualization_msgs.msg import Marker
import torch
from midas.midas_net import MidasNet
from midas.transforms import Resize, NormalizeImage, PrepareForNet
import torch
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg
import struct
import os

class ThreeDMappingNode(Node):
    def __init__(self):
        super().__init__('3d_mapping_node')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('detection_type', rclpy.Parameter.Type.STRING),
                ('desired_objects', rclpy.Parameter.Type.STRING_ARRAY),  # New parameter for filtering
            ]
        )
        self.detection_type = self.get_parameter('detection_type').value
        self.desired_objects = self.get_parameter('desired_objects').value  # Retrieve desired objects

        if self.detection_type == 'coco':
            self.get_logger().info('Generating 3D map for COCO dataset')
            self.object_info_sub = self.create_subscription(
                ObjectInformation,
                '/object_info/filtered_object_info',
                self.coco_listener_callback,
                10)
        elif self.detection_type == 'aruco':
            self.get_logger().info('Generating 3D map for ArUco markers')
            self.aruco_image_sub = self.create_subscription(
                Marker,
                '/object_info/aruco_markers',
                self.aruco_listener_callback,
                10)
        else:
            self.get_logger().error('Invalid detection type. Please enter either "aruco" or "coco"')

        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10)
        self.point_cloud_pub = self.create_publisher(PointCloud2, '/object_info/points', 10)

        # Initialize MiDAS model
        self.midas_model = self.initialize_midas()

    def initialize_midas(self):
        self.get_logger().info('Initializing MiDAS model with GPU acceleration')
        model_path = os.path.join(
            self.get_package_share_directory('universal_robots_cv'),
            'config',
            'midas_model.pt'
        )
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        midas = MidasNet(model_path, non_negative=True)
        midas.to(device)
        midas.eval()
        return midas

    def coco_listener_callback(self, msg):

        self.get_logger().info('Received object info for COCO dataset')

        # Filter desired objects
        filtered_objects = [obj for obj in msg.objects if obj.label in self.desired_objects]
        if not filtered_objects:
            self.get_logger().info('No desired objects detected.')
            return

        bridge = CvBridge()
        try:
            # Assume msg contains an Image message
            cv_image = bridge.imgmsg_to_cv2(msg.image, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')
            return

        # Preprocess image for MiDAS
        transform = Resize(
            384,
            384,
            Resize.SCALE_INTERPOLATION
        )
        image = transform({"image": cv_image})["image"]
        image = NormalizeImage()(image)
        image = PrepareForNet()(image)
        image = torch.from_numpy(image).unsqueeze(0).float().to(self.midas_model.device)

        with torch.no_grad():
            depth = self.midas_model.forward(image)
            depth = depth.squeeze().cpu().numpy()

        # Generate point cloud from depth
        height, width = depth.shape
        fov = 90  # Adjust FOV to match camera
        f_x = f_y = (width / 2) / np.tan(np.deg2rad(fov / 2))

        points = []
        for v in range(height):
            for u in range(width):
                Z = depth[v, u]
                X = (u - width / 2) * Z / f_x
                Y = (v - height / 2) * Z / f_y
                points.append([X, Y, Z])

        if not points:
            self.get_logger().info('No points to publish.')
            return

        # Create PointCloud2 message
        point_cloud = self.create_point_cloud(points)

        # Publish the point cloud
        self.point_cloud_pub.publish(point_cloud)
        self.get_logger().info('Published 3D point cloud for filtered objects')

    def create_point_cloud(self, points):

        header = std_msgs.msg.Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        buffer = []
        for point in points:
            buffer.append(struct.pack('fff', *point))

        point_cloud = PointCloud2()
        point_cloud.header = header
        point_cloud.height = 1
        point_cloud.width = len(points)
        point_cloud.fields = fields
        point_cloud.is_bigendian = False
        point_cloud.point_step = 12
        point_cloud.row_step = point_cloud.point_step * len(points)
        point_cloud.is_dense = True
        point_cloud.data = b''.join(buffer)

        return point_cloud

def main(args=None):
    rclpy.init(args=args)
    node = ThreeDMappingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()