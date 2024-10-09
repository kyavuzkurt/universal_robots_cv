#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from object_recognition_msgs.msg import ObjectInformation
import os
import yaml

class ObjectFilteringNode():
    def __init__(self):
        super().__init__("object_filtering_node")
        self.get_logger().info("Object filtering node has been started")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("object_name", rclpy.Parameter.Type.STRING)
            ]
        )

        self.object_name = self.get_parameter("object_name").value

        self.subscription = self.create_subscription(
            ObjectInformation,
            "/object_info/object_info",
            self.object_information_callback,
            10
        )
        self.filtered_object_pub = self.create_publisher(ObjectInformation, "/object_info/filtered_object_info", 10)
        labels_path = os.path.join(
            self.get_package_share_directory('universal_robots_cv'),
            'config',
            'coco_labels.yaml'
        )
        try:
            with open(labels_path, 'r') as file:
                labels_yaml = yaml.safe_load(file)
                self.coco_labels = labels_yaml['coco_labels']
                self.get_logger().info('COCO labels loaded successfully.')
        except Exception as e:
            self.get_logger().error(f'Failed to load COCO labels: {e}')
            self.coco_labels = {}
    
    def object_information_callback(self, msg):
        self.get_logger().info("Received object information")
        self.get_logger().info(msg)
        if msg.name in self.coco_labels:
            self.get_logger().info("Object is in the list")
            self.filtered_object_pub.publish(msg)
        else:
            self.get_logger().info("Object is not in the list, skipping")


def main(args=None):
    rclpy.init(args=args)
    node = ObjectFilteringNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()