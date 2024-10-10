#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveGroupActionGoal
from moveit_msgs.msg import MoveGroupActionResult
from moveit_msgs.msg import MoveGroupActionFeedback
from sensor_msgs.msg import PointCloud2
from cv_bridge import CvBridge

class MotionPlanningMoveIt(Node):
    def __init__(self):
        super().__init__('motion_planning_moveit')

        self.get_logger().info('Motion planning with MoveIt initialized')
        self.point_cloud_sub = self.create_subscription(
            PointCloud2,
            '/point_cloud',
            self.point_cloud_callback,
            10)

    def point_cloud_callback(self, msg):
        self.get_logger().info('Point cloud callback initialized')
        

