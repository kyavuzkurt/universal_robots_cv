#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from object_recognition_msgs.msg import RecognizedObjectArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class MovementPlanning(Node):
    def __init__(self):
        super().__init__("movement_planning_node")
        self.get_logger().info("Movement planning node initialized")
        self.object_info_subscription = self.create_subscription(
            RecognizedObjectArray,
            "/object_info/filtered_object_info",
            self.object_info_callback,
            10
        )
        self.trajectory_publisher = self.create_publisher(
            JointTrajectory,
            "/planned_trajectory",
            10
        )
        
    def object_info_callback(self, msg):
        self.get_logger().info("Received object info")
        #TODO Implement inverse kinematics functions to calculate the joint angles for the object
        #TODO prediction filter calculations to predict the object's image coordinates while robot is moving. Research on kalman filters if robot is going to move with a constant velocity.
        self.publish_trajectory(msg) #placeholder function, will publish the trajectory from here

    def publish_trajectory(self, msg):
        trajectory = JointTrajectory()
        trajectory.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        trajectory.points = [JointTrajectoryPoint(positions=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], velocities=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], accelerations=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start=Duration(sec=0).to_msg())]
        self.trajectory_publisher.publish(trajectory)

def main():
    rclpy.init()
    node = MovementPlanning()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()