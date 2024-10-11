#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ur_dashboard_msgs.srv import MoveRobot
from object_recognition_msgs.msg import ObjectInformation

class MovementPlanningNode(Node):
    def __init__(self):
        super().__init__("movement_planning_node")
        self.movement_server = self.create_service(MoveRobot, "move_robot", self.move_robot_callback)
        self.subscription = self.create_subscription(ObjectInformation, "/object_info/filtered_object_info", self.object_information_callback, 10)

    def object_information_callback(self, msg):
        self.get_logger().info("Received object information: %s" % msg.name)
        self.object_coordinates = msg.pose.position

    def move_robot_callback(self, request, response):
        self.get_logger().info("Received move robot request")

        return response


def main(args=None):
    rclpy.init(args=args)
    node = MovementPlanningNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()