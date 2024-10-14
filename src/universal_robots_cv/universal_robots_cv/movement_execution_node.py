#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory
from control_msgs.action import FollowJointTrajectory


class MovementExecutionNode(Node):

    def __init__(self):
        super().__init__("ur_movement_execution")
        self.declare_parameter("controller_name", "scaled_joint_trajectory_controller")
        self.declare_parameter("joints", [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ])


        controller_name = self.get_parameter("controller_name").value + "/follow_joint_trajectory"
        self.joints = self.get_parameter("joints").value

        self._action_client = ActionClient(self, FollowJointTrajectory, controller_name)
        self._action_client.wait_for_server()

        self.planned_trajectory_subscription = self.create_subscription(
            JointTrajectory,
            "planned_trajectory",
            self.planned_trajectory_callback,
            10
        )

    def planned_trajectory_callback(self, msg):
        self.get_logger().info("Received planned trajectory")
        self.execute_trajectory(msg)

    def execute_trajectory(self, trajectory):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory
        self._action_client.send_goal_async(goal_msg)
        


def main(args=None):
    rclpy.init(args=args)
    node = MovementExecutionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()