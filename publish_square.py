#!/usr/bin/env python3
import rclpy
import yaml
import pathlib
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

YAML_FILE = pathlib.Path(__file__).with_name("square_traj.yaml")


class SquareSender(Node):
    def __init__(self):
        super().__init__("square_sender")

        data = yaml.safe_load(YAML_FILE.read_text())
        msg = JointTrajectory()
        msg.joint_names = data["joint_names"]

        for point in data["points"]:
            pt = JointTrajectoryPoint()
            pt.positions = point["positions"]
            pt.time_from_start.sec = int(point["time_from_start"])
            msg.points.append(pt)
        self.msg = msg

        self.pub = self.create_publisher(
            JointTrajectory,
            "/scaled_joint_trajectory_controller/joint_trajectory",
            10,
        )
        self.create_timer(1.0, self.tick)

    def tick(self):
        self.pub.publish(self.msg)
        self.get_logger().info("Square trajectory published.")
        rclpy.shutdown()


rclpy.init()
rclpy.spin(SquareSender())
