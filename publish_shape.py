#!/usr/bin/env python3
import rclpy
import yaml
import pathlib
import sys
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class TrajSender(Node):
    def __init__(self, yaml_path):
        super().__init__("traj_sender")

        try:
            data = yaml.safe_load(pathlib.Path(yaml_path).read_text())
        except Exception as e:
            self.get_logger().error(f"Error reading YAML file: {e}")
            rclpy.shutdown()
            return

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
        self.get_logger().info("Trajectory published.")
        rclpy.shutdown()


def main():
    rclpy.init()
    if len(sys.argv) > 1:
        yaml_file = sys.argv[1]
    else:
        print("Use via ./publish_shape.py <YAML_FILE>")
        return
    rclpy.spin(TrajSender(yaml_file))


if __name__ == "__main__":
    main()
