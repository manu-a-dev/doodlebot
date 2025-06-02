#!/usr/bin/env python3
import rclpy
import yaml
import pathlib
import sys
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class Pub(Node):
    def __init__(self, yaml_path):
        super().__init__("pub")

        try:
            data = yaml.safe_load(pathlib.Path(yaml_path).read_text())
        except Exception as e:
            self.get_logger().error(f"Error reading YAML file: {e}")
            rclpy.shutdown()
            return

        msg             = JointTrajectory()
        msg.joint_names = data["joint_names"]

        current_time = 0

        for i in range(len(data["points"]) - 1):
            data_pt_1 = data["points"][i]
            data_pt_2 = data["points"][i + 1]
            
            traj_pt_1 = JointTrajectoryPoint()
            traj_pt_2 = JointTrajectoryPoint()

            traj_pt_1.positions = data_pt_1["positions"]
            traj_pt_2.positions = interpolate_pos(data_pt_1["positions"], data_pt_2["positions"])

            traj_pt_1.time_from_start.sec = current_time
            traj_pt_2.time_from_start.sec = traj_pt_1.time_from_start.sec + 2

            msg.points.append(traj_pt_1)
            msg.points.append(traj_pt_2)

            # print(traj_pt_1, "\n")
            # print(traj_pt_2, "\n")

            current_time += 4

        # adds the last point.
        last_traj_pt                     = JointTrajectoryPoint()
        last_traj_pt.positions           = data["points"][-1]["positions"]
        last_traj_pt.time_from_start.sec = current_time
        msg.points.append(last_traj_pt)

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

def interpolate_pos(a, b):
    arr = []
    for i in range(len(a)):
        arr.append(a[i] + (b[i] - a[i]) * 0.5)
    return arr

def main():
    rclpy.init()
    if len(sys.argv) > 1:
        yaml_file = sys.argv[1]
    else:
        print("Use via ./publish_shape.py <YAML_FILE>")
        return
    rclpy.spin(Pub(yaml_file))

if __name__ == "__main__":
    main()