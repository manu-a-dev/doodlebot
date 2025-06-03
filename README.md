# doodlebot

## Sources:
You'll need to reference ROS2 Jazzy before following Alli's guide:
> source /opt/ros/jazzy/setup.sh
https://github.com/MRRP-lab/equipment-info/blob/main/arms.md

## Positions:
You can use MoveIt or FreeDrive the physical arm. Make sure you hold the FreeDrive while moving the arm and to convert the angles in degrees to radians.

## Commands:
### Subscriber:
> ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=192.168.56.101 kinematics_params_file:=/home/nillesa2/Documents/robot_calibration.yaml headless_mode:=true
### Publisher:
> ./pub.py <SHAPE_TRAJ.YAML>
