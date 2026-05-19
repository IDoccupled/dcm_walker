# dcm_walker

A Divergence Component of Motion (DCM) based humanoid open-loop walking controller in Python.

https://github.com/user-attachments/assets/8fb704c0-a31e-40e4-8270-8e581e657529

The package is purely Python-based and requires no ROS dependencies, making it suitable for standalone use. It includes a ROS 2 node for visualization and testing, which subscribes to velocity commands in `geometry_msgs.msg.Twist` interface and publishes TF frames for RViz.

## Overview

This package provides a DCM-based walking command generator and a ROS 2
visualization node. 

The package itself is purely Python-based and requires no ROS dependencies, making it suitable for standalone use. It includes a ROS 2 node for visualization and testing, which subscribes to velocity commands in `geometry_msgs.msg.Twist` interface and publishes TF frames for RViz. It builds step sequences from `/cmd_vel`, computes DCM and
center-of-mass (CoM) trajectories, converts them into foot commands, and
publishes TF frames for RViz visualization.

## Package layout

- dcm_walker/dcm_planner.py: DCM planner (VRP, DCM, CoM trajectories)
- dcm_walker/foot_step_generator.py: Step sequence generation
- dcm_walker/step_commander.py: Convert steps and CoM to foot commands
- dcm_walker/dcm_walker_visual_node.py: ROS 2 node; subscribes to /cmd_vel and broadcasts TF
- launch/visual.launch.py: RViz launch
- rviz/rviz.rviz: RViz config

## System environment

- Tested on Ubuntu 24.04 with ROS 2 Jazzy

## Build

From the workspace root:

### Build and source the virtual environment (suggested):

```bash
bash ./resource/setup.sh
source ~/venv/venv_dcm_walker/bin/activate
```

### In the venv, build the package:

```bash
colcon build --packages-select dcm_walker
source install/setup.bash
```

## Try each single script
The package is designed to be modular, so you can try each script separately. For example:

```bash
python3 dcm_walker/dcm_planner.py
```

In the main function of each script you can see how to use the classes and functions. The scripts will generate plots to visualize the results.

## Run

Run the ROS2 node to visualize the walking commands in RViz:

```bash
ros2 run dcm_walker visual_node
```

Launch RViz:

```bash
ros2 launch dcm_walker visual.launch.py
```

Use the way you like to send a velocity command:

```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1}, angular: {z: 0.5}}"
```
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Topics

- Subscribed: `/cmd_vel` (geometry_msgs/msg/Twist) using `linear.x` and `angular.z`. `linear.x` is scaled to control forward speed and setted to 0 when the input is negative. `angular.z` is scaled to control rotational speed where positive values indicate counter-clockwise rotation.
- Published: TF frames for steps and foot poses (see the node for frame names)

## Configuration

Step length/width/height, timing, and other tuning values are constants at the
top of `dcm_walker/dcm_walker_visual_node.py`. There are no ROS parameters yet.

## Dependencies

- ROS 2 Jazzy (rclpy, tf2_ros, geometry_msgs, tf_transformations)
- Python: numpy, pinocchio
- For plots: matplotlib

## License

MIT
