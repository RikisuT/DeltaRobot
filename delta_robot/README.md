# `delta_robot` Package Overview
This package has 4 nodes:
1. `motor_controller`
2. `kinematics`
3. `motion_planner`
4. `trajectory_generator`

## Motor Controller Node
Responsible for interfacing (using the hardware) with the 3 motors controlling the robot. This node implements interfacing with dynamixel motors but can be extended to other motor types as long as the inter-node connections are maintained.

## Kinematics Node
Enables several kinematic capabilities for any delta robot. The node accepts a config file that specifies the robot's geometry, link lengths, and joint limits. Forward and Inverse Kinematics are implemented here as well as the Jacobian for creating joint-velocity trajectories from position trajectories.

## Motion Planner Node
Contains movement functions and is the top-level node to **launch the entire project**. Pre-programmed position and velocity trajectories are built in for running movement demos.

## Trajectory Generator
Responsible for generating trajectories for the robot to follow. Currently, some hardcoded trajectories are baked into the node, but will be extended to offer parametric trajectory generation, resulting in position-velocity trajectories.