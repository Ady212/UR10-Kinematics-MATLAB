# UR10 Forward and Inverse Kinematics in MATLAB

Spatial Robotics coursework: forward and inverse kinematics for a **Universal Robots UR10** six-joint arm, written in MATLAB.

## What's in it

| Script | Description |
|---|---|
| `Task1_ForwardKinematics.m` | Forward kinematics simulator. Builds the UR10 transformation chain from Denavit-Hartenberg parameters, takes six joint angles and plots every joint and the end-effector in 3D. |
| `Task3_InverseKinematics_Validation.m` | Inverse kinematics for the wrist centre: given a target (x, y, z), solves the first three joint angles analytically, with clamping so the cosine terms stay in range. The solution is then checked by running it back through forward kinematics and printing the position error. |

## Also done in the coursework (not included here)

A demonstration on a real UR10 with ROS 2 in a simulated factory scenario: pick up a cube from a table, wait for a mobile robot's position, move the wrist to that position using the IK solver, drop the cube on the robot, return home and repeat.

## Running it

Requires MATLAB (R2016b or later, since helper functions are defined inside the script files). Open a script and run it; the target position and joint angles are set at the top of each file.

The UR10 Python control library and robot connection scripts supplied with the course are not included in this repository.
