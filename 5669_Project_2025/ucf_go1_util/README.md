# ucf_go1_util

This package provides utility scripts and functions that can be used with the Unitree Go1 robot, particularly when interfacing through ROS. One of the main features is a Python script for generating and publishing sinusoidal joint trajectories, which can be useful for testing controllers, joint limits, and simulation environments.

## Overview
The primary script included here, function_generator.py, publishes sinusoidal joint trajectories for each of the 12 joints on the Go1 quadruped robot. By default, the generated sine waves span from the lower to the upper joint limit, centered around the midpoint of each joint’s range of motion. This provides a convenient way to exercise the robot’s joints, verify control signals, and test simulation setups.

## Features
- Sinusoidal Joint Trajectories: Each joint’s motion is defined by a configurable sinusoidal function. The amplitude, frequency, phase, and offset are derived from the robot’s URDF-defined joint limits, ensuring joint motion stays within safe boundaries.
- Automatic Calculation of Parameters: Amplitudes and offsets are automatically determined based on joint limits specified in the URDF. This removes guesswork and simplifies setup.
- Configurable Frequencies and Phases: Users can easily adjust the frequency and phase of each sinusoid to test various dynamic conditions.
- ROS Integration: Uses ROS publishers and the trajectory_msgs/JointTrajectory message type to send commands to trajectory controllers.
- Simulation and Hardware Compatibility: The script can be used in simulation environments (e.g., Gazebo) or on hardware (with appropriate safety measures in place and proper controllers).

## Dependencies
- ROS: This script is intended to run in a ROS environment (tested with ROS Melodic/Noetic). Ensure you have a working ROS installation.
- URDF Parser: Requires urdf_parser_py for parsing the robot’s URDF. This can typically be installed via sudo apt-get install ros-<distro>-urdf-parser-py.
- Numpy: For handling numeric computations.
- Trajectory Controller: A running ROS controller capable of consuming JointTrajectory messages (e.g., ros_controllers).

## Installation and Setup
1. Clone the Repository:
cd ~/catkin_ws/src
git clone <this_repository_url>.git
cd ..
catkin_make
source devel/setup.bash
2. Load Robot Description on Parameter Server:
Ensure your Go1 robot’s URDF is loaded onto the parameter server (e.g., using robot_state_publisher or a launch file that loads the robot_description parameter).
3. Check Joint Names:
The script expects a certain naming convention for the Go1’s joints. By default, it uses names like FL_hip_joint, FL_thigh_joint, FL_calf_joint, etc., for each of the four legs (FL, FR, RL, RR). Confirm that the URDF and robot model you’re using follows this naming scheme.

## Usage
1.Starting a Simulation (Example with Gazebo):
roslaunch ucf_go1_bringup gazeboSim.launch
2.Running the Function Generator:
rosrun ucf_go1_util function_generator.py /traj_controller/command:=/go1_gazebo/traj_controller/command
3.
- Frequency: Currently set to 0.1 Hz for all joints. Modify the frequencies list in function_generator.py to change this.
- Duration and Update Rate: By default, the script generates a new trajectory with a 1-second duration and a 10 Hz update rate. Adjust duration, rate_hz, and pub_rate_hz within the script as needed.
- Phases: All phases are set to zero by default. Change the phases list to introduce phase shifts between joints.

## Notes and Best Practices
- Controller Requirements: The trajectory controller on the receiving end of these commands should be capable of smoothly executing JointTrajectory messages. Verify your controller setup before running the script.
- URDF Limits: Joint limits must be correctly defined in the URDF for the script to compute appropriate amplitudes and offsets. Double-check your URDF if you see unexpected motion ranges.
Troubleshooting:
- If no motion is observed, ensure the correct topic remapping and verify that the trajectory controller is running.
- Check ROS logs (rospy.loginfo, rospy.logerr) and console output for hints.
- Confirm that the generated JointTrajectory messages include valid joint names and that they match the controller’s expected joint list.

## function_generator.py
This class publishes sinusoidal joint trajectories for each of the 12 go1 joints
By default, the script publishes a sinusoid that operates over the joints limits centered
at the center of those limits. It publishes a sinusoid over a duration and then updates the trajectory
at some rate.

Example (remaps the default command output to gazebo command)
```
roslaunch ucf_go1_bringup gazeboSim.launch
rosrun ucf_go1_util function_generator.py /traj_controller/command:=/go1_gazebo/traj_controller/command
```
