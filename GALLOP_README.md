# Gallop Gait Optimization

## Overview
Implemented and optimized gallop gait (gait:=5) for Unitree Go1 in Gazebo simulation.

## Key Changes

### gazebo.xacro
- Disabled depth camera for RTF improvement (0.6 → 0.97)
- Increased foot contact sensor rate to 200 Hz
- IMU rate reduced to 500 Hz

### body_controller.cpp
- Added COM pitch compensation for gallop stability
- Adjusted phase offsets for better ground contact overlap
- FL: 0.0, FR: 0.15, RL: 0.55, RR: 0.45

## Results
- Upright time improved from ~5s to ~16s
- RTF improved from 0.6 to 0.97

## Usage
```bash
# Terminal 1
roslaunch ucf_go1_bringup gazeboSim.launch wname:=earth

# Terminal 2
roslaunch ucf_go1_bringup control.launch gait:=5 sim:=true

# Terminal 3
rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.25}" -r 10
```

## Data Logging
```bash
python3 ~/catkin_ws/src/ucf_go1_bringup/scripts/log_gait_data.py
python3 ~/catkin_ws/src/ucf_go1_bringup/scripts/plot_gait_data.py <csv_file>
```
