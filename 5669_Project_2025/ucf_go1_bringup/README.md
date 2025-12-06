# ucf_go1_bringup
This is a bringup package for unitree go1 with UCF control and adjustments
This contains config required to launch and control both the real and simulated robot



Launch gazebo simulation
```
roslaunch ucf_go1_bringup gazeboSim.launch wname:=earth
```
Launch rviz
```
roslaunch ucf_go1_bringup go1_debug.launch
```

Launch controllers with specific gait
0 Passive (not working)
1 Stand (not working)
2 Walk
3 Trot
4 Canter
5 Gallop
```
roslaunch ucf_go1_bringup control.launch gait:=3 sim:=true
```

Modify ucf_go1_bringup/real_robot_control.yaml's control_mode to 0 for testing a simple simulated interface
```
roslaunch ucf_go1_bringup go1_hw.launch
roslaunch ucf_go1_bringup control.launch gait:=2
roslaunch ucf_go1_bringup go1_debug.launch 
```
Modify ucf_go1_bringup/real_robot_control.yaml's control_mode to 1 for testing udp launches fine (this is the mode for on robot)
```
roslaunch ucf_go1_bringup go1_hw.launch
```