# ucf_go1_control
This package provides high level control functionality to control unitree_go1 gaits and output joint trajectory messages to be used with ros control
This package is agnostic to simulation or real robot control so long as the back-end implementation can support joint trajectory messages.

The highest level transformation can be represented as:
Twist -> Joint Trajectories

Inside this diagram we have smaller controlllers:

Twist->Body controller -> Leg Trajectory Controller -> Joint Trajectories
The body controller manages gaits and synchronization between each leg, and the leg controller manages trajectories for each joint in a given leg. 