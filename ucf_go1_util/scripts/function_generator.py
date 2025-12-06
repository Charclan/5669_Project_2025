#!/usr/bin/env python

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
from urdf_parser_py.urdf import URDF


def generate_sinusoidal_trajectory(
    joint_names, time_array, amplitudes, frequencies, phases, offsets
):
    """
    Generate a JointTrajectory message for multiple joints following sinusoidal profiles.

    :param joint_names: List of joint names (list of strings).
    :param time_array: Array of time steps (numpy array).
    :param amplitudes: List of amplitudes for each joint (list of floats).
    :param frequencies: List of frequencies for each joint in Hz (list of floats).
    :param phases: List of phases for each joint in radians (list of floats).
    :param offsets: List of offsets for each joint (list of floats).
    :return: JointTrajectory message.
    """
    if not (
        len(joint_names)
        == len(amplitudes)
        == len(frequencies)
        == len(phases)
        == len(offsets)
    ):
        raise ValueError(
            "All parameter lists must have the same length as joint_names."
        )

    # Initialize the trajectory message
    traj_msg = JointTrajectory()
    traj_msg.header.stamp = rospy.Time.now()
    traj_msg.joint_names = joint_names

    # Generate trajectory points
    for t in time_array:
        point = JointTrajectoryPoint()
        positions = []
        velocities = []
        # Compute positions for each joint
        for i in range(len(joint_names)):
            # Generate only joint positions. No FF for velocity provided in this utility currently
            position = (
                amplitudes[i] * np.sin(2 * np.pi * frequencies[i] * t + phases[i])
                + offsets[i]
            )
            positions.append(position)

            # Velocity (derivative of position)
            velocity = 2 * np.pi * frequencies[i] * amplitudes[i] * np.cos(2 * np.pi * frequencies[i] * t + phases[i])
            velocities.append(velocity)

        point.positions = positions
        point.velocities = velocities
        point.time_from_start = rospy.Duration(t - time_array[0])
        traj_msg.points.append(point)

    return traj_msg


def main():
    rospy.init_node("sinusoidal_trajectory_publisher")
    pub = rospy.Publisher("/traj_controller/command", JointTrajectory, queue_size=10)
    robot = URDF.from_parameter_server()

    # Make the joint names matching the controller order
    joint_names = [
        f"{prefix}_{suffix}_joint"
        for prefix in ["FL", "FR", "RL", "RR"]
        for suffix in ["hip","thigh", "calf"]
    ]

    # The amplitude is half the total distance over the joints
    amplitudes = [
        (robot.joint_map[j].limit.upper - robot.joint_map[j].limit.lower) / 2.0
        for j in joint_names
    ]

    # Currently just specify the frequency for each joint
    frequencies = [0.1 for j in joint_names]  # Frequency for each joint in Hz

    # Make each sinusoid in the same phase
    phases = [0.0 for j in joint_names]  # Phase for each joint in radians

    # Offset each sine such that the center of the sinusoid is the center of the joint limits
    offsets = [
        robot.joint_map[j].limit.lower + amplitudes[i]
        for i, j in enumerate(joint_names)
    ]
    duration = 1.0  # Total duration of the trajectory in seconds
    rate_hz = 100  # Frequency at which points are generated
    pub_rate_hz = 10  # Frequency at which trajectories are updated

    rate = rospy.Rate(pub_rate_hz)
    while not rospy.is_shutdown():
        # Generate time array
        t_start = rospy.Time.now()
        # Generate the trajectory message
        time_array = np.linspace(
            t_start.to_sec(), t_start.to_sec() + duration, int(duration * rate_hz)
        )
        # Make the trajectory message and publish it
        traj_msg = generate_sinusoidal_trajectory(
            joint_names, time_array, amplitudes, frequencies, phases, offsets
        )
        pub.publish(traj_msg)
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
