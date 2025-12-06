#include <ucf_go1_control/go1_hw_interface.h>

#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace go1_control {
Go1HWInterface::Go1HWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
    : ros_control_boilerplate::GenericHWInterface(nh, urdf_model), safe_(LeggedType::Go1),
      udp_(LOWLEVEL, 8091, "192.168.123.10", 8007) {

  // Load rosparams
  ros::NodeHandle rpnh(nh_, "hardware_interface");
  std::size_t error = 0;
  error += !rosparam_shortcuts::get(name_, rpnh, "control_mode", control_mode_);
  if (error) {
    ROS_WARN_STREAM_NAMED(name_, "HWInterface requires the following config in the yaml:");
    ROS_WARN_STREAM_NAMED(name_, "   control_mode: 0 # 0: sim, 1: real");
  }

  error += !rosparam_shortcuts::get(name_, rpnh, "power_protect", power_protect_);
  if (error) {
    ROS_WARN_STREAM_NAMED(name_, "HWInterface requires the following config in the yaml:");
    ROS_WARN_STREAM_NAMED(name_, "   power_protect: 1 # 1 to 10");
  }

  // Load spring-damper config
  error += !rosparam_shortcuts::get(name_, rpnh, "Kp", kp_);
  if (error) {
    ROS_WARN_STREAM_NAMED(name_, "HWInterface requires the following config in the yaml:");
    ROS_WARN_STREAM_NAMED(name_, "   Kp: - 1.0 # Spring coefficient");
  }

  error += !rosparam_shortcuts::get(name_, rpnh, "Kd", kd_);
  if (error) {
    ROS_WARN_STREAM_NAMED(name_, "HWInterface requires the following config in the yaml:");
    ROS_WARN_STREAM_NAMED(name_, "   Kd: - 1.0 # Damper coefficient");
  }

  rosparam_shortcuts::shutdownIfError(name_, error);

  if (control_mode_ == 1) {
    udp_.InitCmdData(cmd_);
  }
  ROS_INFO_NAMED("go1_hw_interface", "Go1HWInterface Ready.");
}

void Go1HWInterface::read(ros::Duration &elapsed_time) {
  if (control_mode_ == 0) {
    // No reading required in this mode.
    return;
  }
  udp_.Recv();
  udp_.GetRecv(state_);
}

void Go1HWInterface::write(ros::Duration &elapsed_time) {
  // Safety
  enforceLimits(elapsed_time);

  // Sim mode
  if (control_mode_ == 0) {
    for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id) {
      joint_velocity_[joint_id] = joint_velocity_command_[joint_id];
      joint_position_[joint_id] += joint_velocity_command_[joint_id] * elapsed_time.toSec();
    }
    return;
  }

  for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id) {
    joint_position_[joint_id] = state_.motorState[joint_id].q;
    joint_velocity_[joint_id] = state_.motorState[joint_id].dq;
    joint_effort_[joint_id] = state_.motorState[joint_id].tauEst;

    cmd_.motorCmd[joint_id].mode = PMSM;
    cmd_.motorCmd[joint_id].q = PosStopF;
    cmd_.motorCmd[joint_id].dq = joint_velocity_command_[joint_id]; // Update joint velocity
    cmd_.motorCmd[joint_id].Kp = kp_[joint_id]; // Spring coefficient
    cmd_.motorCmd[joint_id].Kd = kd_[joint_id]; // Damper coefficient
    // cmd_.motorCmd[joint_id].tau = 0;
  }
  auto res = safe_.PowerProtect(cmd_, state_, power_protect_);
  if (res < 0) {
    ROS_WARN_STREAM_NAMED(name_, "PowerProtect triggered! res: " << res);
    // Don't send the command if PowerProtect is triggered.
    return;
  }
  udp_.SetSend(cmd_);
  udp_.Send();
}

void Go1HWInterface::enforceLimits(ros::Duration &period) {
  vel_jnt_sat_interface_.enforceLimits(period);
  // vel_jnt_soft_limits_.enforceLimits(period);
}

} // namespace go1_control
