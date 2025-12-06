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

  // Read optional foot force params so we can tune at runtime
  rpnh.param("use_footForceEst", use_footForceEst_, use_footForceEst_);
  rpnh.param("foot_force_scale", foot_force_scale_, foot_force_scale_);
  rpnh.param("foot_force_offset", foot_force_offset_, foot_force_offset_);
  rpnh.param("foot_force_sign", foot_force_sign_, foot_force_sign_);

  ROS_INFO_NAMED("go1_hw_interface",
                 "Foot force params: use_est=%d scale=%f offset=%f sign=%f",
                 (int)use_footForceEst_, foot_force_scale_, foot_force_offset_, foot_force_sign_);

  // Initialize publishers for sim-compatible topics (/visual/FL_foot_contact/the_force ...)
  std::array<std::string, 4> leg_names = {"FL", "FR", "RL", "RR"};
  for (int i = 0; i < 4; ++i)
  {
    std::string topic = "/visual/" + leg_names[i] + "_foot_contact/the_force";
    foot_force_pubs_[i] = nh_.advertise<geometry_msgs::WrenchStamped>(topic, 1);
  }
}

void Go1HWInterface::read(ros::Duration &elapsed_time) {
  if (control_mode_ == 0) {
    // No reading required in this mode.
    return;
  }
  udp_.Recv();
  udp_.GetRecv(state_);

  // Publish foot forces (converted & scaled) so the controller uses same topics as sim
  std::array<double, 4> foot_forces_n;
  for (int i = 0; i < 4; ++i)
  {
    // Choose which array to use. footForceEst is typically zero/unused on many units.
    int16_t raw = use_footForceEst_ ? state_.footForceEst[i] : state_.footForce[i];
    // Convert and scale; sign/offset params allow live calibration
    foot_forces_n[i] = foot_force_sign_ * (foot_force_scale_ * static_cast<double>(raw) + foot_force_offset_);
  }

  // Publish each as geometry_msgs::WrenchStamped on the sim topics
  for (int i = 0; i < 4; ++i)
  {
    geometry_msgs::WrenchStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = ""; // TODO: fill if we have a meaningful frame?
    msg.wrench.force.x = 0.0;
    msg.wrench.force.y = 0.0;
    msg.wrench.force.z = foot_forces_n[i];
    msg.wrench.torque.x = 0.0;
    msg.wrench.torque.y = 0.0;
    msg.wrench.torque.z = 0.0;
    foot_force_pubs_[i].publish(msg);
  }
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
