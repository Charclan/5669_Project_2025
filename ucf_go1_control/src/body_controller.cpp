#include "ucf_go1_control/body_controller.h"
#include "ros/ros.h"
#include <algorithm>
#include <cmath>
using namespace ucf;

// TODO get these from unitree constants
const int kLegFL = 0;
const int kLegFR = 1;
const int kLegRL = 2;
const int kLegRR = 3;

BodyController::BodyController(QuadrupedRobot &robotModel, Gait gait, nav_msgs::Path swingProfile,
                               std::vector<geometry_msgs::Twist> velocityProfile, double loop_rate)
    : robotModel_(robotModel), gait_(gait), swingProfile_(swingProfile), currentPhase_(0.0),
      velocityProfile_(velocityProfile), loop_rate_(loop_rate) {

    // --- Step 1: Initialize FSM state & contact ---
    for (int i = 0; i < 4; ++i) {
        leg_phase_[i] = LegPhase::kStance;   // start in stance
        leg_contact_[i] = false;
    }
}

void BodyController::computeVerticalImpedanceTorques(
    const sensor_msgs::JointState &jointState,
    const geometry_msgs::WrenchStamped footForce[4],
    std::array<double, 12> &torques_out)
{
  // Start with zero torques (safe default).
  torques_out.fill(0.0);

  // --- 1) Build Vec12 q in a fixed canonical order ---
  // This order matches trajMsg.joint_names in getJointTrajectory
  // and the Unitree model's expected q layout.
  static const std::array<std::string, 12> kJointOrder = {
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"};

  Vec12 q;
  q.setZero();

  bool mapping_ok = true;
  for (int i = 0; i < 12; ++i) {
    const auto &joint_name = kJointOrder[i];

    auto it = std::find(jointState.name.begin(), jointState.name.end(), joint_name);
    if (it == jointState.name.end()) {
      ROS_ERROR_THROTTLE(1.0,
                         "computeVerticalImpedanceTorques: joint '%s' not found in JointState.",
                         joint_name.c_str());
      mapping_ok = false;
      break;
    }
    int idx_js = std::distance(jointState.name.begin(), it);
    if (idx_js >= static_cast<int>(jointState.position.size())) {
      ROS_ERROR_THROTTLE(1.0,
                         "computeVerticalImpedanceTorques: joint '%s' has no position entry.",
                         joint_name.c_str());
      mapping_ok = false;
      break;
    }
    q[i] = jointState.position[idx_js];
  }

  if (!mapping_ok) {
    // Leave torques_out as zeros if we can't build a consistent q.
    return;
  }

  // --- 2) Build Vec34 feetForceCmd with vertical commands in stance legs only ---
  Vec34 feetForceCmd;
  feetForceCmd.setZero();

  for (int leg = 0; leg < 4; ++leg) {
    // Only apply impedance for stance legs (FSM from phaseStateMachine)
    if (leg_phase_[leg] != LegPhase::kStance) {
      continue;
    }

    // Measured vertical force for this leg from contact sensors
    double Fz_meas = footForce[leg].wrench.force.z;

    // Simple force regulation around imp_Fz_ref_
    double Fz_err = imp_Fz_ref_ - Fz_meas;
    double Fz_cmd = imp_F_gain_ * Fz_err;  // [N]

    // Fill z component (row 2) of the feetForceCmd for this leg.
    // Vec34 is used in getQ/getQd with position.col(j)(0/1/2) = x/y/z,
    // so we use the same convention here: row 2 = z.
    feetForceCmd(2, leg) = Fz_cmd;
  }

  // --- 3) Let Unitree model compute τ = J^T F for all legs at once ---
  Vec12 tau = robotModel_.getTau(q, feetForceCmd);

  // --- 4) Copy into torques_out with saturation for safety ---
  for (int i = 0; i < 12; ++i) {
    double t = tau[i];
    if (t > imp_tau_max_)  t = imp_tau_max_;
    if (t < -imp_tau_max_) t = -imp_tau_max_;
    torques_out[i] = t;
  }
}



trajectory_msgs::JointTrajectory BodyController::getJointTrajectory( const geometry_msgs::Twist &twist,
                                                                     const sensor_msgs::JointState &jointState,
                                                                     const geometry_msgs::WrenchStamped footForce[4]) {

  trajectory_msgs::JointTrajectory trajMsg;
  trajMsg.header.frame_id = "base";
  trajMsg.header.stamp = ros::Time::now();
  trajMsg.joint_names = {
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"
};


  auto footPosVel = this->getFoot(twist, jointState, footForce);
  trajMsg.points.reserve(footPosVel[0].swingProfile.poses.size());

  // --- Compute vertical impedance torques for this cycle ---
  std::array<double, 12> impedance_torques;
  computeVerticalImpedanceTorques(jointState, footForce, impedance_torques);


  for (int i = 0; i < footPosVel[0].swingProfile.poses.size(); ++i) {
    Vec34 position;
    Vec34 velocity;
    for (int j = 0; j < footPosVel.size(); ++j) {
      position.col(j)(0) = footPosVel[j].swingProfile.poses[i].pose.position.x;
      position.col(j)(1) = footPosVel[j].swingProfile.poses[i].pose.position.y;
      position.col(j)(2) = footPosVel[j].swingProfile.poses[i].pose.position.z;

      if (!velocityProfile_.empty()) {
        velocity.col(j)(0) = footPosVel[j].velocityProfile[i].linear.x;
        velocity.col(j)(1) = footPosVel[j].velocityProfile[i].linear.y;
        velocity.col(j)(2) = footPosVel[j].velocityProfile[i].linear.z;
      }
    }
    // Do inverse kinematics over all feet positions obtaining 12 joint positions
    auto jointPositions = robotModel_.getQ(position, FrameType::BODY);
    // Drop nan-values to avoid simulation crashes. It would probably be better to
    // handle this properly so joints don't jump around if getQ fails.
    jointPositions = jointPositions.unaryExpr([](double x) { return std::isnan(x) ? 0 : x; });

    trajectory_msgs::JointTrajectoryPoint point;
    // Time_from_start is already given by the timestamped positions
    point.time_from_start =
        footPosVel[0].swingProfile.poses[i].header.stamp - footPosVel[0].swingProfile.poses.front().header.stamp;
    // Copy data from matrix to vector for messaging



    point.positions = std::vector<double>(jointPositions.data(),
                                          jointPositions.data() + jointPositions.rows() * jointPositions.cols());
    if (!velocityProfile_.empty()) {
      auto jointVel = robotModel_.getQd(position, velocity, FrameType::BODY);
      jointVel = jointVel.unaryExpr([](double x) { return std::isnan(x) ? 0 : x; });
      point.velocities = std::vector<double>(jointVel.data(), jointVel.data() + jointVel.rows() * jointVel.cols());
    }

    // NEW: fill effort
    point.effort.resize(jointState.name.size());
    for (std::size_t j = 0; j < jointState.name.size(); ++j) {
      if (j < impedance_torques.size()) {
        point.effort[j] = impedance_torques[j];
      } else {
        point.effort[j] = 0.0;
      }
    }

    trajMsg.points.push_back(point);
  }
  return trajMsg;
}

std::array<BodyController::PositionVelocity, 4> BodyController::getFoot(const geometry_msgs::Twist &twist,
                                                                        const sensor_msgs::JointState &jointState,
                                                                        const geometry_msgs::WrenchStamped footForce[4]) {

  std::array<BodyController::PositionVelocity, 4> footPaths;

  // The first linear.x index is the velocity of the stance phase.
  const double vo = velocityProfile_[1].linear.x;
  // Desired velocity magnitude. Negative because foot moves backward with respect to the body to move forward
  double vd = -twist.linear.x;
  ROS_INFO_THROTTLE(1, "vo: %f, vd: %f", vo, vd);
  if (fabs(vo) < 1E-6 || fabs(vd) < 1E-6) {
    return footPaths;
  }
  double s = vd / vo;
  if (s <= 0) {
    return footPaths; // Return early for invalid scale factor
  }

  double xs = sqrt(s);           // Distance scale
  double duration = 1 / sqrt(s); // Time scale

  footPhase_ = phaseStateMachine(currentPhase_, gait_, twist.angular, footForce);
  auto current_ts = ros::Time::now();

  const int kLookaheadSteps = 100;

  for (int legId = 0; legId < footPhase_.size(); ++legId) {
    double legPhase = footPhase_[legId];
    // Initialize vectors as a copy of the original vector with a specified number of steps for lookahead
    std::copy_n(swingProfile_.poses.begin(), kLookaheadSteps, std::back_inserter(footPaths[legId].swingProfile.poses));
    if (!velocityProfile_.empty()) {
      std::copy_n(velocityProfile_.begin(), kLookaheadSteps, std::back_inserter(footPaths[legId].velocityProfile));
    }

    int phaseIdx = min(legPhase * swingProfile_.poses.size(), swingProfile_.poses.size() - 1);
    for (int i = 0; i < kLookaheadSteps; ++i) {
      auto &pose = footPaths[legId].swingProfile.poses[i];
      pose.header.stamp = current_ts + ros::Duration(duration * i / swingProfile_.poses.size());
      pose.pose = swingProfile_.poses[phaseIdx].pose;
      // Scale the x distance by the desired position scale. This adjusts the step length as well as the horizontal
      // distance of the swing phase
      pose.pose.position.x *= xs;

      if (!velocityProfile_.empty()) {
        auto &twist = footPaths[legId].velocityProfile[i];
        twist = velocityProfile_[phaseIdx];
        twist.linear.x /= duration;
        twist.linear.y /= duration;
        twist.linear.z /= duration;
      }

      // Move phase pointer, looping over the end of the vector
      phaseIdx = ++phaseIdx % swingProfile_.poses.size();
    }
  }
  // Loop overall phase to keep it between [0,1)
  // This 20 needs to be from the publishing rate to progress phase until we track it explicitly
  currentPhase_ = fmod(currentPhase_ + (1 / (duration * loop_rate_)), 1.0);

  // Shift for proper frame (rel body frame)
  for (auto &pose : footPaths[kLegFL].swingProfile.poses) {
    pose.pose.position.x += 0.1681;
    pose.pose.position.y += -0.1300;
    // pose.pose.position.z += -0.3200;
  }
  for (auto &pose : footPaths[kLegFR].swingProfile.poses) {
    pose.pose.position.x += 0.1681;
    pose.pose.position.y += 0.1300;
    // pose.pose.position.z += -0.3200;
  }
  for (auto &pose : footPaths[kLegRL].swingProfile.poses) {
    pose.pose.position.x += -0.2081;
    pose.pose.position.y += -0.1300;
    // pose.pose.position.z += -0.3200;
  }
  for (auto &pose : footPaths[kLegRR].swingProfile.poses) {
    pose.pose.position.x += -0.2081;
    pose.pose.position.y += 0.1300;
    // pose.pose.position.z += -0.3200;
  }

  return footPaths;
}

double BodyController::scalePhase(double phase) {
  if (phase < 0.75) {
    double output_end = 0.50;
    double output_start = 0.0;
    double input_end = 0.75;
    double input_start = 0.0;
    double slope = 1.0 * (output_end - output_start) / (input_end - input_start);
    double contactPhase = output_start + slope * (phase - input_start);
    return contactPhase;
  } else {
    double output_end = 1.0;
    double output_start = 0.5;
    double input_end = 1.0;
    double input_start = 0.75;
    double slope = 1.0 * (output_end - output_start) / (input_end - input_start);
    double swingPhase = output_start + slope * (phase - input_start);
    return swingPhase;
  }
}

std::array<double, 4> BodyController::phaseStateMachine(double phase, BodyController::Gait gait,
                                                        geometry_msgs::Vector3 comAngle,
                                                        const geometry_msgs::WrenchStamped footForce[4]) {
   // Example: just print for now (like in the branch)
  printf("\nFL: %lf FR:%lf RL:%lf RR:%lf\n",
         footForce[0].wrench.force.z,
         footForce[1].wrench.force.z,
         footForce[2].wrench.force.z,
         footForce[3].wrench.force.z);

  std::array<double, 4> footPhase;
  switch (gait) {
  case BodyController::Gait::kPassive: {
    footPhase[kLegFL] = 0.0;
    footPhase[kLegFR] = 0.0;
    footPhase[kLegRL] = 0.0;
    footPhase[kLegRR] = 0.0;
    // level out COM
    comAngle.x = 0.0;
    comAngle.y = 0.0;
    break;
  }
  case BodyController::Gait::kStand: {
    footPhase[kLegFL] = 0.0;
    footPhase[kLegFR] = 0.0;
    footPhase[kLegRL] = 0.0;
    footPhase[kLegRR] = 0.0;
    // level out COM
    comAngle.x = 0.0;
    comAngle.y = 0.0;
    break;
  }
  case BodyController::Gait::kWalk: {
    footPhase[kLegFL] = scalePhase(fmod(phase + 0.0, 1.0));
    footPhase[kLegFR] = scalePhase(fmod(phase + 0.5, 1.0));
    footPhase[kLegRL] = scalePhase(fmod(phase + 0.25, 1.0));
    footPhase[kLegRR] = scalePhase(fmod(phase + 0.75, 1.0));

    if (phase < 0.50) { // leans ensure stability on 3 legs
      // COM lean right
      comAngle.x = 0.50; // these angles are currently arbitrary
      comAngle.y = 0.50;
    } else if (phase >= 0.50) {
      // COM lean left
      comAngle.x = -0.50; // these angles are currently arbitrary
      comAngle.y = -0.50;
    }
    break;
  }
  case BodyController::Gait::kTrot: {
    footPhase[kLegFL] = fmod(phase + 0.5, 1.0);
    footPhase[kLegFR] = fmod(phase + 0.0, 1.0);
    footPhase[kLegRL] = fmod(phase + 0.0, 1.0);
    footPhase[kLegRR] = fmod(phase + 0.5, 1.0);
    break;
  }
  case BodyController::Gait::kCanter: {
    footPhase[kLegFL] = fmod(phase + 0.0, 1.0);
    footPhase[kLegFR] = fmod(phase + 0.3, 1.0);
    footPhase[kLegRL] = fmod(phase + 0.7, 1.0);
    footPhase[kLegRR] = fmod(phase + 0.0, 1.0);
    break;
  }
  case BodyController::Gait::kGallop: {
    footPhase[kLegFL] = fmod(phase + 0.0, 1.0);
    footPhase[kLegFR] = fmod(phase + 0.1, 1.0);
    footPhase[kLegRL] = fmod(phase + 0.6, 1.0);
    footPhase[kLegRR] = fmod(phase + 0.5, 1.0);
    break;
  }
  }

      // --- STEP 1 (revised): contact-based early touchdown ONLY ---
  for (int leg = 0; leg < 4; ++leg) {

    double Fz = footForce[leg].wrench.force.z;

    // contact only when force is clearly non-zero
    bool contact = (Fz > contact_force_threshold_);
    leg_contact_[leg] = contact;

    // nominal swing = phase > 0.5, stance = phase <= 0.5 (given your current gaits)
    bool nominalSwing  = (footPhase[leg] > 0.5);
    bool nominalStance = !nominalSwing;

    switch (leg_phase_[leg]) {

    case LegPhase::kSwing:
      // Swing -> Stance: only if we are in nominal swing AND get a solid contact
      if (nominalSwing && contact) {
        leg_phase_[leg] = LegPhase::kStance;

        // instead of jumping all the way back to 0,
        // keep phase in a small stance-ish region to avoid big discontinuities
        if (footPhase[leg] > 0.6) {
          footPhase[leg] = 0.6;   // small snap
        }
      }
      break;

    case LegPhase::kStance:
      // For now: DO NOT use contact to start swing.
      // Let the original phase timing handle stance -> swing
      // (prevents choppy early liftoff).
      break;
    }
  }

  return footPhase;
}


trajectory_msgs::JointTrajectory BodyController::getStandTrajectory(const std::vector<double> &targetPos,
                                                                    const sensor_msgs::JointState &jointState) {
  trajectory_msgs::JointTrajectory trajMsg;
  trajMsg.header.frame_id = "base";
  trajMsg.header.stamp = ros::Time::now();
  trajMsg.joint_names = {"FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"};

  double duration = 5.0;
  for (int i = 0; i < 100; ++i) {
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.resize(trajMsg.joint_names.size());
    point.velocities.resize(trajMsg.joint_names.size());
    // Time_from_start is already given by the timestamped positions
    point.time_from_start = ros::Duration(duration * (i / 100.0));
    // Interpolate from current joint position to desired joint position.
    // This is a little convuluted due to joint state publisher being in different order
    // from the robot joints & controller order
    int j = 0;
    for (const auto &name : jointState.name) {
      auto it = std::find(trajMsg.joint_names.begin(), trajMsg.joint_names.end(), name);
      if (it != trajMsg.joint_names.end()) {
        int idx = std::distance(trajMsg.joint_names.begin(), it);
        double distance = targetPos[idx] - jointState.position[j];
        point.positions[idx] = (jointState.position[j] + (i / 100.0) * distance);
        point.velocities[idx] = distance / duration;
        j++;
      }
    }
    trajMsg.points.push_back(point);
  }
  for (auto &vel : trajMsg.points.back().velocities) {
    vel = 0.0;
  }
  return trajMsg;
}

trajectory_msgs::JointTrajectory BodyController::getEmptyTrajectory() {
  trajectory_msgs::JointTrajectory trajMsg;
  trajMsg.header.frame_id = "base";
  trajMsg.header.stamp = ros::Time::now();
  trajMsg.joint_names = {"FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"};
  return trajMsg;
}