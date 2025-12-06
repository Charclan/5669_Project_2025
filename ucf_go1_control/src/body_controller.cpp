#include "ucf_go1_control/body_controller.h"
#include "ros/ros.h"
#include <algorithm>
#include <cmath>
#include "message/LowlevelState.h"
using namespace ucf;

// TODO get these from unitree constants
const int kLegFL = 0;
const int kLegFR = 1;
const int kLegRL = 2;
const int kLegRR = 3;

BodyController::BodyController(QuadrupedRobot &robotModel, Gait gait, nav_msgs::Path swingProfile,
                               std::vector<geometry_msgs::Twist> velocityProfile, double loop_rate)
    : robotModel_(robotModel), gait_(gait), swingProfile_(swingProfile), currentPhase_(0.0),
      velocityProfile_(velocityProfile), loop_rate_(loop_rate)
{
  ros::NodeHandle pn("~");
  pn.param("imp_Kp", Kp_, Kp_);
  pn.param("imp_Kd", Kd_, Kd_);
  pn.param("imp_Kf", Kf_, Kf_);
  pn.param("imp_desired_force", desiredContactForce_, desiredContactForce_);
  pn.param("imp_contact_threshold", contactThreshold_, contactThreshold_);
  pn.param("imp_torque_limit", torqueLimit_, torqueLimit_);
  ROS_INFO("Impedance control params: Kp: %f, Kd: %f, Kf: %f, desired_force: %f, contact_threshold: %f, torque_limit: %f",
           Kp_, Kd_, Kf_, desiredContactForce_, contactThreshold_, torqueLimit_);

  int gait_i = static_cast<int>(gait_); // default fallback to existing value
  if (pn.getParam("gait", gait_i))
  {
    gait_ = static_cast<Gait>(gait_i);
    ROS_INFO("Loaded gait param: %d", gait_i);
  }
  else
  {
    ROS_INFO("gait param not set, using default %d", static_cast<int>(gait_));
  }

  std::array<std::string, 4> legNames = {"FL", "FR", "RL", "RR"};
  for (int i = 0; i < 4; ++i)
  {
    std::string topic = "/visual/" + legNames[i] + "_foot_contact/the_force";
    footWrenchSubs_[i] = pn.subscribe<geometry_msgs::WrenchStamped>(
        topic, 1, boost::bind(&BodyController::footWrenchCallback, this, _1, i));
    footWrenches_[i].wrench.force.x = 0.0;
    footWrenches_[i].wrench.force.y = 0.0;
    footWrenches_[i].wrench.force.z = 0.0;
    footWrenchFilteredZ_[i] = 0.0;
  }
}

void BodyController::footWrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg, int leg)
{
  std::lock_guard<std::mutex> lk(footWrenchMutex_);
  // store raw message
  footWrenches_[leg] = *msg;

  // simple exponential low-pass on z to reduce noise:
  double raw_z = msg->wrench.force.z;
  footWrenchFilteredZ_[leg] = footWrenchAlpha_ * raw_z + (1.0 - footWrenchAlpha_) * footWrenchFilteredZ_[leg];
}

// thread-safe accessor
double BodyController::getMeasuredFz(int leg, bool use_filtered)
{
  std::lock_guard<std::mutex> lk(footWrenchMutex_);
  if (use_filtered)
    return footWrenchFilteredZ_[leg];
  return footWrenches_[leg].wrench.force.z;
}

namespace
{
  double stanceWeightFromPhase(double phase)
  {
    // phase in [0,1), stance is [0,0.5)
    const double transition = 0.10; // 10% of phase for fade in/out

    if (phase < 0.0)
      phase = 0.0;
    if (phase >= 1.0)
      phase = std::fmod(phase, 1.0);

    if (phase >= 0.5)
    {
      // swing phase: no stance impedance
      return 0.0;
    }

    // stance phase: smooth fade in/out
    if (phase < transition)
    {
      // early stance: 0 -> 1
      return phase / transition;
    }
    else if (phase > (0.5 - transition))
    {
      // late stance: 1 -> 0
      return (0.5 - phase) / transition;
    }
    else
    {
      // mid stance: full impedance
      return 1.0;
    }
  }
} // anonymous namespace

trajectory_msgs::JointTrajectory BodyController::getJointTrajectory(const geometry_msgs::Twist &twist,
                                                                    const sensor_msgs::JointState &jointState,
                                                                    const geometry_msgs::WrenchStamped footForce[4],
                                                                    LowlevelState &lowState)
{

  trajectory_msgs::JointTrajectory trajMsg;
  trajMsg.header.frame_id = "base";
  trajMsg.header.stamp = ros::Time::now();
  trajMsg.joint_names = {"FR_hip_joint", "FR_thigh_joint", "FR_calf_joint", "FL_hip_joint",
                         "FL_thigh_joint", "FL_calf_joint", "RR_hip_joint", "RR_thigh_joint",
                         "RR_calf_joint", "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"};

  auto footPosVel = this->getFoot(twist, jointState, footForce);
  trajMsg.points.reserve(footPosVel[0].swingProfile.poses.size());
  for (int i = 0; i < footPosVel[0].swingProfile.poses.size(); ++i)
  {
    Vec34 position;
    Vec34 velocity;
    for (int j = 0; j < footPosVel.size(); ++j)
    {
      position.col(j)(0) = footPosVel[j].swingProfile.poses[i].pose.position.x;
      position.col(j)(1) = footPosVel[j].swingProfile.poses[i].pose.position.y;
      position.col(j)(2) = footPosVel[j].swingProfile.poses[i].pose.position.z;

      if (!velocityProfile_.empty())
      {
        velocity.col(j)(0) = footPosVel[j].velocityProfile[i].linear.x;
        velocity.col(j)(1) = footPosVel[j].velocityProfile[i].linear.y;
        velocity.col(j)(2) = footPosVel[j].velocityProfile[i].linear.z;
      }
    }
    // Do inverse kinematics over all feet positions obtaining 12 joint positions
    auto jointPositions = robotModel_.getQ(position, FrameType::BODY);
    // Compute Cartesian impedance torques (τ = JᵀF)
    Eigen::Matrix<double, 12, 1> jointTorques;
    jointTorques.setZero();

    // Gains (can be tuned per-leg if needed)
    // double Kp = 110.0; // N/m
    // double Kd = 18.0;  // Ns/m

    for (int leg = 0; leg < 4; ++leg)
    {
      // Vec3 p_des = position.col(leg);
      // Vec3 v_des = velocity.col(leg);

      // Vec3 p_now = robotModel_.getFootPosition(lowState, leg, FrameType::BODY);
      // Vec3 v_now = robotModel_.getFootVelocity(lowState, leg);
      // Mat3 J = robotModel_.getJaco(lowState, leg);

      // // Smooth phase-based stance weight in [0,1]
      // double w = stanceWeightFromPhase(footPhase_[leg]);

      // Vec3 F_leg = Kp * (p_des - p_now) + Kd * (v_des - v_now);
      // F_leg *= w; // fade forces in/out

      // jointTorques.segment<3>(leg * 3) = J.transpose() * F_leg;

      Vec3 p_des = position.col(leg);
      Vec3 v_des = velocity.col(leg);

      Vec3 p_now = robotModel_.getFootPosition(lowState, leg, FrameType::BODY);
      Vec3 v_now = robotModel_.getFootVelocity(lowState, leg);
      Mat3 J = robotModel_.getJaco(lowState, leg);

      // Smooth phase-based stance weight in [0,1]
      double w = stanceWeightFromPhase(footPhase_[leg]);

      // Position/velocity impedance term
      Vec3 F_imp = Kp_ * (p_des - p_now) + Kd_ * (v_des - v_now);

      // Force-control term (z-axis only) - compare measured foot force against desired contact force
      Vec3 F_force_ctrl;
      F_force_ctrl.setZero();

      // measured vertical foot force (from simulation/hardware). Be careful with sign conventions
      // double measured_Fz = footForce[leg].wrench.force.z;
      double measured_Fz = getMeasuredFz(leg, false); // uses filtered value by default
      ROS_INFO("measured_Fz: %f", measured_Fz);

      // If foot is in contact (measuredFz magnitude > threshold), use force control in z
      if (std::abs(measured_Fz) > contactThreshold_)
      {
        double errorFz = desiredContactForce_ - measured_Fz;
        // positive errorFz means we want *more* upward force (depends on sign conv)
        F_force_ctrl(2) = Kf_ * errorFz;
      }

      // Combine impedance and force control. Multiply by stance weight so swing legs are compliant.
      Vec3 F_leg = (F_imp + F_force_ctrl) * w;
      ROS_INFO("Leg %d: stance weight: %f, F_imp_z: %f, F_force_ctrl_z: %f, total_Fz: %f", leg, w, F_imp(2),
               F_force_ctrl(2), F_leg(2));

      // Convert to joint torques tau = J^T F
      Eigen::Matrix<double, 3, 1> tau3 = J.transpose() * F_leg;

      // Clamp per-joint torques to safe limits
      for (int i = 0; i < 3; ++i)
      {
        if (tau3(i) > torqueLimit_)
          tau3(i) = torqueLimit_;
        if (tau3(i) < -torqueLimit_)
          tau3(i) = -torqueLimit_;
      }

      jointTorques.segment<3>(leg * 3) = tau3;
    }

    // Drop nan-values to avoid simulation crashes. It would probably be better to
    // handle this properly so joints don't jump around if getQ fails.
    jointPositions = jointPositions.unaryExpr([](double x)
                                              { return std::isnan(x) ? 0 : x; });

    trajectory_msgs::JointTrajectoryPoint point;
    // Time_from_start is already given by the timestamped positions
    point.time_from_start =
        footPosVel[0].swingProfile.poses[i].header.stamp - footPosVel[0].swingProfile.poses.front().header.stamp;
    // Copy data from matrix to vector for messaging
    point.positions = std::vector<double>(jointPositions.data(),
                                          jointPositions.data() + jointPositions.rows() * jointPositions.cols());
    if (!velocityProfile_.empty())
    {
      auto jointVel = robotModel_.getQd(position, velocity, FrameType::BODY);
      jointVel = jointVel.unaryExpr([](double x)
                                    { return std::isnan(x) ? 0 : x; });
      point.velocities = std::vector<double>(jointVel.data(), jointVel.data() + jointVel.rows() * jointVel.cols());
    }
    point.effort = std::vector<double>(jointTorques.data(),
                                       jointTorques.data() + jointTorques.size());

    trajMsg.points.push_back(point);
  }
  return trajMsg;
}

std::array<BodyController::PositionVelocity, 4> BodyController::getFoot(const geometry_msgs::Twist &twist,
                                                                        const sensor_msgs::JointState &jointState,
                                                                        const geometry_msgs::WrenchStamped footForce[4])
{

  std::array<BodyController::PositionVelocity, 4> footPaths;

  // The first linear.x index is the velocity of the stance phase.
  const double vo = velocityProfile_[1].linear.x;
  // Desired velocity magnitude. Negative because foot moves backward with respect to the body to move forward
  double vd = -twist.linear.x;
  ROS_INFO_THROTTLE(1, "vo: %f, vd: %f", vo, vd);
  if (fabs(vo) < 1E-6 || fabs(vd) < 1E-6)
  {
    return footPaths;
  }
  double s = vd / vo;
  if (s <= 0)
  {
    return footPaths; // Return early for invalid scale factor
  }

  double xs = sqrt(s);           // Distance scale
  double duration = 1 / sqrt(s); // Time scale

  footPhase_ = phaseStateMachine(currentPhase_, gait_, twist.angular, footForce);
  auto current_ts = ros::Time::now();

  const int kLookaheadSteps = 100;

  for (int legId = 0; legId < footPhase_.size(); ++legId)
  {
    double legPhase = footPhase_[legId];
    // Initialize vectors as a copy of the original vector with a specified number of steps for lookahead
    std::copy_n(swingProfile_.poses.begin(), kLookaheadSteps, std::back_inserter(footPaths[legId].swingProfile.poses));
    if (!velocityProfile_.empty())
    {
      std::copy_n(velocityProfile_.begin(), kLookaheadSteps, std::back_inserter(footPaths[legId].velocityProfile));
    }

    int phaseIdx = min(legPhase * swingProfile_.poses.size(), swingProfile_.poses.size() - 1);
    for (int i = 0; i < kLookaheadSteps; ++i)
    {
      auto &pose = footPaths[legId].swingProfile.poses[i];
      pose.header.stamp = current_ts + ros::Duration(duration * i / swingProfile_.poses.size());
      pose.pose = swingProfile_.poses[phaseIdx].pose;
      // Scale the x distance by the desired position scale. This adjusts the step length as well as the horizontal
      // distance of the swing phase
      pose.pose.position.x *= xs;

      if (!velocityProfile_.empty())
      {
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
  for (auto &pose : footPaths[kLegFL].swingProfile.poses)
  {
    pose.pose.position.x += 0.1681;
    pose.pose.position.y += -0.1300;
    // pose.pose.position.z += -0.3200;
  }
  for (auto &pose : footPaths[kLegFR].swingProfile.poses)
  {
    pose.pose.position.x += 0.1681;
    pose.pose.position.y += 0.1300;
    // pose.pose.position.z += -0.3200;
  }
  for (auto &pose : footPaths[kLegRL].swingProfile.poses)
  {
    pose.pose.position.x += -0.2081;
    pose.pose.position.y += -0.1300;
    // pose.pose.position.z += -0.3200;
  }
  for (auto &pose : footPaths[kLegRR].swingProfile.poses)
  {
    pose.pose.position.x += -0.2081;
    pose.pose.position.y += 0.1300;
    // pose.pose.position.z += -0.3200;
  }

  return footPaths;
}

double BodyController::scalePhase(double phase)
{
  if (phase < 0.75)
  {
    double output_end = 0.50;
    double output_start = 0.0;
    double input_end = 0.75;
    double input_start = 0.0;
    double slope = 1.0 * (output_end - output_start) / (input_end - input_start);
    double contactPhase = output_start + slope * (phase - input_start);
    return contactPhase;
  }
  else
  {
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
                                                        const geometry_msgs::WrenchStamped footForce[4])
{
  // Example: just print for now (like in the branch)
  printf("\nFL: %lf FR:%lf RL:%lf RR:%lf\n",
         footForce[0].wrench.force.z,
         footForce[1].wrench.force.z,
         footForce[2].wrench.force.z,
         footForce[3].wrench.force.z);

  std::array<double, 4> footPhase;
  switch (gait)
  {
  case BodyController::Gait::kPassive:
  {
    footPhase[kLegFL] = 0.0;
    footPhase[kLegFR] = 0.0;
    footPhase[kLegRL] = 0.0;
    footPhase[kLegRR] = 0.0;
    // level out COM
    comAngle.x = 0.0;
    comAngle.y = 0.0;
    break;
  }
  case BodyController::Gait::kStand:
  {
    footPhase[kLegFL] = 0.0;
    footPhase[kLegFR] = 0.0;
    footPhase[kLegRL] = 0.0;
    footPhase[kLegRR] = 0.0;
    // level out COM
    comAngle.x = 0.0;
    comAngle.y = 0.0;
    break;
  }
  case BodyController::Gait::kWalk:
  {
    footPhase[kLegFL] = scalePhase(fmod(phase + 0.0, 1.0));
    footPhase[kLegFR] = scalePhase(fmod(phase + 0.5, 1.0));
    footPhase[kLegRL] = scalePhase(fmod(phase + 0.25, 1.0));
    footPhase[kLegRR] = scalePhase(fmod(phase + 0.75, 1.0));

    if (phase < 0.50)
    { // leans ensure stability on 3 legs
      // COM lean right
      comAngle.x = 0.50; // these angles are currently arbitrary
      comAngle.y = 0.50;
    }
    else if (phase >= 0.50)
    {
      // COM lean left
      comAngle.x = -0.50; // these angles are currently arbitrary
      comAngle.y = -0.50;
    }
    break;
  }
  case BodyController::Gait::kTrot:
  {
    footPhase[kLegFL] = fmod(phase + 0.5, 1.0);
    footPhase[kLegFR] = fmod(phase + 0.0, 1.0);
    footPhase[kLegRL] = fmod(phase + 0.0, 1.0);
    footPhase[kLegRR] = fmod(phase + 0.5, 1.0);
    break;
  }
  case BodyController::Gait::kCanter:
  {
    footPhase[kLegFL] = fmod(phase + 0.0, 1.0);
    footPhase[kLegFR] = fmod(phase + 0.3, 1.0);
    footPhase[kLegRL] = fmod(phase + 0.7, 1.0);
    footPhase[kLegRR] = fmod(phase + 0.0, 1.0);
    break;
  }
  case BodyController::Gait::kGallop:
  {
    footPhase[kLegFL] = fmod(phase + 0.0, 1.0);
    footPhase[kLegFR] = fmod(phase + 0.1, 1.0);
    footPhase[kLegRL] = fmod(phase + 0.6, 1.0);
    footPhase[kLegRR] = fmod(phase + 0.5, 1.0);
    break;
  }
  }
  return footPhase;
}

trajectory_msgs::JointTrajectory BodyController::getStandTrajectory(const std::vector<double> &targetPos,
                                                                    const sensor_msgs::JointState &jointState)
{
  trajectory_msgs::JointTrajectory trajMsg;
  trajMsg.header.frame_id = "base";
  trajMsg.header.stamp = ros::Time::now();
  trajMsg.joint_names = {"FR_hip_joint", "FR_thigh_joint", "FR_calf_joint", "FL_hip_joint",
                         "FL_thigh_joint", "FL_calf_joint", "RR_hip_joint", "RR_thigh_joint",
                         "RR_calf_joint", "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"};

  double duration = 5.0;
  for (int i = 0; i < 100; ++i)
  {
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.resize(trajMsg.joint_names.size());
    point.velocities.resize(trajMsg.joint_names.size());
    // Time_from_start is already given by the timestamped positions
    point.time_from_start = ros::Duration(duration * (i / 100.0));
    // Interpolate from current joint position to desired joint position.
    // This is a little convuluted due to joint state publisher being in different order
    // from the robot joints & controller order
    int j = 0;
    for (const auto &name : jointState.name)
    {
      auto it = std::find(trajMsg.joint_names.begin(), trajMsg.joint_names.end(), name);
      if (it != trajMsg.joint_names.end())
      {
        int idx = std::distance(trajMsg.joint_names.begin(), it);
        double distance = targetPos[idx] - jointState.position[j];
        point.positions[idx] = (jointState.position[j] + (i / 100.0) * distance);
        point.velocities[idx] = distance / duration;
        j++;
      }
    }
    trajMsg.points.push_back(point);
  }
  for (auto &vel : trajMsg.points.back().velocities)
  {
    vel = 0.0;
  }
  return trajMsg;
}

trajectory_msgs::JointTrajectory BodyController::getEmptyTrajectory()
{
  trajectory_msgs::JointTrajectory trajMsg;
  trajMsg.header.frame_id = "base";
  trajMsg.header.stamp = ros::Time::now();
  trajMsg.joint_names = {"FR_hip_joint", "FR_thigh_joint", "FR_calf_joint", "FL_hip_joint",
                         "FL_thigh_joint", "FL_calf_joint", "RR_hip_joint", "RR_thigh_joint",
                         "RR_calf_joint", "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"};
  return trajMsg;
}