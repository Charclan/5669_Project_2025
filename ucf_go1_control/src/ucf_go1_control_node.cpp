
#include "common/unitreeRobot.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"
#include <cmath>
#include <iostream>
#include <memory>
#include <ros/package.h>
// #include <unitree_legged_msgs/LowState.h>
#include "ucf_go1_control/body_controller.h"
#include "ucf_go1_control/csv_loader.h"
#include <geometry_msgs/WrenchStamped.h>
#include "message/LowlevelState.h"

geometry_msgs::WrenchStamped footForce[4];

ros::Subscriber sub_FL;
ros::Subscriber sub_FR;
ros::Subscriber sub_RL;
ros::Subscriber sub_RR;

// SIM callbacks (Gazebo contact sensors)
void FLfootCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg)
{
  footForce[0].wrench.force.z = msg->wrench.force.z; // FL
}
void FRfootCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg)
{
  footForce[1].wrench.force.z = msg->wrench.force.z; // FR
}
void RLfootCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg)
{
  footForce[2].wrench.force.z = msg->wrench.force.z; // RL
}
void RRfootCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg)
{
  footForce[3].wrench.force.z = msg->wrench.force.z; // RR
}

// HARDWARE callback (real robot low state)
// void lowStateCallback(const unitree_legged_msgs::LowState &msg) {
// use foot_force_est (estimated foot forces) – better for contact
//  for (int i = 0; i < 4; ++i) {
//    footForce[i].wrench.force.z = msg.foot_force_est[i];
//  }
//}

// Make a simple cycloidal profile for testing purposes
// Phase should be [0,1). A complete Cycloid profile
// and the closed base is generated over 100 total steps. 50 are in the base
// and 50 are in the curve.
nav_msgs::Path makeCycloid()
{
  auto ts = ros::Time::now();
  nav_msgs::Path cycloidPath{};
  cycloidPath.header.stamp = ts;
  cycloidPath.header.frame_id = "base";

  double kHeight = 0.1;
  double kHeightOffset = -0.225;
  double kStepLength = 0.1;
  int steps = 100;
  int kSwingSteps = 25;
  for (int t = 0; t < kSwingSteps; ++t)
  {
    geometry_msgs::PoseStamped pose;
    float phasePI = 2 * M_PI * t / kSwingSteps;
    pose.header.frame_id = "base";
    pose.pose.position.x = -kStepLength + 2 * kStepLength * (phasePI - sin(phasePI)) / (2 * M_PI);
    pose.pose.position.z = kHeight * (1 - cos(phasePI)) / 2 + kHeightOffset;
    cycloidPath.poses.push_back(pose);
  }
  int kContactSteps = steps - kSwingSteps;
  for (int t = kSwingSteps; t < steps; ++t)
  {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "base";
    pose.pose.position.x = -kStepLength + 2 * kStepLength * (steps - t) / kContactSteps;
    pose.pose.position.z = kHeightOffset;
    cycloidPath.poses.push_back(pose);
  }
  return cycloidPath;
}

/// @brief Make a profile from a loaded csv
/// @param parsedCsv The parsed csv file (rows, columns)
/// @return Path, VectorTwist pair
std::pair<nav_msgs::Path, std::vector<geometry_msgs::Twist>>
makeProfileFromCsv(const std::vector<std::vector<std::string>> &parsedCsv)
{
  nav_msgs::Path path;
  std::vector<geometry_msgs::Twist> vel;
  path.header.frame_id = "base";
  for (const auto &line : parsedCsv)
  {
    double X, Z, Vx, Vz;
    Z = std::stod(line[0]);
    X = std::stod(line[1]);
    Vz = std::stod(line[2]);
    Vx = std::stod(line[3]);
    geometry_msgs::PoseStamped pose{};
    pose.pose.position.x = X;
    pose.pose.position.z = Z;
    path.poses.push_back(pose);
    geometry_msgs::Twist twist{};
    twist.linear.x = Vx;
    twist.linear.z = Vz;
    vel.push_back(twist);
  }
  return {path, vel};
}

LowlevelState jointStateToLowlevelState(const sensor_msgs::JointState &jointMsg)
{
  LowlevelState state;
  for (size_t i = 0; i < jointMsg.name.size(); ++i)
  {
    if (i < 12)
    {
      state.motorState[i].q = jointMsg.position[i];
      state.motorState[i].dq = jointMsg.velocity[i];
    }
  }
  return state;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ucf_go1_control");
  ros::NodeHandle nh;

  // CHANGE BELOW TO FALSE WHEN USING THE REAL ROBOT IN ORDER TO GET THE ACTUAL FOOT FORCE DATA
  bool use_sim = true;
  if (nh.hasParam("use_sim"))
  {
    nh.getParam("use_sim", use_sim);
  }

  ucf::BodyController::Gait gait;
  if (nh.hasParam("gait"))
  {
    int gaitParam;
    nh.getParam("gait", gaitParam);
    gait = static_cast<ucf::BodyController::Gait>(gaitParam);
  }
  else
  {
    gait = ucf::BodyController::Gait::kPassive;
  }

  std::string profileStr;
  if (nh.hasParam("profile"))
  {
    nh.getParam("profile", profileStr);
  }

  double profile_height_offset = 0.0;
  if (nh.hasParam("profile_height_offset"))
  {
    nh.getParam("profile_height_offset", profile_height_offset);
  }

  std::vector<double> stand_joint = {0.0, 0.67, -1.3, 0.0, 0.67, -1.3, 0.0, 0.67, -1.3, 0.0, 0.67, -1.3};
  if (nh.hasParam("stand_joint"))
  {
    nh.getParam("stand_joint", stand_joint);
  }

  double trajectory_publish_rate = 50.0;
  if (nh.hasParam("trajectory_publish_rate"))
  {
    nh.getParam("trajectory_publish_rate", trajectory_publish_rate);
  }

  auto robotModel = std::make_unique<Go1Robot>();

  geometry_msgs::Twist currentTwist;
  sensor_msgs::JointState currentState;

  nav_msgs::Path swingProfile;
  std::vector<geometry_msgs::Twist> velProfile;
  if (profileStr.empty())
  {
    swingProfile = makeCycloid();
  }
  else
  {
    auto csv = ucf::loadCSV(ros::package::getPath("ucf_go1_control") + "/profiles/" + profileStr);
    auto posVel = makeProfileFromCsv(csv);
    swingProfile = posVel.first;
    velProfile = posVel.second;
    for (auto &pose : swingProfile.poses)
    {
      pose.pose.position.z += profile_height_offset; // Shift profile upward
      pose.pose.position.x *= 1.0;                   // Scale the step size
    }
  }

  auto bodyController =
      std::make_unique<ucf::BodyController>(*robotModel, gait, swingProfile, velProfile, trajectory_publish_rate);

  ros::Subscriber sub_twist =
      nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, [&currentTwist](const geometry_msgs::Twist::ConstPtr &msg)
                                         {
        ROS_INFO_THROTTLE(0.5, "Twist received x: {%f} y: {%f} yaw: {%f}", msg->linear.x, msg->linear.y,
                          msg->angular.z);
        currentTwist = *msg; });

  bool receivedState = false;
  ros::Subscriber sub_joint = nh.subscribe<sensor_msgs::JointState>(
      "joint_state", 1, [&currentState, &receivedState](const sensor_msgs::JointState::ConstPtr &msg)
      {
        ROS_INFO_THROTTLE(1, "Joint State received.");
        receivedState = true;
        currentState = *msg; });

  // Foot force subscribers
  // Foot force subscribers
  if (use_sim)
  {
    sub_FL = nh.subscribe<geometry_msgs::WrenchStamped>(
        "/visual/FL_foot_contact/the_force", 1, &FLfootCallback);
    sub_FR = nh.subscribe<geometry_msgs::WrenchStamped>(
        "/visual/FR_foot_contact/the_force", 1, &FRfootCallback);
    sub_RL = nh.subscribe<geometry_msgs::WrenchStamped>(
        "/visual/RL_foot_contact/the_force", 1, &RLfootCallback);
    sub_RR = nh.subscribe<geometry_msgs::WrenchStamped>(
        "/visual/RR_foot_contact/the_force", 1, &RRfootCallback);
  }
  else
  {
    // hardware path (when unitree_legged_msgs is available)
    // sub_low_state = nh.subscribe<unitree_legged_msgs::LowState>(
    //     "/low_state", 1, &lowStateCallback);
  }

  ros::Publisher pub_traj = nh.advertise<trajectory_msgs::JointTrajectory>("command", 1);
  // Publish path for debugging purposes
  ros::Publisher pub_profile = nh.advertise<nav_msgs::Path>("path", 1, true);
  pub_profile.publish(swingProfile);

  ros::Rate rate(trajectory_publish_rate);
  bool oneShot = false;
  while (ros::ok())
  {
    if (gait == ucf::BodyController::Gait::kStand)
    {
      if (receivedState && !oneShot)
      {
        LowlevelState dummyLowState; // fill if needed in future
        auto jointTraj = bodyController->getJointTrajectory(currentTwist, currentState, footForce, dummyLowState);

        pub_traj.publish(jointTraj);
        oneShot = true;
      }
    }
    else if (gait == ucf::BodyController::Gait::kPassive)
    {
      if (!oneShot)
      {
        auto jointTraj = bodyController->getEmptyTrajectory();
        pub_traj.publish(jointTraj);
        oneShot = true;
      }
    }
    else
    {
      LowlevelState lowState = jointStateToLowlevelState(currentState);
      auto jointTraj = bodyController->getJointTrajectory(currentTwist, currentState, footForce, lowState);

      pub_traj.publish(jointTraj);
    }
    ros::spinOnce();
    rate.sleep();
  }
}