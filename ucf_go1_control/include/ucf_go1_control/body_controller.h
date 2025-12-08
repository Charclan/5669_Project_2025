#ifndef __BODY_CONTROLLER_H__
#define __BODY_CONTROLLER_H__

#include "geometry_msgs/Twist.h"
#include "nav_msgs/Path.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"
#include <array>

#include "common/unitreeRobot.h"

namespace ucf {
/// @brief Provides functionality for calculating joint trajectories from desired twist
/// and current states. Provides several different gaits as options to execute.
class BodyController {
public:
  enum class Gait { kPassive = 0, kStand, kWalk, kTrot, kCanter, kGallop };

  struct PositionVelocity {
    nav_msgs::Path swingProfile;
    std::vector<geometry_msgs::Twist> velocityProfile;
  };

  BodyController(QuadrupedRobot &robotModel, Gait gait, nav_msgs::Path swingProfile,
                 std::vector<geometry_msgs::Twist> velocityProfile = {}, double loop_rate = 10.0);

  /// @brief Calculate joint trajectory from current joint state and twist command
  /// @param twist Desired velocities
  /// @param jointState Current joint states
  /// @return Joint trajectories for 12 joints
  trajectory_msgs::JointTrajectory getJointTrajectory(const geometry_msgs::Twist &twist,
                                                      const sensor_msgs::JointState &jointState);

  /// @brief Generate a trajectory to move to standing position from current position
  ///        using linear interpolation
  /// @param targetPos Desired joint positions for standing
  /// @param jointState
  /// @return Joint trajectory
  trajectory_msgs::JointTrajectory getStandTrajectory(const std::vector<double> &targetPos, const sensor_msgs::JointState &jointState);

  /// @brief Add an empty trajectory message for "gait" 0 to clear commands
  /// @return minimal trajectory message
  trajectory_msgs::JointTrajectory getEmptyTrajectory();

protected:
  /// @brief Get timestamped foot positions for each leg
  /// @param twist Current velocity
  /// @param jointState Current joint state
  /// @return Array of 4 leg timestamped paths
  std::array<PositionVelocity, 4> getFoot(const geometry_msgs::Twist &twist, const sensor_msgs::JointState &jointState);

  // some gaits have faster swing phases vs contact phases. This scales them
  double scalePhase(double phase);

  /// @brief Determine which phase each foot is in based on current phase, desired gait, and commanded angle
  /// @param phase Current overall "baseline" phase
  /// @param gait Desired gait
  /// @param comAngle Commanded angle
  /// @return Phase of each foot
  std::array<double, 4> phaseStateMachine(double phase, BodyController::Gait gait, geometry_msgs::Vector3 comAngle);

private:
  QuadrupedRobot &robotModel_;
  Gait gait_;
  nav_msgs::Path swingProfile_;
  std::vector<geometry_msgs::Twist> velocityProfile_;
  double currentPhase_;
  int legId;
  std::array<double, 4> left_to_go_x;


  std::array<double, 4> footPhase_;

  double loop_rate_;
};
} // namespace ucf

#endif // __BODY_CONTROLLER_H__