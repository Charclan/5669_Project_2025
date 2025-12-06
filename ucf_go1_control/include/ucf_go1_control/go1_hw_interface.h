#ifndef GO1_CONTROL__GO1_HW_INTERFACE_H
#define GO1_CONTROL__GO1_HW_INTERFACE_H

#include <ros_control_boilerplate/generic_hw_interface.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <geometry_msgs/WrenchStamped.h>
#include <array>
#define PMSM      (0x0A)
#define BRAKE     (0x00)
using namespace UNITREE_LEGGED_SDK;

namespace go1_control
{
/// \brief Hardware interface for a robot
class Go1HWInterface : public ros_control_boilerplate::GenericHWInterface
{
public:
  /**
   * \brief Constructor
   * \param nh - Node handle for topics.
   */
  Go1HWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model = NULL);

  /** \brief Read the state from the robot hardware. */
  virtual void read(ros::Duration& elapsed_time);

  /** \brief Write the command to the robot hardware. */
  virtual void write(ros::Duration& elapsed_time);

  /** \brief Enforce limits for all values before writing */
  virtual void enforceLimits(ros::Duration& period);

private:
  // 0 for "simulation", 1 for real robot
  int control_mode_ = 0;
  int power_protect_ = 1;
  Safety safe_;
  UDP udp_;
  LowCmd cmd_ = {0};
  LowState state_ = {0};
  std::vector<double> kp_;
  std::vector<double> kd_;

  // Foot force publishers (publish the same topics sim uses)
  std::array<ros::Publisher, 4> foot_force_pubs_;

  // Parameters to control publishing & scaling
  bool use_footForceEst_ = false;   // if true use state_.footForceEst (reserve, often zero)
  double foot_force_scale_ = 1.0;   // multiply raw sensor value -> output
  double foot_force_offset_ = 0.0;  // add offset after scaling
  double foot_force_sign_ = 1.0;    // flip sign if needed (+1 or -1)

};  // class

}  // namespace go1_control

#endif
