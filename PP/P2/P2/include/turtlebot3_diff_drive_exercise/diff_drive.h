#ifndef TURTLEBOT3_DIFF_DRIVE_H__
#define TURTLEBOT3_DIFF_DRIVE_H__

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>



namespace turtlebot3
{
class DiffDrive
{
public:
  explicit DiffDrive(ros::NodeHandle& nh);
  
  /**
   * @brief Computes wheel velocities of a differential drive in order to follow given twist command.
   * This computation respects the maximum wheel velocities.
   * @param twist_msg Twist command to execute
   * @param wheel_l_vel [return] Output velocity for the left wheel
   * @param wheel_r_vel [return] Output velocity for the right wheel
   */
  void computeWheelVelocities(const geometry_msgs::Twist& twist_msg, double& wheel_l_vel, double& wheel_r_vel) const;

protected:
  double wheel_radius_;
  double wheel_separation_;
  double max_wheel_vel_;

};
}

#endif
