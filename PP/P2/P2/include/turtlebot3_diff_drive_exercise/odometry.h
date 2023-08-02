#ifndef TURTLEBOT3_ODOMETRY_H__
#define TURTLEBOT3_ODOMETRY_H__

#include <ros/ros.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>



namespace turtlebot3
{
class Odometry
{
public:
  explicit Odometry(ros::NodeHandle& nh, size_t rolling_window_size = 1);

  /**
   * @brief Resets the current internal state estimation
   */
  void reset();

  /**
   * @brief Updates odometry based on given wheel joint and imu state
   * @param joint_state_msg Current joint states
   * @param imu_msg Current IMU state
   * @return Updated odometry message
   */
  const nav_msgs::Odometry& updateOdometry(const sensor_msgs::JointState::ConstPtr& joint_state_msg, const sensor_msgs::Imu::ConstPtr& imu_msg);

protected:

  // | ----- Put new member variables here ----- |
  // v                                           v
  double pos_x;
  double pos_y;
  double pos_theta;
  double radius;
  double omega;
  double Vr;
  double Vl;
  double V;

  // ^                                           ^
  // | ------ New member variables end --------- |

  sensor_msgs::JointState::ConstPtr last_joint_state_msg_;

  // odometry message which is kept updated
  nav_msgs::Odometry odom_msg_;

  // parameters
  double wheel_radius_;
  double wheel_separation_;
};
}

#endif
