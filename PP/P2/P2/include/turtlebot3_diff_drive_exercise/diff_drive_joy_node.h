#ifndef TURTLEBOT3_DIFF_DRIVE_JOY_NODE_H__
#define TURTLEBOT3_DIFF_DRIVE_JOY_NODE_H__

#include <ros/ros.h>

#include <sensor_msgs/Joy.h>



namespace turtlebot3
{
class DiffDriveJoyNode
{
public:
  DiffDriveJoyNode(ros::NodeHandle& nh);
  virtual ~DiffDriveJoyNode();

protected:
  void joyCb(const sensor_msgs::Joy::ConstPtr joy_msg);

  // parameters
  double max_linear_vel_;
  double max_angular_vel_;

  int linear_joy_axis_;
  int angular_joy_axis_;
  int enable_button_;
  int disable_button_;

  bool enabled_;

  // subscriber
  ros::Subscriber cmd_vel_sub_;

  // publisher
  ros::Publisher cmd_vel_pub_;
};
}

#endif
