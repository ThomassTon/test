#include <turtlebot3_diff_drive_exercise/diff_drive_joy_node.h>

#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>



namespace turtlebot3
{
DiffDriveJoyNode::DiffDriveJoyNode(ros::NodeHandle& nh)
  : enabled_(false)
{
  // get parameters
  max_linear_vel_ = nh.param("diff_drive/max_linear_vel", 1.0);
  max_angular_vel_ = nh.param("diff_drive/max_angular_vel", 1.0);

  linear_joy_axis_ = nh.param("diff_drive/linear_joy_axis", 1);
  if (nh.param("diff_drive/invert_linear_joy_axis", false))
      linear_joy_axis_ = -linear_joy_axis_;

  angular_joy_axis_ = nh.param("diff_drive/angular_joy_axis", 0);
  if (nh.param("diff_drive/invert_angular_joy_axis", false))
      angular_joy_axis_ = -angular_joy_axis_;

  enable_button_ = nh.param("diff_drive/enable_button", 7);
  disable_button_ = nh.param("diff_drive/disable_button", 6);

  // init subscriber
  cmd_vel_sub_ = nh.subscribe("joy", 1, &DiffDriveJoyNode::joyCb, this);

  // init publisher
  cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
}

DiffDriveJoyNode::~DiffDriveJoyNode()
{
}

void DiffDriveJoyNode::joyCb(const sensor_msgs::Joy::ConstPtr joy_msg)
{
  // check if (de-)activation button is pressed
  if (joy_msg->buttons[enable_button_])
    enabled_ = true;
  if (joy_msg->buttons[disable_button_])
  {
    if (enabled_)
      cmd_vel_pub_.publish(geometry_msgs::Twist());
    enabled_ = false;
  }

  if (!enabled_)
    return;

  geometry_msgs::Twist cmd_msg;

  if (joy_msg->axes.size() < std::max(linear_joy_axis_, angular_joy_axis_) + 1)
  {
    ROS_ERROR_ONCE("[DiffDriveJoy] It appears that wrong controller was connected or joystick axis assignment is wrong!");
    return;
  }

  cmd_msg.linear.x = joy_msg->axes[linear_joy_axis_] * max_linear_vel_;
  cmd_msg.angular.z = joy_msg->axes[angular_joy_axis_] * max_angular_vel_;

  // publish command
  cmd_vel_pub_.publish(cmd_msg);
}
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "turtlebot3_diff_drive_joy");

  ros::NodeHandle nh;
  turtlebot3::DiffDriveJoyNode node(nh);
  ros::spin();

  return 0;
}
