#ifndef TURTLEBOT3_ARM_CONTROLLER_H__
#define TURTLEBOT3_ARM_CONTROLLER_H__

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>



namespace turtlebot3
{
class ArmController
{
public:
  explicit ArmController(ros::NodeHandle& nh);

  void update();
  void updateJoint(const std::string &joint_name, double delta_time,
                   double &effort_clamped, double &position_error);

protected:
  void jointStateCb(const sensor_msgs::JointState::ConstPtr& joint_state_msg);
  void goalPositionCb(const std::string& joint_name, const std_msgs::Float64::ConstPtr& goal_msg);

  ros::Time last_call_time_;

  std::map<std::string, double> current_position_;
  std::map<std::string, double> current_velocity_;

  std::map<std::string, double> goal_position_;

  std::map<std::string, double> p_;
  std::map<std::string, double> i_;
  std::map<std::string, double> d_;
  std::map<std::string, double> max_effort_;

  std::map<std::string, double> sum_error_;
  std::map<std::string, double> last_dpos_;
  std::map<std::string, double> aw_;
  std::vector<std::string> joint_names_;

  ros::Subscriber joint_state_sub_;
  std::map<std::string, ros::Subscriber> goal_position_sub_;
  std::map<std::string, ros::Publisher> joint_effort_pub_;
  std::map<std::string, ros::Publisher> joint_controller_state_pub_;
  std::map<std::string, double> last_error_;

};
}

#endif
