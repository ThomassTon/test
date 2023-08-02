#ifndef TURTLEBOT3_DIFF_DRIVE_NODE_H__
#define TURTLEBOT3_DIFF_DRIVE_NODE_H__

#include <ros/ros.h>

#include <tf/transform_listener.h>

#include <actionlib/server/simple_action_server.h>

#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

#include <turtlebot3_exercise_msgs/MoveBaseAction.h>

#include <turtlebot3_diff_drive_exercise/move_base_marker.h>
#include <turtlebot3_diff_drive_exercise/odometry.h>
#include <turtlebot3_diff_drive_exercise/diff_drive.h>
#include <turtlebot3_diff_drive_exercise/diff_drive_control.h>



namespace turtlebot3
{
class DiffDriveNode
{
public:
  typedef actionlib::SimpleActionServer<turtlebot3_exercise_msgs::MoveBaseAction> MoveBaseActionServer;

  DiffDriveNode(ros::NodeHandle& nh);
  virtual ~DiffDriveNode();

  void update(const ros::Duration& time_diff);

protected:
  // helper
  bool getRobotPose(geometry_msgs::PoseStamped& pose) const;

  // UI callbacks
  void snapToCurrentPose(const visualization_msgs::InteractiveMarkerFeedbackConstPtr feedback);
  void moveBaseCmd(const visualization_msgs::InteractiveMarkerFeedbackConstPtr feedback);
  void stop(const visualization_msgs::InteractiveMarkerFeedbackConstPtr feedback);

  // ROS API callbacks
  void jointStateCb(const sensor_msgs::JointState::ConstPtr msg);
  void imuCb(const sensor_msgs::Imu::ConstPtr msg);
  void cmdVelCb(const ros::MessageEvent<geometry_msgs::Twist const>& event);
  void moveBaseCb(const geometry_msgs::PoseStamped& msg);
  void resetCb(const std_msgs::StringConstPtr msg);
  void moveBaseActionGoalCb();
  void moveBaseActionPreemptCb();

  void moveBase(const geometry_msgs::PoseStamped& goal);

  // class members
  MoveBaseMarker::Ptr move_base_marker_;
  Odometry odometry_;
  DiffDrive diff_drive_;
  DiffDriveControl diff_drive_control_;

  // parameters
  std::string nav_frame_;
  std::string base_frame_;

  bool auto_snap_to_current_pose_;

  tf::TransformListener tf_listener_;

  sensor_msgs::JointState::ConstPtr last_joint_state_msg_;
  sensor_msgs::Imu::ConstPtr last_imu_msg_;
  
  ros::Time last_odom_time_stamp_;

  // subscriber
  ros::Subscriber joint_state_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber cmd_vel_sub_;
  ros::Subscriber move_base_sub_;
  ros::Subscriber reset_sub_;

  // publisher
  ros::Publisher odom_pub_;
  ros::Publisher wheel_l_vel_cmd_pub_;
  ros::Publisher wheel_r_vel_cmd_pub_;
  ros::Publisher cmd_vel_pub_;

  // action server
  boost::shared_ptr<MoveBaseActionServer> move_base_as_;
};
}

#endif
