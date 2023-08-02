#ifndef TURTLEBOT3_DIFF_DRIVE_CONTROL_H__
#define TURTLEBOT3_DIFF_DRIVE_CONTROL_H__

#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>



namespace turtlebot3
{
class DiffDriveControl
{
public:
  explicit DiffDriveControl(ros::NodeHandle& nh);

  /**
   * @brief Resets controller to initial state
   */
  void reset();

  /**
   * @brief Sets new target goal and activates controller.
   * @param goal New target goal
   */
  void setGoalPose(const geometry_msgs::Pose& goal);

  /**
   * @brief Stops and resets the controller.
   */
  void stop();

  /**
   * @brief Retrieves if currently a target goal is set and the controller is active.
   * @return true, when controller is active
   */
  bool isActive() const;
  double compute_v(double v, double time_diff,double Vdes );
  double compute_omega(double theta, double om, double time_diff);
  double theta_corr(double theta);
  /**
   * @brief Computes appropriate twist message in order to reach the goal pose.
   * @param time_diff Time since last call
   * @param robot_pose Current robot pose
   * @param odom Current robot odometry
   * @return Twist message to execute
   */
  geometry_msgs::Twist computeTwist(const ros::Duration& time_diff, const geometry_msgs::Pose& robot_pose, const nav_msgs::Odometry& odom);

protected:
  /**
   * @brief Computes the shortest angle path from a1 to a2.
   * @param a1 First angle
   * @param a2 Second angle
   * @return Shortest angular path
   */
  static double getShortestAnglePath(double a1, double a2) ;

  // parameters
  double max_linear_vel_;  
  double max_linear_acc_;
  double max_angular_vel_;
  double max_angular_acc_;
  double pos_tolerance_;
  double angular_tolerance_;

  bool allow_backward_;

  double p_gain_;

  bool is_active_;
  bool first_heading_alignment_done_;
  bool goal_position_reached_;
  double last_erro_theta;
  double last_erro_theta2;
  bool turn_first;
  bool distance_null;
  bool backward;
  bool first;
  geometry_msgs::Pose goal_pose_;
};
}

#endif
