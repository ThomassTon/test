#include <turtlebot3_diff_drive_exercise/diff_drive.h>



namespace turtlebot3
{
DiffDrive::DiffDrive(ros::NodeHandle& nh)
{
  // get parameters
  wheel_radius_ = nh.param("diff_drive/wheel_radius", 0.033);
  wheel_separation_ = nh.param("diff_drive/wheel_separation", 0.160);
  max_wheel_vel_ = nh.param("diff_drive/max_wheel_vel", 1.0);
}

void DiffDrive::computeWheelVelocities(const geometry_msgs::Twist& twist_msg, double& wheel_l_vel, double& wheel_r_vel) const
{
  /// | Implement wheel velocity computation here |
  /// v                                           v
  double omega= twist_msg.angular.z;
  double V=twist_msg.linear.x;
  if(omega==0)
  {
      wheel_l_vel=wheel_r_vel=V/wheel_radius_;
  }
  else
  {
      double R=V/omega;
      wheel_l_vel=omega*(R-wheel_separation_/2)/wheel_radius_;
      wheel_r_vel=omega*(R+wheel_separation_/2)/wheel_radius_;
  }

  if(wheel_l_vel>max_wheel_vel_)
  {
      wheel_l_vel=max_wheel_vel_;
  }
  else if(wheel_l_vel<-max_wheel_vel_)
  {
      wheel_l_vel=-max_wheel_vel_;
  }

  if(wheel_r_vel>max_wheel_vel_)
  {
      wheel_r_vel=max_wheel_vel_;
  }
  else if(wheel_r_vel<-max_wheel_vel_)
  {
      wheel_r_vel=-max_wheel_vel_;
  }
  /// ^                                                   ^
  /// | ------ Wheel velocity computation end ----------- |
}
} // namespace
