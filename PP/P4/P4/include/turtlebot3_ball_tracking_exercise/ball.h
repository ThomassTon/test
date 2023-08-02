#ifndef TURTLEBOT3_BALL_H__
#define TURTLEBOT3_BALL_H__

#include <ros/ros.h>

#include <turtlebot3_exercise_msgs/BallState.h>



namespace turtlebot3
{
class Ball
  : protected turtlebot3_exercise_msgs::BallState
{
public:
  // typedefs
  typedef boost::shared_ptr<Ball> Ptr;
  typedef boost::shared_ptr<const Ball> ConstPtr;

  Ball() = default;

  /**
   * @brief Returns this ball object as turtlebot3_exercise_msgs::BallState
   * @return representative turtlebot3_exercise_msgs::BallState
   */
  const turtlebot3_exercise_msgs::BallState& asMsgConst() const { return *this; }
  turtlebot3_exercise_msgs::BallState& asMsg() { return *this; }

  /* setters and getters */

  void setHeader(const std_msgs::Header& header) { this->header = header; }
  const std_msgs::Header& getHeader() const { return this->header; }

  void setTimestamp(const ros::Time& stamp) { this->header.stamp = stamp; }
  const ros::Time& getTimestamp() const { return this->header.stamp; }

  void setLastObservationTimestamp(const ros::Time& stamp) { this->last_observation_time_ = stamp; }
  const ros::Time& getLastObservationTimestamp() const { return this->last_observation_time_; }

  void setUId(int uid) { this->uid = uid; }
  int getUId() const { return this->uid; }

  void setPosition(double x, double y, double z) { this->position.x = x; this->position.y = y; this->position.z = z;}
  void setPosition(const geometry_msgs::Point& position) { this->position = position; }
  const geometry_msgs::Point& getPosition() const { return this->position; }

  void setVelocity(double x, double y, double z) { this->velocity.x = x; this->velocity.y = y; this->velocity.z = z;}
  void setVelocity(const geometry_msgs::Point& velocity) { this->velocity = velocity; }
  const geometry_msgs::Point& getVelocity() const { return this->velocity; }

  void setDiameter(double diameter) { this->diameter = diameter; }
  double getDiameter() const { return this->diameter; }

  /**
   * Row-major representation of the 3x3 covariance matrix
   * The orientation parameters use a fixed-axis representation.
   * In order, the parameters are:
   * (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
   */
  void setCovariance(double x, double y, double z, double angle_x, double angle_y, double angle_z)
  {
    covariance[0] = x;
    covariance[1] = y;
    covariance[2] = z;
    covariance[3] = angle_x;
    covariance[4] = angle_y;
    covariance[5] = angle_z;
  }

  void getCovariance(double& x, double& y, double& z, double& angle_x, double& angle_y, double& angle_z) const
  {
    x = covariance[0];
    y = covariance[1];
    z = covariance[2];
    angle_x = covariance[3];
    angle_y = covariance[4];
    angle_z = covariance[5];
  }

  void setConfidence(double confidence) { this->confidence = confidence; }
  double getConfidence() const { return this->confidence; }

protected:
  ros::Time last_observation_time_;
};
}
#endif
