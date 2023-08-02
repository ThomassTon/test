#ifndef TURTLEBOT3_BALL_MODEL_H__
#define TURTLEBOT3_BALL_MODEL_H__

#include <ros/ros.h>

#include <sstream>
#include <iostream>
#include <time.h>
#include <cmath>

// OpenCV includes
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/video/video.hpp> //for kalman filter
#include <opencv2/highgui/highgui.hpp>

#include <turtlebot3_ball_tracking_exercise/ball.h>

// macro to convert a string to a double
#define SSTR(x) static_cast<std::ostringstream&>(std::ostringstream() << std::dec << x).str()

//macro to round a value to 2 digits
#define ROUND2(x) std::round(x * 100) / 100



namespace turtlebot3
{
class BallModel
  : public Ball
{
public:
  // typedefs
  typedef boost::shared_ptr<BallModel> Ptr;
  typedef boost::shared_ptr<const BallModel> ConstPtr;

  BallModel() = default;

  /* setters and getters */

  void setMatched(bool is_matched) { this->is_matched_ = is_matched; }
  bool isMatched() const { return this->is_matched_; }

  void setLastDetectedPosition(const cv::Point3d& detected_pose) { this->detected_pose_ = detected_pose; }
  const cv::Point3d& getLastDetectedPosition() const { return this->detected_pose_; }

  cv::Point3f getPredictedPosition() const { return cv::Point3f(getPosition().x, getPosition().y, getPosition().z); }
  cv::Point3f getPredictedVelocity() const { return cv::Point3f(getVelocity().x, getVelocity().y, getVelocity().z); }

  void setErrorEllipse(const cv::RotatedRect& error_ellipse) { this->error_ellipse_ = error_ellipse; }
  const cv::RotatedRect& getErrorEllipse() const { return this->error_ellipse_; }

protected:
  bool is_matched_ = false;
  cv::Point3d detected_pose_;
  cv::RotatedRect error_ellipse_;
};
}
#endif
