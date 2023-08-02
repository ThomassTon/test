#ifndef TURTLEBOT3_HELPER_H__
#define TURTLEBOT3_HELPER_H__

#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/CameraInfo.h>

#include <turtlebot3_ball_tracking_exercise/ball_model.h>

#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>


namespace turtlebot3
{
template <typename T1, typename T2>
inline double getDistance2DSq(const T1& p1, const T2& p2)
{
  return (p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y);
}

template <typename T1, typename T2>
inline double getDistance2D(const T1& p1, const T2& p2)
{
  return sqrt(getDistance2DSq(p1, p2));
}

bool pointInEllipse(const cv::Point2f& point, const cv::Point2f& center, const cv::Size2f& size, float angle);

// chisq = 5.991 equals 95% confidence interval for 2 degree of freedoms (table see https://faculty.elgin.edu/dkernler/statistics/ch09/9-3.html)
cv::RotatedRect get2DErrorEllipse(const cv::Point2f& mean, const cv::Mat& cov, double chisq = 5.991);

double resizeKeepAspectRatio(const cv::Mat &in, cv::Mat &out, const cv::Size &dst_size, const cv::Scalar &bgcolor);

cv::Mat drawDetectionImg(cv::Mat& result, const cv::Mat& input_img, const sensor_msgs::CameraInfo& camera_info, const std::vector<BallModel::Ptr>& balls,
                         double rate, const geometry_msgs::TransformStamped& cam_transform);
cv::Mat drawResultImg(cv::Mat& result, const cv::Mat& detection_img, const cv::Mat& binary_img, const std::vector<BallModel::Ptr>& balls);

std::string printBallModel(const BallModel& model);

std::string printBallState(const Ball& ball);
}
#endif
