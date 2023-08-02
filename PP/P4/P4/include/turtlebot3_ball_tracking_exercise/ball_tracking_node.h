// Multiple Kalman Filter by Markus Lamprecht
// Code reference: https://github.com/Myzhar/simple-opencv-kalman-tracker/blob/master/source/opencv-kalman.cpp

#ifndef TURTLEBOT3_BALL_TRACKING_NODE_H__
#define TURTLEBOT3_BALL_TRACKING_NODE_H__

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

#include <dynamic_reconfigure/server.h>
#include <turtlebot3_ball_tracking_exercise/BallTrackingConfig.h>

#include <turtlebot3_exercise_msgs/BallDetections2d.h>
#include <turtlebot3_exercise_msgs/Ball2d.h>
#include <turtlebot3_exercise_msgs/BallState.h>
#include <turtlebot3_exercise_msgs/BallStates.h>
#include <turtlebot3_exercise_msgs/GetBallStatesAction.h>

#include <turtlebot3_ball_tracking_exercise/detector.h>



namespace turtlebot3
{
class BallTrackingNode
{
public:
  typedef actionlib::SimpleActionServer<turtlebot3_exercise_msgs::GetBallStatesAction> GetBallStatesActionServer;

  BallTrackingNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  virtual ~BallTrackingNode();

  void update();

protected:
  void publishBallStates(const std::vector<BallModel::Ptr>& balls) const;
  void publishDetectedBalls2d(const std::vector<Detector::Ball2d>& balls_2d, const ros::Time& stamp);
  // ROS API callbacks
  void imageCb(const sensor_msgs::ImageConstPtr& image);
  void dynamicReconfigureCb(const turtlebot3_ball_tracking_exercise::BallTrackingConfig& config, uint32_t level);

  void ballRequestActionGoalCb();

  // helper
  geometry_msgs::PointStamped getRobotPosition() const;
  std::vector<BallModel> getKNearestBalls(const geometry_msgs::PointStamped& point, int k) const;

  geometry_msgs::Point transformVelocity(const geometry_msgs::TransformStamped& transform, const geometry_msgs::Point& velocity) const;
//  void transformCov(const tf::StampedTransform& transform, const boost::array<double, 6>&cov_in, boost::array<double, 6>& cov_out) const;
  void transformBallState(const std::string& target_frame, const turtlebot3_exercise_msgs::BallState& state_in, turtlebot3_exercise_msgs::BallState& state_out) const;

  // class members
  Detector::Ptr detector_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  sensor_msgs::CameraInfo camera_info_;
  sensor_msgs::ImageConstPtr current_image_;
  sensor_msgs::ImageConstPtr last_processed_image_;

  ros::WallTime last_call_time_;

  // parameters
  std::string world_frame_;
  std::string tracking_frame_;
  std::string optical_frame_;
  std::string image_topic_;
  std::string camera_info_topic_;

  uint32_t img_width_;
  uint32_t img_height_;

  int frame_count_;
  double fps_;

  bool show_debug_img_;
  bool zero_velocity_model_;
  bool write_debug_img_;
  std::string absolute_img_path_;

  // subscriber
  image_transport::Subscriber img_sub_;

  // publisher
  image_transport::Publisher binary_img_pub_;
  image_transport::Publisher debug_img_pub_;
  ros::Publisher ball_states_pub_, ball_2d_pub_;

  // action server
  boost::shared_ptr<GetBallStatesActionServer> get_ball_states_as_;

  // dynamic reconfigure
  dynamic_reconfigure::Server<turtlebot3_ball_tracking_exercise::BallTrackingConfig> reconfigure_server_;

};
}

#endif
