#ifndef TURTLEBOT3_DETECTOR_H__
#define TURTLEBOT3_DETECTOR_H__

#include <turtlebot3_ball_tracking_exercise/ball_model.h>
#include <turtlebot3_ball_tracking_exercise/kalman_filter.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <image_geometry/pinhole_camera_model.h>
#include <geometry_msgs/PointStamped.h>



namespace turtlebot3
{

class Detector
{
public:
  // simple structure to model detected balls in the image plane
  struct  Ball2d{
    cv::Point2f pos = cv::Point2f(0,0);
    double radius = 0;
  };
  // simple structure to model image percepts
  struct BallPercept
  {
    ros::Time stamp;
    cv::Point3d pos;
  };

  // typedefs
  typedef boost::shared_ptr<Detector> Ptr;
  typedef boost::shared_ptr<const Detector> ConstPtr;

  /**
   * @brief Constructor for Detector class
   * @param camera_info camera info of camera producing the input images
   * @param ball_diameter diameter of ball to be detected
   * @param max_age maximal age of unmatched ball models
   */
  Detector(const sensor_msgs::CameraInfo& camera_info, double ball_diameter, double max_age, std::string tracking_frame, std::string optical_frame);

  /**
   * @brief Main update loop for ball detector. This method processes the input image
   * and updates the internal models using a Kalman filter.
   * @param dt time since last call
   * @param img input image to be processed
   * @param zero_velocity_model true if the velocity of a ball is assumed to be zero
   */
  void update(const cv::Mat& img, const ros::Time& stamp, double dt, bool zero_velocity_model);

  /**
   * @brief Retrieves all ball models
   * @return list of pointers of ball models
   */
  const std::vector<BallModel::Ptr>& getBallModels() const { return this->ball_models_; }

  /**
   * @brief Retrieves the latest detected balls in image coordinates
   * @return list of all detected balls
   */
  const std::vector<Ball2d>& getBalls2d() const {return this->latest_balls_2d_;}

  /**
   * @brief Returns last generated binary image used for ball detection
   * @return binary image produced by ball detector
   */
  const cv::Mat& getBinaryImage() const { return this->binary_img_; }

  /**
   * @brief Sets color thresholds in HSV space in order to detect areas
   * of interest.
   * @param min_hue min hue value [0..180]; just divide real hue by 2
   * @param max_hue max hue value [0..180]; just divide real hue by 2
   * @param min_sat min saturation [0..255]
   * @param max_sat max saturation [0..255]
   * @param min_val min value (intensity) [0..255]
   * @param max_val max value (intensity) [0..255]
   */
  void setHSVValues(int min_hue, int max_hue, int min_sat, int max_sat, int min_val, int max_val)
  {
    this->min_hue_ = min_hue;
    this->max_hue_ = max_hue;
    this->min_sat_ = min_sat;
    this->max_sat_ = max_sat;
    this->min_val_ = min_val;
    this->max_val_ = max_val;
  }

protected:
  /**
   * @brief Detects balls in pixel coordinates.
   * 
   * @param img input color image
   * @return std::vector<Ball2d> list of ball percepts in pixel coordinates
   */
  std::vector<Ball2d> detectBalls2d(const cv::Mat& img);
  
  /**
   * @brief converts balls from pixel coordinates into 3d coordinates using the camera link frame as coordinate system.
   * 
   * @param balls_2d the input detections
   * @param stamp time as the image was taken
   * @return std::vector<BallPercept>  list of ball percepts given in camera link frame
   */
  std::vector<BallPercept> convert2dTo3d(const std::vector<Ball2d>& balls_2d, const ros::Time& stamp);

  /**
   * @brief Detects balls from a given color image. Combines @ref detectBalls2d and @ref convert2dTo3d.
   * @param img input color image
   * @param stamp time stamp when image was taken
   * @return list of ball percepts given in camera link frame
   */
  std::vector<BallPercept> detectBalls(const cv::Mat& img, const ros::Time& stamp);

  /**
   * @brief Triggers the prediction step of the Kalman filter.
   * @param zero_velocity_model true if the velocity of a ball is assumed to be zero
   */
  void predictionStep(double dt, bool zero_velocity_model);

  /**
   * @brief Performs correction step of all models using Kalman filtering
   * approach. Hereby all given percepts will be matched to existing ball
   * models. All unmatchable percepts will result in ball models.
   * @param ball_percepts list of ball percepts
   * @param stamp time stamp when image was taken (= time of detection)
   */
  void correctionStep(const std::vector<BallPercept>& ball_percepts, const ros::Time& stamp);

  /**
   * @brief Projects ball image coordinate to 3d space using a pinhole model
   * @param p 2d image coordinate of ball
   * @param diameter diameter [px] of detected ball in image
   * @return projected 3d position of ball
   */
  cv::Point3d projectBallPerceptTo3d(const cv::Point2d& p, double diameter) const;

  /**
   * @brief Percept matching method in order to find closest ball percept within
   * the error confidence ellipse of a given ball model.
   * @param ball_percepts list of ball percept
   * @param ball_model ball model for which the best percept canditate should be matched
   * @return index of matched percept based on given input list, if no match was found -1.
   */
  int getClosestBallPercept(const std::vector<BallPercept>& ball_percepts, BallModel::ConstPtr ball_model);

  // used pinhole camera model
  image_geometry::PinholeCameraModel pinhole_camera_;

  // tf
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // stored ball models and corresponding Kalman filters
  std::vector<BallModel::Ptr> ball_models_;
  std::map<BallModel::Ptr, KalmanFilter> kfilters_;

  //latest 2d detections of balls in image
  std::vector<Ball2d> latest_balls_2d_;

  // black white detection matrix
  cv::Mat binary_img_;

  // internal counter used for assigning unique ids for each ball model
  int ball_uid_counter_;

  // parameters
  int min_hue_;
  int max_hue_;
  int min_sat_;
  int max_sat_;
  int min_val_;
  int max_val_;

  double ball_diameter_;

  double max_age_;
  std::string tracking_frame_;
  std::string optical_frame_;
};

}
#endif // FRAME_H
