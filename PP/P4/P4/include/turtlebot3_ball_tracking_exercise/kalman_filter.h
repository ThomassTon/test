#ifndef TURTLEBOT3_KALMAN_FILTER_H__
#define TURTLEBOT3_KALMAN_FILTER_H__

#include <ros/ros.h>

#include <opencv2/video/video.hpp>

#include <turtlebot3_ball_tracking_exercise/ball_model.h>



namespace turtlebot3
{
class KalmanFilter
{
public:
  /**
   * @brief Constructor of KalmanFilter class
   * @param ball_model associated ball model which state is estimated using a Kalman filter
   */
  KalmanFilter(BallModel::Ptr ball_model);

  /**
   * @brief Empty constructor of KalmanFilter class
   */
  KalmanFilter();
  
  /**
   * @brief Destructor
   */
  virtual ~KalmanFilter();

  /**
   * @brief Resets state to current measurement
   */
  void resetState();

  /**
   * @brief Updates the Measurement state[x, y, z, vx, vy, vz] vector with new measurement values
   * @param position position of detected position of ball [m]
   * @param stamp time stamp when detection was done
   */
  void updateMeasurement(const cv::Point3d& position, const ros::Time& stamp);

  /**
   * @brief Performs time update on Kalman filter after transition matrix
   * (and if implemented processNoiseCov) update and updates ball model
   * @param dt time since last call
   * @param zero_velocity_model true if the velocity of a ball is assumed to be zero
   */
  void timeUpdateStep(double dt, bool zero_velocity_model = false);

  /**
   * @brief Performs measurement update on Kalman filter and update ball model
   */
  void correctionStep();

  // Getter and setter
  cv::Point3f getPredictedPosition() const { return cv::Point3f(state_.at<float>(0), state_.at<float>(1), state_.at<float>(2)); }
  cv::Point3f getPredictedVelocity() const { return cv::Point3f(state_.at<float>(3), state_.at<float>(4), state_.at<float>(5)); }

  cv::Mat getErrorCovPre() const { return kf_.errorCovPre; }
  cv::Mat getErrorCovPost() const { return kf_.errorCovPost; }
  void setA(float dt);
protected:
  /**
   * @brief Initializes member variables and OpenCV Kalman filter attributes
   * @param state_size size of state vector
   * @param measurement_size size of measurement vector
   */
  void initKalman(int state_size = 6, int measurement_size = 3);


  // Protected member variables
  cv::KalmanFilter kf_;

  BallModel::Ptr ball_model_;

  cv::Mat state_;
  cv::Mat measurement_;
};
}
#endif
