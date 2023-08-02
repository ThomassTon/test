#include <turtlebot3_ball_tracking_exercise/kalman_filter.h>



namespace turtlebot3
{
KalmanFilter::KalmanFilter(BallModel::Ptr ball_model)
  : ball_model_(ball_model)
{
  initKalman();
  updateMeasurement(ball_model->getLastDetectedPosition(), ball_model->getLastObservationTimestamp());
  resetState();
}

KalmanFilter::KalmanFilter()
{

}

KalmanFilter::~KalmanFilter()
{
}
void KalmanFilter::setA(float dt)
{
    kf_.transitionMatrix = (cv::Mat_<float>(6, 6) <<
            1.0, 0.0, 0.0, dt, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0, dt, 0.0,
            0.0, 0.0, 1.0, 0.0, 0.0, dt,
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
}

void KalmanFilter::resetState()
{
  state_.at<float>(0) = measurement_.at<float>(0);
  state_.at<float>(1) = measurement_.at<float>(1);
  state_.at<float>(2) = measurement_.at<float>(2);
  state_.at<float>(3) = 0.0f;
  state_.at<float>(4) = 0.0f;
  state_.at<float>(5) = 0.0f;
  kf_.statePost = state_;
}

void KalmanFilter::updateMeasurement(const cv::Point3d& position, const ros::Time& stamp)
{
  /// | Implement your code here |
  /// v                          v
    measurement_.at<float>(0)=position.x;
    measurement_.at<float>(1)=position.y;
    measurement_.at<float>(2)=position.z;
  // fill measurement vector with detected position values

  /// ^                          ^
  /// | -------- End ----------- |

  ball_model_->setLastObservationTimestamp(stamp);
  ball_model_->setLastDetectedPosition(position);
}

void KalmanFilter::timeUpdateStep(double dt, bool zero_velocity_model)
{
  /// | Implement your code here |
  /// v                          v
  setA(dt);
  state_=kf_.predict();
  /// ^                          ^
  /// | -------- End ----------- |

  ball_model_->setTimestamp(ros::Time::now());
  ball_model_->setPosition(getPredictedPosition().x, getPredictedPosition().y, getPredictedPosition().z);
  ball_model_->setVelocity(getPredictedVelocity().x, getPredictedVelocity().y, getPredictedVelocity().z);
}

void KalmanFilter::correctionStep()
{
  /// | Implement your code here |
  /// v                          v
  state_=kf_.correct(measurement_);
  /// ^                          ^
  /// | -------- End ----------- |

  ball_model_->setTimestamp(ros::Time::now());
  ball_model_->setPosition(getPredictedPosition().x, getPredictedPosition().y, getPredictedPosition().z);
  ball_model_->setVelocity(getPredictedVelocity().x, getPredictedVelocity().y, getPredictedVelocity().z);
}

void KalmanFilter::initKalman(int state_size, int measurement_size)
{
  int type = CV_32F;

  kf_ = cv::KalmanFilter(state_size, measurement_size, 0, type);
  state_ = cv::Mat(state_size, 1, type); // [x, y, z, v_x, v_y, v_z]
  measurement_ = cv::Mat(measurement_size, 1, type); // [z_x, z_y, z_z]

  /// | Implement your code here |
  /// v                          v

  // Transition State Matrix A
  // Note: set dt at each processing step!
  // [ 1  0  0  dt   0   0]
  // [ 0  1  0   0  dt   0]
  // [ 0  0  1   0   0  dt]
  // [ 0  0  0   1   0   0]
  // [ 0  0  0   0   1   0]
  // [ 0  0  0   0   0   1]
  setA(0.1);
  // Measurement Matrix H
  // [ 1 0 0 0 0 0]
  // [ 0 1 0 0 0 0]
  // [ 0 0 1 0 0 0]
  kf_.measurementMatrix = (cv::Mat_<float>(3,6)  <<
               1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 1.0, 0.0, 0.0, 0.0);
  // Process Noise Covariance Matrix Q
  // [ Ex   0    0    0     0     0   ]
  // [ 0    Ey   0    0     0     0   ]
  // [ 0    0    Ez   0     0     0   ]
  // [ 0    0    0    Ev_y  0     0   ]
  // [ 0    0    0    0     Ev_y  0   ]
  // [ 0    0    0    0     0     Ev_y]
  kf_.processNoiseCov = (cv::Mat_<float>(6, 6) <<
            10e-5, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 10e-5, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 10e-5, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 10e-4, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 10e-4, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 10e-4);
  // Measures Noise Covariance Matrix R
  kf_.measurementNoiseCov = (cv::Mat_<float>(3,3)  <<
               0.001, 0.0, 0.0,
               0.0, 0.001, 0.0,
               0.0, 0.0, 0.001);
  /// ^                          ^
  /// | -------- End ----------- |

  // Initialize diagonal error covariance of time update with low confidence
  kf_.errorCovPre.at<float>(0)  = 0.02;
  kf_.errorCovPre.at<float>(7)  = 0.02;
  kf_.errorCovPre.at<float>(14) = 0.02;
  kf_.errorCovPre.at<float>(21) = 0.10;
  kf_.errorCovPre.at<float>(28) = 0.10;
  kf_.errorCovPre.at<float>(35) = 0.10;

  // Initialize diagonal error covariance of measurement update with low confidence
  kf_.errorCovPost.at<float>(0)  = 0.02;
  kf_.errorCovPost.at<float>(7)  = 0.02;
  kf_.errorCovPost.at<float>(14) = 0.02;
  kf_.errorCovPost.at<float>(21) = 0.10;
  kf_.errorCovPost.at<float>(28) = 0.10;
  kf_.errorCovPost.at<float>(35) = 0.10;
}
}
