#include <turtlebot3_ball_tracking_exercise/detector.h>

#include <turtlebot3_ball_tracking_exercise/helper.h>



namespace turtlebot3
{
Detector::Detector(const sensor_msgs::CameraInfo& camera_info, double ball_diameter, double max_age, std::string tracking_frame, std::string optical_frame)
  : tf_listener_(tf_buffer_),
    ball_uid_counter_(0),
    ball_diameter_(ball_diameter),
    max_age_(max_age),
    tracking_frame_(tracking_frame),
    optical_frame_(optical_frame)

{
  pinhole_camera_.fromCameraInfo(camera_info);
}
cv::Point averg(std::vector<cv::Point> v)
{
    double xs=0;
    double ys=0;
    for(int i=0;i<v.size();i++)
    {
        xs+=v[i].x;
        ys+=v[i].y;
    }
    return cv::Point(xs/v.size(),ys/v.size());

}
double rad(std::vector<cv::Point> v,cv::Point pos)
{
    double sum=0;
    for(int i=0;i<v.size();i++)
    {
        double disx = v[i].x-pos.x;
        double disy = v[i].y-pos.y;
        sum +=sqrt(disx*disx+disy*disy);
    }
    return sum/v.size();

}
void Detector::update(const cv::Mat& img, const ros::Time& stamp, double dt, bool zero_velocity_model)
{
  // detect balls in image
  std::vector<BallPercept> percepts = detectBalls(img, stamp);

  // Kalman prediction step
  predictionStep(dt, zero_velocity_model);

  // Kalman correction step
  correctionStep(percepts, stamp);
}

std::vector<Detector::Ball2d> Detector::detectBalls2d(const cv::Mat& img){
  /// | Implement your code here |
  /// v                          v
  std::vector<Ball2d> percepts;
  binary_img_ = cv::Mat::zeros(img.size(), CV_8UC1); /// set your binary image here in order to display at debug visualization
  cv::Mat out;
  cv::GaussianBlur(img,out,cv::Size(7,7),3,3);
  cv::cvtColor(out,out,CV_BGR2HSV);
  cv::inRange(out, cv::Scalar(min_hue_, min_sat_, min_val_), cv::Scalar(max_hue_, max_sat_, max_val_),out);
  cv::Mat kernel = getStructuringElement(cv::MORPH_RECT, cv::Size(25,25));
  cv::morphologyEx(out, out, cv::MORPH_DILATE, kernel);
  cv::morphologyEx(out, out, cv::MORPH_ERODE, kernel);
  binary_img_=out;
  std::vector<cv::Vec3f> hough;
  cv::HoughCircles(binary_img_, hough, CV_HOUGH_GRADIENT, 1, 10, 100, 10, 10, 500);
  std::vector<std::vector<cv::Point> > contours;
  std::vector<double> surface;
  findContours(binary_img_, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
  for(int i = 0; i< contours.size();i++)
    {
        surface.push_back(contourArea(contours[i]));
    }
  for(int i = 0;i<contours.size();i++)
    {
        //double radius =0;
        double min=10;
        double max = -1;
        for(int j=0;j<hough.size();j++)
        {
            Ball2d ball;
            ball.pos = cv::Point2f(hough[j][0],hough[j][1]);
            ball.radius = hough[j][2];

            if(pointPolygonTest(contours[i],ball.pos,false)>0)
            {
                double s = ball.radius*ball.radius*M_PI;
                s = s/surface[i];
                if((s<1.5)&&(s>0.6))
                {
                    double d= pointPolygonTest(contours[i],ball.pos,true);
                    if(d/ball.radius>0.5)
                    {
                       if(abs(s-1)<min)
                       {
                           min=abs(s-1);
                           max=j;
                           //radius = d;
                       }
                    }
                }
            }
        }
        if(max>-1)
        {
            Ball2d ball2d;
  //          ball2d.pos = cv::Point2f(hough[max][0],hough[max][1]);
            ball2d.pos=averg(contours[i]);
  //          ball2d.radius = hough[max][2];
            ball2d.radius = rad(contours[i],ball2d.pos);
            percepts.push_back(ball2d);
        }

    }
  // data structure to store all detections from image

  return percepts;
}

std::vector<Detector::BallPercept> Detector::convert2dTo3d(const std::vector<Detector::Ball2d>& balls_2d, const ros::Time& stamp){
  /// | Implement your code here |
  /// v                          v
  std::vector<Detector::BallPercept> percepts;
    for (int i=0;i<balls_2d.size();i++)
    {
        BallPercept ball;
        ball.pos=projectBallPerceptTo3d(balls_2d[i].pos,balls_2d[i].radius*2);
        ball.stamp=stamp;
        percepts.push_back(ball);
    }
  return percepts;
}

std::vector<Detector::BallPercept> Detector::detectBalls(const cv::Mat& img, const ros::Time& stamp)
{
  latest_balls_2d_ = detectBalls2d(img);
  return convert2dTo3d(latest_balls_2d_, stamp);
}

void Detector::predictionStep(double dt, bool zero_velocity_model)
{
  for (BallModel::Ptr ball_model : ball_models_)
  {
    KalmanFilter& kf = kfilters_[ball_model];

    kf.timeUpdateStep(dt, zero_velocity_model);

    // update error ellipse
    cv::Mat cov = kf.getErrorCovPost();
    cv::Mat cov2d = cov(cv::Range(0, 2), cv::Range(0, 2)); // use only sub covariance of position state
    ball_model->setErrorEllipse(get2DErrorEllipse(cv::Point2f(ball_model->getPredictedPosition().x, ball_model->getPredictedPosition().y), cov2d, 5.991));
    ball_model->setCovariance(cov.at<float>(0), cov.at<float>(7), cov.at<float>(14), 0.0, 0.0, ball_model->getErrorEllipse().angle); /// @TODO: Works only for axis aligned covariance
    ball_model->setConfidence(1.0); /// @TODO: Compute feasible confidence
  }
}

void Detector::correctionStep(const std::vector<BallPercept>& ball_percepts, const ros::Time& stamp)
{
  std::vector<BallPercept> percepts = ball_percepts;

  // checked if matched ball
  for (BallModel::Ptr ball_model : ball_models_)
  {
    ball_model->setMatched(false);

    int idx = getClosestBallPercept(percepts, ball_model);

    if (idx != -1) // percept canditate found
    {
      // get corresponding kalman filter
      KalmanFilter& kf = kfilters_[ball_model];

      // perform correction step      

      kf.updateMeasurement(percepts[idx].pos, percepts[idx].stamp);
      kf.correctionStep();

      // mark ball as matched
      ball_model->setMatched(true);

      // remove percept from list
      percepts.erase(percepts.begin() + idx);
    }
  }

  // for each unmatched percept just create a new ball model
  for (const BallPercept& percept : percepts)
  {
    BallModel::Ptr ball_model(new BallModel());
    ball_model->setUId(ball_uid_counter_++);
    ball_model->setMatched(true);
    ball_model->setPosition(percept.pos.x, percept.pos.y, percept.pos.z);
    ball_model->setDiameter(ball_diameter_);
    ball_model->setLastObservationTimestamp(percept.stamp);
    ball_model->setLastDetectedPosition(percept.pos);
    ball_models_.push_back(ball_model);

    kfilters_[ball_model] = KalmanFilter(ball_model);
  }

  // removed ball models which haven't been matched in a specific time
  for (std::vector<BallModel::Ptr>::iterator itr = ball_models_.begin(); itr != ball_models_.end();)
  {
    BallModel::Ptr ball = *itr;

    if (!ball->isMatched() && (stamp - ball->getLastObservationTimestamp()).toSec() > max_age_)
      itr = ball_models_.erase(itr);
    else
      itr++;
  }
}

cv::Point3d Detector::projectBallPerceptTo3d(const cv::Point2d& p, double diameter) const
{
  double distance_x = (ball_diameter_ * pinhole_camera_.fx()) / diameter; // [m]
  double distance_y = (ball_diameter_ * pinhole_camera_.fy()) / diameter; // [m]

  cv::Point3d point = pinhole_camera_.projectPixelTo3dRay(p);
  point = point * 0.5 * (distance_x + distance_y);

  // TODO: Change function parameter to point_stamped instead of constructing here
  // OR: add header parameter
  geometry_msgs::PointStamped point_stamped;
  point_stamped.point.x = point.x;
  point_stamped.point.y = point.y;
  point_stamped.point.z = point.z;
  point_stamped.header.frame_id = optical_frame_;
  point_stamped.header.stamp = ros::Time::now();

  geometry_msgs::PointStamped point_world;
  try
  {
    tf_buffer_.transform(point_stamped, point_world, tracking_frame_, ros::Duration(1));
  }
  catch (tf2::TransformException &ex)
  {
    // TODO: Should not happen, handle somehow?
    ROS_WARN("%s",ex.what());
  }

  return cv::Point3d(point_world.point.x, point_world.point.y, point_world.point.z);
}

int Detector::getClosestBallPercept(const std::vector<BallPercept>& ball_percepts, BallModel::ConstPtr ball_model)
{
  int idx = -1;
  double min_dist_sq = std::numeric_limits<double>::max();

  // determine closest percept within the curren error ellipse of the given ball model
  for (size_t i = 0; i < ball_percepts.size(); i++)
  {
    cv::Point2f percept_position = cv::Point2f(ball_percepts[i].pos.x, ball_percepts[i].pos.y);
    cv::RotatedRect ellipse = ball_model->getErrorEllipse();

    cv::Size2f size(std::max(ellipse.size.width,  static_cast<float>(2.0*ball_diameter_)),
                    std::max(ellipse.size.height, static_cast<float>(2.0*ball_diameter_)));

    // check if current percept is within error ellipse
    if (pointInEllipse(percept_position, ellipse.center, size, ellipse.angle))
    {
      double dist_sq = getDistance2DSq(percept_position, ellipse.center);

      // check if current percept is closer than previous best candidate
      if (dist_sq < min_dist_sq)
      {
        min_dist_sq = dist_sq;
        idx = i;
      }
    }
  }

  return idx;
}
}
