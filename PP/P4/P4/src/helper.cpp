#include <turtlebot3_ball_tracking_exercise/helper.h>

#include <image_geometry/pinhole_camera_model.h>

namespace turtlebot3
{
bool pointInEllipse(const cv::Point2f& point, const cv::Point2f& center, const cv::Size2f& size, float angle)
{
  float cosa = std::cos(angle);
  float sina = std::sin(angle);
  float height_sq = size.height * size.height;
  float width_sq = size.width * size.width;

  float a = std::pow(cosa * (point.x - center.x) + sina * (point.y - center.y), 2.0f);
  float b = std::pow(sina * (point.x - center.x) - cosa * (point.y - center.y), 2.0f);
  float ell = (a / height_sq) + (b / width_sq);

  return ell <= 1;
}

cv::RotatedRect get2DErrorEllipse(const cv::Point2f& mean, const cv::Mat& cov, double chisq)
{
  try
  {
    // get the eigenvalues and eigenvectors
    cv::Mat eigen_values, eigen_vectors;
    cv::eigen(cov, eigen_values, eigen_vectors);

    // calculate the angle between the largest eigenvector and the x-axis
    float angle = atan2(eigen_vectors.at<float>(0, 1), eigen_vectors.at<float>(0, 0));

    // shift the angle to the [0, 2pi] interval instead of [-pi, pi]
    if(angle < 0)
      angle += 2.0*M_PI;

    // convert to degrees instead of radians
    angle = 180.0 * angle/M_PI;

    // calculate the size of the minor and major axes
    float half_major_axis_size = chisq*sqrt(eigen_values.at<float>(0));
    float half_minor_axis_size = chisq*sqrt(eigen_values.at<float>(1));

    // return the oriented ellipse
    // the -angle is used because OpenCV defines the angle clockwise instead of anti-clockwise
    return cv::RotatedRect(mean, cv::Size2f(half_minor_axis_size, half_major_axis_size), -angle);
  }
  catch (const cv::Exception& e)
  {
    ROS_ERROR("Exception caught in get2DErrorEllipse:\n%s", e.what());
    return cv::RotatedRect(mean, cv::Size2f(0.0f, 0.0f), 0.0f);
  }
}

// resize input image keeping the aspect ratio
double resizeKeepAspectRatio(const cv::Mat& in, cv::Mat& out, const cv::Size& dst_size, const cv::Scalar& bgcolor)
{
  double scale_width = dst_size.width/(double)in.cols;
  double scale_height = dst_size.height/(double)in.rows;
  double img_scale = std::min(scale_width, scale_height);

  cv::resize(in, out, cv::Size(img_scale*in.cols, img_scale*in.rows));

  int top = std::floor((dst_size.height - out.rows) / 2);
  int down = std::floor((dst_size.height - out.rows+1) / 2);
  int left = std::floor((dst_size.width - out.cols) / 2);
  int right = std::floor((dst_size.width - out.cols+1) / 2);

  cv::copyMakeBorder(out, out, top, down, left, right, cv::BORDER_CONSTANT, bgcolor);

  return img_scale;
}

// draw debug image methods
cv::Mat drawDetectionImg(cv::Mat& result, const cv::Mat& input_img, const sensor_msgs::CameraInfo& camera_info, const std::vector<BallModel::Ptr>& balls,
                         double rate, const geometry_msgs::TransformStamped& cam_transform)
{
  result = input_img;

  image_geometry::PinholeCameraModel pinhole_camera;
  pinhole_camera.fromCameraInfo(camera_info);


  for (BallModel::ConstPtr ball : balls)
  {
    // Transform from world to camera frame
    geometry_msgs::Point ball_position;
    ball_position.x = ball->getLastDetectedPosition().x;
    ball_position.y = ball->getLastDetectedPosition().y;
    ball_position.z = ball->getLastDetectedPosition().z;
    tf2::doTransform(ball_position, ball_position, cam_transform);
    // back project detected ball position into image
    cv::Point2d pixel = pinhole_camera.project3dToPixel(cv::Point3d(ball_position.x, ball_position.y, ball_position.z));
    double r = (0.5*(pinhole_camera.fy()+pinhole_camera.fy()) / ball_position.z) * 0.5*ball->getDiameter();

    // Check if ball is inside of current image
    if (pixel.x >= 0 && pixel.x < camera_info.width && pixel.y >= 0 && pixel.y < camera_info.height) {
      if (r >= 0) {
        int radius_int = static_cast<int>(r);
        if (ball->isMatched())
          cv::circle(result, pixel, radius_int, CV_RGB(0, 255, 0), 2);
        else
          cv::circle(result, pixel, radius_int, CV_RGB(0, 100, 0), 2);
      }
    }
  }

  // draw legend above
  cv::putText(result, "detected ball", cv::Point(10, 15), cv::FONT_HERSHEY_SIMPLEX, 0.6, CV_RGB(0, 255, 0), 2);
  cv::putText(result, "last detected ball", cv::Point(150, 15), cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(0, 100, 0), 2);
  cv::putText(result, "estimated ball", cv::Point(350, 15), cv::FONT_HERSHEY_SIMPLEX, 0.6, CV_RGB(255, 0, 255), 2);
  cv::putText(result, "error ellipse", cv::Point(500, 15), cv::FONT_HERSHEY_SIMPLEX, 0.6, CV_RGB(255, 0, 0), 2);

  // draw legend below
  cv::putText(result, "fps: " + std::to_string(static_cast<int>(rate)), cv::Point(10, 70), cv::FONT_HERSHEY_SIMPLEX, 0.75, CV_RGB(255, 255, 0), 2);
  return result;
}


// draw debug image methods
cv::Mat drawResultImg(cv::Mat& result, const cv::Mat& detection_img, const cv::Mat& binary_img, const std::vector<BallModel::Ptr>& balls)
{
  result = cv::Mat::zeros(480, 960, CV_8UC3);

  // Resize image to hard coded (*puke*) size display on the left
  cv::Mat mat1 = cv::Mat::zeros(480, 640, CV_8UC3);
  resizeKeepAspectRatio(detection_img, mat1, cv::Size(640, 480), cv::Scalar(0, 0, 0));

  // Resize binary image to fit to top right
  cv::Mat mat2 = cv::Mat::zeros(240, 320, CV_8UC3);
  resizeKeepAspectRatio(binary_img, mat2, cv::Size(320, 240), cv::Scalar(0, 0, 0));
  cv::cvtColor(mat2, mat2, CV_GRAY2RGB);

  // bottom right img
  cv::Mat mat3 = cv::Mat::zeros(240, 320, CV_8UC3);

  // print coordinate system right pic=mat3
  cv::line(mat3, cv::Point(0, 240), cv::Point(320, 240), cv::Scalar(255, 150, 255), 5);
  cv::line(mat3, cv::Point(160, 40), cv::Point(160, 240), cv::Scalar(255, 150, 255), 2);

  int width = 320;
  int height = 240;
  cv::line(mat3, cv::Point(0, height), cv::Point(width, height), cv::Scalar(255, 150, 255), 5);         // horizontal y-Axis
  cv::line(mat3, cv::Point(width/2.0, 0), cv::Point(width/2.0, height), cv::Scalar(255, 150, 255), 2);  // vertical x-Axis

  double m_to_px_scale = 200.0; // 1m corresponds to 200 px;

  // draw ticks each 20 [px]
  int tick_sep = 20;
  int offset = 5;

  // draw vertical x-axis
  int tick_count = height / tick_sep;
  for (int i = 1; i <= tick_count; i++)
    cv::putText(mat3, SSTR((tick_sep * i)/m_to_px_scale), cv::Point(width/2 + offset, height - tick_sep * i), cv::FONT_HERSHEY_SIMPLEX, 0.2, CV_RGB(255, 150, 255), 0.8);

  // draw horizontal y-Axis
  tick_count = width / tick_sep;
  for (int i = 1; i <= tick_count; i++)
  {
    cv::putText(mat3, SSTR( (tick_sep * i)/m_to_px_scale), cv::Point(width/2 - tick_sep * i, height - offset), cv::FONT_HERSHEY_SIMPLEX, 0.18, CV_RGB(255, 150, 255), 0.8); // in cm x-Axis
    cv::putText(mat3, SSTR(-(tick_sep * i)/m_to_px_scale), cv::Point(width/2 + tick_sep * i, height - offset), cv::FONT_HERSHEY_SIMPLEX, 0.18, CV_RGB(255, 150, 255), 0.6); // in cm y-Axis
  }

  // draw detected balls
  for (int p = 0; p < balls.size(); p++)
  {
    const BallModel& model = *(balls[p]);
    double scale = m_to_px_scale;
    double ball_radius_px = 0.5*model.getDiameter()*scale;

    cv::putText(mat2, printBallModel(model), cv::Point(2, 10 + 13 * p), cv::FONT_HERSHEY_SIMPLEX, 0.3, CV_RGB(255, 255, 255), 1.0);

    // print state
    cv::putText(mat2, printBallState(model), cv::Point(50, 150 + 13 * p), cv::FONT_HERSHEY_SIMPLEX, 0.3, CV_RGB(255, 100, 255), 1.0);

    // green
    if (model.isMatched())
      cv::circle(mat3, cv::Point(width/2 - model.getLastDetectedPosition().y * scale, height - model.getLastDetectedPosition().x * scale), ball_radius_px, CV_RGB(100, 255, 0), -1, 8, 0);
    else
      cv::circle(mat3, cv::Point(width/2 - model.getLastDetectedPosition().y * scale, height - model.getLastDetectedPosition().x * scale), ball_radius_px, CV_RGB(0, 100, 0), -1, 8, 0); // dark green

    // pink
    cv::circle(mat3, cv::Point(width/2 - model.getPredictedPosition().y * scale, height - model.getPredictedPosition().x * scale), ball_radius_px, CV_RGB(255, 0, 255), 1, 8, 0);

    // red
    cv::RotatedRect rRect = model.getErrorEllipse();
    rRect.center.x = -model.getErrorEllipse().center.y * scale + width/2;
    rRect.center.y = -model.getErrorEllipse().center.x * scale + height;
    rRect.size.height = model.getErrorEllipse().size.width * scale;
    rRect.size.width = model.getErrorEllipse().size.height * scale;
    cv::ellipse(mat3, rRect, CV_RGB(255, 0, 0), 1, 7);

    // blue
    offset = 1.1*ball_radius_px;
    cv::putText(mat3, SSTR(model.getUId()), cv::Point(width/2 + offset - model.getPredictedPosition().y * scale, height - offset - model.getPredictedPosition().x * scale), cv::FONT_HERSHEY_SIMPLEX, 0.3, CV_RGB(100, 150, 255), 1);
  }

  cv::Mat mat23 = cv::Mat::zeros(480, 320, CV_8UC3);
  cv::vconcat(mat2, mat3, mat23);
  cv::hconcat(mat1, mat23, result);

  return result;
}

std::string printBallModel(const BallModel& model)
{
  return "" + SSTR(model.getUId()) + ""
      + "" + SSTR(model.isMatched());
      //+ "x: "+SSTR(ROUND2(result_ray.x*100))+" "+SSTR(ROUND2(result_ray.y*100))+" "+SSTR(ROUND2(result_ray.z*100));
      // + " |g:(" + SSTR(ROUND2(detected_ball_.x)) + "," + SSTR(ROUND2(detected_ball_.y)) + ")"
      // + " p:(" + SSTR(ROUND2(predicted_ball_.x)) + "," + SSTR(ROUND2(predicted_ball_.y)) + ")"
      // + " r:(" + SSTR(ROUND2(error_ellipse_.center.x)) + "," + SSTR(ROUND2(error_ellipse_.center.y)) + "," + SSTR(ROUND2(error_ellipse_.size.width * 100)) + "," + SSTR(ROUND2(error_ellipse_.size.height * 100)) + ")";
}

std::string printBallState(const Ball& ball)
{
  return "" + SSTR(ball.getUId()) + ":"
      + " x: " + SSTR(ROUND2(ball.getPosition().x))+ ","
      + " y: " + SSTR(ROUND2(ball.getPosition().y)) + ","
      + " z: " + SSTR(ROUND2(ball.getPosition().z)) + ""
      + " | vx: " + SSTR(ROUND2(ball.getVelocity().x)) + ","
      + " vy: " + SSTR(ROUND2(ball.getVelocity().y)) + ","
      + " vz: " + SSTR(ROUND2(ball.getVelocity().y));
}
}
