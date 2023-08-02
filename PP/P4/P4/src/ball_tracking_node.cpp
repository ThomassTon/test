#include <turtlebot3_ball_tracking_exercise/ball_tracking_node.h>

#include <turtlebot3_ball_tracking_exercise/helper.h>



namespace turtlebot3
{
BallTrackingNode::BallTrackingNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : tf_listener_(tf_buffer_),
    frame_count_(0),
    fps_(0.0),
    reconfigure_server_(ros::NodeHandle(pnh, "hsv_filter"))
{
  // get params
  world_frame_ = pnh.param("world_frame", std::string("world"));
  tracking_frame_ = pnh.param("tracking_frame", std::string("world"));
  optical_frame_ = pnh.param("optical_frame", std::string("camera_optical_frame"));
  image_topic_ = pnh.param("image_topic", std::string("sensor/camera/image_rect_color"));
  camera_info_topic_ = pnh.param("camera_info_topic", std::string("sensor/camera/camera_info"));

  img_width_ = static_cast<uint32_t>(pnh.param("width", 640));
  img_height_ = static_cast<uint32_t>(pnh.param("height", 480));

  show_debug_img_ = pnh.param("show_debug_img", false);
  zero_velocity_model_ = pnh.param("zero_velocity_model", false);
  write_debug_img_ = pnh.param("write_debug_img", false);
  absolute_img_path_ = pnh.param("img_output_path", std::string());

  // get camera info once (@TODO: Use ImageTransport to keep updated if camera info changes)
  camera_info_ = *(ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_info_topic_));

  // consider rescale
  double scale_x = static_cast<double>(img_width_) / static_cast<double>(camera_info_.width);
  double scale_y = static_cast<double>(img_height_) / static_cast<double>(camera_info_.height);
  double min_scale = std::min(scale_x, scale_y);

  camera_info_.K[0] = camera_info_.K[0] * min_scale;  // fx
  camera_info_.K[2] = camera_info_.K[2] * scale_x;    // cx
  camera_info_.K[4] = camera_info_.K[4] * min_scale;  // fy
  camera_info_.K[5] = camera_info_.K[5] * scale_y;    // cy

  camera_info_.P[0] = camera_info_.P[0] * min_scale;  // fx
  camera_info_.P[2] = camera_info_.P[2] * scale_x;    // cx
  camera_info_.P[3] = camera_info_.P[3] * scale_x;    // T
  camera_info_.P[5] = camera_info_.P[5] * min_scale;  // fy
  camera_info_.P[6] = camera_info_.P[6] * scale_y;    // cy

  camera_info_.width = img_width_;
  camera_info_.height = img_height_;

  // initialize detector
  detector_.reset(new Detector(camera_info_, nh.param("ball_diameter", 0.064), nh.param("max_age", 10.0), tracking_frame_, optical_frame_));

  // Dynamic reconfigure
  dynamic_reconfigure::Server<turtlebot3_ball_tracking_exercise::BallTrackingConfig>::CallbackType reconfigure_callback_fun;
  reconfigure_callback_fun = boost::bind(&BallTrackingNode::dynamicReconfigureCb, this, _1, _2);
  reconfigure_server_.setCallback(reconfigure_callback_fun);

  // subscriber
  image_transport::ImageTransport it(nh);
  img_sub_ = it.subscribe(image_topic_, 1, &BallTrackingNode::imageCb, this);

  // publisher
  binary_img_pub_ = it.advertise("ball_tracking/binary_img", 1);
  debug_img_pub_ = it.advertise("ball_tracking/debug_img", 1);
  ball_states_pub_ = nh.advertise<turtlebot3_exercise_msgs::BallStates>("ball_tracking/ball_states", 1);
  ball_2d_pub_ = nh.advertise<turtlebot3_exercise_msgs::BallDetections2d>("ball_tracking/detections_2d", 1);

  // init action servers
  get_ball_states_as_.reset(new GetBallStatesActionServer(pnh, "get_ball_states", false));
  get_ball_states_as_->registerGoalCallback(boost::bind(&BallTrackingNode::ballRequestActionGoalCb, this));
  get_ball_states_as_->start();
}

BallTrackingNode::~BallTrackingNode()
{
}

void BallTrackingNode::update()
{
  // time statistics
  ros::WallTime now = ros::WallTime::now();
  if (!last_call_time_.isZero())
    fps_ = 0.5 * (1.0/(now - last_call_time_).toSec()) + 0.5 * fps_;
  last_call_time_ = now;

  // process image if received
  if (current_image_)
  {
    if (last_processed_image_)
    {
      double dt = (current_image_->header.stamp - last_processed_image_->header.stamp).toSec();

      cv::Mat input_img_;

      // convert from the ROS image message into a CvImage
      try
      {
        // openCV expects color images to use BGR channel order.
        cv_bridge::CvImagePtr cv_ptr_ = cv_bridge::toCvCopy(current_image_, sensor_msgs::image_encodings::BGR8);
        resizeKeepAspectRatio(cv_ptr_->image, input_img_, cv::Size(img_width_, img_height_), cv::Scalar(0, 0, 0));
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("[BallTrackingNode] imageCb: cv_bridge exception: %s", e.what());
        return;
      }

      // only process if image have different times
      if (dt > 0.0)
      {
        detector_->update(input_img_, current_image_->header.stamp, dt, zero_velocity_model_);
        publishDetectedBalls2d(detector_->getBalls2d(), current_image_->header.stamp);
        frame_count_++;
      }

      // display the image for debugging using OpenCV
      if (!detector_->getBinaryImage().empty())
      {
        geometry_msgs::TransformStamped transform;
        try
        {
          transform = tf_buffer_.lookupTransform(optical_frame_, tracking_frame_, current_image_->header.stamp, ros::Duration(1));
        }
        catch (tf2::TransformException ex)
        {
          ROS_ERROR("[BallTrackingNode] update: %s", ex.what());
        }
        cv::Mat detection_img;
        drawDetectionImg(detection_img, input_img_, camera_info_, detector_->getBallModels(), fps_, transform);
        cv::Mat debug_img;
        drawResultImg(debug_img, detection_img, detector_->getBinaryImage(), detector_->getBallModels());

        // publish images
        if (binary_img_pub_.getNumSubscribers() > 0)
          binary_img_pub_.publish(cv_bridge::CvImage(std_msgs::Header(), "mono8", detector_->getBinaryImage()).toImageMsg());
        if (debug_img_pub_.getNumSubscribers() > 0)
          debug_img_pub_.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", detection_img).toImageMsg());

        // write image to disk
        if (write_debug_img_)
          cv::imwrite(absolute_img_path_ + SSTR(frame_count_) + ".jpg", debug_img);

        // show image on CV window
        if (show_debug_img_)
        {
          cv::imshow("Ball Tracking Exercise", debug_img);
          cv::waitKey(3);
        }
      }
    }

    last_processed_image_ = current_image_;
  }

  // publish current state
  publishBallStates(detector_->getBallModels());
}

void BallTrackingNode::publishBallStates(const std::vector<BallModel::Ptr>& balls) const
{
  if (ball_states_pub_.getNumSubscribers() > 0)
  {
    turtlebot3_exercise_msgs::BallStates msg;

    for (BallModel::ConstPtr ball_model : balls)
    {
      turtlebot3_exercise_msgs::BallState ball_state = ball_model->asMsgConst();
      ball_state.header.frame_id = tracking_frame_;
      transformBallState(world_frame_, ball_state, ball_state);
      msg.ball_states.push_back(ball_state);
    }
    msg.frame_number = frame_count_;

    ball_states_pub_.publish(msg);
  }
}

void BallTrackingNode::publishDetectedBalls2d(const std::vector<Detector::Ball2d>& balls_2d, const ros::Time& stamp){
  if (ball_2d_pub_.getNumSubscribers() > 0)
  {
    turtlebot3_exercise_msgs::BallDetections2d msg;

    for (auto ball : balls_2d)
    {
      turtlebot3_exercise_msgs::Ball2d ball_msg;
      ball_msg.pos.x = ball.pos.x;
      ball_msg.pos.y = ball.pos.y;
      ball_msg.radius = ball.radius;
      msg.balls.push_back(ball_msg);
    }
    msg.frame_number = frame_count_;
    msg.header.stamp = stamp;
    ball_2d_pub_.publish(msg);
  }
}

void BallTrackingNode::imageCb(const sensor_msgs::ImageConstPtr& image)
{
  current_image_ = image;
}

void BallTrackingNode::dynamicReconfigureCb(const turtlebot3_ball_tracking_exercise::BallTrackingConfig& config, uint32_t level)
{
  ROS_INFO("Parameters updated:\nHue: [%f, %f]\nSaturation: [%f, %f]\nValue: [%f, %f]",
           config.min_hue, config.max_hue, config.min_sat, config.max_sat, config.min_val, config.max_val);
  detector_->setHSVValues(config.min_hue, config.max_hue, config.min_sat, config.max_sat, config.min_val, config.max_val);
}

void BallTrackingNode::ballRequestActionGoalCb()
{
  // check if new goal was preempted in the meantime
  if (get_ball_states_as_->isPreemptRequested())
  {
    get_ball_states_as_->setPreempted();
    return;
  }

  // accept the new goal
  const turtlebot3_exercise_msgs::GetBallStatesGoalConstPtr goal = get_ball_states_as_->acceptNewGoal();

  // return result
  if (get_ball_states_as_->isActive())
  {
    // if empty, use robot position
    geometry_msgs::PointStamped position = goal->position.header.frame_id.empty() ? getRobotPosition() : goal->position;

    // get k nearest balls and return them in camera_link coordinate system
    std::vector<BallModel> k_nearest_balls = getKNearestBalls(position, goal->number_nearest_balls);

    // transform balls to target_frame
    turtlebot3_exercise_msgs::GetBallStatesResult action_result;

    for (BallModel& ball_model : k_nearest_balls)
    {
      turtlebot3_exercise_msgs::BallState& ball_state = ball_model.asMsg();
      ball_state.header.frame_id = tracking_frame_;
      transformBallState(goal->target_frame, ball_state, ball_state);
      action_result.ball_states.push_back(ball_state);
    }

    // send action result
    get_ball_states_as_->setSucceeded(action_result);
  }
}

geometry_msgs::PointStamped BallTrackingNode::getRobotPosition() const
{
  geometry_msgs::PointStamped position;
  position.header.frame_id = "base_footprint";

  try
  {
    tf_buffer_.transform(position, position, "world", ros::Duration(1));
  }
  catch (tf2::TransformException ex)
  {
    ROS_ERROR("[BallTrackingNode] Cannot determine robot position: %s", ex.what());
  }

  return position;
}

std::vector<BallModel> BallTrackingNode::getKNearestBalls(const geometry_msgs::PointStamped& point, int k) const
{
  geometry_msgs::PointStamped point_stamped = point;

  // transform query point into local base frame
  if (point_stamped.header.frame_id != tracking_frame_)
  {
    try
    {
      tf_buffer_.transform(point_stamped, point_stamped, tracking_frame_, ros::Duration(1));
    }
    catch (tf2::TransformException ex)
    {
      ROS_ERROR("[BallTrackingNode] getKNearestBalls: %s", ex.what());
    }
  }

  // get all distances between the input point and all balls
  const std::vector<BallModel::Ptr>& balls = detector_->getBallModels();
  typedef std::pair<double, BallModel::ConstPtr> pairType;
  std::vector<pairType> ordered_balls_vec;

  for (BallModel::ConstPtr ball : balls)
  {
    geometry_msgs::Point ball_pos;
    ball_pos.x = static_cast<double>(ball->getPredictedPosition().x);
    ball_pos.y = static_cast<double>(ball->getPredictedPosition().y);
    ball_pos.z = static_cast<double>(ball->getPredictedPosition().z);

    double dist = getDistance2D(point_stamped.point, ball_pos);

    ordered_balls_vec.push_back(std::make_pair(dist, ball));
  }

  // sort list ascending
  std::sort(std::begin(ordered_balls_vec), std::end(ordered_balls_vec), [](pairType& left, pairType& right)->bool { return left.first < right.first; });

  // get k nearest balls
  std::vector<BallModel> k_nearest_balls(std::min(static_cast<size_t>(k), ordered_balls_vec.size()));
  for (size_t i = 0; i < k_nearest_balls.size(); i++)
    k_nearest_balls.at(i) = *(ordered_balls_vec.at(i).second);

  return k_nearest_balls;
}

geometry_msgs::Point BallTrackingNode::transformVelocity(const geometry_msgs::TransformStamped& transform, const geometry_msgs::Point& velocity) const
{
  tf2::Stamped<tf2::Transform> tf2_transform;
  tf2::fromMsg(transform, tf2_transform);
  tf2_transform.setOrigin(tf2::Vector3(0, 0, 0));
  geometry_msgs::Point velocity_out;
  geometry_msgs::TransformStamped rotation = tf2::toMsg(tf2_transform);
  tf2::doTransform(velocity, velocity_out, rotation);
  return velocity_out;
}

//void BallTrackingNode::transformCov(const tf::StampedTransform& transform, const boost::array<double, 6>& cov_in, boost::array<double, 6>& cov_out) const
//{
//  tf::Quaternion q;
//  q.setRPY(cov_in[3], cov_in[4], cov_in[5]);
//  tf::Matrix3x3 cov = tf::Matrix3x3(transform.getRotation())*tf::Matrix3x3(q);
//  cov.getRPY(cov_out[3], cov_out[4], cov_out[5]);
//}

void BallTrackingNode::transformBallState(const std::string& target_frame, const turtlebot3_exercise_msgs::BallState& state_in, turtlebot3_exercise_msgs::BallState& state_out) const
{
  state_out = state_in;
  if (state_in.header.frame_id != target_frame)
  {
    if (tf_buffer_.canTransform(target_frame, state_in.header.frame_id, state_in.header.stamp))
    {

      geometry_msgs::TransformStamped transform;
      try
      {
        transform = tf_buffer_.lookupTransform(target_frame, state_in.header.frame_id, state_in.header.stamp);
      }
      catch (tf2::TransformException ex)
      {
        ROS_ERROR("[BallTrackingNode] transformBallState: %s", ex.what());
      }
      tf2::doTransform(state_in.position, state_out.position, transform);
      state_out.velocity = transformVelocity(transform, state_in.velocity);
      // TODO: Transform Covariance

      state_out.header.frame_id = target_frame;
    }
  }
}
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "turtlebot3_ball_tracking");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  turtlebot3::BallTrackingNode node(nh, pnh);
  ros::Rate loop_rate(pnh.param("update_rate", 1.0));

  while (ros::ok())
  {
    ros::spinOnce();
    node.update();

    if (!loop_rate.sleep())
      ROS_WARN_THROTTLE(10.0, "[turtlebot3_ball_tracking] Update rate was not met!");
  }

  return 0;
}
