#include <turtlebot3_diff_drive_exercise/diff_drive_node.h>

#include <std_msgs/Float64.h>



namespace turtlebot3
{
DiffDriveNode::DiffDriveNode(ros::NodeHandle& nh)
  : odometry_(nh, 3)
  , diff_drive_(nh)
  , diff_drive_control_(nh)
  , auto_snap_to_current_pose_(true)
{
  // get parameters
  nav_frame_ = nh.param("diff_drive/nav_frame", std::string("odom"));
  base_frame_ = nh.param("diff_drive/base_frame", std::string("base_footprint"));

  // init marker
  move_base_marker_.reset(new turtlebot3::MoveBaseMarker("move_to", nh.param("diff_drive/nav_frame", std::string("odom")), nh.param("diff_drive/marker_scale", 1.0)));

  // init marker menu
  move_base_marker_->insertMenuItem("Snap to Robot", boost::bind(&DiffDriveNode::snapToCurrentPose, this, _1));
  move_base_marker_->insertMenuItem("Move", boost::bind(&DiffDriveNode::moveBaseCmd, this, _1));
  move_base_marker_->insertMenuItem("Stop", boost::bind(&DiffDriveNode::stop, this, _1));

  // subscriber
  joint_state_sub_ = nh.subscribe("joints/joint_states", 1, &DiffDriveNode::jointStateCb, this);
  imu_sub_ = nh.subscribe("sensor/imu_filtered", 1, &DiffDriveNode::imuCb, this);
  cmd_vel_sub_ = nh.subscribe("cmd_vel", 1, &DiffDriveNode::cmdVelCb, this);
  reset_sub_ = nh.subscribe("reset", 1, &DiffDriveNode::resetCb, this);

  // publisher
  odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom", 1, true);
  wheel_l_vel_cmd_pub_ = nh.advertise<std_msgs::Float64>("joints/wheel_left_controller/command", 1, true);
  wheel_r_vel_cmd_pub_ = nh.advertise<std_msgs::Float64>("joints/wheel_right_controller/command", 1, true);
  cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);

  // init action servers
  move_base_as_.reset(new MoveBaseActionServer(nh, "move_base", false));
  move_base_as_->registerGoalCallback(boost::bind(&DiffDriveNode::moveBaseActionGoalCb, this));
  move_base_as_->registerPreemptCallback(boost::bind(&DiffDriveNode::moveBaseActionPreemptCb, this));
  move_base_as_->start();
  
  last_odom_time_stamp_ = ros::Time::now();
}

DiffDriveNode::~DiffDriveNode()
{
}

void DiffDriveNode::update(const ros::Duration& time_diff)
{
  // update odometry
  nav_msgs::Odometry odom_msg;
  if (last_joint_state_msg_ && last_imu_msg_)
  {
    odom_msg = odometry_.updateOdometry(last_joint_state_msg_, last_imu_msg_);
    if (!odom_msg.header.stamp.isZero() && odom_msg.header.stamp != last_odom_time_stamp_)
    {
      odom_pub_.publish(odom_msg);
      last_odom_time_stamp_ = odom_msg.header.stamp;
    }
  }

  // execute diff drive controller
  if (diff_drive_control_.isActive())
  {
    geometry_msgs::PoseStamped pose;
    if (getRobotPose(pose))
      cmd_vel_pub_.publish(diff_drive_control_.computeTwist(time_diff, pose.pose, odom_msg));
  }
  else if (move_base_as_->isActive())
  {
    // send action result
    move_base_as_->setSucceeded();
  }

  // handle auto snapping of marker at the very beginning
  if (auto_snap_to_current_pose_)
  {
    geometry_msgs::PoseStamped pose;
    if (getRobotPose(pose))
    {
      move_base_marker_->setPose(pose.pose, pose.header);
      auto_snap_to_current_pose_ = false;
    }
  }
}

bool DiffDriveNode::getRobotPose(geometry_msgs::PoseStamped& pose) const
{
  try
  {
    if (tf_listener_.canTransform(nav_frame_, base_frame_, ros::Time(0)))
    {
      tf::StampedTransform transform;
      tf_listener_.lookupTransform(nav_frame_, base_frame_, ros::Time(0), transform);
      tf::poseStampedTFToMsg(tf::Stamped<tf::Pose>(transform, transform.stamp_, transform.frame_id_), pose);
      return true;
    }
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("[DiffDriveNode] %s", ex.what());
    ros::Duration(1.0).sleep();
  }

  return false;
}

void DiffDriveNode::snapToCurrentPose(const visualization_msgs::InteractiveMarkerFeedbackConstPtr /*feedback*/)
{
  // move marker
  geometry_msgs::PoseStamped pose;
  if (getRobotPose(pose))
    move_base_marker_->setPose(pose.pose, pose.header);
}

void DiffDriveNode::moveBaseCmd(const visualization_msgs::InteractiveMarkerFeedbackConstPtr /*feedback*/)
{
  geometry_msgs::PoseStamped goal = move_base_marker_->getPose();
  goal.header.stamp = ros::Time::now();
  moveBase(goal);
}

void DiffDriveNode::stop(const visualization_msgs::InteractiveMarkerFeedbackConstPtr /*feedback*/)
{
  diff_drive_control_.stop();
  cmd_vel_pub_.publish(geometry_msgs::Twist()); // use cmd_vel publisher in order to update ui as well
}

void DiffDriveNode::jointStateCb(const sensor_msgs::JointState::ConstPtr msg)
{
  last_joint_state_msg_ = msg;
}

void DiffDriveNode::imuCb(const sensor_msgs::Imu::ConstPtr msg)
{
  last_imu_msg_ = msg;
}

void DiffDriveNode::cmdVelCb(const ros::MessageEvent<geometry_msgs::Twist const>& event)
{
  // stop diff drive controller on external commands
  if (diff_drive_control_.isActive() && event.getPublisherName() != ros::this_node::getName())
  {
    ROS_WARN("[DiffDriveNode] Diff Drive Controller interrupted by '%s'.", event.getPublisherName().c_str());
    diff_drive_control_.stop();

    if (move_base_as_->isActive())
    {
      // send action result
      move_base_as_->setPreempted();
    }
  }

  const geometry_msgs::TwistConstPtr& msg = event.getMessage();

  // convert twist message into wheel velocities
  std_msgs::Float64 wheel_l_vel_msg;
  std_msgs::Float64 wheel_r_vel_msg;

  diff_drive_.computeWheelVelocities(*msg, wheel_l_vel_msg.data, wheel_r_vel_msg.data);

  // publish command
  wheel_l_vel_cmd_pub_.publish(wheel_l_vel_msg);
  wheel_r_vel_cmd_pub_.publish(wheel_r_vel_msg);
}

void DiffDriveNode::moveBaseCb(const geometry_msgs::PoseStamped& msg)
{
  geometry_msgs::PoseStamped goal = msg;

  try
  {
    std::string error;
    if (tf_listener_.canTransform(nav_frame_, goal.header.frame_id, goal.header.stamp, &error))
      tf_listener_.transformPose(nav_frame_, goal, goal);
    else
      ROS_ERROR("[DiffDriveNode] Cannot transform from '%s' to '%s': %s", goal.header.frame_id.c_str(), nav_frame_.c_str(), error.c_str());
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("[DiffDriveNode] %s", ex.what());
    return;
  }

  // constrain z-level
  geometry_msgs::PoseStamped robot_pose;
  if (getRobotPose(robot_pose))
    goal.pose.position.z = robot_pose.pose.position.z;

  move_base_marker_->setPose(goal.pose, goal.header);
  moveBase(goal);
}

void DiffDriveNode::resetCb(const std_msgs::StringConstPtr msg)
{
  if (msg->data == "world" || msg->data == "odom")
    odometry_.reset();
}

void DiffDriveNode::moveBaseActionGoalCb()
{
  // accept the new goal
  turtlebot3_exercise_msgs::MoveBaseGoalConstPtr goal = move_base_as_->acceptNewGoal();

  // check if new goal was preempted in the meantime
  if (move_base_as_->isPreemptRequested())
  {
    move_base_as_->setPreempted();
    return;
  }

  moveBaseCb(goal->goal_pose);
}

void DiffDriveNode::moveBaseActionPreemptCb()
{
  move_base_as_->setPreempted();
  stop(visualization_msgs::InteractiveMarkerFeedbackConstPtr());
}

void DiffDriveNode::moveBase(const geometry_msgs::PoseStamped& goal)
{
  // check if pose must be transformed
  if (goal.header.frame_id != nav_frame_)
  {
    try
    {
      std::string error;
      if (tf_listener_.canTransform(nav_frame_, goal.header.frame_id, goal.header.stamp, &error))
      {
        geometry_msgs::PoseStamped goal_transformed;
        tf_listener_.transformPose(nav_frame_, goal, goal_transformed);
        diff_drive_control_.setGoalPose(goal_transformed.pose);
      }
      else
        ROS_ERROR("[DiffDriveNode] Cannot transform from '%s' to '%s': %s", goal.header.frame_id.c_str(), nav_frame_.c_str(), error.c_str());
    }
    catch (tf::TransformException& ex)
    {
      ROS_ERROR("[DiffDriveNode] %s", ex.what());
      return;
    }
  }
  else
    diff_drive_control_.setGoalPose(goal.pose);
}
} // namespace



int main(int argc, char **argv)
{
  ros::init(argc, argv, "turtlebot3_diff_drive");

  ros::NodeHandle nh;

  turtlebot3::DiffDriveNode node(nh);

  ros::Rate loop_rate(nh.param("update_rate", 100.0));

  ros::Time last_call = ros::Time::now();

  while (ros::ok())
  {
    ros::Time now = ros::Time::now();
    ros::spinOnce();
    node.update(now-last_call);
    loop_rate.sleep();
    last_call = now;
  }

  return 0;
}
