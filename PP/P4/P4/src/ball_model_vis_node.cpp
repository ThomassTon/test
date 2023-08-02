#include <turtlebot3_ball_tracking_exercise/ball_model_vis_node.h>

#include <tf/tf.h>

#include <turtlebot3_ball_tracking_exercise/helper.h>



namespace turtlebot3
{
BallModelVisNode::BallModelVisNode(ros::NodeHandle& nh)
{
  base_frame_ = nh.param("ball_tracking/tracking_frame", std::string("world"));

  visual_tools_.reset(new rviz_visual_tools::RvizVisualTools(base_frame_, "ball_tracking/ball_states_vis"));

  // subscriber
  ball_states_sub_ = nh.subscribe("ball_tracking/ball_states", 1, &BallModelVisNode::ballStatesCb, this);
}

BallModelVisNode::~BallModelVisNode()
{
}

void BallModelVisNode::ballStatesCb(turtlebot3_exercise_msgs::BallStatesConstPtr msg) const
{
  // Remove all existing markers
  visual_tools_->deleteAllMarkers();

  // generate new markers
  for (const turtlebot3_exercise_msgs::BallState& state : msg->ball_states)
  {
    visualization_msgs::Marker text_marker;
    // create ball vis
    visualization_msgs::Marker sphere_marker;
    sphere_marker.action = visualization_msgs::Marker::ADD;
    sphere_marker.type = visualization_msgs::Marker::SPHERE;
    sphere_marker.ns = "sphere";
    sphere_marker.id = static_cast<int>(state.uid);
    sphere_marker.header = state.header;
    sphere_marker.pose.position = state.position;
    sphere_marker.pose.orientation.w = 1.0;
    sphere_marker.scale.x = state.diameter;
    sphere_marker.scale.y = state.diameter;
    sphere_marker.scale.z = state.diameter;
    sphere_marker.color = visual_tools_->getColorScale(state.confidence);
    visual_tools_->publishMarker(sphere_marker);

    // draw covariance as disc
    visualization_msgs::Marker cov_markers;
    cov_markers = sphere_marker;
    cov_markers.ns = "covariance";
    float data[4] = {static_cast<float>(state.covariance[0]), 0.0f, 0.0f, static_cast<float>(state.covariance[1])};
    cv::Mat cov2d = cv::Mat(2, 2, CV_32F, data);
    cv::RotatedRect rect = get2DErrorEllipse(cv::Point2f(state.position.x, state.position.y), cov2d, 5.991);
    cov_markers.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(state.covariance[3], state.covariance[4], state.covariance[5] - rect.angle);
    cov_markers.scale.x = static_cast<double>(rect.size.width);
    cov_markers.scale.y = static_cast<double>(rect.size.height);
    cov_markers.scale.z = 0.01;
    cov_markers.color = visual_tools_->getColor(rviz_visual_tools::RED);
    cov_markers.color.a = 0.5;
    visual_tools_->publishMarker(cov_markers);

    // add text
    text_marker = sphere_marker;
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.ns = "text";
    text_marker.pose.position.x += 0.6 * state.diameter;
    text_marker.pose.position.y -= 0.6 * state.diameter;
    text_marker.pose.position.z += 0.6 * state.diameter;
    text_marker.color = visual_tools_->getColor(rviz_visual_tools::WHITE);
    text_marker.text = std::to_string(state.uid);
    visual_tools_->publishMarker(text_marker);
  }
  // trigger the publisher
  visual_tools_->trigger();
}
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "turtlebot3_ball_model_vis");

  ros::NodeHandle nh;

  turtlebot3::BallModelVisNode node(nh);
  ros::spin();

  return 0;
}
