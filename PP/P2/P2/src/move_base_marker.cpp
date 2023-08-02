#include <turtlebot3_diff_drive_exercise/move_base_marker.h>

#include <tf/tf.h>



namespace turtlebot3
{
MoveBaseMarker::MoveBaseMarker(const std::string& topic, const std::string& nav_frame, double marker_scale)
  : is_moving_(false)
{
  server_.reset(new interactive_markers::InteractiveMarkerServer(topic, "", false));

  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = nav_frame;
  int_marker.pose.orientation.w = 1.0;
  int_marker.scale = marker_scale;

  int_marker.name = INT_MARKER_NAME;

  visualization_msgs::InteractiveMarkerControl control;
  control.always_visible = true;

  /// init ghost hand
  visualization_msgs::Marker marker;
  //marker.type = visualization_msgs::Marker::SPHERE;

  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.mesh_resource = "package://tuda_turtlebot3_description/meshes/tuda_turtlebot3_no_arm.stl";
  marker.pose.position.x = 0.032;
  marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, M_PI);
  marker.scale.x = 0.001;
  marker.scale.y = 0.001;
  marker.scale.z = 0.001;
  marker.color.r = 0.1;
  marker.color.g = 0.8;
  marker.color.b = 0.1;
  marker.color.a = 1.0;
  control.markers.push_back(marker);
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;
  int_marker.controls.push_back(control);

  /// init moving axes
  control = visualization_msgs::InteractiveMarkerControl();

  // x-axis
  control.orientation.x = 0.707;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.orientation.w = 0.707;
  control.name = "move_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  // y-axis
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 0.707;
  control.orientation.w = 0.707;
  control.name = "move_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  // z-axis
  control.orientation.x = 0;
  control.orientation.y = 0.707;
  control.orientation.z = 0;
  control.orientation.w = 0.707;
  control.name = "rotate_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);

  /// Menu handler
  marker = visualization_msgs::Marker();
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = int_marker.scale * 0.1;
  marker.scale.y = int_marker.scale * 0.1;
  marker.scale.z = int_marker.scale * 0.1;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;
  control = visualization_msgs::InteractiveMarkerControl();
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
  control.always_visible = true;
  control.description = "Options";
  control.markers.push_back(marker);
  int_marker.controls.push_back(control);

  // init interactive server marker
  server_->insert(int_marker);
  server_->setCallback(int_marker.name, boost::bind(&MoveBaseMarker::processFeedback, this, _1));

  menu_handler_.apply(*server_, INT_MARKER_NAME);

  server_->applyChanges();
}

interactive_markers::MenuHandler::EntryHandle MoveBaseMarker::insertMenuItem(const std::string& title, const interactive_markers::MenuHandler::FeedbackCallback& feedback_cb)
{
  interactive_markers::MenuHandler::EntryHandle handle = menu_handler_.insert(title, feedback_cb);
  menu_handler_.reApply(*server_);
  server_->applyChanges();
  return handle;
}

interactive_markers::MenuHandler::EntryHandle MoveBaseMarker::insertCheckableMenuItem(const std::string& title, const interactive_markers::MenuHandler::CheckState& state, const interactive_markers::MenuHandler::FeedbackCallback& feedback_cb)
{
  interactive_markers::MenuHandler::EntryHandle handle = menu_handler_.setCheckState(menu_handler_.insert(title, feedback_cb), state);
  menu_handler_.reApply(*server_);
  server_->applyChanges();
  return handle;
}

interactive_markers::MenuHandler::CheckState MoveBaseMarker::getCheckState(const interactive_markers::MenuHandler::EntryHandle& handle) const
{
  interactive_markers::MenuHandler::CheckState state;
  menu_handler_.getCheckState(handle, state);
  return state;
}

void MoveBaseMarker::setCheckState(const interactive_markers::MenuHandler::EntryHandle& handle, const interactive_markers::MenuHandler::CheckState& state)
{
  menu_handler_.setCheckState(handle, state);
  menu_handler_.reApply(*server_);
  server_->applyChanges();
}

void MoveBaseMarker::setVisible(const interactive_markers::MenuHandler::EntryHandle& handle, bool visible)
{
  menu_handler_.setVisible(handle, visible);
  menu_handler_.reApply(*server_);
  server_->applyChanges();
}

MoveBaseMarker::~MoveBaseMarker()
{
  server_->clear();
}

void MoveBaseMarker::setPose(const geometry_msgs::Pose& pose, const std_msgs::Header& header)
{
  server_->setPose(INT_MARKER_NAME, pose);
  server_->applyChanges();
  pose_.header = header;
  pose_.pose = pose;
}

const geometry_msgs::PoseStamped& MoveBaseMarker::getPose() const
{
  return pose_;
}

void MoveBaseMarker::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr feedback)
{
  //ROS_INFO_STREAM(*feedback);

  // only trigger on mouse release
  if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP)
    is_moving_ = false;
  else
    is_moving_ = true;

  // constrain z-level
  geometry_msgs::Pose pose = feedback->pose;
  pose.position.z = getPose().pose.position.z;

  // update
  setPose(pose, feedback->header);
}
} // namespace
