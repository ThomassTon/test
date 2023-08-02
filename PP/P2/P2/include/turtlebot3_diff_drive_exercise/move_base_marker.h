#ifndef TURTLEBOT3_MOVE_BASE_MARKER_H__
#define TURTLEBOT3_MOVE_BASE_MARKER_H__

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#define INT_MARKER_NAME "move_base"



namespace turtlebot3
{
class MoveBaseMarker
{
public:
  // typedefs
  typedef boost::shared_ptr<MoveBaseMarker> Ptr;
  typedef boost::shared_ptr<const MoveBaseMarker> ConstPtr;

  MoveBaseMarker(const std::string& topic, const std::string& nav_frame = "odom", double marker_scale = 1.0);
  virtual ~MoveBaseMarker();

  /**
   * @brief Insert menu element to right click panel
   * @param title Title of element
   * @param feedback_cb Callback when element was changed/clicked
   * @return Handle of menu item
   */
  interactive_markers::MenuHandler::EntryHandle insertMenuItem(const std::string& title, const interactive_markers::MenuHandler::FeedbackCallback& feedback_cb);

  /**
   * @brief Insert checkable menu element to right click panel
   * @param title Title of element
   * @param state Intial checked state
   * @param feedback_cb Callback when element was changed/clicked
   * @return Handle of menu item
   */
  interactive_markers::MenuHandler::EntryHandle insertCheckableMenuItem(const std::string& title, const interactive_markers::MenuHandler::CheckState& state, const interactive_markers::MenuHandler::FeedbackCallback& feedback_cb);

  /**
   * @brief Gets current checked state from menu item
   * @param handle Handle of menu item
   * @return checked state of menu item
   */
  interactive_markers::MenuHandler::CheckState getCheckState(const interactive_markers::MenuHandler::EntryHandle& handle) const;

  /**
   * @brief Sets new checked state of menu item
   * @param handle Handle of menu item
   * @param state New checked state of menu item
   */
  void setCheckState(const interactive_markers::MenuHandler::EntryHandle& handle, const interactive_markers::MenuHandler::CheckState& state);

  /**
   * @brief Controls visibility of single menu items
   * @param handle Handle of menu item
   * @param visible Sets the visibility of the selected menu item
   */
  void setVisible(const interactive_markers::MenuHandler::EntryHandle& handle, bool visible);

  /**
   * @brief Sets new pose of interactive marker.
   * @param pose new pose
   * @param header optional header defining frame in which the pose is given
   */
  void setPose(const geometry_msgs::Pose& pose, const std_msgs::Header& header = std_msgs::Header());

  /**
   * @brief Returns current pose of the interactive marker.
   * @return Current pose of interactive marker
   */
  const geometry_msgs::PoseStamped& getPose() const;

  /**
   * @brief Returns state if marker is moved
   * @return true if marker is currently moving
   */
  inline bool isMoving() const { return is_moving_; }

protected:
  void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr feedback);

  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;

  interactive_markers::MenuHandler menu_handler_;

  geometry_msgs::PoseStamped pose_;

  bool is_moving_;
};
}

#endif
