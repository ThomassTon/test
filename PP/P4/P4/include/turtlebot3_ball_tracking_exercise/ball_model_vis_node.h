#ifndef TURTLEBOT3_BALL_MODEL_VIS_NODE_H__
#define TURTLEBOT3_BALL_MODEL_VIS_NODE_H__

#include <ros/ros.h>

#include <rviz_visual_tools/rviz_visual_tools.h>

#include <turtlebot3_exercise_msgs/BallStates.h>



namespace turtlebot3
{
class BallModelVisNode
{
public:
  BallModelVisNode(ros::NodeHandle& nh);
  virtual ~BallModelVisNode();

protected:
  void ballStatesCb(turtlebot3_exercise_msgs::BallStatesConstPtr msg) const;

  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
  std::string base_frame_;

  // subscriber
  ros::Subscriber ball_states_sub_;
};
}

#endif
