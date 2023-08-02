#ifndef TURTLEBOT3_ARM_CONTROLLER_NODE_H__
#define TURTLEBOT3_ARM_CONTROLLER_NODE_H__

#include <ros/ros.h>

#include <turtlebot3_arm_controller_exercise/arm_controller.h>



namespace turtlebot3
{
class ArmControllerNode
{
public:
  ArmControllerNode(ros::NodeHandle& nh);
  virtual ~ArmControllerNode();

  void update();

protected:
  ArmController controller_;
};
}

#endif
