#include <turtlebot3_arm_controller_exercise/arm_controller_node.h>



namespace turtlebot3
{
ArmControllerNode::ArmControllerNode(ros::NodeHandle& nh)
  : controller_(nh)
{
}

ArmControllerNode::~ArmControllerNode()
{
}

void ArmControllerNode::update()
{
  controller_.update();
}
} // namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "turtlebot3_arm_controller");

  ros::NodeHandle nh;

  turtlebot3::ArmControllerNode node(nh);

  ros::Rate loop_rate(nh.param("arm_controller/control_rate", 1000.0));  //1000

  while (ros::ok())
  {
    ros::spinOnce();
    node.update();
    loop_rate.sleep();
  }

  return 0;
}
