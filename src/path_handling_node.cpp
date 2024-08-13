#include "shark_path_handler/path_handling_node.hpp"

PathHandlingNode::PathHandlingNode(){
  ROS_INFO("constructed!!");
}

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "path_handling_node");
  auto path_handling_node = PathHandlingNode();
  ros::spin();
  ros::shutdown();
  return 0;
}
