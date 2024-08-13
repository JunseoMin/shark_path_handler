#ifndef SHARK_PATH_HANDLER__PATH_HANDLING_NODE_H_
#define SHARK_PATH_HANDLER__PATH_HANDLING_NODE_H_

#include <ros/ros.h>

class PathHandlingNode
{
public:
  PathHandlingNode();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh_;

};
#endif  // SHARK_PATH_HANDLER__PATH_HANDLING_NODE_H_
