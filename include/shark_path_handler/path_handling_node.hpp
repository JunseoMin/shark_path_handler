#ifndef SHARK_PATH_HANDLER__PATH_HANDLING_NODE_H_
#define SHARK_PATH_HANDLER__PATH_HANDLING_NODE_H_

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

#include <tf/tf.h>

class PathHandlingNode
{
public:
  PathHandlingNode();

private:
  void gps_cb(const sensor_msgs::NavSatFix::ConstPtr& msg);
  void calc_sudo_gps();

  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh_;
  
  ros::Subscriber gps_subs_;
  ros::Publisher sudo_gps_pub_;

  sensor_msgs::NavSatFix sudo_gps_;
  
};
#endif  // SHARK_PATH_HANDLER__PATH_HANDLING_NODE_H_
