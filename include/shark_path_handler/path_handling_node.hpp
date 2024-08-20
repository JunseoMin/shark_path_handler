#ifndef SHARK_PATH_HANDLER__PATH_HANDLING_NODE_H_
#define SHARK_PATH_HANDLER__PATH_HANDLING_NODE_H_

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>

#include <tf/tf.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistWithCovariance.h>

#include <morai_msgs/GPSMessage.h>

class PathHandlingNode
{
public:
  PathHandlingNode(ros::NodeHandle nh_);

private:
  void odom_cb(const nav_msgs::Odometry::ConstPtr& msg);
  void gps_cb(const morai_msgs::GPSMessage::ConstPtr& msg);
  void timer_cb(const ros::TimerEvent&);
  void set_odom();

  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh_;
  
  ros::Subscriber gps_subs_;
  ros::Publisher sudo_gps_pub_;

  ros::Subscriber odom_subs_;
  ros::Publisher odom_pub_;

  sensor_msgs::NavSatFix sudo_gps_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  ros::Timer timer_;
  geometry_msgs::PoseWithCovariance pose_;
  geometry_msgs::TwistWithCovariance twist_;

  nav_msgs::Odometry odom_;

  bool black_flag_;
};
#endif  // SHARK_PATH_HANDLER__PATH_HANDLING_NODE_H_
