#include "shark_path_handler/path_handling_node.hpp"

PathHandlingNode::PathHandlingNode(ros::NodeHandle nh_)
:nh_(nh_),priv_nh_("~"),black_flag_(false),tf_buffer_(),tf_listener_(tf_buffer_)
{
  ROS_INFO("constructed!!");
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom",10);
  odom_subs_ = nh_.subscribe<nav_msgs::Odometry>("/Odometry", 10, &PathHandlingNode::odom_cb, this);  

  gps_subs_ = nh_.subscribe<morai_msgs::GPSMessage>("/gps",10, &PathHandlingNode::gps_cb, this);
  timer_ = nh_.createTimer(ros::Duration(0.1), &PathHandlingNode::timer_cb, this);

  set_odom();
}

void PathHandlingNode::odom_cb(const nav_msgs::Odometry::ConstPtr& msg){
  if (!black_flag_) //case not blackout
  {
    return;
  }
  
  pose_ = msg->pose;
  twist_ = msg->twist;
}

void PathHandlingNode::gps_cb(const morai_msgs::GPSMessage::ConstPtr& msg){
  if( msg->altitude != 0.0 || msg->latitude != 0.0 || msg->longitude != 0.0){
    black_flag_ = false;
    ROS_INFO("stable.. return!");
    return;
  }
  ROS_INFO("blackout situation!");
  black_flag_ = true;
}

void PathHandlingNode::timer_cb(const ros::TimerEvent&){
  if (!black_flag_)
  {
    return;
  }

  odom_.header.stamp = ros::Time::now();
  odom_.pose = pose_;
  odom_.twist = twist_;
  odom_pub_.publish(odom_);
}

void PathHandlingNode::set_odom(){
  odom_.child_frame_id = "base_link";
  odom_.header.frame_id = "camera_init";
}

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "path_handling_node");

  ros::NodeHandle nh;
  PathHandlingNode handling_node(nh);
  ros::spin();
  ros::shutdown();
  
  return 0;
}
