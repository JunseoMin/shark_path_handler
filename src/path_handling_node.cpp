#include "shark_path_handler/path_handling_node.hpp"

PathHandlingNode::PathHandlingNode(ros::NodeHandle nh_)
:nh_(nh_),priv_nh_("~"),black_flag_(false),tf_buffer_(),tf_listener_(tf_buffer_)
{
  ROS_INFO("constructed!!");
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom",10);
  odom_subs_ = nh_.subscribe<nav_msgs::Odometry>("/Odometry", 10, &PathHandlingNode::odom_cb, this);  
  odom_subs_valid_ = nh_.subscribe<nav_msgs::Odometry>("/odom", 10, &PathHandlingNode::odom_cb_valid, this);  

  gps_subs_ = nh_.subscribe<morai_msgs::GPSMessage>("/gps",10, &PathHandlingNode::gps_cb, this);
  timer_ = nh_.createTimer(ros::Duration(0.1), &PathHandlingNode::timer_cb, this);

  set_odom();
}

void PathHandlingNode::odom_cb(const nav_msgs::Odometry::ConstPtr& msg){
  // start if blackout
  if (!black_flag_) 
  {
    return;
  }

  geometry_msgs::PoseStamped pose_camera_init;
  pose_camera_init.pose = msg->pose.pose;
  pose_camera_init.header.frame_id = "camera_init";
  pose_camera_init.header.stamp = ros::Time::now();

  geometry_msgs::PoseStamped pose_odom;
  tf2::doTransform(pose_camera_init, pose_odom, last_transform_);

  odom_.pose.pose.position.x += pose_odom.pose.position.x;
  odom_.pose.pose.position.y += pose_odom.pose.position.y;
  odom_.pose.pose.position.z += pose_odom.pose.position.z;

  odom_.pose.pose.orientation = pose_odom.pose.orientation;
  odom_.twist = msg->twist;
}

void PathHandlingNode::odom_cb_valid(const nav_msgs::Odometry::ConstPtr& msg){
  //case not blackout
  if (!black_flag_) 
  {
    pose_ = msg->pose;
    twist_ = msg->twist;
    odom_ = *msg;  
  }
  return;
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

void PathHandlingNode::timer_cb(const ros::TimerEvent&) {
  if (!black_flag_) {
    return;
  }

  try {
    ros::Time now = ros::Time::now();

    geometry_msgs::TransformStamped T_cb;
    geometry_msgs::TransformStamped T_ob;
    geometry_msgs::TransformStamped T_co;

    T_cb = tf_buffer_.lookupTransform("camera_init", "base_link", now);
    T_ob = tf_buffer_.lookupTransform("odom", "base_link", now);

    tf2::Transform tf2_T_cb, tf2_T_ob;
    tf2::fromMsg(T_cb.transform, tf2_T_cb);
    tf2::fromMsg(T_ob.transform, tf2_T_ob);

    tf2::Transform tf2_T_co = tf2_T_cb.inverse() * tf2_T_ob;

    T_co.transform = tf2::toMsg(tf2_T_co);
    T_co.header.stamp = now;
    T_co.header.frame_id = "camera_init";
    T_co.child_frame_id = "odom";

    last_transform_ = T_co;

    odom_.header.stamp = now;
    odom_pub_.publish(odom_);
  } 
  catch (tf2::TransformException &ex) {
    ROS_WARN("Could not transform pose: %s", ex.what());
  }
}


void PathHandlingNode::set_odom(){
  odom_.child_frame_id = "base_link";
  odom_.header.frame_id = "odom";
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