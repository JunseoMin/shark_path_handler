#include "shark_path_handler/path_handling_node.hpp"

PathHandlingNode::PathHandlingNode()
:nh_(),priv_nh_("~")
{
  ROS_INFO("constructed!!");
  gps_subs_ = nh_.subscribe<sensor_msgs::NavSatFix>("/gps",10,&PathHandlingNode::gps_cb);
  sudo_gps_pub_ = nh_.advertise<sensor_msgs::NavSatFix>("/sudo_gps",10);
}

void PathHandlingNode::gps_cb(const sensor_msgs::NavSatFix::ConstPtr& msg){
  if( msg->altitude != 0.0 && msg->latitude != 0.0 and msg->longitude != 0.0){
    return;
  }

  assert(!msg->altitude);
  sudo_gps_.header.frame_id = "base_link";

  calc_sudo_gps();

  sudo_gps_pub_.publish(sudo_gps_);
}

void PathHandlingNode::calc_sudo_gps(){
  // create gps from map->base_link tf
  // longitude 1 == 90180m
  // latitude 1 == 110940m

  
}

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "path_handling_node");
  auto path_handling_node = PathHandlingNode();
  ros::spin();
  ros::shutdown();
  return 0;
}
