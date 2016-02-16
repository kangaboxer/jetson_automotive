#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
 
int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;
  
  ros::Rate r(10);
  
  tf::TransformBroadcaster broadcaster;
  
  while(n.ok()){
    broadcaster.sendTransform(
       tf::StampedTransform(
	  tf::Transform(tf::Quaternion(-1, 0, 0, 1), tf::Vector3(0.0, -0.05, -0.10)),
	  ros::Time::now(), 
	  "ORB_SLAM/Camera",
	  "base_link"));
    broadcaster.sendTransform(
       tf::StampedTransform(
	  tf::Transform(tf::Quaternion(0, 0, 1, -1), tf::Vector3(0.0, 0, 0)),
	  ros::Time::now(), 
	  "base_link",
	  "laser"));
    broadcaster.sendTransform(
       tf::StampedTransform(
	  tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0, 0)),
	  ros::Time::now(), 
	  "global",
	  "map"));
    broadcaster.sendTransform(
       tf::StampedTransform(
	  tf::Transform(tf::Quaternion(1, 0, 0, 1.0), tf::Vector3(0, -0.10, 0.05)),
	  ros::Time::now(), 
	  "global",
	  "ORB_SLAM/World"));
    r.sleep();
  }
}
