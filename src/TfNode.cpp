#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
 
int main(int argc, char** argv){
  ros::init(argc, argv, "laser_tf_Node");
  ros::NodeHandle node;
 
  ros::Rate r(100);
 
  tf::TransformBroadcaster broadcaster;
 
  while(ros::ok()){
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(7, 0.0, 0.0)),ros::Time::now(),"laser1", "laser2"));
    r.sleep();
    ros::spinOnce();
  }
}
