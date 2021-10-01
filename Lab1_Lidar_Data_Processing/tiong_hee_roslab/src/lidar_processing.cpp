#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float64.h"
#include "tiong_hee_roslab/scan_range.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
ros::Publisher lidar_closest_pub;
ros::Publisher lidar_furtest_pub;
ros::Publisher lidar_scan_range_pub;
std_msgs::Float64 closest_distance; 
std_msgs::Float64  furtest_distance;
tiong_hee_roslab::scan_range scan_range; 

void LidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  ROS_INFO("I heard: max:[%f], min:[%f]", msg->range_max, msg->range_min);
  closest_distance.data = msg->range_min;
  furtest_distance.data = msg->range_max; 
  scan_range.max_range = msg->range_max; 
  scan_range.min_range = msg->range_min; 

  lidar_closest_pub.publish(closest_distance); 
  lidar_furtest_pub.publish(furtest_distance);
  lidar_scan_range_pub.publish(scan_range); 

  
}



int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "listener");

  
  ros::NodeHandle n;
  lidar_closest_pub = n.advertise<std_msgs::Float64>("closest_point", 1000);
  lidar_furtest_pub = n.advertise<std_msgs::Float64>("farthest_point", 1000);
  lidar_scan_range_pub = n.advertise<tiong_hee_roslab::scan_range>("scan_range", 1000);
 
  ros::Subscriber sub = n.subscribe("scan", 1000, LidarCallback);



 

  
  
  ros::spin();
  

  return 0;
}
