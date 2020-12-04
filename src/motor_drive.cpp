#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Imu.h>

void sub_read_imu(const sensor_msgs::Imu& msg)
{
  ROS_INFO_STREAM("I heard: " << msg.header.frame_id);
  ROS_INFO_STREAM("x acc: " << msg.linear_acceleration.x);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motor_drive");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("imu", 1000, sub_read_imu);

  ros::spin();

  return 0;
}
