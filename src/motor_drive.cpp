#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Imu.h>
#include <cmath>

double pitch_angle_comp_filtered = 0.0;

void sub_read_imu(const sensor_msgs::Imu& msg)
{
  double p_accel = 0.0, dp_gyro = 0.0, dT = 0.02;
  double alpha = 0.98; // To tune
  ROS_INFO_STREAM("I heard: " << msg.header.frame_id);
  ROS_INFO_STREAM("x acc: " << msg.linear_acceleration.x);
  p_accel = atan2(msg.linear_acceleration.x, msg.linear_acceleration.z) - 0.205; // radian
  dp_gyro = msg.angular_velocity.y;
  pitch_angle_comp_filtered = (1-alpha) * p_accel + alpha * (pitch_angle_comp_filtered + dp_gyro * dT);
  ROS_INFO_STREAM("Filtered pitch angle is: " << pitch_angle_comp_filtered); 
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motor_drive");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("imu", 1000, sub_read_imu);
  ros::spin();

  ROS_INFO_STREAM("*** motor_drive main function");

  return 0;
}
