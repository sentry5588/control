#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Imu.h>
#include <cmath>
#include "motor_drive/motor_drive_diagnostics.h"

double pitch_angle_comp_filtered = 0.0;
motor_drive::motor_drive_diagnostics m_diag_msg;

void sub_read_imu(const sensor_msgs::Imu& msg)
{
  double p_accel = 0.0, dp_gyro = 0.0, dT = 0.02;
  double alpha = 0.98; // To tune
//  ROS_INFO_STREAM("I heard: " << msg.header.frame_id);
//  ROS_INFO_STREAM("x acc: " << msg.linear_acceleration.x);
  p_accel = atan2(msg.linear_acceleration.z, msg.linear_acceleration.x) - 0.205; // radian
  dp_gyro = msg.angular_velocity.y;
  pitch_angle_comp_filtered = (1-alpha) * p_accel + alpha * (pitch_angle_comp_filtered + dp_gyro * dT);
  ROS_INFO_STREAM("Filtered pitch angle is: " << pitch_angle_comp_filtered);

  m_diag_msg.pitch_ang_meas = p_accel;
  m_diag_msg.pitch_ang_rate = dp_gyro;
  m_diag_msg.x_accel = msg.linear_acceleration.x;
  m_diag_msg.z_accel = msg.linear_acceleration.z;
  m_diag_msg.pitch_ang_filtered = pitch_angle_comp_filtered;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motor_drive");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("imu", 1000, sub_read_imu);
  ros::Publisher pub = n.advertise<motor_drive::motor_drive_diagnostics>("compl_filter_diag", 10);
  ros::Rate rate(50); // Hz

  

  while(ros::ok()) {
    pub.publish(m_diag_msg);

    ROS_INFO_STREAM("motor_drive main function");
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
