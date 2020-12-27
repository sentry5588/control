#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <cmath>
#include "motor_drive/motor_drive_diagnostics.h"
#include <algorithm>

double PITCH_ANGLE_CTRL_KP = 0.0;
double PITCH_ANGLE_CTRL_KI = 0.0;
double PITCH_ANGLE_CTRL_KD = 0.0;
double PITCH_ANGLE_ERR_INTEGRAL_LIMIT = 0.0;
double PITCH_ANGLE_PID_SATURATION_LIMIT = 0.0;

double pitch_angle = 0.0;

void obtain_control_calibrations(const ros::NodeHandle &n) {
  n.getParam("/pitch_angle_ctrl_kp", PITCH_ANGLE_CTRL_KP);
  n.getParam("/pitch_angle_ctrl_ki", PITCH_ANGLE_CTRL_KI);
  n.getParam("/pitch_angle_ctrl_kd", PITCH_ANGLE_CTRL_KD);
  n.getParam("/pitch_angle_err_integral_limit", PITCH_ANGLE_ERR_INTEGRAL_LIMIT);
  n.getParam("/pitch_angle_pid_saturation_limit", PITCH_ANGLE_PID_SATURATION_LIMIT);
}

void read_pitch_cb(const std_msgs::Float32& msg)
{
  pitch_angle = (double) msg.data;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motor_drive");
  ros::NodeHandle n;
  ros::Subscriber imu_sub = n.subscribe("pitch_angle", 1000, read_pitch_cb);
//  ros::Publisher left_motor_pub = n.advertise<motor_drive::motor_drive_diagnostics>("compl_filter_diag", 10);
  ros::Publisher left_motor_pub = n.advertise<std_msgs::Int32>("left_motor_hz", 10);
  ros::Publisher right_motor_pub = n.advertise<std_msgs::Int32>("right_motor_hz", 10);
  ros::Rate rate(50); // Hz

  obtain_control_calibrations(n);

  double pitch_angle_err_integral = 0.0, pitch_angle_err = 0.0;
  double pitch_angle_P_ctrl = 0.0, pitch_angle_I_ctrl = 0.0, pitch_angle_PID_ctrl = 0.0;
  while(ros::ok()) {
    std_msgs::Int32 left_pwm_msg, right_pwm_msg;

    // Pitch Angle PID loop
    pitch_angle_err = pitch_angle;
    pitch_angle_P_ctrl = pitch_angle_err * PITCH_ANGLE_CTRL_KP;
    pitch_angle_err_integral += pitch_angle_err * 0.02;
    // anti-windup I: integral limit
    pitch_angle_err_integral = std::max(pitch_angle_err_integral, -PITCH_ANGLE_ERR_INTEGRAL_LIMIT);
    pitch_angle_err_integral = std::min(pitch_angle_err_integral, PITCH_ANGLE_ERR_INTEGRAL_LIMIT);
    pitch_angle_I_ctrl = pitch_angle_err_integral * PITCH_ANGLE_CTRL_KI;
    pitch_angle_PID_ctrl = pitch_angle_P_ctrl + pitch_angle_I_ctrl;
    // PID control output limit
    pitch_angle_PID_ctrl = std::max(pitch_angle_PID_ctrl, -PITCH_ANGLE_PID_SATURATION_LIMIT);
    pitch_angle_PID_ctrl = std::min(pitch_angle_PID_ctrl, PITCH_ANGLE_PID_SATURATION_LIMIT);
    if (pitch_angle_err > 0.5 || pitch_angle_err < -0.5) { // if loss control, then disable PID outputs
      pitch_angle_PID_ctrl = 0.0;
    }

    left_pwm_msg.data = (int32_t) (pitch_angle_PID_ctrl);
    right_pwm_msg.data = (int32_t) (pitch_angle_PID_ctrl);
    left_motor_pub.publish(left_pwm_msg);
    right_motor_pub.publish(right_pwm_msg);

//    ROS_INFO_STREAM("motor_drive main function");
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
