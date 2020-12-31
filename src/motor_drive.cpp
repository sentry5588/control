#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <cmath>
#include <algorithm>

double PITCH_ANGLE_CTRL_KP = 0.0;
double PITCH_ANGLE_CTRL_KI = 0.0;
double PITCH_ANGLE_CTRL_KD = 0.0;
double PITCH_ANGLE_ERR_INTEGRAL_LIMIT = 0.0;
double PITCH_ANGLE_PID_SATURATION_LIMIT = 0.0;
double DISTANCE_CTRL_KP = 0.0;
double DISTANCE_CTRL_KD = 0.0;

void obtain_control_calibrations(const ros::NodeHandle &n) {
  n.getParam("/pitch_angle_ctrl_kp", PITCH_ANGLE_CTRL_KP);
  n.getParam("/pitch_angle_ctrl_ki", PITCH_ANGLE_CTRL_KI);
  n.getParam("/pitch_angle_ctrl_kd", PITCH_ANGLE_CTRL_KD);
  n.getParam("/pitch_angle_err_integral_limit", PITCH_ANGLE_ERR_INTEGRAL_LIMIT);
  n.getParam("/pitch_angle_pid_saturation_limit", PITCH_ANGLE_PID_SATURATION_LIMIT);
  n.getParam("/distance_ctrl_kp", DISTANCE_CTRL_KP);
  n.getParam("/distance_ctrl_kd", DISTANCE_CTRL_KD);
  
}

class PID_Control {
public:
  PID_Control() {
    left_motor_pub_ = n_.advertise<std_msgs::Int32>("/left_motor_hz", 10);
    right_motor_pub_ = n_.advertise<std_msgs::Int32>("/right_motor_hz", 10);
    sub_ = n_.subscribe("/pitch_angle", 1, &PID_Control::read_pitch_angle_callback, this);
  obtain_control_calibrations(n_);
  }
  void read_pitch_angle_callback(const std_msgs::Float32& msg)
  {
    std_msgs::Int32 left_pwm_msg, right_pwm_msg;

    distance_traveled_ = distance_traveled_ + pitch_angle_PID_ctrl * dT_ * 0.000236;
    pitch_angle_ref_ = distance_traveled_ * DISTANCE_CTRL_KP + pitch_angle_PID_ctrl * DISTANCE_CTRL_KD;
    ROS_INFO_STREAM("dist: " << distance_traveled_ << "; p ref: " << pitch_angle_ref_);

    // Pitch Angle PID loop
    pitch_angle_err = pitch_angle_ref_ - (double) msg.data;
    pitch_angle_P_ctrl = pitch_angle_err * PITCH_ANGLE_CTRL_KP;
    pitch_angle_err_integral += pitch_angle_err * dT_;
    // anti-windup I: integral limit
    pitch_angle_err_integral = std::max(pitch_angle_err_integral, -PITCH_ANGLE_ERR_INTEGRAL_LIMIT);
    pitch_angle_err_integral = std::min(pitch_angle_err_integral, PITCH_ANGLE_ERR_INTEGRAL_LIMIT);
    pitch_angle_I_ctrl = pitch_angle_err_integral * PITCH_ANGLE_CTRL_KI;
    pitch_angle_PID_ctrl = pitch_angle_P_ctrl + pitch_angle_I_ctrl;
    // PID control output limit
    pitch_angle_PID_ctrl = std::max(pitch_angle_PID_ctrl, -PITCH_ANGLE_PID_SATURATION_LIMIT);
    pitch_angle_PID_ctrl = std::min(pitch_angle_PID_ctrl, PITCH_ANGLE_PID_SATURATION_LIMIT);
    if (msg.data > 0.5 || msg.data < -0.5) { // if loss control, then disable PID outputs
      pitch_angle_PID_ctrl = 0.0;
    }

    left_pwm_msg.data = (int32_t) (pitch_angle_PID_ctrl);
    right_pwm_msg.data = (int32_t) (pitch_angle_PID_ctrl);
    left_motor_pub_.publish(left_pwm_msg);
    right_motor_pub_.publish(right_pwm_msg);
  }
private:
  ros::NodeHandle n_; 
  ros::Publisher left_motor_pub_;
  ros::Publisher right_motor_pub_;
  ros::Subscriber sub_;
  double pitch_angle_err_integral = 0.0, pitch_angle_err = 0.0;
  double pitch_angle_P_ctrl = 0.0, pitch_angle_I_ctrl = 0.0, pitch_angle_PID_ctrl = 0.0;
  double distance_traveled_ = 0.0, pitch_angle_ref_ = 0.0;
  int pitch_angle_loop_count = 0;
  const double dT_ = 0.01;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motor_drive");

  //Create an object of class SubscribeAndPublish that will take care of everything
  PID_Control control_algo;
  ros::spin();

  return 0;
}
