#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <algorithm>

float speed = 0;
float set_point;

class PID_control
{
  private:
    float kp, ki, kd;
    float smoothing_factor = 0.5, last_filtered_speed = 0, filtered_speed = 0;
    ros::Time last_time_PID;
    float error = 0, last_error = 0, error_sum = 0, p = 0, i = 0 , d = 0;
    ros::NodeHandle nh;
    ros::Subscriber read_speed;
    ros::Subscriber cmd_vel_sub;
  public:

    //constructor function to take the values of kp, ki and kd
    PID_control(float kp, float ki , float kd)
    {
      this->kd = kd;
      this->ki = ki;
      this->kp = kp;
      read_speed = nh.subscribe("/speed", 10, &PID_control::filter_speed, this);
      cmd_vel_sub = nh.subscribe("/cmd_vel", 10, &PID_control::cmd_vel_callback, this);
      last_time_PID = ros::Time::now();
    }

    void cmd_vel_callback(const std_msgs::Float64::ConstPtr& msg)
    {
      set_point = msg->data; 
    }

    // function used to filter the motor speed
    void filter_speed(const std_msgs::Float64::ConstPtr& msg)
    {
      double speed = msg->data; 
      filtered_speed = (smoothing_factor * last_filtered_speed) + ((1 - smoothing_factor) * speed); // filtiring the speed
      last_filtered_speed = filtered_speed;
    }

    int compute_PID()
    {
      error = set_point - filtered_speed;
      p = kp * error;
  
      ros::Time current_time_PID = ros::Time::now();
      float time = (current_time_PID - last_time_PID).toSec();


      error_sum += error * time;
      i = ki * error_sum;

      float der = (error - last_error) / time;
      d = kd * der;

      float pid_output = p + i + d;
      last_error = error;
      last_time_PID = current_time_PID;
      return  std::clamp(static_cast<int>(pid_output), 0, 255);
    }
};



