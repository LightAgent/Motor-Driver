#define interrupt_encoder 1 
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#if interrupt_ecnoder
#include "InterruptEncoder.h"
#else
#include "TimedEncoder.h"
#endif

#include <algorithm>
#include "math.h"

float set_point;

cmd_vel_sub = nh.subscribe("/cmd_vel", 10, &PID_control::cmd_vel_callback, this);
float minimum_error = 0.01;
float max_integral = 100.0; 
float min_integral = -100.0;
class PID_control
{
private:
    float kp, ki, kd;
    float smoothing_factor = 0.5, last_filtered_speed = 0, filtered_speed = 0;
    unsigned long last_time_PID;
    float error = 0, last_error = 0, error_sum = 0, p = 0, i = 0, d = 0;
    ros::NodeHandle nh;
    ros::Subscriber cmd_vel_sub;

public:
    // constructor function to take the values of kp, ki and kd
    PID_control(float kp, float ki, float kd)
    {
        this->kd = kd;
        this->ki = ki;
        this->kp = kp;
        this->speed = speed;
        last_time_PID = millis();
    }

    void cmd_vel_callback(const std_msgs::Float64::ConstPtr &msg)
    {
        set_point = msg->data;
    }

    // function used to filter the motor speed
    /*void filter_speed(const std_msgs::Float64::ConstPtr &msg)
    {
        double speed = msg->data;
        filtered_speed = (smoothing_factor * last_filtered_speed) + ((1 - smoothing_factor) * speed); // filtiring the speed
        last_filtered_speed = filtered_speed;
    }*/

    int compute_PID()
    {
        error = set_point - speed;

        p = kp * error;

        unsigned long current_time_PID = millis();
        float time = (current_time_PID - last_time_PID) / 1000;

        error_sum += error * time;
        i = ki * error_sum;
        i = std::clamp(i, min_integral, max_integral);

        float der = (error - last_error) / time;
        d = kd * der;

        if(fabs(error < minimum_error))
        {
            error = 0;

            p = 0;
            i = 0;
            d = 0;
            
        }
        
        p = kp * error;

        

        float pid_output = p + i + d;
        last_error = error;
        last_time_PID = current_time_PID;
        return std::clamp(static_cast<int>(pid_output), 0, 255); 
    }
};