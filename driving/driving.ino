#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "pid.h"

#define PWM PA8
#define INA PA0
#define INB PA1 

class Drive
{
  private:
  ros::NodeHandle nh;
  ros::Publisher wheels_pub;
  PID_control& pid_controller;

  public:
    Drive(PID_control& pid_controller) : pid_controller(pid_controller){
  }
  void publish_velocities()
  {
    int pwm = pid_controller.compute_PID();

    digitalWrite(INA, HIGH);
    digitalWrite(INB,LOW);
    analogWrite(PWM, pwm);
  }
PID_control pid_controller(1.0, 0.0, 0.5);
Drive drive_system(pid_controller);

}
void setup() {
  
  pinMode(PWM,OUTPUT);
  pinMode(INA,OUTPUT);
  pinMode(INB,OUTPUT);
  
  ros::init(argc, argv, "motor_controller");

}

void loop() {
    drive_system.publish_velocities();
    delay(100);
}
