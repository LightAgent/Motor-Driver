#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"

class Drive
{
  private:
  ros::NodeHandle nh;
  ros::Publisher wheels_pub;
  PID_control& pid_controller;

  public:
  Drive(PID_control& pid_controller)
  {
    this->pid_controller = pid_controller;
    wheels_pub = nh.advertise<std_msgs::Float64MultiArray>("/wheels_velocities", 10); // topic name will be changed
  }
  void publish_velocities()
  {

    std_msgs::Float64MultiArray velocities;
    velocities.data.resize(4);

    int velocity = pid_controller.compute_PID();
    for (int i = 0 ; i < 4 ; i++)
    {
       velocities.data[i] = velocity;
    }
    wheels_pub.publish(velocities);
  }

}
void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
