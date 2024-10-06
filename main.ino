#include <ros.h>
#include <std_msgs/Float32.h>
#include "InterruptEncoder.h"

ros::NodeHandle nh;
std_msgs::Float32 speed_msg;
ros::Publisher speed_pub("/speed", &speed_msg);
InterruptEncoder interruptEncoder(PA0, PA1);
TimedEncoder timedEncoder(PA0, PA1, 100); // 100 ms interval for speed calculation

void setup() {
  nh.initNode();
  nh.advertise(speed_pub);
  interruptEncoder.init();
  timedEncoder.init();
}

void loop() {
  timedEncoder.updatePosition();
  float speed = interruptEncoder.getSpeed();
  // float speed = timedEncoder.getSpeed();

  speed_msg.data = speed;
  speed_pub.publish(&speed_msg);

  nh.spinOnce();

  delay(100);
}
