
#include <DualVNH5019MotorShield.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

#define SPEED_MAX 400

DualVNH5019MotorShield md;
ros::NodeHandle nh;

// ROS Subscriber to read joystick command
void joystickCb(const geometry_msgs::Twist& vel)
{
  // The the linear portion for the motor speed
  md.setM1Speed(floorf(SPEED_MAX * vel.linear.x));
}
ros::Subscriber<geometry_msgs::Twist> sub("auto_rover/cmd_vel", &joystickCb);

void setup()
{
  // Setup Motor Controller
  md.init();

  // Setup ROS
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{
  nh.spinOnce();
  delay(0.1);
}