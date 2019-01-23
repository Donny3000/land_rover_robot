
#include <DualVNH5019MotorShield.h>
#define USE_USBCON
#include <ros.h>
#include <geometry_msgs/Twist.h>

#define SPEED_MAX 400

DualVNH5019MotorShield md;
ros::NodeHandle nh;

geometry_msgs::Twist resp;
ros::Publisher cmd_vel_resp("auto_rover/cmd_vel_resp", &resp);

// ROS Subscriber to read joystick command
void joystickCb(const geometry_msgs::Twist& vel)
{
    // The the linear portion for the motor speed
  md.setM1Speed(vel.linear.x);

  resp.linear.x = vel.linear.x;
  resp.angular.z = vel.angular.z;
}
ros::Subscriber<geometry_msgs::Twist> sub("auto_rover/cmd_vel", &joystickCb);

void setup()
{
  // Setup the LED for diagnostics
  pinMode(LED_BUILTIN, OUTPUT);

  // Setup Motor Controller
  md.init();

  // Setup ROS
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(cmd_vel_resp);
}

void loop()
{
  nh.spinOnce();
  delay(0.01);

  if (!nh.connected())
  {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
  }
}
