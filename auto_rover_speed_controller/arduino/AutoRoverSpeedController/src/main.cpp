
#include <DualVNH5019MotorShield.h>
#define USE_USBCON
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <RedBot.h>

#define SPEED_MAX    400
#define CNTS_PER_REV 100

// Configure library with pins as remapped for single-channel operation
// this lets the single motor be controlled as if it were "motor 1"
DualVNH5019MotorShield md(2, 7, 9, 6, A0, 2, 7, 9, 12, A1);
RedBotEncoder encoders(A3, A4);
uint32_t front_cnt;
uint32_t rear_cnt;

ros::NodeHandle nh;

geometry_msgs::Twist resp;
ros::Publisher cmd_vel_resp("auto_rover/cmd_vel_resp", &resp);

// ROS Subscriber to read joystick command
void joystickCb(const geometry_msgs::Twist& vel)
{
    // The the linear portion for the motor speed
  md.setM1Speed(static_cast<int16_t>(vel.linear.x));

  resp.linear.x = vel.linear.x;
  resp.angular.z = vel.angular.z;
}
ros::Subscriber<geometry_msgs::Twist> sub("auto_rover/cmd_vel", &joystickCb);

void setup()
{
  // Setup the LED for diagnostics
  pinMode(LED_BUILTIN, OUTPUT);

  // Clear both motor encoders
  encoders.clearEnc(BOTH);

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
