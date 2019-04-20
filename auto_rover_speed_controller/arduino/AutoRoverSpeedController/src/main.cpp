
#include <DualVNH5019MotorShield.h>
#define USE_USBCON
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <auto_rover_speed_controller/Encoders.h>

#define SPEED_MAX     400
#define CNTS_PER_REV  100
#define INT_PIN_FRONT 0
#define INT_PIN_REAR  1

// Configure library with pins as remapped for single-channel operation
// this lets the single motor be controlled as if it were "motor 1"
DualVNH5019MotorShield md(2, 7, 9, 6, A0, 2, 7, 9, 12, A1);

ros::NodeHandle nh;

geometry_msgs::Twist resp;
ros::Publisher cmd_vel_resp("auto_rover/cmd_vel_resp", &resp);

// Wheel Encoder Setup
volatile uint32_t front_cnt_;
volatile uint32_t rear_cnt_;
auto_rover_speed_controller::Encoders encoder_cnts;
ros::Publisher encoder_cnts_pub("auto_rover/encoders", &encoder_cnts);
void _front_ticks()
{
  front_cnt_++;
}

void _rear_ticks()
{
  rear_cnt_++;
}

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
  front_cnt_ = 0;
  rear_cnt_ = 0;
  pinMode(INT_PIN_FRONT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INT_PIN_FRONT), _front_ticks, RISING);
  pinMode(INT_PIN_REAR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INT_PIN_REAR), _rear_ticks, RISING);

  // Setup Motor Controller
  md.init();

  // Setup ROS
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(cmd_vel_resp);
  nh.advertise(encoder_cnts_pub);
}

void loop()
{
  if (!nh.connected())
  {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
  }
  else
  {
    encoder_cnts.front_cnt_ = front_cnt_;
    encoder_cnts.rear_cnt_ = rear_cnt_;
    encoder_cnts_pub.publish( &encoder_cnts );
  }

  nh.spinOnce();
  delay(0.01);
}
