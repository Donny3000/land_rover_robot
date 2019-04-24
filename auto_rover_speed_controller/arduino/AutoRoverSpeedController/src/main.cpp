#include <Servo.h>
#include <Wire.h>
#include <DualVNH5019MotorShield.h>
#include "DFRobot_BNO055.h"

//#define USE_USBCON
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <auto_rover_speed_controller/Encoders.h>

#define SPEED_MAX         400
#define ENCODER_CH_A      19
#define ENCODER_CH_B      18
#define FRONT_WHEEL_SERVO 3
#define REAR_WHEEL_SERVO  11
#define SERVO_OFFSET      90

typedef DFRobot_BNO055_IIC BNO; // ******** use abbreviations instead of full names ********

// Configure library with pins as remapped for single-channel operation
// this lets the single motor be controlled as if it were "motor 1"
DualVNH5019MotorShield md(2, 7, 9, 6, A0, 2, 7, 9, 12, A1);

// Setup the DFRobot IMU
BNO bno_(&Wire, 0x28);

// Setup the servo for front_wheels_
Servo front_wheels_;
Servo rear_wheels_;

ros::NodeHandle nh;

// Wheel Encoder Setup
volatile bool fired_;
volatile bool up_;
auto_rover_speed_controller::Encoders encoder_;
ros::Publisher encoder_pub_("auto_rover/encoders", &encoder_);

// Setup IMU publisher
sensor_msgs::Imu imu_;
BNO::sAxisAnalog_t sGyrAnalog, sLiaAnalog, sGrvAnalog;
BNO::sQuaAnalog_t  sQuaAnalog;
ros::Publisher imu_pub_("auto_rover/imu/data", &imu_);

void isr()
{
  if (digitalRead(ENCODER_CH_A))
    up_ = digitalRead(ENCODER_CH_B);
  else
    up_ = !digitalRead(ENCODER_CH_B);

  fired_ = true;
}

int16_t prev_speed, cur_speed, prev_steer, cur_steer;
// ROS Subscriber to read joystick command
void joystickCb(const geometry_msgs::Twist& vel)
{
  cur_speed = static_cast<int16_t>(vel.linear.x);
  cur_steer = static_cast<int16_t>(vel.angular.z) + SERVO_OFFSET;

  // The the linear portion for the motor speed
  if (cur_speed != prev_speed)
  {
    md.setM1Speed(cur_speed);
    prev_speed = cur_speed;
  }

  if (cur_steer != prev_steer)
  {
    front_wheels_.write(cur_steer);
    rear_wheels_.write(-cur_steer);
    prev_steer = cur_steer;
  }
}
ros::Subscriber<geometry_msgs::Twist> sub("auto_rover/cmd_vel", &joystickCb);

void setup()
{
  // Setup the LED for diagnostics
  pinMode(LED_BUILTIN, OUTPUT);

  bno_.reset();
  while (bno_.begin() != BNO::eStatusOK)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }
//   mpu.setMode(mpu.eNORMAL_POWER_MODE, mpu.eFASTEST_MODE);

  // Setup the servo
  front_wheels_.attach(FRONT_WHEEL_SERVO);
  rear_wheels_.attach(REAR_WHEEL_SERVO);

  // Clear both motor encoders
  pinMode(ENCODER_CH_A, INPUT_PULLUP);
  pinMode(ENCODER_CH_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_CH_A), isr, CHANGE);

  // Setup Motor Controller
  md.init();

  // Setup ROS
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(encoder_pub_);
}

void loop()
{
  nh.spinOnce();
  delay(10);

  if (!nh.connected())
  {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
  }
  else if (fired_)
  {
    if (up_)
      encoder_.wheel_ticks++;
    else
      encoder_.wheel_ticks--;

    encoder_pub_.publish( &encoder_ );

    fired_ = false;
  }
}
