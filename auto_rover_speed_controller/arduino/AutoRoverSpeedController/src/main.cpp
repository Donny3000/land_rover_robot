#include <Servo.h>
#include <Wire.h>
#include <DualVNH5019MotorShield.h>

//#define USE_USBCON
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <auto_rover_speed_controller/Encoders.h>

#define SPEED_MAX          400
#define REAR_LEFT_ENCODER  19
#define REAR_RIGHT_ENCODER 18
#define FRONT_WHEEL_SERVO  3
#define REAR_WHEEL_SERVO   11
#define SERVO_OFFSET       90

unsigned long start_time_;

// Configure library with pins as remapped for single-channel operation
// this lets the single motor be controlled as if it were "motor 1"
DualVNH5019MotorShield md(2, 7, 9, 6, A0, 2, 7, 9, 12, A1);

// Setup the servo for front_wheels_
Servo front_wheels_;
Servo rear_wheels_;

ros::NodeHandle nh;

// Wheel Encoder Setup
volatile uint16_t rear_left_ticks_, rear_right_ticks_;
volatile uint16_t front_left_ticks_, front_right_ticks_;
auto_rover_speed_controller::Encoders encoder_;
ros::Publisher encoder_pub_("auto_rover/encoders", &encoder_);

void isr_rear_left_encoder()
{
  rear_left_ticks_++;
}

void isr_rear_right_encoder()
{
  rear_right_ticks_++;
}

// ROS Subscriber to read joystick command
void joystickCb(const geometry_msgs::Twist& vel)
{
  static int16_t prev_speed = 0, prev_steer = 0;

  int16_t cur_speed = static_cast<int16_t>(vel.linear.x);
  int16_t cur_steer = static_cast<int16_t>(vel.angular.z) + SERVO_OFFSET;

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

  // Initialize the encoder ticks
  rear_left_ticks_ = rear_right_ticks_ = 0;
  front_left_ticks_ = front_right_ticks_ = 0;
  encoder_.rear_left_ = encoder_.rear_right_ = 0;
  encoder_.front_left_ = encoder_.front_right_ = 0;

  // Setup the servo
  front_wheels_.attach(FRONT_WHEEL_SERVO);
  rear_wheels_.attach(REAR_WHEEL_SERVO);

  // Clear both motor encoders
  pinMode(REAR_LEFT_ENCODER, INPUT_PULLUP);
  pinMode(REAR_RIGHT_ENCODER, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(REAR_LEFT_ENCODER), isr_rear_left_encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(REAR_RIGHT_ENCODER), isr_rear_right_encoder, CHANGE);

  // Setup Motor Controller
  md.init();

  // Setup ROS
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(encoder_pub_);
}

void loop()
{
  start_time_ = millis();
  delay(10);

  if (!nh.connected())
  {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
  }
  else
  {
    encoder_.millis_ = millis() - start_time_;
    encoder_.rear_left_ = rear_left_ticks_;
    encoder_.rear_right_ = rear_right_ticks_;

    encoder_pub_.publish( &encoder_ );
  }

  nh.spinOnce();
}
