#include <Servo.h>
#include <DualVNH5019MotorShield.h>
//#define USE_USBCON
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <auto_rover_speed_controller/Encoders.h>

#define SPEED_MAX    400
#define ENCODER_CH_A 18
#define ENCODER_CH_B 19
#define SERVO_PIN    3
#define SERVO_OFFSET 90

// Configure library with pins as remapped for single-channel operation
// this lets the single motor be controlled as if it were "motor 1"
DualVNH5019MotorShield md(2, 7, 9, 6, A0, 2, 7, 9, 12, A1);

// Setup the servo for steering
Servo steering;

ros::NodeHandle nh;

// Wheel Encoder Setup
volatile bool fired_;
volatile bool up_;
auto_rover_speed_controller::Encoders encoder_;
ros::Publisher encoder_pub_("auto_rover/encoders", &encoder_);

void isr()
{
  if (digitalRead(ENCODER_CH_A))
    up_ = digitalRead(ENCODER_CH_B);
  else
    up_ = !digitalRead(ENCODER_CH_B);

  fired_ = true;
}

// ROS Subscriber to read joystick command
void joystickCb(const geometry_msgs::Twist& vel)
{
  // The the linear portion for the motor speed
  md.setM1Speed(static_cast<int16_t>(vel.linear.x));
  steering.write(static_cast<int16_t>(vel.angular.z) + SERVO_OFFSET);
}
ros::Subscriber<geometry_msgs::Twist> sub("auto_rover/cmd_vel", &joystickCb);

void setup()
{
  // Setup the LED for diagnostics
  pinMode(LED_BUILTIN, OUTPUT);

  // Setup the servo
  steering.attach(SERVO_PIN);

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
  delay(0.01);

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
