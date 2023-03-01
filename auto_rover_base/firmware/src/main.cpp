#include <ros.h>
#include <Arduino.h>
#include <PWMServo.h>
#include <Encoder.h>
#include <DualVNH5019MotorShield.h>

#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <auto_rover_speed_controller/HouseKeeping.h>

/* Definitions ----------------------------------------------------------->>>*/
#define CTRL_LOOP_PERIOD   1000 // microseconds
#define SPEED_MAX          400
#define WHEELBASE          5.125 // Inches from center of axle
#define LEFT_ENCODER_A     3
#define LEFT_ENCODER_B     5
#define RIGHT_ENCODER_A    11
#define RIGHT_ENCODER_B    13
//#define THROTTLE_SERVO     3
//#define STEERING_SERVO     5
//#define AUX_SERVO        
#define SERVO_OFFSET       90
#define BOUND_SPEED(s)   { max<int16_t, int16_t>(-SPEED_MAX, min<int16_t, int16_t>(SPEED_MAX, s)) }
/*------------------------------------------------------------------------<<<*/

/* Globals --------------------------------------------------------------->>>*/
/*------------------------------------------------------------------------<<<*/

/* ROS Variables ---------------------------------------------------------<<<*/
ros::NodeHandle nh_;
auto_rover_speed_controller::HouseKeeping hk_;
ros::Publisher hk_pub_("motor_hk", &hk_);
/*------------------------------------------------------------------------>>>*/

/* Third-Party Library Instantiations ------------------------------------>>>*/
// Create a servo object for steering and throttle servos
PWMServo servo_throt_, servo_steer_, servo_aux_;

// Left/Right motor encoders
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder          left_enc_ (LEFT_ENCODER_A , LEFT_ENCODER_B);
Encoder          right_enc_(RIGHT_ENCODER_A, RIGHT_ENCODER_B);

DualVNH5019MotorShield md_(2, 7, 9, 6, A0, 2, 7, 9, 12, A1);

// RC Receiver PWM read Setup
volatile uint16_t ch_throttle_, ch_steering_, ch_aux_;
/*---------------------------------------------------------------------------*/

/* ROS Subscribers -------------------------------------------------------<<<*/
void commandVelocityCb(const geometry_msgs::Twist& msg)
{
  int16_t cmd_speed   = BOUND_SPEED(static_cast<int16_t>(msg.linear.x));
  int16_t delta_speed = static_cast<int16_t>(WHEELBASE * tanf(msg.angular.z));
  
  // Turning left
  if (msg.angular.z > 0)
  {
    md_.setM1Speed(cmd_speed);
    md_.setM2Speed(BOUND_SPEED(cmd_speed - delta_speed));
  }
  // Turning right
  else
  {
    md_.setM2Speed(cmd_speed);
    md_.setM1Speed(BOUND_SPEED(cmd_speed - delta_speed));
  }
}
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &commandVelocityCb);
/*------------------------------------------------------------------------>>>*/

void update_hk()
{
  hk_.left_motor_current_  = md_.getM1CurrentMilliamps();
  hk_.right_motor_current_ = md_.getM2CurrentMilliamps();
  hk_.left_motor_fault_    = md_.getM1Fault();
  hk_.right_motor_fault_   = md_.getM2Fault();
  hk_.left_motor_pos_      = left_enc_.read();
  hk_.right_motor_pos_     = right_enc_.read();
}

void setup()
{
  // Initialize ROS rosserial node
  nh_.initNode();

  // Advertise our housekeeping publisher
  nh_.advertise(hk_pub_);

  // Setup Motor Controller
  md_.init();
}

void loop()
{
  static uint32_t prevMicro = 0, currMicro;

  // Keep the control loop running at 1kHz
  currMicro = micros();
  if ((currMicro - prevMicro) >= CTRL_LOOP_PERIOD)
  {
    prevMicro = currMicro;

    // Publish the house keeping packet
    update_hk();
    hk_pub_.publish(&hk_);

    nh_.spinOnce();
  }
}