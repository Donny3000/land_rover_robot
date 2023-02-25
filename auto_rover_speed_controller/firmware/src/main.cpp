#include <Arduino.h>
#include <PWMServo.h>
#include <DualVNH5019MotorShield.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>

/* Definitions ----------------------------------------------------------->>>*/
#define SPEED_MAX          400
#define WHEELBASE          5.125 // Inches from center of axle
#define REAR_LEFT_ENCODER  18
#define REAR_RIGHT_ENCODER 19
#define THROTTLE_SERVO     3
#define STEERING_SERVO     5
#define AUX_SERVO        
#define SERVO_OFFSET       90
#define BOUND_SPEED(s)   { max(-SPEED_MAX, min(SPEED_MAX, s)) }
#define RCCHECK(fn)      { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn)  { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
/*------------------------------------------------------------------------<<<*/

/* Function Prototypes ---------------------------------------------------<<<*/
void commandVelocityCb(const void * cmd_vel);
/*------------------------------------------------------------------------>>>*/

// Create a servo object for steering and throttle servos
PWMServo servo_throt_, servo_steer_, servo_aux_;
DualVNH5019MotorShield md_(2, 7, 9, 6, A0, 2, 7, 9, 12, A1);

// RC Receiver PWM read Setup
volatile uint16_t ch_throttle_, ch_steering_, ch_aux_;

// Setup the diagnostic publisher and and command twist subscriber
rcl_publisher_t             diagnostic_pub_;
std_msgs__msg__Int32        diagnostic_msg_;
rcl_subscription_t          command_velocity_sub_;
geometry_msgs__msg__Twist   command_velocity_msg_;

rclc_executor_t             executor_;
rclc_support_t              support_;
rcl_allocator_t             allocator_;
rcl_node_t                  node_;
rcl_timer_t                 timer_;

void error_loop(){
  while(1){
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&diagnostic_pub_, &diagnostic_msg_, NULL));
    diagnostic_msg_.data++;
  }
}

void setup() {
  // Configure serial transport
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  
  delay(2000);

  // Setup Motor Controller
  md_.init();

  allocator_ = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support_, 0, NULL, &allocator_));

  // create node
  RCCHECK(rclc_node_init_default(&node_, "micro_ros_arduino_node", "", &support_));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &diagnostic_pub_,
    &node_,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "diagnostics")
  );
  
  // Create subscriber
  RCCHECK(rclc_subscription_init_default(
    &command_velocity_sub_,
    &node_,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "command_velocity")
  );

  // create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer_,
    &support_,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback)
  );

  // create executor
  RCCHECK(rclc_executor_init            (&executor_, &support_.context, 1, &allocator_));
  RCCHECK(rclc_executor_add_timer       (&executor_, &timer_));
  RCCHECK(rclc_executor_add_subscription(&executor_, &command_velocity_sub_, &command_velocity_msg_, &commandVelocityCb, ON_NEW_DATA));

  diagnostic_msg_.data          = 0;
  command_velocity_msg_.linear.x = command_velocity_msg_.angular.x = 0;
  command_velocity_msg_.linear.y = command_velocity_msg_.angular.y = 0;
  command_velocity_msg_.linear.z = command_velocity_msg_.angular.z = 0;
}

void loop()
{
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(100)));
}

// ROS Subscriber to read joystick command
void commandVelocityCb(const void * msg)
{
  const geometry_msgs__msg__Twist * cmd_vel = \
    static_cast<const geometry_msgs__msg__Twist *>(msg);
  int16_t cmd_speed   = BOUND_SPEED(static_cast<int16_t>(cmd_vel->linear.x));
  int16_t delta_speed = static_cast<int16_t>(WHEELBASE * tanf(cmd_vel->angular.z));
  
  // Turning left
  if (cmd_vel->angular.z > 0)
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