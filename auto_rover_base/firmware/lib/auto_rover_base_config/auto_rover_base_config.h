#ifndef AUTO_ROVER_BASE_CONFIG_H
#define AUTO_ROVER_BASE_CONFIG_H

#define NUM_OF_JOINTS                    2
#define E_STOP_COMMAND_RECEIVED_DURATION 5  // Stop motors if no command was received after this amount of seconds

// Encoder Pins
#define ENCODER_LEFT_A                   3
#define ENCODER_LEFT_B                   5
#define ENCODER_RIGHT_A                  11
#define ENCODER_RIGHT_B                  13

// Pololu Motor Controller
#define CTRL_LOOP_PERIOD                 1000 // microseconds
#define POLOLU_VNH5019_SPEED_MAX         400
#define POLOLU_VNH5019_BRAKE_MAX         400
#define WHEELBASE                        5.125 // Inches from center of axle

// Encoder resolution used for initialization 
// will be read from parameter server
#define ENCODER_RESOLUTION               2249

/// DFRobot IMU i2c address
#define IMU_DRIVER_ADDR                  0x60

#define FF                               0.8 // PID Feed-forward term
#define K_P                              15 //0.6 // P constant
#define K_I                              0.0 //0.3 // I constant
#define K_D                              0.0 //0.5 // D constant
#define PWM_BITS                         16  // PWM Resolution of the microcontroller

#define UPDATE_RATE_CONTROL              100
#define UPDATE_RATE_IMU                  100
#define UPDATE_RATE_DEBUG                10

#define PWM_MAX                          pow(2, PWM_BITS) - 1
#define PWM_MIN                          -(PWM_MAX)

#endif // AUTO_ROVER_BASE_CONFIG_H