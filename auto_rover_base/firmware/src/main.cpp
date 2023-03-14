#include <ros.h>
#//include <PWMServo.h>
#include "auto_rover_base_config.h"
#include "auto_rover_base_controller.h"
#include "pololu_dual_vnh5019_motor_shield.h"

using namespace auto_rover;

/* Definitions ----------------------------------------------------------->>>*/
/*------------------------------------------------------------------------<<<*/

/* Globals --------------------------------------------------------------->>>*/
/*------------------------------------------------------------------------<<<*/

/* ROS Variables ---------------------------------------------------------<<<*/
ros::NodeHandle nh_;
/*------------------------------------------------------------------------>>>*/

/* Third-Party Library Instantiations ------------------------------------>>>*/
PololuDualVNH5019MotorController mc_ = PololuDualVNH5019MotorController();
BaseController<PololuDualVNH5019MotorController, DualVNH5019MotorShield> bc_(nh_, &mc_);
// Create a servo object for steering and throttle servos
// PWMServo servo_throt_, servo_steer_, servo_aux_;

// RC Receiver PWM read Setup
//volatile uint16_t ch_throttle_, ch_steering_, ch_aux_;
/*---------------------------------------------------------------------------*/

void setup()
{
    // Read the base controller's parameters from the ROS parameter server,
    // initialize the ROS node handle and setup publishers/subscribers.
    bc_.setup();
    bc_.init();
    nh_.loginfo("auto_rover_base controller setup/initialization complete");
}

void loop()
{
    //static bool is_imu_initialized = false;

    // Here is the main control loop for the base controller. This block drives
    // the robot based on a defined control rate.
    double command_dt = nh_.now().toSec() - bc_.lastUpdateTime().control.toSec();
    if (command_dt >= bc_.publishRate().period().control_)
    {
        bc_.read();
        bc_.write();
        bc_.lastUpdateTime().control = nh_.now();
    }

    // This block stops the motors when no wheel command is received from the
    // high-level hardware_interface::RobotHW.
    command_dt = nh_.now().toSec() - bc_.lastUpdateTime().command_received.toSec();
    if (command_dt >= ros::Duration(E_STOP_COMMAND_RECEIVED_DURATION, 0).toSec())
    {
        nh_.logwarn("Emergency STOP");
        bc_.eStop();
    }

    // This block publishes IMU data based on a defined IMU rate
    // double imu_dt = nh_.now().toSec() - bc_.lastUpdateTime().imu.toSec();
    // if (imu_dt >= bc_.publishRate().period().imu_)
    // {
    //     // Sanity check if the IMU is connected
    //     if (!is_imu_initialized)
    //     {
    //         if (is_imu_initialized)
    //         {
    //             nh_.loginfo("IMU Initialized");
    //         }
    //         else
    //         {
    //             nh_.logfatal("IMU failed to initialize. Check you IMU connection.");
    //         }
    //     }

    //     bc_.lastUpdateTime().imu = nh_.now();
    // }

    // This block displays the encoder readings. Change DEBUG to 0 if you
    // don't want to display.
    if (bc_.debug())
    {
        double debug_dt = nh_.now().toSec() - bc_.lastUpdateTime().debug.toSec();
        if (debug_dt >= bc_.publishRate().period().debug_)
        {
            bc_.printDebug();
            bc_.lastUpdateTime().debug = nh_.now();
        }
    }

    // Call all the callbacks waiting to be called
    nh_.spinOnce();
    delay(5);
}