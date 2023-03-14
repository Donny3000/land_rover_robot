#ifndef POLOLU_DUAL_VNH5019_MOTOR_SHIELD_H
#define POLOLU_DUAL_VNH5019_MOTOR_SHIELD_H

// The special bit of code in this example is the use of Arduino's Wire library. 
// Wire is a I2C library that simplifies reading and writing to the I2C bus.
#include <Wire.h>
#include <DualVNH5019MotorShield.h>
#include "auto_rover_base_config.h"
#include <motor_controller_interface.h>

namespace auto_rover
{
    /** \brief Implementation of the MotorControllerIntf for the Pololu Dual VNH5019 Motor Shield
     * 
     * Implements the abstract setSpeed method from the MotorControllerIntf
     * 
     * The class makes use of the DualVNH5019MotorShield library.
     * 
     * \note for more details see
     * https://www.pololu.com/docs/0J49
     */
    class PololuDualVNH5019MotorController : public MotorControllerIntf<DualVNH5019MotorShield>
    {
        public:
            /** \brief Construct an \ref PololuDualVNH5019MotorController for two motor
             * 
             * Specify the motor to control \p motor_num (use 3 or 4 for Auto Rover).
             * Specify the i2c address to be used. The default address is 0x60 but it
             * can be changed by soldering different address switches on the bottom of
             * the PCB.
             * 
             * \param motor_num Number of the motor to control (one of 1, 2, 3, 4). Auto Rover uses motors 3 and 4.
             * \param addr i2c address used to communicate with the motor driver.
             */
            PololuDualVNH5019MotorController();

            /** \brief Alternate constructor for shield connections remapped by user.
             * 
             * If PWM1 and PWM2 are remapped, it will try to use analogWrite instead of timer1.
             * 
             * \param INA1
             * \param INB1
             * \param PWM1
             * \param EN1DIAG1
             * \param CS1
             * \param INA2
             * \param INB2
             * \param PWM2
             * \param EN2DIAG2
             * \param CS2
            */
            PololuDualVNH5019MotorController(
                uint8_t INA1, 
                uint8_t INB1,
                uint8_t PWM1,
                uint8_t EN1DIAG1,
                uint8_t CS1,
                uint8_t INA2,
                uint8_t INB2,
                uint8_t PWM2,
                uint8_t EN2DIAG2,
                uint8_t CS2
            );
            
            /** \brief Set speed and direction for motor (left) \ref pMotor_.
             * 
             * \p value should be between -400 and 400. 400 corresponds 
             * to motor current flowing from M1A to M1B. -400 corresponds
             * to motor current flowing from M1B to M1A. 0 corresponds to
             * full coast.
             * 
             * \note for more details see
             * https://github.com/pololu/dual-vnh5019-motor-shield
             * 
             * \param value positive or negative value to set the direction
             * and speed of the motor.
             */
            void setMotorLeftSpeed(int16_t value) override;

            /** \brief Set speed and direction for motor 1 (right) \ref pMotor_.
             * 
             * \p value should be between -400 and 400. 400 corresponds 
             * to motor current flowing from M2A to M2B. -400 corresponds
             * to motor current flowing from M2B to M2A. 0 corresponds to
             * full coast.
             * 
             * \note for more details see
             * https://github.com/pololu/dual-vnh5019-motor-shield
             * 
             * \param value positive or negative value to set the direction
             * and speed of the motor.
             */
            void setMotorRightSpeed(int16_t value) override;

            /** \brief Set brake for motor 1. \ref pMotor_.
             * 
             * Brake should be between 0 and 400. 0 corresponds to full coast,
             * and 400 corresponds to full brake.
             * 
             * \note for more details see
             * https://github.com/pololu/dual-vnh5019-motor-shield
             * 
             * \param brake positive break strength for motor 1.
             */
            void setMotorLeftBrake(uint16_t brake) override;

            /** \brief Set brake for motor 1. \ref pMotor_.
             * 
             * Brake should be between 0 and 400. 0 corresponds to full coast,
             * and 400 corresponds to full brake.
             * 
             * \note for more details see
             * https://github.com/pololu/dual-vnh5019-motor-shield
             * 
             * \param brake positive break strength for motor 2.
             */
            void setMotorRightBrake(uint16_t brake) override;

            /** \brief Set speed and direction of the left and right motors \ref pMotor_.
             * 
             * \note for more details see
             * https://github.com/pololu/dual-vnh5019-motor-shield
             * 
             * \param left positive or negative value to set the direction
             * and speed of motor 1.
             * \param right positive or negative value to set the direction
             * and speed of motor 2.
             */
            void setMotorSpeeds(int16_t left, int16_t right) override;

            /** \brief Set brake for the left and right motor. \ref pMotor_.
             * 
             * \note for more details see
             * https://github.com/pololu/dual-vnh5019-motor-shield
             * 
             * \param m1speed positive value to set the left brake.
             * \param m2speed positive value to set the right brake.
             */
            void setMotorBrakes(uint16_t left, uint16_t right) override;

            /** \brief Returns current reading from motor 1 in milliamps. \ref pMotor_.
             * 
             * See the notes in the "Current readings" section below.
             * 
             * \note for more details see
             * https://github.com/pololu/dual-vnh5019-motor-shield
             */
            uint16_t getM1CurrentMilliamps();

            /** \brief Returns current reading from motor 2 in milliamps. \ref pMotor_.
             * 
             * See the notes in the "Current readings" section below.
             * 
             * \note for more details see
             * https://github.com/pololu/dual-vnh5019-motor-shield
             */
            uint16_t getM2CurrentMilliamps();

            /** \brief Returns 1 if there is a fault on motor driver 1, 0 if no fault. \ref pMotor_.
             * 
             * \note for more details see
             * https://github.com/pololu/dual-vnh5019-motor-shield
             */
            uint8_t getM1Fault();

            /** \brief Returns 1 if there is a fault on motor driver 2, 0 if no fault \ref pMotor_.
             * 
             * \note for more details see
             * https://github.com/pololu/dual-vnh5019-motor-shield
             */
            uint8_t getM2Fault();
    };

}

#endif // POLOLU_DUAL_VNH5019_MOTOR_SHIELD_H