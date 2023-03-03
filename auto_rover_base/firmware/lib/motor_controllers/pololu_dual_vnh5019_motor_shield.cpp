#include "pololu_dual_vnh5019_motor_shield.h"

#define BOUND_SPEED(s) (max<int, int>(-POLOLU_VNH5019_SPEED_MAX , min<int, int>(POLOLU_VNH5019_SPEED_MAX , s)))
#define BOUND_BRAKE(s) (max<int, int>(0                         , min<int, int>(POLOLU_VNH5019_SPEED_MAX , s)))

namespace auto_rover_base
{
    PololuDualVNH5019MotorController::PololuDualVNH5019MotorController()
    {
        motor_driver_ = DualVNH5019MotorShield();
        motor_driver_.init();
    }

    void PololuDualVNH5019MotorController::setMotorLeftSpeed(int16_t value)
    {
        motor_driver_.setM1Speed(BOUND_SPEED(value));
    }

    void PololuDualVNH5019MotorController::setMotorRightSpeed(int16_t value)
    {
        motor_driver_.setM2Speed(BOUND_SPEED(value));
    }

    void PololuDualVNH5019MotorController::setMotorLeftBrake(uint16_t brake)
    {
        motor_driver_.setM1Brake(BOUND_BRAKE(brake));
    }

    void PololuDualVNH5019MotorController::setMotorRightBrake(uint16_t brake)
    {
        motor_driver_.setM2Brake(BOUND_BRAKE(brake));
    }

    void PololuDualVNH5019MotorController::setMotorSpeeds(int16_t left, int16_t right)
    {
        motor_driver_.setSpeeds(
            BOUND_SPEED(left),
            BOUND_SPEED(right)
        );
    }

    void PololuDualVNH5019MotorController::setMotorBrakes(uint16_t left, uint16_t right)
    {
        motor_driver_.setBrakes(
            BOUND_BRAKE(left),
            BOUND_BRAKE(right)
        );
    }

    uint16_t PololuDualVNH5019MotorController::getM1CurrentMilliamps()
    {
        return motor_driver_.getM1CurrentMilliamps();
    }

    uint16_t PololuDualVNH5019MotorController::getM2CurrentMilliamps()
    {
        return motor_driver_.getM2CurrentMilliamps();
    }

    uint8_t PololuDualVNH5019MotorController::getM1Fault()
    {
        return motor_driver_.getM1Fault();
    }

    uint8_t PololuDualVNH5019MotorController::getM2Fault()
    {
        return motor_driver_.getM2Fault();
    }
}