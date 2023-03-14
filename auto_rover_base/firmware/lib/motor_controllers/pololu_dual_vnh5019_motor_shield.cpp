#include "pololu_dual_vnh5019_motor_shield.h"

namespace auto_rover
{
    PololuDualVNH5019MotorController::PololuDualVNH5019MotorController()
    {
        motor_driver_ = DualVNH5019MotorShield();
        motor_driver_.init();
    }

    void PololuDualVNH5019MotorController::setMotorLeftSpeed(int16_t value)
    {
        motor_driver_.setM1Speed(value);
    }

    void PololuDualVNH5019MotorController::setMotorRightSpeed(int16_t value)
    {
        motor_driver_.setM2Speed(value);
    }

    void PololuDualVNH5019MotorController::setMotorLeftBrake(uint16_t brake)
    {
        motor_driver_.setM1Brake(brake);
    }

    void PololuDualVNH5019MotorController::setMotorRightBrake(uint16_t brake)
    {
        motor_driver_.setM2Brake(brake);
    }

    void PololuDualVNH5019MotorController::setMotorSpeeds(int16_t left, int16_t right)
    {
        motor_driver_.setSpeeds(left, right);
    }

    void PololuDualVNH5019MotorController::setMotorBrakes(uint16_t left, uint16_t right)
    {
        motor_driver_.setBrakes(left, right);
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