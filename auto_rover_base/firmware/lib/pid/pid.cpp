#include "Arduino.h"
#include "pid.h"

namespace auto_rover
{
    PID::PID(float min_val, float max_val, float kp, float ki, float kd):
    min_val_(min_val),
    max_val_(max_val),
    kp_(kp),
    ki_(ki),
    kd_(kd)
    {
    }

    double PID::compute(float setpoint, float measured_value)
    {
        double error;

        //setpoint is constrained between min and max to prevent pid from having too much error
        error = setpoint - measured_value;
        proportional_ = error;
        integral_ += error;
        derivative_ = error - prev_error_;

        if(setpoint == 0 && error == 0)
        {
            integral_ = 0;
        }

        output_ = (kp_ * proportional_) + (ki_ * integral_) + (kd_ * derivative_);
        prev_error_ = error;

        return constrain(output_, min_val_, max_val_);
    }

    void PID::updateConstants(float kp, float ki, float kd)
    {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }
}