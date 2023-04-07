#include "Arduino.h"
#include "pid.h"
#include "auto_rover_base_config.h"

namespace auto_rover
{
    PID::PID(float min_val, float max_val, float kp, float ki, float kd):
    min_val_(min_val),
    max_val_(max_val),
    kp_(kp),
    ki_(ki),
    kd_(kd),
    p_term_(0.0),
    i_term_(0.0),
    d_term_(0.0),
    prev_error_(0.0),
    error_(0.0),
    p_error_(0.0),
    i_error_(0.0),
    d_error_(0.0),
    output_(0.0)
    {
    }

    double PID::compute(float setpoint, float measured_value, double delta)
    {
        // Setpoint is constrained between min and max to prevent pid from having too much error
        error_   = setpoint - measured_value;
        d_error_ = error_   - prev_error_;

        // Positional error is the target - state
        p_error_ = error_;
        p_term_ = kp_ * p_error_;
        
        // Calculate the integral of the position error
        i_error_ += p_error_ * delta;
        if (ki_ > 0 && ANTIWINDUP)
        {
            constrain(i_error_, I_MIN / ki_, I_MAX / ki_);
        }

        // Calculate the integral contribution to output
        i_term_ += ki_ * i_error_;

        if (!ANTIWINDUP)
        {
            // Limit the I term so the output is meaningful
            constrain(i_term_, I_MIN, I_MAX);
        }

        // Calculate the derivative term
        d_term_ = kd_ * d_error_;

        if(setpoint == 0 && error_ == 0)
        {
            i_term_ = 0;
        }

        output_ = p_term_ + i_term_ + d_term_;

        prev_error_ = error_;

        return constrain(output_, min_val_, max_val_);
    }

    void PID::updateConstants(float kp, float ki, float kd)
    {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }
}