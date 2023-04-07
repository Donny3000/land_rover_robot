#ifndef PID_H
#define PID_H

#include <ros/duration.h>
#include <control_toolbox/SetPidGains.h>

namespace auto_rover
{
    class PID
    {
    public:
        PID(float min_val, float max_val, float kp, float ki, float kd);
        double compute(float setpoint, float measured_value, double delta);
        void updateConstants(float kp, float ki, float kd);

        inline float p_gain() { return kp_; }
        inline float i_gain() { return ki_; }
        inline float d_gain() { return kd_; }

        inline float min_output() { return min_val_; }
        inline float max_output() { return max_val_; }

        inline double error()  { return error_;  }
        inline double p_term() { return p_term_; }
        inline double i_term() { return i_term_; }
        inline double d_term() { return d_term_; }
        inline double prev_error() { return prev_error_; }
        inline double output() { return output_; }

    private:
        float min_val_;
        float max_val_;
        float kp_;
        float ki_;
        float kd_;
        double p_term_;
        double i_term_;
        double d_term_;
        double prev_error_;
        double error_;
        double p_error_;
        double i_error_;
        double d_error_;
        double output_;
    };
}

#endif // PID_H