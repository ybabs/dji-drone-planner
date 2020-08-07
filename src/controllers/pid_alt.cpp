#include "uav_agent/controllers/pid_alt.h"

void PIDController::PIDInit(float kp, float ki, float kd, float max, float min)
{
    pid_.Kp = kp;
    pid_.Ki = ki;
    pid_.Kd = kd;
    pid_.tau = 0.02;
    pid_.max_limit = max;
    pid_.min_limit = min;
    pid_.int_max_limit = max/2;
    pid_.int_min_limit = min/2;
    pid_.integrator = 0;
    pid_.prev_error = 0;

    pid_.differentiator = 0;
    pid_.prev_value = 0;

    pid_.output = 0;
}

float PIDController::PIDUpdate(float setpoint, float measurement, float dt)
{
    // error signal
    float error = setpoint - measurement;

    // Proportional
    float proportional = pid_.Kp * error;

    // integral
    pid_.integrator = pid_.integrator + 0.5 * pid_.Ki * pid_.dt * (error + pid_.prev_error);

    // INtegrator clamping 
    if(pid_.integrator > pid_.int_max_limit)
    {
        pid_.integrator = pid_.int_max_limit;
    }
    else if(pid_.integrator < pid_.int_min_limit)
    {
         pid_.integrator =pid_.int_min_limit;
    }

    // derivative (band-limited differentiator)

    pid_.differentiator = -(2 * pid_.Kd * (measurement - pid_.prev_value)
                          + (2 * pid_.tau - pid_.dt) * pid_.differentiator)
                          / (2 * pid_.tau + pid_.dt);
    
    //compute output and clamp
    pid_.output = proportional + pid_.integrator + pid_.differentiator;

    if(pid_.output > pid_.max_limit)
    {
        pid_.output = pid_.max_limit;
    }     
    else if(pid_.output < pid_.min_limit)
    {
        pid_.output = pid_.min_limit;
    }

    pid_.prev_error = error;
    pid_.prev_value = measurement;
    return pid_.output;
}
