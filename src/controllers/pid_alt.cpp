#include "uav_agent/controllers/pid_alt.h"

void PIDInit(PIDControl *pid)
{
    pid->integrator = 0;
    pid->prev_error = 0;

    pid -> differentiator = 0;
    pid->prev_value = 0;

    pid->output = 0;
}

float PIDUpdate(PIDControl *pid, float setpoint, float measurement)
{
    // error signal
    float error = setpoint - measurement;

    // Proportional
    float proportional = pid->kp * error;

    // integral
    pid->integrator = pid->integrator + 0.5 * pid->Ki * pid->dt * (error + pid->prev_error);

    // INtegrator clamping 
    if(pid->integrator > pid->max_limit)
    {
        pid->integrator = pid->max_limit;
    }
    else if(pid->integrator < pid->min_limit)
    {
         pid->integrator =pid->min_limit;
    }

    // derivative (band-limited differentiator)

    pid->differentiator = -(2 * pid->Kd * (measurement - prev_value)
                          + (2 * pid->tau - pid->dt) * pid->differentiator)
                          / (2 * pid->tau + pid->dt);
    
    //compute output and clamp
    pid->output = proportional + pid->integrator + pid->differentiator;

    if(pid->output > pid->max_limit)
    {
        pid->output = pid->max_limit;
    }     
    else if(pid->output < pid->min_limit)
    {
        pid->output = pid->min_limit;
    }

    pid->prev_error = error;
    pid->prev_value = measurement;
    return pid->output;
}
