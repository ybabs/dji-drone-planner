
#include <iostream>
#include "uav_agent/controllers/pid.h"

PIDController::PIDController()
{

}

void PIDController::init(float kp, float ki , float kd, float max, float min)
{
    pid_.current_effort = 0.0;
    pid_.target_effort = 0.0;
    pid_.error = 0.0;
    pid_.prev_position = 0.0;
    pid_.prev_error = 0.0;
    pid_.output = 0.0;
    pid_.integral = 0.0;
    pid_.Kp = kp;
    pid_.Ki = ki;
    pid_.Kd = kd;
    pid_.max_effort = max;
    pid_.min_effort = min;
    pid_.sample_time = 0.02;
}

void PIDController::reset()
{
    pid_.integral = 0.0;
    pid_.error = 0.0;
    pid_.prev_error = 0.0;
    pid_.Kp = 0.0;
    pid_.Ki = 0.0;
    pid_.Kd = 0.0;
}

float PIDController::update(float setpoint, float actual, float dt)
{
    pid_.target_effort = setpoint;
    pid_.current_effort = actual;
    pid_.sample_time = dt;
    
    pid_.error = pid_.target_effort - pid_.current_effort;
    pid_.integral += pid_.error *pid_.sample_time;

    if(pid_.integral > pid_.max_effort)
    {
        pid_.integral = pid_.max_effort;
    }

    else if(pid_.integral < pid_.min_effort)
    {
      pid_.integral   = pid_.min_effort;
    }
     
    float derivative = (pid_.error - pid_.prev_error) / pid_.sample_time; 
    float derivative_kick_factor = (pid_.current_effort - pid_.prev_position) /pid_.sample_time; 

    float P = pid_.Kp * pid_.error;
    float I = pid_.Ki * pid_.integral;
    float D = pid_.Kd * derivative_kick_factor; 

    pid_.output = P + I + D;

    // update previoous positions
    pid_.prev_error = pid_.error;
    pid_.prev_position = pid_.current_effort;

    if(pid_.output > pid_.max_effort)
    {
        pid_.output = pid_.max_effort;
    }

    else if(pid_.output < pid_.min_effort)
    {
        pid_.output = pid_.min_effort;
    }

    return pid_.output;
}
