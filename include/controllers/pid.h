#ifndef _PID_H_
#define _PID_H_

#include <ros/ros.h>

 struct pid {
     float current_effort;
     float target_effort;
     float prev_position;  // to get rid of derivative kick
     float error;
     float prev_error;
     float  Kp;
     float Ki;
     float Kd;
     float output;
     float integral;
     float max_effort;
     float min_effort;
     float sample_time;
 } 

 class PIDController
 {
     public:
        void init(float kp, float ki, float kd, float max, float min);
        void reset();
        float update(float setpoint, float current, float dt);
     private:
        pid pid_;
        int index;
 }

#endif