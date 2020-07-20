// Alternative implementation of PID controller from : https://github.com/pms67/PID

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

struct PIDControl {

    // Gains
    float Kp;
    float Ki;
    float Kd;

    /* LPF  time constant derivative */
    float tau;

    // Output limits
    float max_limit;
    float min_limit;

    // Sample Time 
    float dt;

    float integrator;
    float prev_error;
    float differentiator;
    float prev_value;


    // output 
    float output;


};

void  PIDInit(PIDControl * pid);
float PIDUpdate(PIDControl *pid, float setpoint, float measurement);

#endif