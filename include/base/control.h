#ifndef CONTROL_H
#define CONTROL_H

#include "uav_agent/base/base.h"

class Control : public Base
{
    public:
        bool takeoff();
        bool land();
        void rth();
        float computeTimeToLand();

    private:
        float land_time;

};



#endif