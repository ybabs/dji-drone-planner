#ifndef PLANNER_H
#define PLANNER_H

#include <tuple>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <queue>

#include "uav_agent/controllers/pid.h"
#include "uav_agent/base/base.h"
#include "uav_agent/base/control.h"

class Control;

enum UavState
{
    IDLE = 0,
    NEW_WAYPOINT = 1,
    ARRIVED = 2,
    FINISHED = 3,
    RTH = 4
};

class Planner: public Base
{
    public:
        Planner();
        ~Planner();

};





#endif