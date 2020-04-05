#ifndef CONTROL_H
#define CONTROL_H

#include "uav_agent/base/base.h"

class HLControl : public Base
{
    public:
        HLControl();
        bool takeoff();
        bool land();
        void rth();
        float computeTimeToLand();
        bool takeoffLand(int task);

    private:
        float land_time;
        int motor_wait_time;
        int takeoff_timeout;
        float takeoff_altitude;

};



#endif


// if you dont listen to me and whatever i say
// there will be consequences 
// random shit will start to show up like this in your code 
// but you wont be this lucky
// cuz you wont be able find it 
// 
// Whuahahahahahahahahhahahahahahahahahahahahahaha
// 
//       _..--""````""--.._
//     .'       |\/|       '.
//    /    /`._ |  | _.'\    \
//   ;    /              \    |
//   |   /                \   |
//   ;  / .-.          .-. \  ;
//    \ |/   \.-.  .-./   \| /
//     '._       \/       _.'
//        ''--..____..--''
// 
// whuahahahahahahahahahahahahahahahahahahahahaha
