#ifndef FLIGHT_BASE
#define FLIGHT_BASE

#include <ros/ros.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/UInt8.h>
#include <tf/tf.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>

#include <djiosdk/dji_vehicle.hpp>
#include <dji_sdk/dji_sdk_node.h>
#include "dji_sdk/dji_sdk.h"
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/QueryDroneVersion.h>
#include <dji_sdk/SetLocalPosRef.h>
#include <dji_sdk/Activation.h>


#include "uav_agent/utils/utils.h"

struct UAV
{
    typedef enum
    {
        M100 = 0,
        N3 = 1,
        A3 = 2,
        UNKNOWN = 2
    } Type;
};

class Base
{
    public:
        Base();
        ~Base();
        void activate(); // activate UAV for SDK control
        bool obtainControl();
        bool releaseControl();
        void localPositionCallback(const geometry_msgs::PointStamped::ConstPtr &msg);
        void attitudeCallback(const geometry_msgs::QuaternionStamped::ConstPtr &msg);
        void flightStatusCallback(const std_msgs::UInt8::ConstPtr& msg);
        void displayModeCallback(const std_msgs::UInt8::ConstPtr& msg);
        void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
        void gpsHealthCallback(const std_msgs::UInt8::ConstPtr& msg);
        void altitudeCallback(const std_msgs::Float32::ConstPtr& msg);
        UAV::Type checkUAVType();
        bool setLocalPosition();
        void flightAnomalyCallback(const dji_sdk::FlightAnomaly::ConstPtr &msg);
        void velocityCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
        
        
        

    protected:
        ros::NodeHandle nh;
        ros::ServiceClient ctrl_authority_service;
        ros::ServiceClient drone_task_service;
        ros::ServiceClient query_version_service;
        ros::ServiceClient drone_activation_service;
        ros::ServiceClient set_local_pos_reference;
        ros::Subscriber gps_subscriber;
        ros::Subscriber gps_health_subscriber;
        ros::Subscriber flight_status_subscriber;
        ros::Subscriber altitude_subscriber;
        ros::Subscriber attitude_subscriber;
        ros::Subscriber local_position_subscriber;
        ros::Subscriber velocity_subscriber;


        
        geometry_msgs::Vector3Stamped velocity_data;
        geometry_msgs::Point current_local_position;
        sensor_msgs::NavSatFix current_gps_location;
        geometry_msgs::Quaternion current_drone_attitude;
        
        uint8_t flight_status;
        uint8_t display_mode;
        uint8_t gps_health;
        float altitude_above_takeoff;
        UAV::Type uav_model;
        
        ros::Time current_time;
        ros::Time prev_time;
        double dt;


};






#endif