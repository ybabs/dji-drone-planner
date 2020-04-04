#ifndef PLANNER_H
#define PLANNER_H

#include <tuple>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <queue>

#include "uav_agent/controllers/pid.h"
#include "uav_agent/base/base.h"
#include "uav_agent/base/control.h"

#include "gcs_msgs/Action.h"
#include "gcs_msgs/Waypoint.h"
#include "gcs_msgs/Missionparameters.h"


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
        void setWaypoint(sensor_msgs::NavSatFix new_waypoint);
        void fly();
        void reset();
        void horizontalControl();
        void flyHome();
        void setYawAngle();
        void getLocalPositionOffset(geometry_msgs::Vector3 &deltaENU, sensor_msgs::NavSatFix &target, sensor_msgs::NavSatFix &origin);
        bool isMissionFinished();
        bool reachedWaypoint();
        void onWaypointReached();
        void onMissionFinished();
        void runMission();
        void prepareFlightPlan(double lat, double lon, double alt, unsigned char sampling_task, float samplingTime);
        void appendFlightPlan(sensor_msgs::NavSatFix newWaypoint, unsigned char land, float samplingTime);
        void droneControlSignal(double x, double y, double z, double yaw, bool use_yaw_rate = true, bool use_ground_frame = true);
        void setZOffset(double offset);
        Eigen::Vector3d getEffort(Eigen::Vector3d &target);
        Eigen::Vector2d getHorizontalEffort(Eigen::Vector2d &target);
        Eigen::Vector3d getHomeEffort(Eigen::Vector3d &target);
        Eigen::Vector3d setTarget(float x, float y, float z);
        Eigen::Vector3d setHomeTarget(float x, float y, float z);
        void flightAnomalyCallback(const dji_sdk::FlightAnomaly::ConstPtr &msg)
        void missionParamCallback(const gcs_msgs::Missionparameters::ConstPtr &msg);
        void missionPauseCallback(const std_msgs::UInt8::ConstPtr& msg);
        void waypointCallback(const gcs_msgs::Waypoint::ConstPtr &msg);
        void missionActionCallback(const gcs_msgs::Action::ConstPtr &msg);

    private:
        Control control;

        int uav_state; // 
        int vert_control;
        int waypoint_index;
        int waypoint_count;

        int uav_speed;
        int mission_end_action;
        gcs_msgs::Waypoint waypoint;
        int drone_action;
        int play_pause;

        Eigen::3d target_position_vector;
        Eigen::3d home_position_vector;
        float target_yaw_angle;
        float desired_yaw_angle;
        float current_yaw_angle;
        float z_offset_takeoff;
        float yaw_limit;
        float distance_to_setpoint;
        float xy_setpoint_dist;
        int drone_version;
        int ctrl_flag;
        uint32_t flight_anomaly_data;

        sensor_msgs::NavSatFix start_gps_location;
        sensor_msgs::NavSatFix home_gps_location;
        sensor_msgs::NavSatFix home_start_gps_location;
        geometry_msgs::Point start_local_position;

        std::vector<sensor_msgs::NavSatFix> flight_plan;

        // check if landing is required at the waypoint
         std::queue<std::tuple<std::vector<sensor_msgs::NavSatFix>, unsigned char, float>> waypoint_lists;
         bool waypoint_finished;
         bool yaw_flag;
         bool takeoff_result;
         bool rth_complete;

         uint8_t control_flag = (DJISDK::VERTICAL_VELOCITY |
                                DJISDK::HORIZONTAL_VELOCITY |
                                DJISDK::YAW_RATE |
                                DJISDK::HORIZONTAL_GROUND |
                                DJISDK::STABLE_ENABLE);

         ros::Publisher control_publisher;
         


};





#endif