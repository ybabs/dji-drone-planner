#ifndef PLANNER_H
#define PLANNER_H


#include <tuple>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <queue>
#include "uav_agent/controllers/pid.h"
#include "uav_agent/base/base.h"
#include "uav_agent/base/control.h"
#include "gcs/Action.h"
#include "gcs/Waypoint.h"
#include "gcs/Missionparameters.h"


class HLControl;
class PIDController;

enum MissionState
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
        void flyHome();
        void setYawAngle();
        void getLocalPositionOffset(geometry_msgs::Vector3 &deltaENU, sensor_msgs::NavSatFix &target, sensor_msgs::NavSatFix &origin);
        bool isMissionFinished();
        bool reachedWaypoint();
        void onWaypointReached();
        void onMissionFinished();
        void runMission();
        void prepareFlightPlan(gcs::Waypoint waypoint);
        void appendFlightPlan(gcs::Waypoint waypoint);
        void droneControlSignal(double x, double y, double z, double yaw, bool use_yaw_rate = true, bool use_ground_frame = true);
        void setZOffset(double offset);
        Eigen::Vector3d getEffort(Eigen::Vector3d &target);
        Eigen::Vector2d getHorizontalEffort(Eigen::Vector2d &target);
        Eigen::Vector3d getHomeEffort(Eigen::Vector3d &target);
        Eigen::Vector3d setTargetVector(float target_x, float target_y, float target_z);
        Eigen::Vector3d setHomeTarget(float x, float y, float z);
        float setYaw(float yaw);
        void flightAnomalyCallback(const dji_sdk::FlightAnomaly::ConstPtr &msg);
        void missionParamCallback(const gcs::Missionparameters::ConstPtr &msg);
        void missionPauseCallback(const std_msgs::UInt8::ConstPtr& msg);
        void waypointCallback(const gcs::Waypoint::ConstPtr &msg);
        void missionActionCallback(const gcs::Action::ConstPtr &msg);
        double getPositionError(sensor_msgs::NavSatFix &origin, gcs::Waypoint &target);
        void computeLandingError();

    private:
        HLControl control;
        PIDController pid_pos;
        PIDController pid_z;
        PIDController pid_yaw;

        int uav_state; // 
        int alti_control;
        int waypoint_index;
        int waypoint_count;

        int uav_speed;
        int mission_end_action;
        int drone_action;
        int play_pause;

        Eigen::Vector3d target_position_vector;
        Eigen::Vector3d home_position_vector;
        float target_yaw_angle;
        float desired_yaw_angle;
        float current_yaw_angle;
        float z_offset_takeoff;
        float yaw_limit;
        float distance_to_setpoint;
        float xy_setpoint_dist;
        float home_target_norm;
        float home_distance;
        float target_norm;
        float hori_target_norm;
        int drone_version;
        int ctrl_flag;
        uint32_t flight_anomaly_data;
        float kp, ki, kd;
        float kp_z, ki_z, kd_z;
        float kp_y, ki_y, kd_y;
        float z_max = 5;
        float z_min = -4;
        float z_clamp_p;
        float z_clamp_n;
        double total_landing_error;
       

        sensor_msgs::NavSatFix start_gps_location;
        sensor_msgs::NavSatFix home_gps_location;
        sensor_msgs::NavSatFix home_start_gps_location;
        geometry_msgs::Point start_local_position;

        std::vector<gcs::Waypoint> flight_plan;

        ros::Subscriber param_subscriber;
        ros::Subscriber waypoint_subscriber;
        ros::Subscriber mission_pause_subscriber;
        ros::Subscriber mission_action_subscriber;
        ros::Subscriber flight_anomaly_subscriber;

        ros::Publisher daqrecord_publisher;

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
         ros::Publisher error_publisher;

         


};





#endif