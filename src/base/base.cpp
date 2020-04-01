#include "uav_agent/base/base.h"


using namespace DJI::OSDK

Base::Base()
{

   ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
   drone_task_service     = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
   query_version_service      = nh.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");
   drone_activation_service = nh.serviceClient<dji_sdk::Activation>("dji_sdk/activation");
   set_local_pos_reference    = nh.serviceClient<dji_sdk::SetLocalPosRef> ("dji_sdk/set_local_pos_ref");

   gps_subscriber = nh.subscribe("dji_sdk/gps_position", 10, &Base::gpsCallback, this);
   gps_health_subscriber = nh.subscribe("dji_sdk/gps_health", 10, &Base::gpsHealthCallback, this);
   flight_status_subscriber = nh.subscribe("dji_sdk/flight_status", 10, &Base::flightStatusCallback, this);
   height_subscriber = nh.subscribe("/dji_sdk/height_above_takeoff",10, &Base::heightCallback, this);
   velocity_subscriber = nh.subscribe("/dji_sdk/velocity", 10,  &Base::velocityCallback, this);  
   local_position_subscriber = nh.subscribe("/dji_sdk/local_position", 10, &Base::localPositionCallback, this);
   attitude_subscriber = nh.subscribe("/dji_sdk/attitude", 10, &Base::attitudeCallback, this);
   param_subscriber = nh.subscribe("/gcs/params", 10, &Base::missionParamCallback, this);
   waypoint_subscriber = nh.subscribe("/gcs/waypoint", 10, &Base::waypointCallback, this);
   mission_pause_subscriber = nh.subscribe("/gcs/mission_pause", 10, &Base::missionPauseCallback, this);
   mission_action_subscriber = nh.subscribe("/gcs/mission_action", 10, &Base::missionActionCallback, this );
   flight_anomaly_subscriber = nh.subscribe("dji_sdk/flight_anomaly", 10, &Base::flightAnomalyCallback, this);

}

Base::~Base()
{

}