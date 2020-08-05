#include "uav_agent/base/base.h"


using namespace DJI::OSDK;

Base::Base()
{

   ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("m100/sdk_control_authority");
   drone_task_service     = nh.serviceClient<dji_sdk::DroneTaskControl>("m100/drone_task_control");
   query_version_service      = nh.serviceClient<dji_sdk::QueryDroneVersion>("m100/query_drone_version");
   drone_activation_service = nh.serviceClient<dji_sdk::Activation>("m100/activation");
   set_local_pos_reference    = nh.serviceClient<dji_sdk::SetLocalPosRef> ("m100/set_local_pos_ref");

   gps_subscriber = nh.subscribe("m100/gps_position", 10, &Base::gpsCallback, this);
   gps_health_subscriber = nh.subscribe("m100/gps_health", 10, &Base::gpsHealthCallback, this);
   flight_status_subscriber = nh.subscribe("m100/flight_status", 10, &Base::flightStatusCallback, this);
   altitude_subscriber = nh.subscribe("/m100/height_above_takeoff",10, &Base::altitudeCallback, this);
   velocity_subscriber = nh.subscribe("/m100/velocity", 10,  &Base::velocityCallback, this);  
   local_position_subscriber = nh.subscribe("/m100/local_position", 10, &Base::localPositionCallback, this);
   attitude_subscriber = nh.subscribe("/m100/attitude", 10, &Base::attitudeCallback, this);


   checkUAVType();

   if(uav_model == UAV::Type::M100)
   {
      ROS_INFO("UAV Type: DJI M100");

   }
   else if (uav_model == UAV::Type::N3)
   {
      ROS_INFO("UAV Type: N3 Autopilot");
   }

   else if (uav_model == UAV::Type::A3)
   {
      ROS_INFO("UAV Type: A3 Autopilot");
   }

}

Base::~Base()
{

   bool release_control = releaseControl();

   if(release_control == true)
   {
      ROS_INFO(" Program released control");
   }

   else
   {
      ROS_ERROR("Release control failed");
   }

}

void Base::localPositionCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
     current_time = ros::Time::now();
      ros::Duration time = current_time - prev_time;
     current_local_position = msg->point;
     prev_time = current_time;

     dt = time.toSec();
     

}

void Base::attitudeCallback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
    current_drone_attitude = msg->quaternion;
}

void Base::altitudeCallback(const std_msgs::Float32::ConstPtr& msg)
{
    altitude_above_takeoff = msg->data;

}

void Base::velocityCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
    velocity_data.vector = msg->vector;
    velocity_data.header = msg->header;
}

void Base::flightStatusCallback(const std_msgs::UInt8::ConstPtr& msg)
{
    flight_status = msg->data;
}

void Base::displayModeCallback(const std_msgs::UInt8::ConstPtr& msg)
{
    display_mode = msg->data;
}

void Base::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  current_gps_location.latitude = msg->latitude;
  current_gps_location.longitude = msg->longitude;
  current_gps_location.altitude = msg->altitude;
}

void Base::gpsHealthCallback(const std_msgs::UInt8::ConstPtr& msg)
{
  gps_health = msg->data;
  ROS_INFO_ONCE ("GPS Health: %i", gps_health);
}

void Base::activate()
{
   dji_sdk::Activation activation;
   drone_activation_service.call(activation);
   
   if(!activation.response.result)
   {
       ROS_ERROR("Drone control program not activated. Please check your app key");
   }
   else
   {
      ROS_INFO("Activation Successful");
   }

}

bool Base::setLocalPosition()
{
  dji_sdk::SetLocalPosRef localPosReferenceSetter;
  set_local_pos_reference.call(localPosReferenceSetter);
  return localPosReferenceSetter.response.result;
}

UAV::Type Base::checkUAVType()
{
   dji_sdk::QueryDroneVersion query;
   query_version_service.call(query);
   

   if(query.response.hardware == std::string(Version::M100))
   {

      uav_model = UAV::Type::M100;

   }

   else if (query.response.hardware ==  std::string(Version::N3))
   {
      uav_model = UAV::Type::N3;
   }

   else if (query.response.hardware ==  std::string(Version::A3))
   {
      uav_model = UAV::Type::A3;

   }

   return uav_model;
}

bool Base::releaseControl()
{
   dji_sdk::SDKControlAuthority authority;
    authority.request.control_enable = 0;
    ctrl_authority_service.call(authority);
    if(authority.response.result)
    {
        ROS_INFO("Program has released control");
        return true;
    }

    if(!authority.response.result)
    {
        ROS_INFO("Program failed to release control");
        return false;
    }

    return true;
}

bool Base::obtainControl()
{
    dji_sdk::SDKControlAuthority authority;
    authority.request.control_enable = 1;
    ctrl_authority_service.call(authority);
    if(authority.response.result)
    {
        ROS_INFO("Program has obtained control");
        return true;
    }
    return false;
}



