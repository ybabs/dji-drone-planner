
#include "uav_agent/base/planner.h"

Planner::Planner()
{

      // param_subscriber = nh.subscribe("/gcs/params", 10, &Base::missionParamCallback, this);
   // waypoint_subscriber = nh.subscribe("/gcs/waypoint", 10, &Base::waypointCallback, this);
   // mission_pause_subscriber = nh.subscribe("/gcs/mission_pause", 10, &Base::missionPauseCallback, this);
   // mission_action_subscriber = nh.subscribe("/gcs/mission_action", 10, &Base::missionActionCallback, this );
   // flight_anomaly_subscriber = nh.subscribe("dji_sdk/flight_anomaly", 10, &Base::flightAnomalyCallback, this);

}

Planner::~Planner()
{

}

void Planner::setWaypoint(sensor_msgs::NavSatFix new_waypoint)
{

}
void Planner::fly()
{

}
void Planner::reset()
{

}
void Planner::horizontalControl()
{

}
void Planner::flyHome()
{

}
void Planner::setYawAngle()
{

}
void Planner::getLocalPositionOffset(geometry_msgs::Vector3 &deltaENU, sensor_msgs::NavSatFix &target, sensor_msgs::NavSatFix &origin)
{

}
bool Planner::isMissionFinished()
{
   return  false;
}
bool Planner::reachedWaypoint()
{
    return  false;
}
void Planner::onWaypointReached()
{

}
void Planner::onMissionFinished()
{

}
void Planner::runMission()
{

}
void Planner::prepareFlightPlan(double lat, double lon, double alt, unsigned char sampling_task, float samplingTime)
{

}
void Planner::appendFlightPlan(sensor_msgs::NavSatFix newWaypoint, unsigned char land, float samplingTime)
{

}
void Planner::droneControlSignal(double x, double y, double z, double yaw, bool use_yaw_rate, bool use_ground_frame)
{

}
void Planner::setZOffset(double offset)
{

}
Eigen::Vector3d Planner::getEffort(Eigen::Vector3d &target)
{
   return target_position_vector;
}
Eigen::Vector2d Planner::getHorizontalEffort(Eigen::Vector2d &target)
{
   Eigen::Vector2d position_unit_vector = Eigen::Vector2d::Zero();

   return position_unit_vector;

}
Eigen::Vector3d Planner::getHomeEffort(Eigen::Vector3d &target)
{
   return target_position_vector;
}
Eigen::Vector3d Planner::setTarget(float x, float y, float z)
{
   return target_position_vector;
}
Eigen::Vector3d Planner::setHomeTarget(float x, float y, float z)
{
   return target_position_vector;

}
void Planner::flightAnomalyCallback(const dji_sdk::FlightAnomaly::ConstPtr &msg)
{

}


void Planner::missionParamCallback(const gcs_msgs::Missionparameters::ConstPtr &msg)
{
   uav_speed = msg->uavSpeed;
   mission_end_action = msg->missionEndAction;
}

void Planner::missionPauseCallback(const std_msgs::UInt8::ConstPtr& msg)
{
   play_pause = msg->data;
}

void Planner::waypointCallback(const gcs_msgs::Waypoint::ConstPtr &msg)
{
  // waypoint = msg;
}

void Planner::missionActionCallback(const gcs_msgs::Action::ConstPtr &msg)
{
    drone_action = msg->droneaction;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "Flight Control");

    ROS_INFO("Running");

    ros::spin();

    return 0;
}