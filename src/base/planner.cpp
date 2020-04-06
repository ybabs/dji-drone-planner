
#include "uav_agent/base/planner.h"

Planner::Planner()
{
   param_subscriber = nh.subscribe("/gcs/params", 10, &Base::missionParamCallback, this);
   waypoint_subscriber = nh.subscribe("/gcs/waypoint", 10, &Base::waypointCallback, this);
   mission_pause_subscriber = nh.subscribe("/gcs/mission_pause", 10, &Base::missionPauseCallback, this);
   mission_action_subscriber = nh.subscribe("/gcs/mission_action", 10, &Base::missionActionCallback, this );
   flight_anomaly_subscriber = nh.subscribe("dji_sdk/flight_anomaly", 10, &Base::flightAnomalyCallback, this);

   bool control_res;
   control_res = obtainControl();

   if(control_res == true)
   {
      if(setLocalPosition())
      {
         ROS_INFO("Local Position set succesfully!");
      }
   }

   yaw_limit = DegToRad(180);

   // set initial paramters;
   uav_state = MissionState::IDLE;
   waypoint_index = 0;
   waypoint_finished = false;
   yaw_flag = true;
   rth_complete = false;
   target_position_vector = Eigen::Vector3d::Zero();
   target_yaw_angle = 0;
   alti_control = 1;
   ctrl_flag = 0;

}

Planner::~Planner()
{

}

void Planner::setWaypoint(sensor_msgs::NavSatFix new_waypoint)
{
   geometry_msgs::Vector3 offset_from_target;

   getLocalPositionOffset(offset_from_target, new_waypoint, start_gps_location);

   // Set UAV Target
   setTargetVector(offset_from_target.x, offset_from_target.y, offset_from_target.z);

}

Eigen::Vector3d Planner::setTarget(float target_x, float target_y, float target_z)
{
   ROS_INFO("Target distance in 3D %fm, %fm, %fm", target_x, target_y, target_z);
   target_position_vector << target_x, target_y, target_z;

   target_norm = target_position_vector.norm();
   return target_position_vector;
}

void Planner::prepareFlightPlan(gcs_msgs::Waypoint waypoint)
{

   float altitude_offset = current_gps_location.altitude;
   waypoint.altitude = waypoint.altitude + altitude_offset;

   ROS_INFO("Altitude offset for waypoint %d set as %f, drone altitude is now at %f", waypoint_index, 
            altitude_offset,  waypoint.altitude);

   // Add Waypoint to list of waypoints to be visited
   appendFlightPlan(waypoint);   

}

void Planner::appendFlightPlan(gcs_msgs::Waypoint new_waypoint)
{
   // add new waypoint to vector
   flight_plan.push_back(new_waypoint);

   waypoint_count = flight_plan.size();
   ROS_INFO("Waypoint # %d added", waypoint_count);

   ROS_INFO("Landing Task %d", new_waypoint.sample);
   ROS_INFO("Sample Period %f", new_waypoint.sampleTime);

}

void Planner::onWaypointReached()
{
   gcs_msgs::Waypoint current_waypoint = flight_plan[waypoint_index];
   uint8_t checkTask = current_waypoint.sample;
   float recording_time = current_waypoint.sampleTime;

   if(!flight_plan.empty())
   {
      if(checkTask == 1)
      {  
         bool initiate_record;

         initiate_record = control.land();

         if(initiate_record)
         {
             // start recording stuff here
             ros::Duration(recording_time).sleep();
             
             // check if we are not at the last waypoint
             // and if mission end is set to autoland
             if(flight_plan.size() > 1)
             {
                verti_control = 1;
                control.takeoff();
             }

             //if we need to return home
             if(flight_plan.size() ==1 && mission_end_action ==  MissionEndAction::RTH)
             {
                verti_control = 1;
                control.takeoff();
             }
         }

         else
         {
            ROS_ERROR("UAV landed unsucessfully");
         }
      }
   }

   waypoint_index++;

    // remove current waypoint from the list
    waypoint_lists.pop();

   state = MissionState::ARRIVED;

}

void Planner::onMissionFinished()
{
   ROS_INFO("Mission Ended");
   // clear flight pplan and reset waypoint information
   flight_plan.clear();
   waypoint_count = 0;
   waypoint_index = 0;

}

void Planner::fly()
{
   geometry_msgs::Vector3 position_offset;
   // Compute the local position delta from the current waypoint to the next waypoint
   getLocalPositionOffset(position_offset, current_gps_location, start_gps_location);

    double x_offset_left = target_position_vector[0] - position_offset.x;
    double y_offset_left = target_position_vector[1] - position_offset.y;
    double z_offset_left = target_position_vector[2] - position_offset.z;

    Eigen::Vector3d effort;
    effort << x_offset_left, y_offset_left, z_offset_left;
    Eigen::Vector3d cmd_vector;

    cmd_vector = getEffort(effort);

   //  double pid_effort = ;


}

Eigen::Vector3d Planner::getEffort(Eigen::Vector3d &target)
{
   double vector_length = target.norm();
   distance_to_setpoint = vector_length;

   // Initialise unit vector
   Eigen::Vector3d position_unit_vector = Eigen::Vector3d::Zero();
   // check for zero magnitude
   if(vector_length != 0)
   {
      // normalise vector;
      position_unit_vector = target.normalized();
   }

   return position_unit_vector;
}
void Planner::horizontalControl()
{

}
void Planner::flyHome()
{

}

void Planner::getLocalPositionOffset(geometry_msgs::Vector3 &deltaENU, sensor_msgs::NavSatFix &target, sensor_msgs::NavSatFix &origin)
{
   double deltaLon = target.longitude - origin.longitude;
   double deltaLat = target.latitude - origin.latitude;

    deltaENU.y = DegToRad(deltaLat) * C_EARTH;
    deltaENU.x = DegToRad(deltaLon) * C_EARTH * cos(DegToRad(target.latitude));
    deltaENU.z = target.altitude - origin.altitude;

}
bool Planner::isMissionFinished()
{
    if (waypoint_index >= waypoint_count)
    {
        ROS_INFO("Mission Waypoint count: %d", waypoint_count);
        ROS_INFO("Mission Finished Waypoint index %d", waypoint_index);
        return true;
    }

    else
    {
        ROS_INFO("Mission isn't finished yet. Next waypoint");
        return false;
    }
}
bool Planner::reachedWaypoint()
{
    if (waypoint_finished)
    {
        ROS_INFO("We've reached waypoint %d", waypoint_index + 1);
        return true;
    }

    else
    {
        return false;
    }
}


void Planner::runMission()
{
   switch(uav_state)
   {
      case MissionState::IDLE:
      {
         if(waypoint_count != 0)
         {
            waypoint_finished = false;
            yaw_flag = true;
            gcs_msgs::Waypoint temp_waypoint = flight_plan[waypoint_index];
            sensor_msgs::NavSatFix waypoint_global_coords;
            waypoint_global_coords.latitude = temp_waypoint.latitude;
            waypoint_global_coords.longitude = temp_waypoint.longitude;
            setWaypoint(waypoint_global_coords);

            state = MissionState::NEW_WAYPOINT;
         }
         else
         {
            ROS_INFO_THROTTLE(2, "Mission is in idle state, waiting for mission plan");

            start_gps_location = current_gps_location;
            setLocalPosition();
         }
         break;
      }

      case MissionState::NEW_WAYPOINT:
      {  
         if(!reachedWaypoint())
         {
            if(yaw_flag)
            {
               setYawAngle();
            }

            if(!yaw_flag)
            {
               if(ctrl_flag == 0)
               {
                  fly();
               }

               else
               {
                  horizontalControl();
               }
            }
         }

         else
         {
            onWaypointReached();
         }
         break;
      }

      case MissionState::ARRIVED:
      {
         if(!isMissionFinished())
         {
            waypoint_finished = false;
            yaw_flag = true;
            start_gps_location = current_gps_location;
            start_local_position = current_local_position;
            gcs_msgs::Waypoint temp_waypoint = flight_plan[waypoint_index];
            sensor_msgs::NavSatFix waypoint_global_coords;
            waypoint_global_coords.latitude = temp_waypoint.latitude;
            waypoint_global_coords.longitude = temp_waypoint.longitude;
            setWaypoint(waypoint_global_coords);

            ROS_INFO("ARRIVED AT WAYPOINT %d / %d ", waypoint_index + 1, waypoint_count);
            state = MissionState::NEW_WAYPOINT;
         }
         else
         {
            onMissionFinished();
         }
         break;
      }

      case MissionState::FINISHED:
      {
         ros::Duration(0.1).sleep();
         
         switch(mission_end_action)
         {
            case 1:
            {
               ROS_INFO("Hovering at position");
               droneControlSignal(0,0,0,0, true, true);
               break;

            }

            case 2:
            {
               state = MissionState::RTH;
               break;
            }

            case 3:
            {
               control.land();
               state = MissionState::IDLE;
               break;
            }

            default
            {
               state = MissionState::IDLE;
               break;
            }
         }
  
         break;

      }

      case MissionState::RTH:
      {
         geometry_msgs::Vector3 distance_from_home;
         getLocalPositionOffset(distance_from_home, home_gps_location, current_gps_location);
         setHomeTarget(distance_from_home.x, distance_from_home.y, distance_from_home.z);
         home_start_gps_location = current_gps_location;

         if(!rth_complete)
         {
            flyHome();
         }

         ROS_INFO("RTH complete %d", rth_complete);
         break;

      }

      default:
      {
         break;
      }
   }

}

void Planner::setYawAngle()
{

}

void Planner::droneControlSignal(double x, double y, double z, double yaw, bool use_yaw_rate, bool use_ground_frame)
{
   sensor_msgs::Joy controlPosYaw;
   controlPosYaw.axes.push_back(x);
   controlPosYaw.axes.push_back(y);
   controlPosYaw.axes.push_back(z);
   controlPosYaw.axes.push_back(yaw);

    if (use_yaw_rate && use_ground_frame) // using yaw rate and ground frame
    {
        controlPosYaw.axes.push_back(control_flag);
    }

    control_publisher.publish(controlPosYaw);



}
void Planner::setZOffset(double offset)
{

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

Eigen::Vector3d Planner::setHomeTarget(float x, float y, float z)
{
   return target_position_vector;

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

    gcs_msgs::Waypoint waypoint;

   waypoint.latitude = msg->latitude;
   waypoint.longitude = msg->longitude;
   waypoint.altitude = msg->altitude;
   waypoint.sample = msg->sample;
   waypoint.sampleTime = msg-> sampleTime;

   prepareFlightPlan(waypoint);


}


void Planner::missionActionCallback(const gcs_msgs::Action::ConstPtr &msg)
{
    drone_action = msg->droneaction;
}

void Planner::flightAnomalyCallback(const dji_sdk::FlightAnomaly::ConstPtr &msg)
{

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "Flight Control");

    Planner planner;

    ROS_INFO("Running");

    ros::spin();

    return 0;
}