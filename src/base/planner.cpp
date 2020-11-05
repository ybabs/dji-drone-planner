
#include "uav_agent/base/planner.h"

Planner::Planner()
{
   param_subscriber = nh.subscribe("/gcs/params", 10, &Planner::missionParamCallback, this);
   waypoint_subscriber = nh.subscribe("/gcs/waypoint", 100, &Planner::waypointCallback, this);
   mission_pause_subscriber = nh.subscribe("/gcs/mission_pause", 10, &Planner::missionPauseCallback, this);
   mission_action_subscriber = nh.subscribe("/gcs/mission_action", 10, &Planner::missionActionCallback, this );
   flight_anomaly_subscriber = nh.subscribe("a3/flight_anomaly", 10, &Planner::flightAnomalyCallback, this);

    control_publisher = nh.advertise<sensor_msgs::Joy>("a3/flight_control_setpoint_generic", 10);

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
   ki = 0;
   kp = 1.2;
   kd = 0.05;

   kp_z = 0.8;
   ki_z = 0;
   kd_z = 0.001;

}

Planner::~Planner()
{

}

void Planner::setWaypoint(sensor_msgs::NavSatFix new_waypoint)
{
   geometry_msgs::Vector3 offset_from_target;

   getLocalPositionOffset(offset_from_target, new_waypoint, start_gps_location);
   ROS_INFO("offset X = %f", offset_from_target.x );
    ROS_INFO("offset Y = %f", offset_from_target.y );
     ROS_INFO("offset Z = %f", offset_from_target.z );


   // Set UAV Target
   setTargetVector(offset_from_target.x, offset_from_target.y, offset_from_target.z);

}

Eigen::Vector3d Planner::setTargetVector(float target_x, float target_y, float target_z)
{
   ROS_INFO("Target distance in 3D %fm, %fm, %fm", target_x, target_y, target_z);
   target_position_vector << target_x, target_y, target_z;
   Eigen::Vector2d xy_pos_vec;
   xy_pos_vec << target_x, target_y;

   hori_target_norm = xy_pos_vec.norm();
   target_norm = target_position_vector.norm();
   return target_position_vector;
}

void Planner::prepareFlightPlan(gcs::Waypoint waypoint)
{

   float altitude_offset = current_gps_location.altitude;
   waypoint.altitude = waypoint.altitude + altitude_offset;

   ROS_INFO("Altitude offset for waypoint %d set as %f, drone altitude is now at %f", waypoint_index, 
            altitude_offset,  waypoint.altitude);

   // Add Waypoint to list of waypoints to be visited
   appendFlightPlan(waypoint);   

}

void Planner::appendFlightPlan(gcs::Waypoint new_waypoint)
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
   gcs::Waypoint current_waypoint = flight_plan[waypoint_index];
   uint8_t checkTask = current_waypoint.sample;
   float recording_time = current_waypoint.sampleTime;

   if(!flight_plan.empty())
   {
      if(checkTask == 1)
      {  
         bool initiate_record;
         
         if(uav_model == UAV::Type::M100)
         {
             initiate_record = control.M100Land();
         }
         else
         {
            initiate_record = control.land();
         }
        

         if(initiate_record)
         {
             // start recording stuff here
             ros::Duration(recording_time).sleep();
             
             // check if we are not at the last waypoint
             // and if mission end is set to autoland
             if(uav_model == UAV::Type::M100)
             {

                ROS_INFO("M100 UAV Takeoff");

               if(flight_plan.size() > 1)
               {
                  alti_control = 1;
                  //control.M100Takeoff();
                  ROS_INFO("first case");
               }

               //if we need to return home
               if(flight_plan.size() <=1 && mission_end_action == 2)
               {
                  ROS_INFO("Second Case");
                  alti_control = 1;
                  //control.M100Takeoff();
               }

             }
             else
             {

               ROS_INFO("N3/A3 UAV Takeoff");

               if(flight_plan.size() > 1)
               {
                  alti_control = 1;
                  //control.takeoff();
                  ROS_INFO("first case");
               }

               //if we need to return home
               if(flight_plan.size() <=1 && mission_end_action == 2)
               {
                  ROS_INFO("Second Case");
                  alti_control = 1;
                  //control.takeoff();
               }

             }

         }

         else
         {
            ROS_ERROR("UAV landed unsucessfully");
         }
      }
   }

   else
   {
      ROS_INFO("No more flight plans");
   }

   ROS_INFO("Here");
   waypoint_index++;

     ROS_INFO("Here 3");

   uav_state = MissionState::ARRIVED;

}

void Planner::onMissionFinished()
{
   ROS_INFO("Mission Ended");
   // clear flight pplan and reset waypoint information
   flight_plan.clear();
   waypoint_count = 0;
   waypoint_index = 0;

   if(mission_end_action == 2)
   {
      if(uav_model == UAV::Type::M100)
      {
         control.M100Takeoff();
      }

      else
      {
         control.takeoff();
      }
   }

   uav_state = MissionState::FINISHED;

}

void Planner::fly()
{
   geometry_msgs::Vector3 position_offset;
   // Compute the local position delta from the current waypoint to the next waypoint
   getLocalPositionOffset(position_offset, current_gps_location, start_gps_location);

    double x_offset_left = target_position_vector[0] - position_offset.x;
    double y_offset_left = target_position_vector[1] - position_offset.y;
    double z_offset_left = target_position_vector[2] - position_offset.z;   

    Eigen::Vector2d xy_effort;
    xy_effort << x_offset_left, y_offset_left;
    Eigen::Vector2d cmd_vector;
    cmd_vector = getHorizontalEffort(xy_effort);

   //  Eigen::Vector3d effort;
   //  effort << x_offset_left, y_offset_left, z_offset_left;
   //  Eigen::Vector3d cmd_vector;
   //  cmd_vector = getEffort(effort);

   //ROS_INFO("Target_norm:  %f", target_norm);
   //ROS_INFO("Distance to Setpoint:  %f", distance_to_setpoint);
   //double pid_effort = pid_pos.update(target_norm, distance_to_setpoint, dt);

   //ROS_INFO("H_Target_norm:  %f", hori_target_norm);
   //ROS_INFO("Distance to Setpoint:  %f", xy_setpoint_dist);
   double temp_val = hori_target_norm - xy_setpoint_dist;
    double pid_effort = pid_pos.update(hori_target_norm, temp_val, dt);
   

     // decouple z from xy;
     double z_pid_effort = pid_z.update(target_position_vector[2], current_local_position.z, dt);

         // commands to be sent to control publisher
      double x_cmd, y_cmd, z_cmd;
      x_cmd = pid_effort * cmd_vector[0];
      y_cmd = pid_effort * cmd_vector[1];
      //z_cmd = pid_effort * cmd_vector[2];
      z_cmd = z_pid_effort;

      // ROS_INFO("PID Effort %f", pid_effort);
       

      // Wait for UAV to gain required altitude
      if(alti_control == 1 && z_offset_left > 0.5)
      {
         //ROS_INFO("Z Offset: %f", z_offset_left);
         droneControlSignal(0,0,z_cmd, 0);
      }

      else
      {
         alti_control = 0;
         droneControlSignal(x_cmd, y_cmd, 0, 0);
        // ROS_INFO("X cmd: %f", x_cmd);
        // ROS_INFO("Y cmd: %f", y_cmd);

      }

      // if(distance_to_setpoint < 0.5)
      // {
      //    droneControlSignal(0,0,0,0);
      //    ROS_INFO("At Setpoint");
      //    waypoint_finished = true;
      // }
         if (xy_setpoint_dist < 0.5)
         {
            droneControlSignal(0, 0, 0, 0);
            ROS_INFO("Setpoint");
            waypoint_finished = true;
         }





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


// }
void Planner::flyHome()
{
   geometry_msgs::Vector3 offset_from_home;
   getLocalPositionOffset(offset_from_home, current_gps_location, home_start_gps_location);

   double home_x_offset_left = home_position_vector[0] - offset_from_home.x;
   double home_y_offset_left = home_position_vector[1] - offset_from_home.y;
   double home_z_offset_left = home_position_vector[2] - offset_from_home.z;

   Eigen::Vector3d effort;
   effort << home_x_offset_left, home_y_offset_left, home_z_offset_left;
   ROS_INFO("STEP offset LEFT  x: %f y: %f z: %f ", home_x_offset_left, home_y_offset_left, home_z_offset_left);

   Eigen::Vector3d cmd_vector;
   cmd_vector = getHomeEffort(effort);
   double temp_val = home_target_norm - home_distance;

   double pid_effort = pid_pos.update(home_target_norm, temp_val , dt);
   ROS_INFO("home pid effort = %f", pid_effort);
   ROS_INFO("home target = %f", home_target_norm);
   ROS_INFO("home distance = %f", home_distance);

   double x_cmd, y_cmd, z_cmd;

    x_cmd = pid_effort * cmd_vector[0];
    y_cmd = pid_effort * cmd_vector[1];
    z_cmd = pid_effort * cmd_vector[2];
    ROS_INFO("Home X CMD = %f", x_cmd);
    ROS_INFO("Home Y CMD = %f", y_cmd);
    ROS_INFO("Home Z CMD = %f", z_cmd);


    if(home_z_offset_left > 1)
    {
       droneControlSignal(0,0, z_cmd, 0);
    }

    else
    {
        droneControlSignal(x_cmd, y_cmd, 0, 0, true, true);
    }

    if(home_distance <= 1)
    {
       droneControlSignal(0,0,0,0);
      if(uav_model == UAV::Type::M100)
       {
          control.M100Land();

       }
       else{

         control.land();

       }
       rth_complete = true;
    }


}
Eigen::Vector3d Planner::getHomeEffort(Eigen::Vector3d &target)
{
    double vector_length = target.norm();
    home_distance = vector_length;
    Eigen::Vector3d position_unit_vector = Eigen::Vector3d::Zero();

    // check for zero magnitude
    if (vector_length != 0)
    {
        // normalize vector;
        position_unit_vector = target.normalized();
    }
    return position_unit_vector;
}

Eigen::Vector3d Planner::setHomeTarget(float x, float y, float z)
{
   home_position_vector << x, y, z;
   home_target_norm = home_position_vector.norm();
   return home_position_vector;
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
            gcs::Waypoint temp_waypoint = flight_plan[waypoint_index];
            sensor_msgs::NavSatFix waypoint_global_coords;
            waypoint_global_coords.latitude = temp_waypoint.latitude;
            waypoint_global_coords.longitude = temp_waypoint.longitude;
            waypoint_global_coords.altitude = temp_waypoint.altitude;
            setWaypoint(waypoint_global_coords);

            uav_state = MissionState::NEW_WAYPOINT;
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
               fly();
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
            gcs::Waypoint temp_waypoint = flight_plan[waypoint_index];
            sensor_msgs::NavSatFix waypoint_global_coords;
            waypoint_global_coords.latitude = temp_waypoint.latitude;
            waypoint_global_coords.longitude = temp_waypoint.longitude;
             waypoint_global_coords.altitude = temp_waypoint.altitude;
            setWaypoint(waypoint_global_coords);

            ROS_INFO("ARRIVED AT WAYPOINT %d / %d ", waypoint_index , waypoint_count);
            if(uav_model == UAV::Type::M100)
            {
               control.M100Takeoff();
            }

            else
            {
               control.takeoff();
            }
            uav_state = MissionState::NEW_WAYPOINT;
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
               //uav_state = MissionState::RTH;
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

            case 3:
            {
               if(uav_model == UAV::Type::M100)
               {
                  control.M100Land();

               }
               else{

                  control.land();

               }
               uav_state = MissionState::IDLE;
               break;
            }

            default:
            {
               uav_state = MissionState::IDLE;
               break;
            }
         }
  
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
   geometry_msgs::Vector3 position_offset;
   getLocalPositionOffset(position_offset, current_gps_location, start_gps_location);

   double x_offset_left = target_position_vector[0] - position_offset.x;
   double y_offset_left = target_position_vector[1] - position_offset.y;

   current_yaw_angle = toEulerAngle(current_drone_attitude).z;
   current_yaw_angle = (current_yaw_angle >= 0) ? current_yaw_angle : current_yaw_angle + 2 * C_PI;

    desired_yaw_angle = atan2(y_offset_left, x_offset_left);
    desired_yaw_angle = (desired_yaw_angle >= 0) ? desired_yaw_angle : desired_yaw_angle + 2 * C_PI;

    double yaw_pid = pid_yaw.update(desired_yaw_angle, current_yaw_angle, dt);
   //double yaw_pid = pid_yaw.PIDUpdate(desired_yaw_angle, current_yaw_angle, dt);
    double desired_yaw_angle_deg = RadToDeg(desired_yaw_angle);
    double current_yaw_deg = RadToDeg(current_yaw_angle);

     //ROS_INFO("Desired angle degree= %f", desired_yaw_angle_deg);
   //  ROS_INFO("Current Yaw Angle %f", current_yaw_deg);
   //  ROS_INFO("Yaw PID %f", yaw_pid);
   //  ROS_INFO("dt %f", dt);

    droneControlSignal(0, 0, 0, yaw_pid, true, true);
    // Check if we are close to the required yaw.
    if (fabs(desired_yaw_angle_deg - current_yaw_deg) < 0.9)
    {
        yaw_flag = false;
    }

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
   z_offset_takeoff = offset;
   ROS_INFO("Z position offset applied as %f m", z_offset_takeoff);
}

Eigen::Vector2d Planner::getHorizontalEffort(Eigen::Vector2d &target)
{
   double vector_length = target.norm();
   xy_setpoint_dist = vector_length;

   Eigen::Vector2d position_unit_vector = Eigen::Vector2d::Zero();

   if(vector_length != 0)
   {
      position_unit_vector = target.normalized();
   }

   return position_unit_vector;
}

void Planner::missionParamCallback(const gcs::Missionparameters::ConstPtr &msg)
{
   uav_speed = msg->uavSpeed;
   mission_end_action = msg->missionEndAction;

   ROS_INFO ("Drone Speed = %d m/s", uav_speed);
   ROS_INFO("Mission End Action = %d", mission_end_action);
}

void Planner::missionPauseCallback(const std_msgs::UInt8::ConstPtr& msg)
{
   play_pause = msg->data;
   if(play_pause == 1)
   {
      ROS_INFO("Mission Paused");
      droneControlSignal(0,0,0,0);
   }
}

void Planner::waypointCallback(const gcs::Waypoint::ConstPtr &msg)
{
  // ROS_INFO("Callback Mission Waypoint");
   ROS_INFO("ID: %d", msg->id);

   if(msg->id == 1)
   {
       gcs::Waypoint waypoint;

      waypoint.latitude = msg->latitude;
      waypoint.longitude = msg->longitude;
      waypoint.altitude = msg->altitude;
      waypoint.sample = msg->sample;
      waypoint.sampleTime = msg-> sampleTime;

      prepareFlightPlan(waypoint);
   }

}


void Planner::missionActionCallback(const gcs::Action::ConstPtr &msg)
{
   ROS_INFO("Callbacvk");
   if(msg->id == 1)
   {
    drone_action = msg->droneaction;
    switch(drone_action)
    {
       case 1:
       {
         
          if(uav_model == UAV::Type::M100)
          {
             ROS_INFO("M100 UAV taking off");
             control.M100Takeoff();
          }
          else
          {
             ROS_INFO("N3/A3 UAV taking off");
             control.takeoff();
          }
          
          break;
       }

       case 2:
       {
          ROS_INFO("UAV landing");
          if(uav_model == UAV::Type::M100)
          {
             control.M100Land();
          }

          else
          {
              control.land();
          }
         
          break;
       }

       case 3:
       {
          ROS_INFO("UAV RTH");
          control.rth();
          break;
       }

       case 4:
       {  
         
          ros::Rate loop_rate(50);
          home_gps_location = current_gps_location;
          ROS_INFO("Logged home GPS location at Lat:%f , Lon:%f", 
                  home_gps_location.latitude, home_gps_location.longitude);

          // Set home GPS altitude as a 5 metre offset. 
          //TODO make this changeable in GCS settings
          home_gps_location.altitude = current_gps_location.altitude + 5.0;
          start_gps_location = current_gps_location;
          start_local_position = current_local_position;
          bool is_takeoff;

          if(uav_model == UAV::Type::M100)
          {
               ROS_INFO("M100 Taking off");
              is_takeoff =  control.M100Takeoff();
          }

          else
          {
               ROS_INFO("A3/N3 Taking off");
               is_takeoff = control.takeoff();
          }

          
          if(is_takeoff)
          {
             // Initialise PID
             ROS_INFO("Mission Speed %f", uav_speed);
 
             pid_pos.init(kp, ki, kd, uav_speed, -uav_speed);
             pid_z.init(kp, ki, kd, 3, -3);
             //pid_yaw.init(kp, ki, kd, yaw_limit, -yaw_limit); 
             pid_yaw.init(kp, ki, kd, uav_speed, -uav_speed);

            //  if(uav_model == UAV::Type::M100)
            //  {
            //     ROS_INFO("Setting Z offset for M100");
            //     setZOffset(current_local_position.z);
            //  }

            

             ROS_INFO("Starting mission");

             while(ros::ok())
             {
                ros::spinOnce();
                runMission();
                loop_rate.sleep();
             }


          }

          
          break;
       }


       default:
       {
          break;
       }
    }

   }
   
}

void Planner::flightAnomalyCallback(const dji_sdk::FlightAnomaly::ConstPtr &msg)
{

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "Control");

    Planner planner;

    ROS_INFO("Running");

    ros::spin();

    return 0;
}