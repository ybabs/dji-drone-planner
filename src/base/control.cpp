#include "uav_agent/base/control.h"

using namespace DJI::OSDK;


Control::Control()
{
    motor_wait_time = 5;
    takeoff_timeout = 10;
    takeoff_altitude = 0.5;
}

void Control::takeoff()
{

    ros::Time start_time = ros::Time::now();

    if(uav_model ==== UAV::Type::N3 || UAV::Type::A3)
    {
        if (!takeoffLand(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF))
        {
            return false;
        }
        ros::Duration(0.01).sleep();
        ros::spinOnce();

        // Spin UAV Motors and wait for it to spin for 5 seconds
        while(flight_status != DJISDK::FlightStatus::STATUS_ON_GROUND &&
              display_mode != DJISDK::DisplayMode::MODE_ENGINE_START &&
              ros::Time::now() - start_time < ros::Duration(motor_wait_time))
        {
            ros::Duration(0.01).sleep();
            ros::spinOnce();
        }

        if (ros::Time::now() - start_time > ros::Duration(motor_wait_time))
        {
            ROS_ERROR("Takeoff failed. Motors are not spinnning.");
            return false;
        }

        // Get UAV in the air
        // Step 1.2: Get in to the air
        while (flight_status != DJISDK::FlightStatus::STATUS_IN_AIR &&
              (display_mode != DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF 
              || display_mode != DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
               ros::Time::now() - start_time < ros::Duration(takeoff_timeout))
        {
            ros::Duration(0.01).sleep();
            ros::spinOnce();
        }
        if (ros::Time::now() - start_time > ros::Duration(takeoff_timeout))
        {
            ROS_ERROR("Takeoff failed. Aircraft is still on the ground, but the motors are spinning.");
            return false;
        }
        else
        {
            start_time = ros::Time::now();
            ROS_INFO("Ascending...");
            ros::spinOnce();
        }
         // Final check: Finished takeoff
        while ((display_mode == DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF ||
               display_mode == DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
              ros::Time::now() - start_time < ros::Duration(takeoff_timeout))
        {
            ros::Duration(0.01).sleep();
            ros::spinOnce();
        }
        if (display_mode != DJISDK::DisplayMode::MODE_P_GPS || display_mode != DJISDK::DisplayMode::MODE_ATTITUDE)
        {
            ROS_INFO("Successful takeoff!");
            start_time = ros::Time::now();
        }
        else
        {
            ROS_ERROR("Takeoff finished, but the aircraft is in an unexpected mode. Please connect DJI GO.");
            return false;
        }

    }

    else if(uav_model == UAV::Type::M100)
    {

        float home_takeoff = altitude_above_takeoff;
        if (!takeoffLand(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF))
        {
            ROS_INFO("Takeoff Service failed");
            return false;
        }

        ros::Duration(0.01).sleep();
        ros::spinOnce();

        // Step 1: If M100 is not in the air after timeout seconds, fail.
        while (ros::Time::now() - start_time < ros::Duration(takeoff_timeout))
        {
            ros::Duration(0.01).sleep();
            ros::spinOnce();
        }

        if (flight_status != DJISDK::M100FlightStatus::M100_STATUS_IN_AIR || altitude_above_takeoff - home_takeoff < takeoff_altitude)
        {
            if (flight_status == DJISDK::M100FlightStatus::M100_STATUS_FINISHED_LANDING)
            {
            ROS_ERROR("Flight status: Drone Finished Landing");
            }

            if (flight_status == DJISDK::M100FlightStatus::M100_STATUS_LANDING)
            {
            ROS_ERROR("Flight status: Drone Landing");
            }

            if (flight_status == DJISDK::M100FlightStatus::M100_STATUS_ON_GROUND)
            {
            ROS_ERROR("Flight status: Drone still on the ground");
            }

            if (flight_status == DJISDK::M100FlightStatus::M100_STATUS_TAKINGOFF)
            {
            ROS_INFO("Flight status: Drone Taking off");
            }
            ROS_INFO("Home Takeoff point: %f", home_takeoff);
            ROS_INFO("Current Height above takeoff position: %f", home_takeoff);
            ROS_INFO("Difference: %f m", altitude_above_takeoff - home_takeoff);

            ROS_ERROR("Takeoff failed.");

            return false;
        }

        else
        {
            start_time = ros::Time::now();
            ROS_INFO("Successful takeoff!");
            ros::spinOnce();
        }

    }
    return true;

}

bool Control::land()
{

    float uav_land = computeTimeToLand();
    ros::Time start_time = ros::Time::now();
    if(uav_model == UAV::Type::N3 || UAV::Type::A3)
    {
        if (!takeoffLand(dji_sdk::DroneTaskControl::Request::TASK_LAND))
        {
            return false;
        }

        ros::Duration(0.01).sleep();
        ros::spinOnce();

        while (flight_status != DJISDK::FlightStatus::STATUS_ON_GROUND &&
                display_mode != DJISDK::DisplayMode::MODE_AUTO_LANDING &&
                ros::Time::now() - start_time < ros::Duration(uav_land))
        {
            ros::Duration(0.01).sleep();
            ros::spinOnce();
        }

        if (ros::Time::now() - start_time > ros::Duration(uav_land))
        {
            ROS_ERROR("Landing Failed");
            return false;
        }
        else
        {
            start_time = ros::Time::now();
            ROS_INFO("Drone Landing ...");
            ros::spinOnce();
        }
        // Final check: Finished Landing
        while ((display_mode == DJISDK::DisplayMode::MODE_AUTO_LANDING) &&
                ros::Time::now() - start_time < ros::Duration(N3_land))
        {
            ros::Duration(0.01).sleep();
            ros::spinOnce();
        }

        if (flight_status != DJISDK::FlightStatus::STATUS_ON_GROUND)
        {
            ROS_ERROR_ONCE("Failed Landing!");
            return false;
        }
        else
        {
            ROS_INFO("A3/N3 Drone has successfully landed");
            start_time = ros::Time::now();
            ros::spinOnce();
        }
    }
    else if(uav_model == UAV::Type::M100)
    {

        float m100_time_to_land = computeTimeToLand() + 5;
        float home_altitude = current_gps_location.altitude;

        if (!takeoffLand(dji_sdk::DroneTaskControl::Request::TASK_LAND))
        {
            return false;
        }

        ros::Duration(0.01).sleep();
        ros::spinOnce();

        // compute wait time to be based on relative altitude of drone / drone descent speed : which is at 1 m/s
        while (ros::Time::now() - start_time < ros::Duration(rosTime_to_land) 
              || flight_status == DJISDK::M100FlightStatus::M100_STATUS_LANDING)
        {
            ros::Duration(0.01).sleep();
            ros::spinOnce();
        }

        if (flight_status != DJISDK::M100FlightStatus::M100_STATUS_ON_GROUND ||
            current_gps_location.altitude - home_altitude > 1.0)
        {
            ROS_ERROR_ONCE("Landing failed.");
            return false;
        }
        else
        {
            start_time = ros::Time::now();
            ROS_INFO_ONCE("Successful Landing!");
            ros::spinOnce();
        }
    }

    return true;

}

void Control::rth()
{
    dji_sdk::DroneTaskControl droneTaskControl;
    droneTaskControl.request.task = dji_sdk::DroneTaskControl::Request::TASK_GOHOME;
    drone_task_service.call(droneTaskControl);
    ROS_INFO("UAV Returning Home!");

    if (!droneTaskControl.response.result)
    { 
        ROS_ERROR("RTH Error");
    }
}

float Control::computeTimeToLand()
{

    float uav_land_speed;
    float current_altitude;

    if(uav_model == UAV::Type::M100)
    {
        uav_land_speed = 1;
        current_altitude = altitude_above_takeoff;
        land_time = (current_altitude/uav_land_speed);

        // Maximum time M100 drone will take to land from an altitude of 100 metres is 55 seconds..
        // Tested in simulator
        //TODO use Velocity Z to compute variable time to land)

        if (land_time > 55)
        {
            land_time = 55;
        }

        ROS_INFO_THROTTLE(5, "Time required to land is %f", land_time);

    }

    else if(uav_model == UAV::Type::N3 || UAV::Type::A3)
    {
        droneLandSpeed = fabs(velocity_data.vector.z);
        current_height = altitude_above_takeoff;

        if (current_height >= 40)
        {
            land_time = (current_height / droneLandSpeed) + 50;
        }
        else
        {
            land_time = (current_height / droneLandSpeed);
        }

        if (std::isnan(land_time))
        {
            ROS_INFO_ONCE("NAN result, setting to  zero");
            land_time = 0;
        }

        // empirically, time to land from the N3 at 100m is 90 seconds.
        // so cap time to land to 90 seconds
        if (land_time > 90)
        {
            land_time = 90;
        }

        if (!std::isinf(land_time))
        {
            ROS_INFO_THROTTLE(5, "Time required to land is %f", land_time); 
        }
    }
    return land_time + 5;
}

void Control::takeoffLand(int task)
{
    dji_sdk::DroneTaskControl droneTaskControl;
    droneTaskControl.request.task = task;
    drone_task_service.call(droneTaskControl);

    if (!droneTaskControl.response.result)
    {
        ROS_ERROR("UAV Takoff Land Service failed");
        return false;
    }
    
    return true;
}