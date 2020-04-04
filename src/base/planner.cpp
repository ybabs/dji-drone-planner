








void Planner::missionParamCallback(const gcs_msgs::Missionparameters::ConstPtr &msg)
{
   uav_speed = msg->UavSpeed;
   mission_end_action = msg->missionEndAction;
}

void Planner::missionPauseCallback(const std_msgs::UInt8::ConstPtr& msg)
{
   play_pause = msg->data
}

void Planner::waypointCallback(const gcs_msgs::Waypoint::ConstPtr &msg)
{
   waypoint = msg;
}

void Planner::missionActionCallback(const gcs_msgs::Action::ConstPtr &msg)
{
    drone_action = msg->data
}
