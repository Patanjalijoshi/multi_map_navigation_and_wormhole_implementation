#include "multi_map_server/MultiMapServer.h"

NavigateToGoalAction::NavigateToGoalAction(std::string name) //constructor
    : m_as(m_nh, name, boost::bind(&NavigateToGoalAction::navigateToGoalServerCallBack, this, _1), false)
    , m_actionName(name)
    , m_moveBaseActionClient("move_base", true)
  {
    m_as.start();
    mapSub = m_nh.subscribe("/current_map", 10, &NavigateToGoalAction::currentMapCallBack, this);
    currentPoseSub = m_nh.subscribe<geometry_msgs::Pose>("/robot_pose", 10, &NavigateToGoalAction::currentPoseCallBack, this);
    loadMapSrvc = m_nh.serviceClient<nav_msgs::LoadMap>("/change_map");
    initPosePub = m_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose",1);
  }

NavigateToGoalAction::~NavigateToGoalAction(void){ } //destructor

void NavigateToGoalAction::currentPoseCallBack(const geometry_msgs::Pose::ConstPtr& msg)
{
    m_currentRobotPose = *msg;
}

void NavigateToGoalAction::currentMapCallBack(const std_msgs::String::ConstPtr& msg)
{
    m_currentMap = msg->data;
}

bool NavigateToGoalAction::getMapDetails(const std::string& map_name, geometry_msgs::Pose& pose)
{
    
    m_dbAccess = sqlite3_open("/home/patanjali/maps.db", &m_db);
    if (m_dbAccess)
    {
      ROS_ERROR("Can't open database: %s", sqlite3_errmsg(m_db));
      return false;
    }

    m_sql =
        "SELECT position_x, position_y, position_z, orientation_x, orientation_y, orientation_z, orientation_w FROM "
        "maps WHERE id = ?";

    m_dbAccess = sqlite3_prepare_v2(m_db, m_sql.c_str(), -1, &m_query, 0);
    if (m_dbAccess != SQLITE_OK)
    {
      ROS_ERROR("SQL error: %s", sqlite3_errmsg(m_db));
      sqlite3_close(m_db);
      return false;
    }

    sqlite3_bind_text(m_query, 1, map_name.c_str(), -1, SQLITE_STATIC);

    if (sqlite3_step(m_query) == SQLITE_ROW)
    {
      pose.position.x = sqlite3_column_double(m_query, 0);
      pose.position.y = sqlite3_column_double(m_query, 1);
      pose.position.z = sqlite3_column_double(m_query, 2);
      pose.orientation.x = sqlite3_column_double(m_query, 3);
      pose.orientation.y = sqlite3_column_double(m_query, 4);
      pose.orientation.z = sqlite3_column_double(m_query, 5);
      pose.orientation.w = sqlite3_column_double(m_query, 6);
    }
    else
    {
      ROS_ERROR("No data found for map name: %s", map_name.c_str());
      sqlite3_finalize(m_query);
      sqlite3_close(m_db);
      return false;
    }

    sqlite3_finalize(m_query);
    sqlite3_close(m_db);
    return true;
}

bool NavigateToGoalAction::sendMoveBaseGoal(const geometry_msgs::Pose& goal_pose)
{

    while (!m_moveBaseActionClient.waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal move_base_goal;
    move_base_goal.target_pose.header.frame_id = "map";
    move_base_goal.target_pose.header.stamp = ros::Time::now();
    move_base_goal.target_pose.pose = goal_pose;

    m_moveBaseActionClient.sendGoal(move_base_goal);

    while (ros::ok())
    {
      if (m_moveBaseActionClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        ROS_INFO("Goal reached successfully.");
        return true;
      }
      else if (m_moveBaseActionClient.getState() == actionlib::SimpleClientGoalState::ABORTED ||
               m_moveBaseActionClient.getState() == actionlib::SimpleClientGoalState::REJECTED)
      {
        ROS_ERROR("Failed to reach goal.");
        return false;
      }

      ros::Duration(0.1).sleep();  // Sleep for a short duration to prevent busy-waiting
    }

    return false;  // In case ROS shuts down or the loop exits unexpectedly

}

bool NavigateToGoalAction::goalInMap(const multi_map_server::NavigateToGoalGoalConstPtr& goal)
{
  m_maxMap1.position.x = 9.0;
  m_maxMap1.position.y = 9.0;
  m_minMap1.position.x = -9.0;
  m_minMap1.position.y = -9.0;

  m_maxMap2.position.x = 6.0;
  m_maxMap2.position.y = 15.0;
  m_minMap2.position.x = 2.0;
  m_minMap2.position.y = 5.0;

  const std::string map = goal->map_name;

  if((map == "map_1") && (goal->goal_pose.position.x < m_maxMap1.position.x) && (goal->goal_pose.position.x > m_minMap1.position.x) 
                      && (goal->goal_pose.position.y < m_maxMap1.position.y) && (goal->goal_pose.position.y > m_minMap1.position.y))
  {
    ROS_INFO("Given Goal is in the map");
    return true;
  }
  else if((map == "map_2") && (goal->goal_pose.position.x < m_maxMap2.position.x) && (goal->goal_pose.position.x > m_minMap2.position.x) 
                           && (goal->goal_pose.position.y < m_maxMap2.position.y) && (goal->goal_pose.position.y > m_minMap2.position.y))
  { 
    ROS_INFO("Given Goal is in the map");
    return true;
  }
  
    ROS_INFO("Failed, Given Goal is Outside The Map");
    return false;
  
}

void NavigateToGoalAction::navigateToGoalServerCallBack(const multi_map_server::NavigateToGoalGoalConstPtr& goal)
{
    ros::Rate r(1); // 1 Hz rate
    m_goalReached = false;
    m_success = true; 
    m_GoalInMap = goalInMap(goal);

    
    if (m_currentMap == goal->map_name) // Directly navigate to the goal in the current map
    {  
      

        while (!m_goalReached)
        {
            if (m_as.isPreemptRequested() || !ros::ok()|| !m_GoalInMap)
            {
                ROS_INFO("%s: Preempted", m_actionName.c_str());
                m_as.setPreempted();
                m_success = false;
                break;
            }

            m_goalReached = sendMoveBaseGoal(goal->goal_pose);

            // Publish feedback
            m_feedback.current_pose = m_currentRobotPose;
            m_as.publishFeedback(m_feedback);

            r.sleep();
        }

        if (m_success)
        {
            ROS_INFO("Goal reached");
            m_result.result = true;
            m_as.setSucceeded(m_result);
        }
        else
        {
            m_result.result = false;
            m_as.setAborted(m_result);
        }
    }
    else //goal map and the current map are different 
    {
        
        ROS_INFO("Switching maps");

        if (!getMapDetails(m_currentMap, m_currentMapWormHoleLocation)) //retriving details from the db
        {
            ROS_ERROR("Failed to retrieve current map details."); //retriving details about the wormhole location of the current map
            m_result.result = false;
            m_as.setAborted(m_result);
            return;
        }

        if (!getMapDetails(goal->map_name, m_newMapInitPose))   //retreving details to localize in the goal map
        {
            ROS_ERROR("Failed to retrieve goal map details.");
            m_result.result = false;
            m_as.setAborted(m_result);
            return;
        }

        ROS_INFO("GOT the details from DB");
        
        m_success = sendMoveBaseGoal(m_currentMapWormHoleLocation);
        
        while (!m_success)
        {
            if (m_as.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("%s: Preempted", m_actionName.c_str());
                m_as.setPreempted();
                m_success = false;
                break;
            }

            // Publish feedback
            m_feedback.current_pose = m_currentRobotPose;
            m_as.publishFeedback(m_feedback);

            r.sleep();
        }

        if (m_success)
        {
            m_loadMapSrv.request.map_url = "/home/patanjali/catkin_ws/src/AR100/anscer_navigation/maps/" + goal->map_name + ".yaml";
            loadMapSrvc.call(m_loadMapSrv);
            ROS_INFO_STREAM(m_loadMapSrv.response.result);

            m_initPose.pose.pose = m_newMapInitPose;
            initPosePub.publish(m_initPose);

            

            while (!m_success)
            {
                if (m_as.isPreemptRequested() || !ros::ok() || !m_GoalInMap)
                {
                    ROS_INFO("%s: Preempted", m_actionName.c_str());
                    m_as.setPreempted();
                    m_success = false;
                    break;
                }
                m_success = sendMoveBaseGoal(goal->goal_pose);
                // Publish feedback
                m_feedback.current_pose = m_currentRobotPose;
                m_as.publishFeedback(m_feedback);

                r.sleep();
            }
        }

        if (m_success)
        {
            ROS_INFO("%s: Succeeded", m_actionName.c_str());
            m_result.result = true;
            m_as.setSucceeded(m_result);
        }
        else
        {
            ROS_INFO("%s: Failed", m_actionName.c_str());
            m_result.result = false;
            m_as.setAborted(m_result);
        }
    }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "navigate_to_goal");

  NavigateToGoalAction navigate_to_goal(ros::this_node::getName());
  ros::spin();

  return 0;
}