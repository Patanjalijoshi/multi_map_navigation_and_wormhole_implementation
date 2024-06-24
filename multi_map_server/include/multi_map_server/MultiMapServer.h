#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/String.h>
#include <nav_msgs/LoadMap.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <multi_map_server/NavigateToGoalAction.h>
#include <sqlite3.h>
#include <string.h>
//#include <ros/package.h>

typedef actionlib::SimpleActionServer<multi_map_server::NavigateToGoalAction> server;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> moveBaseClient;

class NavigateToGoalAction
{ 
private:
    ros::NodeHandle m_nh;

    ros::Subscriber mapSub;

    ros::ServiceClient loadMapSrvc;
    nav_msgs::LoadMap m_loadMapSrv;
    
    ros::Publisher initPosePub;
    geometry_msgs::PoseWithCovarianceStamped m_initPose;

    ros::Subscriber currentPoseSub;

    server m_as;
    std::string m_actionName;
    multi_map_server::NavigateToGoalActionFeedback m_feedBack;
    multi_map_server::NavigateToGoalResult m_result;

    moveBaseClient m_moveBaseActionClient;

    std::string m_currentMap;

    geometry_msgs::Pose m_currentMapWormHoleLocation;
    geometry_msgs::Pose m_newMapInitPose;
    geometry_msgs::Pose m_currentRobotPose;

    geometry_msgs::Pose m_maxMap1;
    geometry_msgs::Pose m_minMap1;

    geometry_msgs::Pose m_maxMap2;
    geometry_msgs::Pose m_minMap2;

    multi_map_server::NavigateToGoalFeedback m_feedback;
    bool Goalreached = false;
    bool success = true;

    sqlite3* m_db;
    sqlite3_stmt* m_query;
    bool m_dbAccess;
    std::string m_sql;

    bool m_goalReached;
    bool m_success; 
    bool m_GoalInMap;

    // std::string m_packagePath;
    // std::string m_mapPath;

public:
//all member functions here

NavigateToGoalAction(std::string name); //constructor

~NavigateToGoalAction(void);  //destructor

void currentPoseCallBack(const geometry_msgs::Pose::ConstPtr& msg); //publishes the current pose of the bot

void currentMapCallBack(const std_msgs::String::ConstPtr& msg);   //function that gets the current map

bool getMapDetails(const std::string& map_name, geometry_msgs::Pose& pose); //gets map details from the database

bool sendMoveBaseGoal(const geometry_msgs::Pose& goal_pose); //function to send a goal to movebase 

void navigateToGoalServerCallBack(const multi_map_server::NavigateToGoalGoalConstPtr& goal); //callback function of the action server

bool goalInMap(const multi_map_server::NavigateToGoalGoalConstPtr& goal);

};
