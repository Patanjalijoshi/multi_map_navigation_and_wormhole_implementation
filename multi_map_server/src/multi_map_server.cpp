// // #include <ros/ros.h>
// // #include <actionlib/server/simple_action_server.h>
// // #include <geometry_msgs/Pose.h>
// // #include <geometry_msgs/PoseWithCovarianceStamped.h>
// // #include <std_msgs/String.h>
// // #include <nav_msgs/LoadMap.h>
// // #include <move_base_msgs/MoveBaseAction.h>
// // #include <actionlib/client/simple_action_client.h>
// // #include <multi_map_server/NavigateToGoalAction.h>
// // #include <sqlite3.h>

// // //#include<MultiMapServer.h>

// // typedef actionlib::SimpleActionServer<multi_map_server::NavigateToGoalAction> Server;

// // class NavigateToGoalAction
// // {
// // protected:
// //   ros::NodeHandle nh_;
// //   Server as_;  // Action server
// //   std::string action_name_;
// //   multi_map_server::NavigateToGoalFeedback feedback_;
// //   multi_map_server::NavigateToGoalResult result_;

// //   ros::Subscriber map_subscriber_;
// //   ros::Subscriber pose_subscriber_;
// //   std::string current_map_;
// //   geometry_msgs::Pose current_pose_;
// //   typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
// //   MoveBaseClient move_base_client_;
// //   ros::ServiceClient client = nh_.serviceClient<nav_msgs::LoadMap>("/change_map");
// //   nav_msgs::LoadMap srv;
// //   ros::Publisher pub = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
// //   geometry_msgs::PoseWithCovarianceStamped m_initpose;
// //   geometry_msgs::Pose current_map_pose;
// //   geometry_msgs::Pose new_map_pose;
// public:
//   NavigateToGoalAction(std::string name)//
//     : as_(nh_, name, boost::bind(&NavigateToGoalAction::executeCB, this, _1), false)
//     , action_name_(name)
//     , move_base_client_("move_base", true)
//   {
//     as_.start();
//     map_subscriber_ = nh_.subscribe("current_map", 10, &NavigateToGoalAction::mapCB, this);
//     pose_subscriber_ = nh_.subscribe("robot_pose", 10, &NavigateToGoalAction::poseCB, this);
//   }

//   ~NavigateToGoalAction(void)//
//   {
//   }

//   void mapCB(const std_msgs::String::ConstPtr& msg)//
//   {
//     current_map_ = msg->data;
//   }

//   void executeCB(const multi_map_server::NavigateToGoalGoalConstPtr& goal)
//   {
//     current_pose_ = *msg;
//   }

//   bool getMapDetails(const std::string& map_name, geometry_msgs::Pose& pose)//
//   {
//     sqlite3* db;
//     sqlite3_stmt* stmt;
//     int rc = sqlite3_open("/home/patanjali/maps.db", &db);
//     if (rc)
//     {
//       ROS_ERROR("Can't open database: %s", sqlite3_errmsg(db));
//       return false;
//     }

//     std::string sql =
//         "SELECT position_x, position_y, position_z, orientation_x, orientation_y, orientation_z, orientation_w FROM "
//         "maps WHERE id = ?";
//     rc = sqlite3_prepare_v2(db, sql.c_str(), -1, &stmt, 0);
//     if (rc != SQLITE_OK)
//     {
//       ROS_ERROR("SQL error: %s", sqlite3_errmsg(db));
//       sqlite3_close(db);
//       return false;
//     }

//     sqlite3_bind_text(stmt, 1, map_name.c_str(), -1, SQLITE_STATIC);

//     if (sqlite3_step(stmt) == SQLITE_ROW)
//     {
//       pose.position.x = sqlite3_column_double(stmt, 0);
//       pose.position.y = sqlite3_column_double(stmt, 1);
//       pose.position.z = sqlite3_column_double(stmt, 2);
//       pose.orientation.x = sqlite3_column_double(stmt, 3);
//       pose.orientation.y = sqlite3_column_double(stmt, 4);
//       pose.orientation.z = sqlite3_column_double(stmt, 5);
//       pose.orientation.w = sqlite3_column_double(stmt, 6);
//     }
//     else
//     {
//       ROS_ERROR("No data found for map name: %s", map_name.c_str());
//       sqlite3_finalize(stmt);
//       sqlite3_close(db);
//       return false;
//     }

//     sqlite3_finalize(stmt);
//     sqlite3_close(db);
//     return true;
//   }

//   bool sendMoveBaseGoal(const geometry_msgs::Pose& goal_pose)
//   {
//     while (!move_base_client_.waitForServer(ros::Duration(5.0)))
//     {
//       ROS_INFO("Waiting for the move_base action server to come up");
//     }

//     move_base_msgs::MoveBaseGoal move_base_goal;
//     move_base_goal.target_pose.header.frame_id = "map";
//     move_base_goal.target_pose.header.stamp = ros::Time::now();
//     move_base_goal.target_pose.pose = goal_pose;

//     move_base_client_.sendGoal(move_base_goal);

//     while (ros::ok())
//     {
//       if (move_base_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
//       {
//         ROS_INFO("Goal reached successfully.");
//         return true;
//       }
//       else if (move_base_client_.getState() == actionlib::SimpleClientGoalState::ABORTED ||
//                move_base_client_.getState() == actionlib::SimpleClientGoalState::REJECTED)
//       {
//         ROS_ERROR("Failed to reach goal.");
//         return false;
//       }

//       ros::Duration(0.1).sleep();  // Sleep for a short duration to prevent busy-waiting
//     }

//     return false;  // In case ROS shuts down or the loop exits unexpectedly
//   }

//   void executeCB(const multi_map_server::NavigateToGoalGoalConstPtr& goal) //
//   {
//     ros::Rate r(1);
//     multi_map_server::NavigateToGoalFeedback m_feedback;
//     bool Goalreached = false;
//     bool success = true;

//     if (current_map_ == goal->map_name)
//     {
//       Goalreached = sendMoveBaseGoal(goal->goal_pose);
//       if (Goalreached)
//       {
//         ROS_INFO("Wormhole reached");

//       }
//     }

//     else if (current_map_ != goal->map_name)
//     {
//       ROS_INFO("Switch map");
      

//       if (getMapDetails(current_map_, current_map_pose))
//       {
//         ROS_INFO_STREAM("Current map details: " << current_map_pose);
//       }
//       else
//       {
//         ROS_ERROR("Failed to retrieve current map details.");
//         result_.result = false;
//         as_.setAborted(result_);
//         return;
//       }

//       if (getMapDetails(goal->map_name, new_map_pose))
//       {
//         ROS_INFO_STREAM("Goal map details: " << new_map_pose);
//       }
//       else
//       {
//         ROS_ERROR("Failed to retrieve goal map details.");
//         result_.result = false;
//         as_.setAborted(result_);
//         return;
//       }

//       // Add logic to handle map switching if necessary
//       ROS_INFO("GOT the details from DB");

//       success = sendMoveBaseGoal(current_map_pose);
//       if (success)
//       {
//         srv.request.map_url = "/home/patanjali/catkin_ws/src/AR100/anscer_navigation/maps/" + goal->map_name  + ".yaml";
//         client.call(srv);
//         ROS_INFO_STREAM(srv.response.result);

//         m_initpose.pose.pose = new_map_pose;
//         pub.publish(m_initpose);

//         success = sendMoveBaseGoal(goal->goal_pose);
//       }
//       else
//       {
//         ROS_INFO("%s: Aborted, current map is different from goal map.", action_name_.c_str());
//         result_.result = false;
//         as_.setAborted(result_);
//         return;
//       }

//       if (success)
//       {
//         ROS_INFO("%s: Succeeded", action_name_.c_str());
//         result_.result = true;
//         as_.setSucceeded(result_);
//       }
//       else
//       {
//         ROS_INFO("%s: Failed", action_name_.c_str());
//         result_.result = false;
//         as_.setAborted(result_);
//       }
//     }
//   }
// };
// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "navigate_to_goal");

//   NavigateToGoalAction navigate_to_goal(ros::this_node::getName());
//   ros::spin();

//   return 0;
// }
