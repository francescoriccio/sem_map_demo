#include <ros/ros.h>
#include <math.h> 
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <pnp_msgs/PNPAction.h>
#include <pnp_msgs/PNPCondition.h>
#include <pnp_ros/PNPActionServer.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <living_at_diag/FollowPersonAction.h>
#include <living_at_diag/GetCloserAction.h>
#include <living_at_diag/GoToTargetAction.h>
#include <rococo_navigation/FollowCorridorAction.h>
#include <rococo_navigation/TurnAction.h>

#include <boost/thread/thread.hpp>
#include <boost/algorithm/string.hpp>

#include "MyMovementActions.h"
#include "MyActions.h"

#define radians(a) ((a)/180.0*M_PI)
#define degrees(a) ((a)/M_PI*180.0)

// defined in robotpose.cpp
bool getRobotPose(std::string robotname, double &x, double &y, double &th_rad);

using namespace std;

std::string followCorridor_topic        = "followCorridor";
std::string followPerson_topic          = "followPerson";
std::string getCloser_topic             = "getCloser";
std::string goToTarget_topic            = "goToTarget";
std::string movebase_topic              = "move_base";
std::string turn_topic                  = "turn";

actionlib::SimpleActionClient<living_at_diag::FollowPersonAction> *ac_followPerson              = NULL;
actionlib::SimpleActionClient<living_at_diag::GetCloserAction> *ac_getCloser                    = NULL;
actionlib::SimpleActionClient<living_at_diag::GoToTargetAction> *ac_goToTarget                  = NULL;
actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> *ac_movebase                      = NULL;  
actionlib::SimpleActionClient<rococo_navigation::FollowCorridorAction> *ac_followCorridor       = NULL;
actionlib::SimpleActionClient<rococo_navigation::TurnAction> *ac_turn                           = NULL;

// Mathematical Functions
float norm180(float a){
  while (a>180)
    a = a-360;
  while (a<=-180)
    a = a+360;
  return a;
}

// Support functions
void exec_followCorridor(float GX, float GY, bool* run){
  cout << "### Executing Follow Corridor to: " << GX <<","<< GY <<endl;
  // Define the action client (true: we want to spin a thread)
  if (ac_followCorridor==NULL) {
    ac_followCorridor = new actionlib::SimpleActionClient<rococo_navigation::FollowCorridorAction>(followCorridor_topic, true);  

    // Wait for the action server to come up
    while(!ac_followCorridor->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for Follow Corridor action server to come up");
    }
  }
  
  // Cancel all goals (JUST IN CASE SOME GOAL WAS NOT CLOSED BEFORE)
  ac_followCorridor->cancelAllGoals(); 
  ros::Duration(0.5).sleep(); // wait .5 sec
  
  // Creating the goal
  rococo_navigation::FollowCorridorGoal goal;
  goal.target_X = GX;
  goal.target_Y = GY;

  // Send the goal
  ROS_INFO("Sending goal");
  ac_followCorridor->sendGoal(goal);

  // Wait for termination
  while (!ac_followCorridor->waitForResult(ros::Duration(1.0))) {
    if (!*run){
      ac_followCorridor->cancelGoal();
      ROS_INFO("Follow Corridor Stopped");
    }
  }
  
  ros::Duration(0.250).sleep();
  
  // Print result
  if (ac_followCorridor->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Follow Corridor successful");
}

void exec_gotopose(string robot_name, float GX, float GY, float GTh, bool *run) {
  double d_threshold=1, d=d_threshold+1.0;
  double RX,RY,RTh;
  
  if (ac_movebase==NULL) {
    // Define the action client (true: we want to spin a thread)
    ac_movebase = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>(movebase_topic, true);  

    // Wait for the action server to come up
    while(!ac_movebase->waitForServer(ros::Duration(5.0))){
                    ROS_INFO("Waiting for move_base action server to come up");
    }
  }

  // Cancel all goals (JUST IN CASE SOME GOAL WAS NOT CLOSED BEFORE)
  ac_movebase->cancelAllGoals(); 
  ros::Duration(0.5).sleep(); // wait .5 secs
  
  // Read time
  double secs = ros::Time::now().toSec();
  
  while (secs==0) // NEEDED OTHERWISE CLOCK WILL BE 0 AND GOAL_ID IS NOT SET CORRECTLY 
  {  
    ROS_ERROR_STREAM("Time is null: " << ros::Time::now());
    ros::Duration(1.0).sleep();
    secs =ros::Time::now().toSec();
  }

  // Set the goal (MAP frame)
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id      = "/map";
  goal.target_pose.header.stamp         = ros::Time::now();
  goal.target_pose.pose.position.x      = GX;
  goal.target_pose.pose.position.y      = GY;
  goal.target_pose.pose.orientation.z   = sin(radians(GTh)/2);
  goal.target_pose.pose.orientation.w   = cos(radians(GTh)/2);

  // Send the goal
  ROS_INFO("Sending goal");
  ac_movebase->sendGoal(goal);

  // Wait for termination
  while (!ac_movebase->waitForResult(ros::Duration(0.5)) && (d>d_threshold)) {
    if (!*run){
      cout<< "cancelling_goal"<< endl;
      ac_movebase->cancelAllGoals();
    }
    else if (getRobotPose(robot_name, RX, RY, RTh))
      d = fabs(GX-RX)+fabs(GY-RY)+fabs(GTh-RTh);
  }

  // Print result
  if (!(*run))
    ROS_INFO("External interrupt!!!");
  else if (d<=d_threshold) 
    ROS_INFO("Target reached (Internal check)");
  else if (ac_movebase->getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The base failed to reach the move_base goal for some reason");
  else
    ROS_INFO("!move_base goal reached!");

}

void exec_turn(string params, bool* run, bool global_angle=false){
   cout << "### Executing Turn ... " << params << endl;
   
  // Define the action client (true: we want to spin a thread)
  if (ac_turn==NULL) {
    cout << "defining new turn action client" << endl;
    ac_turn = new actionlib::SimpleActionClient<rococo_navigation::TurnAction>(turn_topic, true);  
    cout << "defining new action client created" << endl;
    // Wait for the action server to come up
    while(!ac_turn->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for Turn action server to come up");
    }
  }
  
  // Cancel all goals (JUST IN CASE SOME GOAL WAS NOT CLOSED BEFORE)
  ac_turn->cancelAllGoals(); 
  ros::Duration(0.5).sleep(); // wait .5 sec
  cout << "cancelled all turn actions" << endl;
  
  float GTh = 0;
  if (params == "left")
    GTh= 90; 
  else if (params == "right")
    GTh= -90; 
  else if (params == "180" || params == "around")
    GTh= 180; 
  else if (params == "right45" || params == "rightabit")
    GTh= -45; 
  else if (params == "left45" || params == "leftabit")
    GTh= 45;
  else
    ROS_ERROR_STREAM("Unknwon Turn Parameter " << params);
  
  // Set the goal
  rococo_navigation::TurnGoal goal;
  goal.target_angle = GTh;  // deg
  if (global_angle)
    goal.absolute_relative_flag = "GLOB";
  else
    goal.absolute_relative_flag = "REL";
  goal.max_ang_vel = 10.0;  // deg/s

  // Send the goal
  ROS_INFO("Sending goal");
  ac_turn->sendGoal(goal);

  // Wait for termination
  while (!ac_turn->waitForResult(ros::Duration(1.0))) {
    if (!*run){
      ac_turn->cancelGoal();
      ROS_INFO("Turn Stopped");
    }
  }
  // ROS_INFO_STREAM("Finished [" << ac.getState().toString() << "]");
  ros::Duration(0.250).sleep();
  
  // Print result
  if (ac_turn->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Turn successful");
}

void exec_turn_to(string robot_name, float GX, float GY, bool* run){
  std::ostringstream ss;
  float GTh = 0;
  
  double RX,RY,RTh;
  if(getRobotPose(robot_name, RX, RY, RTh)){
    GTh = atan2(GY-RY,GX-RX);
    GTh = degrees(GTh);
    GTh = norm180(GTh);
  }
  else
    ROS_ERROR_STREAM("Could not retrieve robot pose");
  
  ss << GTh;
  exec_turn(ss.str(), run, true);
  
}

// Action implementation
void followCorridor(string robot_name, string params, bool *run){
  int i         = params.find("_");     
  int j         = params.find("_",i+1);
  float GX      = atof(params.substr(0,i).c_str());
  float GY      = atof(params.substr(i+1,j).c_str());
  string GTh    = params.substr(j+1).c_str();
  
  exec_followCorridor(GX, GY, run);
  exec_turn(GTh, run);

  if (*run)
    cout << "### Finished Follow Corridor " << endl;
  else
    cout << "### Aborted Follow Corridor  " << endl;
}
  
void followPerson(string robot_name, string params, bool *run){
cout << "### Executing Follow Person ... " << params << endl;

  // Define the action client (true: we want to spin a thread)
  if (ac_followPerson==NULL) {
    ac_followPerson = new actionlib::SimpleActionClient<living_at_diag::FollowPersonAction>(followPerson_topic, true);  

    // Wait for the action server to come up
    while(!ac_followPerson->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for Follow Person action server to come up");
    }
  }
  
  // Cancel all goals (JUST IN CASE SOME GOAL WAS NOT CLOSED BEFORE)
  ac_followPerson->cancelAllGoals(); 
  ros::Duration(0.5).sleep(); // wait .5 sec

  // Send the goal
  ROS_INFO("Sending goal");
  living_at_diag::FollowPersonGoal goal;
  ac_followPerson->sendGoal(goal);

  // Wait for termination
  while (!ac_followPerson->waitForResult(ros::Duration(1.0))) {
    if (!*run){
      ac_followPerson->cancelGoal();
      ROS_INFO("Follow Person Stopped");
    }
  }
  
  ros::Duration(0.250).sleep();
  
  // Print result
  if (ac_followPerson->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Follow Person successful");

  if (*run)
      cout << "### Finished Follow Person " << endl;
  else
      cout << "### Aborted Follow Person  " << endl;
}

void getCloser(string robot_name, string params, bool *run){
cout << "### Executing Get Closer ... " << params << endl;

  // Define the action client (true: we want to spin a thread)
  if (ac_getCloser==NULL) {
    ac_getCloser = new actionlib::SimpleActionClient<living_at_diag::GetCloserAction>(getCloser_topic, true);  

    // Wait for the action server to come up
    while(!ac_getCloser->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for Get Closer action server to come up");
    }
  }
  
  // Cancel all goals (JUST IN CASE SOME GOAL WAS NOT CLOSED BEFORE)
  ac_getCloser->cancelAllGoals(); 
  ros::Duration(0.5).sleep(); // wait .5 sec

  // Send the goal
  ROS_INFO("Sending goal");
  living_at_diag::GetCloserGoal goal;
  ac_getCloser->sendGoal(goal);

  // Wait for termination
  while (!ac_getCloser->waitForResult(ros::Duration(1.0))) {
    if (!*run){
      ac_getCloser->cancelGoal();
      ROS_INFO("Get Closer Stopped");
    }
  }
  
  ros::Duration(0.250).sleep();
  
  // Print result
  if (ac_getCloser->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Get Closer successful");

  if (*run)
      cout << "### Finished Get Closer" << endl;
  else
      cout << "### Aborted Get Closer" << endl;
}

void gotopose(string robot_name, string params, bool *run){
  cout << "### Executing Gotopose ... " << params << endl;
  
  int i         = params.find("_");     
  int j         = params.find("_",i+1);
  float GX      = atof(params.substr(0,i).c_str());
  float GY      = atof(params.substr(i+1,j).c_str());
  float GTh     = atof(params.substr(j+1).c_str());
  
  exec_gotopose(robot_name, GX, GY, GTh, run);

  if (*run)
    cout << "### Finished Gotopose " << endl;
  else
    cout << "### Aborted Gotopose  " << endl;
}

void goAndLookAt(string robot_name, string params, bool *run){
cout << "### Executing Go And Look At ... " << params << endl;
  int i         = params.find("_");
  float GX      = atof(params.substr(0,i).c_str());
  float GY      = atof(params.substr(i+1).c_str());

  exec_followCorridor(GX, GY, run);
  exec_turn_to(robot_name, GX, GY, run);
  
  if (*run)
      cout << "### Finished Go And Look At " << endl;
  else
      cout << "### Aborted Go And Look At  " << endl;
}

void home(string robot_name, string params, bool *run) {
  double GX, GY, GTh;
  if(!ros::param::get("/home_x", GX))
  {
    ROS_ERROR("Can't read parameter /home_x. Using the default one");
    GX = 8.0;
  }
  if (!ros::param::get("/home_y", GY))
  {
    ROS_ERROR("Can't read parameter /home_y. Using the default one");
    GY = 2.0;
  }
  if(!ros::param::get("/home_th", GTh))
  {
    ROS_ERROR("Can't read parameter /home_th. Using the default one");
    GTh = 0;
  }
  
  cout << "### Executing Home with coords (" << GX << "," << GY << "," << GTh << ")... " << endl;

  exec_gotopose(robot_name, GX, GY, GTh, run);

  if (*run)
    cout << "### Finished Home " << endl;
  else
    cout << "### Aborted Home  " << endl;
}

void turn(string robot_name, string params, bool *run){
  
  exec_turn(params, run);

  if (*run)
    cout << "### Finished Turn " << endl;
  else
    cout << "### Aborted Turn  " << endl;
}
