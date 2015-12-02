#include <ros/ros.h>
#include <math.h> 
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <pnp_msgs/PNPAction.h>
#include <pnp_ros/PNPActionServer.h>
#include <pnp_msgs/PNPCondition.h>
#include <pnp_msgs/PNPSetVariableValue.h>
#include <std_msgs/String.h>
#include <living_at_diag/WaitForUserAction.h>

#include <sapienzbot_reasoning/RemoveObject.h>
#include <sapienzbot_reasoning/EvaluateCondition.h>
#include <sapienzbot_reasoning/GroundCommand.h>
#include <sapienzbot_dialog/SaySrv.h>
#include <sapienzbot_dialog/QueryLoop.h>
#include <combine_pnp/UpdateAction.h>
#include <combine_pnp/GeneratePlanSrv.h>
#include <dot_detector/TakePicture.h>

#include <boost/thread/thread.hpp>
#include <boost/algorithm/string.hpp>

#include "MyActions.h"
#include "MyMovementActions.h"
#include <algorithm>

using namespace std;

std::string waitForUser_topic        = "waitForUser";
actionlib::SimpleActionClient<living_at_diag::WaitForUserAction> *ac_waitForUser    = NULL;


static unsigned memQueryLoopIndex = -1;
string last_object_type;

bool string_stars_with(string a, string b){
    return (a.substr(0,b.length()) == b);
}

void exec_waitForUser(bool* run){
  cout << "### Executing Wait For User" <<endl;
  // Define the action client (true: we want to spin a thread)
  if (ac_waitForUser==NULL) {
    ac_waitForUser = new actionlib::SimpleActionClient<living_at_diag::WaitForUserAction>(waitForUser_topic, true);  

    // Wait for the action server to come up
    while(!ac_waitForUser->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for Follow Corridor action server to come up");
    }
  }
  
  // Cancel all goals (JUST IN CASE SOME GOAL WAS NOT CLOSED BEFORE)
  ac_waitForUser->cancelAllGoals(); 
  ros::Duration(0.5).sleep(); // wait .5 sec
  
  // Creating the goal
  living_at_diag::WaitForUserGoal goal;

  // Send the goal
  ROS_INFO("Sending goal");
  ac_waitForUser->sendGoal(goal);

  // Wait for termination
  while (!ac_waitForUser->waitForResult(ros::Duration(1.0))) {
    if (!*run){
      ac_waitForUser->cancelGoal();
      ROS_INFO("Wait For User Stopped");
    }
  }
  
  ros::Duration(0.250).sleep();
  
  // Print result
  if (ac_waitForUser->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Wait For User successful");
}

// Action implementation
void evaluateCondition(string robot_name, string params, bool *run){
  cout << "### Executing Evaluate Condition ... " << params << endl;
  sapienzbot_reasoning::EvaluateCondition srv;
  srv.request.condition = params;
  
  if (!ros::service::call("/" + robot_name + "/evaluateCondition",srv))
    cout<< "Failed to call service Evaluate Condition" << endl;  
    
  if (*run)
      cout << "### Finished Evaluate Condition " << endl;
  else
      cout << "### Aborted Evaluate Condition  " << endl;
}

void forget(string robot_name, string params, bool *run){
  cout << "### Executing Forget ... " << params << endl;
  sapienzbot_reasoning::RemoveObject srv;
  srv.request.objName = params;
  
  if (ros::service::call("/remove_object_from_cellMap",srv))
    cout << "Published message: " << params << endl;
  else
    cout<< "Failed to call service SaySrv" << endl;  
    
  if (*run)
      cout << "### Finished Forget " << endl;
  else
      cout << "### Aborted Forget  " << endl;
}

void generatePlan(string robot_name, string params, bool *run){
  cout << "### Executing Generate Plan ... " << params << endl;
  combine_pnp::GeneratePlanSrv srv;
  int i                 = params.find("_");
  srv.request.name      = params.substr(0,i);
  srv.request.pcl       = params.substr(i+1);

  if (ros::service::call("/combine_pnp/generate_plan",srv))
    cout << "Sent generate plan command with plan name " << srv.request.name << " and pcl " << srv.request.pcl << endl;
  else
    cout << "Failed to call service generate plan" << endl;  
  
  if (*run)
    cout << "### Finished Generate Plan " << endl;
  else
    cout << "### Aborted Generate Plan  " << endl;
}

void groundAtom(string robot_name, string params, bool *run){ 
  cout << "### Executing Ground Atom ... " << params << endl;
  sapienzbot_reasoning::GroundCommand srv;
  int i                = params.find("#");
  srv.request.frame    = params.substr(0,i);
  srv.request.target   = params.substr(i+1);

  if (srv.request.target == "anyone")
  {
    srv.request.frame = "CLOSEST";
    srv.request.target = last_object_type;
  }
  else if (srv.request.frame == "NOFRAME")
    srv.request.frame = "NEAR";

  if (ros::service::call("/" + robot_name + "/ground_atom",srv))
    cout << "Sent ground command with frame " << srv.request.frame << " and target " << srv.request.target << endl;
  else
    cout << "Failed to call service atom grounding" << endl;  

  if (string_stars_with(srv.response.response, "Multiple"))
  {
    int i                = srv.response.response.rfind("_");     
    last_object_type     = srv.response.response.substr(i+1);
  } 
  
  if (*run)
    cout << "### Finished Ground Atom " << endl;
  else
    cout << "### Aborted Ground Atom  " << endl;
}

void groundGoTo(string robot_name, string params, bool *run){
  cout << "### Executing Ground GoTo ... " << params << endl;
  
  sapienzbot_reasoning::GroundCommand srv;
    
  int i                 = params.find("#");
  srv.request.frame     = params.substr(0,i);
  srv.request.target    = params.substr(i+1);
  
  if (srv.request.target == "anyone")
  {
    srv.request.frame = "CLOSEST";
    srv.request.target = last_object_type;
  }
  
  else if (srv.request.frame == "NOFRAME")
    srv.request.frame = "NEAR";
  
  if (ros::service::call("/" + robot_name + "/ground_goto",srv))
    cout << "Sent ground command with frame " << srv.request.frame << " and target " << srv.request.target << endl;
  else
    cout << "Failed to call service goto grounding" << endl;  
  
  if (string_stars_with(srv.response.response, "Multiple"))
  {
    int i                = srv.response.response.rfind("_");     
    last_object_type     = srv.response.response.substr(i+1);
  } 
    
  if (*run)
    cout << "### Finished Ground GoTo " << endl;
  else
    cout << "### Aborted Ground GoTo  " << endl;
}

void insertAction(string robot_name, string params, bool *run){
  cout << "### Executing Insert Action ... " << params << endl;
  combine_pnp::UpdateAction srv;
  
  std::stringstream test(params);
  std::string segment;
  std::vector<std::string> seglist;

  while(std::getline(test, segment, '_'))
  {
    seglist.push_back(segment);
  }
  
  srv.request.update_command            = seglist[0];
  srv.request.action_to_be_modified     = seglist[1];
  srv.request.action_to_be_added        = seglist[2];
  srv.request.planName                  = seglist[3];
  
  cout << "params: " << srv.request.update_command << " " << srv.request.action_to_be_modified << " " << srv.request.action_to_be_added << " " << srv.request.planName << endl;
  
  if (ros::service::call("/combine_pnp/insert_action_in_plan",srv))
    cout << "Sent insert action command with actions " << srv.request.action_to_be_modified << ", " << srv.request.action_to_be_added << " and plan " << srv.request.planName << endl;
  else
    cout << "Failed to call service Insert Action" << endl;  
  
  if (*run)
    cout << "### Finished Insert Action " << endl;
  else
    cout << "### Aborted Insert Action  " << endl;
}

void memQueryLoop(string robot_name, string params, bool *run){
  cout << "### Executing MemQueryLoop ... " << params << endl;
  
  ++ memQueryLoopIndex;
  
  sapienzbot_dialog::QueryLoop srv;
  srv.request.objList = params;
  srv.request.index = memQueryLoopIndex;
  
  if (ros::service::call("/" + robot_name + "/queryLoop",srv))
    cout << "Sent QueryLoop message" << endl;
  else
    cout<< "Failed to call service QueryLoop" << endl;  
    
  if (srv.response.result == "EndLoop")
    memQueryLoopIndex = -1;
  
  if (*run)
      cout << "### Finished MemQueryLoop " << endl;
  else
      cout << "### Aborted MemQueryLoop  " << endl;
}

void removeAndFeedback(string robot_name, string params, bool *run){
  cout << "### Executing removeAndFeedback ... " << params << endl;
  
  vector <string> splitted_params;
  boost::split(splitted_params, params, boost::is_any_of(","));
  
  forget(robot_name, splitted_params[memQueryLoopIndex], run);
  say(robot_name, "I removed it from my memory", run);
  
  if (*run)
      cout << "### Finished removeAndFeedback " << endl;
  else
      cout << "### Aborted removeAndFeedback  " << endl;
}

void removeAction(string robot_name, string params, bool *run){
  cout << "### Executing Remove Action ... " << params << endl;
  combine_pnp::UpdateAction srv;
  int i                                 = params.find("_");
  srv.request.action_to_be_modified     = params.substr(0,i);
  srv.request.planName                  = params.substr(i+1);
  
  if (ros::service::call("/combine_pnp/remove_action_from_plan",srv))
    cout << "Sent remove action command with action " << srv.request.action_to_be_modified << " and plan " << srv.request.planName << endl;
  else
    cout << "Failed to call service Remove Action" << endl;  
  
  if (*run)
    cout << "### Finished Remove Action " << endl;
  else
    cout << "### Aborted Remove Action  " << endl;
}

void replaceAction(string robot_name, string params, bool *run){
  cout << "### Executing Replace Action ... " << params << endl;
  combine_pnp::UpdateAction srv;
    
  std::stringstream test(params);
  std::string segment;
  std::vector<std::string> seglist;

  while(std::getline(test, segment, '_'))
  {
    seglist.push_back(segment);
  }
  
  srv.request.action_to_be_modified     = seglist[0];
  srv.request.action_to_be_added        = seglist[1];
  srv.request.planName                  = seglist[2];
  
  if (ros::service::call("/combine_pnp/replace_action_in_plan",srv))
    cout << "Sent replace action command with actions " << srv.request.action_to_be_modified << ", " << srv.request.action_to_be_added << " and plan " << srv.request.planName << endl;
  else
    cout << "Failed to call service Replace Action" << endl;  
  
  if (*run)
    cout << "### Finished Replace Action " << endl;
  else
    cout << "### Aborted Replace Action  " << endl;
}

void takePicture(string robot_name, string params, bool *run){
  cout << "### Executing Take Picture ... " << params << endl;
  
  dot_detector::TakePicture srv;
  pnp_msgs::PNPSetVariableValue srv2;
  if (!ros::service::call("/TakePicture",srv)) {
    cout<< "Failed to call service Take Picture" << endl; 
    srv2.request.variable     = "PictureName";
    srv2.request.value        = "dummyFile.png";
    cout<< "Instantiating variable " << srv2.request.variable << " to dummy value " << srv2.request.value << endl;
  }
  else {
    srv2.request.variable     = "PictureName";
    srv2.request.value        = srv.response.pictureName;
  }
  
  if (!ros::service::call("/" + robot_name + "/PNPSetVariableValue",srv2)) 
    cout<< "Failed to call service PNPSetVariableValue" << endl;  
  
  if (*run)
    cout << "### Finished Take Picture " << endl;
  else
    cout << "### Aborted Take Picture " << endl;
}

void say(string robot_name, string params, bool *run){
    cout << "### Executing Say ... " << params << endl;
    
    std::replace( params.begin(), params.end(), '_', ' ');
    
    sapienzbot_dialog::SaySrv srv;
    srv.request.message = params;
    
    if (!ros::service::call("/SaySrv",srv))
      cout<< "Failed to call service SaySrv" << endl;  
    
    if (*run)
        cout << "### Finished Say " << endl;
    else
        cout << "### Aborted Say  " << endl;
}

void waitForUser(string robot_name, string params, bool *run){
    
  exec_waitForUser(run); 
    
    if (*run)
        cout << "### Finished Wait For User " << endl;
    else
        cout << "### Aborted Wait For User  " << endl;
}