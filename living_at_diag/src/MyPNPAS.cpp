#include <ros/ros.h>
#include <std_msgs/String.h>
#include <pnp_ros/PNPActionServer.h>
#include <pnp_msgs/PNPLastEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/algorithm/string.hpp>
#include <boost/thread/thread.hpp>
#include <sapienzbot_reasoning/RemoveObject.h>
#include <sapienzbot_reasoning/UpdateObject.h>
#include <sapienzbot_reasoning/GroundCommand.h>
#include <combine_pnp/TriggerSrv.h>
#include <combine_pnp/ForgetSrv.h>
#include <combine_pnp/InitSrv.h>
#include <combine_pnp/AcquireSrv.h>
#include <prolog_interface/prologSrv.h>

#include "boost/filesystem.hpp"
#include <iostream>

#include "MyMovementActions.h"
#include "MyActions.h"

#define MAX_RELATIONS 6

using namespace std; 

class MyPNPActionServer : public PNPActionServer
{
private:
  ros::NodeHandle handle;
  
  // ROS parameters
  string robot_name, plan_folder;
  
  // publishers
  ros::Publisher asr_pub;
  ros::Publisher dotDetector_trigger_pub;
  
  // internal variables
  std_msgs::String msg;
  string last_object_type;
  bool service_called;
  
public:

  MyPNPActionServer() : PNPActionServer() { 
    // ROS parameters
    handle.param<string>("/robotname", robot_name, "robot_0");
    handle.param<string>("/plan_folder", plan_folder, "/Desktop");
    register_MRaction("evaluateCondition",&evaluateCondition);
    register_MRaction("followCorridor",&followCorridor);
    register_MRaction("followPerson",&followPerson);
    register_MRaction("forget",&forget);
    register_MRaction("generatePlan",&generatePlan);
    register_MRaction("getCloser",&getCloser);
    register_MRaction("gotopose",&gotopose);
    register_MRaction("goAndLookAt",&goAndLookAt);
    register_MRaction("groundAtom",&groundAtom);
    register_MRaction("groundGoTo",&groundGoTo);
    register_MRaction("home",&home);
    register_MRaction("insertAction",&insertAction);
    register_MRaction("memQueryLoop",&memQueryLoop);
    register_MRaction("removeAndFeedback",&removeAndFeedback);
    register_MRaction("removeAction",&removeAction);
    register_MRaction("replaceAction",&replaceAction);
    register_MRaction("takePicture",&takePicture);
    register_MRaction("turn",&turn);
    register_MRaction("sayPhrase",&say);
    register_MRaction("waitForUser",&waitForUser);
    
    
    // publishers
    asr_pub                     = handle.advertise<std_msgs::String>("/" + robot_name + "/ASR", 1000);
    dotDetector_trigger_pub     = handle.advertise<std_msgs::String>("/" + robot_name + "/dotDetector/trigger", 1000);
    
    // internal variables
    last_object_type = "";
    service_called = 0;
  }
  
  
  int evalCondition(string cond) {
    
    // checking if the transition is a variable test 
    // expecting KEYWORD<string,string1> or KEYWORD<@X,string> or KEYWORD<string,@X> or KEYWORD<@X,@Y> as test condition
    if (cond.find('<') != std::string::npos && cond.find('>') != std::string::npos && cond.find(',') != std::string::npos)
      return verifies_variable_test(cond);
      
    // reset internal variables
    else if (cond == "ResetInternalValues"){
      service_called = 0;
      
      // turns off the dot detector
      msg.data = "DEACTIVATED";      
      dotDetector_trigger_pub.publish(msg);
      return 1;
    }
    // reset condition buffer
    else if (cond == "ResetBuffer"){
      internal_clear_buffer();
      return 1;
    }
    // Enables the dot_detector in the three possible modalities (Recognize, Memorize and Update)
    else if (cond == "Activatedot_detectorRec" || cond == "Activatedot_detectorMem" || cond == "Activatedot_detectorUpd"){
      internal_clear_buffer();
      msg.data = "ACTIVE("+ cond.substr(cond.size()-3,3) +")";      
      dotDetector_trigger_pub.publish(msg);
      return 1;
    }
    // Handles the memorize action by sending the object name to the semantic_map_extraction module that waits a dot detected + a label
    else if (string_stars_with(cond, "MemRes")){
      msg.data = "CATEGORIZE(\"" + get_variable_value("Obj") + "\")";      
      asr_pub.publish(msg);
    }
    // Handling the Tell Object In Sight action
    else if (string_stars_with(cond, "ObjInSightGround") and !service_called){
      service_called = 1;
      sapienzbot_reasoning::GroundCommand srv;
      srv.request.target        = "ObjInSightGround";
      
      if (ros::service::call("/" + robot_name + "/ground_objInSight",srv))
        cout << "Sent ground command with target " << srv.request.target << endl;
      else
        cout << "Failed to call service Object In Sight grounding" << endl;  
    }
    // Handling the Update Object Position
    else if (string_stars_with(cond, "ObjUpdated")){
      // calling the sapienzbot_reasoning service that will replace the old object information with the old info of the object while remembering both
      sapienzbot_reasoning::UpdateObject srv;
      srv.request.oldObjName = get_variable_value("ObjAtom");
      if (ros::service::call("/update_object_from_cellMap", srv))
        cout << "Sent update command for object " << srv.request.oldObjName << endl;
      else
        cout << "Failed to call service Update Object Position" << endl;  
      return 1;
    }
    else if (string_stars_with(cond, "ObjMoved")){
      // sending the label of the object to the semantic_map_extraction module for adding a new copy of the object
      msg.data = "CATEGORIZE(\"" + get_variable_value("ObjType") + "\")";      
      asr_pub.publish(msg);
    }
    // handling the plan learning
    else if (string_stars_with(cond, "PCLProcessed")){
      combine_pnp::TriggerSrv cmd;
      if (!ros::service::call("/combine_pnp/analyze_pcl",cmd))
        cout << "Failed to call service /combine_pnp/analyze_pcl" << endl; 
      else return 1;
    }
    else if (string_stars_with(cond, "PCLUndefChildAdded")){
      combine_pnp::TriggerSrv cmd;
      if (!ros::service::call("/combine_pnp/add_undefined_child",cmd))
        cout << "Failed to call service /combine_pnp/add_undefined_child" << endl; 
      else return 1;
    }
    else if (string_stars_with(cond, "InitAcquisition")){
      combine_pnp::InitSrv cmd;
      cmd.request.plan_name     = get_variable_value("PlanName");
      cmd.request.param_list    = get_variable_value("ParamList", "[]");
      if (!ros::service::call("/combine_pnp/init_acquisition",cmd))
        cout << "Failed to call service /combine_pnp/init_acquisition" << endl; 
      else return 1;
    }
    else if (string_stars_with(cond, "PlanUnknown")){
      prolog_interface::prologSrv cmd;
      cmd.request.predicate     = "combinedPlan";
      vector<string> args;
      std::string planName = get_variable_value("PlanName");
      planName[0] = tolower(planName[0]);
      args.push_back(planName);
      cmd.request.arg           = args;
      
      if (!ros::service::call("/prolog_interface/prolog_query",cmd))
        cout << "Failed to call service /prolog_interface/prolog_query" << endl; 
      
      else if (cmd.response.ris.size() == 0)
        return 1;
    }
    
    else if (string_stars_with(cond, "PlanKnown")){
      prolog_interface::prologSrv cmd;
      cmd.request.predicate     = "combinedPlan";
      vector<string> args;
      std::string planName = get_variable_value("PlanName");
      planName[0] = tolower(planName[0]);
      args.push_back(planName);
      cmd.request.arg           = args;
      
      if (!ros::service::call("/prolog_interface/prolog_query",cmd))
        cout << "Failed to call service /prolog_interface/prolog_query" << endl; 
      
      else if (cmd.response.ris.size() != 0)
        return 1;
    }
    // handling the plan removing
    else if (string_stars_with(cond, "ForgotPlan")){
      combine_pnp::ForgetSrv cmd;
      cmd.request.plan = get_variable_value("PlanName");
      if (!ros::service::call("/combine_pnp/forget",cmd))
        cout << "Failed to call service comb_pnp" << endl; 
      return 1;
    }
    else if (cond == "PCLSent"){
      combine_pnp::AcquireSrv cmd;
      cmd.request.pcl = get_variable_value("PCL");
      if (!ros::service::call("/combine_pnp/acquire_plan",cmd))
        cout << "Failed to call service comb_pnp" << endl; 
      return 1;
    }
    return -1;
  }
  
  bool find_file(const boost::filesystem::path& dir_path, const string& file_name, boost::filesystem::path& path_found ) {
    if ( !exists( dir_path ) )
      return false;
    
    boost::filesystem::directory_iterator end_itr;
    for (boost::filesystem::directory_iterator itr( dir_path ); itr != end_itr; ++itr ) {
      if ( itr->path().filename() == file_name ) {
        path_found = itr->path();
        return true;
      }
    }
    return false;
  }
  
  bool string_stars_with(string a, string b){
    return (a.substr(0,b.length()) == b);
  }
  
  bool verifies_variable_test(std::string cond){
    // expecting KEYWORD<string,string1> or KEYWORD<@X,string> or KEYWORD<string,@X> or KEYWORD<@X,@Y> as test condition
    vector<std::string> splitted;
    string keyword,var1,var2;
    
    boost::split(splitted, cond, boost::is_any_of("<"));
    keyword = splitted[0];
    boost::split(splitted, splitted[1], boost::is_any_of(","));
    
    if (splitted[0][0] == '@')
      var1 = splitted[0].substr(1,splitted[0].size()-1);
    else
      var1 = splitted[0];
    
    if (splitted[1][0] == '@')
      var2 = splitted[1].substr(1,splitted[1].size()-2);
    else
      var2 = splitted[1].substr(0,splitted[1].size()-1);
    
    if (keyword == "eq") {                        // checks if the arguments values are equal to the input value
      if (splitted[0][0] == '@')
        var1 = get_variable_value(var1);
      if (splitted[1][0] == '@')
        var2 = get_variable_value(var2);
      
      return var1 == var2;
    }
    
    else if (keyword == "neq") {                  // checks if the arguments values are different from the input value
      if (splitted[0][0] == '@')
        var1 = get_variable_value(var1);
      if (splitted[1][0] == '@')
        var2 = get_variable_value(var2);
      
      return var1 != var2;
    }
    
    else if (keyword == "inst") {                 // instantiate the variable to the input value
      if (splitted[0][0] != '@') {                // the first argument of an instatiation must be a variable
        ROS_ERROR("pnp_ros instatiantion needs a variable as first argument! Correct form: [inst<@Variable, @Variable2] or [inst<@Variable, string]");
        return 0;
      }
        
      if (splitted[1][0] == '@')
        var2 = get_variable_value(var2);
      
      update_variable_with_value(var1, var2);
      return 1; 
    }
    
    else
      return 0;
  }
  
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mypnpas");

  MyPNPActionServer mypnpas;
  mypnpas.start();
  ros::spin();

  return 0;
}