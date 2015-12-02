#!/usr/bin/env python
PKG = 'sapienzbot_reasoning'
NODE = 'forget_grounding'

import roslib; roslib.load_manifest(PKG)
import sys
import rospy

from grounder import ground_request
from prolog_interface.srv import *
from sapienzbot_reasoning.srv import *
from semantic_map_extraction.srv import *
from std_msgs.msg import String

###################################################################################
### Support functions

def get_target_type(atom):
    return prolog_handle ('objectType', [atom, 'X']).ris[0].atoms[1]

def log_to_terminal(string):
  print "["+NODE+"] \033[96m"+string+"\033[0m"

###################################################################################
### Service CallBack

def handle_forget_execute_command(req):    
    log_to_terminal("Processing command frame: %s target: %s"%(req.frame, req.target))
    
    target_atom_list = ground_request(req.frame, req.target)
    
    if (len(target_atom_list) == 0):
      event = "ForgetGroundUnknown_I do not know any object of that kind"
      response = "None"
      
    elif (len(target_atom_list) == 1):
      target_type = get_target_type(target_atom_list[0])    
      event = "ForgetGroundSingle_%s_%s"%(target_atom_list[0].replace("_","#"), target_type)
      response = "Single"+"_"+target_type
      
    else:
      target_type = get_target_type(target_atom_list[0])    
      event = "ForgetGroundMultiple_I know %s %ss in the environment. Which one do you mean?"%(len(target_atom_list),target_type)
      response = "Multiple"+"_"+target_type
 
    event_pub.publish(event)
    log_to_terminal("Sent event: " + event)
    return GroundCommandResponse(response)

###################################################################################
### Main

if __name__ == "__main__":
    rospy.init_node(NODE)
    
    robot_name                          = rospy.get_param("/robotname", "robot_0")

    prolog_handle                       = rospy.ServiceProxy('/prolog_interface/prolog_query', prologSrv)
    conversion_coord_from_cells_handle  = rospy.ServiceProxy('/semantic_map_extraction/get_coords_by_cell', GetCoordsByCell)
    
    event_pub                           = rospy.Publisher("/"+robot_name+"/PNPConditionEvent", String)

    rospy.Service('/'+robot_name+'/ground_forget', GroundCommand, handle_forget_execute_command)
    
    log_to_terminal("Ready to execute commands")
    
    rospy.spin()
