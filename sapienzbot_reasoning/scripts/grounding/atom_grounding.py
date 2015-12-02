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

global atom_list_memory
atom_list_memory = []

###################################################################################
### Support functions

def get_target_type(atom):
    return prolog_handle ('objectType', [atom, 'X']).ris[0].atoms[1]

def log_to_terminal(string):
  print "["+NODE+"] \033[96m"+string+"\033[0m"

###################################################################################
### Service CallBack

def handle_forget_execute_command(req):    
    global atom_list_memory
    log_to_terminal("Processing command frame: %s target: %s"%(req.frame, req.target))
    
    target_atom_list = ground_request(req.frame, req.target)
    
    log_to_terminal("retrieved target atom list: %s"%target_atom_list)
    
    # implementing the contextual memory in the robot.
    # Example: if I say "go to the socket near the closet" but the robot knows two sockets with that property,
    # the next specification should remember the previous commands
    # so if I now tell the robot to go to the socket near the window it should understand
    # that I am referring to one of the sockets that were near the closet
    if atom_list_memory != []:
      for i in xrange(len(target_atom_list)):
        if not target_atom_list[i] in atom_list_memory:
          target_atom_list.pop(i)
       
    log_to_terminal("filtered target atom list: %s"%target_atom_list)
    
    if (len(target_atom_list) == 0):
      event = "GroundAtom_None_None_None_I do not know any object of that kind"
      response = "None"
      atom_list_memory = []
      
    elif (len(target_atom_list) == 1):
      target_type = get_target_type(target_atom_list[0])    
      event = "GroundAtom_Single_%s_%s_None"%(target_atom_list[0].replace("_","#"), target_type)
      response = "Single"+"_"+target_type
      atom_list_memory = []
      
    else:
      target_type = get_target_type(target_atom_list[0])    
      event = "GroundAtom_Multiple_%s_%s_I know %s %ss in the environment. Which one do you mean?"%(target_atom_list[0].replace("_","#"), target_type, len(target_atom_list), target_type)
      response = "Multiple"+"_"+target_type
      atom_list_memory = target_atom_list
 
    event_pub.publish(event)
    log_to_terminal("Sent event: " + event)
    
    log_to_terminal("atom list: %s"%atom_list_memory)
    
    return GroundCommandResponse(response)
    
def reset_grounding_memory(msg):
    global atom_list_memory
    atom_list_memory = []
    return "Done"

###################################################################################
### Main

if __name__ == "__main__":
    rospy.init_node(NODE)
    
    robot_name                          = rospy.get_param("/robotname", "robot_0")

    prolog_handle                       = rospy.ServiceProxy('/prolog_interface/prolog_query', prologSrv)
    conversion_coord_from_cells_handle  = rospy.ServiceProxy('/semantic_map_extraction/get_coords_by_cell', GetCoordsByCell)
    
    event_pub                           = rospy.Publisher("/"+robot_name+"/PNPConditionEvent", String, queue_size=1)
    
    sub_reset_mem                       = rospy.Subscriber("/"+PKG+"/reset_grounding_memory", String, reset_grounding_memory, queue_size=1)
    
    rospy.Service('/'+robot_name+'/ground_atom', GroundCommand, handle_forget_execute_command)
    
    log_to_terminal("Ready to execute commands")
    
    rospy.spin()
