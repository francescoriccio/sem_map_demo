#!/usr/bin/env python
PKG = 'sapienzbot_reasoning'
NODE = 'goto_grounding'

import roslib; roslib.load_manifest(PKG)
import sys
import rospy
import math
import robotpose

from grounder import ground_request
from prolog_interface.srv import *
from sapienzbot_reasoning.srv import *
from semantic_map_extraction.srv import *
from std_msgs.msg import String

global atom_list_memory
atom_list_memory = []

###################################################################################
### Support functions

def get_nearest_cell_in_room(atom):
  robot_pose = [0,0,0]
  robotpose.getRobotPose(robot_pose)
  converted_robot_pose = conversion_cells_from_coord_handle(robot_pose[0], robot_pose[1])
  query_results = prolog_handle ('get_cell_near_destination_room', [str(converted_robot_pose.cell_x), str(converted_robot_pose.cell_y), atom, 'X', 'Y']).ris[0]
  coords = [query_results.atoms[3], query_results.atoms[4]]
  return [coords[0],coords[1], 0]
  
def get_pos_in_center_of_room(atom):
  query_results = prolog_handle ('roomRealCentroid', [atom, 'X', 'Y']).ris[0]
  coords = [query_results.atoms[1], query_results.atoms[2]]
  return [coords[0],coords[1], 0]

def get_pos_in_front_of_target(atom):
  query_results = prolog_handle ('front_of_obj', [atom, 'X', 'Y']).ris[0]
  coords = [query_results.atoms[1], query_results.atoms[2]]
  coords.append(360-float(prolog_handle ('objectAngle', [atom, 'X']).ris[0].atoms[1]))
  return [coords[0],coords[1], coords[2]]

def get_target_type(atom):
  ris = prolog_handle ('targetType', [atom, 'X', 'Y']).ris[0]
  return [ris.atoms[1], ris.atoms[2]]

def is_robot_already_there(target_pose, target_category, target_id):
  robot_pose            = [0,0,0]
  threshold             = 0.5
  robotpose.getRobotPose(robot_pose)
  converted_robot_pose  = conversion_cells_from_coord_handle(robot_pose[0], robot_pose[1])

  is_robot_near         = math.sqrt(math.pow(float(robot_pose[0]) - float(target_pose[0]), 2) + math.pow(float(robot_pose[1]) - float(target_pose[1]),2)) < threshold
  robot_current_room    = prolog_handle('cellCoordIsPartOf', [str(int(converted_robot_pose.cell_x)), str(int(converted_robot_pose.cell_y)), 'X']).ris[0].atoms[2]
  is_robot_in_room      = (target_id == robot_current_room)
  
  if (target_category == "object" and is_robot_near) or (target_category == "room" and is_robot_in_room):
    return True
  else:
    return False
  
def log_to_terminal(string):
  print "["+NODE+"] \033[95m"+string+"\033[0m"

###################################################################################
### Service CallBack

def handle_goto_execute_command(req):   
    global atom_list_memory
    log_to_terminal("Processing command frame: %s target: %s"%(req.frame, req.target))
    
    if req.frame == "CLOSEST":
      target_atom_list = ground_request(req.frame, atom_list_memory)
    else:
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
      event = "GoToGroundUnknown_I do not know any object of that kind"
      response = "None"
      atom_list_memory = []
    
    elif (len(target_atom_list) == 1):
      target_type, target_category = get_target_type(target_atom_list[0])
      if (target_category == "object"):
        target_pos = get_pos_in_front_of_target(target_atom_list[0])
      else:
        target_pos = get_nearest_cell_in_room(target_atom_list[0])
        
      if is_robot_already_there(target_pos, target_category, target_atom_list[0]):
        event = "GoToGroundAlreadyThere_I am already there"
        response = "None"
        atom_list_memory = []
      else:
        event = "GoToGroundSingle_%s_%s_%s_%s"%(target_type, target_pos[0], target_pos[1], target_pos[2])
        response = "Single"+"_"+target_type
        atom_list_memory = []
  
    else:
      target_type, target_category = get_target_type(target_atom_list[0])
      event = "GoToGroundMultiple_I know %s %ss in the environment. Which one do you mean?"%(len(target_atom_list),target_type)
      response = "Multiple"+"_"+target_type
      atom_list_memory = target_atom_list
      
    event_pub.publish(event)
    log_to_terminal("Sent event: " + event)
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
    conversion_cells_from_coord_handle = rospy.ServiceProxy('/semantic_map_extraction/get_cell_by_coords', GetCellByCoords)
    
    event_pub                           = rospy.Publisher("/"+robot_name+"/PNPConditionEvent", String, queue_size=1)

    sub_reset_mem                       = rospy.Subscriber("/"+PKG+"/reset_grounding_memory", String, reset_grounding_memory, queue_size=1)

    rospy.Service('/'+robot_name+'/ground_goto', GroundCommand, handle_goto_execute_command)
    
    log_to_terminal("Ready to execute commands")
    
    rospy.spin()
