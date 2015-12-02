#!/usr/bin/env python
PKG = 'sapienzbot_reasoning'
NODE = 'condition_evaluator'

import roslib; roslib.load_manifest(PKG)
import sys
import rospy
import math
import robotpose

from math import degrees, sqrt, atan2, fabs
from prolog_interface.srv import *
from sapienzbot_reasoning.srv import *
from semantic_map_extraction.srv import *
from std_msgs.msg import String

SIGHT_FAR_THRESHOLD = 5.0
SIGHT_CLOSE_THRESHOLD = 0.2
ANGULAR_THRESHOLD = 60

def convert_from_map_euclidian_to_robot_angular(x,y,rp):
  x_rel = x - rp[0]
  y_rel = y - rp[1]
  r = sqrt(x_rel * x_rel + y_rel * y_rel)
  phi = normalize(degrees(atan2(y_rel, x_rel)))
  phi_rel = phi - normalize(degrees(rp[2]))
  return [r,phi_rel]

def getObjectsInSight():
  rp=[0,0,0]
  robotpose.getRobotPose(rp)
  
  try:
    log_to_terminal("Getting objects in sight for robot in position [%s, %s, %s]"%(float(rp[0]),float(rp[1]),normalize(degrees(rp[2]))))
    answer = conversion_cells_from_coord_handle(float(rp[0]),float(rp[1]))
    result = prolog_handle('allObjectsInRoom', ["%s"%answer.cell_x,"%s"%answer.cell_y, "L"])
    
    itemList = result.ris[0].atoms[2]
    itemList = itemList.replace("\'","").replace("[[","").replace("]]","").split("],[")
    
    objectsList = []
    objectsName = []
    
    for i in itemList:
      splitted_item = i.split(",")
      euclid_coords = conversion_coord_from_cells_handle(int(splitted_item[1]),int(splitted_item[2]))
      polar_coords = convert_from_map_euclidian_to_robot_angular(euclid_coords.real_x, euclid_coords.real_y,rp)
      objectsList.append([splitted_item[0],polar_coords[0],polar_coords[1]])
    
    objectsList = [item for item in objectsList if item[1] < SIGHT_FAR_THRESHOLD and item[1] > SIGHT_CLOSE_THRESHOLD and (abs(item[2]) < ANGULAR_THRESHOLD or abs(item[2]) > 360 - ANGULAR_THRESHOLD)] 
    objectsList = removeElementsWithSameName(objectsList)
    
    for i in objectsList:
      result = prolog_handle('objectType', [i[0],"X"])
      objectsName.append(result.ris[0].atoms[1].replace("_"," "))
      
    return objectsName
    
  except:
    return []

def normalize(a):
    while (a >= 360) :
      a = a-360
    while (a <= 0):
      a = a+360
    return a

def removeElementsWithSameName(listToBeFiltered):
    outlist=[]
    added_keys = set()
    
    for row in listToBeFiltered:
      if row[0] not in added_keys:
        outlist.append(row)
        added_keys.add(row[0])
      
    return outlist  

###################################################################################
### Support functions

def handle_evaluate_condition_command(req):
  condition     = req.condition
  result        = "False"
  
  if condition.startswith("in{"):
    argument = condition.replace("in{","")[:-1]
    robot_pose            = [0,0,0]
    robotpose.getRobotPose(robot_pose)
    converted_robot_pose  = conversion_cells_from_coord_handle(robot_pose[0], robot_pose[1])
    robot_current_room    = prolog_handle('cellCoordIsPartOf', [str(int(converted_robot_pose.cell_x)), str(int(converted_robot_pose.cell_y)), 'X']).ris[0].atoms[2]
    log_to_terminal("robot_current_room:%s ?== argument:%s"%(robot_current_room,argument))
    if robot_current_room == argument:
      result = "True"
  
  elif condition.startswith("isPerceived{"):
    argument            = condition.replace("isPerceived{","")[:-1]
    objectsInSight      = getObjectsInSight()
    
    log_to_terminal("argument:%s ?in list:%s"%(argument, objectsInSight))

    if argument in objectsInSight:
      result = "True"

  else:
    rospy.logwarn("Could not recognize the condition %s. Considering it as False"%condition)
  
  event_pub.publish("ConditionVerified_%s"%result)
  return result

def log_to_terminal(string):
  print "["+NODE+"] \033[95m"+string+"\033[0m"

###################################################################################
### Main

if __name__ == "__main__":
    rospy.init_node(NODE)
    
    robot_name                          = rospy.get_param("/robotname", "robot_0")

    prolog_handle                       = rospy.ServiceProxy('/prolog_interface/prolog_query', prologSrv)
    conversion_cells_from_coord_handle  = rospy.ServiceProxy('/semantic_map_extraction/get_cell_by_coords', GetCellByCoords)
    conversion_coord_from_cells_handle  = rospy.ServiceProxy('/semantic_map_extraction/get_coords_by_cell', GetCoordsByCell)

    event_pub                           = rospy.Publisher("/"+robot_name+"/PNPConditionEvent", String, queue_size=1)

    rospy.Service('/'+robot_name+'/evaluateCondition', EvaluateCondition, handle_evaluate_condition_command)
    
    log_to_terminal("Ready to execute commands")
    
    rospy.spin()
