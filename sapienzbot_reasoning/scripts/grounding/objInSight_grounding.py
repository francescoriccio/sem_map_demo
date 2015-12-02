#!/usr/bin/env python
PKG = 'sapienzbot_reasoning'
NODE = 'get_objects_in_sight'

import roslib; roslib.load_manifest(PKG)
import sys
import rospy
import robotpose

from itertools import groupby
from math import degrees, sqrt, atan2, fabs
from prolog_interface.srv import *
from sapienzbot_reasoning.srv import *
from semantic_map_extraction.srv import *
from std_msgs.msg import String


###################################################################################
### Support functions

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
      
      #print "objects in the room:"
      #print objectsList
      
      objectsList = [item for item in objectsList if item[1] < SIGHT_FAR_THRESHOLD and item[1] > SIGHT_CLOSE_THRESHOLD and (abs(item[2]) < ANGULAR_THRESHOLD or abs(item[2]) > 360 - ANGULAR_THRESHOLD)]
       
      objectsList = removeElementsWithSameName(objectsList)
      
      #print "\nobjects in front of the robot:"
      #print objectsList
      
      for i in objectsList:
        result = prolog_handle('objectType', [i[0],"X"])
        objectsName.append(result.ris[0].atoms[1].replace("_"," "))
        
      return objectsName
      
    except:
      return []

def log_to_terminal(string):
    print "["+NODE+"] \033[94m"+string+"\033[0m"
  
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

def text_from_list(objList):
    objList = sorted(objList)
    
    if len(objList) == 0:
      text = "I do not know any object in front of me"
    
    else:
      countList = [len(list(group)) for key, group in groupby(objList)]
      fileterdObjList = sorted(list(set(objList)))
      
      if len(countList) == 1:
        if countList[0] > 1:
          text = "I can see %s %ss in front of me"%(countList[0], objList[0])
        else:
          text = "I can see %s %s in front of me"%(countList[0], objList[0])
      else:
        text = "I can see"
        for i in xrange(len(fileterdObjList) - 1):
          if countList[i] > 1:
            text = text + " %s %ss"%(countList[i], fileterdObjList[i])
          else:
            text = text + " %s %s"%(countList[i], fileterdObjList[i])
        if countList[-1] > 1:
          text = text + " and %s %ss in front of me"%(countList[-1], fileterdObjList[-1])
        else:
          text = text + " and %s %s in front of me"%(countList[-1], fileterdObjList[-1])
        
    return text
 
###################################################################################
### Service CallBack

def objectsInSight_cb(req):
    objectsInSight = getObjectsInSight()
    event = "ObjInSightGround_" + text_from_list(objectsInSight)
      
    event_pub.publish(event)
    log_to_terminal("Sent event: " + event)
    return GroundCommandResponse("%s ObjInSight"%len(objectsInSight))
  
###################################################################################
### Main

if __name__ == "__main__":
    rospy.init_node(NODE)

    robot_name                          = rospy.get_param("/robotname", "robot_0")

    prolog_handle                       = rospy.ServiceProxy('/prolog_interface/prolog_query', prologSrv)
    conversion_cells_from_coord_handle  = rospy.ServiceProxy('/semantic_map_extraction/get_cell_by_coords', GetCellByCoords)
    conversion_coord_from_cells_handle  = rospy.ServiceProxy('/semantic_map_extraction/get_coords_by_cell', GetCoordsByCell)
    
    event_pub                           = rospy.Publisher("/"+robot_name+"/PNPConditionEvent", String, queue_size=1)

    rospy.Service('/'+robot_name+'/ground_objInSight', GroundCommand, objectsInSight_cb)
    
    log_to_terminal("Ready to execute commands")
    
    rospy.spin()