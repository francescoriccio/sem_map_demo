#!/usr/bin/env python
PKG = 'sapienzbot_reasoning'
NODE = 'knowledge_generator'
import roslib; roslib.load_manifest(PKG)
import rospy
import global_vars
import os

from init_kb import *
from objects_processing.srv import *
from os.path import expanduser
from update_kb import *
from sapienzbot_reasoning.srv import *
from sapienzbot_reasoning.msg import *
from semantic_map_extraction.srv import *
from support_functions import *
from prolog_interface.srv import *
from prolog_interface.msg import *
from std_msgs.msg import *
  
###################################################################################
### Services CallBacks

def generate_kb(req):
  log_to_terminal("Received a %s x %s matrix"%(req.height,req.width)) 
  
  if (global_vars.initialize):
    init_kb(req.width, req.matrix, req.cell_centers_x, req.cell_centers_y)
    global_vars.initialize = False
  else:
    update_kb(req.width, req.matrix)
    global_vars.event_pub.publish("ObjMoved") #Needed for object updating

  
  return "Done"

def remove_object_from_cellMap(req):
    # Due to the PNP parser some underscore had to be replaced with # during string manipulation
    object_atom = req.objName.replace("#","_")

    obj_name = get_obj_sem_map_name(object_atom)
    
    remember(object_atom)
    global_vars.remove_object_by_name.publish(String(obj_name)) #remove obj in cell map
    
    log_to_terminal("Removed Object: %s"%obj_name)
    return "Done"
  
def update_object_from_cellMap(req):  
    # Due to the PNP parser some underscore had to be replaced with # during string manipulation
    new_object_atom = global_vars.last_object_memorized
    old_object_atom = req.oldObjName.replace("#","_")
    obj_name = get_obj_sem_map_name(old_object_atom)
    
    remember(old_object_atom, new_object_atom)
    global_vars.remove_object_by_name.publish(String(obj_name)) #remove obj in cell map
    
    log_to_terminal("Updated Object: %s"%obj_name)
    return "Done"
  
def reset_prolog(req):
  # after a reset the sem_map_extractor sends the newly produced matrix to this node
  log_to_terminal("Requested to reset prolog knowledge")
      
  try:
    reset_semantic_map = rospy.ServiceProxy('/semantic_map_extraction/delete_map', DeleteMap)
    resp1 = reset_semantic_map("Reset")
    
    if(resp1.ack == True):
      log_to_terminal("Semantic map extraction node resetted")
      open("old_objects.pl", 'w').close() #deletes the history of the objects learnt
      global_vars.prolog_files["old_objects"] = open("old_objects.pl", 'a')
    else:
      err_to_terminal("Could not reset map extraction node")
      
  except:
    err_to_terminal("Could not contact map extraction node")
  
  return "Done"

#######################################################################################
### Main  

if __name__ == "__main__":
  rospy.init_node(NODE)
  directory   = rospy.get_param(NODE+'/prolog_path', "/pl/")
  robot_name  = rospy.get_param('/robotname', "robot_0")
  
  global_vars.images_log_folder = rospy.get_param('/log_images_dir_path', expanduser("~")+'/Desktop')
  
  global_vars.remove_object_by_name       = rospy.Publisher('remove_object_by_name', String, queue_size=1)
  global_vars.event_pub                   = rospy.Publisher("/"+robot_name+"/PNPConditionEvent", String, queue_size=1)
  
  global_vars.prolog_service_handle       = rospy.ServiceProxy('/prolog_interface/prolog_query', prologSrv)
  global_vars.prolog_assert_handle        = rospy.ServiceProxy('/prolog_interface/prolog_assert', prologAssertSrv)
  global_vars.prolog_retract_handle       = rospy.ServiceProxy('/prolog_interface/prolog_retract', prologRetractSrv)
  global_vars.prolog_retractAll_handle    = rospy.ServiceProxy('/prolog_interface/prolog_retractall', prologRetractSrv)
  global_vars.prolog_reloadAll_handle     = rospy.ServiceProxy('/prolog_interface/prolog_reload_all', prologReloadSrv)
  global_vars.prolog_load_handle          = rospy.ServiceProxy('/prolog_interface/prolog_load', prologReloadSrv)
  global_vars.create_obj_model            = rospy.ServiceProxy('/objects_processing/extract_object', ExtractObject)
  
  rospy.Service('/reset_prolog', ResetProlog, reset_prolog)
  rospy.Service('/remove_object_from_cellMap', RemoveObject, remove_object_from_cellMap)
  rospy.Service('/update_object_from_cellMap', UpdateObject, update_object_from_cellMap)
  rospy.Service('/create_topological_graph', GenerateKB, generate_kb)
  
  os.chdir(directory)    
  log_to_terminal("Prolog path: " + directory)
  log_to_terminal("Robot name: " + robot_name)
  log_to_terminal("Images log directory: " + global_vars.images_log_folder)
  log_to_terminal("Ready to create the topological graph.")
  
  rospy.spin()