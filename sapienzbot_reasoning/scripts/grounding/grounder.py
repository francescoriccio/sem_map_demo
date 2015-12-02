#!/usr/bin/env python
PKG = 'sapienzbot_reasoning'

import roslib; roslib.load_manifest(PKG)
import rospy
import robotpose

from prolog_interface.srv import *
from semantic_map_extraction.srv import *

def ground_request(frame, target):
  prolog_handle = rospy.ServiceProxy('/prolog_interface/prolog_query', prologSrv)
  conversion_cells_from_coord_handle = rospy.ServiceProxy('/semantic_map_extraction/get_cell_by_coords', GetCellByCoords)
  conversion_coord_from_cells_handle = rospy.ServiceProxy('/semantic_map_extraction/get_coords_by_cell', GetCoordsByCell)
  
  robot_pose = [0,0,0]
  robotpose.getRobotPose(robot_pose)
  #converted_robot_pose = conversion_cells_from_coord_handle(robot_pose[0], robot_pose[1])
  
  # grounding the commands with frame CLOSEST and target: OBJECT (e.g. bed)
  if (frame == "CLOSEST"):
    targetList = str(target).replace(" ","").replace("\'","")
    print targetList
    try:
      target_atoms_list = prolog_handle('nearest_atom_in_list_from', [targetList, str(robot_pose[0]), str(robot_pose[1]), "X"])
      target_atoms_list = [target_atoms_list.ris[0].atoms[3]]

    except:
      target_atoms_list = []

  # grounding other commands
  else:
    target_list = target.split(" ")
  
    try:
      listProlog = str(target_list).replace("\"","").replace("\'","").replace(" ","")
      print listProlog
      query_results = prolog_handle('getAtom', [listProlog, "V", str("%.2f"%robot_pose[0]), str("%.2f"%robot_pose[1])]).ris
      
      target_atoms_list = []
      
      for result in query_results:
        target_atoms_list.append(result.atoms[1])
        
    except:
      target_atoms_list = []

  return list(set(target_atoms_list))