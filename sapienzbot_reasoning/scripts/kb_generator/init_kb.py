#!/usr/bin/env python
PACKAGE='sapienzbot_reasoning'
import roslib
roslib.load_manifest(PACKAGE)

import rospy
import global_vars
from support_functions import *

### Initialization Function
def init_kb(width, matrix, cellVectorX, cellVectorY):

  log_to_terminal("Initializing KB...")
  
  if global_vars.LOG_KB:
    log_kb()
  
  reset_prolog_files(cellVectorX, cellVectorY)
  
  doors = [[]]
  dyn_nodes = []
  room_list = []
  
  print "resetted prolog"
  
  for i in xrange(len(matrix)):
    if (matrix[i] != "0"):
      room_tag, obj_tag, door_adj, obj_par      = matrix[i].split(";")
      obj_list                                  = obj_tag.split("#")
      par_list                                  = obj_par.split("#")
      door_map                                  = {}
      column                                    = int(i % width)
      row                                       = int(i / width)
      
      if(room_tag != "0"):
        if not (room_tag in room_list):
          room_list.append(room_tag)
        
        logic_form = "cellCoordIsPartOf(%s,%s,%s)"%(column,row,room_tag)
        global_vars.prolog_files["cells"].write(logic_form + ".\n")

      if (obj_tag != "0"):
        for index in xrange(len(obj_list)):
          learn(room_tag, obj_list[index], par_list[index], column, row)
          
          #if ("room_center" in obj_list[index]):
            #logic_form = "dyn_node(%s_center,%s,%s)"%(room_tag,column,row)
            #global_vars.prolog_files["graph"].write(logic_form + ".\n")
            #dyn_nodes.append(room_tag)
          #else:
    
      if (door_adj != "false#false#false#false"):
        doors.append(["%s#%s%s"%(room_tag,column,row),column,row])
  
  print "processed all the cells"
  
  global_vars.prolog_files["cells"].flush()
  global_vars.prolog_load_handle("cells.pl")
  
  calculate_and_assert_rooms_center_and_type(room_list)
  global_vars.prolog_load_handle("rooms.pl")

  write_doors_to_file(doors)
  create_topological_graph(room_list, doors)
  flush_prolog_files()

  global_vars.prolog_load_handle("graph.pl")
  
  global_vars.old_matrix = matrix  
  log_to_terminal("Done Initializing KB")
  
  if global_vars.DRAW_GRAPH:
    drawGraph()