#!/usr/bin/env python
PACKAGE='sapienzbot_reasoning'
import roslib
roslib.load_manifest(PACKAGE)

import rospy
import global_vars
from support_functions import *

### Processing Function
def update_kb(width, matrix):
  log_to_terminal("Updating KB...")
  
  predicates = get_file_content("objects")
  models_list_to_be_created = []
  assertions_list = [""]
  
  for i in xrange(len(matrix)):
    # Since we already have a matricial representation of the map we just need to check the differences between the old and the new one
    if matrix[i] != global_vars.old_matrix[i]:
      log_to_terminal("Matrix element:\n\'%s\'\ndiffers from old one:\n\'%s\'\nProcessing it..."%(matrix[i],global_vars.old_matrix[i]))      
      column    = int(i % width)
      row       = int(i / width)
      tags      = get_tags(matrix[i])
      old_tags  = get_tags(global_vars.old_matrix[i])
      
      
      objects = tags[1]
      pars    = tags[3] 
      old_objects = old_tags[1]
      old_pars    = old_tags[3]
      
      filter_objects_pars_lists(objects, pars, old_objects, old_pars)
      
      for obj in old_objects:
        if obj != '0':
          forget(obj)
      
      for index_obj in xrange(len(objects)):
        if objects[index_obj] != '0':
          learn(tags[0], objects[index_obj], pars[index_obj], column, row, assertions_list)
          inconsistency = process_inconsistencies(tags[0], objects[index_obj])
          if("objImagedata" in pars[index_obj] and ("EMPTY" in inconsistency or "DIFFERENT" in inconsistency)):
            models_list_to_be_created.append(re.findall('raw-\d+', pars[index_obj])[0].split("-")[1])

  for i in list(set(models_list_to_be_created)):
    create_obj_model_with_id(i)
  
  flush_prolog_files()
  
  global_vars.old_matrix = matrix
  
  log_to_terminal("Done updating KB")