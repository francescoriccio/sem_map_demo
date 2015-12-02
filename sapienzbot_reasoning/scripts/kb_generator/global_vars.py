#!/usr/bin/env python
PACKAGE='sapienzbot_reasoning'
import roslib
roslib.load_manifest(PACKAGE)

import rospy

# ROS stuff initialized in knowledge generator
global remove_object_by_name, event_pub, prolog_service_handle, prolog_retractAll_handle
global prolog_reloadAll_handle, prolog_load_handle, prolog_assert_handle, prolog_retract_handle
global create_obj_model, images_log_folder

old_matrix                      = []
DRAW_GRAPH                      = False
LOG_KB                          = True
initialize                      = True
prolog_files                    = {"cells":"", "centroids":"", "graph":"", "objects":"", "old_objects":"", "plans":"", "rooms":""} 
last_object_memorized           = ""


if(DRAW_GRAPH):
  try:
    import matplotlib.pyplot as plt
  except:
    raise
  import networkx as nx

  G = nx.Graph()