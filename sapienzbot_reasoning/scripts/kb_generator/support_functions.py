#!/usr/bin/env python
PACKAGE='sapienzbot_reasoning'
import roslib
roslib.load_manifest(PACKAGE)

import rospy
import re
import string
import datetime
import global_vars
import math
import os 
import shutil

from std_msgs.msg import *
from objects_processing.srv import *

if(global_vars.DRAW_GRAPH):
  try:
    import matplotlib.pyplot as plt
  except:
    raise
  import networkx as nx

###################################################################################
### Printing Functions

def err_to_terminal(string):
  print "[knowledge_generator] \033[91m"+string+"\033[0m"
  
def log_to_terminal(string):
  print "[knowledge_generator] \033[92m"+string+"\033[0m"

###################################################################################
### Getters

def get_obj_atom(room, tag):
  #assuming to get tag = possible_obj_type_strings~2014-02-09_11:31:44
  return "%s_%s%s"%(room, tag.split("~")[0],tag.split("~")[1].replace("-","").replace("_","").replace(":",""))
  
def get_obj_time(tag):
  #assuming to get tag = possible_obj_type_strings~2014-02-09_11:31:44
  return tag.split("~")[1].replace("_","-").replace(":","-")

def get_obj_type(obj):
  #assuming to get tag = possible_obj_type_strings~2014-02-09_11:31:44
  if ("~" in obj):
    obj_type = obj.split("~")[0]
  else:
    obj_type = obj.replace(max(re.findall('\d+',obj), key=len), "").replace(obj.split("_")[0]+"_","")
  
  return get_synonym(obj_type)

def get_obj_sem_map_name(object_atom):
  object_atom = object_atom.replace(object_atom.split("_")[0]+"_","")
  numlist = max(re.findall('\d+',object_atom), key=len)
  objType = object_atom.split(numlist[0])[0]
  numlist = list(numlist)
  numlist.insert(12,':')
  numlist.insert(10,':')
  numlist.insert(8,'_')
  numlist.insert(6,'-')
  numlist.insert(4,'-')
  numlist.insert(0,'~')
  identity = string.maketrans("", "")
  return objType+str(numlist).translate(identity, ",[] \'")

def get_synonym(obj):
  synonym = obj
  
  try:
    synonym = global_vars.prolog_service_handle ('synonym', [obj, "X"]).ris[0].atoms[1]
    log_to_terminal("Found the synonym %s"%synonym)
  
  except:
    log_to_terminal("Could not find a synonym for %s"%synonym)
  
  return synonym
  
def get_tags(element):
  try: 
    if element != '0':
      room_tag, obj_tag, door_adj, obj_par      = element.split(";")
      obj_tag                                   = obj_tag.split("#")
      obj_par                                   = obj_par.split("#")
    else:
      room_tag = '0'
      obj_tag  = []
      door_adj = ''
      obj_par  = []

  except:
    err_to_terminal("element %s can not be parsed in tags")
  
  return [room_tag, obj_tag, door_adj, obj_par]

###################################################################################
### File Functions  

def reset_prolog_files(cellVectorX, cellVectorY):
  for key in global_vars.prolog_files.keys():
    if(os.path.exists(key+".pl") and key != "old_objects" and key != "plans"):
      os.remove(key+".pl")
    global_vars.prolog_files[key] = open(key+".pl", 'a')
    
    if key == "rooms":
      global_vars.prolog_files[key].write(":- dynamic roomType/2, roomCenter/3.\n")
    elif key == "objects":
      global_vars.prolog_files[key].write(":- dynamic objectPos/4, objectType/2, objectLastSeen/2, objectAngle/2.\n")
    elif key == "graph":
      global_vars.prolog_files[key].write(":- dynamic edge/2, dyn_node/3, stat_node/3.\n")
    elif key == "centroids":
      for i in xrange(len(cellVectorX)):
        for j in xrange(len(cellVectorY)):
          global_vars.prolog_files[key].write("cellCenterInMap(%s,%s,%s,%s).\n"%(i, j, cellVectorX[i], cellVectorY[j]))
    
    global_vars.prolog_files[key].flush()
    
  global_vars.prolog_load_handle("centroids.pl")
  global_vars.prolog_load_handle("plans.pl")
  global_vars.prolog_load_handle("searchGraph.pl")
  global_vars.prolog_load_handle("cone_based_reasoner.pl")
  global_vars.prolog_load_handle("logicalFormTranslator.pl")
  global_vars.prolog_load_handle("plan_reasoner.pl")

def flush_prolog_files():
  for key in global_vars.prolog_files.keys():
    global_vars.prolog_files[key].flush()
    
def get_file_content(key):
  global_vars.prolog_files[key].close()
  global_vars.prolog_files[key] = open(key+'.pl', 'r')
  content = global_vars.prolog_files[key].readlines()
  global_vars.prolog_files[key].close()
  global_vars.prolog_files[key] = open(key+'.pl', 'a')
  return content

def log_kb():
  if not os.path.isdir("logged_pl"):
    os.mkdir("logged_pl")
  
  for key in global_vars.prolog_files.keys():
    try:
      shutil.copy('%s.pl'%key, 'logged_pl/%s-%s.pl'%(key,str(datetime.datetime.now())))
    except:
      log_to_terminal('Could not log file %s.pl because it does not exist'%key)

###################################################################################
### Doors processing

def are_same_door(coord1, coord2):
  for x,y in [(coord1[1]+i,coord1[2]+j) for i in (-1,0,1) for j in (-1,0,1) if i != 0 or j != 0]:
    if x == coord2[1] and y == coord2[2] and coord1[0].split("#")[0] == coord2[0].split("#")[0]:
      return True
  return False
  
def are_adjacent_door(coord1, coord2):
  for x,y in [(coord1[1]+i,coord1[2]+j) for i in (-1,0,1) for j in (-1,0,1) if i != 0 or j != 0]:
    if x == coord2[1] and y == coord2[2] and coord1[0].split("#")[0] != coord2[0].split("#")[0]:
      return True
  return False
  
def write_doors_to_file(doors):
  door_set = {}
  if len(doors) != 0:
    doors.pop(0)
  
  for door in doors:
    door_atom = door[0].replace("#","_door")
    logical_form = "objectPos(%s,%s,%s,0)"%(door_atom, door[1], door[2])
    global_vars.prolog_files["objects"].write(logical_form + ".\n")
    global_vars.prolog_assert_handle(logical_form)
    
    logical_form = "stat_node(%s,%s,%s)"%(door_atom, door[1], door[2])
    global_vars.prolog_files["graph"].write(logical_form + ".\n")
    
    for other_door in doors:
      if (door[0] != other_door[0] and are_same_door(door,other_door)):
        other_door[0] = door[0]
  
  for door in doors:
    if not door[0] in door_set:
      logical_form = "objectType(%s,door)"%door[0].replace("#","_door")
      global_vars.prolog_files["objects"].write(logical_form + ".\n")
      global_vars.prolog_assert_handle(logical_form)
      door_set[door[0]] = ""

###################################################################################
### Topological Graph Generator

def create_topological_graph(dyn_nodes, doors):
  if len(dyn_nodes) != 0:
    dyn_nodes.pop(0)
  door_set = {}
  
  # attaching dyn nodes to static
  for node in dyn_nodes:
    for door in doors:
      if not door[0] in door_set and door[0].split("#")[0] == node:
        logical_form = "edge(%s,%s)"%(node,door[0].replace("#","_door"))
        global_vars.prolog_files["graph"].write(logical_form + ".\n")
        door_set[door[0]] = ""
        if(global_vars.DRAW_GRAPH):
          global_vars.G.add_edge(node, door[0].replace("#","_door"), weight=0.4)

  # attaching static nodes
  door_set.clear()
  for door in doors:
    if not door[0] in door_set: 
      for other_door in doors:
        if are_adjacent_door(door,other_door):
          logical_form = "edge(%s,%s)"%(door[0].replace("#","_door"),other_door[0].replace("#","_door"))
          global_vars.prolog_files["graph"].write(logical_form + ".\n")
          door_set[door[0]] = ""
          if(global_vars.DRAW_GRAPH):
            global_vars.G.add_edge(other_door[0].replace("#","_door"), door[0].replace("#","_door"), weight=0.4)
          break

###################################################################################
### Draw Topological Graph

def drawGraph():
  elarge=[(u,v) for (u,v,d) in global_vars.G.edges(data=True) if d['weight'] >0.5]
  esmall=[(u,v) for (u,v,d) in global_vars.G.edges(data=True) if d['weight'] <=0.5]

  pos=nx.spring_layout(global_vars.G, dim=2, pos=None, fixed=None, iterations=100, weight='weight', scale=10) # positions for all nodes

  nx.draw_networkx_nodes(global_vars.G,pos,alpha=0.5,node_shape='^',node_size=70)
  nx.draw_networkx_edges(global_vars.G,pos,edgelist=elarge,width=3)
  nx.draw_networkx_edges(global_vars.G,pos,edgelist=esmall,width=3,alpha=0.5,edge_color='b',style='dashed')

  for key in pos:
    pos[key][1] += 0.3 
    
  nx.draw_networkx_labels(global_vars.G,pos,font_size=14, font_color='k',font_family='sans-serif')

  plt.axis('off')
  plt.savefig("topological_graph.png")
  plt.show()
  log_to_terminal("Saved topological_graph image as topological_graph.png")

###################################################################################
### Object Handling 

def forget(obj_tag):
  obj_atom = get_obj_atom('', obj_tag)
  log_to_terminal("forgetting %s"%obj_atom)
  global_vars.prolog_files["objects"].close()
  global_vars.prolog_files["objects"] = open('objects.pl', 'r')
  content = global_vars.prolog_files["objects"].readlines()
  global_vars.prolog_files["objects"].close()
  os.remove("objects.pl")
  global_vars.prolog_files["objects"] = open('objects.pl', 'a')

  for line in content:
    if (not obj_atom in line) or ( ("replaced_by" in line) and (obj_atom + ")." in line)):
      global_vars.prolog_files["objects"].write(line)
    else:
      global_vars.prolog_retract_handle(line.replace(".\n",""))

def learn(room_tag, obj_tag, obj_par, column, row, assertions_list=[""], asserting=True):
  obj_atom = get_obj_atom(room_tag, obj_tag)
  obj_time = get_obj_time(obj_tag)
  obj_type = get_obj_type(obj_tag)
  log_to_terminal("learning %s"%obj_atom)

  logical_form = "objectPos(%s,%s,%s,0)"%(obj_atom, column, row)
  global_vars.prolog_files["objects"].write(logical_form + ".\n")
  if asserting:
    global_vars.prolog_assert_handle(logical_form)
  
  logical_form = "objectType(%s,%s)"%(obj_atom, obj_type)
  if not (logical_form in assertions_list):
    assertions_list.append(logical_form)
    global_vars.prolog_files["objects"].write(logical_form + ".\n")
    if asserting:
      global_vars.prolog_assert_handle(logical_form)
  
  logical_form = "objectLastSeen(%s,%s)"%(obj_atom, obj_time)
  if not (logical_form in assertions_list):
    assertions_list.append(logical_form)
    global_vars.prolog_files["objects"].write(logical_form + ".\n")
    if asserting:
      global_vars.prolog_assert_handle(logical_form)
  
  #Expecting obj1par1~obj1par1value1^obj1par1value2...~obj1par2...#obj2par1~obj2par1value1^obj2par1value2...
  if (obj_par != "0"):
    obj_par_list = obj_par.split("~")
    for k in xrange(int(len(obj_par_list)/2)):
      par_name_index = 2*k
      par_value_index = 2*k+1
      if obj_par_list[par_name_index] != "objectAngle":
        logical_form = "%s(%s,%s)"%(obj_par_list[par_name_index], obj_atom,obj_par_list[par_value_index])
      else:
        angle = float(obj_par_list[par_value_index])*180/math.pi
        logical_form = "%s(%s,%s)"%(obj_par_list[par_name_index], obj_atom, int(round(angle,-1)))
        
      if not (logical_form in assertions_list):
        assertions_list.append(logical_form)
        global_vars.prolog_files["objects"].write(logical_form + ".\n")
        if asserting:
          global_vars.prolog_assert_handle(logical_form)
  
  global_vars.last_object_memorized = obj_atom
  
def remember(old_object_atom, new_object_atom = "", asserting = True):
  global_vars.prolog_files["objects"].close()
  global_vars.prolog_files["objects"] = open('objects.pl', 'r')
  content = global_vars.prolog_files["objects"].readlines()
  global_vars.prolog_files["objects"].close()
  os.remove("objects.pl")
  global_vars.prolog_files["objects"] = open('objects.pl', 'a')
  
  for line in content:
    if not old_object_atom in line:
      global_vars.prolog_files["objects"].write(line)
    else:
      global_vars.prolog_retract_handle(line.replace(".\n",""))
      logical_form = "prev_"+line
      global_vars.prolog_files["old_objects"].write(logical_form)
      if asserting:
          global_vars.prolog_assert_handle(logical_form)
      
  if (new_object_atom != ""):
    logical_form = "replaced_by(%s,%s)"%(old_object_atom, new_object_atom)
    global_vars.prolog_files["old_objects"].write(logical_form+".\n")
    if asserting:
      global_vars.prolog_assert_handle(logical_form)

###################################################################################
### Handle Inconsistencies

def get_inconsistency_for_element(room, element):
    
    old_objects         = []
    element_cells       = []
    element_type        = get_obj_type(element)
    element             = get_obj_atom(room, element)
    
    result = global_vars.prolog_service_handle("objectPos", [element, "X", "Y", "Z"])
    for answer in result.ris:
      element_cells.append([answer.atoms[1], answer.atoms[2]])

    for coords in element_cells:
      result = global_vars.prolog_service_handle("objectPos", ["X", coords[0], coords[1], "Z"])
      for answer in result.ris:
        old_objects.append(answer.atoms[0])
    
    #remove double entries
    old_objects = list(set(old_objects))
    old_objects.remove(element)
    
    inconsistency = ["EMPTY",element,[""]]
    
    for i in xrange(len(old_objects)):
      old_object_type = get_obj_type(old_objects[i])
      
      if old_object_type == element_type:
        inconsistency = ["EQUAL",element,[old_objects[i]]]
        break
      elif is_object_synonym_of(element_type,old_object_type):
        inconsistency = ["SYNONYM",element,[old_objects[i]]]
        break
      elif is_object_generalization_of(element_type,old_object_type):
        inconsistency = ["MORE_GENERAL",element,[old_objects[i]]]
        break
      elif is_object_specialization_of(element_type,old_object_type):
        inconsistency = ["LESS_GENERAL",element,[old_objects[i]]]
        break
      else:
        inconsistency = ["DIFFERENT",element,old_objects]
    
    log_to_terminal("Found inconsistency %s for object %s."%(inconsistency, element))
    return inconsistency
    
def process_inconsistencies(room, element):
    inconsistency = get_inconsistency_for_element(room, element) #inconsistency = [type of inconsistency, new_object_ID, [old_related_object_ID]]
    objectType = global_vars.prolog_service_handle("objectType", [inconsistency[1], "Z"]).ris[0].atoms[1].replace("_", " ")
      
    if ("EMPTY" == inconsistency[0]):
      msg ="MemResOther_I have memorized %s"%objectType
    
    elif ("EQUAL" == inconsistency[0]):  
      global_vars.remove_object_by_name.publish(String(get_obj_sem_map_name(inconsistency[1])))
      msg = "MemResOther_I already know that %s is here"%objectType
      
    elif ("SYNONYM" == inconsistency[0]):
      oldObjectType = global_vars.prolog_service_handle("objectType", [inconsistency[2][0], "Z"]).ris[0].atoms[1].replace("_", " ")
      global_vars.remove_object_by_name.publish(String(get_obj_sem_map_name(inconsistency[1])))
      msg = "MemResOther_I know that %s is here and that %s is a synonym"%(oldObjectType,objectType)
      
    elif ("MORE_GENERAL" == inconsistency[0]):
      oldObjectType = global_vars.prolog_service_handle("objectType", [inconsistency[2][0], "Z"]).ris[0].atoms[1].replace("_", " ")
      global_vars.remove_object_by_name.publish(String(get_obj_sem_map_name(inconsistency[1])))
      msg = "MemResOther_I know that %s is here and that is more specific than %s"%(oldObjectType,objectType)
    
    elif ("LESS_GENERAL" == inconsistency[0]):
      oldObjectType = global_vars.prolog_service_handle("objectType", [inconsistency[2][0], "Z"]).ris[0].atoms[1].replace("_", " ")
      global_vars.remove_object_by_name.publish(String(get_obj_sem_map_name(inconsistency[2][0])))
      msg = "MemResOther_I know that %s is here and that is more general than %s"%(oldObjectType,objectType)
      
    elif ("DIFFERENT" == inconsistency[0]):
      objectType = ""
      objectID = ""
      for i in inconsistency[2]:
        result = global_vars.prolog_service_handle("objectType", [i, "Z"])
        objectType = (objectType + result.ris[0].atoms[1].replace("_", " ") + ",").replace("_","#")
        objectID = (objectID + i + ",").replace("_","#")
      msg = "MemResDiff_"+objectType[:-1]+"_"+objectID[:-1]
    
    log_to_terminal("Sending acknowledge mesage %s for object %s"%(msg, element))
    global_vars.event_pub.publish(msg)
    return inconsistency[0]

###################################################################################
### Object linguistical relation

def is_object_synonym_of(object1, object2):
    try:
      result = global_vars.prolog_service_handle('synonym', [object1, object2])
      if (len(result.ris) != 0):
        return True
      else:
        return False

    except:
      return False

def is_object_generalization_of(object1, object2):
    try:
      result = global_vars.prolog_service_handle('isA', [object2, object1])
      if (len(result.ris) != 0):
        return True

    except:
      return False
      
def is_object_specialization_of(object1, object2):
    try:
      result = global_vars.prolog_service_handle('isA', [object1, object2])
      if (len(result.ris) != 0):
        return True

    except:
      return False
      
###################################################################################
### Others

def calculate_and_assert_rooms_center_and_type(room_list, asserting = True):
  room_type_list      = ["office", "room", "corridor", "printerRoom", "phdRoom"]
  
  for room in room_list:
    centroid            = global_vars.prolog_service_handle("roomCellCentroid", [room, "X", "Y"]).ris[0]
    room_center         = [int(round(float(centroid.atoms[1]))), int(round(float(centroid.atoms[2])))]
    room_type_in_list   = False
    
    for room_type in room_type_list:
      if room_type in room.lower():
        logical_form            = "roomType(%s,%s)"%(room, room_type)
        room_type_in_list       = True
    
    if not room_type_in_list:
      logical_form      = "roomType(%s,%s)"%(room, room)
      
    logical_form1       = "roomCenter(%s,%s,%s)"%(room, room_center[0], room_center[1])
    logical_form2       = "dyn_node(%s,%s,%s)"%(room, room_center[0], room_center[1])
    
    global_vars.prolog_files["rooms"].write(logical_form + ".\n")
    global_vars.prolog_files["rooms"].write(logical_form1 + ".\n")
    global_vars.prolog_files["graph"].write(logical_form2 + ".\n")
    
    if asserting:
      global_vars.prolog_assert_handle(logical_form)
      global_vars.prolog_assert_handle(logical_form1)
      global_vars.prolog_assert_handle(logical_form2)
  

def create_obj_model_with_id(obj_id):
  pose_file     = global_vars.images_log_folder + '/pose-'  + obj_id + ".txt"
  rgb_file      = global_vars.images_log_folder + '/rgb-'   + obj_id + ".png"
  raw_file      = global_vars.images_log_folder + '/raw-'   + obj_id + ".png"
  depth_file    = global_vars.images_log_folder + '/depth-' + obj_id + ".png"
  
  log_to_terminal("Creating object model for id %s from files: %s, %s, %s, %s"%(obj_id,rgb_file,raw_file,depth_file, pose_file))
  
  try:
    global_vars.create_obj_model(rgb_file, raw_file, depth_file, pose_file)
  
  except:
    err_to_terminal("Could not call service extract_object")

def filter_objects_pars_lists(objects, pars, old_objects, old_pars):
  for index_obj in reversed(xrange(len(objects))):
    for index_old_obj in reversed(xrange(len(old_objects))):
      if objects[index_obj] == old_objects[index_old_obj]:
        old_objects.pop(index_old_obj)
        old_pars.pop(index_old_obj)
        objects.pop(index_obj)
        pars.pop(index_obj)
        break
