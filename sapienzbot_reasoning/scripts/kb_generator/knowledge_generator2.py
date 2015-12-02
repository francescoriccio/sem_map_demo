#!/usr/bin/env python
import roslib; roslib.load_manifest('sapienzbot_reasoning')
PKG = 'sapienzbot_reasoning'
NODE = 'knowledge_generator'

import os
import re
import sys
import shutil
import datetime
import difflib

import sapienzbot_reasoning
from sapienzbot_reasoning.srv import *
from sapienzbot_reasoning.msg import *
from semantic_map_extraction.srv import *
from prolog_interface.srv import *
from prolog_interface.msg import *
from std_msgs.msg import *

import rospy

DEBUG = False
DRAWING = False
WRITE_PROLOG_FILES = True
LOG_KB = True

WAITING_TIMEOUT = 10

tag_object_can_be_deleted = ""
categorizeRelationAnswer = ""
initializing = False
forms_vector = []
parameters_list = []
matrix_old = []
old_width = 0

if(DRAWING):
  try:
    import matplotlib.pyplot as plt
  except:
    raise
  import networkx as nx
  G=nx.Graph()

def already_attached_to_other_stat_node(stat_node_name):
    result = []
    answer = False
    room = stat_node_name.split("_")[0]
    
    if (DEBUG):
      print "["+NODE+"] \033[92mChecking if stat_node %s already attached to other stat_node...\033[0m"%(stat_node_name)
        
    try:
      result = prolog_service_handle ('edge', ["X", stat_node_name])
      for i in xrange(len(result.ris)):
        to_be_analyzed = result.ris[i].atoms[0]
        if (to_be_analyzed.split("_")[0] != room):
          answer = True 

    except rospy.ServiceException, e:
      if (DEBUG):
        print "["+NODE+"] \033[91mCould not find any stat_node attached:\033[0m"
        print "%s"%e

    if (answer):
      if (DEBUG):
        print "["+NODE+"] \033[92mstat_node %s already attached to other stat node\033[0m"%(stat_node_name)
    else:
      if (DEBUG):
        print "["+NODE+"] \033[92mstat_node %s not attached to any stat node, proceding with the attachment...\033[0m"%(stat_node_name)

    return answer

def assert_room_type(room_tag, file_node_name):
  lowerName = room_tag.lower()
  #writing room type
  if ("office" in lowerName):
    logical_form = "roomType(%s_center,office).\n"%(room_tag)
    assert_to_prolog(logical_form)
    if(WRITE_PROLOG_FILES):
      file_node_name.write(logical_form)
  elif ("corridor" in lowerName):
    logical_form = "roomType(%s_center,corridor).\n"%(room_tag)
    assert_to_prolog(logical_form)
    if(WRITE_PROLOG_FILES):
      file_node_name.write(logical_form)
  elif ("toilet" in lowerName):
    logical_form = "roomType(%s_center,toilet).\n"%(room_tag)
    assert_to_prolog(logical_form)
    if(WRITE_PROLOG_FILES):
      file_node_name.write(logical_form)
  elif ("kitchen" in lowerName):
    logical_form = "roomType(%s_center,kitchen).\n"%(room_tag)
    assert_to_prolog(logical_form)
    if(WRITE_PROLOG_FILES):
      file_node_name.write(logical_form)
  elif ("living_room" in lowerName):
    logical_form = "roomType(%s_center,living_room).\n"%(room_tag)
    assert_to_prolog(logical_form)
    if(WRITE_PROLOG_FILES):
      file_node_name.write(logical_form)
  elif ("room" in lowerName):
    logical_form = "roomType(%s_center,room).\n"%(room_tag)
    assert_to_prolog(logical_form)
    if(WRITE_PROLOG_FILES):
      file_node_name.write(logical_form)
      
  #writing room spec
  if ("nardi" in lowerName):
    logical_form = "roomSpec(%s_center,nardi).\n"%(room_tag)
    assert_to_prolog(logical_form)
    if(WRITE_PROLOG_FILES):
      file_node_name.write(logical_form)
  elif ("ciciani" in lowerName):
    logical_form = "roomSpec(%s_center,ciciani).\n"%(room_tag)
    assert_to_prolog(logical_form)
    if(WRITE_PROLOG_FILES):
      file_node_name.write(logical_form)
  elif ("phd" in lowerName):
    logical_form = "roomSpec(%s_center,phd).\n"%(room_tag)
    assert_to_prolog(logical_form)
    if(WRITE_PROLOG_FILES):
      file_node_name.write(logical_form)
  elif ("printer" in lowerName):
    logical_form = "roomSpec(%s_center,printer).\n"%(room_tag)
    assert_to_prolog(logical_form)
    if(WRITE_PROLOG_FILES):
      file_node_name.write(logical_form)

def assert_to_prolog(assertion):
    if (DEBUG):
      print "["+NODE+"] \033[92mAsserting: %s\033[0m"%(assertion.replace("\n", ""))
    
    service_handle = rospy.ServiceProxy('/prolog_interface/prolog_assert', prologAssertSrv)
    result = service_handle(assertion.replace(".\n", ""))
    
def build_topological_graph(forms_vector, file_graph_name):   
  for i in xrange(len(forms_vector)):
    node, argument = forms_vector[i].split("(")
    if (node == "stat_node"):
      stat_node_name = argument.split(",")[0]

      if(is_corresponding_dyn_node_present(stat_node_name)):
        assertion = "edge(%s_center,%s).\n"%(stat_node_name.split("_door")[0],stat_node_name)
        assert_to_prolog(assertion)
        if(WRITE_PROLOG_FILES):
          file_graph_name.write(assertion)

        if(DRAWING):
          G.add_edge("%s"%stat_node_name.split("_")[0],"%s_door"%stat_node_name.split("_")[0],weight=0.4)

        if (DEBUG):
          print "["+NODE+"] \033[92mStat_node %s attached to node %s_center.\033[0m"%(stat_node_name, stat_node_name.split("_")[0])

      if (not already_attached_to_other_stat_node(stat_node_name)):
        other_node_name = search_for_corresponding_stat_node(stat_node_name, argument.split(";")[1])
        if (other_node_name != "NULL"):
          assertion = "edge(%s,%s).\n"%(stat_node_name,other_node_name)
          assert_to_prolog(assertion)
          if(WRITE_PROLOG_FILES):
            file_graph_name.write(assertion)
          
          if(DRAWING):
            G.add_edge("%s_door"%stat_node_name.split("_")[0],"%s_door"%other_node_name.split("_")[0],weight=0.4)
          
          if (DEBUG):
            print "["+NODE+"] \033[92mStat_node %s attached to node %s.\033[0m"%(stat_node_name, other_node_name)
  
  print "["+NODE+"] \033[92mDone writing topological graph.\033[0m"

def drawGraph():
  elarge=[(u,v) for (u,v,d) in G.edges(data=True) if d['weight'] >0.5]
  esmall=[(u,v) for (u,v,d) in G.edges(data=True) if d['weight'] <=0.5]

  pos=nx.spring_layout(G, dim=2, pos=None, fixed=None, iterations=100, weight='weight', scale=10) # positions for all nodes

  nx.draw_networkx_nodes(G,pos,alpha=0.5,node_shape='^',node_size=70)
  nx.draw_networkx_edges(G,pos,edgelist=elarge,width=3)
  nx.draw_networkx_edges(G,pos,edgelist=esmall,width=3,alpha=0.5,edge_color='b',style='dashed')

  for key in pos:
    pos[key][1] += 0.3 
    
  nx.draw_networkx_labels(G,pos,font_size=14, font_color='k',font_family='sans-serif')

  plt.axis('off')
  plt.savefig("topological_graph.png")
  print "["+NODE+"] \033[92mDrew topological_graph.png\033[0m"
  #plt.show()
  
def get_inconsistency_for_element(element):
    
    if (DEBUG):
      print "["+NODE+"] \033[92mCalled element inconsistency method.\033[0m" 
    
    old_objects         = []
    element_cells       = []
    element_type        = get_object_type(element)
    
    result = prolog_service_handle("objectPos", [element, "X", "Y", "Z"])
    for i in result.ris:
      element_cells.append([i.atoms[1],i.atoms[2]])

    for i in element_cells:
      result = prolog_service_handle("objectPos", ["X", i[0], i[1], "Z"])
      for j in result.ris:
        old_objects.append(j.atoms[0])
    
    #remove double entries
    old_objects = removeDuplicatesElementsInList(old_objects)
    old_objects.remove(element)
    
    inconsistency = ["EMPTY",element,[""]]
    
    for i in xrange(len(old_objects)):
      old_object_type = get_object_type(old_objects[i])
      
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
          
    #if (DEBUG):
    print "["+NODE+"] \033[92mFound inconsistency %s for object %s.\033[0m"%(inconsistency, element) 
    
    return inconsistency
 
def get_object_type(a):
  return a.split("~")[0].replace(a.split("_")[0]+"_","")

def get_synonym(obj):
    if (DEBUG):
      print "["+NODE+"] \033[92mLooking up the ground synonym of %s\033[0m"%(obj)
    
    synonym = ""
    
    try:
      result = prolog_service_handle ('synonym', [obj, "X"])
      if (len(result.ris) != 0):
        if (DEBUG):
          print "["+NODE+"] \033[92mFound the synonym %s.\033[0m"%result.ris[0].atoms[1]
        synonym = result.ris[0].atoms[1]

    except rospy.ServiceException, e:
      if (DEBUG):
        print "["+NODE+"] \033[91mCould not find a synonym for %s.\033[0m"%obj
    
    if (synonym == ""):
      return obj
    else:
      return synonym

def is_corresponding_dyn_node_present(stat_node_name):
    room = "%s_center"%(stat_node_name.split("_door")[0])
    result = []

    if (DEBUG):
      print "["+NODE+"] \033[92mLooking dyn_node %s for stat_node %s...\033[0m"%(room,stat_node_name)
    
    try:
        result = prolog_service_handle ('dyn_node', [room, "X", "Y"])
        if (len(result.ris) != 0):
          if (DEBUG):
            print "["+NODE+"] \033[92mFound!\033[0m"
          return True

    except rospy.ServiceException, e:
      if (DEBUG):
            print "["+NODE+"] \033[91mCould not find corresponding dyn_node for the following reason:\033[0m%s"%e
      return False
 
def is_object_synonym_of(object1, object2):
    if (DEBUG):
      print "["+NODE+"] \033[92mChecking if %s is synonym of %s...\033[0m"%(object1,object2)
    try:
      result = prolog_service_handle ('synonym', [object1, object2])
      if (len(result.ris) != 0):
        if (DEBUG):
          print "["+NODE+"] \033[92mThey are synonyms.\033[0m"
        return True
      else:
        return False

    except rospy.ServiceException, e:
      if (DEBUG):
        print "["+NODE+"] \033[91mThey do not appear to be synonyms.\033[0m"
      return False
      
def is_object_generalization_of(object1, object2):
    if (DEBUG):
      print "["+NODE+"] \033[92mChecking if %s is a generalizaiont of %s...\033[0m"%(object1,object2)
    try:
      result = prolog_service_handle ('isA', [object2, object1])
      if (len(result.ris) != 0):
        if (DEBUG):
          print "["+NODE+"] \033[92mFound it as a generalization.\033[0m"
        return True

    except rospy.ServiceException, e:
      if (DEBUG):
            print "["+NODE+"] \033[91mIt does not appear to be a generalization.\033[0m"
      return False
      
def is_object_specialization_of(object1, object2):
    if (DEBUG):
      print "["+NODE+"] \033[92mChecking if %s is a specialization of %s...\033[0m"%(object1,object2)
    try:
      result = prolog_service_handle ('isA', [object1, object2])
      if (len(result.ris) != 0):
        if (DEBUG):
          print "["+NODE+"] \033[92mFound it as a specialization.\033[0m"
        return True

    except rospy.ServiceException, e:
      if (DEBUG):
            print "["+NODE+"] \033[91mIt does not appear to be a specialization.\033[0m"
      return False
 
def is_number(s):
    try:
      float(s)
      return True
    except ValueError:
      return False
      
def process_door_adj(door_adj, room_tag, column, row):
  logical_form = "stat_node(%s_door%s%s,%s,%s).\n"%(room_tag,column,row,column,row)
  assert_to_prolog(logical_form)
  if(WRITE_PROLOG_FILES):
    file_node_name.write(logical_form)

  if (door_adj.split('#')[0] == "true"):
    forms_vector.append("%s;%s"%(logical_form,1))
  elif (door_adj.split('#')[1] == "true"):
    forms_vector.append("%s;%s"%(logical_form,2))
  elif (door_adj.split('#')[2] == "true"):
    forms_vector.append("%s;%s"%(logical_form,3))
  elif (door_adj.split('#')[3] == "true"):
    forms_vector.append("%s;%s"%(logical_form,4))
    
  logical_form = "objectPos(%s_door%s%s,%s,%s,0).\n"%(room_tag,column,row,column,row)
  assert_to_prolog(logical_form)
  if(WRITE_PROLOG_FILES):
    file_obj_name.write(logical_form)
  
  logical_form = "objectType(%s_door%s%s,%s).\n"%(room_tag,column,row,get_synonym("door"))
  assert_to_prolog(logical_form)
  if(WRITE_PROLOG_FILES):
    file_obj_name.write(logical_form)

def process_inconsistencies(element):                           #checking if there are inconsistencies in the just memorized cells, if so handle the specific case. 
    inconsistency = get_inconsistency_for_element(element)      #inconsistency = [type of inconsistency, new_object_ID, [old_related_object_ID]]
    result = prolog_service_handle("objectType", [inconsistency[1], "Z"])
    objectType = result.ris[0].atoms[1].replace("_", " ")
      
    if ("EMPTY" == inconsistency[0]):
      msg ="MemResOther_I have memorized %s"%objectType
    
    elif ("EQUAL" == inconsistency[0]):  
      remove_object_from_cellMap(inconsistency[1])
      msg = "MemResOther_I already know that %s is here"%objectType
      
    elif ("SYNONYM" == inconsistency[0]):
      result = prolog_service_handle("objectType", [inconsistency[2][0], "Z"])
      oldObjectType = result.ris[0].atoms[1].replace("_", " ")
      remove_object_from_cellMap(inconsistency[1])
      msg = "MemResOther_I know that %s is here and that %s is a synonym"%(oldObjectType,objectType)
      
    elif ("MORE_GENERAL" == inconsistency[0]):
      result = prolog_service_handle("objectType", [inconsistency[2][0], "Z"])
      oldObjectType = result.ris[0].atoms[1].replace("_", " ")
      remove_object_from_cellMap(inconsistency[1])
      msg = "MemResOther_I know that %s is here and that is more specific than %s"%(oldObjectType,objectType)
    
    elif ("LESS_GENERAL" == inconsistency[0]):
      result = prolog_service_handle("objectType", [inconsistency[2][0], "Z"])
      oldObjectType = result.ris[0].atoms[1].replace("_", " ")
      remove_object_from_cellMap(inconsistency[2][0])
      msg = "MemResOther_I know that %s is here and that is more general than %s"%(oldObjectType,objectType)
      
    elif ("DIFFERENT" == inconsistency[0]):
      objectType = ""
      objectID = ""
      for i in inconsistency[2]:
        result = prolog_service_handle("objectType", [i, "Z"])
        objectType = (objectType + result.ris[0].atoms[1].replace("_", " ") + ",").replace("_","#")
        objectID = (objectID + i + ",").replace("_","#")
      msg = "MemResDiff_"+objectType[:-1]+"_"+objectID[:-1]
    
    print "["+NODE+"] \033[92mSending acknowledge mesage %s for object %s\033[0m"%(msg, element)
    event_pub.publish(msg)

def process_object_tag(obj_tag, obj_par, room_tag, column, row):
  global parameters_list
  obj_tag = obj_tag.split("#")
  obj_par = obj_par.split("#")
  
  if(room_tag == "0"):
    print "["+NODE+"] \033[91mWarning: found objects %s with cell coordinates %s,%s in a non classified room!. Setting this room to the one containing the robot.\033[0m"%(obj_tag,i/width,i%width)
    logical_form = "cellCoordIsPartOf(%s,%s,%s_center)"
    assert_to_prolog(logical_form)
    if(WRITE_PROLOG_FILES):
        file_node_name.write(logical_form)
    
  if len(obj_par) != len(obj_tag):
    for i in xrange(len(obj_tag)-len(obj_par)):
      obj_par.append("0")
    
  for j in xrange(len(obj_tag)):
    if (obj_tag[j].split("~")[0] == "room_center"):
      logical_form = "dyn_node(%s_center,%s,%s).\n"%(room_tag,column,row)
      forms_vector.append("%s;%s"%(logical_form,0))
      assert_to_prolog(logical_form)
      assert_room_type(room_tag, file_node_name)
      if(WRITE_PROLOG_FILES):
        file_node_name.write(logical_form)
      
    else: 
      logical_form = []
      logical_form.append("objectPos(%s_%s,%s,%s,0).\n"%(room_tag,obj_tag[j],column,row))
      logical_form.append("objectType(%s_%s,%s).\n"%(room_tag,obj_tag[j],get_synonym(obj_tag[j].split("~")[0])))
      
      #checking if i have a single parameter or multiples
      #Expecting a string formatted as follows: obj1par1~obj1par1value1^obj1par1value2...~obj1par2~obj1par2value1^obj1par2value2...#obj2par1~obj2par1value1^obj2par1value2...~obj2par2~obj2par2value1^obj2par2value2...
      if (obj_par[j] != "0"):
        par_list = obj_par[j].split("~")
        for k in xrange(int(len(par_list)/2)):
          logical_form.append("%s(%s_%s,%s).\n"%(par_list[2*k],room_tag, obj_tag[j],par_list[2*k+1]))
          if (not par_list[2*k] in parameters_list):
            parameters_list.append(par_list[2*k])
          
      for k in xrange(len(logical_form)):
        assert_to_prolog(logical_form[k])
        if(WRITE_PROLOG_FILES):
          file_obj_name.write(logical_form[k])

def process_room_tag(room_tag, column, row):
  logical_form = "cellCoordIsPartOf(%s,%s,%s_center).\n"%(column,row,room_tag) #sto mettendo il _center per fare il room of
  assert_to_prolog(logical_form)
  if(WRITE_PROLOG_FILES):
    file_cells_name.write(logical_form)

def removeDuplicatesElementsInList(listToBeFiltered):
    copy = [listToBeFiltered[0]]
    for i in listToBeFiltered:
      foundDuplicate = False
      for j in copy:
        if i == j:
          foundDuplicate = True
          break

      if not foundDuplicate:
        copy.append(i)
    
    return copy 

def remove_object_from_cellMap(objectName):
    if (DEBUG):
      print "["+NODE+"] \033[92mMethod remove objects called\033[0m"
    
    if not isinstance(objectName, basestring): #used to call the method both by code and by sevice 
      objectName = str(objectName.data)
      
    objectName = objectName.replace("#","_")
    obj_type = objectName.replace(objectName.split("_")[0]+"_","")
    obj_room = objectName.split("_")[0]
    remove_object_by_name.publish(String(obj_type)) #remove obj in cell map
    #remove_obj_from_prolog("%s;%s;;"%(obj_room,obj_type)) #remove obj in prolog
    if (DEBUG):
      print "["+NODE+"] \033[92mRemoved Object: %s\033[0m"%(objectName)
      
def remove_object_from_cellMap_srv(req):
    remove_object_from_cellMap(req.objName)
    return RemoveObjectResponse("Removed")

def remove_obj_from_prolog(oldElement):
  global file_obj_name
  room_tag, obj_tag, door_adj, obj_par = oldElement.split(";")
  logical_form = ""
  logical_form_1 = ""
  print "asked to remove " + oldElement
  if (obj_tag != "0"):
    obj_tag = obj_tag.split("#")
    obj_par = obj_par.split("#")
    if len(obj_par) != len(obj_tag):
      for i in xrange(len(obj_tag)-len(obj_par)):
        obj_par.append("0")
    
    for j in xrange(len(obj_tag)):
      if (obj_tag[j].split("~")[0] != "room_center"): 
        logical_form = []
        logical_form.append("objectPos(%s_%s,X,Y,T)"%(room_tag,obj_tag[j]))
        logical_form.append("objectType(%s_%s,%s)"%(room_tag,obj_tag[j],get_synonym(obj_tag[j].split("~")[0])))
        
        #checking if i have a single parameter or multiples
        #Expecting a string formatted as follows: obj1par1~obj1par1value1^obj1par1value2...~obj1par2~obj1par2value1^obj1par2value2...#obj2par1~obj2par1value1^obj2par1value2...~obj2par2~obj2par2value1^obj2par2value2...
        if (obj_par[j] != "0"):
          par_list = obj_par[j].split("~")
          for k in xrange(len(parameters_list)):
            logical_form.append("%s(%s_%s,X)"%(parameters_list[k],room_tag, obj_tag[j]))

        for k in xrange(len(logical_form)):
          retractAll_handle(logical_form[k])
        
        #canceling lines in prolog file objects.pl
        if(WRITE_PROLOG_FILES):
          file_obj_name.close()
          file_obj_name = open('objects.pl', 'r')
          content = file_obj_name.readlines()
          file_obj_name.close()
          os.remove("objects.pl")
          file_obj_name = open('objects.pl', 'a')
          for line in content:      
            if(not obj_tag[j] in line):
              file_obj_name.write(line)

def retract_to_prolog(retraction):
    service_handle = rospy.ServiceProxy('/prolog_interface/prolog_retract', prologRetractSrv)
    result = service_handle(retraction.replace(".\n", ""))
    if (DEBUG):
      print "["+NODE+"] \033[92mRetracted: %s\033[0m"%(retraction.replace("\n", ""))
 
def search_for_corresponding_stat_node(stat_node_name, case):
    if (DEBUG):
        print "["+NODE+"] \033[92mLooking for corresponding stat_node for %s with case %d\033[0m"%(stat_node_name,int(case))
      
    try:
      stat_node_result = prolog_service_handle('stat_node', [stat_node_name,"X","Y"])
      
      if (is_number(stat_node_result.ris[0].atoms[1]) and is_number(stat_node_result.ris[0].atoms[2])):
        if (int(case) == 1):
          x = int(stat_node_result.ris[0].atoms[1])
          y = int(stat_node_result.ris[0].atoms[2]) - 1
        elif (int(case) == 2):
          x = int(stat_node_result.ris[0].atoms[1]) + 1
          y = int(stat_node_result.ris[0].atoms[2]) 
        elif (int(case) == 3):
          x = int(stat_node_result.ris[0].atoms[1]) 
          y = int(stat_node_result.ris[0].atoms[2]) + 1
        elif (int(case) == 4):
          x = int(stat_node_result.ris[0].atoms[1]) - 1
          y = int(stat_node_result.ris[0].atoms[2]) 
      
        try:
          result = prolog_service_handle('stat_node', ["Name",str(x),str(y)])
          if (DEBUG):
            print "["+NODE+"] \033[92mFound: %s\033[0m"%(result.ris)
          for i in xrange(len(result.ris)):
            if (stat_node_name != result.ris[i].atoms[0]):
              return result.ris[i].atoms[0]
            
        except rospy.ServiceException, e:
          if (DEBUG):
            print "["+NODE+"] \033[92mCould not find corresponding stat_node for the following reason:\033[0m%s"%e
          return "NULL"

      else:
        if (DEBUG):
          print "["+NODE+"] \033[92mSkipping result %s due to not acceptable coordinates \033[0m"%(stat_node_result.ris[0])

    except rospy.ServiceException, e:
      if (DEBUG):
          print "["+NODE+"] \033[92mCould not find corresponding stat_node for the following reason:\033[0m%s"%e 

#######################################################################################
### Services  

def reset_prolog(req):
    print "["+NODE+"] \033[92mRequested to reset prolog knowledge\033[0m"
      
    try:
      reset_semantic_map = rospy.ServiceProxy('/semantic_map_extraction/delete_map', DeleteMap)
      resp1 = reset_semantic_map("Reset")
      if(resp1.ack == True):
        print "["+NODE+"] \033[92mSemantic map extraction node resetted\033[0m"
      else:
        print "["+NODE+"] \033[91mCould not reset map extraction node\033[0m"
        
    except:
      print "["+NODE+"] \033[91mCould not contact map extraction node\033[0m"
    return "Done"

def handle_create_topological_graph(req):
    global graph_matrix
    graph_matrix = []
    print "["+NODE+"] \033[92mReceived matrix.\033[0m" 
    aknowledgment = write_graph(req.height, req.width, req.matrix)
    return CreateTopologicalGraphResponse(aknowledgment)

def write_graph(height, width, matrix):
    global forms_vector, forms_vector_to_delete, matrix_old, G
    matrix_needs_to_be_updated = True
    new_object = None
    newElement = None

    print "["+NODE+"] \033[92mReceived matrix with width %s and length %s. Writinig facts...\033[0m"%(width,len(matrix)/width)

    if(matrix_old == []):
      for index_matrix in xrange(len(matrix)):
        if (matrix[index_matrix] != "0"):
          try:
            room_tag, obj_tag, door_adj, obj_par = matrix[index_matrix].split(";")
            row = int(index_matrix/width)
            column = int(index_matrix%width)
            if(room_tag != "0"):
              process_room_tag(room_tag, column, row)

            if (obj_tag != "0"):
              process_object_tag(obj_tag, obj_par, room_tag, column, row)
          
            if (door_adj != "false#false#false#false"):
              process_door_adj(door_adj, room_tag, column, row)

          except:
            print "["+NODE+"] \033[91mCould not process matrix element \"%s\". Skipping this cell...\033[0m"%(matrix[index_matrix])
            
      print "["+NODE+"] \033[92mDone writinig facts. Writing topological graph...\033[0m"
      
      # WRITING THE ACTUAL TOPOLOGICAL GRAPH INTO topological_graph.pl
      build_topological_graph(forms_vector, file_graph_name)

    else:
      for index_matrix in xrange(len(matrix)):
        if (matrix[index_matrix] != matrix_old[index_matrix]):
        # Since we already have a matricial representation of the map we just need to check the differences between the old and the new one
          newCell = matrix[index_matrix]
          oldCell = matrix_old[index_matrix]
         
          print "["+NODE+"] \033[92mMatrix element %s of value \"%s\" differs from old matrix element value \"%s\". Processing it...\033[0m"%(index_matrix,newCell,oldCell)

          if(oldCell != "0"):
            # Retracting the old past assertions for cell i
            remove_obj_from_prolog(oldCell)
           
          if (newCell != "0"):
            room_tag, obj_tag, door_adj, obj_par = newCell.split(";")
            
            if(obj_tag != "0"):
              # Asserting the new facts
              newElement = room_tag+"_"+obj_tag.split("#")[-1]
              
              row = int(index_matrix/width)
              column = int(index_matrix%width)
              
              process_object_tag(obj_tag, obj_par, room_tag, column, row)
              if (oldCell == "0"):
                #asserting the new cell
                process_room_tag(room_tag, column, row)

    if(matrix_needs_to_be_updated):
      matrix_old = matrix
      old_width = width

    if(DRAWING):
      drawGraph()
    
    if(WRITE_PROLOG_FILES):
      file_node_name.flush()
      file_obj_name.flush()
      file_graph_name.flush()
      file_cells_name.flush()
    
    if(LOG_KB):
      if not os.path.isdir("logged_pl"):
        os.mkdir("logged_pl")
      shutil.copy('objects.pl', 'logged_pl/objects-%s.pl'%str(datetime.datetime.now()))
      shutil.copy('cells.pl', 'logged_pl/cells-%s.pl'%str(datetime.datetime.now()))
      shutil.copy('topological_graph_facts.pl', 'logged_pl/topological_graph_facts-%s.pl'%str(datetime.datetime.now()))
      shutil.copy('topological_graph.pl', 'logged_pl/topological_graph-%s.pl'%str(datetime.datetime.now()))
      
    if newElement != None:
      process_inconsistencies(newElement)
      print "["+NODE+"] \033[92mDone writing facts.\033[0m"
    
    print "["+NODE+"] \033[92mReady to process another matrix.\033[0m"

 
#######################################################################################
### Main  

if __name__ == "__main__":
    rospy.init_node(NODE)
    rospy.wait_for_service('/prolog_interface/prolog_retract')
    rospy.wait_for_service('/prolog_interface/prolog_query')
    directory = rospy.get_param(NODE+'/prolog_path', "/pl/")
    robot_name = rospy.get_param('/robotname', "robot_0")
    remove_object_by_name = rospy.Publisher('remove_object_by_name', String)
    event_pub = rospy.Publisher("/"+robot_name+"/PNPConditionEvent", String)
    
    prolog_service_handle = rospy.ServiceProxy('/prolog_interface/prolog_query', prologSrv)
    retractAll_handle = rospy.ServiceProxy('/prolog_interface/prolog_retractall', prologRetractSrv)
    
    rospy.Subscriber("/remove_object_by_name_from_cellMap", String, remove_object_from_cellMap)
    
    rospy.Service('/reset_prolog', ResetProlog, reset_prolog)
    rospy.Service('/remove_object_from_cellMap', RemoveObject, remove_object_from_cellMap_srv)
    rospy.Service('/create_topological_graph', CreateTopologicalGraph, handle_create_topological_graph)
    
    print "["+NODE+"] \033[92mReady to create the topological graph.\033[0m"
    rospy.wait_for_service('/prolog_interface/prolog_assert')

    
    os.chdir(directory)
    if(WRITE_PROLOG_FILES):
      if(os.path.exists("topological_graph_facts.pl")):
        os.remove("topological_graph_facts.pl")
      if(os.path.exists("objects.pl")):
        os.remove("objects.pl")
      if(os.path.exists("topological_graph.pl")):
        os.remove("topological_graph.pl")
      if(os.path.exists("cells.pl")):
        os.remove("cells.pl")
        
      file_node_name = open('topological_graph_facts.pl', 'a')
      file_obj_name = open('objects.pl', 'a')
      file_graph_name = open('topological_graph.pl', 'a')
      file_cells_name = open('cells.pl', 'a')
    
    rospy.spin()
    