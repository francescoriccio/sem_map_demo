import global_vars
import os
import rospy
import re
import time

import matplotlib.pyplot as plt
import networkx as nx
from lxml import etree 
from support_functions import *

#######################################################################################
### Parsing Support Functions

### returns an incremental id for a variable. Used to name new variables
def getId():
  global_vars.idCounter = global_vars.idCounter + 1
  return "~[%s]"%global_vars.idCounter
  
### returns the list of the variables contained in a string
def get_variable_list(string):
  l = re.compile('~\[([0-9]*)\]').findall(string)
  for i in xrange(len(l)):
    l[i] = "~[%s]"%l[i]
  return l

### substitutes the values of the variables found in the string with their value name
def replace_children_with_vars(string, mapping, string_map):
 for x in mapping:
    x[0] = recursively_substitute_vars_in(x[0], string_map)

 for x in mapping:
    string = string.replace(x[0], x[1], 1)
 return string

### returns a string with all its variable instantiated recursively
def recursively_substitute_vars_in(string, string_map):
  if not "~" in string:
    return string
  else:
    l = re.compile('~\[([0-9]*)\]').findall(string)
    for i in xrange(len(l)):
      l[i] = "~[%s]"%l[i]
      string = string.replace(l[i], recursively_substitute_vars_in(string_map[l[i]], string_map), 1)
  return string
  
#######################################################################################
### Plan Creation Support Functions

### returns a plan which is the concatenation of the two input plans
def concatenate_action_trees(tree, second_tree):
  renormalize_tree_ids(second_tree, tree)
  newTree       = tree
  goalPlace     = getGoal(newTree)
  initialPlace  = getInitialPlace(second_tree)
  
  setName(goalPlace, goalPlace.get("id").title())
  
  for element in second_tree.getroot().find("net"):
    if element != initialPlace:
      newTree.getroot().find("net").append(element)
  
  for arc in newTree.find("net").iter("arc"):
    if arc.get("source") == initialPlace.get("id"):
      arc.set("source", goalPlace.get("id"))
    elif arc.get("target") == initialPlace.get("id"):
      arc.set("target", goalPlace.get("id"))
  
  return newTree
  
def condition_action_trees(condition, first_tree, second_tree):
  condition_tree        = getTreeFromFile("Conditional.pnml")
  instantiation         = create_instantiated_transition_tree("[inst<@Condition,%s>]"%(condition))
  renormalize_tree_ids(instantiation, condition_tree)
  condition_tree        = concatenate_action_trees(instantiation, condition_tree)
  renormalize_tree_ids(first_tree, condition_tree)
  renormalize_tree_ids(second_tree, first_tree)
  
  firstGoalPlace                        = getGoal(first_tree)
  firstInitialPlace                     = getInitialPlace(first_tree)
  secondGoalPlace                       = getGoal(second_tree)
  secondInitialPlace                    = getInitialPlace(second_tree)
  firstLeaf, secondLeaf                 = getAllLeaves(condition_tree)
  
  for element in first_tree.getroot().find("net"):
    if element != firstInitialPlace:
      condition_tree.getroot().find("net").append(element)
  
  for element in second_tree.getroot().find("net"):
    if element != secondInitialPlace and element != secondGoalPlace:
      condition_tree.getroot().find("net").append(element)
  
  for arc in condition_tree.find("net").iter("arc"):
    if arc.get("source") == firstInitialPlace.get("id"):
      arc.set("source", firstLeaf.get("id"))
    elif arc.get("source") == secondInitialPlace.get("id"):
      arc.set("source", secondLeaf.get("id"))
    
    elif arc.get("target") == secondGoalPlace.get("id"):
      arc.set("target", firstGoalPlace.get("id"))
  
  return condition_tree

def condition_and_loop_action_tree(condition, tree):
  condition_loop_tree   = getTreeFromFile("ConditionalLoop.pnml")  
  print "got tree conditional loop"
  instantiation         = create_instantiated_transition_tree("[inst<@Condition,%s>]"%(condition))
  renormalize_tree_ids(instantiation, tree)
  print "creted instantiation transition and renormalized it"
  tree                  = concatenate_action_trees(tree, instantiation)
  print "concatenated with tre"
  renormalize_tree_ids(tree, condition_loop_tree)
  
  treeGoalPlace                 = getGoal(tree)
  treeInitialPlace              = getInitialPlace(tree)
  conditionLoopInitialPlace     = getInitialPlace(condition_loop_tree)
  setName(treeGoalPlace, getGoal(tree).get("id").title())
  
  for element in tree.getroot().find("net"):
    if element != treeInitialPlace:
      condition_loop_tree.getroot().find("net").append(element)
  
  for arc in condition_loop_tree.find("net").iter("arc"):
    if arc.get("source") == conditionLoopInitialPlace.get("id"):
      arc.set("source", treeGoalPlace.get("id"))
      
  for arc in condition_loop_tree.find("net").iter("arc"):
    if arc.get("source") == treeInitialPlace.get("id"):
      arc.set("source", conditionLoopInitialPlace.get("id"))
  
  return condition_loop_tree

### returns a plan composed of a single transition called with the input transition name
def create_instantiated_transition_tree(tran_name):
  transition = getTreeFromFile("SimpleTransition.pnml")
  for tran in transition.find("net").iter("transition"):
    setName(tran, tran_name)
    break
  return transition

### returns a plan composed of a single action that calls a newly saved plan with the variables instantiated 
### (e.g. input: (turn, [right]) output: singleActionPlan that points to a newly saved TurnRight.pnml)
def create_single_action_tree(action_name, action_params, action_output = "", saving_in_SapienzBot = False):
  os.chdir(global_vars.plan_directory)
  planTree      = getTreeFromFile("SimplePlan.pnml")
  initPlace     = getInitialPlace(planTree)
  finalPlace    = getGoal(planTree)
  for place in planTree.find("net").iter("place"):
    if not (place == initPlace or place == finalPlace):
      placeToBeModified = place
      break
  
  plan_action_name = getPlanActionName(action_name, "") + ".exec"
  setName(placeToBeModified, plan_action_name)
  setName(finalPlace, "goal")
  
  if not saving_in_SapienzBot:
    if action_output != "":
      action_output             = action_output.replace("@","")
      output_name               = getOutputName(action_name[0].lower() + action_name[1:])[0]
      
      assert output_name != "", 'Could not retrieve output name for action %s'%(action_name[0].lower() + action_name[1:])
      
      instantiating_transition  = create_instantiated_transition_tree("[inst<@%s,@%s>]"%(action_output[0].upper() + action_output[1:], output_name))
      planTree                  = concatenate_action_trees(planTree, instantiating_transition)
    
    param_names = getParamNames(action_name[0].lower() + action_name[1:])
    param_names = [x for x in param_names if x != ""]
    
    assert len(action_params) == len(param_names), 'len(action_params) %s != len(param_names) %s for action %s'%(action_params, param_names, action_name)
    
    for i in xrange(len(param_names)):
      if action_params[i][0] == "@":                            ### Forcing all paramenters to be capitalized
        action_params[i] = "@" + action_params[i][1].upper() + action_params[i][2:]
        
      instantiating_transition = create_instantiated_transition_tree("[inst<@%s,%s>]"%(param_names[i].title(),action_params[i]))
      planTree = concatenate_action_trees(instantiating_transition, planTree)
  
  return planTree
 
### returns a plan which is the PNP fork of the two input plans
def fork_action_trees(first_tree, second_tree):
  fork_tree             = getTreeFromFile("Fork.pnml")
  second_fork_tree      = getTreeFromFile("Fork.pnml")
  
  renormalize_tree_ids(first_tree, fork_tree)
  renormalize_tree_ids(second_tree, first_tree)
  renormalize_tree_ids(second_fork_tree, second_tree)
  
  firstGoalPlace                        = getGoal(first_tree)
  firstInitialPlace                     = getInitialPlace(first_tree)
  secondGoalPlace                       = getGoal(second_tree)
  secondInitialPlace                    = getInitialPlace(second_tree)
  firstLeaf, secondLeaf                 = getAllLeaves(fork_tree)
  endingFirstLeaf, endingSecondLeaf     = getAllLeaves(second_fork_tree)
  newGoal                               = getInitialPlace(second_fork_tree)
  
  setName(newGoal, "goal")
  setTokens(newGoal, 0)

  for element in second_fork_tree.getroot().find("net"):
    fork_tree.getroot().find("net").append(element)
  
  for element in first_tree.getroot().find("net"):
    if element != firstInitialPlace and element != firstGoalPlace:
      fork_tree.getroot().find("net").append(element)
  
  for element in second_tree.getroot().find("net"):
    if element != secondInitialPlace and element != secondGoalPlace:
      fork_tree.getroot().find("net").append(element)
  
  for arc in fork_tree.find("net").iter("arc"):
    if arc.get("source") == newGoal.get("id") or arc.get("target") == endingSecondLeaf.get("id") or arc.get("target") == endingFirstLeaf.get("id"):
      invert_arc(arc)
  
    if arc.get("source") == firstInitialPlace.get("id"):
      arc.set("source", firstLeaf.get("id"))
    elif arc.get("source") == secondInitialPlace.get("id"):
      arc.set("source", secondLeaf.get("id"))
    
    if arc.get("target") == firstGoalPlace.get("id"):
      arc.set("target", endingFirstLeaf.get("id"))
    elif arc.get("target") == secondGoalPlace.get("id"):
      arc.set("target", endingSecondLeaf.get("id"))
  
  return fork_tree

### graphycally tries to displace the pnp plan in a nicer way
def optimize(tree):
  global_vars.G         = nx.DiGraph() 
  x_max                 = 0
  y_max                 = 0
  x_factor              = 1
  y_factor              = 1
  
  for arc in tree.find("net").iter("arc"):
    global_vars.G.add_edge("%s"%arc.get("source"),"%s"%arc.get("target"))
    if arc[0].tag == "graphics":
      arc[0].getparent().remove(arc[0])

  pos = nx.graphviz_layout(global_vars.G)
  
  for place in tree.getroot().find("net").iter("place"):
    position = getPosition(place)
    
    if position[0] > x_max:
      x_max = position[0]
    if position[1] > y_max:
      y_max = position[1]
  
  if x_max > global_vars.MAX_PNP_PLAN_X:
    x_factor = global_vars.MAX_PNP_PLAN_X/x_max
    
  if y_max > global_vars.MAX_PNP_PLAN_Y:
    y_factor = global_vars.MAX_PNP_PLAN_Y/y_max
    
  for place in tree.getroot().find("net").iter("place"):
    place.find("graphics").find("position").set("x",str(int(int(pos[place.get("id")][0])*x_factor)))
    place.find("graphics").find("position").set("y",str(int(int(pos[place.get("id")][1])*y_factor)))
    
  for transition in tree.getroot().find("net").iter("transition"):
    transition.find("graphics").find("position").set("x",str(int(int(pos[transition.get("id")][0])*x_factor)))
    transition.find("graphics").find("position").set("y",str(int(int(pos[transition.get("id")][1])*y_factor)))

### returns a plan which merges the two input trees by combining their initial and goal places
def parallelize_action_trees(first_tree, second_tree): 
  second_tree_copy      = second_tree
  newTree               = first_tree
  
  renormalize_tree_ids(second_tree_copy, newTree)
  
  firstGoalPlace        = getGoal(newTree)
  firstInitialPlace     = getInitialPlace(newTree)
  secondGoalPlace       = getGoal(second_tree_copy)
  secondInitialPlace    = getInitialPlace(second_tree_copy)
  
  for element in second_tree_copy.getroot().find("net"):
    if element != secondInitialPlace and element != secondGoalPlace:
      newTree.getroot().find("net").append(element)
  
  for arc in newTree.find("net").iter("arc"):
    if arc.get("source") == secondInitialPlace.get("id"):
      arc.set("source", firstInitialPlace.get("id"))
    elif arc.get("target") == secondGoalPlace.get("id"):
      arc.set("target", firstGoalPlace.get("id"))
  
  return newTree
  
### changes the ids of the newNet with the following rules: 
### placeId = oldPlaceId + maxPlaceId gotten from the input net 
### transId = oldTransId + maxTransId gotten from the input net 
def renormalize_tree_ids(newNet, net):
  maxPlaceId    = 1
  maxTranId     = 1
  maxArcId      = 1
  
  for place in net.find("net").iter("place"):
    if int(place.get("id")[1:]) > maxPlaceId:
      maxPlaceId = int(place.get("id")[1:])

  for transition in net.find("net").iter("transition"):
    if int(transition.get("id")[1:]) > maxTranId:
      maxTranId = int(transition.get("id")[1:])
      
  for arc in net.find("net").iter("arc"):
    if int(arc.get("id")[1:]) > maxArcId:
      maxArcId = int(arc.get("id")[1:])

  for place in newNet.find("net").iter("place"):      
    if place[1][0].text == "P%s"%place.get("id")[1:]:
      place[1][0].text = "P%s"%(int(place.get("id")[1:])+maxPlaceId)
    place.set("id","p%s"%(int(place.get("id")[1:])+maxPlaceId))

  for transition in newNet.find("net").iter("transition"):
    if transition[1][0].text == "T%s"%transition.get("id")[1:]:
      transition[1][0].text = "T%s"%(int(transition.get("id")[1:])+maxTranId)
    transition.set("id","t%s"%(int(transition.get("id")[1:])+maxTranId))
  
  for arc in newNet.find("net").iter("arc"):
    arc.set("id", "a%s"%(int(arc.get("id")[1:])+maxArcId))
    
    if arc.get("source")[0] == "p":
      arc.set("source","p%s"%(int(arc.get("source")[1:])+maxPlaceId))
    else:
      arc.set("source","t%s"%(int(arc.get("source")[1:])+maxTranId))

    if arc.get("target")[0] == "p":
      arc.set("target","p%s"%(int(arc.get("target")[1:])+maxPlaceId))
    else:
      arc.set("target","t%s"%(int(arc.get("target")[1:])+maxTranId))
    
  return newNet  
  
### saves the combinedPlan name in the kb
def save_combinedPlan_name_in_kb(kbPlanName, paramsList, pcl):
  paramString   = ""
  for i in paramsList:
    if i != "":
      paramString += i[0].lower() + i[1:] + ","
 
  assertion     = "combinedPlan(%s)"%kbPlanName
  assertion2    = "plan(%s,[%s],[])"%(kbPlanName, paramString[:-1])
  assertion3    = "pcl(%s,\'%s\')"%(kbPlanName, pcl)
  
  global_vars.prolog_assert_handle(assertion)
  global_vars.prolog_assert_handle(assertion2)
  global_vars.prolog_assert_handle(assertion3)

  os.chdir(global_vars.prolog_directory)
  f = open("plans.pl", 'a')
  f.write(assertion + ".\n")
  f.write(assertion2 + ".\n")
  f.write(assertion3 + ".\n")
  f.close() 
    
    
### saves a graphically optimized version of the input tree in the file plan_directory/fileName.pnml
def save_optimized_plan(tree, name):
  optimize(tree)
  save_plan(tree, name)
  
### saves a tree in the file plan_directory/fileName.pnml
def save_plan(tree, name):
  tree_to_be_saved = getTreeFromFile("InitialPlan.pnml")
  tree_to_be_saved.getroot().find("net").remove(getInitialPlace(tree_to_be_saved))
  
  for place in tree.find("net").iter("place"):
    tree_to_be_saved.getroot().find("net").append(place)

  for transition in tree.find("net").iter("transition"):
    tree_to_be_saved.getroot().find("net").append(transition)

  for arc in tree.find("net").iter("arc"):
    tree_to_be_saved.getroot().find("net").append(arc)

  if not name.endswith(".pnml"):
    name = name + ".pnml"
  name = name[0].title() + name[1:]
  
  os.chdir(global_vars.plan_directory)
  f     = open(name,'w+')
  f.write(etree.tostring(tree_to_be_saved, encoding="iso-8859-1"))
  f.close()
  global_vars.log.publish("SAVEPLAN:%s"%name)
  
#######################################################################################
### Forget Support Functions

### deletes the file found at the input file_path from the hard disk
def remove_file(file_path):
  try:
    os.remove(file_path)
  except:
    rospy.logwarn("Can not find and therefore delete the following file: %s"%file_path)

### removes all the lines in the prolog plan file plan.pl that contain the input substring_to_be_deleted
def remove_from_plan_prolog_file(substring_to_be_deleted):
  os.chdir(global_vars.prolog_directory)
  content = open("plans.pl", 'r').readlines()
  f = open("plans.pl", 'w')
  for line in content:
    if not (substring_to_be_deleted in line):
      f.write(line)
  f.close()
  
### removes all ancestors and descendents of all the places called as the input element name from the input plan name
def remove_all_relatives_of_all_elements_named_from(element_name, plan_name_to_be_modified):
  tree_to_be_modified   = getTreeFromFile(plan_name_to_be_modified)
  net                   = tree_to_be_modified.getroot().find("net")
  elementsToBeDeleted   = []
  execPlace             = None
  
  for execPlace in tree_to_be_modified.find("net").iter("place"):
    if (getName(execPlace) == element_name):
      elementsToBeDeleted = elementsToBeDeleted + getAllElementsAttachedTo(execPlace, tree_to_be_modified)
    
  for element in elementsToBeDeleted:
    if element.tag == "arc":
      net.remove(element)
      
    elif element.tag == "place" and not (isInitial(element) or isGoal(element)):
     net.remove(element)
     
    elif element.tag == "transition":
      net.remove(element)

  save_plan(tree_to_be_modified, plan_name_to_be_modified)
  
### reset the prolog file plan.pl to the state where no new actions were learnt
def reset_plan_prolog_file(actions_learnt_list):
  if len(actions_learnt_list) != 0:
    os.chdir(global_vars.prolog_directory)
    content             = open("plans.pl", 'r').readlines()
    f                   = open("plans.pl", 'w')

    os.chdir(global_vars.prolog_directory)

    for element in actions_learnt_list:
      content = [x for x in content if not element in x]
    
    for line in content:
      f.write(line)
    
    f.close()
  
#######################################################################################
### Getters

### returns a the action parameters and output names queried to prolog using an input string (e.g. say:[something,stupid])
def getActionParamsAndOutput(string):
    if "@" in string and "=" in string:
      output, rest        = string.split("=")
      name, params        = rest.split(":")
    
    else:
      output        = ""
      name, params  = string.split(":")
  
    if params == "[]" or params == ['']:
      params              = []
    else:
      params              = ''.join(x for x in params if x not in '[]')
      params              = params.split(",")
    
    print params
    print "returning: %s"%[name, params, output]
    return [name, params, output]
  
### returns all the elements (places, transitions or arcs) connected with the input element (place or transition) in the input tree
def getAllElementsAttachedTo(element, tree):
  el_id         = element.get('id')
  parents       = getAncestorsIds(el_id, tree)
  children      = getDescendentsIds(el_id, tree)
  ids           = [el_id] + parents + children
  answer        = []

  for arc in tree.find("net").iter("arc"):
    if arc.get("source") in ids and arc.get("target") in ids:
      answer.append(arc)
  
  for p in tree.find("net").iter("place"):
    if p.get("id") in ids:
      answer.append(p)
  
  for tran in tree.find("net").iter("transition"):
    if tran.get("id") in ids:
      answer.append(tran)
  
  return list(set(answer))
  
### returns all the leaf places found in the net
def getAllLeaves(tree):
  leaves        = []
  for place in tree.find("net").iter("place"):
    if isLeafInTree(place, tree):
      leaves.append(place)
  return leaves 

### returns a list of all the ancestors of a given element Id
def getAncestorsIds(el_id, tree):
  
  if isIdRootInTree(el_id, tree):
    return [el_id]
  else:
    answer = [el_id]
    
    for arc in tree.find("net").iter("arc"):
      if arc.get("target") == el_id:
        answer = answer + getAncestorsIds(arc.get("source"), tree)
    return answer
    
### returns a list of all the ancestors of a given element Id
def getDescendentsIds(el_id, tree):
  
  if isIdLeafInTree(el_id, tree):
    return [el_id]
  else:
    answer = [el_id]
    
    for arc in tree.find("net").iter("arc"):
      if arc.get("source") == el_id:
        answer = answer + getDescendentsIds(arc.get("target"), tree)
    
    return answer
  
### returns the input condition in the formatted form used in combine-pnp
def getFormattedCondition(string):
  return string.replace("(","").replace(")","").replace(":[","{").replace("]","}").replace(" ","")
  
### returns the goal place found in the net
def getGoal(net):
  leaves = []
  for place in net.find("net").iter("place"):
    if getName(place) == "goal":
      leaves.append(place)
  if len(leaves) > 1:
    rospy.logwarn("Found multiple goals for the current plan, choosing the first one of the list")
  return leaves[0]
  
### returns the initial place found in the net
def getInitialPlace(net):
  initialPlaces = []
  for place in net.find("net").iter("place"):
    if isInitial(place):
      initialPlaces.append(place)
  if len(initialPlaces) != 1:
    rospy.logwarn("Found multiple possible initial places of the plan to be added, choosing the first one of the list")
  return initialPlaces[0]
  
### returns the name of the plan in the kb given a certain string (e.g. say_I am-going.somewhere -> sayIAm-goingSomewhere)
def getKbPlanName(action, action_params):
  answer = getPlanActionName(action, action_params)
  return answer[0].lower() + answer[1:]
  
### returns the name of the input element
def getName(element):
  return element.find("name")[0].text

### returns a list of parameters for the input action
def getParamNames(action):
  print "looking for parameters of plan %s"%action
  kbPlanName = action
  
  try:
    query_results = global_vars.prolog_handle ('plan', ["%s"%kbPlanName, 'X', 'Y']).ris

    if len(query_results) == 1:
      paramList = query_results[0].atoms[1].replace("[","").replace("]","").replace(" ","").split(",")
      for i in xrange(len(paramList)):
        paramList[i] = paramList[i].title()
      
      paramList = [value for value in paramList if value != ""]
      print "parameters retrieved for plan %s: %s"%(kbPlanName, paramList)
      return paramList
      
    else:
      return []

  except:
    rospy.logerr("Could not retrieve details for plan named %s"%(kbPlanName))
    return []

### returns the name of the plan in the kb given a certain string (e.g. say_I am-going.somewhere -> sayIAm-goingSomewhere)
def getPlanActionName(action, paramList):
  action = action[0].title() + action[1:]
  for par in paramList:
    action = action + (''.join(x for x in par if x not in "[]'\" .?!#@()" )).title()
  return action
  
### returns the graphical display position of an element
def getPosition(element):
  return [float(element.find("graphics").find("position").get("x")), float(element.find("graphics").find("position").get("y"))]
  
### returns the name of the output of the input action
def getOutputName(action):
  kbPlanName = action
  try:
    query_results = global_vars.prolog_handle ('plan', ["%s"%kbPlanName, 'X', 'Y']).ris

    if len(query_results) == 1:
      paramList = query_results[0].atoms[2].replace("[","").replace("]","").replace(" ","").split(",")
      for i in xrange(len(paramList)):
        paramList[i] = paramList[i][0].upper() + paramList[i][1:] 
      return paramList
      
    else:
      return []
  
  except:
    rospy.logerr("Could not retrieve details for plan named %s"%(kbPlanName))
    return []
  
### loads a PNP plan tree from an input fileName 
def getTreeFromFile(fileName):
  if not fileName.endswith(".pnml"):
    fileName = fileName + ".pnml"
  
  os.chdir(global_vars.plan_directory)
  return etree.parse(fileName)
  
#######################################################################################
### Checkers  

### checks if the place is an initial PNP place or not
def isInitial(place):
  if int(place.find("initialMarking")[0].text) != 0:
    return True
  return False
  
### checks if the place is a leaf PNP place (i.e. goal) or not
def isGoal(place):
  if getName(place) == "goal":
    return True
  return False
  
### checks if the element is a leaf  of the PNP tree or not
def isLeafInTree(place, tree):
  for arc in tree.find("net").iter("arc"):
    if arc.get("source") == place.get("id"):
      return False
  return True
  
### checks if the element corresponding to the id is a leaf of the PNP tree or not
def isIdLeafInTree(el_id, tree):
  for arc in tree.find("net").iter("arc"):
    if arc.get("source") == el_id:
      return False
  return True

### checks if the element is a root of the PNP tree or not
def isRootInTree(place, tree):
  for arc in tree.find("net").iter("arc"):
    if arc.get("target") == place.get("id"):
      return False
  return True
  
### checks if the element corresponding to the id is a root of the PNP tree or not
def isIdRootInTree(el_id, tree):
  for arc in tree.find("net").iter("arc"):
    if arc.get("target") == el_id:
      return False
  return True

#######################################################################################
### Setters  
  
### inverts the target and the source of the input arc
def invert_arc(arc):
  target = arc.get("target")
  source = arc.get("source")
  arc.set("target",source)
  arc.set("source",target)
  
### sets the name of the input element to the input name
def setName(element, name):
  element.find("name")[0].text = name
  
### sets the number of tokens in the input element to the input number
def setTokens(element, n_tokens):
  element.find("initialMarking")[0].text = str(n_tokens)
 
#######################################################################################
### Debugging Functions

### analyzes the input tree and tries to find some common errors in it
def analyze_tree(tree):
  placeList     = []
  tranList      = []
  arcList       = []
  sourced       = set()
  targeted      = set()
  
  for element in tree.find("net").iter("place"):
    elid = element.get("id") 
    if elid in sourced or elid in targeted:
      rospy.logerr("Found two different elements with the same id %s"%elid)
    placeList.append(element)
    sourced.add(elid)
    targeted.add(elid)

  for element in tree.find("net").iter("transition"):
    elid = element.get("id") 
    if elid in sourced or elid in targeted:
      rospy.logerr("Found two different elements with the same id %s"%elid)
    tranList.append(element)
    sourced.add(elid)
    targeted.add(elid)
  
  elementIds = sourced.union(targeted)
  
  for arc in tree.find("net").iter("arc"):
    if arc.get("id") in arcList:
      rospy.logerr("Found two different elements with the same id %s"%elid)
    else:
      arcList.append(arc.get("id"))
   
    s = arc.get("source")
    t = arc.get("target")
    
    if not s in elementIds:
      rospy.logerr("arc %s sources a not existant element with id %s"%(arc.get("id"), s))
    if not t in elementIds:
      rospy.logerr("arc %s targets a not existant element with id %s"%(arc.get("id"), t))
    
    if s in sourced:
      sourced.remove(s)
    if t in targeted:
      targeted.remove(t)
  
  rospy.loginfo("The following elements are not sourced in the tree: ")
  for i in sourced:
    rospy.loginfo("%s "%i)
    
  rospy.loginfo("The following elements are not targeted in the tree: ")
  for i in targeted:
    rospy.loginfo("%s "%i)
 
### Displays the tree as a directed graph like structure image
def show_tree(tree):
  for place in tree.find("net").iter("place"):
    global_vars.G.add_node(place.get("id"))
  
  for tran in tree.find("net").iter("transition"):
    global_vars.G.add_node(tran.get("id"))

  for arc in tree.find("net").iter("arc"):
    global_vars.G.add_edge(arc.get("source"), arc.get("target"), weight=0.4)
  
  elarge        = [(u,v) for (u,v,d) in global_vars.G.edges(data=True) if d['weight'] >0.5]
  esmall        = [(u,v) for (u,v,d) in global_vars.G.edges(data=True) if d['weight'] <=0.5]

  pos           = nx.spectral_layout(global_vars.G) # positions for all nodes

  nx.draw_networkx_nodes(global_vars.G,pos,alpha=0.5,node_shape='o',node_size=70)
  nx.draw_networkx_edges(global_vars.G,pos,edgelist=elarge,width=3)
  nx.draw_networkx_edges(global_vars.G,pos,edgelist=esmall,width=3,alpha=0.5,edge_color='b',style='dashed')

  nx.draw_networkx_labels(global_vars.G,pos,font_size=14, font_color='k',font_family='sans-serif')

  plt.axis('off')
  plt.show()
  print "done"
  
#########################################################################
### Others

def find_nth_substring_start_index(haystack, needle, n):
    start = haystack.find(needle)
    while start >= 0 and n > 1:
        start = haystack.find(needle, start+1)
        n -= 1
    return start
    
def normalize_pcl(pcl):
  pcl = pcl. replace("( A )","")
  for i in re.findall('#(.+?)@',pcl):
    pcl = pcl.replace("#"+i, "")
  
  pcl = " ".join(pcl.split()) # removes multiple subsequent spaces in pcl
  pcl = pcl.rstrip().lstrip() # removes beginning and ending spaces
  
  if pcl[0] != "(":
    pcl = "( " + pcl
  if pcl[-1] != ")":
    pcl = pcl + " )"
  
  #actions = re.findall('\( [\w]+\:\[[\w]*\] \)', pcl)
  #actions2 = re.findall('\( [\w]+\:\[[\w]+\] \)', pcl)
  #print actions
  #print actions2
  #if len(actions) > 1:
      #s         = actions[0]
      #s1        = "do-sequentially " + actions[0]
      #for i in xrange(len(actions)-1):
        #s  += " " + actions[i+1]
        #s1 += " " + actions[i+1]
        
      #print s
      #print s1
      
      #if s in pcl and not s1 in pcl:
        #pcl = pcl.replace(s, " ( do-sequentially " + s + " )")
  
  return pcl