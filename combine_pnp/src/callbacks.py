import os
import rospy
import global_vars
import re

from lxml import etree 
from support_functions import *
from support_functions import *

#######################################################################################
### TestPlanSrv

def test_plan_cb(req):
  instantiation         = req.name
  pcl                   = normalize_pcl(req.pcl)

  i                     = pcl.rfind("(")
  j                     = pcl[i:].find(")")
  elementary_pcl        = pcl[i:i+j]
  
  try:
    variable    = elementary_pcl.split("@")[1].replace(" ","")
  except:
    variable    = ""
  
  if (instantiation == "") and (variable in global_vars.pcl_var_instances_map):
    elementary_pcl = elementary_pcl.split("@")[0] + "%s] )"%global_vars.pcl_var_instances_map[variable]
    
    tree          = getTreeFromFile('InitialPlan.pnml')
    string_map    = {}
    root          = parse(elementary_pcl.replace(" ",""), string_map)[0]
    tree          = create_plan(root, string_map)
    
    save_optimized_plan(tree, "TestingPCL")
    global_vars.event_pub.publish("ExecutePCL_%s"%pcl)
    
  elif instantiation != "":
    assert (":[@" in elementary_pcl), "Passed elementary %s pcl with no parameters and an instance %s"%(elementary_pcl,instantiation)
    
    global_vars.pcl_var_instances_map[variable]         = instantiation
    elementary_pcl                                      = elementary_pcl.split("@")[0] + "%s] )"%instantiation
    tree                                                = getTreeFromFile('InitialPlan.pnml')
    string_map                                          = {}
    root                                                = parse(elementary_pcl.replace(" ",""), string_map)[0]
    tree                                                = create_plan(root, string_map)
    
    save_optimized_plan(tree, "TestingPCL")
    global_vars.event_pub.publish("ExecutePCL_%s"%pcl)
    
  else:
    if (":[@" in elementary_pcl):
      rospy.logwarn("Could not test the %s PCL due to missing value for its variable %s"%(pcl,variable))
    global_vars.event_pub.publish("NotAbleToExecute_%s"%pcl)
  
  return "Done"
  
#######################################################################################
### GeneratePlanSrv

#########################
### Callback Function

def generate_plan_cb(req):
  generate_plan(req.name, req.pcl)
  global_vars.send_server_message.publish("[ACTION_LEARNT]")
  global_vars.event_pub.publish("Done_I am done learning the action")
  return 'Done'

def generate_plan(name, pcl):
  rospy.loginfo("received PCL %s for plan %s"%(pcl, name))
  
  kbPlanName                            = name[0].lower() + name[1:]
  pcl                                   = normalize_pcl(str(pcl))
  
  query_results                         = global_vars.prolog_handle('combinedPlan', [kbPlanName])

  if len(query_results.ris) != 0:
    rospy.logwarn("Plan name %s already used for another plan learnt previously. Not learning this new plan."%kbPlanName)

  else:
    tree          = getTreeFromFile('InitialPlan.pnml')
    string_map    = {}
    root          = parse(pcl.replace(" ",""), string_map)[0]
    tree          = create_plan(root, string_map)
    save_plan_in_SapienzBot(tree, name, pcl)

  global_vars.log.publish("Generated pnp for plan %s from pcl %s"%(name,pcl))
  return

#########################
### Functions Called

### returns the root of the parsed pcl, modifying the string_map of the variables
def parse(string, string_map):
  op_index              = -1
  cp_index              = -1
  index                 = 0
  toBeSubstituted       = []
  
  while True:
    if string[index] == ")":
      assert op_index == 0 and cp_index <0, "wrongly formatted string!"
      sId               = getId()
      sub_string        = replace_children_with_vars(string[op_index:index+1], toBeSubstituted, string_map) 
      string_map[sId]   = sub_string
      return [sub_string, sId]
      
    if string[index] == "(":
      if op_index < 0:
        op_index = index
     
      else:
        x                       = parse(string[index:], string_map)
        exploded_substring      = recursively_substitute_vars_in(x[0], string_map)
        index                   = index + len(exploded_substring) - 1
        toBeSubstituted.append(x)
    
    index = index + 1

### creates the PNP plan from the root and the string_map of the variables
def create_plan(root, string_map):
  root = ''.join(x for x in root if x not in '()' )
  tree = None
  print root
  
  if ":[" in root:
    action, action_params, action_output = getActionParamsAndOutput(root)
    for i in xrange(len(action_params)):
      if "#" in action_params[i]:
        action_params[i] = action_params[i].replace("#"+re.findall('#(.+?)@',action_params[i])[0],"").replace(")","")
      
      if "@" in action_params[i]:
        action_params[i] = "@" + action_params[i][1].title() + action_params[i][2:]
      
    tree = create_single_action_tree(action, action_params, action_output)

  elif root.startswith("while"):
    var_list    = get_variable_list(root)
    subtree1    = create_plan(string_map[var_list[0]], string_map)
    subtree2    = create_plan(string_map[var_list[1]], string_map)
    tree        = fork_action_trees(subtree1, subtree2)
  
  elif root.startswith("do-sequentially"):
    var_list    = get_variable_list(root)
    tree        = create_plan(string_map[var_list.pop(0)], string_map)
    
    for i in var_list:
      tree_to_be_concatenated   = create_plan(string_map[i], string_map)
      tree                      = concatenate_action_trees(tree, tree_to_be_concatenated)

  elif root.startswith("do-n-times"):    
    var_list    = get_variable_list(root)
    repetitions = int(root.split("]")[1])
    tree        = create_plan(string_map[var_list[0]], string_map)
    
    for i in range(repetitions - 1):
      tree_to_be_concatenated   = create_plan(string_map[var_list[0]], string_map)
      tree                      = concatenate_action_trees(tree, tree_to_be_concatenated)

  elif root.startswith("if-condition"):
    var_list            = get_variable_list(root)
    condition           = getFormattedCondition(string_map[var_list[0]])
    first_branch_tree   = create_plan(string_map[var_list[1]], string_map)
    second_branch_tree  = create_plan(string_map[var_list[2]], string_map)
    tree                = condition_action_trees(condition, first_branch_tree, second_branch_tree)
  
  elif root.startswith("do") and "until" in root:
    var_list            = get_variable_list(root)
    condition           = getFormattedCondition(string_map[var_list[1]])
    tree                = create_plan(string_map[var_list[0]], string_map)
    tree                = condition_and_loop_action_tree(condition, tree)
  
  else:
    rospy.logerr("unknown command %s"%root)
    tree = etree.parse('InitialPlan.pnml')
    
  return tree
  
### saves the generated plan in a standalone file and in parallel to ChooseAction.pnml
def save_plan_in_SapienzBot(generated_plan, name, pcl):
  kbPlanName    = name[0].lower() + name[1:]
  planName      = name[0].upper() + name[1:]
  guardName     = planName

  for i in global_vars.currentPlanParamList:
    if i != '':
      guardName += "_@" + i[0].upper() + i[1:]

  save_combinedPlan_name_in_kb(kbPlanName, global_vars.currentPlanParamList, pcl)
  save_optimized_plan(generated_plan, planName)
  
  loaded_tree                   = getTreeFromFile("ChooseAction.pnml")
  guard                         = create_instantiated_transition_tree("[%s]"%guardName)
  plan                          = create_single_action_tree(name, [], "", True) 
  plan                          = concatenate_action_trees(guard, plan)
  new_choose_action_plan        = parallelize_action_trees(loaded_tree, plan)
  
  save_plan(new_choose_action_plan, "ChooseAction.pnml")
  rospy.loginfo("Plan saved as %s"%planName)
  
#######################################################################################
### ForgetSrv

### deletes a learnt action from the system
def forget(req):
  answer = forget_plan(req.plan)
  
  if answer == "Done":
    global_vars.send_server_message.publish("[ACTION_DELETED] %s"%req.plan)

  rospy.loginfo("Plan %s forgot"%req.plan)
  global_vars.log.publish("Forgot plan %s"%(req.plan))

  return answer
  
def forget_plan(plan):
  try:
    query_results = global_vars.prolog_handle ('combinedPlan', [plan]).ris
    
    if len(query_results) != 0:
      kbPlanName        = plan[0].lower() + plan[1:]
      planName          = kbPlanName[0].upper() + kbPlanName[1:]
      retract           = 'combinedPlan(%s)'%kbPlanName
      retract2          = 'plan(%s,X,Y)'%kbPlanName
      retract3          = 'pcl(%s,X)'%kbPlanName
      
      global_vars.prolog_retractAll_handle(retract)
      global_vars.prolog_retractAll_handle(retract2)
      global_vars.prolog_retractAll_handle(retract3)
      remove_from_plan_prolog_file(kbPlanName)
      remove_all_relatives_of_all_elements_named_from(planName + ".exec", "ChooseAction.pnml")
      remove_file(global_vars.plan_directory + "/" + planName + ".pnml")
        
      return "Done"
    else:
      return "Plan Not Known"
    
  except:
    rospy.logerr("Could not contact prolog node")
    return "False"
  
#######################################################################################
### ResetSrv

### deletes all the learnt actions from the system
def reset(req):
  try:
    actions_learnt_list = []
    query_results       = global_vars.prolog_handle ('combinedPlan', ["X"]).ris
    
    for i in query_results:
      kbPlanName        = i.atoms[0]
      planName          = kbPlanName[0].upper() + kbPlanName[1:]
      
      actions_learnt_list.append(kbPlanName)
      global_vars.prolog_retract_handle('plan(%s,X,Y)'%kbPlanName)
      global_vars.prolog_retract_handle('pcl(%s,X)'%kbPlanName)
      remove_file(global_vars.plan_directory + "/" + planName + ".pnml")
      remove_all_relatives_of_all_elements_named_from(planName + ".exec", "ChooseAction.pnml")
      
    global_vars.prolog_retractAll_handle('combinedPlan(X)')
    reset_plan_prolog_file(actions_learnt_list)
    
  except:
    rospy.logerr("Could not contact the PrologInterface node")
    
  rospy.loginfo("Combine_pnp resetted")
  return "Done"
  
#######################################################################################
### UpdateActionSrv

### deletes a particular action from the input plan
### used to process the commands like UPDATE_ACTION(planName) + REMOVE_ACTION(3-goTo:[something]) 
def remove_action_from_plan(req):
  rospy.loginfo("Remove action from plan request received")
  
  kbPlanName    = req.planName[0].lower() + req.planName[1:]
  pcl           = global_vars.prolog_handle('pcl', [kbPlanName, 'X']).ris[0].atoms[1].replace("\'","")
  input_pcl     = pcl
  number        = -1
    
  rospy.loginfo("Retrieved pcl %s for plan %s."%(pcl, kbPlanName))
  
  try:
    number      = re.findall("\d+-", req.action_to_be_modified)[0][:-1]
    action_pcl  = normalize_pcl(req.action_to_be_modified.split("-")[1])
  except:
    action_pcl  = normalize_pcl(req.action_to_be_modified)
    rospy.loginfo("I could not find any number in the input action")

  rospy.loginfo("Removing action %s..."%action_pcl)
  
  action_name                   = action_pcl.split(":")[0].replace("(","").replace(" ","")
  n_action_name_occurrencies    = len([i for i in range(len(pcl)) if pcl.startswith(action_name, i)])
  n_action_occurrencies         = len([i for i in range(len(pcl)) if pcl.startswith(action_pcl.replace("(","").replace(" ","").replace(")",""), i)])
  
  if (number != -1) and (int(number) > int(n_action_name_occurrencies)) and (n_action_name_occurrencies == 1):
    global_vars.event_pub.publish("CouldNotGround_There is only one %s action in this plan. Please tell me again which action I should remove."%action_name)
    return "Done"
  elif (number != -1) and (int(number) > int(n_action_name_occurrencies)):
    global_vars.event_pub.publish("CouldNotGround_There are only %s %s actions in this plan. Please tell me again which action I should remove."%(n_action_name_occurrencies,action_name))
    return "Done"
  elif (number == -1) and (("[]" in action_pcl and n_action_name_occurrencies > 1) or (not "[]" in action_pcl and n_action_occurrencies > 1)):
    global_vars.event_pub.publish("CouldNotGround_There are multiple %s actions in this plan. Please tell me which one you are referring to."%action_name)
    return "Done"
  elif ("[]" in action_pcl) and (n_action_name_occurrencies < 1):
    global_vars.event_pub.publish("CouldNotGround_There are no %s actions in this plan. Please tell me again which action I should remove."%action_name)
    return "Done"
  elif (not "[]" in action_pcl) and (n_action_occurrencies < 1):
    global_vars.event_pub.publish("CouldNotGround_There are no %s actions of that kind in this plan. Please tell me again which action I should remove."%action_name)
    return "Done"
    
  global_vars.currentPlanParamList = getParamNames(kbPlanName)
  
  if number == -1:
    number = 1

  if "[]" in action_pcl:
    i = find_nth_substring_start_index(pcl, action_name, int(number))

  else:
    i = find_nth_substring_start_index(pcl, action_pcl.replace("(","").replace(" ","").replace(")",""), int(number))
    
  k = pcl[:i].rfind("(")
  j = pcl[i:].find(")")
  
  print action_name
  print n_action_name_occurrencies
  print n_action_occurrencies
  print pcl[:k]
  print pcl[i+j+2:]
  
  pcl = pcl[:k] + pcl[i+j+2:]

  global_vars.log.publish("Removed action %s from %s plan pcl %s obtaining pcl %s "%(action_pcl,req.planName,input_pcl,pcl))
  rospy.loginfo("Processing the new pcl %s"%pcl)
  
  forget_plan(kbPlanName)
  generate_plan(kbPlanName, pcl)
  
  global_vars.event_pub.publish("ActionRemoved_I updated the %s plan"%req.planName)
  
  return "Done"  
  
### replaces a particular action in the input plan with another action
### used to process the commands like UPDATE_ACTION(planName) + REPLACE_ACTION(3-goTo:[something], goTo:[anywhere]) 
def replace_action_in_plan(req):
  rospy.loginfo("Replace action in plan request received")
  
  kbPlanName            = req.planName[0].lower() + req.planName[1:]
  pcl                   = global_vars.prolog_handle('pcl', [kbPlanName, 'X']).ris[0].atoms[1].replace("\'","")
  input_pcl             = pcl
  second_action_pcl     = " " + normalize_pcl(req.action_to_be_added) + " "
  number                = -1
    
  rospy.loginfo("Retrieved pcl %s for plan %s."%(pcl, kbPlanName))
  
  try:
    number              = re.findall("\d+-", req.action_to_be_modified)[0][:-1]
    first_action_pcl    = normalize_pcl(req.action_to_be_modified.split("-")[1])
  except:
    first_action_pcl    = normalize_pcl(req.action_to_be_modified)
    rospy.loginfo("I could not find any number in the input action")
  
  rospy.loginfo("Replacing action %s with action %s..."%(first_action_pcl,second_action_pcl))
  
  action_name                   = first_action_pcl.split(":")[0].replace("(","").replace(" ","")
  n_action_name_occurrencies    = len([i for i in range(len(pcl)) if pcl.startswith(action_name, i)])
  n_action_occurrencies         = len([i for i in range(len(pcl)) if pcl.startswith(first_action_pcl.replace("(","").replace(" ","").replace(")",""), i)])
  
  if (number != -1) and (int(number) > int(n_action_name_occurrencies)) and (n_action_name_occurrencies == 1):
    global_vars.event_pub.publish("CouldNotGround_There is only one %s action in this plan. Please tell me again which action I should replace."%action_name)
    return "Done"
  elif (number != -1) and (int(number) > int(n_action_name_occurrencies)):
    global_vars.event_pub.publish("CouldNotGround_There are only %s %s actions in this plan. Please tell me again which action I should replace."%(n_action_name_occurrencies,action_name))
    return "Done"
  elif (number == -1) and (("[]" in first_action_pcl and n_action_name_occurrencies > 1) or (not "[]" in first_action_pcl and n_action_occurrencies > 1)):
    global_vars.event_pub.publish("CouldNotGround_There are multiple %s actions in this plan. Please tell me which one you are referring to."%action_name)
    return "Done"
  elif ("[]" in first_action_pcl) and (n_action_name_occurrencies < 1):
    global_vars.event_pub.publish("CouldNotGround_There are no %s actions in this plan. Please tell me again which action I should replace."%action_name)
    return "Done"
  elif (not "[]" in first_action_pcl) and (n_action_occurrencies < 1):
    global_vars.event_pub.publish("CouldNotGround_There are no %s actions of that kind in this plan. Please tell me again which action I should replace."%action_name)
    return "Done"
  
  global_vars.currentPlanParamList = getParamNames(kbPlanName)
  
  if number == -1:
    number = 1

  if "[]" in first_action_pcl:
    i = find_nth_substring_start_index(pcl, action_name, int(number))

  else:
    i = find_nth_substring_start_index(pcl, first_action_pcl.replace("(","").replace(" ","").replace(")",""), int(number))
    
  k = pcl[:i].rfind("(")
  j = pcl[i:].find(")")
  
  pcl = pcl[:k] + " " + second_action_pcl + " " + pcl[i+j+2:]
  
  rospy.loginfo("Processing the new pcl %s"%pcl)
  
  forget_plan(kbPlanName)
  generate_plan(kbPlanName, pcl)
  
  global_vars.event_pub.publish("ActionReplaced_I updated the %s plan"%req.planName)
  global_vars.log.publish("Replaced action %s with %s in %s plan pcl %s obtaining pcl %s "%(first_action_pcl, second_action_pcl,req.planName,input_pcl,pcl))

  return "Done"

### inserts a particular action in the input plan after or before another action
### used to process the commands like UPDATE_ACTION(planName) + INSERT_ACTION(after, 3-goTo:[something], goTo:[anywhere]) 
def insert_action_in_plan(req):
  rospy.loginfo("Insert action in plan request received")
  
  kbPlanName            = req.planName[0].lower() + req.planName[1:]
  where                 = req.update_command.lower()
  pcl                   = global_vars.prolog_handle('pcl', [kbPlanName, 'X']).ris[0].atoms[1].replace("\'","")
  input_pcl             = pcl
  second_action_pcl     = " " + normalize_pcl(req.action_to_be_added) + " "
  number                = -1
  
  rospy.loginfo("Retrieved pcl %s for plan %s."%(pcl, kbPlanName))
  
  try:
    number              = re.findall("\d+-", req.action_to_be_modified)[0][:-1]
    first_action_pcl    = normalize_pcl(req.action_to_be_modified.split("-")[1])
  except:
    first_action_pcl    = normalize_pcl(req.action_to_be_modified)
    rospy.loginfo("I could not find any number in the input action")
  
  rospy.loginfo("Inserting the action %s %s the action %s..."%(second_action_pcl, where, first_action_pcl))
  
  action_name                   = first_action_pcl.split(":")[0].replace("(","").replace(" ","")
  n_action_name_occurrencies    = len([i for i in range(len(pcl)) if pcl.startswith(action_name, i)])
  n_action_occurrencies         = len([i for i in range(len(pcl)) if pcl.startswith(first_action_pcl.replace("(","").replace(" ","").replace(")",""), i)])
  
  print first_action_pcl.replace("(","").replace(" ","")
  print number
  print n_action_name_occurrencies
  print n_action_occurrencies
  
  if (number != -1) and (int(number) > int(n_action_name_occurrencies)) and (n_action_name_occurrencies == 1):
    global_vars.event_pub.publish("CouldNotGround_There is only one %s action in this plan. Please tell me again where I should insert the new action."%action_name)
    return "Done"
  elif (number != -1) and (int(number) > int(n_action_name_occurrencies)): 
    global_vars.event_pub.publish("CouldNotGround_There are only %s %s actions in this plan. Please tell me again where I should insert the new action."%(n_action_name_occurrencies,action_name))
    return "Done"
  elif (number == -1) and (("[]" in first_action_pcl and n_action_name_occurrencies > 1) or (not "[]" in first_action_pcl and n_action_occurrencies > 1)):
    global_vars.event_pub.publish("CouldNotGround_There are multiple %s actions in this plan. Please tell me which one you are referring to."%action_name)
    return "Done"
  elif ("[]" in first_action_pcl) and (n_action_name_occurrencies < 1):
    global_vars.event_pub.publish("CouldNotGround_There are no %s actions in this plan. Please tell me again where I should insert the new action."%action_name)
    return "Done"
  elif (not "[]" in first_action_pcl) and (n_action_occurrencies < 1):
    global_vars.event_pub.publish("CouldNotGround_There are no %s actions of that kind in this plan. Please tell me again where I should insert the new action."%action_name)
    return "Done"
  
  global_vars.currentPlanParamList = getParamNames(kbPlanName)
  
  if number == -1:
    number = 1

  if "[]" in first_action_pcl:
    i = find_nth_substring_start_index(pcl, action_name, int(number))

  else:
    i = find_nth_substring_start_index(pcl, first_action_pcl.replace("(","").replace(" ","").replace(")",""), int(number))
    
  k = pcl[:i].rfind("(")
  j = pcl[i:].find(")")

  if where == "after":
    pcl = pcl[:i+j+2] + " " + second_action_pcl + " " + pcl[i+j+2:]
  else:
    pcl = pcl[:k] + " " + second_action_pcl + " " + pcl[k:]
      
  rospy.loginfo("Processing the new pcl %s"%pcl)
  
  forget_plan(kbPlanName)
  generate_plan(kbPlanName, pcl)
  
  global_vars.log.publish("Inserted action %s %s action %s in %s plan pcl %s obtaining pcl %s "%(second_action_pcl, where, first_action_pcl, req.planName,input_pcl,pcl))
  global_vars.event_pub.publish("ActionInserted_I updated the %s plan"%req.planName)
  
  return "Done"