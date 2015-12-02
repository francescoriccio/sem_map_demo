#!/usr/bin/env python
PKG = 'combine_pnp'
NODE = 'combine_pnp_server'

import global_vars
import networkx as nx
import os
import roslib; roslib.load_manifest(PKG)
import rospy
import shutil

from analyze_PCL import *
from callbacks import *
from combine_pnp.srv import *
from lxml import etree 
from os.path import expanduser
from prolog_interface.srv import *
from std_msgs.msg import *
from support_functions import *
  
#######################################################################################
### Main

if __name__ == "__main__":
  rospy.init_node(NODE, anonymous=True)
  
  global_vars.robot_name                        = rospy.get_param("/robotname", "robot_0")  
  global_vars.plan_directory                    = rospy.get_param(NODE + '/plans_folder_path', expanduser("~")+'/Desktop')
  global_vars.prolog_directory                  = rospy.get_param(NODE + '/prolog_folder_path', expanduser("~")+'/Desktop')
  global_vars.prolog_handle                     = rospy.ServiceProxy('/prolog_interface/prolog_query', prologSrv)
  global_vars.prolog_assert_handle              = rospy.ServiceProxy('/prolog_interface/prolog_assert', prologAssertSrv)
  global_vars.prolog_retract_handle             = rospy.ServiceProxy('/prolog_interface/prolog_retract', prologRetractSrv)
  global_vars.prolog_retractAll_handle          = rospy.ServiceProxy('/prolog_interface/prolog_retractall', prologRetractSrv)
  global_vars.event_pub                         = rospy.Publisher("/"+global_vars.robot_name+"/PNPConditionEvent", String, queue_size=5)
  global_vars.send_server_message               = rospy.Publisher("/SapienzBotDialog/send_message_topic", String, queue_size=5)
  global_vars.log                               = rospy.Publisher("/logger/logging", String, queue_size=5)
  
  os.chdir(global_vars.plan_directory)
  rospy.loginfo("plan_directory: " + global_vars.plan_directory)
  rospy.loginfo("prolog_directory: " + global_vars.prolog_directory)
  
  rospy.Service(PKG + '/acquire_plan', AcquireSrv, acquire_plan)
  rospy.Service(PKG + '/generate_plan', GeneratePlanSrv, generate_plan_cb)
  rospy.Service(PKG + '/test_plan', GeneratePlanSrv, test_plan_cb)
  rospy.Service(PKG + '/add_undefined_child', TriggerSrv, add_undefined_child)
  rospy.Service(PKG + '/analyze_pcl', TriggerSrv, analyze_pcl)
  rospy.Service(PKG + '/forget', ForgetSrv, forget)
  rospy.Service(PKG + '/init_acquisition', InitSrv, init_acquisition)
  rospy.Service(PKG + '/reset', TriggerSrv, reset)
  rospy.Service(PKG + '/remove_action_from_plan', UpdateAction, remove_action_from_plan)
  rospy.Service(PKG + '/replace_action_in_plan', UpdateAction, replace_action_in_plan)
  rospy.Service(PKG + '/insert_action_in_plan', UpdateAction, insert_action_in_plan)

  rospy.spin()