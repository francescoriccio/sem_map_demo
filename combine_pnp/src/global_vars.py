#!/usr/bin/env python
import rospy
import matplotlib.pyplot as plt
import networkx as nx

event_pub                       = None
current_pcl                     = None
G                               = nx.DiGraph() 
idCounter                       = 0
currentPlanParamList            = []
MAX_PNP_PLAN_X                  = 1500.
MAX_PNP_PLAN_Y                  = 1400.
pclQueue                        = []
plan_directory                  = "/"
plan_name                       = ""
prolog_directory                = "/"
prolog_handle                   = None
prolog_assert_handle            = None
prolog_retract_handle           = None
prolog_retractAll_handle        = None
analyze_PCL_resetted            = False
robot_name                      = None
log                             = None
pcl_var_instances_map           = {}
