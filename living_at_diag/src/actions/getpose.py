#!/usr/bin/python
# -*- coding: utf-8 -*-


import sys
import datetime
import time
import math
from math import fabs,sqrt,pi,degrees,radians,atan2,tan,sin,cos

PKG = 'living_at_diag'
NODE = 'getpose'

import roslib; roslib.load_manifest(PKG)

import rospy
import tf
import actionlib
import robotpose

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

from living_at_diag.srv import RobotPose


tfl = 0

def _CANC_getRobotPose():
  global tfl
  print '*** getRobotPose...'
  found = False
  while (not found and not rospy.is_shutdown()):
    try:
      source = '/map'
      dest = '/'+robotname+'/base_link'
      print '*** lookup transform: '+source+' to '+dest
      
      (trans,rot) = tfl.lookupTransform(source, dest, rospy.Time(0))

      (r, p, y) = euler_from_quaternion(rot)

      print '*** T = ',trans
      print '*** R = ',rot,'  yaw = ',degrees(y),' deg'
      found = True
  
    except:
# (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      rospy.sleep(0.2)
      print '*** exception here: ', sys.exc_info()[0]
      continue



def initNode(NODE):
  global pub_cmd
  global sub_odom
  global tfl
  rospy.init_node(NODE+'_') 
  tfl = tf.TransformListener()
  
  #rospy.sleep(1.0)  
  #pub_cmd = rospy.Publisher('/'+robotname+'/ps3joy_cmd_vel', Twist)
  #sub_odom = rospy.Subscriber('/'+robotname+'/odom', Odometry, odom_callback)
  rospy.sleep(1.0)


#  MAIN 

if __name__ == '__main__':
  global robotname
  initNode(NODE)
  robot_name  = rospy.get_param('/robotname', "robot_0")
  V = [0,0,0]
  #getRobotPose_tf(V)
  #print "Direct tf get robot pose : ",V
  robotpose.getRobotPose(V)
  print "Service get robot pose : ",V

