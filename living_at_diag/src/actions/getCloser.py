#! /usr/bin/env python

import roslib; roslib.load_manifest('living_at_diag')
import rospy

import actionlib
import robotpose

from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

from laser_analysis.msg import LaserObstacle
from math import degrees, radians, pi

from living_at_diag.msg import *

MAX_TRANS_VEL   = 1.0
MIN_TRANS_VEL   = 0.4 ### min velocity
ZERO_TRANS_VEL  = 0.2 ## put vel to zero if vel below this
MAX_PERSON_DIST = 8.0
MIN_PERSON_DIST = 1.0
MAX_ANG_VEL     = 1.0

#######################################################################################
### Support Functions
def norm180(a):
  while (a>180):
    a = a-360
  while (a<=-180):
    a = a+360
  return a
    
def roundPI2(a):
    if ((a>=-pi/4 and a<=pi/4) or (a>=7*pi/4 and a<=2*pi)):
        return 0
    elif (a>=pi/4 and a<=3*pi/4):
        return pi/2
    elif ((a>=3*pi/4 and a<=5*pi/4) or (a>=-pi and a<=-3*pi/4)):
        return pi
    elif ((a>=5*pi/4 and a<=7*pi/4) or (a>=-3*pi/4 and a<=-pi/4)):
        return -pi/2;

#######################################################################################
### GetCloser Action Server

class GetCloserAction(object):
  _feedback             = GetCloserFeedback()
  _result               = GetCloserResult()
  robotname             = rospy.get_param("/robotname", "robot_0")
  pub_cmd               = rospy.Publisher('/'+robotname+'/desired_cmd_vel', Twist, queue_size = 1)
  person_dist           = -1
  person_dist_y         = 0
  
  def __init__(self, name):
    self._action_name =  name    
    self._as = actionlib.SimpleActionServer(self._action_name, living_at_diag.msg.GetCloserAction, execute_cb=self.execute_cb, auto_start = False)  
    self.sub_lob = rospy.Subscriber('/'+self.robotname+'/laser_obstacle', LaserObstacle, self.laser_obstacle_callback)
    self._as.start()
    
  def execute_cb(self, goal):
    
    # publish info to the console for the user
    rospy.loginfo('%s: Executing, getCloser...' % (self._action_name))
    
    # check that preempt has not been requested by the client
    if self._as.is_preempt_requested():
      self.preempt()
      return
    
    # feedback
    self._feedback.feedback = "Getting closer"
    self._as.publish_feedback(self._feedback)
    
    self.do_getcloser()
    
    self._result.result = "Succeeded"
    rospy.loginfo('%s: Succeeded' % self._action_name)
    self._as.set_succeeded(self._result)
        
  def do_getcloser(self):
    rp = [0,0,0]
    while (not robotpose.getRobotPose(rp)):
      rospy.sleep(0.25)

    I_err       = 0
    prev_err    = 0
    yaw         = rp[2]
    GTh         = roundPI2(yaw)
    cmd_vel     = Twist()
    status      = 1 # go
    
    while((not self._as.is_preempt_requested()) and status==1):
      if (not robotpose.getRobotPose(rp)):
        continue

      X         = rp[0]
      Y         = rp[1]
      yaw       = rp[2]
      adist     = norm180(degrees(GTh-yaw))
      trans_vel = 0
      midang=35

      if (status==1):
          if (self.person_dist>MIN_PERSON_DIST):
              trans_vel = max(MIN_TRANS_VEL,MAX_TRANS_VEL*min(1.0,(self.person_dist-MIN_PERSON_DIST)/0.6))
          elif (self.person_dist<0):
              pass
          else:
              status = 2 # Finish

      if (abs(adist)>midang):
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = MAX_ANG_VEL
      else:
        cmd_vel.linear.x = trans_vel * (midang-abs(adist))/midang
        cmd_vel.angular.z = MAX_ANG_VEL

      if (cmd_vel.linear.x < ZERO_TRANS_VEL):
        cmd_vel.linear.x = 0

      if (abs(adist)>40 and abs(adist)<50):
        I_err = 0
      
      # PID controller
      err = radians(adist)
      P_err = err
      I_err += err
      D_err = err - prev_err
      prev_err = err
      Kp = 0.50
      Ki = 0.000
      Kd = 0.001
      v = Kp * P_err + Ki * I_err + Kd * D_err
      cmd_vel.angular.z *= v

      self.pub_cmd.publish(cmd_vel)
      rospy.sleep(0.10)
    # end while
    self.stop_getcloser()


  def stop_getcloser(self):
    cmd_vel               = Twist()
    cmd_vel.linear.x      = 0
    cmd_vel.linear.y      = 0
    cmd_vel.linear.z      = 0
    cmd_vel.angular.x     = 0
    cmd_vel.angular.y     = 0
    cmd_vel.angular.z     = 0
    self.pub_cmd.publish(cmd_vel)
 
  def laser_obstacle_callback(self, data):
    if (data.npoints>2 and data.mindist<MAX_PERSON_DIST):
        self.person_dist = data.mindist
    else:
        self.person_dist = -1
 
#######################################################################################
### Main 

if __name__ == '__main__':
    rospy.init_node('getCloser')
    robotname     = rospy.get_param("/robotname", "robot_0")
    GetCloserAction("/"+ robotname +"/getCloser")
    rospy.spin()
