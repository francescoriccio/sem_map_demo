#! /usr/bin/env python

import roslib; roslib.load_manifest('living_at_diag')
import rospy

import actionlib
import Tkinter
import threading
import time
from Tkinter import *
from std_msgs.msg import *

from living_at_diag.msg import *

global buttonPressed
buttonPressed = False

def buttonClick():
  global buttonPressed
  buttonPressed = True

def button1Click(event):
  global buttonPressed
  buttonPressed = True

#######################################################################################
### WaitForUser Action Server

class MyTkApp(threading.Thread):
  def __init__(self):
    threading.Thread.__init__(self)
    self.start()

  def quit(self):
    self.root.withdraw()
    self.root.quit()

  def run(self):
    self.root = Tkinter.Tk()
    self.root.wm_title("Wait For User")
    self.root.protocol("WM_DELETE_WINDOW", buttonClick)
    self.frame = Frame(self.root)
    self.frame.pack()

    label = Tkinter.Label(self.frame,text="Click the Ok button when done", font=("Helvetica", 70))
    label.pack()

    button1=Tkinter.Button(self.frame,text="Ok", font=("Helvetica", 70))
    button1.pack(side=BOTTOM)
    button1.bind("<Button-1>", button1Click)

    self.root.mainloop()

class WaitForUserAction(object):
  _feedback             = WaitForUserFeedback()
  _result               = WaitForUserResult()
  robotname             = rospy.get_param("/robotname", "robot_0")
  
  def __init__(self, name):
    self._action_name =  name    
    self._as = actionlib.SimpleActionServer(self._action_name, living_at_diag.msg.WaitForUserAction, execute_cb=self.execute_cb, auto_start = False)  
    self._as.start()
  
  def execute_cb(self, goal):
    # publish info to the console for the user
    rospy.loginfo('%s: Executing, WaitForUser ...'%self._action_name)
    
    # check that preempt has not been requested by the client
    if self._as.is_preempt_requested():
      self.preempt()
      return
    
    # feedback
    self._feedback.feedback = "Waiting for user to press the button in the GUI"
    self._as.publish_feedback(self._feedback)
    
    # Wait for user to press the button in the GUI
    global buttonPressed
    buttonPressed = False
    app = MyTkApp()
    
    while (not self._as.is_preempt_requested() and not buttonPressed):
      time.sleep(0.2)
    
    app.quit()
    
    if (not self._as.is_preempt_requested()):
      self._result.result = "Succeeded"
      rospy.loginfo('%s: Succeeded' % self._action_name)
      self._as.set_succeeded(self._result)
    else:
      self._result.result = "Aborted"
      rospy.loginfo('%s: Aborted' % self._action_name)
      self._as.set_succeeded(self._result)
 
#######################################################################################
### Main 

if __name__ == '__main__':
    rospy.init_node('waitForUser')
    robotname     = rospy.get_param("/robotname", "robot_0")
    event_pub     = rospy.Publisher("/" + robotname + "/PNPConditionEvent", String, queue_size = 1)
    WaitForUserAction("/"+ robotname +"/waitForUser")
    rospy.spin()
