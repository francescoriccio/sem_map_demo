#!/usr/bin/env python
import roslib; roslib.load_manifest('sapienzbot_dialog')
import rospy
import Tkinter
import threading
import sys
import re
import os

from combine_pnp.srv import TriggerSrv
from Tkinter import *
from std_msgs.msg import *
from sapienzbot_reasoning.srv import ResetProlog

#######################################################################################
### Callbacks

def addEvent_cb():
    pub.publish(String(event.get()))
  
def categorize_cb():
    pub.publish("CATEGORIZE(\"%s\")"%categorizeTarget.get())
  
def connect_cb():
    pub.publish("CONNECT")

def disconnect_cb():
    pub.publish("DISCONNECT")

def dotdetected_cb():
    pub.publish("DotDetected")
 
def goto_cb():
    pub.publish(String("GOTO(\"to the %s\",\"default\")"%goToTarget.get()))
    
def goto_color_cb():
    pub.publish(String("NO_FRAME(\"the %s %s\")"%(goToColorColor.get(), goToColorTarget.get())))

def key(event):
    tell_cb()
    
def historyUp(event):
    global counter, inputString
    if command_history != [] and counter + 1 < len(command_history):
      counter += 1
      inputString.delete(0, Tkinter.END)
      inputString.insert(END, command_history[len(command_history)-counter-1].replace("\n",""))
      
def historyDown(event):
    global counter, inputString
    if counter == 0 and command_history != []:
      counter -= 1
      inputString.delete(0, Tkinter.END)
    elif command_history != [] and counter - 1 >= 0:
      counter -= 1
      inputString.delete(0, Tkinter.END)
      inputString.insert(END, command_history[len(command_history)-counter-1].replace("\n",""))
    
def relation_cb():
    pub.publish(String("NO_FRAME(\"to the %s %s the %s\")"%(goToRelationTarget.get(), goToRelation.get(), goToRelationTarget2.get())))
    
def forget_cb():
    pub.publish(String("FORGET(\"to the %s\")"%forgetTarget.get()))
    
def update_cb():
    pub.publish(String("UPDATE(\"to the %s\")"%updateTarget.get()))

def tell_cb():
    global command_history, counter
    pub.publish(String(inputString.get()))
    command_history.append(inputString.get())
    command_history_file.write('\n'+inputString.get())
    command_history_file.flush()
    inputString.delete(0, Tkinter.END)
    counter = -1

def say_cb(string):
    global say
    say.set("The robot says: %s"%string.data.replace("\n",""))

def cancel_cb():
  pub.publish("CONFIRM(\"cancel\")")
  pub.publish("ABORT")
  
def yes_cb():
  pub.publish("CONFIRM(\"yes\")")
  
def no_cb():
  pub.publish("CONFIRM(\"no\")")
  
def mem_cb():
  pub.publish("MEMORIZE(")

def getObjInSight_cb():
  pub.publish("TellObjInSight")

def reset_actions_cb():
  reset_combine()

def reset_obj_cb():
  try:
    resp1 = reset_prolog("reset")
    
    if(resp1.response != 'Failed'):
      print "Prolog resetted"
    else:
      print "Couldn't reset prolog"
    
  except rospy.ServiceException, e:
    print "Service call failed: %s"%e

def anyone_cb():
  pub.publish('NO_FRAME(\"anyone\")')
  
def here_cb():
  pub.publish('MovedHere')
  
def rm_command_history_cb():
  global command_history_file, command_history, counter
  os.remove("command_history")
  directory                   = rospy.get_param('sapienzbot_dialogPath')
  os.chdir(directory)
  command_history_file        = open("command_history", 'a+')
  counter                     = -1
  command_history             = command_history_file.readlines()

def log_cb():
  log.publish("##########################################")
  log.publish("# NEW EXPERIMENT STARTED                 #")
  log.publish("##########################################")
  
#######################################################################################
### GUI & main

PKG = 'sapienzbot_dialog'
NODE = 'tester'
eventList = ["TURN[right]", "TURN[left]", "TURN[left45]", "TURN[right45]", "TURN[around]", "LOOK[right]", "LOOK[left]", "LOOK[straight]", "FollowMe", "GetCloser", "Home", "Recognize"]
targetList = ["socket", "window", "closet", "bin", "fire-extinguisher", "trash", "hydrant", "exit", "plant", "bench", "sink", "couch", "bed", "heater", "table", "fridge", "oven", "freezer"]
colorList = ["black", "red", "blue", "green", "white", "brown"]
relationList = ["near", "next to", "behind", "to the left of", "to the right of", "in front of"] 
buttons =  ['Move Robot ...', addEvent_cb, 'Goto ...', goto_cb, #'Goto Color ...', goto_color_cb, 
            'The ... Relation ...', relation_cb, 'This is a ...', categorize_cb, 'Forget ...', forget_cb, 'Update ...', update_cb, 'Tell Robot ... ', tell_cb,
            'Memorize', mem_cb, 'Dot Detected', dotdetected_cb, 'Connect',connect_cb, 'Disconnect',disconnect_cb, 'Say Anyone', anyone_cb, 'Say Here', here_cb,
             'Tell Me Objects In Sight', getObjInSight_cb, 'Reset Objects', reset_obj_cb, 'Reset Actions', reset_actions_cb, 'Yes', yes_cb, 'No', no_cb, 
             'Cancel', cancel_cb, 'Remove cmd_history', rm_command_history_cb, 'Log New Experiment', log_cb]

def init_subAndPub():
  global reset_prolog, getObjectsInSight, pub, log
  global reset_combine
  rospy.Subscriber(PKG+"/say_command", String, say_cb, queue_size=1)
    
  reset_prolog  = rospy.ServiceProxy('reset_prolog', ResetProlog)
  reset_combine = rospy.ServiceProxy('/combine_pnp/reset', TriggerSrv)
  
  pub = rospy.Publisher(PKG+"/receive_msg_topic", String, queue_size=1)
  log = rospy.Publisher("/logger/logging", String, queue_size=1)
            
def initEventGUI():
  global event, top, goToTarget, categorizeTarget, forgetTarget, updateTarget, say, inputString
  global goToColorTarget, goToColorColor, goToRelationTarget, goToRelationTarget2, goToRelation

  top = Tkinter.Tk() 
  top.wm_title("PNP Event GUI") 
  
  say = StringVar()
  Label(top, textvariable=say).grid(row=0,column=0,columnspan=4)
  
  event = StringVar(top)
  event.set("Event to Add") # default value
  eventMenu = apply(OptionMenu, (top, event) + tuple(sorted(eventList)))
  eventMenu.config(width=20)
  eventMenu.grid(row=1, column=1)
  
  goToTarget = StringVar(top)
  goToTarget.set("Target") # default value
  targetMenu = apply(OptionMenu, (top, goToTarget) + tuple(sorted(targetList)))
  targetMenu.config(width=20)
  targetMenu.grid(row=2, column=1)
  
  goToRelationTarget = StringVar(top)
  goToRelationTarget.set("Target") # default value
  targetMenu = apply(OptionMenu, (top, goToRelationTarget) + tuple(sorted(targetList)))
  targetMenu.config(width=20)
  targetMenu.grid(row=3, column=1)
  
  goToRelation = StringVar(top)
  goToRelation.set("Relation") # default value
  targetMenu = apply(OptionMenu, (top, goToRelation) + tuple(sorted(relationList)))
  targetMenu.config(width=20)
  targetMenu.grid(row=3, column=2)
  
  goToRelationTarget2 = StringVar(top)
  goToRelationTarget2.set("Reference Target") # default value
  targetMenu = apply(OptionMenu, (top, goToRelationTarget2) + tuple(sorted(targetList)))
  targetMenu.config(width=20)
  targetMenu.grid(row=3, column=3)
  
  categorizeTarget = StringVar(top)
  categorizeTarget.set("Label") # default value
  targetMenu = apply(OptionMenu, (top, categorizeTarget) + tuple(sorted(targetList)))
  targetMenu.config(width=20)
  targetMenu.grid(row=4, column=1)
  
  forgetTarget = StringVar(top)
  forgetTarget.set("Target") # default value
  targetMenu = apply(OptionMenu, (top, forgetTarget) + tuple(sorted(targetList)))
  targetMenu.config(width=20)
  targetMenu.grid(row=5, column=1)
  
  updateTarget = StringVar(top)
  updateTarget.set("Target") # default value
  targetMenu = apply(OptionMenu, (top, updateTarget) + tuple(sorted(targetList)))
  targetMenu.config(width=20)
  targetMenu.grid(row=6, column=1)
  
  inputString = Entry(top)
  inputString.grid(row=7, column=1)
  inputString.bind("<KeyRelease-Return>", key)
  inputString.bind("<KeyRelease-Up>", historyUp)
  inputString.bind("<KeyRelease-Down>", historyDown)
  
  i = 0 # iterator
  r = 7 # number of buttons that require a menu in the list
  c = 4 # number of columns to display the buttons
  w = [0] * (len(buttons)/2)
  
  while (i<len(buttons)):
    w[i/2] = Tkinter.Button ( top, text=buttons[i], command=buttons[i+1], width=20 )
    if i/2<r:
      w[i/2].grid(row=int(i/2+1), column=0)
    else:
      w[i/2].grid(row=(i/2-r)/c+(r+1), column=(i/2-r)%c)
    i = i+2
  
  rospy.loginfo("Gui initialized")
  top.mainloop()
    
    
if __name__ == '__main__':
    global command_history_file, command_history, counter
    directory                   = rospy.get_param('/tester/sapienzbot_dialogPath')
    os.chdir(directory)
    command_history_file        = open("command_history", 'a+')
    counter                     = -1
    command_history             = command_history_file.readlines()
    rospy.init_node(NODE)
    init_subAndPub()
    initEventGUI()
