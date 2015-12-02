#!/usr/bin/env python
PKG = 'combine_pnp'
NODE = 'tester'

import roslib; roslib.load_manifest(PKG)
import rospy
import Tkinter

from Tkinter import *
from std_msgs.msg import *
from combine_pnp.srv import *

#######################################################################################
### Callbacks

def send_cb():
  #generate_plan("TestPlan", "(while (goTo:[the,phd,room]) do (do-sequentially (say:[we,are,going,to,the,phd,room]) (say:[this,is,the,phd,room])))")
  #generate_plan("Tour", "(while (goTo:[nardi's,office]) do (do-sequentially (say:[we,are,going,to,nardi's,office]) (say:[this,is,nardi's,office]) (while (goTo:[the,phd,room]) do (do-sequentially (say:[we,are,going,to,the,phd,room]) (say:[this,is,the,phd,room])))  (say:[we,have,ended,the,tour])))")
  #generate_plan("TestPlan", "(do-sequentially (say:[we,are,going,to,the,phd,room]) (say:[this,is,the,phd,room]))")
  generate_plan("Test", "( while ( goTo:[somewhere] ) do ( do-sequentially ( say:[hello] ) ) )")
  
#######################################################################################
### GUI & main


def initEventGUI():
  global event, top

  top = Tkinter.Tk() 
  top.wm_title("PNP Event GUI") 
  
  w = Tkinter.Button( top, text="Send Graph", command=send_cb, width=20 ) 
  w.grid(row=0, column=0)
  
  rospy.loginfo("Gui initialized")
  top.mainloop()
    
    
if __name__ == '__main__':
    rospy.init_node(NODE)
    generate_plan  = rospy.ServiceProxy('/combine_pnp/generate_plan', GeneratePlanSrv)
    initEventGUI()
