#!/usr/bin/env python
import roslib; roslib.load_manifest('sapienzbot_dialog')
import rospy

from std_msgs.msg import String
from sapienzbot_dialog.srv import *

PKG = 'sapienzbot_dialog'
NODE = 'queryLoop'

def queery_loop(req):
    objList = req.objList.split(",")
    objListAtoms = req.objListAtoms.split(",")
    index = req.index
    
    print "index is %s and len(Obj) %s"%(index,len(objList))
    
    if (index < len(objList)):
      say("I know that " + objList[index] + " is also here. Should I keep it in my memory?")
      event_pub.publish("QueryUser")
      return QueryLoopResponse("QueryUser")
    else:
      event_pub.publish("EndLoop")
      return QueryLoopResponse("EndLoop")
      
if __name__ == '__main__':
    rospy.init_node(NODE)
    robotname   = rospy.get_param("/robotname", "robot_0")
    say         = rospy.ServiceProxy('/SaySrv', SaySrv)
    event_pub   = rospy.Publisher("/" + robotname + "/PNPConditionEvent", String, queue_size=1)

    rospy.Service('/' + robotname + '/queryLoop', QueryLoop, queery_loop)

    rospy.spin()
