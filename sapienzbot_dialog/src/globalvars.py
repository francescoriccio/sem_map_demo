#!/usr/bin/env python
import roslib; roslib.load_manifest('sapienzbot_dialog')
import rospy
import socket
from std_msgs.msg import String

global sockServer
sockServer = None

global pub_say
pub_say = rospy.Publisher("simple_FSM/say_command", String, queue_size=1)
