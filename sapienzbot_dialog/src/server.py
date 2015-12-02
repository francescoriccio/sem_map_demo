#!/usr/bin/env python
import roslib; roslib.load_manifest('sapienzbot_dialog')
import threading
import socket
import time
import sys
import rospy
import globalvars
from std_msgs.msg import String

HOST_SSR = '10.42.0.93'
PORT_SSR = 1800
TOTAL_NUMBER_OF_RETRIES = 10
run_sockets = False
pub_msg = rospy.Publisher("/sapienzbot_dialog/receive_msg_topic", String, queue_size=5)

def connectSockets():
  global run_sockets
  run_sockets = False
  numberOfRetries = 0
  rospy.loginfo('Connecting to SSR server %s:%d ...' % (HOST_SSR, PORT_SSR))
  
  try:
    globalvars.sockServer = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    globalvars.sockServer.connect((HOST_SSR, PORT_SSR))
    rospy.loginfo('Connection OK')
    th = threading.Thread(None, runServerEvent,args=())
    th.start()
    run_sockets = True
    
  except socket.error, errmsg:
    rospy.logerr('Connection error: %s'%errmsg)
    time.sleep(3)
    if (numberOfRetries != TOTAL_NUMBER_OF_RETRIES):
      rospy.loginfo('Retrying...')
      numberOfRetries += 1
    else:
      rospy.logwarn('Reached maximum number of retries')
      sys.exit(1)
  except KeyboardInterrupt:
    rospy.loginfo('Exit')
    sys.exit(1)
  except:
    rospy.logerr(sys.exc_info()[0])
    
def disconnectSockets():
  global run_sockets
  run_sockets = False
  if (not globalvars.sockServer is None):
    globalvars.sockServer.close()
    globalvars.sockServer = None

def runServerEvent():
  global run_sockets
  globalvars.sockServer.send("[CONNECT]Robot\n")
  time.sleep(0.2)
  globalvars.sockServer.send("[INIT]\n")
  globalvars.sockServer.send("[LOAD_GRAMMAR] frame_grammar\n")
  time.sleep(0.2)
  
  rospy.loginfo('Server-event thread started')
  while (run_sockets):
    try:
      received = globalvars.sockServer.recv(1024)
      if (len(received)>0):
        received = received[0:len(received)-2]
        rospy.loginfo('STO RICEVENDO UN MESSAGGIO Received: %s' % received)
        pub_msg.publish(received)
      else:
        rospy.logerr('Received null string. Exiting...')
        run_sockets=False
    except socket.error, msg:
      rospy.logerr('*** Exception in socket receive: '+str(msg))
      rospy.logerr(sys.exc_info()[0])
      time.sleep(1)
    except KeyboardInterrupt:
      rospy.loginfo('Exit')
      run_sockets=False
  rospy.loginfo('Server-event thread finished')
