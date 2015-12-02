#!/usr/bin/env python
import roslib; roslib.load_manifest('sapienzbot_dialog')
import rospy
import globalvars
import re
import robotpose
import socket

from std_msgs.msg import String
from server import connectSockets, disconnectSockets
from sapienzbot_dialog.srv import *
from sapienzbot_reasoning.msg import relationMsg
from pnp_msgs.srv import PNPClearBuffer
from combine_pnp.srv import *
from dot_detector.msg import Obs
from math import sin, cos, degrees

PKG                     = 'sapienzbot_dialog'
NODE                    = 'sapienzbot_dialog_node'
pnp_place               = ""
deepest_subplan_active  = ""
updatedOnTransition     = False
tested_pcl              = False

#######################################################################################
### Handle Tablet & GUI Messages

def messageReceived_cb(data):
    global tested_pcl
    if isinstance(data, basestring):
      message = data
      log.publish("Received message \"%s\" from GUI"%message)
    else: 
      message = data.data.replace("\n","")
      log.publish("Received message \"%s\" from ASR"%message)

    if ("followPerson" in pnp_place and not "END_SYNTH" in message):
        print "publishing stop"
        event_pub.publish("Stop")
    
    rospy.loginfo("Received message: %s"%message)
    
    if message.startswith("LEARN_START"):
        tested_pcl = False
        event_pub.publish("LearnPlan")
    
    elif message.startswith("LEARN("):
        tokens          = message.split(",")
        action_name     = tokens[0].split("(")[1].replace(")","")
        param_list      = []
        for tok in tokens[1:]:
          if "#" in tok:
            param_list.append(tok.replace("#"+re.findall('#(.+?)@',tok)[0]+"@","").replace(")",""))
          else:
            param_list.append(tok.replace("@","").replace(")",""))
        
        for i in param_list:
          action_name += i.title()
        param_list = str(param_list).replace("\'","").replace("\'","")
        event_pub.publish("PlanNamed_%s_%s"%(action_name, param_list))
    
    # for testing expecting something like ( do-sequentially ( goTo:[@location] ) )~office
    elif message.startswith("PCL:"):
      if ("~" in message) or tested_pcl:
        tested_pcl      = True
        
        try:
          pcl,instance = message.split("~")
          test_pcl(instance, pcl.replace("PCL:",""))
        except:
          test_pcl("", message.replace("PCL:",""))
          rospy.logwarn("I received the message %s which does not have a default value. Trying to bind it to the previous one..."%message)
      else:
        acquire_pcl(message.replace("PCL:",""))
    
    elif message.startswith("PERFORM("):
        message = message.replace("PERFORM(","").replace(")","")
        message = message[0].title() + message[1:]
        event_pub.publish(message)
    
    elif message.startswith("DELETE_ACTION("):
        message = message.replace("DELETE_ACTION(","").replace(")","")
        event_pub.publish("ForgetPlan_"+message)
    
    elif message.startswith("UPDATE_ACTION("):
        message = message.replace("UPDATE_ACTION(","").replace(")","")
        event_pub.publish("UpdatePlan_"+message)
    
    elif message.startswith("REMOVE_ACTION("):
        message = message.replace("REMOVE_ACTION(","").replace(")","")
        event_pub.publish("Remove_"+message)
    
    elif message.startswith("REPLACE_ACTION("):
        message = message.replace("REPLACE_ACTION(","").replace(")","").replace(" ","").split(",")
        if len(message) == 1:
          event_pub.publish("Replace_"+message)
        elif len(message) == 2:
          event_pub.publish("Replace_%s_%s"%(message[0], message[1]))
        else:
          rospy.logerr("Received an wrongly formatted REPLACE_ACTION command")
    
    elif message.startswith("INSERT_ACTION("):
        message = message.replace("INSERT_ACTION(","").replace(")","").replace(" ","").split(",")
        if len(message) == 2:
          event_pub.publish("Insert_%s_%s"%(message[0],message[1]))
        elif len(message) == 3:
          event_pub.publish("Insert_%s_%s_%s"%(message[0], message[1], message[2]))
        else:
          rospy.logerr("Received an wrongly formatted INSERT_ACTION command")
    
    elif message.startswith("TURN"):
      if "around" in message:
        event_pub.publish("Turn_180")
      elif "left45" in message:
        event_pub.publish("Turn_left45")
      elif "right45" in message:
        event_pub.publish("Turn_right45")
      elif "left" in message:
        event_pub.publish("Turn_left")
      elif "right" in message:
        event_pub.publish("Turn_right")
    
    elif message.startswith("LOOK"):
      if "straight" in message:
        event_pub.publish("Memorize")
      elif "left" in message:
        event_pub.publish("Look_left45_right45")
      elif "right" in message:
        event_pub.publish("Look_right45_left45")
      
    elif message.startswith("GOTO"):
      if "closer" in message:
        event_pub.publish("GetCloser")
      else:
        event_pub.publish("GoTo_GOTO#" + message.split("\",\"")[0].split("\"")[1])
      
    elif (message.startswith("NO_FRAME") and "CGeneral" in pnp_place):
      event_pub.publish("GoTo_NOFRAME#" + message.split("\"")[1])
      event_pub.publish("Forget_NOFRAME#" + message.split("\"")[1])
      event_pub.publish("Update_NOFRAME#" + message.split("\"")[1])
    
    elif message.startswith("FORGET"):
      event_pub.publish("Forget_FORGET#" + message.split("\",\"")[0].split("\"")[1])
      
    elif message.startswith("UPDATE"):
      event_pub.publish("Update_" + message.split("\",\"")[0].split("\"")[1])
    
    elif message.startswith("ABORT"):
      event_pub.publish("Stop")
    
    elif message.startswith("CONNECT"):
      connectSockets()

    elif message.startswith("DISCONNECT"):
      disconnectSockets()

    elif message.startswith("DotDetected"):
      publish_dot()
      event_pub.publish("DotDetected")
    
    elif message.startswith("MEMORIZE"):
      event_pub.publish("Memorize")
      
    elif message.startswith("FOLLOW"):
      event_pub.publish("FollowMe")
      
    elif message.startswith("CATEGORIZE"):
      obj_type = message[str(message).find('"')+1 : str(message).rfind('"')]
      event_pub.publish("Categ_%s_Should I memorize %s?"%(obj_type, obj_type))
      
    elif message.startswith("CONFIRM"):
      if ("yes" in message):
        event_pub.publish("Yes")
      elif ("no" in message):
        event_pub.publish("No")
      elif "cancel" in message:
        event_pub.publish("Stop")
        reset_mem_pub.publish("Reset")

    elif message.startswith("RESTART"):
        event_pub.publish("Home")

    elif message.startswith("RECOGNIZE"):
        event_pub.publish("Recognize")
    
    else:
      event_pub.publish(message)

def loadGrammar_cb(grammar):
  rospy.loginfo('Loading grammar: ' +grammar.data)
  log.publish("Sent \"[LOAD_GRAMMAR] %s\\n\" to ASR"%grammar.data)
  if (not globalvars.sockServer is None): 
      globalvars.sockServer.send('[LOAD_GRAMMAR] '+grammar.data+'\n')
    
def sendServerMsg_cb(message):
  message = message.data
  rospy.loginfo('Sent message \'%s\' to the server' %message)
  log.publish("Sent \"%s\" to ASR"%message)
  if (not globalvars.sockServer is None): 
    globalvars.sockServer.send('%s\n'%message)

def publish_dot():
    rp=[0,0,0]
    robotpose.getRobotPose(rp)
    print "Robot pose: ",rp
    obs = Obs()
    obs.posx=rp[0] + 1.2*cos(rp[2])
    obs.posy=rp[1] + 1.2*sin(rp[2])
    obs.theta=rp[2]+3.14
    print "Tagged object: ",obs.posx,obs.posy
    obs.dimx=0.50
    obs.dimy=0.50
    obs.dimz=0.03
    #obs.properties="objImagedata~rgb-20130731174135.png^raw-20130731174135.png^depth-20130731174135.png^laser-20130731174135.png"
    pub_dot.publish(obs)
    say('[BEEP]500|400\n')
    
def update_pnp_place_cb(message):
  global pnp_place, updatedOnTransition, deepest_subplan_active
  
  if (message.data != pnp_place and updatedOnTransition == True):
    updatedOnTransition = False
  
  pnp_place = message.data
  temp = pnp_place.split(";")[0]

  if ( (deepest_subplan_active != temp) and (".exec" in temp) and (temp[0] == temp[0].title()) ):
    deepest_subplan_active = temp
    log.publish("Started subplan %s"%deepest_subplan_active.replace(".exec",""))
  
  if (updatedOnTransition == False): 
    #log.publish("Executed PNP transition to %s"%pnp_place)

    if ("CConfirm" in pnp_place):
      load_grammar_pub.publish('frame_confirm')

    elif ("CCategorize" in pnp_place):
      load_grammar_pub.publish('frame_categorization')

    elif ("CGeneral" in pnp_place):
        load_grammar_pub.publish('general')

    elif ("CGrammar" in pnp_place):
        load_grammar_pub.publish('frame_grammar')
    
    elif ("CPCL_procedures_confirm" in pnp_place):
        load_grammar_pub.publish("PCL_procedures_confirm")
    
    elif ("CPCL_procedures" in pnp_place):
        load_grammar_pub.publish('PCL_procedures')
    
    elif ("CPCL_action_learning" in pnp_place):
        load_grammar_pub.publish("PCL_action_learning")
    
    elif ("CPCL_action_update" in pnp_place):
        load_grammar_pub.publish("PCL_insert_replace_remove")
     
    elif ("CPCL_insert2" in pnp_place):
        load_grammar_pub.publish("PCL_insert_action_step2")
    
    elif ("CPCL_replace2" in pnp_place):
        load_grammar_pub.publish("PCL_replace_action_step2")
    
    elif ("CPCL_remove2" in pnp_place):
        load_grammar_pub.publish("PCL_remove_action_step2")
   
    updatedOnTransition = True
    
#######################################################################################
### Implement Say Action

def say_cb(req):
    message = str(chekIfKeywordAndTransform(req.message))
    say(message)
    return SaySrvResponse("OK")

def chekIfKeywordAndTransform(message):
    if message == "MemReady" :
      return 'I am ready to memorize an object'
    elif message == "OtherLabel" :
      return 'Ok, please tell me another label'
    elif message == "Which?" :
      return 'I know multiple objects of that type, which one do you mean?'
    elif message == "PointReady" :
      return 'Please point at the new location of the object'
    elif message == "PointReady" :
      return 'Please point at the new location of the object'
    else:
      return message

def say(message):
    rospy.loginfo("Saying: "+message)
    log.publish("Sent \"[SAY] %s\\n\" to ASR"%message)
    pub_say.publish(String(message))
    
    if (not globalvars.sockServer is None):
      sstr = '[SAY] '+message+'\n'
      globalvars.sockServer.send(sstr+'\n') 

#######################################################################################
### Main

if __name__ == "__main__":
    rospy.init_node(NODE)
    
    robot_name = rospy.get_param("/robotname", "robot_0")
    rospy.loginfo("Robot Name: %s"%(robot_name))
    
    sub_msg             = rospy.Subscriber("/"+PKG+"/receive_msg_topic", String, messageReceived_cb, queue_size=5)
    sub_loadGrammar     = rospy.Subscriber("/"+PKG+"/load_grammar_topic", String, loadGrammar_cb, queue_size=5)
    sub_sendMsg         = rospy.Subscriber("/"+PKG+"/send_message_topic", String, sendServerMsg_cb, queue_size=5)
    sub_pnp_places      = rospy.Subscriber("/"+robot_name+"/pnp_node/currentActivePlaces", String, update_pnp_place_cb, queue_size=5)
    
    pub_say             = rospy.Publisher("/"+PKG+"/say_command", String, queue_size=5)
    pub_dot             = rospy.Publisher("/"+robot_name+"/ObservationTopic", Obs, queue_size=5)
    event_pub           = rospy.Publisher("/"+robot_name+"/PNPConditionEvent", String, queue_size=5)
    load_grammar_pub    = rospy.Publisher("/sapienzbot_dialog/load_grammar_topic", String, queue_size=5)
    reset_mem_pub       = rospy.Publisher("/sapienzbot_reasoning/reset_grounding_memory", String, queue_size=5)
    log                 = rospy.Publisher("/logger/logging", String, queue_size=5)

    s           = rospy.Service('SaySrv', SaySrv, say_cb)
    acquire_pcl = rospy.ServiceProxy('/combine_pnp/acquire_plan', AcquireSrv)
    test_pcl    = rospy.ServiceProxy('/combine_pnp/test_plan', GeneratePlanSrv)
    
    rospy.loginfo("The sapienzbot_dialog is ready and runnig...")
    rospy.spin()
