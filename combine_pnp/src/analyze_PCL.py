import global_vars
import re
from callbacks import *
from PCL import *

#primitives = ["while","do-sequentially","do","if","then","else","do-n-times","until"]

def buildPCL(stringPcl):
  tokens        = stringPcl.split()
  actualPcl     = tokens[1:len(tokens)-1]
  
  if actualPcl[0].startswith("while"):
    pcl         = PCL_while()
    innerPcl    = getInnerPcl(actualPcl)
    pcl.setWhileChild(buildPCL(innerPcl))

    openBrackets        = 0
    closedBrackets      = 0
    i                   = 0
    
    for s in actualPcl:
      if openBrackets != 0 and openBrackets != 0 and openBrackets == closedBrackets:
        break
      if s == "(" or s.startswith("("):
        openBrackets += 1
      elif s == ")" or s.endswith(")"):
        closedBrackets += 1
      i += 1

    if actualPcl[i] == "do":
      innerPcl2 = getInnerPcl(actualPcl[i:])
      pcl.setDoChild(buildPCL(innerPcl2))
    else:
      rospy.logerr("OOPPS, there's an index error!")
    
  elif actualPcl[0].startswith("do-sequentially"):
    pcl                   = PCL_doseq()
    openBrackets          = 0
    closedBrackets        = 0
    retPcl                = ""

    for s in actualPcl:
      if closedBrackets > openBrackets:
        break
      if openBrackets != 0 and openBrackets != 0 and openBrackets == closedBrackets:
        openBrackets      = 0
        closedBrackets    = 0
        pcl.addChild(buildPCL(retPcl.strip()))
        retPcl            = ""

      if s == "(" or s.startswith("("):
        openBrackets += 1
      elif s == ")" or s.endswith(")"):
        closedBrackets += 1
      if openBrackets > 0:
        retPcl += " " + s
    
    if openBrackets != 0 and openBrackets != 0 and openBrackets == closedBrackets:
      pcl.addChild(buildPCL(retPcl.strip()))
    
  elif actualPcl[0].startswith("do-n-times"):
    pcl         = PCL_don()
    innerPcl    = getInnerPcl(actualPcl)
    pcl.setDoChild(buildPCL(innerPcl))
    pcl.setN(actualPcl[len(actualPcl)-1])
    
  elif actualPcl[0].startswith("do"):
    pcl      = PCL_dountil()
    innerPcl = getInnerPcl(actualPcl)
    pcl.setDoChild(buildPCL(innerPcl))

    openBrackets          = 0
    closedBrackets        = 0
    i                     = 0

    for s in actualPcl:
      if openBrackets != 0 and openBrackets != 0 and openBrackets == closedBrackets:
              break
      if s == "(" or s.startswith("("):
              openBrackets += 1
      elif s == ")" or s.endswith(")"):
              closedBrackets += 1
      i += 1

    if actualPcl[i] == "until":
      innerPcl2 = getInnerPcl(actualPcl[i:])
      pcl.setUntilChild(buildPCL(innerPcl2))
    else:
      rospy.logerr("OOPPS, there's an index error!")
    
  elif actualPcl[0].startswith("if-condition"):
    pcl           = PCL_ifthenelse()          
    innerPcl      = getInnerPcl(actualPcl)
    
    pcl.setIfChild(buildPCL(innerPcl))
    
    openBrackets          = 0
    closedBrackets        = 0
    i                     = 0
    
    for s in actualPcl:
      if openBrackets != 0 and openBrackets != 0 and openBrackets == closedBrackets:
        break
      if s == "(" or s.startswith("("):
        openBrackets += 1
      elif s == ")" or s.endswith(")"):
        closedBrackets += 1
      i += 1

    if actualPcl[i] == "then":
      innerPcl2 = getInnerPcl(actualPcl[i:])
      pcl.setThenChild(buildPCL(innerPcl2))
      
    else:
      rospy.logerr("OOPPS, there's an index error!")

    openBrackets          = 0
    closedBrackets        = 0
    j                     = 0
    
    for j in range(i,len(actualPcl)):
      s = actualPcl[j]
      if openBrackets != 0 and openBrackets != 0 and openBrackets == closedBrackets:
        break
      if s == "(" or s.startswith("("):
        openBrackets += 1
      elif s == ")" or s.endswith(")"):
        closedBrackets += 1

    if actualPcl[j] == "else":
      innerPcl3 = getInnerPcl(actualPcl[j:])
      pcl.setElseChild(buildPCL(innerPcl3))
    else:
      rospy.logerr("OOPPS, there's an index error!")
    
  elif actualPcl[0].startswith("A"):
    pcl = PCL_undefined("A")
  else:
    pcl = PCL_leaf(stringPcl)
  return pcl 

def getInnerPcl(arrayPcl):
  openBrackets          = 0
  closedBrackets        = 0
  retPcl                = ""

  for s in arrayPcl:
    if openBrackets != 0 and openBrackets != 0 and openBrackets == closedBrackets:
      return retPcl.strip()

    if s == "(" or s.startswith("("):
      openBrackets += 1
      
    elif s == ")" or s.endswith(")"):
      closedBrackets += 1
      
    if openBrackets > 0:
      retPcl += " " + s

  if openBrackets != 0 and openBrackets != 0 and openBrackets == closedBrackets:
    return retPcl.strip()

def dialogue():
  pcl = global_vars.currentPcl
  print "current pcl in dialog: %s"%pcl
  global_vars.pclQueue.append(pcl)
  undefinedChild = pcl.getUndefinedChild()
  global_vars.analyze_PCL_resetted    = True
  if undefinedChild is None and pcl.pclType == "do-sequentially":
    print "called dialog -> PCLDefAsk_Should I do something else afterwards?"
    global_vars.event_pub.publish("PCLDefAsk_Should I do something else afterwards?")
    
  elif undefinedChild is None:
    print "called dialog -> PCLDef"
    global_vars.event_pub.publish("PCLDef")

  else:
    print "called dialog -> PCLUndef_What should I do to complete the %s?"%pcl.pclType
    global_vars.event_pub.publish("PCLUndef_What should I do to complete the %s?"%pcl.pclType)
    
def analyze_pcl(req):
  print len(global_vars.pclQueue)
  print "called analyze_pcl %s"%global_vars.pclQueue
  
  if len(global_vars.pclQueue) == 1:
    global_vars.analyze_PCL_resetted    = False
    global_vars.event_pub.publish("Done_%s"%global_vars.currentPcl)    
  
  elif len(global_vars.pclQueue) > 1:
    global_vars.pclQueue.pop(len(global_vars.pclQueue)-1)
    fatherPcl = global_vars.pclQueue[len(global_vars.pclQueue)-1]
    fatherPcl.setUndefinedChild(global_vars.currentPcl)
    global_vars.pclQueue.pop(len(global_vars.pclQueue)-1)
    global_vars.currentPcl = fatherPcl
    dialogue()
    global_vars.event_pub.publish("NotDone")
    
  else:
    rospy.logerr("Unexpected error: length of pclQueue <= 0")
  
  return "Done"

def add_undefined_child(req):
  pcl                   = global_vars.currentPcl
  undefinedChild        = pcl.getUndefinedChild()
  
  if undefinedChild is None:
    undefinedPcl                = PCL_undefined("A")
    pcl.addChild(undefinedPcl)
    global_vars.currentPcl      = pcl
    
  return "Done"
  
def acquire_plan(req):
  if global_vars.analyze_PCL_resetted:
    global_vars.event_pub.publish("PCLReceived")
    global_vars.currentPcl              = buildPCL(req.pcl)
    global_vars.analyze_PCL_resetted    = False
    dialogue()
    return "Done"
  else:
    return "Failed"
  
def init_acquisition(req):
  rospy.loginfo("analyze_PCL node resetted")
  global_vars.currentPcl                = None
  global_vars.pclQueue                  = []
  global_vars.currentPlanParamList      = []
  global_vars.pcl_var_instances_map     = {}
  tokens                                = re.split(' |\.|-|_', req.plan_name)
  global_vars.plan_name                 = ''.join(x[:1].upper() + x[1:] for x in tokens)  
  global_vars.currentPlanParamList      = req.param_list.replace(" ","").replace("[","").replace("]","").split(",")
  global_vars.analyze_PCL_resetted      = True
  
  return "Done"