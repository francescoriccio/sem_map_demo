from conceptnet.models import Concept
import os

objectsList = ["cylinder"]#["office","room","corridor","toilet","door","window","exit","table","chair","fire extinguisher","socket","power","printer","emergency exit","closet","book","chair","bench","trash bin","plant","hydrant","printer","professor"]

if(os.path.exists("conceptnetPropertyList.txt")):
	os.remove("conceptnetPropertyList.txt")
	
file_name = open('conceptnetPropertyList.txt', 'a')

for i in xrange(len(objectsList)):  
  print "retrieving predicates of %s from concpetNet"%objectsList[i]
  file_name.write("#### OBJECT %s: %s\n"%(i+1,objectsList[i]))
  try:
    obj = Concept.get('%s'%objectsList[i], 'en')
    for fwd in obj.get_assertions():
      #if "Property" in "%s"%fwd:
      	file_name.write("%s\n"%fwd)

    file_name.write("\n")
  except:
    print "could not retrieve anything for %s from conceptNet"%objectsList[i]
