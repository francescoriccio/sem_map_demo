class PCL_undefined:
	def __init__(self, symbol):
		self.pclType =  symbol
	
	def setFather(self,father):
		self.father = father

	def __str__(self):
		return "( %s )" % self.pclType

	def getUndefinedChild(self):
		return self

	def setUndefinedChild(self, newChild):
		return

class PCL_leaf:

	def __init__(self,content):
		self.pclType =  "leaf"
		self.content = content
		self.father =  None

	def setFather(self,father):
		self.father = father

	def getFather(self):
		print "Baaa %s" % self.father

	def __str__(self):
		return self.content

	def getUndefinedChild(self):
		return None

	def setUndefinedChild(self, newChild):
		return

class PCL_while:
	
	def __init__(self):
		self.pclType =  "while-do"

	def setWhileChild(self,whileChild):
		self.whileChild = whileChild
		whileChild.setFather(self)

	def setDoChild(self,doChild):
		self.doChild = doChild
		doChild.setFather(self)

	def setFather(self,father):
		self.father = father
	
	def __str__(self):
     		return "( while %s do %s )" % (self.whileChild, self.doChild)

	def getUndefinedChild(self):
		undefinedChild = self.whileChild.getUndefinedChild()
		
		if undefinedChild != None:
			return undefinedChild
		else:
			undefinedChild = self.doChild.getUndefinedChild()
			return undefinedChild

	def setUndefinedChild(self, newChild):
		if isinstance(self.whileChild, PCL_undefined):
			self.whileChild = newChild
			newChild.setFather(self)
		elif isinstance(self.doChild, PCL_undefined):
			self.doChild = newChild
			newChild.setFather(self)
		else:
			undefinedChild = self.whileChild.getUndefinedChild()
		
			if undefinedChild != None:
				self.whileChild.setUndefinedChild(newChild)
			else:
				undefinedChild = self.doChild.getUndefinedChild()
				if undefinedChild != None:
					self.doChild.setUndefinedChild(newChild)

class PCL_don:
	def __init__(self):
		self.pclType =  "do-n-times"

	def setN(self,n):
		self.n = n

	def setDoChild(self,doChild):
		self.doChild = doChild
		doChild.setFather(self)
	
	def setFather(self,father):
		self.father = father

	def __str__(self):
		return "( do-n-times %s %s )" % (self.doChild, self.n)
	
	def getUndefinedChild(self):
		return self.doChild.getUndefinedChild()

	def setUndefinedChild(self, newChild):
		if isinstance(self.doChild, PCL_undefined):
			newChild.setFather(self)
			self.doChild = newChild
		else:
			self.doChild.setUndefinedChild(newChild)

class PCL_dountil:
	def __init__(self):
		self.pclType =  "do-until"

	def setUntilChild(self,untilChild):
		self.untilChild = untilChild
		untilChild.setFather(self)

	def setDoChild(self,doChild):
		self.doChild = doChild
		doChild.setFather(self)
	
	def setFather(self,father):
		self.father = father

	def __str__(self):
		return "( do %s until %s )" % (self.doChild, self.untilChild)

	def getUndefinedChild(self):
		undefinedChild = self.doChild.getUndefinedChild()
		
		if undefinedChild != None:
			return undefinedChild
		else:
			undefinedChild = self.untilChild.getUndefinedChild()
			return undefinedChild

	def setUndefinedChild(self, newChild):
		if isinstance(self.doChild, PCL_undefined):
			newChild.setFather(self)
			self.doChild = newChild
		elif isinstance(self.untilChild, PCL_undefined):
			newChild.setFather(self)
			self.untilChild = newChild
		else:
			undefinedChild = self.doChild.getUndefinedChild()
		
			if undefinedChild != None:
				self.doChild.setUndefinedChild(newChild)
			else:
				undefinedChild = self.untilChild.getUndefinedChild()
				if undefinedChild != None:
					self.untilChild.setUndefinedChild(newChild)

class PCL_ifthenelse:
	def __init__(self):
		self.pclType =  "if-condition"

	def setIfChild(self,ifChild):
		self.ifChild = ifChild
		ifChild.setFather(self)

	def setThenChild(self,thenChild):
		self.thenChild = thenChild
		thenChild.setFather(self)

	def setElseChild(self,elseChild):
		self.elseChild = elseChild
		elseChild.setFather(self)
	
	def setFather(self,father):
		self.father = father

	def __str__(self):
		return "( if-condition %s then %s else %s )" % (self.ifChild, self.thenChild, self.elseChild)

	def getUndefinedChild(self):
		undefinedChild = self.ifChild.getUndefinedChild()
		
		if undefinedChild != None:
			return undefinedChild
		else:
			undefinedChild = self.thenChild.getUndefinedChild()
			if undefinedChild != None:
				return undefinedChild
			else:
				undefinedChild = self.elseChild.getUndefinedChild()
				return undefinedChild

	def setUndefinedChild(self, newChild):
		if isinstance(self.ifChild, PCL_undefined):
			self.ifChild = newChild
			newChild.setFather(self)
		elif isinstance(self.thenChild, PCL_undefined):
			self.thenChild = newChild
			newChild.setFather(self)
		elif isinstance(self.elseChild, PCL_undefined):
			self.elseChild = newChild
			newChild.setFather(self)
		else:
			undefinedChild = self.ifChild.getUndefinedChild()
	
			if undefinedChild != None:
				self.ifChild.setUndefinedChild(newChild)
			else:
				undefinedChild = self.thenChild.getUndefinedChild()
				if undefinedChild != None:
					self.thenChild.setUndefinedChild(newChild)
				else:
					undefinedChild = self.elseChild.getUndefinedChild()
					if undefinedChild != None:
						self.elseChild.setUndefinedChild(newChild)

class PCL_doseq:
	
	def __init__(self):
		self.pclType =  "do-sequentially"
		self.children = []

	def addChild(self,child):
		self.children.append(child)
		child.setFather(self)
	
	def setFather(self,father):
		self.father = father

	def __str__(self):
		childrenString = ""
		for c in self.children:
			childrenString = "%s %s" % (childrenString,c)
			childrenString = childrenString.strip()
     		return "( do-sequentially %s )" % childrenString
		
	def getUndefinedChild(self):
		undefinedChild = None
		for child in self.children:
			undefinedChild = child.getUndefinedChild()
			if undefinedChild != None:
				return undefinedChild

	def setUndefinedChild(self, newChild):
		i = 0
		for child in self.children:
			if isinstance(child, PCL_undefined):
				newChild.setFather(self)
				self.children[i] = newChild
				break
			else:
				undefinedChild = child.getUndefinedChild()
				if undefinedChild != None:
					child.setUndefinedChild(newChild)
					break
			i += 1

