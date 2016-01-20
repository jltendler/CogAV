import Queue

i = 0
class routeCalc:
	def __init__(self, curX, curY):
		self.curX = curX
		self.curY = curY
	"""	
	path = Queue.Queue()
	path.put((10,0))
	path.put((10,10))
	path.put((0,10))
	path.put((0,0))
	"""
	pathX = [10, 10, 0, 0]
	pathY = [0, 10, 10, 1]
	
	def moveNorth(self):
		print "moving north"
		self.curY += 1
		
	def moveSouth(self):
		print "moving south"
		self.curY -= 1
		
	def moveEast(self):
		print "moving east"
		self.curX += 1
		
	def moveWest(self):
		print "moving west"
		self.curX -= 1
		
	
	def chooseDirection(self):

		if self.pathX[i] - self.curX > 0:
			self.moveEast()
		if self.pathX[i] - self.curX < 0:
			self.moveWest()
		if self.pathY[i] - self.curY > 0:
			self.moveNorth()
		if self.pathY[i] - self.curY < 0:
			self.moveSouth()

	
	
	
def main():
	i = 0
	route = routeCalc(0,0)
	while not ((route.curX == route.pathX[3]) and (route.curY == route.pathY[3])):	#Not done
		if ((route.curX == route.pathX[i]) and (route.curY == route.pathY[i])):  #At current goal
			i += 1
			route.chooseDirection()
		else:
			route.chooseDirection()
			
			
main()
			
		
	
	
		
