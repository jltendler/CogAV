class routeCalc:
	def __init__(self, curX, curY):
		self.curX = curX
		self.curY = curY

	pathX = [0, 10, 10, 0]
	pathY = [0, 0, 10, 10]
	
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
		
	
	
def main():
	i = 0
	route = routeCalc(0,0)
	
	while not ((route.curX == route.pathX[3]) and (route.curY == route.pathY[3])):	#Not done
		if ((route.curX == route.pathX[i]) and (route.curY == route.pathY[i])):  #At current goal
			i += 1
			while route.pathX[i] - route.curX > 0:
				print (route.curX, route.curY)
				route.moveEast()
			while route.pathX[i] - route.curX < 0:
				print (route.curX, route.curY)
				route.moveWest()
			while route.pathY[i] - route.curY > 0:
				print (route.curX, route.curY)
				route.moveNorth()
			while route.pathY[i] - route.curY < 0:
				print (route.curX, route.curY)
				route.moveSouth()
			if route.curX is route.pathX[i] and route.curY is route.pathY[i]:
				print "reached destination point"
				print (route.curX, route.curY)
			
main()
			
		
	
	
		
