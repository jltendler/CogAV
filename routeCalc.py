import numpy as np
import matplotlib.pyplot as plt

class routeCalc:
	def __init__(self, curX, curY):
		self.curX = curX
		self.curY = curY

	pathX = [0, 2, -2, 2, -2]
	pathY = [0, 3, 6, 9, 12]
	
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
	
	#TODO: (Tyler) - figure out this functionality
	#TODO: Eventually figure out how to calculate proper direction (tangent(y/x))
	#TODO: Will need to add new parameter "facing" (in degrees)
	def moveNortheast(self):
		print "moving northeast"
		self.curY += 1
		self.curX += 1
		
	def moveNorthwest(self):
		print "moving northwest"
		self.curY += 1
		self.curX -= 1
		
	def moveSoutheast(self):
		print "moving southeast"
		self.curY -= 1
		self.curX += 1
		
	def moveSouthwest(self):
		print "moving southwest"
		self.curY -= 1
		self.curX -= 1
		
	
	
def main():
	i = 0
	xRoute = []
	yRoute = []
	route = routeCalc(0,0)
	
	while not ((route.curX == route.pathX[len(route.pathX)-1]) and (route.curY == route.pathY[len(route.pathY)-1])):	#Not done
		if ((route.curX == route.pathX[i]) and (route.curY == route.pathY[i])):  #At current goal
			i += 1
			while route.pathX[i] - route.curX > 0:
				print (route.curX, route.curY)
				route.moveEast()
				xRoute.append(route.curX)
				yRoute.append(route.curY)
				
			while route.pathX[i] - route.curX < 0:
				print (route.curX, route.curY)
				route.moveWest()
				xRoute.append(route.curX)
				yRoute.append(route.curY)
				
			while route.pathY[i] - route.curY > 0:
				print (route.curX, route.curY)
				route.moveNorth()
				xRoute.append(route.curX)
				yRoute.append(route.curY)
				
			while route.pathY[i] - route.curY < 0:
				print (route.curX, route.curY)
				route.moveSouth()
				xRoute.append(route.curX)
				yRoute.append(route.curY)
				
			if route.curX is route.pathX[i] and route.curY is route.pathY[i]:
				print "reached destination point"
				print (route.curX, route.curY)
	
	#Plots fastest route with dashed green line
	plt.plot(route.pathX, route.pathY, 'go--')
	plt.axis([(min(route.pathX)-2), (max(route.pathX)+2), (min(route.pathX)-2), (max(route.pathY)+2)])
	
	#Plots actual route with solid blue line
	plt.plot(xRoute, yRoute, '-o')
	
	#Draws graphics with scaled axes
	plt.axis([(min(route.pathX)-2), (max(route.pathX)+2), (min(route.pathX)-2), (max(route.pathY)+2)])
	plt.show()
			
main()
			
		
	
	
		
