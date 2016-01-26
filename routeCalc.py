import numpy as np
import matplotlib.pyplot as plt
from operator import *
import math
from itertools import izip, tee

class routeCalc:
	def __init__(self, curX, curY):
		self.curX = curX
		self.curY = curY

<<<<<<< HEAD
	pathX = [0, -2, -4, 5, 6]
	pathY = [0, 3, 6, 9, 12]
=======
	pathX = [0, 3, 5, 7, 9]
	pathY = [0, 4, 10, 13, 10]
>>>>>>> 79a682f1c28d0df88c815b81db99dec07051ba3e
	
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

#This is used to calculate relative distances between GPS points
#Credit: @pillmuncher - stackoverflow
def pairwise(iterable):
    a, b = tee(iterable)
    next(b, None)
    return izip(a, b)

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
	
	#Calculates and prints relative distances between points
	#Only works when path continues to the right and up...
	newXlist = [i ** 2 for i in route.pathX]
	newYlist = [j ** 2 for j in route.pathY]
	hypotenuselist = map(math.sqrt, map(add, newXlist, newYlist))
	print "original x list: ",route.pathX
	print "original y list: ",route.pathY
	print "squared x list: ",newXlist
	print "squared y list: ",newYlist
	print "Relative distances: ",map(lambda(a, b):abs(b - a), pairwise(hypotenuselist))
	#print "Relative distances: ",[abs(y - x) for x,y in zip(hypotenuselist,hypotenuselist[1:])] #same as above
	
	#Plots actual route with solid blue line
	plt.plot(xRoute, yRoute, '-o')
	
	#Plots fastest route with dashed green line
	plt.plot(route.pathX, route.pathY, 'go--')
	
	#Draws graphics with scaled axes
	plt.axis([(min(route.pathX)-2), (max(route.pathX)+2), (min(route.pathX)-2), (max(route.pathY)+2)])
	
	# calculate polynomial
	z = np.polyfit(route.pathX, route.pathY, 3)
	f = np.poly1d(z)

	# calculate new x's and y's
	x_new = np.linspace(route.pathX[0], route.pathX[-1], 50)
	y_new = f(x_new)
	
	#Draws a smooth path, more natural movement of vehicle. Could be used as a baseline
	plt.plot(x_new, y_new, 'r')
	plt.xlim([route.pathX[0]-1, route.pathX[-1] + 1 ])
	plt.show()
			
main()

#TODO: Eventually figure out how to calculate proper direction (tangent(y/x))
#TODO: Will need to add new parameter "facing" (in degrees)		
		
	
	
		
