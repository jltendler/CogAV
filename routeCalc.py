import numpy as np
import matplotlib.pyplot as plt
from operator import *
import math
from itertools import izip, tee

class routeCalc:
	def __init__(self, curX, curY):
		self.curX = curX
		self.curY = curY

	pathX = [0, -2, -4, 5, 6]
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
	#Check this.... might not work with negatives
	newXlist = [i ** 2 for i in route.pathX]
	newYlist = [j ** 2 for j in route.pathY]
	hypotenuselist = map(math.sqrt, map(add, newXlist, newYlist))
	print "Relative distances: ",map(lambda(a, b):b - a, pairwise(hypotenuselist))
	
	#Plots actual route with solid blue line
	plt.plot(xRoute, yRoute, '-o')
	
	#Plots fastest route with dashed green line
	plt.plot(route.pathX, route.pathY, 'go--')
	
	#Draws graphics with scaled axes
	plt.axis([(min(route.pathX)-2), (max(route.pathX)+2), (min(route.pathX)-2), (max(route.pathY)+2)])
	plt.show()
			
main()

#TODO: Eventually figure out how to calculate proper direction (tangent(y/x))
#TODO: Will need to add new parameter "facing" (in degrees)		
		
	
	
		
