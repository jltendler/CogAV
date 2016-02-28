import numpy as np
import matplotlib.pyplot as plt
from operator import *
import math
from Adafruit_PWM_Servo_Driver import PWM
import time
import random
import decimal

class routeCalc:
    def __init__(self, curX, curY):
        self.curX = curX #gpsd.fix.latitude
        self.curY = curY
        
	waypointCounter = 0 
	threshold = decimal.Decimal(0.000001)
	decimal.getcontext().prec = 9

	pathX = [40.071374, 40.071258, 40.070755, 40.070976, 40.071331]
	pathY = [-105.229788, -105.230026, -105.229717, -105.229191, -105.229466]

	for x in range(len(pathX)):
		pathX[x] = decimal.Decimal(pathX[x])
		pathX[x]+=0

	for x in range(len(pathY)):
		pathY[x] = decimal.Decimal(pathY[x])
		pathY[x]+=0
		
	# Method to be called in move function to read in coordinate from coordinates.txt
    # Used to simulate live GPS data stream
    def read_Coordinate(self, lineNumber):
        fp = open("coordinates.txt")
        lines = fp.readlines()
        return lines[lineNumber] # Index starts at 0

            
    def calcTurn(self):
		
		angle = findCurrAngle()
		heading = gpsd.fix.heading
		diff = angle - heading #positive-turn left negative-turn right
		
		valuePWM = -2*diff + 375
		
		if valuePWM < 150:
			valuePWM = 150
		if valuePWM > 600:    #logical maximum and minimum limits for turning
			valuePWM = 600
		
		return valuePWM #150-FullLeft 375-Straight 600-FullRight
		
    def calcSpeed(self):
		angle = findCurrAngle()
		heading = gpsd.fix.heading
		
		diff = angle - heading
		
		speedPWM = 600 - 2*abs(diff)
		
		if speedPWM <= 375:
			speedPWM = 400
		
		return speedPWM		
		  
    def move(self):
		angle = findCurrAngle()
		heading = gpsd.fix.heading
		
		while heading != angle: #add threshold
			valuePWM = calcTurn()
			speedPWM = calcSpeed()
			pwm.setPWM(0,0,valuePWM)
			pwm.setPWM(0,1,speedPWM)
			
    def findWaypoint(self):		
		distance = findCurrDistance()		
		
		if distance < threshold:
			waypointCounter+=1
			
		return waypointCounter
		
    def findCurrAngle(self):
		waypointCounter = findWaypoint()
				
		x = np.arctan2(self.pathY[waypointCounter] - self.curY, self.pathX[waypointCounter] - self.curX)
		if x < 0:
			x = 2*np.pi + x
		
		print np.degrees(x)	
		return np.degrees(x)

    def findAngle(self):
		#Needs to compare pathX/pathY to curX/curY
        #Calculating the arctan of path
        arctanList = []
        for i in range(len(self.pathX)):
            if i is 0:
                y = np.arctan2(self.pathY[i], self.pathX[i])
                if y < 0:
					acctanList.append((2*np.pi)+y)
                else:
                    arctanList.append(y)
            else:
				x = np.arctan2(self.pathY[i]-self.pathY[i-1], self.pathX[i]-self.pathX[i-1])
				if x < 0:
					arctanList.append((2*np.pi)+(x))
				else:
					arctanList.append(x)

        arctanList = np.degrees(arctanList)
        return arctanList

    def findAbsAngle(self):
        #Calculating the absolute arctans of path
        absArctanList = []
        for i in range(len(self.pathX)):
                absArctanList.append(np.arctan2(self.pathY[i], self.pathX[i]))

        absArctanList = np.degrees(absArctanList)
        return absArctanList
	
	def findCurrDistance(self):
		waypointCounter = findWaypoint()
		
		x = np.sqrt(abs(np.square(self.pathX[waypointCounter] - self.curX) + np.square(self.pathY[waypointCounter] - self.curY)))
		
		return x
	
    def findDistance(self):
        #Calculating the distance between waypoints
        distList = []
        for i in range(len(self.pathX)):
            if i is 0:
                distList.append(np.sqrt(abs(np.square(self.pathX[i]) + np.square(self.pathY[i]))))
            else:
                distList.append(np.sqrt(abs(np.square(self.pathX[i] - self.pathX[i-1]) + np.square(self.pathY[i] - self.pathY[i-1]))))
        return distList

    def findAbsDistance(self):
        #Calculating the distance of each point from origin
        absDistList = []
        for i in range(len(self.pathX)):
                absDistList.append(np.sqrt(abs(np.square(self.pathX[i]) + np.square(self.pathY[i]))))
        return absDistList
        
    def checkIfFinished(self):
        return (checkIfInThreshold(self.curX, self.pathX[len(self.pathX)-1]) and checkIfInThreshold(self.curY, self.pathY[len(self.pathY)-1]))
        
    def checkIfAtPair(self, i):
        #Checks if current coordinates are at the specified pathX/pathY coordinates
        return (checkIfInThreshold(self.curX,self.pathX[i]) and checkIfInThreshold(self.curY,self.pathY[i]))
        
        
	def checkIfInThreshold(self, a, b):
		if(abs(b - a) < self.threshold):
			return True
		else:
			return False

def main():
    i = 0
    xRoute = []
    yRoute = []
    route = routeCalc(0,0)

    while not (route.checkIfFinished()):#Not done
        route.move()
        #(route.curX, route.curY) = route.read_Coordinate(0) #may or may not work         
        
        #Cut out stuff here. look in git if necessary for our tree of while statements.
        
    #Calculates and prints relative distances between points
    xSquaredList = [i ** 2 for i in route.pathX]
    ySquaredList = [j ** 2 for j in route.pathY]
    hypotenuselist = map(math.sqrt, map(add, xSquaredList, ySquaredList))
    absAngles = route.findAbsAngle()
    absDistances = route.findAbsDistance()
    print "original x list: ",route.pathX
    print "original y list: ",route.pathY
    print "squared x list: ",xSquaredList
    print "squared y list: ",ySquaredList
    print "Relative distances: ",route.findDistance()
    print "Absolute distances (from origin): ",absDistances
    print "Relative angles: ",route.findAngle()
    print "Absolute angles: ",absAngles

    #Plots the points on Polar Plot. Note, must use absolute angles and distances
    #TODO: Take in lists from those functions

    angles = np.array(absAngles) 
    lengths = np.array(absDistances) 

    ax = plt.subplot(111, projection='polar')
    ax.plot(angles * np.pi / 180, lengths, 'r', linewidth=3)
    ax.grid(True)
    ax.set_title("Simulating vehicle movement", va='bottom')
    plt.show()



main()

#TODO: Eventually figure out how to calculate proper direction (tangent(y/x))
#TODO: Will need to add new parameter "facing" (in degrees)

