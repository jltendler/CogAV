import numpy as np
#import matplotlib.pyplot as plt
from operator import *
import math
import decimal
import time
from Adafruit_PWM_Servo_Driver import PWM
import sys
import Servo_Example

class routeCalc:
    def __init__(self, curX, curY):
        self.waypointCounter = 0 
        self.curX = decimal.Decimal(curX)
        self.curY = decimal.Decimal(curY)
        self.pwm = PWM(0x40)
	
    pathX = np.array([40.071374, 40.071258, 40.070755, 40.070976, 40.071331])
    pathY = np.array([-105.229788, -105.230026, -105.229717, -105.229191, -105.22946])
    pathX=np.array([decimal.Decimal(abc) for abc in pathX])
    pathY=np.array([decimal.Decimal(abc) for abc in pathY])
    for x in range(len(pathX)):
        pathX[x] = decimal.Decimal(pathX[x])
        pathX[x]+=0

    for x in range(len(pathY)):
        pathY[x] = decimal.Decimal(pathY[x])
        pathY[x]+=0


    def testWaypoint(self):
        self.waypointCount += 1
        if self.waypointCount == 4:
            print "PASS - All waypoints reached"

            

    # Method to be called in move function to read in coordinate from coordinates.txt
    # Used to simulate live GPS data stream
    def read_Coordinate(self, lineNumber):
        fp = open("coordinates.txt")
        lines = fp.readlines()
        temp= tuple(map(decimal.Decimal, lines[lineNumber].split(',')))
        return temp # Index starts at 0

		
    increment = decimal.Decimal(0.0001) #Increment distance
    threshold = decimal.Decimal(0.00001) #threshold to check if within certain distance

    def findAngle(self):
		#Needs to compare pathX/pathY to curX/curY
        #Calculating the arctan of path
        arctanList = []
        for i in range(len(self.pathX)):
            if i is 0:
                y = np.arctan2(float(self.pathY[i]), float(self.pathX[i]))
                if y < 0:
					arctanList.append((2*np.pi)+y)
                else:
                    arctanList.append(y)
            else:
                x1=self.pathY[i]-self.pathY[i-1]
                x2=self.pathX[i]-self.pathX[i-1]
                x = np.arctan2(float(x1), float(x2))
                if x < 0:
                    arctanList.append((2*np.pi)+(x))
                else:
                    arctanList.append(x)

        arctanList = np.degrees(arctanList)
        return arctanList

    def findWayPoint(self):
        if self.checkIfFinished():
            print ("MISSION FUCKING ACCOMPLISHED BOYS")
            return self.waypointCounter
        if self.checkIfAtPair(self.waypointCounter):
            self.waypointCounter += 1
        return self.waypointCounter

    def findCurrAngle(self):
        self.waypointCounter = self.findWayPoint() 
        x1 = self.pathX[self.waypointCounter] - self.curX
        y1 = self.pathY[self.waypointCounter] - self.curY
        a = np.arctan2(float(y1), float(x1))
        if a < 0:
            a = 2.0*np.pi + a
        return np.degrees(a)

    def findCurrDistance(self):
        self.waypointCounter = self.findWayPoint() # Bad!! call to findWayPoint()
        x1 = self.pathX[self.waypointCounter] - self.curX
        y1 = self.pathY[self.waypointCounter] - self.curY
        dist = np.sqrt(abs(np.square(x1) + np.square(y1))) #may have to cast as float here
        return dist

    def findAbsAngle(self):
        #Calculating the absolute arctans of path
        absArctanList = []
        print ("Finding abs angle")
        for i in range(len(self.pathX)):
                absArctanList.append(np.arctan2(float(self.pathY[i]), float(self.pathX[i])))

        absArctanList = np.degrees(absArctanList)
        return absArctanList

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
        
    def calcTurn(self):
        angle = self.findCurrAngle()
        heading = 29.0 #will have to implement real heading later
        diff = angle - heading
        turnPWM = -2.0*diff + 375.0
        if turnPWM < 150:
            turnPWM = 150 #logical minmum for turning
        if turnPWM > 600:
            turnPWM = 600 #logical maximum for turning
        #150-fullLeft
        #375-straight
        #600-fullRight
        return turnPWM

    def calcSpeed(self):
        angle = self.findCurrAngle()
        heading = 79
        diff = angle - heading
        speedPWM = 600 - 2*abs(diff)
        if speedPWM <= 375:
            speedPWM = 400
        return speedPWM

    def updateLocation(self):
        i = 0
        while i < 2500:
            (self.curX, self.curY) = self.read_Coordinate(i)
            i += 1
            print ("curX:",self.curX," curY:", self.curY)

    def move(self):
        i = 0
        while i < 2543:
            (self.curX, self.curY) = self.read_Coordinate(i)
            angle = self.findCurrAngle()
            heading = 35
            turnPWM = self.calcTurn()
            speedPWM = self.calcSpeed()
            print (i)
            print ("curX:", self.curX)
            print ("curY:", self.curY)
            print ("angle is equal to:", angle) 
            print ("setting turn PWM to:", int(turnPWM)) # Also in calcTurn()
            print ("setting speed PWM to:", int(speedPWM)) # Also in calcSpeed()
            self.pwm.setPWM(0,0,int(turnPWM)) #--> moved this call into calcTurn()
            self.pwm.setPWM(0,1,int(speedPWM)) #--> moved this call into calcSpeed()
            i += 1
        
    def checkIfFinished(self):
        #Checks if current coordinates are within threshold of the specified end of path
        return (self.checkIfInThreshold(self.curX, self.pathX[len(self.pathX)-1]) and self.checkIfInThreshold(self.curY, self.pathY[len(self.pathY)-1]))
        
    def checkIfAtPair(self, i):
        #Checks if current coordinates are within threshold of specified point in path
        return (self.checkIfInThreshold(self.curX,self.pathX[i]) and self.checkIfInThreshold(self.curY,self.pathY[i]))
        
        
    def checkIfInThreshold(self, a, b):
        if(abs(decimal.Decimal(b) - decimal.Decimal(a)) < self.threshold):
            return True
        else:
            return False

def main():
    route = routeCalc(decimal.Decimal(40.071374),decimal.Decimal(-105.229788))
    i = 0
    xRoute = []
    yRoute = []

    while not (route.checkIfFinished()): #Not Done
        route.move()
'''
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
    ax.set_rmax(112.602)
    ax.set_rmin(112.600)
    ax.grid(True)
    ax.set_title("Simulating vehicle movement", va='bottom')
    plt.show()
    '''



main()

#TODO: Eventually figure out how to calculate proper direction (tangent(y/x))
#TODO: Will need to add new parameter "facing" (in degrees)


