import numpy as np
import matplotlib.pyplot as plt
from operator import *
import math

class routeCalc:
    def __init__(self, curX, curY):
        self.curX = curX
        self.curY = curY

    pathX = [0, 2, 4, 6, 4]
    pathY = [0, 2, 6, 2, 0]

    def testWaypoint(self):
        self.waypointCount += 1
        if self.waypointCount == 4:
            print "PASS - All waypoints reached"

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

    def findAngle(self):
        #Calculating the arctan of path
        arctanList = []
        for i in range(len(self.pathX)):
            if i is 0:
                arctanList.append(np.arctan2(self.pathY[i], self.pathX[i]))
            else:
                arctanList.append(np.arctan2(self.pathY[i]-self.pathY[i-1], self.pathX[i]-self.pathX[i-1]))

        arctanList = np.degrees(arctanList)
        return arctanList

    def findAbsAngle(self):
        #Calculating the absolute arctans of path
        absArctanList = []
        for i in range(len(self.pathX)):
                absArctanList.append(np.arctan2(self.pathY[i], self.pathX[i]))

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
    def checkIfFinished(self):
        return ((self.curX == self.pathX[len(self.pathX)-1]) and (self.curY == self.pathY[len(self.pathY)-1]))
    def checkIfAtPair(self, i):
        #Checks if current coordinates are at the specified pathX/pathY coordinates
        return ((self.curX == self.pathX[i]) and (self.curY == self.pathY[i]))

def main():
    i = 0
    xRoute = []
    yRoute = []
    route = routeCalc(0,0)
    route.waypointCount = 0

    while not (route.checkIfFinished()):	#Not done
        if (route.checkIfAtPair(i)):  #At current goal
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
                route.testWaypoint()

    #Calculates and prints relative distances between points
    xSquaredList = [i ** 2 for i in route.pathX]
    ySquaredList = [j ** 2 for j in route.pathY]
    hypotenuselist = map(math.sqrt, map(add, xSquaredList, ySquaredList))
    print "original x list: ",route.pathX
    print "original y list: ",route.pathY
    print "squared x list: ",xSquaredList
    print "squared y list: ",ySquaredList
    print "Relative distances: ",route.findDistance()
    print "Absolute distances (from origin): ",route.findAbsDistance()
    print "Relative angles: ",route.findAngle()
    print "Absolute angles: ",route.findAbsAngle()

    #Plots the points on Polar Plot. Note, must use absolute angles and distances
    #TODO: Take in lists from those functions

    lengths = np.array([0, 45, 56, 18, 0]) #looks to be absolute angles
    angles = np.array([0, 2.828, 7.211, 6.324, 4]) #looks to be absolute distances. Why are these hard coded? And named wrong?

    ax = plt.subplot(111, projection='polar')
    ax.plot(lengths * np.pi / 180, angles, 'r', linewidth=3)
    ax.grid(True)
    ax.set_title("Simulating vehicle movement", va='bottom')
    plt.show()



main()

#TODO: Eventually figure out how to calculate proper direction (tangent(y/x))
#TODO: Will need to add new parameter "facing" (in degrees)

