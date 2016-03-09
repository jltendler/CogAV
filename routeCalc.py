import numpy as np
import matplotlib.pyplot as plt
from operator import *
import math
import decimal

class routeCalc:
    def __init__(self, curX, curY):
        self.curX = decimal.Decimal(curX)
        self.curY = decimal.Decimal(curY)

    pathX = np.array([40.071374, 40.071258, 40.070755, 40.070976, 40.071331])
    pathY = np.array([-105.229788, -105.230026, -105.229717, -105.229191, -105.229466])
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
        return decimal.Decimal(lines[lineNumber]) # Index starts at 0

        
    """
    MOVE FUNCTION
    def move(self, angle, distance):
		if heading != angle:
			while heading > 0:
				set PWM(0,0,[375 to 600])
				this turns right
			while heading < 0:
				set PWM(0,0,[150 to 375])
				this turns left
		else:
			set PWM(0,0,375) go straight
	"""
		
    increment = decimal.Decimal(0.0001) #Increment distance
    threshold = decimal.Decimal(0.0001) #threshold to check if within certain distance

    def moveNorth(self):
        print "moving north"
        self.curY += self.increment

    def moveSouth(self):
        print "moving south"
        self.curY -= self.increment

    def moveEast(self):
        print "moving east"
        self.curX += self.increment

    def moveWest(self):
        print "moving west"
        self.curX -= self.increment

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
    route.waypointCount = 0

    while not (route.checkIfFinished()):	#Not done
        if (route.checkIfAtPair(i)):  #At current goal
            i += 1
            while route.pathX[i] - route.curX > 0.0:
                print (route.curX, route.curY)
                route.moveEast()
                xRoute.append(route.curX)
                yRoute.append(route.curY)

            while route.pathX[i] - route.curX < 0.0:
                print (route.curX, route.curY)
                route.moveWest()
                xRoute.append(route.curX)
                yRoute.append(route.curY)

            while route.pathY[i] - route.curY > 0.0:
                print (route.curX, route.curY)
                route.moveNorth()
                xRoute.append(route.curX)
                yRoute.append(route.curY)

            while route.pathY[i] - route.curY < 0.0:
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



main()

#TODO: Eventually figure out how to calculate proper direction (tangent(y/x))
#TODO: Will need to add new parameter "facing" (in degrees)


