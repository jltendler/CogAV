import numpy as np
#import matplotlib.pyplot as plt
from operator import *
import math
import decimal
import time
from Adafruit_PWM_Servo_Driver import PWM
import sys
#import Servo_Example
import os
from gps import *
import threading
from Adafruit_BNO055 import BNO055


gpsd = None
#Initialize serial port for Compass
bno = BNO055.BNO055(serial_port='/dev/ttyUSB0', rst=18)


class GpsPoller(threading.Thread):
  def __init__(self):
    threading.Thread.__init__(self)
    global gpsd #bring it in scope
    gpsd = gps(mode=WATCH_ENABLE) #starting the stream of info
    self.current_value = None
    self.running = True #setting the thread running to true

  def run(self):
    global gpsd
    while gpsp.running:
      gpsd.next() #this will continue to loop and grab EACH set of gpsd info to clear the buffer


class compass(threading.Thread):
  def __init__(self):
    threading.Thread.__init__(self)
    #Initialize serial port for Compass
    self.bno = BNO055.BNO055(serial_port='/dev/ttyUSB0', rst=18)
    # Initialize the BNO055 and stop if something went wrong.
    if not bno.begin():
      raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')
    # Print system status and self test result.
    status, self_test, error = bno.get_system_status()
    print('System status: {0}'.format(status))
    print('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))
    # Print out an error if system status is in error mode.
    if status == 0x01:
        print('System error: {0}'.format(error))
        print('See datasheet section 4.3.59 for the meaning.')

    print('Reading BNO055 data, press Ctrl-C to quit...')
    # Read the Euler angles for heading, roll, pitch (all in degrees).

  def run(self):
    self.heading, pitch, roll = bno.read_euler()
    # Read the calibration status, 0=uncalibrated and 3=fully calibrated.
    sys, gyro, accel, mag = bno.get_calibration_status()
    # Print everything out.
    #print('Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}\tSys_cal={3} Gyro_cal={4} Accel_cal={5} Mag_cal={6}'.format(
    #      self.heading, roll, pitch, sys, gyro, accel, mag))
 

class routeCalc:
    def __init__(self, curX, curY):
        self.waypointCounter = 0 
        self.curX = decimal.Decimal(curX)
        self.curY = decimal.Decimal(curY)
        self.pwm = PWM(0x40)
        self.servoMin = 150  # Min pulse length out of 4096
        self.servoMax = 600  # Max pulse length out of 4096
        self.servoCenter=375
        self.pwm.setPWMFreq(60)
        self.lastHeading=0
        
        
        
	
    pathX = np.array([40.00602, 40.00591, 40.00585, 40.00595])
    pathY = np.array([-105.26239, -105.26247, -105.26231, -105.26224])
    
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
    threshold = decimal.Decimal(0.00003) #threshold to check if within certain distance

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
        print("!!!WAYPOINT!!! :", self.waypointCounter)
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

    def pullHeading(self):
        c_pass.run() #Update the heading angle every time this method is called
        print ("heading = ", c_pass.heading )
        return c_pass.heading
        #if math.isnan(gpsd.fix.track):
        #  print "No Fix"
        #  gpsd.fix.track = 0
        #return gpsd.fix.track
        
    def calcTurn(self):
        angle = self.findCurrAngle()
        heading = self.pullHeading() #will have to implement real heading later
        diff = heading - angle
        turnPWM = -1.0*diff + 375.0
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
        calibration = 5.0
        heading = self.pullHeading()
        diff = heading - angle
        speedPWM = 450 - calibration*abs(diff)
        if speedPWM <= 375:
            speedPWM = 400
        if speedPWM > 575:
            speedPWM = 574
        return speedPWM

    def updateLocation(self):
        i = 0
        while i < 2500:
            (self.curX, self.curY) = self.read_Coordinate(i)
            i += 1
            print ("curX:",self.curX," curY:", self.curY)

    def pullLong(self):
        if math.isnan(gpsd.fix.longitude):
          print "No Fix"
          gpsd.fix.longitude = 0
        return gpsd.fix.longitude

    def pullLat(self):
        if math.isnan(gpsd.fix.latitude):
          print "No Fix"
          gpsd.fix.latitude = 0
        return gpsd.fix.latitude
        

    def move(self):

        self.curX = decimal.Decimal(self.pullLat())
        self.curY = decimal.Decimal(self.pullLong())
        angle = self.findCurrAngle()
        turnPWM = self.calcTurn()
        speedPWM = self.calcSpeed()
        print ("curX:", self.curX)
        print ("curY:", self.curY)
        print ("angle is equal to:", angle) 
        print ("setting turn PWM to:", int(turnPWM)) # Also in calcTurn()
        print ("setting speed PWM to:", int(speedPWM)) # Also in calcSpeed()
        self.pwm.setPWM(0,0,int(turnPWM)) #--> moved this call into calcTurn()
        self.pwm.setPWM(1,0,int(speedPWM)) #--> moved this call into calcSpeed()
        time.sleep(.1)
        
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
    def initializeCar(self):
        print "Running Initialization"
        self.pwm.setPWM(1, 0, self.servoMax)
        time.sleep(.1)
        self.pwm.setPWM(1, 0, self.servoMin)
        time.sleep(.1)
        self.pwm.setPWM(1, 0, self.servoCenter)
        time.sleep(.5)
    def testTurn(self):
        print("hullo")
        self.pwm.setPWM(0,0,self.servoMax)
        time.sleep(.1)
        self.pwm.setPWM(0,0,self.servoMin)
        time.sleep(.1)
        self.pwm.setPWM(0,0,500)
        time.sleep(.1)
        self.pwm.setPWM(0,0,220)
    def stopCar(self):
        print("Interrupted by user.")
        self.pwm.setPWM(1,0,375)
        self.pwm.setPWM(0,0,375)
        sys.exit(0)
def main():
    route = routeCalc(decimal.Decimal(40.00720),decimal.Decimal(-105.26394))
    i = 0
    xRoute = []
    yRoute = []
    route.initializeCar()
    
    
    while not (route.checkIfFinished()): #Not Done
        try:
            
            route.move()
        except KeyboardInterrupt:
            print("Interrupted by user.")
            route.stopCar()
    route.stopCar()
    
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
if __name__ == '__main__':
    gpsp = GpsPoller()
    gpsp.start()
    c_pass = compass()

main()

#TODO: Eventually figure out how to calculate proper direction (tangent(y/x))
#TODO: Will need to add new parameter "facing" (in degrees)


