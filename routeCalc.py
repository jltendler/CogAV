import numpy as np
from operator import *
import math
import decimal
import time
from Adafruit_PWM_Servo_Driver import PWM
import sys
import os
from gps import *
import threading
from Adafruit_BNO055 import BNO055
import RPi.GPIO as GPIO

gpsd = None
#Initialize serial port for Compass
bno = BNO055.BNO055(serial_port='/dev/ttyUSB0', rst=18)

leftSensor = None
rightSensor = None


class rangeFinder(threading.Thread):
  def __init__(self):
    threading.Thread.__init__(self) #create separate thread

  def reading(self, sensor):
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    if sensor == 0: # left sensor
      # point the software to the GPIO pins the sensor is using
      # change these values to the pins you are using
      # GPIO output = the pin that's connected to "Trig" on the sensor
      # GPIO input = the pin that's connected to "Echo" on the sensor

      #left sensor
      GPIO.setup(21,GPIO.OUT)
      GPIO.setup(20,GPIO.IN)
      GPIO.output(21, GPIO.LOW)

      #add delay so it won't crash
      #time.sleep(0.3)

      # start the pulse on the GPIO pin 
      # change this value to the pin you are using
      # GPIO output = the pin that's connected to "Trig" on the sensor
      GPIO.output(21, True) #left sensor

      # wait 10 micro seconds (this is 0.00001 seconds) so the pulse
      # length is 10Us as the sensor expects
      time.sleep(0.00001)

      GPIO.output(21, False) #left sensor
      while GPIO.input(20) == 0: #left sensor
        self.signaloff = time.time()

      while GPIO.input(20) == 1:
        self.signalon = time.time()
      
      self.timepassed = self.signalon - self.signaloff
      self.distance = self.timepassed * 21000

      #return self.distance
      #GPIO.cleanup()
      global leftSensor
      leftSensor = self.distance

    if sensor == 1: # right sensor
       #right sensor
      #21 = 19, 20 = 16
      GPIO.setup(19,GPIO.OUT)
      GPIO.setup(16,GPIO.IN)
      GPIO.output(19, GPIO.LOW)

      #add delay so it won't crash
      time.sleep(0.1)

      GPIO.output(19, True) #right sensor
      # wait 10 micro seconds (this is 0.00001 seconds) so the pulse
      # length is 10Us as the sensor expects
      time.sleep(0.00001)
      GPIO.output(19, False) #right sensor

      while GPIO.input(16) == 0: #right sensor
        self.signaloff_R = time.time()

      while GPIO.input(16) == 1:
        self.signalon_R = time.time()
      self.timepassed_R = self.signalon_R - self.signaloff_R
      self.distance_R = self.timepassed_R * 21000
      global rightSensor
      rightSensor = self.distance_R
      

      
      

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
        
        
        
	
    #pathY = np.array([40.007300, 40.007277, 40.007214, 40.00712, 40.00715, 40.007114, 40.007267, 40.007303, 40.007314, 40.007346, 40.007349, 40.007311, 40.007298, 40.007286, 40.007514, 40.007901, 40.008169, 40.008325, 40.008376, 40.008453])
    #pathX = np.array([-105.264190, -105.264066, -105.264030, -105.26408, -105.26416, -105.264197, -105.264413, -105.264790, -105.264934, -105.265380, -105.266120, -105.266362, -105.266699, -105.267039, -105.267041, -105.267031, -105.267021, -105.267046, -105.267378, -105.267720])

    #outside kobel field
    pathY = np.array([40.006003, 40.006008])
    pathX = np.array([-105.264075, -105.264346])
    #Business field far from sidewalk
    #pathY = np.array([40.005397, 40.005392, 40.005230, 40.005262])
    #pathX = np.array([-105.262467, -105.262114, -105.262173, -105.262445])

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
    threshold = decimal.Decimal(0.00002) #threshold to check if within certain distance

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
            print ("Final Waypoint Reached!!")
            return self.waypointCounter
        if self.checkIfAtPair(self.waypointCounter):
            self.waypointCounter += 1
        return self.waypointCounter

    def findCurrAngle(self):
        self.waypointCounter = self.findWayPoint() 
        x1 = self.pathX[self.waypointCounter] - self.curX
        y1 = self.pathY[self.waypointCounter] - self.curY
        a = np.arctan2(float(y1), float(x1))
        a = np.degrees(a)
        if a <= 90:
          compass_angle = 90 - a
          #print "arctan < 90"
        else:
          compass_angle = 360 - (a - 90)
          #print "arctan > 90"
          
        #print ("angle of direction=", compass_angle)
        temp = (360 - self.pullHeading()) + compass_angle
        temp = temp % 360
        temp2 = temp - 360
        if min(temp, abs(temp2)) == abs(temp2):
          return temp2
        else:
          return temp

    def findCurrDistance(self):
        self.waypointCounter = self.findWayPoint() 
        x1 = self.pathX[self.waypointCounter] - self.curX
        y1 = self.pathY[self.waypointCounter] - self.curY
        dist = np.sqrt(abs(np.square(x1) + np.square(y1))) 
        return dist

    def findAbsAngle(self):
        #Calculating the absolute arctans of path
        absArctanList = []
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
        return c_pass.heading
        
    def calcTurn(self):
        angle = self.findCurrAngle()
        turnPWM = -1.75*angle + 375.0
        if turnPWM < 150:
            turnPWM = 150 #logical minmum for turning
        if turnPWM > 600:
            turnPWM = 600 #logical maximum for turning
        #150-fullRight
        #375-straight
        #600-fullLeft
        return turnPWM

    def calcSpeed(self):
        angle = self.findCurrAngle()
        calibration = 4.0
        speedPWM = 450 - calibration*abs(angle)
        if speedPWM <= 375:
            speedPWM = 420
        if speedPWM > 575:
            speedPWM = 574
        # set to constant speed for testing
        return 405 #speedPWM


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

    def avoid2Left(self):
        self.pwm.setPWM(1,0,int(395)) 
        self.pwm.setPWM(0,0,int(600))

    def avoid2Right(self):
        self.pwm.setPWM(1,0,int(395))
        self.pwm.setPWM(0,0,int(150))

    def avoidObstacle(self, left, right):
        l = left
        r = right
        #chooses to turn in direction of largest sensor distance
        if (l > r):
          self.avoid2Left()
          new_left = leftSensor
          if ((new_left / r) < (l / r)):
            self.avoid2Right()
        elif (r > l):
          self.avoid2Right()
          new_right = rightSensor
          if ((new_right / l) < (r / l)):
            self.avoid2Left()
        
    def checkBetweenTwo(self,x,up,low):
      if (x>low and x<up):
        return True
      else:
        return False
    def move(self):
        Uthreshold = 150.0
        Lthreshold = 10.0

        if (self.checkBetweenTwo(leftSensor,Uthreshold,Lthreshold) or self.checkBetweenTwo(rightSensor,Uthreshold,Lthreshold) ):
          self.avoidObstacle(leftSensor, rightSensor)
          findRange.reading(0)
          findRange.reading(1)
          print("AvoidLeft: "+ str(leftSensor) +" AvoidRight: "+str(rightSensor))

        else:
          self.curY = decimal.Decimal(self.pullLat())
          self.curX = decimal.Decimal(self.pullLong())
          angle = self.findCurrAngle()
          turnPWM = self.calcTurn()
          speedPWM = self.calcSpeed()
          #print ("curX:", self.curX)
          #print ("curY:", self.curY)
          #print ("angle is equal to:", angle) 
          #print ("setting turn PWM to:", int(turnPWM)) # Also in calcTurn()
          #print ("setting speed PWM to:", int(speedPWM)) # Also in calcSpeed()
          self.pwm.setPWM(0,0,int(turnPWM)) #--> moved this call into calcTurn()
          self.pwm.setPWM(1,0,int(speedPWM)) #--> moved this call into calcSpeed()
          #print("current goal:" , (self.pathX[self.waypointCounter], self.pathY[self.waypointCounter]))
          #print ("heading = ", c_pass.heading)
          # make sure sleep commented out when running
          # can turn back on to see data
          findRange.reading(0)
          findRange.reading(1)
          print("Left: "+ str(leftSensor) +" Right: "+str(rightSensor))
          # time.sleep(1)
        
        
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
    route = routeCalc(decimal.Decimal(0.0),decimal.Decimal(0.0))
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
    

if __name__ == '__main__':
    gpsp = GpsPoller()
    gpsp.start()
    c_pass = compass()
    findRange = rangeFinder()

main()

