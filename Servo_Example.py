#!/usr/bin/python

from Adafruit_PWM_Servo_Driver import PWM
import time
import sys
# ===========================================================================
# Example Code
# ===========================================================================

# Initialise the PWM device using the default address
pwm = PWM(0x40)
# Note if you'd like more debug output you can instead run:
#pwm = PWM(0x40, debug=True)

servoMin = 150  # Min pulse length out of 4096
servoMax = 600  # Max pulse length out of 4096
servoCenter=375

def setServoPulse(channel, pulse):
  pulseLength = 1000000                   # 1,000,000 us per second
  pulseLength /= 60                       # 60 Hz
  print "%d us per period" % pulseLength
  pulseLength /= 4096                     # 12 bits of resolution
  print "%d us per bit" % pulseLength
  pulse *= 1000
  pulse /= pulseLength
  pwm.setPWM(channel, 0, pulse)

pwm.setPWMFreq(60)                        # Set frequency to 60 Hz



pwm.setPWM(1,0,servoCenter)
if (True):
  print "Running Initialization"
  pwm.setPWM(1, 0, servoMax)
  time.sleep(.1)
  pwm.setPWM(1, 0, servoMin)
  time.sleep(.1)
  pwm.setPWM(1, 0, servoCenter)
  time.sleep(.5)
  
turn=150
def turnRight():
  print "Turning Right"
  pwm.setPWM(0,0,servoMin)

def turnLeft():
  print "Turning Left"
  pwm.setPWM(0,0,servoMax)

def turnForward():
  print "Turning Straight"
  pwm.setPWM(0,0,servoCenter)

def whatever(potato):
  pwm.setPWM(0,0,potato)
  print(potato)

i=505
#290 is most right turned.
#505 is most left turned

while (True):
  whatever(290)
  time.sleep(1)
  whatever(505)
  time.sleep(1)
  whatever(100)
  time.sleep(1)
  whatever(600)
  time.sleep(1)
  i-=5

'''
    print "Forward"
    turnLeft()
    pwm.setPWM(1,0,servoCenter+25)
    time.sleep(1)
  
    print "still"
    turnForward()
    pwm.setPWM(1,0,servoCenter)
    time.sleep(1)

    print "backwards"
    turnRight()
    pwm.setPWM(1,0,servoCenter-20)
    time.sleep(.1)
    pwm.setPWM(1,0,servoCenter)
    time.sleep(.1)
    pwm.setPWM(1,0,servoCenter-20)
    time.sleep(1)
    
    print "still 2"
    turnForward()
    pwm.setPWM(1,0,servoCenter)
    time.sleep(1)

    print "takeoff"
    pwm.setPWM(1,0,servoCenter+204) #579 is actual max speed.
    time.sleep(5)

    print "still 3"
    pwm.setPWM(1,0,servoCenter)
    time.sleep(2)
  

except KeyboardInterrupt:
    print("Interrupted by user.")
    pwm.setPWM(1,0,servoCenter)
    pwm.setPWM(0,0,servoCenter)
    sys.exit(0)
'''
