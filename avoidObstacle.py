def move(self):
  distance_threshold = 10
  left_sensor = self.getLeft()
  right_sensor = self.getRight()
  if(left_sensor || right_sensor <= distance_threshold):
    avoidObstacle(left_sensor, right_sensor)
  else:
    ... #rest of move
def avoid2Left(self):
  self.pwm.setPWM(1,0,int(415)) 
  self.pwm.setPWM(0,0,int(420))
  #may need to set to curTurning +/- some constant (if we are already turning and need to turn more)
def avoid2Right(self):
  self.pwm.setPWM(1,0,int(415))
  self.pwm.setPWM(0,0,int(325))
  
def avoidObstacle(self, left_sensor, right_sensor):
  left = left_sensor
  right = right_sensor
  #chooses to turn in direction of largest sensor distance output
  if(left > right):
    self.avoid2Left()
    new_left = self.getLeft()
    #if turn decision leads to different obstacle, (i.e. a corner) turn opposite direction
    if((new_left/right) < (left/right)):
      self.avoid2Right()
  
  elif(right > left):
    self.avoid2Right()
    new_right = self.getRight()
    if((new_right/left) < (right/left)):
      self.avoid2Left()
  