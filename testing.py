# Testing Document 

# Tests are going to require dummy/real GPS data/direction to test against our simulator.

def testWaypoint(self):
        self.waypointCount += 1
        if self.waypointCount == len(pathX):
            print "PASS - All waypoints reached"
        else:
			print "FAIL - Missed waypoint(s)"
        
            
def currentLocVsAbsolute(self):
	if curX == gpsLat and curY == gpsLong:
		print "PASS - Current location matches current GPS"
	else:
		print "FAIL - Vehicle off course"
		
def testDirection(self):
	angle = np.arctan2(self.pathY[i]-self.pathY[i-1], self.pathX[i]-self.pathX[i-1])
	if angle == angleFromGPS:
		print "PASS - Car is moving in correct direction" #tests alignment of vehicle
	else:
		print "FAIL - Apply steering to correct alignment"
