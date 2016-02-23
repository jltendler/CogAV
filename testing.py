# Testing Document 

# Tests are going to require dummy/real GPS data/direction to test against our simulator.
import routeCalc
import numpy as np
def testWaypoint(self,route):
        self.waypointCount += 1
        if self.waypointCount == len(route.pathX):
            print "PASS - All waypoints reached"
        else:
			print "FAIL - Missed waypoint(s)"
        
            
def currentLocVsAbsolute(self, route):
	if route.curX == gpsLat and route.curY == gpsLong:
		print "PASS - Current location matches current GPS"
	else:
		print "FAIL - Vehicle off course"
		
def testDirection(self,route,i):
	route.angle = np.arctan2(self.pathY[i]-self.pathY[i-1], self.pathX[i]-self.pathX[i-1])
	if route.angle == angleFromGPS:
		print "PASS - Car is moving in correct direction" #tests alignment of vehicle
	else:
		print "FAIL - Apply steering to correct alignment"

def testRouteHasPath(self,route):
    if (self.len(route.pathx)==0 or self.len(route.pathy)==0):
        print "FAIL - Route is Empty"
    else:
        print "PASS - Route has Path"
