import random
import decimal

decimal.getcontext().prec = 9

latPath = [40.071374, 40.071258, 40.070755, 40.070976, 40.071331]
lonPath = [-105.229788, -105.230026, -105.229717, -105.229191, -105.229466]

for x in range(len(latPath)):
	latPath[x] = decimal.Decimal(latPath[x])
	latPath[x]+=0

for x in range(len(lonPath)):
	lonPath[x] = decimal.Decimal(lonPath[x])
	lonPath[x]+=0
	

"""
This will generate random points around each position as car moves from start of path
to the end of path in a step wise motion.
TODO: use the outputed text file: coordinates.txt to read into routeCalc.py
and figure out how to make it useful in our route calculation
TODO: fit random GPS generation to a poly-fit curve path or fastest route
"""
def gen_random_waypoints_onPath():
	coordinate_list = []
	curLat = latPath[0]
	curLon = lonPath[0]

		
	i = 0
	while not ((curLat == latPath[len(latPath) - 1]) and (curLon == lonPath[len(lonPath) - 1])): #Not at the end
		if ((curLat == latPath[i]) and (curLon == lonPath[i])): #At point in path
			print "Hit point in path"
			i += 1
			increment = decimal.Decimal(.000001)
			error = decimal.Decimal(random.randrange(10))/1000000 
			while latPath[i] - curLat > 0.0:
				curLat +=increment
				randLat = curLat + error #add random float to current position to account for GPS error
				coordString = str((randLat, curLon))
				coordString.split("'")
				coordinate_list.append((str((randLat, curLon))))

			while latPath[i] - curLat < 0.0:
				curLat -= increment
				randLat = curLat + error #add random float to current position to account for GPS error
				coordinate_list.append((str((randLat, curLon))))
				
			while lonPath[i] - curLon > 0.0:
				curLon += increment
				randLon = curLon + error #add random float to current position to account for GPS error
				coordinate_list.append((str((curLat, randLon))))
				
			while lonPath[i] - curLon < 0.0:
				curLon -= increment
				randLon = curLon + error #add random float to current position to account for GPS error
				coordinate_list.append((str((curLat, randLon))))

	return coordinate_list

def main():
	c = gen_random_waypoints_onPath()
	final_list = []
	for x in c:
		lat = x[10:19] # Hard coded, could make more abstract
		lon = x[33:43] # Hard coded, could make more abstract
		#final_list.append((lat, lon))
		final_list.append(lat +","+lon)
	coordinate_file = open("coordinates.txt", 'w')
	for i in range(len(final_list)):
		coordinate_file.write(str(final_list[i]) + '\n')
	coordinate_file.close()
main()