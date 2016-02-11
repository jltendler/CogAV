import random

latPath = [50, 55, 60, 65, 70, 65, 50, 45, 40]
lonPath = [40, 45, 50, 55, 60, 55, 50, 35, 30]
"""
This will generate random points around each position as car moves from start of path
to the end of path in a step wise motion.
TODO: use the outputed text file: coordinates.txt to read into routeCalc.py
and figure out how to make it useful in our route calculation
TODO: fit random GPS generation to a poly-fit curve path or fastest route
"""
def gen_random_waypoints_onPath():
	lattitude_list = []
	longitude_list = []
	curLat = latPath[0]
	curLon = lonPath[0]

	i = 0
	while not ((curLat == latPath[len(latPath) - 1]) and (curLon == lonPath[len(lonPath) - 1])): # Not at the end
		if ((curLat == latPath[i]) and (curLon == lonPath[i])): # At point in path
			i += 1
			while latPath[i] - curLat > 0:
				curLat += 1
				randLat = float(curLat) + random.uniform(0, 10) # add random float between 0 and 10 to current position
				lattitude_list.append(randLat)
			
			while latPath[i] - curLat < 0:
				curLat -= 1
				randLat = float(curLat) + random.uniform(0, 10) # add random float between 0 and 10 to current position
				lattitude_list.append(randLat)
			
			while lonPath[i] - curLon > 0:
				curLon += 1
				randLon = float(curLon) + random.uniform(0, 10) # add random float between 0 and 10 to current position
				longitude_list.append(randLon)
			
			while lonPath[i] - curLon < 0:
				curLon -= 1
				randLon = float(curLon) + random.uniform(0, 10) # add random float between 0 and 10 to current position
				longitude_list.append(randLon)
	
	return lattitude_list, longitude_list

def main():
	(lat, lon) = gen_random_waypoints_onPath()
	coordinate_file = open("coordinates.txt", 'w')
	for x in range(len(lon)):
		coordinate_file.write(str((lat[x], lon[x])) + '\n')
	coordinate_file.close()

main()
