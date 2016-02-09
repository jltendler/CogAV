import random

start_Long = 50
end_Long = 50

start_Lat = 50
end_Lat = 50

"""
This script is a basic attempt at generating random GPS coordinates defined within a certain range
TODO: Mess with the numbers to make them generate more acurtely
TODO: Make it more abstract to be able to work with live coordinate list
"""
def gen_random_waypoints_long(low, high):
	longitude_list = []
	for i in range(100):
		while low > 0 and high <= 100:
			i = random.uniform(low, high) #Generates a random float imbetween range low and high
			longitude_list.append(i)
			low -= 1
			high +=1
	return longitude_list

def gen_random_waypoints_lat(low, high):
	lattitude_list = []
	for i in range(100):
		while low > 0 and high <= 100:
			i = random.uniform(low, high) #Generates a random float imbetween range low and high
			lattitude_list.append(i)
			low -= 1
			high += 1
	return lattitude_list

lon = gen_random_waypoints_long(start_Long, end_Long)
lat = gen_random_waypoints_lat(start_Lat, end_Lat)

coordinate_file = open("coordinates.txt", 'w')
for x in range(len(lon)):
	coordinate_file.write(str((lon[x], lat[x])) + "\n")

coordinate_file.close()

