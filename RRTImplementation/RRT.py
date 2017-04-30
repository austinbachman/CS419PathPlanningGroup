#Created by Robert Trimble
#CS 491 - Introduction to Aerial Robotics
#Implements RRT and RRT*

import math
import random
import numpy
import pylab

###############################################################
###############################################################
#Values to tweak
graphSize = 100		#Range for random sampling
startPoint = (1,1)	#Start point
endPoint = (90,90)	#Destination
endRadius = 2 		#How close to end result one needs to get to destiination
starRadius = 5		#Radius to make an RRT* connection
################################################################
################################################################


#Distance formula
def getDistance(x1,y1,x2,y2):
	dist = math.sqrt((x2-x1)**2 + (y2-y1)**2)
	return dist

#Check for object collisions
def objCollision(newX, newY, closestKey):
	x0 = newX
	y0 = newY
	x1 = points.get(closestKey)[0]
	y1 = points.get(closestKey)[1]

	#Get all points betweeen two points
	points_in_line = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = -1 if x0 > x1 else 1
        sy = -1 if y0 > y1 else 1
        if dx > dy:
            err = dx / 2.0
            while x != x1:
                points_in_line.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y1:
                points_in_line.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
        points_in_line.append((x, y))
        
	#Check if line overlaps the object
	if set(obj) & set(points_in_line):
		return True
	else:
		return False
	


numPoints = 0
points = {} #contains (X, Y, <Key to closest point>, distance to point)

#Add the destination
pylab.scatter(endPoint[0], endPoint[1], color = "red")

#Add the starting point
pylab.scatter(startPoint[0], startPoint[1], color = "green")
points = {numPoints: (startPoint[0], startPoint[1], -1, 0)}
numPoints += 1

#Create an object location
objX = random.randint(30, 60)
objY = random.randint(30, 60)

#Make the object bigger
obj = []
for x in range(-20,20):
	for y in range(-20,20):
		obj.append((objX + x, objY + y))

#Plot the object
for x in range(0,40):
	pylab.scatter(obj[x][0], obj[x][1], color = "yellow")
for x in range(1540,1600):
	pylab.scatter(obj[x][0], obj[x][1], color = "yellow")
for x in range(0,1600, 40):
	pylab.scatter(obj[x][0], obj[x][1], color = "yellow")
for x in range(39,1599, 40):
	pylab.scatter(obj[x][0], obj[x][1], color = "yellow")



#Decide whether or not to to do RRT or RRT*
data = input("Enter 1 for RRT or 2 for RRT*: \n")
if data is 1:
	star = False
elif data is 2:
	star = True
else:
	print "Invalid input\n"
	exit()

################
#Preform the RRT (or RRT*)
################
while True:
	#Pick a random spot
	newX = random.randint(0,graphSize)
	newY = random.randint(0,graphSize)

	#Find the exsisting closest point
	#################################
	distance = 1000
	closestKey = 0
	distanceFromStart = 0
	starPoints = []

	#Loop through all exsisting points
	for k, v in points.items():
		tempDistance = getDistance(newX,newY,v[0],v[1])
		#Update with closest point
		if tempDistance < distance:
			distance = tempDistance
			closestKey = k
			distanceFromStart = v[3] + distance
		
		#Get nearby points for RRT*
		if tempDistance < starRadius:
			starPoints.append(k)
	
	#Get new point if its too far away
	if getDistance(newX, newY, points.get(k)[0], points.get(k)[1]) > starRadius*3:
		continue

	#Make sure there is no object collision
	if objCollision(newX, newY, closestKey):
		continue

	#Add a point and connecting line
	points.update({numPoints: [newX, newY, closestKey, distanceFromStart]})
	numPoints += 1

	############################################
	############################################
	#If doing RRT*
	if star is True:
		#Loop through points
		for k in points.keys():
			#If point was flagged as close
			if k in starPoints:
				#See if a new connection is shorter
				tempDistance = getDistance(newX, newY, points.get(k)[0], points.get(k)[1])
				if distanceFromStart + tempDistance < points.get(k)[3]:
					#Update the closest point and distance from start
					points[k][2] = numPoints - 1
					points[k][3] = distanceFromStart + tempDistance
	#############################################
	#############################################

	#Check if destination was reached
	if getDistance(newX, newY, endPoint[0], endPoint[1]) < endRadius:
		break

########################
#Paint all of the routes
########################
for k, v in points.items():
	if k > 0:
		x,y = zip(*((v[0], v[1]), (points.get(v[2])[0], points.get(v[2])[1])))
		pylab.plot(x,y, color = "blue")

#####################
#Paint the best route
#####################
currentPoint = numPoints - 1 #Start with last point found
while True:
	point1X = points.get(currentPoint)[0]
	point1Y = points.get(currentPoint)[1]

	currentPoint = points.get(currentPoint)[2]
	#Check if youre back to start point
	if currentPoint is -1:
		break

	point2X = points.get(currentPoint)[0]
	point2Y = points.get(currentPoint)[1]

	x,y = zip(*((point1X, point1Y), (point2X, point2Y)))
	pylab.plot(x,y, color = "black")
		

#Plot
pylab.show()
