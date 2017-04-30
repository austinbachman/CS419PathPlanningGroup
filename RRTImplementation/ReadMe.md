This program was written in python and tested in Ubuntu 14.04 by Robert Trimble.
To run the program, one must go to its proper location in enter "python RRT.py" in the terminal.

The program implements both RRT and RRT* in a 2D environment. The choice of which to use is presented to the user through a command prompt.

RRT works by selecting a random point for a node, and the connecting that node to the nearest other node. RRT will continue to do this until it selects a point close to the destination target.

RRT* works in the same way as RRT, except when a node connection is made, the new node will also look for other nearby nodes. If one is found, RRT* will determine if a shorter path would actually be breaking that nodes current connection and replacing it with the newly created node.

NOTES:
Current configuration values are as follows:

graphSize = 100		#Range for random sampling
startPoint = (1,1)	#Start point
endPoint = (90,90)	#Destination
endRadius = 2 		#How close to end result one needs to get to destiination
starRadius = 5		#Radius to make an RRT* connection

These values can be changed in the RRT.py file and are located at the top.

This program will randomly place one 20-by-20 size object in the middle of the graph that will be avoided. While the default start point and destination will not interfer with the object, that can not be gaurenteed if changed.

ERRORS:
The only code breaking error is if the user does not provide an integer input. All else with the program seems to work as required.
