#!/usr/bin/env python
#Editing and Commenting by: Andrew @ 8:55pm  17/6/2019
from heapq import heappush, heappop
import math, time
import roslib
import rospy
from std_msgs.msg import Int32
import os
from   std_msgs.msg import Int32MultiArray
from   std_msgs.msg import String
import numpy as np

# set the varibales:
#start point: (pose of the robot to move)
xA=0
yA=0
#goal point: (pose of the robot to move)
xB=0
yB=0

#pose of robot1:
xR1=0
yR1=0
#goal of robot1:
xR1_goal=0
yR1_goal=0

#pose of robot2:
xR2=0
yR2=0
#goal of robot2:
xR2_goal=0
yR2_goal=0

#pose of robot3:
xR3=0
yR3=0
#goal of robot3:
xR3_goal=0
yR3_goal=0

#pose of obstacle1:
xO1=0
yO1=0
#pose of obstacle2:
xO2=0
yO2=0
list_goal=[]
#------------------------------------------------------------------------------#
# create the publisher: (name it Route)
Route= rospy.Publisher('route_Rob_to_move',String, queue_size=10)
pub_path_int = rospy.Publisher('path_int',Int32MultiArray, queue_size=10)
time.sleep(5)
#------------------------------------------------------------------------------#
## the node function
class node:
    xPos = 0
    yPos = 0
    G = 0
    F = 0

    def __init__(self, xPos, yPos, G, F):
        self.xPos = xPos
        self.yPos = yPos
        self.G = G
        self.F = F

    def __lt__(self, other):
        return self.F < other.F

    def updateF(self, xEnd, yEnd):
        self.F = self.G + self.H(xEnd, yEnd) * 10

    def H(self, xEnd, yEnd):
        xd = xEnd - self.xPos
        yd = yEnd - self.yPos
        d = math.sqrt(xd*xd + yd*yd)
        return(d)

    def updateG(self, dirs, d):
        if dirs == 8 and d % 2 != 0:
            self.G = self.G + 14
        else:
            self.G = self.G +10
#------------------------------------------------------------------------------#
#the path planning function that gives me the route as String:
'''
    n     :  no. of rows
    m     :  no. of columns
    xA,yA :  start point
    xB,yB :  goal point
    dirs  :  no. of directions
    deltX :  ??
    deltaY:  ??
'''
def pathPlanner(the_map, n, m, dirs, deltX, deltaY, xA, yA, xB, yB):
    closed_nodes = []
    open_nodes = []
    parent_dir = []
    row = [0] * n
    for i in range(m):
        closed_nodes.append(list(row))
        open_nodes.append(list(row))
        parent_dir.append(list(row))

    pq = [[], []]
    pqi = 0

    nA = node(xA, yA, 0, 0)
    nA.updateF(xB, yB)
    heappush (pq[pqi], nA)
    open_nodes[yA][xA] = nA.F

    while len(pq[pqi]) > 0:

        n1 = pq[pqi][0]
        nA = node(n1.xPos, n1.yPos, n1.G, n1.F)
        x = nA.xPos
        y = nA.yPos
        heappop(pq[pqi])
        open_nodes[y][x] = 0
        closed_nodes[y][x] = 1

        if x == xB and y ==  yB:
            path = ''

            while not (x == xA and y == yA):
                j = parent_dir[y][x]
                c = str((j + dirs/2) % dirs)
                path = c + path
                x = x + dx[j]
                y = y + dy[j]

            return path

        for i in range(dirs):
            xdx = x + dx[i]
            ydy = y + dy[i]

            if not (xdx < 0 or xdx > n-1 or ydy < 0 or ydy > m-1 or the_map[ydy][xdx] == 1
                    or closed_nodes[ydy][xdx] == 1):

                m0 = node(xdx, ydy, nA.G, nA.F)
                m0.updateG(dirs, i)
                m0.updateF(xB, yB)

                if open_nodes[ydy][xdx] == 0:
                    open_nodes[ydy][xdx] = m0.F
                    heappush(pq[pqi], m0)
                    parent_dir[ydy][xdx] = (i + dirs/2) % dirs

                elif open_nodes[ydy][xdx] > m0.F:
                    open_nodes[ydy][xdx] = m0.F
                    parent_dir[ydy][xdx] = (i + dirs/2) % dirs

                    while not (pq[pqi][0].xPos == xdx and pq[pqi][0].yPos == ydy):
                        heappush(pq[1-pqi], pq[pqi][0])
                        heappop(pq[pqi])
                    heappop(pq[pqi])

                    if len(pq[pqi]) > len(pq[1-pqi]):
                        pqi = 1 - pqi
                    while len(pq[pqi]) > 0:
                        heappush(pq[1-pqi], pq[pqi][0])
                        heappop(pq[pqi])
                    pqi = 1 - pqi
                    heappush(pq[pqi], m0)

    return 'failed'

#------------------------------------------------------------------------------#
##set the callbacks function
def callback1(data):# for pose of the robot to move
	global xA,yA
	xA=data.data[0]
	yA=data.data[1]
def callback2(data):# for goal of the robot to move
	global xB,yB
	xB=data.data[0]
	yB=data.data[1]
####################################################
def callback3(data):# for pose of robot1
	global xR1,yR1
	xR1=data.data[0]
	yR1=data.data[1]
def callback4(data):# for goal of the robot1
	global xR1_goal,yR1_goal
	xR1_goal=data.data[0]
	yR1_goal=data.data[1]
####################################################
def callback5(data):# for pose of robot2
	global xR2,yR2
	xR2=data.data[0]
	yR2=data.data[1]
def callback6(data):# for goal of the robot2
	global xR2_goal,yR2_goal
	xR2_goal=data.data[0]
	yR2_goal=data.data[1]
####################################################
def callback7(data):# for pose of robot3
	global xR3,yR3
	xR3=data.data[0]
	yR3=data.data[1]
def callback8(data):# for goal of the robot3
	global xR3_goal,yR3_goal
	xR3_goal=data.data[0]
	yR3_goal=data.data[1]
####################################################
def callback9(data):# for pose of obstacle 1
	global xO1,yO1
	xO1=data.data[0]
	yO1=data.data[1]
def callback10(data):# for pose of obstacle 2
	global xO2,yO2
	xO2=data.data[0]
	yO2=data.data[1]

	map_()

#------------------------------------------------------------------------------#
def listener():
    rospy.init_node('robot_map')
    # Subscribe to robto to move position from the over head cam
    rospy.Subscriber('rob_to_move_pose_px' ,Int32MultiArray,callback1)
    rospy.Subscriber('robot_to_move_goal_px' ,Int32MultiArray,callback2)
    # Subscribe to robot1 position from the over head cam
    rospy.Subscriber('rob1_pose_px' ,Int32MultiArray,callback3)
    rospy.Subscriber('robot1_goal_px' ,Int32MultiArray,callback4)
	# Subscribe to robot2 position from the over head cam
    rospy.Subscriber('rob2_pose_px' ,Int32MultiArray,callback5)
    rospy.Subscriber('robot2_goal_px' ,Int32MultiArray,callback6)
    # Subscribe to robot3 position from the over head cam
    rospy.Subscriber('rob3_pose_px' ,Int32MultiArray,callback7)
    rospy.Subscriber('robot3_goal_px' ,Int32MultiArray,callback8)
    # Subscribe to obstcales position from the over head cam
    rospy.Subscriber('Ob1_pose_px',Int32MultiArray,callback9)
    rospy.Subscriber('Ob2_pose_px',Int32MultiArray,callback10)

#------------------------------------------------------------------------------#
def map_():
	if((xB>0) or (yB>0)):
		global dx,dy,list_goal
		#set our possible directions (ups and downs and diagonals)
		dirs = 8
		dx = [1, 1, 0, -1, -1, -1, 0, 1]
		dy = [0, 1, 1, 1, 0, -1, -1, -1]
		#set our map dimension
		n=10
		m=10
		the_map = []
		row = [0] *n
		for i in range(m):
			the_map.append(list(row))

		#xA = xA
		#yA = yA
		#xB = xB
		#yB = yB
	############################################################################
		##set the obstacles and the extremes cases
		## for each the robot the other two robots and their goals are obstacles
		## so it the route will be more safier
        if (xR2 ==0 ):
            the_map[yR2][xR2]     = 1
            the_map[yR2-1][xR2]   = 1
            the_map[yR2-1][xR2+1] = 1
            the_map[yR2][xR2+1]   = 1
            the_map[yR2+1][xR2+1] = 1
            the_map[yR2+1][xR2]   = 1

        elif (xR2 ==9 ):
            the_map[yR2][xR2]     = 1
            the_map[yR2-1][xR2]   = 1
            the_map[yR2+1][xR2]   = 1
            the_map[yR2+1][xR2-1] = 1
            the_map[yR2][xR2-1]   = 1
            the_map[yR2-1][xR2-1] = 1

        elif (yR2 ==0 ):
            the_map[yR2][xR2]     = 1
            the_map[yR2][xR2+1]   = 1
            the_map[yR2+1][xR2+1] = 1
            the_map[yR2+1][xR2]   = 1
            the_map[yR2+1][xR2-1] = 1
            the_map[yR2][xR2-1]   = 1

        elif (yR2 ==9):
            the_map[yR2][xR2]     = 1
            the_map[yR2-1][xR2]   = 1
            the_map[yR2-1][xR2+1] = 1
            the_map[yR2][xR2+1]   = 1
            the_map[yR2][xR2-1]   = 1
            the_map[yR2-1][xR2-1] = 1

        elif ((xR2 ==0) and (yR2 ==9)):
            the_map[yR2][xR2]     = 1
            the_map[yR2-1][xR2]   = 1
            the_map[yR2-1][xR2+1] = 1
            the_map[yR2][xR2+1]   = 1

        elif ((xR2 ==9) and (yR2 ==0)):
            the_map[yR2][xR2]     = 1
            the_map[yR2+1][xR2]   = 1
            the_map[yR2+1][xR2-1] = 1
            the_map[yR2][xR2-1]   = 1

        elif ((xR2 ==9) and (yR2 ==9)):
            the_map[yR2][xR2]     = 1
            the_map[yR2][xR2-1]   = 1
            the_map[yR2-1][xR2-1] = 1
            the_map[yR2-1][xR2]   = 1

        else:
            the_map[yR2][xR2]     = 1
            the_map[yR2-1][xR2]   = 1
            the_map[yR2-1][xR2+1] = 1
            the_map[yR2][xR2+1]   = 1
            the_map[yR2+1][xR2+1] = 1
            the_map[yR2+1][xR2]   = 1
            the_map[yR2+1][xR2-1] = 1
            the_map[yR2][xR2-1]   = 1
            the_map[yR2-1][xR2-1] = 1
        #########################################################################
        if (xR3 ==0 ):
            the_map[yR3][xR3]     = 1
            the_map[yR3-1][xR3]   = 1
            the_map[yR3-1][xR3+1] = 1
            the_map[yR3][xR3+1]   = 1
            the_map[yR3+1][xR3+1] = 1
            the_map[yR3+1][xR3]   = 1

        elif (xR3 ==9 ):
            the_map[yR3][xR3]     = 1
            the_map[yR3-1][xR3]   = 1
            the_map[yR3+1][xR3]   = 1
            the_map[yR3+1][xR3-1] = 1
            the_map[yR3][xR3-1]   = 1
            the_map[yR3-1][xR3-1] = 1

        elif (yR3 ==0 ):
            the_map[yR3][xR3]     = 1
            the_map[yR3][xR3+1]   = 1
            the_map[yR3+1][xR3+1] = 1
            the_map[yR3+1][xR3]   = 1
            the_map[yR3+1][xR3-1] = 1
            the_map[yR3][xR3-1]   = 1

        elif (yR3 ==9):
            the_map[yR3][xR3]     = 1
            the_map[yR3-1][xR3]   = 1
            the_map[yR3-1][xR3+1] = 1
            the_map[yR3][xR3+1]   = 1
            the_map[yR3][xR3-1]   = 1
            the_map[yR3-1][xR3-1] = 1

        elif ((xR3 ==0) and (yR3 ==9)):

            the_map[yR3][xR3]     = 1
            the_map[yR3-1][xR3]   = 1
            the_map[yR3-1][xR3+1] = 1
            the_map[yR3][xR3+1]   = 1

        elif ((xR3 ==9) and (yR3 ==0)):

            the_map[yR3][xR3]     = 1
            the_map[yR3+1][xR3]   = 1
            the_map[yR3+1][xR3-1] = 1
            the_map[yR3][xR3-1]   = 1

        elif ((xR3 ==9) and (yR3 ==9)):

            the_map[yR3][xR3]     = 1
            the_map[yR3][xR3-1]   = 1
            the_map[yR3-1][xR3-1] = 1
            the_map[yR3-1][xR3]   = 1

        else:
            the_map[yR3][xR3]     = 1
            the_map[yR3-1][xR3]   = 1
            the_map[yR3-1][xR3+1] = 1
            the_map[yR3][xR3+1]   = 1
            the_map[yR3+1][xR3+1] = 1
            the_map[yR3+1][xR3]   = 1
            the_map[yR3+1][xR3-1] = 1
            the_map[yR3][xR3-1]   = 1
            the_map[yR3-1][xR3-1] = 1
        #########################################################################
        if ((xR1 ==0 ) and (yR1 != 9)):
        	the_map[yR1][xR1]     = 1
        	the_map[yR1-1][xR1]   = 1
        	the_map[yR1-1][xR1+1] = 1
        	the_map[yR1][xR1+1]   = 1
        	the_map[yR1+1][xR1+1] = 1
        	the_map[yR1+1][xR1]   = 1

        elif ((xR1 ==9 )and (yR1 != 9) and (yR1 !=0)):
        	the_map[yR1][xR1]     = 1
        	the_map[yR1-1][xR1]   = 1
        	the_map[yR1+1][xR1]   = 1
        	the_map[yR1+1][xR1-1] = 1
        	the_map[yR1][xR1-1]   = 1
        	the_map[yR1-1][xR1-1] = 1

        elif ((yR1 ==0 )and (xR1 != 9)):
        	the_map[yR1][xR1]     = 1
        	the_map[yR1][xR1+1]   = 1
        	the_map[yR1+1][xR1+1] = 1
        	the_map[yR1+1][xR1]   = 1
        	the_map[yR1+1][xR1-1] = 1
        	the_map[yR1][xR1-1]   = 1

        elif ((yR1 ==9) and (xR1 != 9) and (xR1 !=0)):
        	the_map[yR1][xR1]     = 1
        	the_map[yR1-1][xR1]   = 1
        	the_map[yR1-1][xR1+1] = 1
        	the_map[yR1][xR1+1]   = 1
        	the_map[yR1][xR1-1]   = 1
        	the_map[yR1-1][xR1-1] = 1

        elif ((xR1 ==0) and (yR1 ==9)):

        	the_map[yR1][xR1]     = 1
        	the_map[yR1-1][xR1]   = 1
        	the_map[yR1-1][xR1+1] = 1
        	the_map[yR1][xR1+1]   = 1

        elif ((xR1 ==9) and (yR1 ==0)):

        	the_map[yR1][xR1]     = 1
        	the_map[yR1+1][xR1]   = 1
        	the_map[yR1+1][xR1-1] = 1
        	the_map[yR1][xR1-1]   = 1

        elif ((xR1 ==9) and (yR1 ==9)):

        	the_map[yR1][xR1]     = 1
        	the_map[yR1-1][xR1]   = 1
        	the_map[yR1][xR1-1]   = 1
        	the_map[yR1-1][xR1-1] = 1

        else:
        	the_map[yR1][xR1]     = 1
        	the_map[yR1-1][xR1]   = 1
        	the_map[yR1-1][xR1+1] = 1
        	the_map[yR1][xR1+1]   = 1
        	the_map[yR1+1][xR1+1] = 1
        	the_map[yR1+1][xR1]   = 1
        	the_map[yR1+1][xR1-1] = 1
        	the_map[yR1][xR1-1]   = 1
        	the_map[yR1-1][xR1-1] = 1

        ########################################################################
        the_map[yR2_goal][xR2_goal] = 1
        the_map[yR1_goal][xR1_goal] = 1
        ########################################################################
        the_map[yO1][xO1] = 1
        the_map[yO1-1][xO1] = 1
        the_map[yO1-1][xO1+1] = 1
        the_map[yO1][xO1+1] = 1
        the_map[yO1+1][xO1+1] = 1
        the_map[yO1+1][xO1] = 1
        the_map[yO1+1][xO1-1] = 1
        the_map[yO1][xO1-1] = 1
        the_map[yO1-1][xO1-1] = 1
        #######################################################################
        the_map[yO2][xO2] = 1
        the_map[yO2-1][xO2] = 1
        the_map[yO2-1][xO2+1] = 1
        the_map[yO2][xO2+1] = 1
        the_map[yO2+1][xO2+1] = 1
        the_map[yO2+1][xO2] = 1
        the_map[yO2+1][xO2-1] = 1
        the_map[yO2][xO2-1] = 1
        the_map[yO2-1][xO2-1] = 1
        the_map[0][0] = 1

        print 'Map size (X,Y):', n, m
        print 'Start:', xA, yA
        print 'Finish', xB, yB

        t = time.time() # get the time now and save it in t
        route=pathPlanner(the_map, n, m, dirs, dx, dy, xA, yA, xB, yB)

        #route1= Int32MultiArray(data=list(route))
        Route.publish(route)
        # subtract the time now from var t to get the time used in algorithm
        print 'Time=', time.time() - t
        print 'Route'
        print route

        if len(route) > 0:
        	x = xA
        	y = yA
        	the_map[y][x] = 2
        	for i in range(len(route)):
        		j = int(route[i])
        		x += dx[j]
        		y += dy[j]
        		the_map[y][x] = 3
                the_map[yB][xB] = 4

        # display the map with the route added
        print 'Map:'
        for y in range(m):
        	for x in range(n):
        		xy = the_map[y][x]
        		if xy == 0:
        			print '.', # space
        		elif xy == 1:
        			print 'o', # leader
        		elif xy == 2:
        			print 'S', # start
        		elif xy == 3:
        			print 'R', # route
        		elif xy == 4:
        			print 'F', # finish
        	print
        #os.system('clear')
        print "list goal b: ", list_goal
        for y in range(m):
            for x in range(n):
                xy = the_map[y][x]
                #print xy
                if  xy == 3 or xy ==4:
                    list_goal.append(x)
                    list_goal.append(y)
                    print "x,y",x,y
                #else: print "error"

	else:
		print'waiting robot to move goal'

        pub_path_int.publish(Int32MultiArray(data= list_goal))
        time.sleep(3)


#------------------------------------------------------------------------------#
if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            listener()
            pub_path_int.publish(Int32MultiArray(data= list_goal))
            time.sleep(3)
            #raw_input('Press Enter...')
            rospy.spin()
    except rospy.ROSInterruptException:
        pass
