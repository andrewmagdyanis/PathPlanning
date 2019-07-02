#!/usr/bin/env python
#Editing and Commenting by: Andrew @ 8:55pm  17/6/2019
from heapq import heappush, heappop
import math, time
import roslib
import rospy
import os
from   std_msgs.msg import Int32MultiArray
from   std_msgs.msg import Int32
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

#pose of robot2:
xR2=0
yR2=0

#pose of robot3:
xR3=0
yR3=0

#pose of obstacle1:
xO1=0
yO1=0
#pose of obstacle2:
xO2=0
yO2=0

rob_num = 0
list_goal=[]
final_goals_list = []
#------------------------------------------------------------------------------#
# create the publisher: (name it Route)
#Route= rospy.Publisher('route_Rob_to_move',String, queue_size=10)
pub_path_int = rospy.Publisher('Planning_Output4',Int32MultiArray, queue_size=10)
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
def update_all_then_map():

	rospy.Subscriber('robot4' ,Int32MultiArray,callback1)
	rospy.Subscriber('next_goals_px' ,Int32MultiArray,callback2)

	# Subscribe to robot1 position from the over head cam
	rospy.Subscriber('robot1' ,Int32MultiArray,callback3)

	# Subscribe to robot2 position from the over head cam
	rospy.Subscriber('robot2' ,Int32MultiArray,callback5)

	# Subscribe to robot3 position from the over head cam
	rospy.Subscriber('robot3' ,Int32MultiArray,callback7)

	# Subscribe to obstcales position from the over head cam
	rospy.Subscriber('obst1',Int32MultiArray,callback9)
	rospy.Subscriber('obst2',Int32MultiArray,callback10)

	rospy.sleep(5)
	map_()

##set the callbacks function
def callback0(data):
	global rob_num
	rob_num = data.data
	if(rob_num == 4):
		update_all_then_map()

def callback1(data):# for pose of the robot to move
	global xA,yA
	xA=data.data[0]
	yA=data.data[1]
def callback2(data):# for goal of the robot to move
	global xB,yB
	xB=data.data[6]#in px #//17.5)#in cm
	#xB = int((round(xB))

	yB=data.data[7]#//17.5)
	#yB = int((int (round(yB))
####################################################
def callback3(data):# for pose of robot1
	global xR1,yR1
	xR1=data.data[0]
	yR1=data.data[1]
####################################################
def callback5(data):# for pose of robot2
	global xR2,yR2
	xR2=data.data[0]
	yR2=data.data[1]
####################################################
def callback7(data):# for pose of robot3
	global xR3,yR3
	xR3=data.data[0]
	yR3=data.data[1]
####################################################
def callback9(data):# for pose of obstacle 1
	global xO1,yO1
	xO1=data.data[0]
	yO1=data.data[1]
def callback10(data):# for pose of obstacle 2
	global xO2,yO2
	xO2=data.data[0]
	yO2=data.data[1]

#------------------------------------------------------------------------------#
def listener():
	rospy.init_node('robot4_map')

	# Subscribe to robto to move position from the over head cam
	rospy.Subscriber('robot_to_be_moved' ,Int32,callback0)
	rospy.Subscriber('robot4' ,Int32MultiArray,callback1)
	rospy.Subscriber('next_goals_px' ,Int32MultiArray,callback2)

	# Subscribe to robot1 position from the over head cam
	rospy.Subscriber('robot1' ,Int32MultiArray,callback3)

	# Subscribe to robot2 position from the over head cam
	rospy.Subscriber('robot2' ,Int32MultiArray,callback5)

	# Subscribe to robot3 position from the over head cam
	rospy.Subscriber('robot3' ,Int32MultiArray,callback7)

	# Subscribe to obstcales position from the over head cam
	rospy.Subscriber('obst1',Int32MultiArray,callback9)
	rospy.Subscriber('obst2',Int32MultiArray,callback10)

#------------------------------------------------------------------------------#
def map_():
	global final_goals_list
	if((xB>0) or (yB>0)):
		global dx,dy
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
			the_4map[yR1][xR1]     = 1
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
		#Route.publish(route)
		# subtract the time now from var t to get the time used in algorithm
		"""print 'Time=', time.time() - t
		print 'Route'
		print route"""

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
		"""print 'Map:'
		for y in range(m):
			for x in range(n):
				xy = the_map[y][x]
				if xy == 0:
					print '.', # space
				elif xy == 1:
				elif xy == 2:
					print 'S', # start
							print 'o', # leader
				elif xy == 3:
					print 'R', # route
				elif xy == 4:
					print 'F', # finish
			print
		#os.system('clear')"""

		for y in range(m):
			for x in range(n):
				xy = the_map[y][x]
				#print xy
				if  xy == 3 or xy ==4:
					list_goal.append(x)
					list_goal.append(y)
					print "x,y",x,y

		# reshaping that array into a list of lists with numpy
		rows = len(list_goal) /2
		reshaped_goals_list = np.reshape( list_goal, (rows,2))
		last_point = [xA, yA]
		while ( last_point[0] != xB ) or ( last_point[1] != yB):
			for i in range ( len(reshaped_goals_list) ):
				if ( reshaped_goals_list[i][0] != -11 ) and ( reshaped_goals_list[i][1] != -11):
					error_x = abs (last_point[0] - reshaped_goals_list[i][0])
					error_y = abs (last_point[1] - reshaped_goals_list[i][1])

					if (error_x == 0 and error_y == 0)or(error_x == 0 and error_y == 1) or (error_x == 1 and error_y == 0) or (error_x == 1 and error_y == 1):
						final_goals_list.append ( [ reshaped_goals_list[i][0], reshaped_goals_list[i][1] ])
						last_point = [reshaped_goals_list[i][0], reshaped_goals_list[i][1]]
						reshaped_goals_list[i] = [-11, -11]
						continue

		final_goals_list = np.array(final_goals_list).flatten().tolist()
		print "Final Goal List is: ", final_goals_list

		pub_path_int.publish(Int32MultiArray(data= final_goals_list))
	else:
		print'waiting robot to move goal'

#------------------------------------------------------------------------------#
if __name__ == '__main__':
	try:
		while not rospy.is_shutdown():
			listener()
			#raw_input('Press Enter...')
			rospy.spin()
	except rospy.ROSInterruptException:
		pass
