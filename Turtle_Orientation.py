#!/usr/bin/env python

import sys
import rospy
from std_msgs.msg import Float32MultiArray, Float32
from geometry_msgs.msg import Twist
import numpy as np
import threading
import time
import os
import matplotlib.pyplot as plt
import math
import timeit

global vel_robot
vel_robot = Twist()
vel_robot.linear.x = 0
vel_robot.linear.y = 0
vel_robot.linear.z = 0
vel_robot.angular.x = 0
vel_robot.angular.y = 0
vel_robot.angular.z = 0

global alpha, beta, rho
rho = 1
alpha = 0
beta = 0

global goal_x, goal_y, goal_orientation, goal_position
goal_position = (1, -1, np.radians(45))
goal_x = (1, -1, 0)
goal_y = (-1, 1, 2)
goal_orientation = (np.radians(45), np.radians(90), np.radians(-90))

# Initial Position
actual_position = (0, -2, np.radians(-45))

global finish, escene
finish = 0 
escene = 0

global kp, ka, kb
kp = 0.6
ka = 1.2
kb = -0.2

global x_turtle, y_turtle, orientation_turtle, delta_x, delta_y
x_turtle = 0
y_turtle = 0
orientation_turtle = 0
delta_x = 0
delta_y = 0

def turtlebot_position(msg):
	global x_turtle, y_turtle, delta_x, delta_y
	linear = msg.linear
	x_turtle = linear.x
	y_turtle = linear.y
	delta_x = goal_position[0] - x_turtle
	delta_y = goal_position[1] - y_turtle

def turtle_orientation(msg):
	global orientation_turtle
	orientation_turtle = msg.data

def polar_coordinate():
	global rho, alpha, beta
	rho = math.sqrt(delta_x**2 + delta_y**2)
	alpha = -orientation_turtle + math.atan2(delta_y, delta_x)
	beta = -orientation_turtle - alpha + goal_position[2]
	print(rho, goal_position[0], goal_position[1], escene, orientation_turtle)

def controlador():
	global goal_position, escene, finish
	rospy.init_node('bono.py', anonymous=True)
	pub_vel = rospy.Publisher('/turtlebot_cmdVel', Twist, queue_size=10)
	rospy.Subscriber('/turtlebot_position', Twist, turtlebot_position)
	rospy.Subscriber('/turtlebot_orientation', Float32, turtle_orientation)
	#rospy.Subscriber('/simulationTime', Float32, timeCallback)

	try:

		rate = rospy.Rate(4)  # 10hz
		while not rospy.is_shutdown():

			if escene < 2: 
				if x_turtle > goal_position[0] - 0.1 and x_turtle < goal_position[0] + 0.1:
					if y_turtle > goal_position[1] - 0.1 and y_turtle < goal_position[1] + 0.1:
						escene = escene + 1
					else: 
						pass
				else:
					pass 
			else: 
				pass

			goal_position = (goal_x[escene], goal_y[escene], goal_orientation[escene])

			polar_coordinate()

			if finish == 0:
				if rho > 0.032:
					vel_robot.linear.x = np.clip(kp*rho, -0.7, 0.7) # Velocidad Lineal del Robot
					vel_robot.angular.z = np.clip(ka*alpha + kb*beta, -math.pi, math.pi) # Velocidad Angular del Robot
				else:
					vel_robot.linear.x = 0
					if (orientation_turtle < goal_position[2] + 0.05 and orientation_turtle > goal_position[2] - 0.05):
						vel_robot.angular.z = 0
						finish = 1
					else:
						vel_robot.angular.z = np.clip(ka*alpha + kb*beta, -math.pi, math.pi)
			else: 
				sys.exit()
			
			pub_vel.publish(vel_robot)

			rate.sleep()

	except rospy.ServiceException as e:
		print("Error!!")


if __name__ == '__main__':
	try:
		controlador()
	except rospy.ROSInterruptException:
		pass