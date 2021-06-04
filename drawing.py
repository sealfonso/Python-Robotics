#!/usr/bin/env python

import pygame #pip3 install pygame
import rospy
import sys
import numpy as np
import time
# from tkinter import * #sudo apt-get install python3-tk
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import os
import math
import timeit
import sys
import roslib
import matplotlib.pyplot as plt

global vel_pioneer, pattern_pioneer
vel_pioneer = Float32MultiArray()
vel_pioneer.data = [0,0]
pattern_pioneer = Float32MultiArray()
pattern_pioneer.data = [0]

global x_pioneer, y_pioneer, orientation_pioneer, delta_x, delta_y
x_pioneer = 0
y_pioneer = 0
orientation_pioneer = 0
delta_x = 0
delta_y = 0

global alpha, beta, rho
rho = 1
alpha = 0
beta = 0

global finished, step, selector, phase, navigation_step
finished = False
step = -1
navigation_step = 0 # CAMBIAR A 0 !!!
selector = 0
phase = 0

global kp, ka, kb
kp = 0.6
ka = 1.2
kb = -0.2

global old_center, navigation
old_center = (4.9, 0.5)
navigation = True


absPath = os.path.abspath(__file__)
nPath = os.path.split(os.path.dirname(absPath))[0]

run = open(os.path.join(nPath + "/resources/orden.txt"), "rt")     

#$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
## Inicializacion del nodo
#$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

def sysCall_init(): 
	print("Ready")
	settingUp()
	pioneer_navigation()
	movement()

#$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
## Lectura de los Parametros de Interfaz
#$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

def settingUp():
	global new_center, theta, scale, extra, dibujo_point
	absPath = os.path.abspath(__file__)
	nPath = os.path.split(os.path.dirname(absPath))[0]
	file = open(os.path.join(nPath + "/resources/puntos.txt"), "r+")
	lines = file.readlines()
	for i in range(len(lines)):
		lines[i] = lines[i].split()
		lines[i] = list(map(float, lines[i]))
	dibujo_point = (lines[2][0], lines[2][1], lines[2][2])
	new_center = (lines[3][0], lines[3][1])
	theta = np.round(-np.radians(lines[4][0]) - math.pi/2, 2)
	scale = lines[5][0]
	extra = lines[6][0]
	sysCall_configuration(theta)
	file.close()

#$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
## Configuracion del Dibujo
#$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

def sysCall_configuration(msg): 
	global diff_position, diff_rotated, diff_scale, R
	R = [[math.cos(theta), math.sin(theta)], [-math.sin(theta), math.cos(theta)]]
	diff_position = (new_center[0] - old_center[0], new_center[1] - old_center[1])
	center_rotated = np.dot(R, [[new_center[0]], [new_center[1]]])
	diff_rotated = (new_center[0] - center_rotated[0][0], new_center[1] - center_rotated[1][0])
	diff_scale = (old_center[0] - old_center[0]*scale, old_center[1] - old_center[1]*scale)

#$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
## Envio de Patron a Dibujar desde Interfaz
#$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

def figure(msg): 
	spheres = msg.data
	if extra == 2:
		if spheres[0] > spheres[1] and spheres[0] > spheres[2]: 
			pez()
			pattern_pioneer.data = [4]
			pub_figure.publish(pattern_pioneer)
		elif spheres[1] > spheres[0] and spheres[1] > spheres[2]:
			gato()
			pattern_pioneer.data = [1]
			pub_figure.publish(pattern_pioneer)
		elif spheres[2] > spheres[1] and spheres[2] > spheres[0]:
			pato()
			pattern_pioneer.data = [3]
			pub_figure.publish(pattern_pioneer)
		else: 
			jirafa()
			pattern_pioneer.data = [2]
			pub_figure.publish(pattern_pioneer)
	else: 
		#print("BONO")
		pattern("batman") # SE DEBE CAMBIAR A BATMAN
		pattern_pioneer.data = [5] # SE DEBE CAMBIAR A BATMAN
		pub_figure.publish(pattern_pioneer)

#$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
## Patrones de Dibujo
#$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

def pattern(msg):
	#print("Se cargo el dibujo")
	global limits_pattern
	absPath = os.path.abspath(__file__)
	nPath = os.path.split(os.path.dirname(absPath))[0]
	file = open(os.path.join(nPath + "/resources/Patrones/" + msg + ".txt"), "r+")
	lines = file.readlines()
	for i in range(len(lines)-2):
		lines[i] = lines[i].split(',')
		lines[i] = list(map(float, lines[i]))
	x_pattern = lines[0]
	y_pattern = lines[1]
	orientation_pattern = np.radians(lines[2])
	goal_point = np.round((lines[3][0], lines[3][1], np.radians(lines[3][2])), 2)

	# Verificacion de los Limites del Movimiento del Dibujo 

	if theta == np.round(-math.pi,2) or theta == np.round(-math.pi*2,2):
		limits_pattern = lines[4].split(',')
	if theta == np.round(-math.pi/2,2) or theta == np.round(-math.pi*3/2,2): 
		limits_pattern = lines[5].split(',')
	else: 
		limits_pattern = lines[4].split(',')

	file.close()

	# Aplicacion de Rotacion del Patron
	rotation(x_pattern, y_pattern, orientation_pattern, goal_point)

#$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
## Aplicacion de Rotacion
#$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

def rotation(msg1, msg2, msg3, msg4):
	global x_pattern, y_pattern, goal_point, orientation_pattern
	x_pattern = np.zeros(len(msg1))
	y_pattern = np.zeros(len(msg2))
	orientation_pattern = np.zeros(len(msg3))
	goal_point = np.zeros(len(msg4))
	for i in range(len(msg1)):
		x_pattern[i] = msg1[i]*scale + diff_scale[0] + diff_position[0]
		y_pattern[i] = msg2[i]*scale + diff_scale[1] + diff_position[1]
		rotated_points = np.dot(R, [[x_pattern[i]], [y_pattern[i]]])
		x_pattern[i] = rotated_points[0] + diff_rotated[0]
		y_pattern[i] = rotated_points[1] + diff_rotated[1]

	# Rotacion del Punto Destino (Goal_Point)

	goal_point[0] = msg4[0]*scale + diff_scale[0] + diff_position[0]
	goal_point[1] = msg4[1]*scale + diff_scale[1] + diff_position[1]
	rotated_goal_point = np.dot(R, [[goal_point[0]], [goal_point[1]]])
	goal_point[0] = rotated_goal_point[0] + diff_rotated[0]
	goal_point[1] = rotated_goal_point[1] + diff_rotated[1]

	# Rotacion de las Orientaciones de las Poses del Patron

	for i in range(len(msg3)):
		if (msg3[i] - theta) > math.pi:
			temp1 = math.pi - msg3[i]
			orientation_pattern[i] = np.round(-math.pi + (-theta - temp1), 2)
		elif (msg3[i] - theta) < -math.pi:
			temp1 = msg3[i] - math.pi
			orientation_pattern[i] = np.round(math.pi - (-theta - temp1), 2)
		else:
			orientation_pattern[i] = np.round(msg3[i] - theta, 2)

	# Rotacion de la Orientacion Inicial del Patron

	if (msg4[2] - theta) > math.pi:
		temp1 = math.pi - msg4[2]
		goal_point[2] = np.round(-math.pi + (-theta - temp1), 2)
	elif (msg4[2] - theta) < -math.pi:
		temp1 = msg4[2] - math.pi
		goal_point[2] = np.round(math.pi - (-theta - temp1), 2)
	else:
		goal_point[2] = np.round(msg4[2] - theta, 2)

#$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
## ALGORITMO DE MOVIMIENTO
#$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

#$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
## Cargue de la Ruta de Navegacion
#$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

def pioneer_navigation():
	global x_navigation, y_navigation, orientation_navigation, final_step, dibujo_step, algoritmo
	absPath = os.path.abspath(__file__)
	nPath = os.path.split(os.path.dirname(absPath))[0]
	file = open(os.path.join(nPath + "/resources/ruta.txt"), "r+")
	lines = file.readlines()
	x_navigation = np.zeros(len(lines))
	y_navigation = np.zeros(len(lines))
	orientation_navigation = np.zeros(len(lines))
	for i in range(len(lines)):
		lines[i] = lines[i].split()
		lines[i] = list(map(float, lines[i]))
		x_navigation[i] = lines[i][0]
		y_navigation[i] = lines[i][1]
		if lines[i][2] > 180 and lines[i][2] < 360: 
			orientation_navigation[i] = np.round(-math.pi + np.radians(lines[i][2] - 180), 2)
		else: 
			orientation_navigation[i] = np.round(np.radians(lines[i][2]),2)

	dibujo_step = len(lines) - 1

	file.close()
	file = open(os.path.join(nPath + "/resources/conteo.txt"), "r+")
	lines = file.readlines()
	final_step = float(lines[0])

	file = open(os.path.join(nPath + "/resources/algoritmo.txt"), "r+")
	lines = file.readlines()
	algoritmo = lines[0][0]

#$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
## Actualizacion de los Puntos de Navegacion
#$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

def update_navigation_points(): 
	global navigation, navigation_step
	temp1 = np.round(orientation_pioneer, 2)
	temp2 = orientation_navigation[navigation_step]
	if navigation_step == 0: 
		if temp1 == temp2 + 0.1:
			stop(True)
			navigation_step = navigation_step + 1
			update_navigation_step()
		else: 
			pioneer_velocity(0, np.clip(3*(orientation_navigation[navigation_step] + 0.1 - orientation_pioneer), -1.5, 1.5))
	
	elif navigation_step == dibujo_step: 
		print("Next Scene - Drawing")
		if rho < 0.3:
			orientation_navigation[dibujo_step] = dibujo_point[2] 
			if temp1 == temp2: 
				stop(True)
				navigation_step = navigation_step + 1
				navigation = False
			else: 
				pioneer_velocity(0, np.clip(3*(orientation_navigation[navigation_step] - orientation_pioneer), -1.5, 1.5))
		else: 
			pioneer_velocity(3*np.clip(kp*rho, -0.7, 0.7), 0)
	else: 
		if rho < 0.2:
			if navigation_step == final_step - 1:
				if rho < 0.05: 
					if temp1 == temp2: 
						stop(True)
						navigation_step = navigation_step + 1
						update_navigation_step()
					else: 
						pioneer_velocity(0, np.clip(3*(orientation_navigation[navigation_step] - orientation_pioneer), -1.5, 1.5))
				else: 
					pioneer_velocity(3*np.clip(kp*rho, -0.7, 0.7), orientation_navigation[navigation_step] - orientation_pioneer)
			elif navigation_step == final_step:
				if temp1 == temp2: 
					stop(True)
					navigation_step = navigation_step + 1
					update_navigation_step()
				else:  
					pioneer_velocity(0, np.clip(3*(orientation_navigation[navigation_step] - orientation_pioneer), -1.5, 1.5))
			else: 
				if algoritmo == "2":
					if temp1 == orientation_navigation[navigation_step + 1]:
						stop(True)
						navigation_step = navigation_step + 1
						update_navigation_step()
					else: 
						pioneer_velocity(0, np.clip(3*(orientation_navigation[navigation_step + 1] - orientation_pioneer), -1.5, 1.5))
				else: 
					#print("Vamos bueno")
					if temp1 == temp2: 
						stop(True)
						navigation_step = navigation_step + 1
						update_navigation_step()
					else: 
						pioneer_velocity(0, np.clip(3*(orientation_navigation[navigation_step] - orientation_pioneer), -1.5, 1.5))

		else: 
			if algoritmo == "2":
				pioneer_velocity(np.clip(kp*rho, -0.7, 0.7), orientation_navigation[navigation_step] - orientation_pioneer)
			else: 
				#print("SEGURO")
				pioneer_velocity(np.clip(kp*rho, -0.7, 0.7), ka*alpha + kb*beta)

def update_navigation_step(): 
	global navigation_step
	while True: 
		if navigation_step < dibujo_step:
			if orientation_navigation[navigation_step] == orientation_navigation[navigation_step + 1]:
				navigation_step = navigation_step + 1
			else: 
				break 
		else: 
			break

#$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
## ALGORITMO DEL DIBUJO
#$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

#$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
## Actualizacion de la Posicion y Orientacion del Rover
#$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

def pioneer_position(msg):
	global x_pioneer, y_pioneer, orientation_pioneer, delta_x, delta_y
	x_pioneer = msg.linear.x
	y_pioneer = msg.linear.y
	orientation_pioneer = msg.angular.z

	if navigation == False: 
		# Diferencia de Coordenadas para la Tarea de Dibujo 
		delta_x = goal_point[0] - x_pioneer
		delta_y = goal_point[1] - y_pioneer
	else: 
		# Diferencia de Coordendas para la Tarea de Navegacion
		delta_x = x_navigation[navigation_step] - x_pioneer
		delta_y = y_navigation[navigation_step] - y_pioneer

#$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
## Actualizacion de la Velocidad del Rover
#$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

def pioneer_velocity(linearVel, angularVel):
	global vel_pioneer
	wheel_radius = 0.195
	wheel_separation = 0.381
	speed_left = (linearVel - angularVel*wheel_separation/2)/wheel_radius
	speed_right = (linearVel + angularVel*wheel_separation/2)/wheel_radius
	vel_pioneer.data = [speed_left, speed_right]
	#print(x_pioneer, y_pioneer, np.round(orientation_pioneer, 2), selector, phase, step, linearVel, angularVel, rho)
	#print(navigation_step)
	#print(linearVel, angularVel)
	#print(x_navigation[navigation_step], y_navigation[navigation_step])
	#print(x_pioneer, y_pioneer, np.round(orientation_pioneer, 2), rho, orientation_navigation[navigation_step])
#$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
## Actualizacion de los Parametros del Controlador
#$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

def polar_coordinate():
	global rho, alpha, beta
	rho = math.sqrt(delta_x**2 + delta_y**2)
	alpha = -orientation_pioneer + math.atan2(delta_y, delta_x)

	if navigation == False:
		beta = -orientation_pioneer - alpha + goal_point[2]
	else: 
		beta = -orientation_pioneer - alpha + orientation_navigation[navigation_step]

#$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
## Actualizacion del Punto Destino
#$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

def update_goal_point(): 
	global step, goal_point, phase 
	if selector == 1 or selector == 3: 
		if rho < 0.06:
			phase = 1
			if np.round(orientation_pioneer, 2) == goal_point[2]:
				stop(True)
				step = step + 1
				phase = 2
			else: 
				pass
		else:
			phase = 0
	else: 
		if limits_pattern[step] == 'Y':
			if y_pioneer >= y_pattern[step] - 0.1 and y_pioneer <= y_pattern[step] + 0.1:
				phase = 1
				if np.round(orientation_pioneer, 2) == orientation_pattern[step]:
					stop(True)
					step = step + 1
					phase = 2
				else: 
					pass
			else: 
				phase = 0
		else: 
			if x_pioneer >= x_pattern[step] - 0.1 and x_pioneer <= x_pattern[step] + 0.1:
				phase = 1
				if np.round(orientation_pioneer, 2) == orientation_pattern[step]:
					stop(True)
					step = step + 1
					phase = 2
				else: 
					pass
			else: 
				phase = 0

#$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
## Determinacion de la Velocidad del Rover en el Proceso de Dibujo
#$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

def curve(): 
	if step == 0: 
		angular = goal_point[2] - orientation_pioneer
		if limits_pattern[step] == 'Y':
			dif = y_pattern[step] - goal_point[1]
			if y_pioneer >= y_pattern[step] - 0.2 and y_pioneer <= y_pattern[step] + 0.2:
				vel = 0.1
			else: 
				vel = 0.3
		else: 
			dif = x_pattern[step] - goal_point[0]
			if x_pioneer >= x_pattern[step] - 0.2 and x_pioneer <= x_pattern[step] + 0.2:
				vel = 0.1
			else: 
				vel = 0.3
	elif step >= 0 and step < len(x_pattern):  
		angular = orientation_pattern[step-1] - orientation_pioneer
		if limits_pattern[step] == 'Y':
			dif = y_pattern[step] - y_pattern[step-1]
			if y_pioneer >= y_pattern[step] - 0.2 and y_pioneer <= y_pattern[step] + 0.2:
				vel = 0.1
			else: 
				vel = 0.3
		else: 
			dif = x_pattern[step] - x_pattern[step-1]
			if x_pioneer >= x_pattern[step] - 0.2 and x_pioneer <= x_pattern[step] + 0.2:
				vel = 0.1
			else: 
				vel = 0.3
	else:
		angular = orientation_pattern[step-1] - orientation_pioneer
		if limits_pattern[step] == 'Y':
			dif = goal_point[1] - y_pattern[step-1]
			if y_pioneer >= goal_point[1] - 0.2 and y_pioneer <= goal_point[1] + 0.2:
				vel = 0.1
			else: 
				vel = 0.3
		else: 
			dif = goal_point[0] - x_pattern[step-1]
			if x_pioneer >= goal_point[0] - 0.2 and x_pioneer <= goal_point[0] + 0.2:
				vel = 0.1
			else: 
				vel = 0.3

	return ((math.pi/dif), vel, angular)

#$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
## Publicacion de la Velocidad en el Topico /pioneer_motorsVel
#$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

def update_movement(msg):
	global finished, selector
	selector = msg 
	update_goal_point()
	if selector == 1: 
		if phase == 1:
			pioneer_velocity(0, 3*(goal_point[2] - orientation_pioneer))
		elif phase == 0: 
			pioneer_velocity(np.clip(kp*rho, -0.7, 0.7), np.clip(ka*alpha + kb*beta, -math.pi, math.pi))
		else: 
			pass
	elif selector == 2:
		if phase == 1:
			pioneer_velocity(0, np.clip(3*(orientation_pattern[step] - orientation_pioneer), -1.5, 1.5))
		elif phase == 0: 
			dif = curve()
			if limits_pattern[step] == 'Y':
				pioneer_velocity(0.9*abs(math.sin(dif[0]*(y_pattern[step] - y_pioneer))) + dif[1], dif[2])
			else: 
				pioneer_velocity(0.9*abs(math.sin(dif[0]*(x_pattern[step] - x_pioneer))) + dif[1], dif[2])
		else: 
			pass
	else: 
		dif = curve()
		if phase == 1: 
			finished = True
			pioneer_velocity(0,0)
		else: 
			if limits_pattern[step] == 'Y':
				pioneer_velocity(0.7*abs(math.sin(dif[0]*(goal_point[1] - y_pioneer))) + dif[1], dif[2])
			else: 
				pioneer_velocity(0.7*abs(math.sin(dif[0]*(goal_point[0] - x_pioneer))) + dif[1], dif[2])

#$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
## Parada en los Bordes de la Figura
#$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

def stop(msg):
	global vel_pioneer
	if msg == True: 
		#print('Stopped')
		vel_pioneer.data = [0,0]
		pub_vel.publish(vel_pioneer)
		time.sleep(1)

#$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
## Funcion Principal
#$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

def movement():
	global finished, step, pub_vel, pub_figure, navigation
	rospy.init_node('drawing', anonymous=True)
	pub_vel = rospy.Publisher('/pioneer_motorsVel', Float32MultiArray, queue_size=10)
	pub_figure = rospy.Publisher('/pioneer_figure', Float32MultiArray, queue_size=1)
	rospy.Subscriber('/pioneer_position', Twist, pioneer_position)

	if extra == 2:
		if navigation_step == final_step + 1:
			#print("Seguro")
			rospy.Subscriber('/pioneer_prizes', Float32MultiArray, figure)
		else: 
			pass
	else: 
		print("Batman")
		rospy.Subscriber('/pioneer_prizes', Float32MultiArray, figure)

	try: 
		rate = rospy.Rate(4)  # 10hz
		while not rospy.is_shutdown():
			if navigation == False: 
				if finished == False: 
					polar_coordinate()
					if step == -1: 
						update_movement(1)
					elif step >= 0 and step < len(x_pattern): 
						update_movement(2)
					else: 
						print('Final Move')
						update_movement(3)
				else: 
					print('Shutting Down')
					orden = open(os.path.join(nPath + "/resources/orden.txt"), "w+")
					orden.write(str(1))
					orden.close()					
					sys.exit()
			else: 
				polar_coordinate()
				update_navigation_points()

			pub_vel.publish(vel_pioneer)

			rate.sleep()
			
	except rospy.ServiceException as e:
		print("Error!!")

if __name__ == '__main__':
	try:
		corre = int(run.read())
		if corre == 4:
			sysCall_init()
	except rospy.ROSInterruptException:
		pass
