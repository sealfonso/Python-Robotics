#!/usr/bin/env python
import matplotlib.pyplot as plt #print("{} -m pip install matplotlib".format(sys.executable))
from std_msgs.msg import Float32MultiArray, Float32
from geometry_msgs.msg import Twist
from threading import Thread
import numpy as np
import sys
import rospy
import math
import time
import os

##########################################SPECS################################################
#Este script se encarga de llevar al robot a un punto destino determinado llamado goal_position
#La idea seria importar una lista de puntos cordenados e iterar cambiando cada vez goal_position
#para seguir una ruta determinada
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


####################### Inicialización de variables #########################################
global vel_robot
vel_robot = Float32MultiArray()

global rho, alpha, beta, arrived
rho = 0
alpha = 0
beta = 0
arrived = False

global kp, ka, kb
kp = 5
ka = 2
kb = -2

global start, termina, time,l,r
#Variables erelacionadas con la cinematica del robot
alpha_1 = np.radians(90)
alpha_2 = np.radians(-90)
betha_1 = 0
betha_2 = np.radians(180)
l = 0.381
r = 0.195
R1 = np.array([[np.sin(alpha_1 + betha_1), -np.cos(alpha_1 + betha_1), -l/2 * np.cos(betha_1)]
                  , [np.sin(alpha_2 + betha_2), -np.cos(alpha_2 + betha_2), -l/2 * np.cos(betha_2)], [np.cos(alpha_1 + betha_1)
                   , np.sin(alpha_1 + betha_1), np.sin(betha_1)]])
R1inv = np.linalg.inv(R1)
R2 = np.array([[r, 0], [0, r]])

global posy_array, posx_array, angle, posx_teo_array, posy_teo_array, error
posx_array = np.array([])
posy_array = np.array([])
angle = 0
posx_teo_array = np.array([])
posy_teo_array = np.array([])
error = np.array([])
posx=0
posy=0

global	time_array, time, inicio, termina, time, tictac, vl, vr, k, w
time_array = np.array([])
inicio = False
termina = False
time = 0
vr = 0
vl = 0
k = 0
w = 0

global velangular, vellinear, vel_lin_x, vel_ang_z, delta_x, delta_y, orientation, goal_position
velangular=0
vellinear=0
vel_lin_x = 0
vel_ang_z = 0
delta_x = 0
delta_y = 0
orientation = 0
goal_position = (0,0, np.radians(0))
#Configuacion inicial de matplotlib

plt.show(block=False)
file = 'gridmap.txt'
#Se lee la imagen y se obtienen los valores binarios del mapeo
absPath = os.path.abspath(__file__)
nabsPath = os.path.split(os.path.dirname(absPath))[0]
f = open(os.path.join(nabsPath + "/resources/" + file), "r")
	
# Escribe en un vector la posición de destino

def destiny():
	global goal_position, k, w, termina, dat

	if k > w:
		dat = f.readline().split()
		goal_position = (float(dat[0]),float(dat[1]),np.radians(float(dat[2])))
	w = k
	goal_position = goal_position	

	print(goal_position)
	print(w)
	return goal_position

#funcion que se encarga de generar la graficas y almacenarlas en las carpeta results
def funcionGrafica():
    global posx_array, posx_teo_array, posy_array, posy_teo_array, inicio, posx, posy

    while not rospy.is_shutdown():

        if inicio == True:
            plt.clf()
            plt.ylim(0, 15)
            plt.xlim(0, 15)
            plt.plot(posx_array+7.5, posy_array+7.5, 'r-', label = 'Simulada')
            plt.plot(posx_teo_array+7.5, posy_teo_array+7.5, label = 'Teórica')
            plt.title('Posición del robot teórica y simulada')
            plt.xlabel('Posición en x')
            plt.ylabel('Posición en y')
            #plt.legend(loc='upper left')
            fig1 = plt.figure(1)
            fig1.canvas.draw()
            plt.pause(.01)

        if termina == True:
            plt.clf()
            plt.plot(time_array, error)
            plt.title('Error vs Tiempo')
            plt.xlabel('tiempo')
            plt.ylabel('error relativo')
            fig2 = plt.figure(2)
            fig2.canvas.draw()
            plt.pause(.01)

# Calcula la odometria del robot
def position(msg):
    global posx, posy, posx_array, posy_array, posx_teo_array, posy_teo_array, R1, vel_robot, R2, R1inv, termina
    global inicio, time, time_array, error, angle,orientation

    posx = msg.linear.x
    posy = msg.linear.y
    angle = msg.angular.z
    orientation = angle

    if termina == False:
        if inicio == False:
            posx_teo_array = np.append(posx_teo_array, posx)
            posy_teo_array = np.append(posy_teo_array, posy)
            posx_array = np.append(posx_array, posx)
            posy_array = np.append(posy_array, posy)
            error = np.append(error, np.sqrt((posx_teo_array[-1] - posx_array[-1]) ** 2 + (posy_teo_array[-1] - posy_array[-1]) ** 2))
            time_array = np.append(time_array, time)
        else:
            posx_teo_array = np.append(posx_teo_array, goal_position[0])
            posy_teo_array = np.append(posy_teo_array, goal_position[1])
            posx_array = np.append(posx_array, posx)
            posy_array = np.append(posy_array, posy)
            error = np.append(error, error[-1] + np.sqrt((posx_teo_array[-1] - posx_array[-1]) ** 2 + (posy_teo_array[-1] - posy_array[-1]) ** 2))
            time_array = np.append(time_array, time)
    inicio = True


#Callback que lee el tiempo de simulacion
def timeCallback(msg):
    global time
    time = msg.data

def polar_coordinate():
	global rho, alpha, beta, orientation, goal_position, posx, posy
	destiny()
	rho = math.sqrt((posx-goal_position[0])**2 + (posy-goal_position[1])**2)
	alpha = -orientation + math.atan2(goal_position[1]-posy, goal_position[0]-posx)
	beta = -orientation - alpha + goal_position[2]
	#print("Turtle orientation", orientation)
	
# Suscriptores y Publicadores
rospy.init_node('movimiento', anonymous=True)
pub_vel = rospy.Publisher("/pioneer_motorsVel", Float32MultiArray, queue_size=10)
rospy.Subscriber("/pioneer_position", Twist, position,   queue_size = 1)
rospy.Subscriber("/simulationTime",Float32,timeCallback,   queue_size = 1)
#rospy.Suscriber("/grid_map",, destiny, queue_size=1)

# Controlador del robot
def controlador():
	global vel_robot, vel_ang_z,vel_lin_x,velangular,vellinear, arrived, termina, l, r, k
	k=1
	vel_robot.data = [0,0]
	try:
		rate = rospy.Rate(4)
		while not rospy.is_shutdown():
			
			velangular = vel_ang_z
			vellinear = vel_lin_x
			
			
			polar_coordinate()

			# llegar a la posición en el plano
			if (rho > 0.09) and (arrived== False):
				
				vel_lin_x= np.clip(kp*rho, -1, 1) # Velocidad Lineal del Robot
				vel_ang_z = np.clip(ka*alpha + kb*beta, -math.pi, math.pi) # Velocidad Angular del Robot

			# llegar al angulo deseado
			else:
				if (arrived== False) and (rho < 0.09):
					arrived = True
					vel_lin_x = 0
					
					if (orientation < goal_position[2] + 1) and (orientation > goal_position[2] - 1) and (termina== False) and (arrived==True):
						vel_ang_z = 0
						vel_lin_x = 0
						k = k+1
						if k == 3:
							termina = True
						print("que te miras Prro")
					#termina = True

				else:

					vel_ang_z = np.clip(ka*alpha + kb*beta, -math.pi, math.pi)

			if termina == True:
				vel_lin_x = 0
				vel_ang_z = 0
			
			# R = np.array([[np.cos(angle), np.sin(angle), 0], [-np.sin(angle), np.cos(angle), 0], [0, 0, 1]])
			# A = np.dot(R1,R)
			# s = np.array([vel_lin_x,0,vel_ang_z])
			# F = np.dot(A,s)
			vl = (vel_lin_x - vel_ang_z*l/2)/r
			vr = (vel_lin_x + vel_ang_z*l/2)/r
			vel_robot.data = [float(vl),float(vr)]
			#print(vel_robot)
			pub_vel.publish(vel_robot)

			rate.sleep()

	except rospy.ServiceException as e:
		print("Error!")

# Función Main
if __name__ == '__main__':
	try:
		Thread(target=funcionGrafica).start()
		controlador()
	except rospy.ROSInterruptException:
		pass