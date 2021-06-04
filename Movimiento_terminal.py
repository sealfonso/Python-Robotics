#!/usr/bin/env python

import sys
print("{} -m pip install matplotlib".format(sys.executable))
import rospy
from std_msgs.msg import Float32MultiArray, Float32
from geometry_msgs.msg import Twist
import numpy as np
from threading import Thread
import time
import os
import matplotlib.pyplot as plt
print("{} -m pip install matplotlib".format(sys.executable))
import math
import timeit
import argparse

print("actualversion")

# Funcion lectora de datos desde la terminal
parser = argparse.ArgumentParser(description='Indique el destino')
parser.add_argument('-x','--x_position', type=float, metavar='', default=-2, help='Posición en X del robot')
parser.add_argument('-y','--y_position', type=float, metavar='', default=-2, help='Posición en Y del robot')
parser.add_argument('-t','--orientation', type=float, metavar='', default=np.radians(-135), help='Orientación del robot')
args = parser.parse_args()

# Escribe en un vector la posición de destino

def destiny(x_position, y_position, orientation):
	global goal_position
	goal_position = (x_position, y_position, np.radians(orientation))
	return goal_position


# Inicialización de variables

global vel_robot
vel_robot = Twist()
vel_robot.linear.x = 0
vel_robot.linear.y = 0
vel_robot.linear.z = 0
vel_robot.angular.x = 0
vel_robot.angular.y = 0
vel_robot.angular.z = 0

global rho, alpha, beta, arrived
rho = 0
alpha = 0
beta = 0
arrived= False


actual_position = (0, 0, np.radians(-180))

global kp, ka, kb
kp = 0.5
ka = 1.2
kb = -0.2


plt.show(block=False)
plt.ion()

global x_turtle, y_turtle, orientation_turtle, delta_x, delta_y
x_turtle = 0
y_turtle = 0
orientation_turtle = 0
delta_x = 0
delta_y = 0

global x_turtle_vec, y_turtle_vec, x_teo, y_teo, theta_teo
x_turtle_vec = np.array([])
y_turtle_vec = np.array([])
x_teo = np.array([])
y_teo = np.array([])
theta_teo = np.array([])
error_vec = np.array([])

global start, end, time, time_vec,velangular,vellinear,vr,vl,l,r
start = False
end = False
time = 0
time_vec =np.array([])
velangular=0
vellinear=0
vr=0
vl=0
l=0.23
r=0.035

# Calcula la odometria del robot y calcula el vector de error
def turtlebot_position(msg):
	global x_turtle, y_turtle, delta_x, delta_y
	global x_turtle_vec, y_turtle_vec, x_teo, y_teo, error_vec,theta_teo
	global start, end, time, time_vec, vr, vl, l, r
	linear = msg.linear
	x_turtle = linear.x
	y_turtle = linear.y
	delta_x = goal_position[0] - x_turtle
	delta_y = goal_position[1] - y_turtle
	
	x_turtle_vec= np.append(x_turtle_vec, x_turtle)
	y_turtle_vec=np.append(y_turtle_vec,y_turtle)

	if end == False:
		if start == False:
			y_teo = np.append(y_teo,0)
			x_teo = np.append(x_teo,0)
			
			time_vec = np.append(time_vec,time)
			error_vec = np.append(error_vec, np.sqrt((x_teo[-1] - x_turtle_vec[-1]) ** 2 + (y_teo[-1] - y_turtle_vec[-1]) ** 2))

		else:
			# Odometria
			angle= orientation_turtle
			Rinv= np.array([[np.cos(angle), -np.sin(angle), 0], [np.sin(angle), np.cos(angle), 0], [0, 0, 1]])
			vector = np.dot(Rinv, [vellinear,0,velangular])
			vx=vector[0]
			vy=vector[1]

			dt = time - time_vec[-1]
			
			y_teo = np.append(y_teo, y_teo[-1]+ vy* dt) 
			x_teo = np.append(x_teo, x_teo[-1]+ vx* dt) 
			

			error_vec = np.append(error_vec, error_vec[-1] + np.sqrt((x_teo[-1] - x_turtle_vec[-1]) ** 2 + (y_teo[-1] - y_turtle_vec[-1]) ** 2))
			time_vec = np.append(time_vec,time)
	
	start = True
	pass

# Callback de la orientacion del robot
def turtle_orientation(msg):
	global orientation_turtle
	orientation_turtle = msg.data

# Callback del tiempo de simulación
def timeCallback(msg):
	global time
	time = msg.data

# Calculo de las cordenadas polares
def polar_coordinate():
	global rho, alpha, beta
	rho = math.sqrt(delta_x**2 + delta_y**2)
	alpha = -orientation_turtle + math.atan2(delta_y, delta_x)
	beta = -orientation_turtle - alpha + goal_position[2]
	print("Turtle orientation", orientation_turtle)


# Suscriptores y Publicadores
rospy.init_node('punto4', anonymous=True)
pub_vel = rospy.Publisher('/turtlebot_cmdVel', Twist, queue_size=10)
rospy.Subscriber('/turtlebot_position', Twist, turtlebot_position)
rospy.Subscriber('/turtlebot_orientation', Float32, turtle_orientation)
rospy.Subscriber('/simulationTime', Float32, timeCallback)

# Controlador del robot
def controlador():
	global velangular, vellinear, arrived,time, end


	try:

		rate = rospy.Rate(4)  # 10hz
		while not rospy.is_shutdown():
			
			velangular= vel_robot.angular.z
			vellinear= vel_robot.linear.x
			
			
			polar_coordinate()

			# llegar a la posición en el plano
			if (rho > 0.032) & (arrived== False):
				
				vel_robot.linear.x = np.clip(kp*rho, -0.7, 0.7) # Velocidad Lineal del Robot
				vel_robot.angular.z = np.clip(ka*alpha + kb*beta, -math.pi, math.pi) # Velocidad Angular del Robot

			# llegar al angulo deseado
			else:
				if (time > 5) & (arrived== False) & (rho < 0.032):
					arrived= True
					vel_robot.linear.x = 0
					

				if (orientation_turtle < goal_position[2] + 0.05) & (orientation_turtle > goal_position[2] - 0.05) & (end== False) & (arrived==True):
					vel_robot.angular.z = 0
					end = True
					vel_robot.linear.x = 0
					

				if end== True:
					vel_robot.linear.x = 0
					vel_robot.angular.z = 0
					print("Se han guardado exitosamente tus graficas en la carpeta Results")


				else:

					vel_robot.angular.z = np.clip(ka*alpha + kb*beta, -math.pi, math.pi)
			
			pub_vel.publish(vel_robot)

			rate.sleep()

	except rospy.ServiceException as e:
		print("Error!!")

# Función que grafica las posiciones del robot y su error
def funcionGrafica():
	global x_turtle_vec, y_turtle_vec
	global x_teo, y_teo, time_vec, error_vec
	global start

	absPath = os.path.abspath(__file__)
	nPath = os.path.split(os.path.dirname(absPath))[0]


	while not rospy.is_shutdown():

		if start == True:
			plt.clf()
			plt.plot(x_turtle_vec, y_turtle_vec, 'r-', label = 'Simulada')
			plt.plot(x_teo, y_teo, label = 'Teorica')
			plt.title('Posicion del robot teorica y simulada')
			plt.xlabel('Posicion en x')
			plt.ylabel('Posicion en y')
			plt.legend(loc='upper left')
			fig1 = plt.figure(1)
			fig1.canvas.draw()
			plt.pause(.01)
			fig1.savefig(nPath + '/results/trayectoria_punto4.png')

		if end == True:
			
			plt.clf()
			plt.plot(time_vec, error_vec)
			plt.title('Error')
			plt.xlabel('time(s)')
			plt.ylabel('Magnitud')
			fig2 = plt.figure(2)
			fig2.canvas.draw()
			plt.pause(.01)
			fig2.savefig(nPath + '/results/error_punto4.png')
	


# Función Main
if __name__ == '__main__':
	try:
		destiny(args.x_position,args.y_position,args.orientation)
		Thread(target=funcionGrafica).start()
		controlador()
	except rospy.ROSInterruptException:
		pass