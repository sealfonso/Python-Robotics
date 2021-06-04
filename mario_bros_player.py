#!/usr/bin/env python
import pygame #pip3 install pygame
import rospy
import random
import sys
import readline
import numpy as np
import cv2
from threading import Thread
import time
from std_msgs.msg import String 
from std_msgs.msg import Int32
from ros_bridge.msg import MarioStatus
from ros_bridge.srv import SetEnvironment, SetEnvironmentResponse
from tkinter import * #sudo apt-get install python3-tk
from sensor_msgs.msg import CompressedImage


global Environment
Environment = "None"
global ready
ready = ""
global statustime
statustime = 0
global statuslife
statuslife = 0
global partida
partida=""
global file
file = ""

def statusCallback(msg):

	global ready, statustime

	ready = msg.done
	statustime = msg.time
	statuslife = msg.life
	
def handle_partida(req):
	
	global partida, Environment, fileg, k, a, ready, statustime, statuslife, file, file, Environment
	
	pub = rospy.Publisher('mario_bros_action',Int32,queue_size=1)

	partida = req.environment
	
	file = open(partida +".txt","rt")

	Environment = file.read(15)

	file.seek(0)


	if Environment =='SuperMarioBros-':
		Environment = file.read(17)
		
	else:
		Environment = file.read(18)
		

	rospy.wait_for_service('set_mario_environment')
	r=rospy.ServiceProxy('set_mario_environment',SetEnvironment)
	set_mario_environment=r(Environment)

	print(Environment) 
	
	while not rospy.is_shutdown():

		if(statustime<=400 and ready==False and statuslife>=0):
			t=file.readline()
			s=file.readline()
			move = int(s)
			move1= int(t)
			print(move1)
			print(statustime)
			if(statustime==move1):
				print(move)
				pub.publish(move)
				rate.sleep()

	rospy.loginfo("Partida: %s", partida)
	return SetEnvironmentResponse('done')
	

# Method: Create the service's variable and set the handle_set_environment method
def partida_server():
	s = rospy.Service('inicio_partida', SetEnvironment, handle_partida)
	print("Abra una nueva terminal para usar rosservice call")
	rospy.spin()


# input_var = input("¿Desea reproducir una partida guardada?  yes/no:")
# print ("Usted escribió " + input_var)

# if input_var=='yes':

#archivo = input("Escriba el nombre de la partida: ")

# elif input_var=='no':
# 	print("Proceso finalizado.")


rospy.init_node('mario_bros_player',anonymous=True)
rospy.Subscriber("/mario_bros_status", MarioStatus, statusCallback)
rate = rospy.Rate(100)


if __name__=='__main__':

	try:

		pygame.init()
		partida_server()
		#readKeyboard()	
		print(partida)
		#partida_server()
		#lectura_ambiente()
		
	except rospy.ROSInterruptException:
		pass