#!/usr/bin/env python
import pygame #pip3 install pygame
import rospy
import random
import sys
import numpy as np
import cv2
from threading import Thread
import time
from std_msgs.msg import String 
from std_msgs.msg import Int32
from ros_bridge.msg import MarioStatus
from ros_bridge.srv import SetEnvironment
from tkinter import * #sudo apt-get install python3-tk
from sensor_msgs.msg import CompressedImage
import os


global statuslife 
statuslife=0
global statustime 
statustime=0
global statusmario 
statusmario=0
global statuscoins  
statuscoins=0
global statusscore
statusscore=0
global time_array
time_array=np.array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10])
global actualenvironment
actualenvironment='None'
global ima
global xpos
xpos=0
global ypos
ypos=0
global k
k = 0
global ready
ready = ""


def statusCallback2(image_data):
	imag = np.fromstring(image_data.data, np.uint8)
	ima = cv2.imdecode(imag, cv2.IMREAD_UNCHANGED)
	ima=cv2.cvtColor(ima,cv2.COLOR_BGR2RGB)
	if (239-ypos+10 >=0 and 239-ypos+50<=239):
		cv2.imshow('Mario', ima[239-ypos+10:239-ypos+50,xpos:xpos+20])
		cv2.waitKey(1)
def statusCallback(msg):

	global statuslife, statustime, statusmario, statuscoins, statusscore, ypos, xpos, ready

	statuslife = msg.life
	statustime = msg.time
	statusmario = msg.status
	statuscoins = msg.coins
	statusscore = msg.score
	xpos = msg.x_pos
	ypos = msg.y_pos
	ready = msg.done


rospy.init_node('mario_bros_teleop',anonymous=True)
rospy.wait_for_service('set_mario_environment')
r=rospy.ServiceProxy('set_mario_environment',SetEnvironment)
actualenvironment=random.choice(['SuperMarioBros-v0', 'SuperMarioBros-v1', 'SuperMarioBros-v2', 'SuperMarioBros-v3', 'SuperMarioBros2-v0', 'SuperMarioBros2-v1'])
set_mario_environment=r(actualenvironment)
rospy.Subscriber("/mario_bros_status", MarioStatus, statusCallback)
rospy.Subscriber("/game_image",CompressedImage, statusCallback2)
rate = rospy.Rate(100)

def readKeyboard():

	global time_array, statuslife
	
	pub = rospy.Publisher('mario_bros_action',Int32,queue_size=1)

	right=0
	left=0
	up=0
	down=0
	a=0
	b=0

	f = open("basefile.txt", "w+")

	while not rospy.is_shutdown():

		global k

		for event in pygame.event.get():
		
			if (event.type==pygame.KEYDOWN):
				if (event.key==pygame.K_RIGHT):
					right=1
				if (event.key==pygame.K_LEFT):
					left=1
				if (event.key==pygame.K_UP):
					up=1
				if (event.key==pygame.K_DOWN):
					down=1
				if (event.key==pygame.K_a):
					a=1
				if (event.key==pygame.K_b):
					b=1

			if (event.type==pygame.KEYUP):
				if (event.key==pygame.K_RIGHT):
					right=0
				if (event.key==pygame.K_LEFT):
					left=0
				if (event.key==pygame.K_UP):
					up=0
				if (event.key==pygame.K_DOWN):
					down=0
				if (event.key==pygame.K_a):
					a=0
				if (event.key==pygame.K_b):
					b=0

			if (right==0 and left==0 and up==0 and down==0 and a==0 and b==0):
				k=0
			
			if (right==1 and left==0 and up==0 and down==0 and a==0 and b==0):
				k=1

			if (right==1 and left==0 and up==0 and down==0 and a==1 and b==0):
				k=2
			
			if (right==1 and left==0 and up==0 and down==0 and a==0 and b==1):
				k=3

			if (right==1 and left==0 and up==0 and down==0 and a==1 and b==1):
				k=4
			
			if (right==0 and left==0 and up==0 and down==0 and a==1 and b==0):
				k=5
				
			if (right==0 and left==1 and up==0 and down==0 and a==0 and b==0):
				k=6
			
			if (right==0 and left==1 and up==0 and down==0 and a==1 and b==0):
				k=7

			if (right==0 and left==1 and up==0 and down==0 and a==0 and b==1):
				k=8
			
			if (right==0 and left==1 and up==0 and down==0 and a==1 and b==1):
				k=9
				
			if (right==0 and left==0 and up==0 and down==1 and a==0 and b==0):
				k=10
			
			if (right==0 and left==0 and up==1 and down==0 and a==0 and b==0):
				k=11

			
		if (statustime<=400 and ready==False and statuslife>=0):
			pub.publish(k)
			f.write(str(statustime)+'\n'+str(k)+'\n')
			rate.sleep()

	f.close()

def keys():
		
	global time_array, actualenvironment

	master=Tk()
	master.title('Estado del juego')
	master.geometry("200x150")
	Label(master, text='Vidas restantes').grid(row=0, column=0) 
	Label(master, text='Monedas').grid(row=1, column=0)
	Label(master, text='Puntaje').grid(row=2, column=0) 
	Label(master, text='Tiempo').grid(row=3, column=0) 
	Label(master, text='Estado de Mario').grid(row=4, column=0)
	contador=0

	while 1:

		global statuslife, statustime, statusmario, statuscoins, statusscore

		vidas = statuslife
		tiempo = statustime
		estado = statusmario
		monedas = statuscoins
		puntaje = statusscore

		v=StringVar(master)
		v.set(str(vidas))
		m=StringVar(master)
		m.set(str(monedas))
		t=StringVar(master)
		t.set(str(tiempo))
		e=StringVar(master)
		e.set(estado)
		s=StringVar(master)
		s.set(puntaje)


		Label(master, textvariable=str(v)).grid(row=0, column=1) 
		Label(master, textvariable=str(m)).grid(row=1, column=1)
		Label(master, textvariable=str(s)).grid(row=2, column=1)
		Label(master, textvariable=t).grid(row=3, column=1) 
		Label(master, textvariable=e).grid(row=4, column=2)
		master.update_idletasks()
		master.update()

		time_array=np.append(time_array,tiempo)
		lenght_time= len(time_array)
		input_var = 0



		if (time_array[lenght_time-9] ==tiempo and vidas<=0):
			
			contador= contador+1

			if contador==3:
				input_var = input("¿Guardar partida? yes/no: ")
				print ("Usted escribió " + input_var)

		if input_var=='yes':

				archivo = input("Asigne un nombre a su archivo de texto: ")
				print("Su archivo se llamará: "+ archivo+".txt")
			
				src = open("basefile.txt", "r")
				fline = actualenvironment
				newline = src.readlines()
				newline.insert(0,fline)
				src.close()

				src = open("basefile.txt", "w")
				src.writelines(newline)
				src.close()
				os.rename("basefile.txt", archivo + ".txt")
				print("Proceso finalizado.") 

		elif input_var=='no':
			print("Proceso finalizado.")


if __name__=='__main__':

	try:

		pygame.init()
		pgscreen=pygame.display.set_mode((1, 1))
		pygame.display.set_caption('keyboard')
		Thread(target=readKeyboard).start()
		Thread(target=keys).start()
		
	except rospy.ROSInterruptException:
		pass


