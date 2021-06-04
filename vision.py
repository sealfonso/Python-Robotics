#!/usr/bin/env python
import pygame #pip3 install pygame
import rospy
import sys
import numpy as np
import cv2
from threading import Thread
import time
from std_msgs.msg import String
from tkinter import * #sudo apt-get install python3-tk
from sensor_msgs.msg import CompressedImage
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import matplotlib.pyplot as plt
import os
import roslib
import sys
try:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
except:
    pass

class Vision:
	def __init__(self):
		
		################ Suscriber al topico que contiene la imagen del sensor ####################################
		rospy.Subscriber("/visionSensorData/image_raw/compressed", CompressedImage, self.callback,  queue_size = 1)
		rospy.Subscriber("/pioneer_position", Twist, self.posicion,   queue_size = 1)

		self.RGB = Float32MultiArray()
		self.R = 0
		self.G = 0
		self.B = 0
		self.image_raw = None
		self.one_blue = False
		self.one_red = False
		self.one_green = False

	def posicion(self, pos_data):
		self.posx = pos_data.linear.x
		self.posy = pos_data.linear.y

	def callback(self, ros_data):
		#### direct conversion to CV2 ####
		self.image_raw = np.fromstring(ros_data.data, np.uint8)
		self.image_raw = cv2.imdecode(self.image_raw, cv2.IMREAD_COLOR)
		self.color_detector()

	def nothing(self,x):
		pass
	
	
	def color_detector(self):	

		frame_resized = cv2.resize(self.image_raw,(10*self.image_raw.shape[1],10*self.image_raw.shape[0]))
		frame_resized = cv2.fastNlMeansDenoisingColored(frame_resized,None,30,20,15,27)

		############## Máscaras ya definidas #######################
		low_blue= np.array([110,200,50])
		upper_blue=np.array([130,255,255])

		low_red= np.array([0,52,17])
		upper_red=np.array([44,255,255])

		low_green= np.array([45,126,101])
		upper_green=np.array([62,212,203])

		############### Observar Mascaras ##########################		
		hsv = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2HSV)
		kernel = np.ones((5,5),np.uint8)
		kernel[0][0:2] = 0
		kernel[0][3:5] = 0
		kernel[4][0:2] = 0
		kernel[4][3:5] = 0
		kernel[1][0] = 0
		kernel[1][4] = 0
		kernel[3][0] = 0
		kernel[3][4] = 0

		############## Azul ########################################
		blue_mask= cv2.inRange(hsv, low_blue, upper_blue) 
		image_one_channel_blue = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, kernel)
		image_one_channel_blue = cv2.morphologyEx(image_one_channel_blue, cv2.MORPH_OPEN, kernel)
		image_one_channel_blue = cv2.erode(image_one_channel_blue, kernel,iterations = 1)
		image_one_channel_blue = cv2.adaptiveThreshold(image_one_channel_blue,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,11,3.5)
		#cv2.imshow('Imagen 1 Canal Azul', image_one_channel_blue)

		############## Rojo ########################################
		red_mask= cv2.inRange(hsv, low_red, upper_red) 
		image_one_channel_red = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
		image_one_channel_red = cv2.morphologyEx(image_one_channel_red, cv2.MORPH_OPEN, kernel)
		image_one_channel_red = cv2.erode(image_one_channel_red, kernel,iterations = 1)
		image_one_channel_red = cv2.adaptiveThreshold(image_one_channel_red,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,11,3.5)
		#cv2.imshow('Imagen 1 Canal Rojo', image_one_channel_red)

		############## Verde ########################################
		green_mask= cv2.inRange(hsv, low_green, upper_green)
		image_one_channel_green = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel)
		image_one_channel_green = cv2.morphologyEx(image_one_channel_green, cv2.MORPH_OPEN, kernel)
		image_one_channel_green = cv2.erode(image_one_channel_green, kernel,iterations = 1)
		image_one_channel_green = cv2.adaptiveThreshold(image_one_channel_green,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,11,3.5)
		#cv2.imshow('Imagen 1 Canal Verde', image_one_channel_green)

		############### Reconocimiento de circulos #################
		minRadius_=37
		maxRadius_=70
		minDistance_=60


		try:
			################################# Azul#######################################
			circles_blue = cv2.HoughCircles(image_one_channel_blue,cv2.HOUGH_GRADIENT,1,minDistance_,
	                             param1=50,param2=30,minRadius=minRadius_,maxRadius=maxRadius_)		
			circles_blue = np.round(circles_blue[0, :]).astype("int")

			
			if circles_blue is not None:
				if self.one_blue == False:
					bx=self.posx
					by=self.posy
					self.B = self.B+1
					self.one_blue = True
				
				if self.posx > bx+1 and self.posx < bx-1 and self.posy > by+1 and self.posy < by-1:
					self.B = self.B+1
					bx=self.posx
					by=self.posy
		except:
			pass

		try:
			################################# Rojo#######################################
			circles_red = cv2.HoughCircles(image_one_channel_red,cv2.HOUGH_GRADIENT,1,minDistance_,
	                             param1=50,param2=30,minRadius=minRadius_,maxRadius=maxRadius_)		
			circles_red = np.round(circles_red[0, :]).astype("int")
 
			if circles_red is not None:
				if self.one_red == False:
					rx=self.posx
					ry=self.posy
					self.R = self.R+1
					self.one_red = True
				
				if self.posx > rx+1 and self.posx < rx-1 and self.posy > ry+1 and self.posy < ry-1:
					self.R = self.R+1
					rx=self.posx
					ry=self.posy
		except:
			pass

		try:
			################################# Verde#######################################

			circles_green = cv2.HoughCircles(image_one_channel_green,cv2.HOUGH_GRADIENT,1,minDistance_,
	                             param1=50,param2=30,minRadius=minRadius_,maxRadius=maxRadius_)		
			circles_green = np.round(circles_green[0, :]).astype("int")
 
			if circles_green is not None:
				if self.one_green == False:
					gx=self.posx
					gy=self.posy
					self.G = self.G+1
					self.one_green = True
				
				if self.posx > gx+1 and self.posx < gx-1 and self.posy > gy+1 and self.posy < gy-1:
					self.G = self.G+1
					gx=self.posx
					gy=self.posy		
		except:
			pass

		self.publicar()
	
	def publicar(self):
		pub_RGB = rospy.Publisher("/pioneer_prizes", Float32MultiArray, queue_size=10)
		self.RGB.data = [int(self.R), int(self.G), int(self.B)]
		pub_RGB.publish(self.RGB)

	def main(args):
	
		while not rospy.is_shutdown():
			try:
				rospy.init_node('vision', anonymous=True)

			except rospy.ROSInterruptException:
				print( "Shutting down ROS Image feature detector module")
				cv2.destroyAllWindows()

if __name__=='__main__':
	Vision().main()
				