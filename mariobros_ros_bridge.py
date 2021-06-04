#!/usr/bin/env python3
# //======================================================================//
# //  This software is free: you can redistribute it and/or modify        //
# //  it under the terms of the GNU General Public License Version 3,     //
# //  as published by the Free Software Foundation.                       //
# //  This software is distributed in the hope that it will be useful,    //
# //  but WITHOUT ANY WARRANTY; without even the implied warranty of      //
# //  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE..  See the      //
# //  GNU General Public License for more details.                        //
# //  You should have received a copy of the GNU General Public License   //
# //  Version 3 in the file COPYING that came with this distribution.     //
# //  If not, see <http://www.gnu.org/licenses/>                          //
# //======================================================================//
# //                                                                      //
# //      Copyright (c) Robotica 2020 - Taller 1                          //             
# //      Universidad de los Andes - Colombia                             //
# //                                                                      //
# //======================================================================//
#### Import packages
import sys
## Import the local services and messages
from ros_bridge.srv import SetEnvironment, SetEnvironmentResponse
from ros_bridge.msg import MarioStatus
from std_msgs.msg import Int32
# Python's library for ROS
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
import timestamp
from nes_py.wrappers import JoypadSpace
import gym_super_mario_bros
from gym_super_mario_bros.actions import COMPLEX_MOVEMENT
import numpy as np
import threading

# Create the principal class of the ROS node
class MarioRosBridge:
	# Initialze the variables and parameters of the class
	def __init__(self):
		# Actions for more complex movement
		self.COMPLEX_MOVEMENT = [
    		['NOOP'],
    		['right'],
    		['right', 'A'],
    		['right', 'B'],
    		['right', 'A', 'B'],
    		['A'],
    		['left'],
    		['left', 'A'],
    		['left', 'B'],
    		['left', 'A', 'B'],
    		['down'],
    		['up'],
		]
		self.env = None
		self.env_name = None
		self.action = 0
		self.env_status = -1
		self.msg_mario_status = MarioStatus()
		self.state = None
		self.image_pub = None
		self.status_pub = None

	# Method: Initialize the set_environment_services to begin the game
	def handle_set_environment(self,req):
		self.env_name = req.environment
		self.env_status = 0
		rospy.loginfo("Environment %s", self.env_name)
		return SetEnvironmentResponse('done')

	# Method: Create the service's variable and set the handle_set_environment method
	def set_environment_server(self):
		s = rospy.Service('set_mario_environment', SetEnvironment, self.handle_set_environment)
		
	# Method: Callback method that recives action from the \mario_bros_action topic
	def actionCallback(self,msg):
		#Receive action from topic /mario_bros_action
		if self.env_name!=None:
			self.action = msg.data
			rospy.loginfo("Received Action %s", self.COMPLEX_MOVEMENT[self.action])
		else:
			rospy.loginfo("Environment missing. Please call service /set_mario_enviroment")

	# Method: Convert the game's image, compressed it and publish it in the \game_image topic
	def imageConverter(self,image): 
		compressedImage = CompressedImage()
		compressedImage.header.stamp = rospy.Time.now()
		compressedImage.format = "jpeg"
		compressedImage.data = np.array(cv2.imencode('.jpg', image)[1]).tostring()
		self.image_pub.publish(compressedImage)

	#Method: Main method
	def main(self):
		#Initialize node 
		rospy.init_node('mario_bros_core')
		#Create service to stablish mario bros' environment
		self.set_environment_server()
		#Subscribe to topic /mario_bros_action to receive and apply action to the game
		rospy.Subscriber("mario_bros_action", Int32, self.actionCallback)
		#Create publisher to publish the game status in  /mario_bros_status topic
		self.status_pub = rospy.Publisher('mario_bros_status', MarioStatus, queue_size=10)
		self.image_pub = rospy.Publisher('game_image', CompressedImage, queue_size = 10)
		rate = rospy.Rate(100)
		print("========== The ros_bridge node is waiting for service ==========================")
		#While the node is alive
		while not rospy.is_shutdown():
			if self.env_status == 0:
				#If received an environment name, create the game
				self.env = gym_super_mario_bros.make(self.env_name)
				self.env = JoypadSpace(self.env, self.COMPLEX_MOVEMENT)
				self.env.reset()
				self.env.render()
				self.env_status = -1

			if self.env_name!=None and self.env_status == -1:
				#Apply action
				self.state, reward, done, info = self.env.step(self.action)
				#Show game image 
				self.env.render()
				self.msg_mario_status.reward = int(reward)
				self.msg_mario_status.done = done
				self.msg_mario_status.coins = int(info['coins'])
				self.msg_mario_status.life = int(info['life'])
				self.msg_mario_status.score = int(info['score'])
				self.msg_mario_status.status = info['status']
				self.msg_mario_status.time = int(info['time'])
				self.msg_mario_status.x_pos = int(info['x_pos_screen'])
				self.msg_mario_status.y_pos = int(info['y_pos'])
				try:
					self.status_pub.publish(self.msg_mario_status)
				except:
					print("====== Game Over =====")
					break
				self.imageConverter(self.state)
				#self.env_status = -1
				rate.sleep()
		self.env.close()

if __name__ == '__main__':
	marioRosBridge = MarioRosBridge()
	marioRosBridge.main()
	
