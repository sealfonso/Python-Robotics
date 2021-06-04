#!/usr/bin/env python

import rospy
import sys, select, tty, termios
import os
#print("{} -m pip install matplotlib".format(sys.executable)) -> copiar y pegar lo que salga en otra terminal y ejecutar
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from tkinter import * #sudo apt-get install python3-tk
from tkinter import ttk
from threading import Thread
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
import time
import numpy as np

#Definición que mapea las teclas
global key_mapping
key_mapping = { 'w': [ 0, 0], 'x': [0, 0],
		'a': [0, 0], 'd': [0, 0],
		's': [ 0, 0] }

#Inicialización de variables de velocidad por interfaz gráfica
global av_entry
av_entry=0
global re_entry
re_entry=0
global gih_entry
gih_entry=0
global gia_entry
gia_entry=0
global av_entrytxt
av_entrytxt=0
global re_entrytxt
re_entrytxt=0
global gih_entrytxt
gih_entrytxt=0
global gia_entrytxt
gia_entrytxt=0
global control_w
control_w=0
global entries
entries=[0,0,0,0]

#Inicialización de variables de posición y de sus respectivos arreglos
global posx
posx=0
global posy
posy=0
global posz
posz=0
global posx_array
posx_array=np.array([])
global posy_array
posy_array=np.array([])
global inicio
inicio = False


#Función que se convoca con el botón de la ventana de velocidades
def entrada1():
	global av_entry, av_entrytxt, re_entry, re_entrytxt, gih_entry, gih_entrytxt, gia_entry, gia_entrytxt, entries, key_mapping
	av_entrytxt=av_entry.get()
	re_entrytxt=re_entry.get()
	gih_entrytxt=gih_entry.get()
	gia_entrytxt=gia_entry.get()
	entries=[av_entrytxt,re_entrytxt,gih_entrytxt,gia_entrytxt]
	key_mapping = { 'w': [ 0, int(entries[0])], 'x': [0, -int(entries[1])],
		'a': [int(entries[2]), 0], 'd': [-int(entries[3]), 0],
		's': [ 0, 0] }
	print(entries[0],entries[1],entries[2],entries[3])


#Función que se convoca a través de la suscripción del tópico de la posición del robot
def callback(msg):

	global posx, posy, posz, posx_array, posy_array, inicio
	posx=msg.linear.x
	posx_array = np.append(posx_array,posx)
	posy=msg.linear.y
	posy_array = np.append(posy_array,posy)
	posz=msg.angular.z
	inicio=True
	pass


#Función que ejecuta la gráfica
def graph():
	
	global posx, posy, posz, posx_array, posy_array, inicio
    
	#Definición de la ruta donde se guarda la imagen de la gráfica
	absPath = os.path.abspath(__file__)
	nPath = os.path.split(os.path.dirname(absPath))[0]


	while not rospy.is_shutdown():
	    if inicio == True:
		    plt.clf()
		    plt.ylim(-2.5, 2.5)
		    plt.xlim(-2.5, 2.5)
		    plt.plot(posx_array, posy_array)
		    fig1 = plt.gcf()
		    fig1.canvas.draw()
		    plt.pause(.01)
		    fig1.savefig(os.path.join(nPath + '/results/trayectoria_punto2.png'))

		
		
	
#Función que ejecuta la ventana para el control de velocidades
def vel_control():

	global key_mapping, control_w, av_entry, re_entry, gih_entry, gia_entry, av_entrytxt, re_entrytxt, gih_entrytxt, gia_entrytxt,entries

	control_w=Tk()
	control_w.title('Control de velocidad - Turtlebot')
	control_w.geometry("400x180")
	av=Label(control_w, text='Avance (w)')
	av.place(x=20,y=10)
	re=Label(control_w, text='Retroceso (x)')
	re.place(x=200,y=10)
	gih=Label(control_w, text='Giro horario (d)')
	gih.place(x=20,y=80)
	gia=Label(control_w, text='Giro antihorario (a)')
	gia.place(x=200,y=80)
	av_entry=Entry(control_w)
	av_entry.place(x=20,y=40)
	re_entry=Entry(control_w)
	re_entry.place(x=200,y=40)
	gih_entry=Entry(control_w)
	gih_entry.place(x=20,y=110)
	gia_entry=Entry(control_w)
	gia_entry.place(x=200,y=110)
	button=ttk.Button(control_w,text="ENTER")
	button.place(x=300,y=140)
	button.config(command=entrada1)
	control_w.mainloop()
	

#Función que mapea y asigna los valores asociados a las teclas 
def keys_cb(msg,twist_pub):
	global key_mapping

	if len(msg.data) == 0 or not key_mapping.__contains__(msg.data[0]):
		return # unknown key
	vels = key_mapping[msg.data[0]]
	t = Twist()
	t.angular.z = vels[0]
	t.linear.x = vels[1]
	twist_pub.publish(t)

#Inicialización del nodo de ROS
rospy.init_node('punto2',anonymous=True)

#Publicación en el tópico de /turtlebot_cmdVel
twist_pub = rospy.Publisher('/turtlebot_cmdVel', Twist, queue_size=1)

#Suscripción al tópico /turtlebot_position
rospy.Subscriber('/turtlebot_position', Twist, callback)
rospy.Subscriber('keys', String, keys_cb, twist_pub)
key_pub = rospy.Publisher('keys', String, queue_size=1)
rate = rospy.Rate(100)

#Orden de ejecución según el pulsado de teclas
def keys_order():
	
	old_attr = termios.tcgetattr(sys.stdin)
	tty.setcbreak(sys.stdin.fileno())
	print("Publicando teclas. Presione Ctrl-C para salir...")

	while not rospy.is_shutdown():
		if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:
			key_pub.publish(sys.stdin.read(1))
		rate.sleep()

	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
	rospy.spin()



if __name__ == '__main__':
	
	try:
		#Ejecución por medio de hilos
		Thread(target=keys_order).start()
		Thread(target=vel_control).start()
		plt.show()
		Thread(target=graph).start()
	

	except rospy.ROSInterruptException:
		pass
	

