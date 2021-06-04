#!/usr/bin/env python
import matplotlib.pyplot as plt #print("{} -m pip install matplotlib".format(sys.executable))
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QLineEdit, QPushButton, QComboBox
from PyQt5.QtGui import QIcon, QPixmap
from PyQt5.QtCore import pyqtSlot
from std_msgs.msg import Float32MultiArray, Float32, Int32
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from PyQt5.QtCore import QSize  
from PyQt5 import QtCore, QtWidgets
from PyQt5.QtWidgets import QMainWindow, QWidget, QLabel, QLineEdit
from PyQt5.QtWidgets import QPushButton
from PyQt5.QtCore import QSize, Qt 
from threading import Thread
import numpy as np
import sys
import rospy 
import math
import time
import os
import timeit
import sys
try:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
except:
    pass
import cv2

##########################################SPECS################################################
#La interfaz debe tener:

# 1. Velocidad de Cada Rueda ~ LISTO
# 2. De Forma gráfica dirección de movimiento y orientación del robot
# 3. Número de esferas encontradas ~ LISTO
# 4. Mostrar el dibujo a realizar ~ LISTO

# Extras:

# 1. Ver lo que está viendo el robot 
# 2. Decirle cual es el algoritmo  ~ LISTO
# 3. Dar el punto inicial y final  ~ LISTO
# 4. Dar la ubicación del área de dibujo  ~ LISTO (el punto de entrada hacia el área de dibujo)
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


class Interfaz(QMainWindow):
	def __init__(self):
		
		self.image_raw=None
		self.wheels_vel='Actualizando...'
		self.spheres='Actualizando...'
		self.rutaimagen=''
		self.punto_inicial = Float32MultiArray()
		self.punto_final = Float32MultiArray()
		self.punto_dibujo = Float32MultiArray()
		self.numero_dibujo = 6
	###########################################
		#Suscribers etc
	###########################################	

		rospy.Subscriber("/visionSensorData/image_raw/compressed", CompressedImage, self.callbackImage,  queue_size = 1)
		rospy.Subscriber("/pioneer_motorsVel", Float32MultiArray, self.callbackWheels,  queue_size = 1)
		rospy.Subscriber("/pioneer_prizes", Float32MultiArray, self.callbackSpheres,  queue_size = 1)
		rospy.Subscriber("/pioneer_figure", Float32MultiArray, self.callbackFigure,  queue_size = 1)

		self.ini = rospy.Publisher("/punto_inicial", Float32MultiArray, queue_size=10)
		self.fin = rospy.Publisher("/punto_final", Float32MultiArray, queue_size=10)
		self.dib = rospy.Publisher("/punto_dibujo", Float32MultiArray, queue_size=10)
	###########################################
		#Ventana
	###########################################	

		QMainWindow.__init__(self)

		self.setMinimumSize(QSize(1000,800))	
		self.setWindowTitle("Pioneer Interface- Grupo 10") 
		self.setAutoFillBackground(True)
		p = self.palette()
		p.setColor(self.backgroundRole(), QtCore.Qt.darkCyan)
		self.setPalette(p)

		# Asignación de la ruta de archivos actual
		self.absPath = os.path.abspath(__file__)
		self.nPath = os.path.split(os.path.dirname(self.absPath))[0]

	##########################################################
		#Punto Inicial, Final, nombre del archivo y algoritmo
	##########################################################
		
		espacio =40
		espacio1=45
		### Renglones:
		fila0=espacio #Configuración
		fila1=fila0+espacio1 #Inicial
		fila2=fila1+espacio #Final
		fila3=fila2+espacio #Dibujo

		filacentrodibujo=fila3+espacio

		fila4=filacentrodibujo+espacio #Orientación Dibujo

		fila5=fila4+espacio# Seleccionar algoritmo
		fila6=fila5+espacio #Box
		fila7=fila6+espacio # Boton

		fila8=fila7+espacio # Velocidad
		filabono= fila8 + espacio

		fila9=filabono+espacio1 # Caja
		fila10=fila9+espacio1 #Esferas
		fila11=fila10+espacio # Caja
		fila12=fila11+espacio1 # Caja
		
		
		fila13= fila12+espacio1 #Dibujo a realizar


		fila14= fila1
		self.fila15= fila2 #Dibujo

		self.col2 = 500
		size_line=80

		self.nameLabel0 = QLabel(self)
		self.nameLabel0.setText('Configuración:')
		self.nameLabel0.setFixedWidth(200)
		self.nameLabel0.move(20, fila0)	
		
		self.nameLabel = QLabel(self)
		self.nameLabel.setText('Inicial:')
		self.nameLabel.move(20, fila1)

		self.linexi = QLineEdit(self)
		self.linexi.move(150, fila1)
		self.linexi.resize(size_line, 32)

		self.lineyi = QLineEdit(self)
		self.lineyi.move(160+size_line, fila1)
		self.lineyi.resize(size_line, 32)

		self.linezi = QLineEdit(self)
		self.linezi.move(170+2*size_line, fila1)
		self.linezi.resize(size_line, 32)

		self.nameLabel2 = QLabel(self)
		self.nameLabel2.setText('Final:')
		self.nameLabel2.move(20, fila2)

		self.linexf = QLineEdit(self)
		self.linexf.move(150, fila2)
		self.linexf.resize(size_line, 32)

		self.lineyf = QLineEdit(self)
		self.lineyf.move(160+size_line, fila2)
		self.lineyf.resize(size_line, 32)

		self.linezf = QLineEdit(self)
		self.linezf.move(170+2*size_line, fila2)
		self.linezf.resize(size_line, 32)

		self.nameLabel2 = QLabel(self)
		self.nameLabel2.setText('Dibujo:')
		self.nameLabel2.move(20, fila3)

		self.linexd = QLineEdit(self)
		self.linexd.move(150, fila3)
		self.linexd.resize(size_line, 32)

		self.lineyd = QLineEdit(self)
		self.lineyd.move(160+size_line, fila3)
		self.lineyd.resize(size_line, 32)

		self.linezd = QLineEdit(self)
		self.linezd.move(170+2*size_line, fila3)
		self.linezd.resize(size_line, 32)

		self.nameLabel2 = QLabel(self)
		self.nameLabel2.setText('Centro Dibujo:')
		self.nameLabel2.setFixedWidth(250)
		self.nameLabel2.move(20, filacentrodibujo)

		self.linexcd = QLineEdit(self)
		self.linexcd.move(240, filacentrodibujo)
		self.linexcd.resize(size_line, 32)

		self.lineycd = QLineEdit(self)
		self.lineycd.move(250+size_line, filacentrodibujo)
		self.lineycd.resize(size_line, 32)


		self.nameLabel3 = QLabel(self)
		self.nameLabel3.setText('Orientación Dibujo (º):')
		self.nameLabel3.setFixedWidth(300)	
		self.nameLabel3.move(20, fila4)	

		self.lineOrientacion = QLineEdit(self)
		self.lineOrientacion.setText("0")
		self.lineOrientacion.move(330, fila4)
		self.lineOrientacion.resize(size_line, 32)

		self.nameLabel3 = QLabel(self)
		self.nameLabel3.setText('Escala del dibujo:')
		self.nameLabel3.setFixedWidth(310)	
		self.nameLabel3.move(20, fila5)	

		self.lineEscala = QLineEdit(self)
		self.lineEscala.setText("1")
		self.lineEscala.move(330, fila5)
		self.lineEscala.resize(size_line, 32)

		self.nameLabel5 = QLabel(self)
		self.nameLabel5.setText('Nombre .pgm:')
		self.nameLabel5.setFixedWidth(600)	
		self.nameLabel5.move(20, fila6)

		self.linel5 = QLineEdit(self)
		self.linel5.setText('map_scene2')
		self.linel5.move(160+size_line, fila6)
		self.linel5.setFixedWidth(2*size_line+10)	
		self.linel5.resize(size_line, 32)

		self.nameLabel3 = QLabel(self)
		self.nameLabel3.setText('Seleccionar algoritmo:')
		self.nameLabel3.setFixedWidth(500)	
		self.nameLabel3.move(20, fila7)		
		
		self.cb = QComboBox(self)
		self.cb.addItems(["Elija un Algoritmo","Djikstra", "A* Manhatan", "A* Euclidian"])
		self.cb.currentIndexChanged.connect(self.algoritmo)
		self.cb.setFixedWidth(400)
		self.cb.move(20, fila8)	

		pybutton = QPushButton('Correr algoritmo', self)
		pybutton.clicked.connect(self.configuracion)
		#pybutton.clicked.connect(self.name)
		pybutton.resize(400,32)
		pybutton.move(20, fila9)

		self.nameLabel3 = QLabel(self)
		self.nameLabel3.setText('Bono:')
		self.nameLabel3.setFixedWidth(80)	
		self.nameLabel3.move(20, filabono)		
		
		self.cbono = QComboBox(self)
		self.cbono.addItems(["Correr el bono?","Sí",'No'])
		self.cbono.currentIndexChanged.connect(self.bono)
		self.cbono.setFixedWidth(300)
		self.cbono.move(120, filabono)

		## Puntos predeterminados

		self.linexi.setText('-6.5')
		self.lineyi.setText('6.5')
		self.linezi.setText('-1.57')

		self.linexf.setText('-2.331')
		self.lineyf.setText('-6.75')
		self.linezf.setText('0')

		self.linexd.setText('3.366')
		self.lineyd.setText('-4.5')
		self.linezd.setText('1.57')
	###########################################
		#Velocidades de las ruedas
	###########################################

		self.nameLabel3 = QLabel(self)
		self.nameLabel3.setText('Velocidad de las Ruedas:')
		self.nameLabel3.setFixedWidth(500)	
		self.nameLabel3.move(20, fila10)
		
		self.nameLabelVel = QLabel(self)
		self.nameLabelVel.setFixedWidth(400)	
		self.nameLabelVel.move(20, fila11)
		self.nameLabelVel.setStyleSheet('background-color: white')

		self.my_timer = QtCore.QTimer()
		self.my_timer.timeout.connect(self.WheelsInfo)
		self.my_timer.start(100) #0.1 seg intervall
		

	###########################################
		#Conteo de Esferas
	###########################################

		self.nameLabel = QLabel(self)
		self.nameLabel.setText('Conteo de Esferas (R,G,B):')
		self.nameLabel.setFixedWidth(500)	
		self.nameLabel.move(20, fila12)
		
		self.nameLabelSphere = QLabel(self)
		self.nameLabelSphere.setFixedWidth(200)	
		self.nameLabelSphere.move(80, fila13)
		self.nameLabelSphere.setStyleSheet('background-color: white')

		self.my_timer1 = QtCore.QTimer()
		self.my_timer1.timeout.connect(self.spheresInfo)
		self.my_timer1.start(100) #0.1 seg intervall

	
	###########################################
		#Mostrar Dibujo
	###########################################	

		self.nameLabel = QLabel(self)
		self.nameLabel.setText('Dibujo a realizar:')
		self.nameLabel.setFixedWidth(500)	
		self.nameLabel.move(self.col2, fila14)

		self.label = QLabel(self)
		self.my_timer3 = QtCore.QTimer()
		self.my_timer3.timeout.connect(self.dibujo)
		self.my_timer3.start(100) #0.1 seg intervall
		

		self.show()

	#$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
	## Devuelve el algoritmo seleccionado en la interfaz
	#$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
	def algoritmo(self,i):
		# 0 Djikstra
		# 1 A* Manhattan
		# 2 A* Euclidean
		self.tipoalgoritmo= str(i)
		file = open(os.path.join(self.nPath +"/resources/algoritmo.txt"), "w+")
		file.write(str(i))
		file.close()


	#$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
	## Obtiene si se realiza bono
	#$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
	
	def bono(self,i):
		self.bono = str(i)	
	#$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
	## Devuelve la configuración establecida en la terminal
	## Guarda en puntos.txt:
	# Linea 1: Punto Inicial
	# Linea 2: Punto Final
	# Linea 3: Punto de inicio del dibujo
	# Linea 4: Punto centro area dibujo
	# Linea 5: Orientación: 0 a 360
	# Linea 6: Escala del dibujo: 1 equivale a 100%
	# Linea 7: Bono: 1=Sí 2=No
	#$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
	def configuracion(self):
		image_raw = None

		file = open(os.path.join(self.nPath +"/resources/puntos.txt"), "w+")
		
		
		file.write(
		str(self.linexi.text())+' '+str(self.lineyi.text())+' '+str(self.linezi.text())+'\n'+
		str(self.linexf.text())+' '+str(self.lineyf.text())+' '+str(self.linezf.text())+'\n'+
		str(self.linexd.text())+' '+str(self.lineyd.text())+' '+str(self.linezd.text())+'\n'+
		str(self.linexcd.text())+' '+str(self.lineycd.text()) +'\n'+
		str(self.lineOrientacion.text())+'\n'+
		str(self.lineEscala.text())+'\n'+
		str(self.bono))
		file.close()

		self.punto_inicial = Float32MultiArray()
		self.punto_final = Float32MultiArray()
		self.punto_dibujo = Float32MultiArray()

		self.punto_inicial.data = [float(self.linexi.text()), float(self.lineyi.text()), float(self.linezi.text()) ]
		self.punto_final.data = [float(self.linexf.text()), float(self.lineyf.text()), float(self.linezf.text()) ]
		self.punto_dibujo.data = [float(self.linexd.text()), float(self.lineyd.text()), float(self.linezd.text()) ]
		self.ini.publish(self.punto_inicial)
		self.fin.publish(self.punto_final)
		self.dib.publish(self.punto_dibujo)
		print('Enviando...')



		try:
			self.punto_inicial1 = [float(self.linexi.text()), float(self.lineyi.text()), float(self.linezi.text())]
			self.punto_final1 = [float(self.linexf.text()), float(self.lineyf.text()), float(self.linezf.text())]
			self.punto_dibujo1 = [float(self.linexd.text()), float(self.lineyd.text()), float(self.linezd.text())]			
			print('Punto Inicial: ' + str(self.punto_inicial1) )
			print('Punto Final: ' + str(self.punto_final1) )
			print('Punto Dibujo: ' + str(self.punto_dibujo1) )
			print('Punto Centro:' + str(self.linexcd.text())+' '+str(self.lineycd.text()))
			print("Algoritmo ",self.cb.currentText(), "Index", self.tipoalgoritmo)
			print('Orientación: ' + str(self.lineOrientacion.text()) )
			print('Escala',str(self.lineEscala.text()))
			print('Realizar bono:', self.cbono.currentText() )

		except:
			print('Revise los datos ingresados, por favor')

	#$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
	## Devuelve la velocidad de las llantas
	#$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

	def callbackWheels(self,ros_data):	
		self.wheels_vel= ros_data.data
		self.vel_x= ros_data.data[0]
		self.vel_y= ros_data.data[1]
		self.wheels_vel= [round(float(self.vel_x),2), round(float(self.vel_y),2)]

	def WheelsInfo(self):
		try:
			self.nameLabelVel.setText(str(self.wheels_vel))
			
		except:
			self.nameLabelVel.setText('(0,0,0)')			
		self.update()
		

	def callbackSpheres(self,ros_data):	
		self.spheres= ros_data.data

	def spheresInfo(self):
		try:
			self.nameLabelSphere.setText(str(self.spheres))
			
		except:
			self.nameLabelSphere.setText('(0,0,0)')			
		self.update()

	def callbackFigure(self,ros_data):	
		self.numero_dibujo= ros_data.data		

	def dibujo(self):
		absPath = os.path.abspath(__file__)
		nPath = os.path.split(os.path.dirname(absPath))[0] 
		
		try: 
			if self.numero_dibujo == 1:
				self.rutaimagen= (nPath + '/resources/Imagenes/contorno_Gato.png')
			
			if self.numero_dibujo == 2:
				self.rutaimagen= (nPath + '/resources/Imagenes/contorno_Jirafa.png')

			if self.numero_dibujo == 3:
				self.rutaimagen= (nPath + '/resources/Imagenes/contorno_Pato.png')
			
			if self.numero_dibujo == 4:
				self.rutaimagen= (nPath + '/resources/Imagenes/contorno_Pez.png')

			if self.numero_dibujo == 5:
				self.rutaimagen= (nPath + '/resources/Imagenes/contorno_Murcielago.png')

			if self.numero_dibujo == 6:
				self.rutaimagen= (nPath + '/resources/Imagenes/nosignal.jpeg')

		except:
			self.numero_dibujo == 6
			

		self.pixmap = QPixmap(str(self.rutaimagen))
		self.label.setPixmap(self.pixmap)
		self.label.resize(450,400)		
		self.label.setPixmap(self.pixmap.scaled(self.label.size(), QtCore.Qt.IgnoreAspectRatio))
		self.label.move(self.col2, self.fila15)

		try:
			self.ini.publish(self.punto_inicial)
			self.fin.publish(self.punto_final)
			self.dib.publish(self.punto_dibujo)
		except:
			pass

	def callbackImage(self,ros_data):
		
		self.image_raw = np.fromstring(ros_data.data, np.uint8)
		self.image_raw = cv2.imdecode(self.image_raw, cv2.IMREAD_COLOR)
		self.image_raw = cv2.resize(self.image_raw,(10*self.image_raw.shape[1],10*self.image_raw.shape[0]))
	
	def name(self):
		
		self.nom= str(self.linel5.text())
		f = open(os.path.join(self.nPath +"/resources/map_name.txt"), "w+")
		f.write(str(self.linel5.text()))
		f.close()
		print('Nombre Archivo:' + self.nom)


rospy.init_node('interfaz', anonymous=True)



if __name__ == "__main__":
	try:
		
		app = QtWidgets.QApplication(sys.argv)
		mainWin = Interfaz()
		mainWin.show()
		sys.exit( app.exec_() )
	except rospy.ROSInterruptException:
		pass


