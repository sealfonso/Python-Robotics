#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
import os
import sys
import numpy as np
import matplotlib.pyplot as plt
import time
from threading import Thread

#Variables donde se almacena la informacion general del programa
file = 'trayectoria.txt'
vel = Float32MultiArray()
posx_array = np.array([])
posy_array = np.array([])
angle = 0
posx_teo_array = np.array([])
posy_teo_array = np.array([])
error = np.array([])
time_array = np.array([])
inicio = False
termina = False
time = 0
tictac = 0


#Configuación inicial de matplotlib
plt.show(block=False)
plt.ion()

#Variables relacionadas con la cinemática del robot
alpha_1 = np.radians(90)
alpha_2 = np.radians(-90)
betha_1 = 0
betha_2 = np.radians(180)
l = 0.23
r = 0.035
R1 = np.array([[np.sin(alpha_1 + betha_1), -np.cos(alpha_1 + betha_1), -l/2 * np.cos(betha_1)]
                  , [np.sin(alpha_2 + betha_2), -np.cos(alpha_2 + betha_2), -l/2 * np.cos(betha_2)], [np.cos(alpha_1 + betha_1)
                   , np.sin(alpha_1 + betha_1), np.sin(betha_1)]])
R1inv = np.linalg.inv(R1)
R2 = np.array([[r, 0], [0, r]])

#función que se encarga de generar la gráficas y almacenarlas en las carpeta results
def graph():
    global posx_array, posx_teo_array, posy_array, posy_teo_array, inicio, posx, posy

    absPath = os.path.abspath(__file__)
    nPath = os.path.split(os.path.dirname(absPath))[0]

    while not rospy.is_shutdown():

        if inicio == True:
            plt.clf()
            plt.ylim(-2.5, 2.5)
            plt.xlim(-2.5, 2.5)
            plt.plot(posx_array, posy_array, 'r-', label = 'Simulada')
            plt.plot(posx_teo_array, posy_teo_array, label = 'Teórica')
            plt.title('Posición del robot teórica y simulada')
            plt.xlabel('Posición en x')
            plt.ylabel('Posición en y')
            plt.legend(loc='upper left')
            fig1 = plt.figure(1)
            fig1.canvas.draw()
            plt.pause(.01)
            fig1.savefig(nPath + '/results/error_punto3.png')

        if termina == True:
            plt.clf()
            plt.plot(time_array, error)
            plt.title('Error vs Tiempo')
            plt.xlabel('tiempo')
            plt.ylabel('error relativo')
            fig2 = plt.figure(2)
            fig2.canvas.draw()
            plt.pause(.01)
            fig2.savefig(nPath + '/results/trayectoria_punto3.png')


#Callback que lee la posición del robot. Además dentro de esta misma funciona se implementan las ecuaciones de la cinemática
#del robot para calcular la posicion teoriica del robot
def callback(msg):
    global posx, posy, posx_array, posy_array, posx_teo_array, posy_teo_array, R1, vel, R2, R1inv, termina
    global inicio, tictac, time, time_array, error, angle

    posx = msg.linear.x
    posy = msg.linear.y

    if termina == False:
        if inicio == False:
            posx_teo_array = np.append(posx_teo_array, posx)
            posy_teo_array = np.append(posy_teo_array, posy)
            posx_array = np.append(posx_array, posx)
            posy_array = np.append(posy_array, posy)
            error = np.append(error, np.sqrt((posx_teo_array[-1] - posx_array[-1]) ** 2 + (posy_teo_array[-1] - posy_array[-1]) ** 2))
            time_array = np.append(time_array, time)
        else:
            R = np.array([[np.cos(angle), np.sin(angle), 0], [-np.sin(angle), np.cos(angle), 0], [0, 0, 1]])
            Ri = np.linalg.inv(R)
            A = np.dot(Ri, R1inv)
            v = np.array([vel.data[1], vel.data[0]])
            D = np.dot(R2, v)
            D = np.append(D, [0])
            si = np.dot(A, D)
            tictac = time
            dt = tictac - time_array[-1]
            dx = si[0] * dt
            dy = si[1] * dt
            posx_teo_array = np.append(posx_teo_array, posx_teo_array[-1] + dx)
            posy_teo_array = np.append(posy_teo_array, posy_teo_array[-1] + dy)
            posx_array = np.append(posx_array, posx)
            posy_array = np.append(posy_array, posy)
            error = np.append(error, error[-1] + np.sqrt((posx_teo_array[-1] - posx_array[-1]) ** 2 + (posy_teo_array[-1] - posy_array[-1]) ** 2))
            time_array = np.append(time_array, tictac)
    inicio = True

#Callback que lee el tiempo de simulacion
def timeCallback(msg):
    global time
    time = msg.data

#lee la orientación del robot con respecto al marco inercial
def orientation(msg):
    global angle
    angle = msg.data

#funcion que se encarga de leer los archivos y controlar velocidades de las ruedas del robot a partir de la informacion
#leida del archivo
rospy.init_node('punto3',anonymous=True)
pub1 = rospy.Publisher('/turtlebot_wheelsVel', Float32MultiArray, queue_size=10)
    # Se suscribe a los topicos de la informacion del robot y la simulacion
rospy.Subscriber('/turtlebot_position', Twist, callback)
rospy.Subscriber('/simulationTime', Float32, timeCallback)
rospy.Subscriber('/turtlebot_orientation',Float32, orientation)

def controller(arg):
    global pub1, posx, posy, posx_array, posy_array, vel, file, time, termina, time_array, error
  
    #file = arg[0]
    
    vel.data = [0, 0]

    try:
        absPath = os.path.abspath(__file__)
        nabsPath = os.path.split(os.path.dirname(absPath))[0]
        f = open(os.path.join(nabsPath + "/resources/" + file), "r")
        T = int(f.readline())
        rate = rospy.Rate(100)  # 10hz
        final = time
        i = 0

        while not rospy.is_shutdown():
            if (T >= 0):

                if (time > final):
                    T -= 1
                    if T >= 0:
                        dat = f.readline().split()
                        final = time + float(dat[2])
                        vel.data = [float(dat[0]), float(dat[1])]

            else:
                termina = True
                vel.data = [0, 0]

            pub1.publish(vel)
            rate.sleep()

    except rospy.ServiceException as e:
        print("¡Error!")

#funcion principal del programa
if __name__ == '__main__':
    try:
        Thread(target=graph).start()
        controller(sys.argv[1:])
    except rospy.ROSInterruptException:
        pass
  