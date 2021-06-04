#!/usr/bin/env python

import rospy
from threading import Thread
import matplotlib.pyplot as plt
from std_msgs.msg import String
import numpy as np
import argparse
import math
import os
import sys
try:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
except:
    pass
import cv2
#from interfaz import Interfaz

# Inicialización del nodo de ROS
rospy.init_node('escena',anonymous=True)

t=0

# top left, top right, bottom left, bottom right
pts = [(0,0),(0,0),(0,0),(0,0)]

# Índice de conteo de selección de esquinas
pointIndex = 0

# Asignación de la ruta de archivos actual
absPath = os.path.abspath(__file__)
nPath = os.path.split(os.path.dirname(absPath))[0]

file = "map_name.txt"
f = open(os.path.join(nPath + "/resources/" + file), "rt")
run = open(os.path.join(nPath + "/resources/orden.txt"), "rt")     

# Método que asigna el video mediante el parámetro
def openfile():
    global gridmap, data

    data = str(f.read())
    a=len(data)
    f.seek(0)
    data = str(f.readline(a))
    gridmap = cv2.imread(os.path.join(nPath + "/results/"+data+".pgm"), -1)
    return gridmap


#mouse callback function
def mouse(event,x,y,flags,param):
    global img
    global pointIndex
    global pts

    if event == cv2.EVENT_LBUTTONDBLCLK:
        cv2.circle(img,(x,y),0,(255,0,0),-1)
        pts[pointIndex] = (x,y)
        pointIndex = pointIndex + 1

cv2.namedWindow('Original')
cv2.setMouseCallback('Original',mouse)

# Método que se encarga de redimencionar y desplegar la ventana con el primer frame del vídeo
def selectPoints():
    
    global img
    global pointIndex

    print("Selecciona las cuatro esquinas del laberinto, haciendo doble click en las coordenadas (x , y) en el siguiente orden: \n\
    (-7.5 , 7.5), (7.5 , 7.5), (-7.5, -7.5), (7.5 , -7.5).")


    while(pointIndex != 4):
        h = int(gridmap.shape[0])
        w = int(gridmap.shape[1])

        img = cv2.resize(img,(w,h))

        cv2.imshow('Original',img)

        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            return False
    
    return True

# Método que se encarga de la transformación de perspectiva, el filtrado de colores e identificación de robots
def grid_map():
    global gridmap, img, RESOLUCION, gridmap_resized
    img = gridmap
    
    # dilatar los obstaculos, tal que las celdas en blanco simbolicen aquellos espacios
    # libres por donde puede pasar el centro del robot. Asi se garantiza que no se acerque
    # a los obstaculos
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11, 11))
    gridmap_dilatated = cv2.dilate( cv2.bitwise_not(gridmap), kernel, iterations=1)
    gridmap_dilatated = cv2.bitwise_not(gridmap_dilatated)
    
    # la resolucion original de la imagen es 1px=5cm
    # reescalamos la imagen para que 1px=20cm
    scale_percent = 25 # percent of original size
    width = int(gridmap_dilatated.shape[1] * scale_percent / 100)
    height = int(gridmap_dilatated.shape[0] * scale_percent / 100)
    dim = (width, height)

    RESOLUCION = (int(gridmap.shape[1]),int(gridmap.shape[0]))
    print(dim)
    print(RESOLUCION)
    pts2 = np.float32([[0,0],[RESOLUCION[0],0],[0,RESOLUCION[1]],[RESOLUCION[0],RESOLUCION[1]]])

    while(1):
        if(selectPoints()):
            pts1 = np.float32([\
                [pts[0][0],pts[0][1]],\
                [pts[1][0],pts[1][1]],\
                [pts[2][0],pts[2][1]],\
                [pts[3][0],pts[3][1]] ])

            Math = cv2.getPerspectiveTransform(pts1,pts2)
            
            while(1):
                frame = gridmap_dilatated
                gridmap_resized = cv2.warpPerspective(frame,Math,RESOLUCION)
                gridmap_resized = cv2.resize(gridmap_resized, dim, interpolation = cv2.INTER_NEAREST)
                
                # convertir celdas de las que no se tiene información a celdas ocupadas, ya que
                # ninguna de las dos se tendrán en cuenta en la generacion del grafo
                #gridmap = np.where(gridmap<255, 0, gridmap)
                gridmap_resized[(gridmap_resized>=179) & (gridmap_resized<=238)] = 0
                gridmap_resized[(gridmap_resized>=241) & (gridmap_resized<=255)] = 255

                fig1 = plt.figure(1,frameon=False, facecolor='blue')
                plt.imshow(gridmap_resized, cmap='gray', vmin=0, vmax=255)
                if not fig1:
                    fig1 = plt.gcf()

                    plt.subplots_adjust(0,0,1,1,0,0)
                for ax in fig1.axes:
                    ax.axis('off')
                    ax.margins(0,0)
                    ax.xaxis.set_major_locator(plt.NullLocator())
                    ax.yaxis.set_major_locator(plt.NullLocator())
                fig1.savefig(os.path.join(nPath + '/results/gridmap_resized.png'),bbox_inches='tight', pad_inches=0)
                plt.show()
                
                orden = open(os.path.join(nPath + "/resources/orden.txt"), "w+")
                orden.write(str(2))
                orden.close()
                sys.exit()
                key = cv2.waitKey(1) & 0xFF

                if key == 27:
                    break

        else:
            print("Exit")
            break

#funcion principal del programa
if __name__ == '__main__':
    try:
        corre = int(run.read())
        if corre == 1:
            openfile()
            grid_map()
            Thread(target=mouse).start()
            Thread(target=selectPoints).start()
        
    except rospy.ROSInterruptException:
        pass


