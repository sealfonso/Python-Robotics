#!/usr/bin/env python

import rospy
import cv2
from threading import Thread
import matplotlib.pyplot as plt
import numpy as np
import argparse
import math
import os
import pygame

# Inicialización del nodo de ROS
rospy.init_node('Contorno',anonymous=True)
t=0

# top left, top right, bottom left, bottom right
pts = []

# Índice de conteo de selección de esquinas
pointIndex = 0

# Uso de funciones para asignar el archivo como parámetro
parser = argparse.ArgumentParser(description='Indique el archivo de imagen a abrir')
parser.add_argument('-f','--f_file', type=str, metavar='', default='', help='Archivo de imagen')
args = parser.parse_args()

# Asignación de la ruta de archivos actual
absPath = os.path.abspath(__file__)
nPath = os.path.split(os.path.dirname(absPath))[0]

# Método que asigna el video mediante el parámetro
def open(f_file):
    global gridmap, file
    file = f_file
    gridmap = cv2.imread(nPath + "/results/"+ f_file + ".png", -1)
    gridmap= cv2.resize(gridmap,(int(gridmap.shape[1]*2),int(gridmap.shape[0]*2)))
    return gridmap

#mouse callback function
def mouse(event,x,y,flags,param):
    global img
    global pointIndex
    global pts

    if event == cv2.EVENT_LBUTTONDBLCLK:
        cv2.circle(img,(x,y),0,(255,0,0),-1)
        pts.append([x,y])
        print(pts)
        pointIndex = pointIndex + 1

cv2.namedWindow('Original')
cv2.setMouseCallback('Original',mouse)

# Método que se encarga de redimencionar y desplegar la ventana con el primer frame del vídeo
def selectPoints():
    global img
    global pointIndex, file

    if file == "Pez":
    	k=9
    if file == "Pato":
    	k=15
    if file == "Jirafa":
    	k=11
    if file == "Gato":
    	k=11
    if file == "Murcielago":
        k=16
    
    print("Seleccione todas las aristas del dibujo, haciendo doble click en ellas, en sentido horario.")
    
    while(pointIndex != k):
        cv2.imshow('Original',img)

        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            return False
    
    return True

# Método que se encarga de la transformación de perspectiva, el filtrado de colores e identificación de robots
def cont():
    global gridmap, img, RESOLUCION, gridmap_resized
    img = gridmap
    fin = False
    negro = (0,0,0)
    blanco = (255,255,255)
    Resolucion = [gridmap.shape[1],gridmap.shape[0]]
    pygame.init()
    pantalla = pygame.display.set_mode(Resolucion)
    pantalla.fill(blanco)
    reloj=pygame.time.Clock()
    while not fin:
        if(selectPoints()):
        	
        	r=0
        	while r <= pointIndex:
        		if r < pointIndex-1:
        			pygame.draw.line(pantalla, negro,(pts[r][0],pts[r][1]), (pts[r+1][0],pts[r+1][1]), 5)
        		if r == pointIndex-1:
        			pygame.draw.line(pantalla, negro,(pts[r][0],pts[r][1]), (pts[0][0],pts[0][1]), 5)
        		r = r+1
        else:
            print("Exit")
            break
        pygame.display.flip()
        pygame.image.save(pantalla,os.path.join(nPath + '/results/'+'contorno_'+file+'.png'))
        reloj.tick(60)
        fin=True

#funcion principal del programa
if __name__ == '__main__':
    try:
        open(args.f_file)
        cont()
        Thread(target=mouse).start()
        Thread(target=selectPoints).start()
        
    except rospy.ROSInterruptException:
        pass


