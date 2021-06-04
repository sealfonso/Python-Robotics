#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from threading import Thread
from std_msgs.msg import Float32MultiArray, Int32
from std_msgs.msg import String
from copy import deepcopy
from random import randint
import pygame
import numpy as np
from pygame.locals import *
import argparse
from mapa1 import Node
from mapa2 import Buscar
import os
import roslib
import sys
try:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
except:
    pass
import cv2


class Grid1():

    def __init__(self):
    
        # Índice de conteo de selección de esquinas
        self.pointIndex = 0

        # estar,end
        self.pts = [(0,0),(0,0)]

        self.multiarray_route = []

        ### draw a colorear
        #searches -------------- busquedas
        #Search ------------------ Buscar
        #search (param) -------- busqueda
        #search() metodo -------- buscar

        absPath = os.path.abspath(__file__)
        nPath = os.path.split(os.path.dirname(absPath))[0]
        #Se lee la imagen y se obtienen los valores binarios del mapeo
        self.gridmap = cv2.imread(os.path.join(nPath + '/results/gridmap_resized.png'),-1)
        self.gridmap_resized = cv2.resize(self.gridmap,(75,75), interpolation = cv2.INTER_NEAREST)
        
        self.obstaculos = np.where(self.gridmap_resized>=200, 0, self.gridmap_resized) #celda libre
        self.obstaculos = np.where(self.gridmap_resized<200, 1, self.obstaculos) #celda ocupada
         
        # Información del gridmap: dimensiones e inicialización de variables
        self.columnas = 75
        self.filas = 75

        self.ancho = int(self.gridmap.shape[1]/self.columnas)
        self.alto = int(self.gridmap.shape[0]/self.filas)

        self.grid = None
        self.start = None
        self.end = None

        self.posx = 0
        self.posy = 0

        # Pygame
        pygame.init()
        self.screen = pygame.display.set_mode((int(self.columnas * self.ancho), int(self.filas * self.alto)))
        self.running = False
        self.routex = np.array([])
        self.routey = np.array([])

        self.absPath = os.path.abspath(__file__)
        self.nPath = os.path.split(os.path.dirname(self.absPath))[0]
        self.puntos = open(os.path.join(self.nPath + "/resources/puntos.txt"), "rt")
        
   


    def argument(self):
        # Uso de funciones para asignar el algoritmo como parámetro

        f = open(os.path.join(self.nPath + "/resources/algoritmo.txt"), "rt")
        a = int(f.read())

        # busquedas
        if a == 1:
            print('You Selected: Dijkstra')
            self.busquedas = [
                Buscar(False, False)  # Dijkstra
            ]

        if a == 2:
            print('You Selected: A* with manhattan heuristics')
            self.busquedas = [
                Buscar(True, False)  # A* (manhattan)
            ]

        if a == 3:
            
            print('You Selected: A* with euclidian heuristics')
            self.busquedas = [
                Buscar(True,  True),  # A* (euclidiana)
            ]

        self.reset()

    #mouse callback function
    def reset(self):
        
        self.grid = [[Node(_x, _y, self.obstaculos[_y,_x,0]) for _y in range(self.filas)] for _x in range(self.columnas)]
        
        ruta = open(os.path.join(self.nPath + "/resources/ruta1.txt"), "rt")
        self.r = int(ruta.read())
        
        if self.r == 1:
            nada = self.puntos.readline().split()

        self.get_start()
        self.get_end()
      
        # Otorga a las busquedas su info (copias)
        for busqueda in self.busquedas:
            busqueda.reset(deepcopy(self.grid), deepcopy(self.start), deepcopy(self.end))


    def get_start(self):
        
        inicio = self.puntos.readline().split()

        inicial_x = int((float(inicio[0])+7.5)/0.2)
        inicial_y = int((-float(inicio[1])+7.5)/0.2)
        self.start = self.grid[inicial_x][inicial_y]#[self.pts[0][0]][self.pts[0][1]]
        self.start.wall = False


    def get_end(self):

        final = self.puntos.readline().split()
        final_x = int((float(final[0])+7.5)/0.2)
        final_y = int((-float(final[1])+7.5)/0.2)
        self.end = self.grid[final_x][final_y]#[self.pts[1][0]][self.pts[1][1]]
        self.end.wall = False


    def colorear(self):

        # Maneja los eventos de PyGame
        for event in pygame.event.get():
            if event.type == KEYDOWN:
                if event.key == K_ESCAPE:
                    self.running = False
                elif event.key == K_r:
                    self.reset()
                    return
            elif event.type == QUIT:
                self.running = False

        # Rellena el fondo del gridmap de blanco
        self.screen.fill((255, 255, 255))

        # Dibuja las paredes en el mapa
        surface = pygame.Surface((self.ancho, self.alto), SRCALPHA) 
        for columna in self.grid:
            for self.fila in columna:
                color = None
                if self.fila.wall:
                    color = (0, 0, 0)
                    #screen.blit(*fila.show(ancho, alto, color, nodes_as_circles))
                    surface.fill(color)
                    self.screen.blit(surface, (self.ancho * self.fila.x, self.alto * self.fila.y))

        #Permite búsquedas para dibujar sus caminos
        for busqueda in self.busquedas:
            busqueda.colorear(self.screen, self.ancho, self.alto)


        
        if self.r == 0:
            pygame.image.save(self.screen,os.path.join(self.nPath + '/results/Ruta1.jpeg'))
        if self.r == 1:
            pygame.image.save(self.screen,os.path.join(self.nPath + '/results/Ruta2.jpeg'))
                
        #Actualiza la imagen en el display
        pygame.display.flip()

    def ruta(self):            

        for busqueda in self.busquedas:
            busqueda.ruta()

        f = open(os.path.join(self.nPath +"/resources/ruta1.txt"), "w+")
        if self.r == 0:
            f.write(str(1))
        if self.r == 1:
            f.write(str(0))
        f.close()


    def main(self):

        while not rospy.is_shutdown():
            rospy.init_node('mapa0',anonymous=True)
            helmi = open(os.path.join(self.nPath + "/resources/orden.txt"), "rt")     
            forest = int(helmi.read())
            if forest == 3:
                self.argument()
                self.running = True
                while self.running:
                    for self.busqueda in self.busquedas:
                        self.busqueda.buscar()
                    self.colorear()
                    self.ruta()


if __name__ == "__main__":
    try:

        Grid1().main()

    except rospy.ROSInterruptException:
        print( "Shutting down ROS Image feature detector module")
        cv2.destroyAllWindows()

    
