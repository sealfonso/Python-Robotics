from math import sqrt, pow
from pygame import gfxdraw
import pygame
import os
import numpy as np
import sys
import time
from pygame.locals import *
from std_msgs.msg import Float32MultiArray



class Buscar:

    def __init__(self, astar, movimiento_diagonal_posible):
        # Datos de la grilla
        self.grid = None
        self.nodo_inicial = None
        self.nodo_final= None
        self.movimiento_diagonal_posible = movimiento_diagonal_posible
     
        # Datos de los algoritmos
        self.astar = astar
        self.visitados = None
        self.observados = None
        self.nodo_siguiente = None
        self.buscando = None

        self.absPath = os.path.abspath(__file__)
        self.nPath = os.path.split(os.path.dirname(self.absPath))[0]
        self.camino = open(os.path.join(self.nPath + "/resources/ruta1.txt"), "rt")
        self.r = int(self.camino.read())

        self.mio = open(os.path.join(self.nPath + "/resources/orden.txt"), "rt")     
        self.corre = int(self.mio.read())

    def reset(self, grid, nodo_inicial, nodo_final ):
        # Guardar los datos de la grilla
        self.grid = grid
        self.nodo_inicial = nodo_inicial
        self.nodo_final= nodo_final

        # Guardar que nodos han sido visitados y que está actualmente abiertos
        self.visitados = []
        self.observados = [self.nodo_inicial] 
        self.nodo_siguiente = None
        self.buscando = True

    def heuristica(self, a, b):
        
        if not self.astar:
            return 0 

        elif self.movimiento_diagonal_posible == True:
            return sqrt((a.x - b.x)**2 + (a.y - b.y)**2) # Heurística euclidiana

        elif self.movimiento_diagonal_posible == False:
            return abs(a.x - b.x) + abs(a.y - b.y)  # Heurística de Manhatan

    def path(self):
        temp = []
        nodo = self.nodo_siguiente
        temp.append(nodo)
        self.reversed_x_array = np.array([])
        self.reversed_y_array = np.array([])
        while nodo.anterior:
            temp.append(nodo.anterior)
            nodo = nodo.anterior
            self.reversed_x_array = np.append(self.reversed_x_array,nodo.x)
            self.reversed_y_array = np.append(self.reversed_y_array,nodo.y)
        return temp


    
    def colorear(self, surface, ancho_nodo, alto_nodo):
        path = self.path()
        nod = self.nodo_siguiente
        
        if ((nod == self.nodo_final) and (self.buscando == False)):
            for col in self.grid:
                for nodo in col:
                    color = None
                    if nodo in self.visitados: # Colorea los nodos visitados
                        color = ((0,0,255))  # Color Azul
                        pygame.draw.rect(surface, color,[nodo.x * ancho_nodo, nodo.y * alto_nodo,ancho_nodo,alto_nodo],0)
                    
                    if nodo in path: # Colorea el camino encontrado
                        color = (255, 128, 0) # Color Naranja
                        pygame.draw.rect(surface, color,[nodo.x * ancho_nodo, nodo.y * alto_nodo,ancho_nodo,alto_nodo],0)
                        
                    if nodo in [self.nodo_inicial]: # Colorea el nodo inicial
                        color = (255, 0, 0) # Color rojo
                        pygame.draw.rect(surface, color,[nodo.x * ancho_nodo, nodo.y * alto_nodo,ancho_nodo,alto_nodo],0)
                        
                    if nodo in [self.nodo_final]: # Colorea el nodo final
                        color = (0,255, 0) # Color verde
                        pygame.draw.rect(surface, color,[nodo.x * ancho_nodo, nodo.y * alto_nodo,ancho_nodo,alto_nodo],0)
    
    def ruta(self):

        path = self.path()
        nod = self.nodo_siguiente            
        self.multiarray_route = []
        self.x_array = self.reversed_x_array[::-1]
        self.y_array = self.reversed_y_array[::-1]
        self.angle_array = np.zeros([])
        self.x_index = np.zeros([])
        self.y_index = np.zeros([])
        self.x_index.resize(1,self.x_array.shape[0])
        self.y_index.resize(1,self.y_array.shape[0])
        self.angle_array.resize(1,self.y_array.shape[0])
    
        if ((nod == self.nodo_final) and (self.buscando == False)):

            
            if self.r == 0:
                route_txt = open(os.path.join(self.nPath + '/resources/ruta.txt'), "w+")
                contador= open(os.path.join(self.nPath + '/resources/conteo.txt'), "w+")
            if self.r == 1:
                route_txt = open(os.path.join(self.nPath + '/resources/ruta.txt'), "a")
                contador= open(os.path.join(self.nPath + '/resources/conteo.txt'), "a")
            
            contador.write(str(self.x_array.shape[0])+'\n')
            contador.close()

            for i in range(self.x_array.shape[0]):
                if i == 0:
                    self.x_index[0,i] = 0
                else:
                    self.x_index[0,i] = self.x_array[i]-self.x_array[i-1]

            for j in range(self.y_array.shape[0]):
                if j == 0:
                    self.y_index[0,j] = 0
                else:
                    self.y_index[0,j] = self.y_array[j]-self.y_array[j-1]

                if ((self.x_index[0,j] == 1) and (self.y_index[0,j] == 0)):
                    self.angle_array[0,j] = 0

                if ((self.x_index[0,j] == -1) and (self.y_index[0,j] == 0)):
                    self.angle_array[0,j] = 180

                if ((self.x_index[0,j] == 0) and (self.y_index[0,j] == 1)):
                    self.angle_array[0,j] = 270

                if ((self.x_index[0,j] == 0) and (self.y_index[0,j] == -1)):
                    self.angle_array[0,j] = 90

                if ((self.x_index[0,j] == 1 and self.y_index[0,j] == 1)):
                    self.angle_array[0,j] = 315

                if ((self.x_index[0,j] == -1 and self.y_index[0,j] == 1)):
                    self.angle_array[0,j] = 225

                if ((self.x_index[0,j] == 1 and self.y_index[0,j] == -1)):
                    self.angle_array[0,j] = 45

                if ((self.x_index[0,j] == -1 and self.y_index[0,j] == -1)):
                    self.angle_array[0,j] = 135

                route_txt.write(str((self.x_array[j]*0.2)-7.5)+ ' '+str((-self.y_array[j]*0.2)+7.5)+ ' '+ str(self.angle_array[0,j])+'\n')
            
            route_txt.close()
            
            self.corre=self.corre+1

            orden = open(os.path.join(self.nPath + "/resources/orden.txt"), "w+")
            orden.write(str(self.corre))
            orden.close()

            pygame.quit()
            

                    
    def buscar(self):

        if not self.buscando:
            return

                # If no nodes to explore, no solution, abort
        if not self.observados:
            print("No solution!")
            self.buscando = False
            return
        #print(self.nodo_siguiente)

        self.nodo_siguiente = None
        # Encuentra el siguiente nodo posible
        for nodo in self.observados:
            if (self.nodo_siguiente is None) or (nodo.costo_total < self.nodo_siguiente.costo_total):
                self.nodo_siguiente = nodo

        #Si el nodo es el nodo final
        nodo = self.nodo_siguiente
        if nodo == self.nodo_final:
            print("End found!")
            self.buscando = False
            return

        # Estamos visitando el nodo así que lo sacamos de observados
        self.observados.remove(nodo)

        # Explorar todos los vecinos del nodo nodo
        for vecino in nodo.vecinos(self.grid, self.movimiento_diagonal_posible, self.astar, not self.movimiento_diagonal_posible):
            
            # Si el nodo vecino ya ha sido visitado salte
            if vecino in self.visitados:
                continue

            # Calcular el nuevo costo del nodo (assume distance from current to this vecino is 1)
            temp_costo_al_nodo = vecino.costo_al_nodo + 1

            # Si el nodo vecino nunca habia sido observado agregarlo aobservados
            if vecino not in self.observados:
                self.observados.append(vecino)
            
            # Si el nuevo costo hacia el nodo es más costoso salte
            elif temp_costo_al_nodo > vecino.costo_al_nodo:
                continue

            # Si pasó las anteriores condiciones este es el camino óptimo
            vecino.anterior = nodo
            vecino.costo_al_nodo = temp_costo_al_nodo
            vecino.costo_total = temp_costo_al_nodo + self.heuristica(vecino, self.nodo_final)

        # Se visitó en el nodo así que se agrega al vector
        self.visitados.append(nodo)
        #print(nodo.x,nodo.y)

        

