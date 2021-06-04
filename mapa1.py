from random import random

import pygame
from pygame.locals import SRCALPHA


class Node:

    def __init__(self, x, y, wall_chance):
        self.x = x
        self.y = y

        self.costo_total = 0  # "fScore" ##################### Notificar cambio
        self.costo_al_nodo = 0  # "gScore" ################### Notificar Cambio
        self.anterior = None ################################# Notificar cambio

        ### neighbours------- vecinos
        ### neighbour---- vecino

        self.wall = wall_chance

        self.__vecinos = None

    def __str__(self):
        return "Node({},{}) Wall={}".format(self.x, self.y, self.wall)

    def __eq__(self, other):
        if other.x != self.x:
            return False
        if other.y != self.y:
            return False
        return True

    #recorre los nodos en la vecindad del nodo actual
    def vecinos(self, grid, allow_diagonal, astar, diagonal_check):
        # Cache so we don't need to calc every time
        if self.__vecinos is not None:
            return self.__vecinos

        self.__vecinos = []

        # Superior
        if self.y > 0:
            self.__vecinos.append(grid[self.x][self.y - 1])
        # Derecha
        if self.x + 1 < len(grid):
            self.__vecinos.append(grid[self.x + 1][self.y])
        # Inferior
        if self.y + 1 < len(grid[0]):
            self.__vecinos.append(grid[self.x][self.y + 1])
        # Izquierda
        if self.x > 0:
            self.__vecinos.append(grid[self.x - 1][self.y])


        # if A* con heuristica euclidiana, recorre vecindades en diagonal
        if allow_diagonal == True:
            # Superior Izquierda (if no Superior/Izquierda walls)
            if self.y > 0 and self.x > 0:
                if not diagonal_check or \
                        (not grid[self.x][self.y - 1].wall and not grid[self.x - 1][self.y].wall):
                    self.__vecinos.append(grid[self.x - 1][self.y - 1])
            # Superior Derecha (if no Superior/Derecha walls)
            if self.y > 0 and self.x + 1 < len(grid):
                if not diagonal_check or \
                        (not grid[self.x][self.y - 1].wall and not grid[self.x + 1][self.y].wall):
                    self.__vecinos.append(grid[self.x + 1][self.y - 1])
            # Inferior Derecha (if no Inferior/Derecha walls)
            if self.y + 1 < len(grid[0]) and self.x + 1 < len(grid):
                if not diagonal_check or \
                        (not grid[self.x][self.y + 1].wall and not grid[self.x + 1][self.y].wall):
                    self.__vecinos.append(grid[self.x + 1][self.y + 1])
            # Inferior Izquierda (if no Inferior/Izquierda walls)
            if self.y + 1 < len(grid[0]) and self.x > 0:
                if not diagonal_check or \
                        (not grid[self.x][self.y + 1].wall and not grid[self.x - 1][self.y].wall):
                    self.__vecinos.append(grid[self.x - 1][self.y + 1])

        if allow_diagonal == False:
            if self.y > 0:
                self.__vecinos.append(grid[self.x][self.y - 1])
            # Derecha
            if self.x + 1 < len(grid):
                self.__vecinos.append(grid[self.x + 1][self.y])
            # Inferior
            if self.y + 1 < len(grid[0]):
                self.__vecinos.append(grid[self.x][self.y + 1])
            # Izquierda
            if self.x > 0:
                self.__vecinos.append(grid[self.x - 1][self.y])

        for node in self.__vecinos.copy():
            if node.wall:
                self.__vecinos.remove(node)

        return self.__vecinos