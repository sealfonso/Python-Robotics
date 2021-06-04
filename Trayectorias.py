#!/usr/bin/env python

import rospy
import sys
import time
import os
import math
import numpy as np
import matplotlib.pyplot as plt
from pandas import DataFrame
from itertools import islice
from sklearn.cluster import KMeans
from sklearn import linear_model, datasets

# Inicialización del nodo de ROS
rospy.init_node('punto2',anonymous=True)

# Asignación de la ruta de archivos actual
absPath = os.path.abspath(__file__)
nPath = os.path.split(os.path.dirname(absPath))[0]

# Método principal
def dev():
	vector_x = np.array([]) # Coordenadas en el eje x de los puntos del sensor 
	vector_y = np.array([]) # Coordenadas en el eje y de los puntos dle sensor

	pos_robot = [1, 0.5, (math.pi)/4] # Ubicación del robot con respecto al marco inercial
	pos_laser = [0.2, 0] # Ubicación del láser en el marco de referencia del robot
	fm_angle = -math.pi/2 # Ángulo de la primera medición 
	sm_angle = math.pi/2 # Ángulo de la segunda medición 

	# Creación de variables para visualización de los datos del láser

	laserdata = np.loadtxt(nPath + "/resources/" + 'laserscan.dat')
	num_data = len(laserdata)
	angle_min = pos_robot[2] + fm_angle
	angle_max = pos_robot[2] + sm_angle
	theta = np.linspace(angle_min, angle_max, num_data)

	# Proyección de los puntos sensados por el láser
	for i in range(num_data): 
		add = pos_laser[0]
		inv_matrot = np.array([[math.cos(theta[i]), -math.sin(theta[i])], [math.sin(theta[i]), math.cos(theta[i])]]) # Matriz de Rotación
		vector_data = ([[laserdata[i] + pos_laser[0]], [0]])
		pos_las = np.matmul(inv_matrot, vector_data)
		vector_x = np.append(vector_x, [pos_las[0]])
		vector_y = np.append(vector_y, [pos_las[1]])

	# Primera Figura - Punto 2.a
	fig1 = plt.figure(1)

	plt.plot(vector_x, vector_y, 'bo')
	robot = plt.Rectangle((pos_robot[0], pos_robot[1]-0.35), 0.5, 0.5, fc = 'red', ec = "black", angle = 45) # Imagen del Robot
	plt.gca().add_patch(robot)
	plt.xlabel('Eje x [m]')
	plt.ylabel('Eje y [m]')
	plt.suptitle('Identificación de Datos Sensados por el Láser')
	fig1.savefig(os.path.join(nPath + '/results/2_a.png'))

	# Punto 2.b - Función KMeans para creación de clusters
	Data = {'x': vector_x, 'y': vector_y}
	df = DataFrame(Data, columns =['x', 'y'])
	number_of_clusters = 6

	# Función KMeans
	kmeans = KMeans(n_clusters = number_of_clusters, random_state = 0).fit(df)
	points = kmeans.labels_.astype(float) 

	# Segunda Figura - Punto 2.b.
	fig2 = plt.figure(2)
	plt.scatter(df['x'], df['y'], c = kmeans.labels_.astype(float), s = 50, alpha = 0.5)
	robot = plt.Rectangle((pos_robot[0], pos_robot[1]-0.35), 0.5, 0.5, fc = 'red', ec = "black", angle = 45)
	plt.gca().add_patch(robot)
	plt.xlabel('Eje x [m]')
	plt.ylabel('Eje y [m]')
	plt.suptitle('Creación de Clusters')
	fig2.savefig(os.path.join(nPath + '/results/2_b.png'))

	# Creación de arreglos con los clusters encontrados
	unique, counts = np.unique(points, return_counts = True) # Retorna la cantidad de elementos asociados a cada Cluster
	Input_vectorx = iter(vector_x)
	Input_vectory = iter(vector_y)

	# Creación de arreglos con los clusters encontrados
	clusters_x = [list(islice(Input_vectorx, elem)) for elem in counts.tolist()]
	clusters_y = [list(islice(Input_vectory, elem)) for elem in counts.tolist()]

	# Seccionamiento de los clusters
	for i in range(number_of_clusters):
		Dat = {'x': clusters_x[i], 'y': clusters_y[i]}
		dfe = DataFrame(Dat, columns =['x', 'y'])

		# RANSAC
		# Fit line using all data
		clusters_x_array = dfe['x'].values.reshape(-1,1)
		clusters_y_array = dfe['y'].values.reshape(-1,1)

		lr = linear_model.LinearRegression()
		lr.fit(clusters_x_array, clusters_y_array)

		# Robustly fit linear model with RANSAC algorithm 
		ransac = linear_model.RANSACRegressor()
		ransac.fit(clusters_x_array, clusters_y_array)
		inlier_mask = ransac.inlier_mask_
		outlier_mask = np.logical_not(inlier_mask)

		#Predict data of estiamted models
		line_X = np.arange(clusters_x_array.min(), clusters_x_array.max())[:, np.newaxis]
		line_y = lr.predict(line_X)
		line_y_ransac = ransac.predict(line_X)

		# Tercera Figura - Punto 2.c
		fig3 = plt.figure(3)
		plt.plot(line_X, line_y_ransac, color = 'black', linewidth = 2)
		plt.scatter(df['x'], df['y'], c = kmeans.labels_.astype(float), s = 50, alpha = 0.5)
		robot = plt.Rectangle((pos_robot[0], pos_robot[1]-0.35), 0.5, 0.5, fc = 'red', ec = "black", angle = 45)
		plt.gca().add_patch(robot)
		plt.xlabel('Eje x [m]')
		plt.ylabel('Eje y [m]')
		plt.suptitle('Identificación de Obstáculos')
		fig3.savefig(os.path.join(nPath + '/results/2_c.png'))

	plt.show()

if __name__ == '__main__':
    try:
        
        dev()

    except rospy.ROSInterruptException:
        pass