#!/usr/bin/env python

import rospy
import cv2
from threading import Thread
import numpy as np
import argparse
import math
import os

# Inicialización del nodo de ROS
rospy.init_node('punto1',anonymous=True)
t=0

# top left, top right, bottom left, bottom right
pts = [(0,0),(0,0),(0,0),(0,0)]

# Índice de conteo de selección de esquinas
pointIndex = 0

# Uso de funciones para asignar el archivo como parámetro
parser = argparse.ArgumentParser(description='Indique el archivo de vídeo a abrir')
parser.add_argument('-f','--f_file', type=str, metavar='', default='', help='Archivo de vídeo')
args = parser.parse_args()

# Asignación de la ruta de archivos actual
absPath = os.path.abspath(__file__)
nPath = os.path.split(os.path.dirname(absPath))[0]

# Método que asigna el video mediante el parámetro
def open(f_file):
    global vid
    vid = cv2.VideoCapture(nPath + "/resources/"+ f_file + ".mp4")
    return vid

# mouse callback function
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

    print("Selecciona las cuatro esquinas de la cancha, haciendo doble click en el siguiente orden: \n\
    esquina superior izquierda, esquina superior derecha, esquina inferior izquierda, esquina inferior derecha.")


    while(pointIndex != 4):
        img = cv2.resize(img,(731, 425))
        cv2.imshow('Original',img)
        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            return False

    return True

# Método que se encarga de la transformación de perspectiva, el filtrado de colores e identificación de robots
def mask():
    global vid, img
    _,img = vid.read()

    RESOLUCION = (900,600)

    pts2 = np.float32([[0,0],[RESOLUCION[0],0],[0,RESOLUCION[1]],[RESOLUCION[0],RESOLUCION[1]]])

    while(1):
        if(selectPoints()):

            pts1 = np.float32([\
                [pts[0][0],pts[0][1]],\
                [pts[1][0],pts[1][1]],\
                [pts[2][0],pts[2][1]],\
                [pts[3][0],pts[3][1]] ])
            
            Math = cv2.getPerspectiveTransform(2*pts1,pts2)

            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            out = cv2.VideoWriter(nPath + '/results/Transformacion.mp4',fourcc, 30, (900,600))

            while(1):
                _,frame = vid.read()
                dst = cv2.warpPerspective(frame,Math,(900,600))
                hsv = cv2.cvtColor(dst, cv2.COLOR_RGB2HSV)
                
                #Azules:
                azul_bajos1 = np.array([170,61,72], dtype=np.uint8)
                azul_altos1 = np.array([255,255,255], dtype=np.uint8)
                azul_bajos2 = np.array([0,150,150], dtype= np.uint8)
                azul_altos2 = np.array([40,255,255], dtype= np.uint8)

                #Verdes:
                verde_bajos = np.array([10,150,150], dtype=np.uint8)
                verde_altos = np.array([80,255,255], dtype=np.uint8)
                
                #Amarillo:
                amarillo_bajos = np.array([70,210,100], dtype=np.uint8)
                amarillo_altos = np.array([100,255,255], dtype=np.uint8)

                #Magentas:
                magenta_bajos = np.array([139,61,72], dtype=np.uint8)
                magenta_altos = np.array([160,255,255], dtype=np.uint8)

                #Detectar los pixeles de la imagen que esten dentro del rango de cada color
                mascara_azul1 = cv2.inRange(hsv, azul_bajos1, azul_altos1)
                mascara_azul2 = cv2.inRange(hsv, azul_bajos2, azul_altos2) 
                mascara_azul = cv2.add(mascara_azul1,mascara_azul2)
                mascara_amarillo = cv2.inRange(hsv, amarillo_bajos, amarillo_altos)
                mascara_verde =cv2.inRange(hsv, verde_bajos, verde_altos)
                mascara_magenta =cv2.inRange(hsv, magenta_bajos, magenta_altos)

                #Filtrar el ruido aplicando un OPEN seguido de un CLOSE
                kernel = np.ones((4,4),np.uint8)
                
                mascara_azul = cv2.morphologyEx(mascara_azul, cv2.MORPH_CLOSE, kernel)
                mascara_azul = cv2.morphologyEx(mascara_azul, cv2.MORPH_OPEN, kernel)
                mascara_amarillo = cv2.morphologyEx(mascara_amarillo, cv2.MORPH_CLOSE, kernel)
                mascara_amarillo = cv2.morphologyEx(mascara_amarillo, cv2.MORPH_OPEN, kernel)
                mascara_verde = cv2.morphologyEx(mascara_verde, cv2.MORPH_CLOSE, kernel)
                mascara_verde = cv2.morphologyEx(mascara_verde, cv2.MORPH_OPEN, kernel)
                mascara_magenta = cv2.morphologyEx(mascara_magenta, cv2.MORPH_CLOSE, kernel)
                mascara_magenta = cv2.morphologyEx(mascara_magenta, cv2.MORPH_OPEN, kernel)

                #Unir las cuatro mascaras con el comando cv2.add()
                mask1 = cv2.add(mascara_amarillo, mascara_azul)
                mask2 = cv2.add(mascara_magenta,mascara_verde)
                mask = cv2.add(mask1,mask2)
                
                #Encontrar el area de los objetos que detecta la camara
                contours1,_ = cv2.findContours(mascara_azul, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                contours2,_ = cv2.findContours(mascara_amarillo, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                contours3,_ = cv2.findContours(mascara_verde, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                contours4,_ = cv2.findContours(mascara_magenta, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                
                i=0
                while i<=3:
                    c_1 = contours1[i]
                    c_2 = contours2[i]

                    (x1,y1),_= cv2.minEnclosingCircle(c_1)
                    center1 = (int(x1),int(y1))
                  
                    (x2,y2),_= cv2.minEnclosingCircle(c_2)
                    center2 = (int(x2),int(y2))
                    
                    font = cv2.FONT_HERSHEY_SIMPLEX 

                    j = 0
                    k = 0
                    l = 0
                    m = 0
                    n = 0

                    while j<=15:
                        c_3 = contours3[j]
                        c_4 = contours4[j]

                        (x3,y3),_= cv2.minEnclosingCircle(c_3)
                        center3 = (int(x3),int(y3))

                        (x4,y4),_= cv2.minEnclosingCircle(c_4)
                        center4 = (int(x4),int(y4))
                        
                        #regiones decision de jugadores
                        if x3>x1-15 and y3>y1-15 and x3<x1+15 and y3<y1+15:
                            k=k+1

                        if x4>x1-15 and y4>y1-15 and x4<x1+15 and y4<y1+15:
                            l=l+1
                            if l==1:
                                cen1=center4
                            if l==2:
                                cen2=center4


                        if x3>x2-15 and y3>y2-15 and x3<x2+15 and y3<y2+15:
                            m=m+1

                        if x4>x2-15 and y4>y2-15 and x4<x2+15 and y4<y2+15:
                            n=n+1
                            if n==1:
                                cen3=center4
                            if n==2:
                                cen4=center4

                        j = j+1

                    center5 = (int(x1+15),int(y1-15))
                    center6 = (int(x2+15),int(y2-15))

                    
                    if k==3 and l==1:
                        cv2.putText(dst,"2",center5,font, 0.8, (255,0,0), 2, cv2.LINE_AA)
                    if k==2 and l==2:
                        distance1 = math.sqrt((cen1[0] - cen2[0])**2 + (cen1[1] - cen2[1])**2)
                        if distance1 <= 17:
                            cv2.putText(dst,"1",center5,font, 0.8, (255,0,0), 2, cv2.LINE_AA)
                        else:
                            cv2.putText(dst,"3",center5,font, 0.8, (255,0,0), 2, cv2.LINE_AA)
                    if k==1 and l==3:
                        cv2.putText(dst,"0",center5,font, 0.8, (255,0,0), 2, cv2.LINE_AA)
                    
                    if m==3 and n==1:
                        cv2.putText(dst,"2",center6,font, 0.8, (0,255,255), 2, cv2.LINE_AA)
                    if m==2 and n==2:
                        distance2 = math.sqrt((cen3[0] - cen4[0])**2 + (cen3[1] - cen4[1])**2)
                        if distance2 <= 17:
                            cv2.putText(dst,"1",center6,font, 0.8, (0,255,255), 2, cv2.LINE_AA)
                        else:
                            cv2.putText(dst,"3",center6,font, 0.8, (0,255,255), 2, cv2.LINE_AA)
                    if m==1 and n==3:
                        cv2.putText(dst,"0",center6,font, 0.8, (0,255,255), 2, cv2.LINE_AA)

                    cv2.circle(dst,center1,15,(255,0,0),2)
                    cv2.circle(dst,center2,15,(0,255,255),2)
                    
                    i = i+1
                 

                cv2.imshow("Transformacion",dst)
                out.write(dst)

                key = cv2.waitKey(1) & 0xFF
                
                if key == 27:
                    break
        else:
            print("Exit")
            break

    vid.release()
    cv2.destroyAllWindows()

#funcion principal del programa
if __name__ == '__main__':
    try:
        open(args.f_file)
        mask()
        Thread(target=mouse).start()
        Thread(target=selectPoints).start()

    except rospy.ROSInterruptException:
        pass





