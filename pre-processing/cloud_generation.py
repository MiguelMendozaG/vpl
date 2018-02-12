import pandas as pd
import glob
from PIL import Image
import csv
import numpy as np
import itertools
import re
from numpy.linalg import inv
import os
import math
from math import atan2


input_dataset_folder = "/home/miguelmg/Documents/CIDETEC/semestre 2/vision 3d/proyecto/6d pose/hinterstoisser/train/02/"

output_dataset_folder = "/home/miguelmg/Documents/CIDETEC/semestre 2/vision 3d/proyecto/6d pose/hinterstoisser/nubes/modelo2/"

images=sorted(glob.glob(input_dataset_folder + "depth/*.png"))
matriz = open(input_dataset_folder + "gt.yml")
x = np.matrix([[0],[0],[-400],[1]])

###### intrinsic paramters are extracted
with open(input_dataset_folder + "info.yml", "r") as text_file:
    for line in itertools.islice(text_file, 2):
        a = line.replace(' cam_K: ','') #reemplaza la cadena '- cam_R_m2c: ' por un espacio ' ' 
        b = re.findall(r'[+-]?[0-9.]+', a) #encuentra todos los numeros flotantes positivos y negativos y los guarda en una matriz
text_file.close()
fx_d = float(b[0])
fy_d = float(b[4])
cx = float(b[2])
cy = float(b[5])

#print fx_d, fy_d, cx, cy

###### folders are created
if not os.path.exists(output_dataset_folder + "absolute"):
    os.makedirs(output_dataset_folder + "absolute")
    os.makedirs(output_dataset_folder + "absolute/model")
    os.makedirs(output_dataset_folder + "absolute/model_background")
    os.makedirs(output_dataset_folder + "absolute/background")
    os.makedirs(output_dataset_folder + "nbv")
    os.makedirs(output_dataset_folder + "position")
    os.makedirs(output_dataset_folder + "position/orientation")
    os.makedirs(output_dataset_folder + "position/pose")
    os.makedirs(output_dataset_folder + "points")

def rotation_matrix(num):
    num = num*5+1
    with open(input_dataset_folder + "gt.yml", "r") as text_file:
        for line in itertools.islice(text_file, num, num+1):
            #a = line.strip('- cam_R_m2c: [')
	    a = line.replace('- cam_R_m2c: ','') #reemplaza la cadena '- cam_R_m2c: ' por un espacio ' ' 
            #b = re.findall("\d+\.\d-", a)
	    b = re.findall(r'[+-]?[0-9.]+', a) #encuentra todos los numeros flotantes positivos y negativos y los guarda en una matriz
	    #b = re.compile(r"[+-]?\d+(?:\.\d+)?", a)
            c = np.matrix([[float(b[0]),float(b[1]),float(b[2]),0],[float(b[3]),float(b[4]),float(b[5]),0],[float(b[6]),float(b[7]),float(b[8]),400],[0,0,0,1]]) #se guarda la matriz de transformacion en c
    text_file.close()
    return c

def pc_model():#creates a point cloud of only the model, the background plane is not included
	count = 0
	for image in images:
	    print "model - imagen" ,count
	    img = Image.open(image)
	    px = img.load()
	    with open(output_dataset_folder + "absolute/model/imagen-" + str(count) + str('.xyz'), 'w') as f:
		for i in range (img.width):
		    for j in range (img.height):
                        if px[i,j]:
  			    mat = rotation_matrix(count)
			    a_x = (i - cx) * px[i,j] / fx_d
			    a_y = (j - cy) * px[i,j] / fy_d
			    a_z = px[i,j]
			    res = mat.I*np.matrix([[a_x],[a_y],[a_z],[1]])
		            f.write(str(res[0,0]/1000))
		            f.write('\t')
                            f.write(str(res[1,0]/1000))
		            f.write('\t')
		            f.write(str(res[2,0]/1000))
		            f.write('\n')
	    count = count +1
	    f.close()

def background():#crea nube de puntos por cada imagen. Solamente genera los planos de fondo
	count = 0
	for image in images:
	    print "background - imagen" ,count
	    img = Image.open(image)
	    px = img.load()
	    with open(output_dataset_folder + "absolute/background/imagen-" + str(count) + str('.xyz'), 'w') as f:
		for i in range (img.width):
		    for j in range (img.height):
                        #if (((i>=x_min) and (i<=x_max)) and ((j>=y_min) and (j<=y_max))):
		        if px[i,j] == 0:
                            px[i,j] = 3000
                            mat = rotation_matrix(count)
                            a_x = (i - cx) * px[i,j] / fx_d
                            a_y = (j - cy) * px[i,j] / fy_d
                            a_z = px[i,j]
                            res = mat.I*np.matrix([[a_x],[a_y],[a_z],[1]])
			    f.write(str(res[0,0]/1000))
			    f.write('\t')
		            f.write(str(res[1,0]/1000))
			    f.write('\t')
			    f.write(str(res[2,0]/1000))
			    f.write('\n')
	    count = count +1
	    f.close()

def model_background(): #creates the moteh including the background
        fx_d = 572.41140000
	fy_d = 573.57043000
	cx = 325.26110000
	cy = 242.04899000
	count = 0
	for image in images:
	    print "model and back - imagen" ,count
	    img = Image.open(image)
	    px = img.load()
	    with open(output_dataset_folder + "absolute/model_background/imagen-" + str(count) + str('.xyz'), 'w') as f:
		for i in range (img.width):
		    for j in range (img.height):
                        if px[i,j] == 0:
                            px[i,j] = 3000
		        #if px[i,j]:
			mat = rotation_matrix(count)
			a_x = (i - cx) * px[i,j] / fx_d
			a_y = (j - cy) * px[i,j] / fy_d
			a_z = px[i,j]
			res = mat.I*np.matrix([[a_x],[a_y],[a_z],[1]])
		        f.write(str(res[0,0]/1000))
		        f.write('\t')
                        f.write(str(res[1,0]/1000))
		        f.write('\t')
		        f.write(str(res[2,0]/1000))
		        f.write('\n')

	    count = count +1
	    f.close()

def pose():#guarda un archivo por imagen, cada rchivo contiene posicion x, y, z
    count = 0
    for image in images:
	#print "imagen" ,count
	with open(output_dataset_folder + "position/pose/imagen_origin_" + str(count) + str('.dat'), 'w') as f:
        #with open(direc, 'w') as f:
            mat = rotation_matrix(count)
            res =mat.I * x 
            f.write(str(res[0,0]/1000))
            f.write('\t')
            f.write(str(res[1,0]/1000))
            f.write('\t')
            f.write(str(res[2,0]/1000))
            f.write('\t')
            f.write('\n')
	    f.close()
        count = count + 1

def orientation():
	count = 0
	for image in images:
	    print "imagen" ,count
	    img = Image.open(image)
	    px = img.load()
	    with open(output_dataset_folder + "position/orientation/imagen-ori-" + str(count) + str('.dat'), 'w') as f:
                mat = rotation_matrix(count)
                yaw = atan2(mat[1,0],mat[0,0])
                pitch = atan2(-mat[2,0],pow(pow(mat[2,1],2)+pow(mat[2,2],2),0.5))
                roll = atan2(mat[2,1],mat[2,2])
		#res = mat.I*np.matrix([[a_x],[a_y],[a_z],[1]])
		f.write(str(math.degrees(yaw)))
		f.write('\t')
		f.write(str(math.degrees(pitch)))
		f.write('\t')
		f.write(str(math.degrees(roll)))
		f.write('\n')
                f.close()
	    count = count +1

model_background()
pc_model()
background()
pose()
orientation()

