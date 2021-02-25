#!/usr/bin/env python

import rospy
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.image as mpimg
import cv2
import imutils
from PIL import Image
import math as m
import numpy.matlib as npmat



img_ht = 50
img_wt = 50
u0 = img_ht/2
v0 = img_wt/2


#--------------SMM of Desired Image--------------#
img = cv2.imread('/home/sam/ur_vs_gazebo/src/ur5_vs/src/Image_1.png')
#img = img.resize((img_wt, img_ht), Image.ANTIALIAS) 
#img1 = img.save('resized_image.png')
#im1 = cv2.imread('/home/sam/ur_vs_gazebo/src/ur5_vs/src/resized_image.png')
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
gray = cv2.resize(gray, dsize=(img_ht, img_wt), interpolation=cv2.INTER_CUBIC)
#cv2.imshow('image', gray)
#cv2.waitKey(0)

var = np.reshape(gray, (img_ht*img_wt,1))

x0 = [[j for i in range(img_ht)] for j in range(img_ht)]
y0 = [[i for i in range(img_ht)] for j in range(img_ht)]
y0 = np.reshape(y0, (1,img_ht*img_wt))
x0 = np.reshape(x0, (1,img_ht*img_wt))
#print(len(x0))
y0 = y0 - v0
x0 = x0 - u0
u = np.column_stack(([x0], [y0]))
u = np.reshape(u, (2,img_ht*img_wt))
u = (u.T)
#print((np.shape(u)))
#print(u)

X = u
var = np.reshape(gray, img_ht*img_wt, 1 )
#print(np.shape(X))
#print(X)

smm_des = np.zeros((img_ht,img_wt))
i = 0


def smm_desired(X, mu, sigma, v, gray):
	d = 2
	mu = npmat.repmat(mu,img_ht*img_wt,1)
	diff = np.subtract(X,mu)
	delta = (np.multiply(np.dot((diff),np.linalg.inv(sigma)),diff))
	delta = delta.sum(axis=1)
	mahal_dist = pow(1 + pow(v,-1)*delta,(v+d)*0.5)
	phi = m.gamma((v+d)*0.5)*pow(np.linalg.det(sigma),-0.5)*pow(pow(3.142*v,d*0.5)*m.gamma(v*0.5)*mahal_dist,-1)
	phi = np.reshape(phi, (img_ht,img_wt))
	return phi

for i in range (0,img_ht*img_wt-1):
	if var.item(i) > 20:
		sigma = np.array([[var.item(i),0.01],[0.01,var.item(i)]])
		v = (2*var.item(i))*m.pow((var.item(i)-1),-1)
		mu = np.array([u.item(i,0), u.item(i,1)])
		smm_des = smm_des + smm_desired(X, mu, sigma, v,gray)
		


x = y = np.arange(0, 50, 1)
X, Y = np.meshgrid(x, y)
Z = smm_des
#print np.shape(X)
#print np.shape(Z)
fig = plt.figure()
ax = plt.axes(projection='3d')
ax.plot_surface(X, Y, Z, rstride=1, cstride=1, cmap='viridis', edgecolor='none')
plt.show()
