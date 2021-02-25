#!/usr/bin/env python

import rospy
import numpy as np
import cv2
import jacobian_func
import Vcamera_to_Base
import Image_Features
from std_msgs.msg import Header
from ur5_vs.msg import joint_angles
from ur5_vs.msg import joint_vel
from ur5_vs.msg import joint_states
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import math as m
import Plots 
import numpy.matlib as npmat
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.image as mpimg

bridge = CvBridge()
th1=th2=th3=th4=th5=th6 = 0
th = [0,0,0,0,0,0]
img = np.zeros((480,640,3),np.uint8)
depth_img = np.zeros((480,640))
hsv = np.zeros((480,640,3),np.uint8)
u0 = 320
v0 = 240
u_des1=u_des2=u_des3=u_des4 = 0
v_des1=v_des2=v_des3=v_des4 = 0
z = 0.6
K = 0.1
r1=r2=r3=r4=0
c1=c2=c3=c4=0
d = np.array([0.089459,0,0,0.10915,0.09465,0.0823])
a = np.array([0,0.42500,0.39225,0,0,0])
alp = np.array([3.142/2, 0, 0, 3.142/2, -3.142/2, 0])
Varm = np.array([0,0,0,0,0,0])
th = np.array([0,0,0,0,0,0])

f = 531.15


def JS_callback(js):
	global th1,th2,th3,th4,th5,th6
	global th

	th1 = js.ang0.data
	th2 = js.ang1.data
	th3 = js.ang2.data
	th4 = js.ang3.data
	th5 = js.ang4.data
	th6 = js.ang5.data
	th  = np.array([th1,th2, th3,th4,th5,th6])
	
	
def depth_callback(data1):
	
	global depth_img
	depth_img = bridge.imgmsg_to_cv2(data1, desired_encoding="passthrough")
	
	
def rgb_callback(data):
	global img,hsv,depth_img
	global th1,th2,th3,th4,th5,th6
	global th
	global r1,r2,r3,r4
	global c1,c2,c3,c4
	global u_des1,u_des2,u_des3,u_des4
	global v_des1,v_des2,v_des3,v_des4
	global Varm
	
	img = bridge.imgmsg_to_cv2(data, "bgr8")
	hsv = cv2.cvtColor (img, cv2.COLOR_BGR2HSV)
	
#--------------Getting Image Features------------------------------#
#defining the Range of Red color
	red_lower = np.array ([0, 50 , 50], np.uint8)
	red_upper = np.array ([20, 255, 255], np.uint8)
#defining the Range of Blue color
	blue_lower = np.array ([100, 150, 50], np.uint8)
	blue_upper = np.array ([120, 255, 255], np.uint8)
#defining the Range of yellow color
	yellow_lower = np.array ([25, 60, 50], np.uint8)
	yellow_upper = np.array ([45, 255, 255], np.uint8)
#defining the range of green color
	green_lower = np.array ([55, 1, 1], np.uint8)
	green_upper = np.array ([80, 255, 255], np.uint8)
#finding the range of red,blue and yellow color in the image
	red = cv2.inRange (hsv, red_lower, red_upper)
	blue = cv2.inRange (hsv, blue_lower, blue_upper)
	yellow = cv2.inRange (hsv, yellow_lower, yellow_upper)
	green = cv2.inRange (hsv, green_lower, green_upper)
	kernal = np.ones ((15, 15), "uint8")
	red = cv2.morphologyEx(red, cv2.MORPH_OPEN, kernal)
# Opening morph(erosion followed by dilation)
	blue = cv2.morphologyEx(blue, cv2.MORPH_OPEN, kernal)
# Opening morph(erosion followed by dilation)
	yellow = cv2.morphologyEx(yellow, cv2.MORPH_OPEN, kernal)
# Opening morph(erosion followed by dilation)
	green = cv2.morphologyEx(green, cv2.MORPH_OPEN, kernal)
	
	#Tracking the Red Color
	(_, contoursred, hierarchy) =cv2.findContours (red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-3:]
	for pic, contourred in enumerate (contoursred):
		area = cv2.contourArea (contourred) 
		if (area > 1000):
			x, y, w, h = cv2.boundingRect (contourred)
			img = cv2.rectangle (img, (x, y), (x + w, y + h), (0, 0, 255), 2)
			cv2.putText(img,"RED",(x,y),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255))
	
	if len(contoursred) > 0:
		# Find the biggest contour
		biggest_contour = max(contoursred, key=cv2.contourArea)

		# Find center of contour and draw filled circle
		moments = cv2.moments(biggest_contour)
		centre_of_contour = (int(moments['m10'] / moments['m00']), int(moments['m01'] / moments['m00']))
		cv2.circle(img, centre_of_contour, 5, (0, 0, 255), -1)
		# Save the center of contour so we draw line tracking it
		center_points1 = centre_of_contour
		r1 = center_points1[0]
		c1 = center_points1[1]

	#Tracking the Blue Color
	(_, contoursblue, hierarchy) =cv2.findContours (blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-3:]
        for pic, contourblue in enumerate (contoursblue):
		area = cv2.contourArea (contourblue)
		if (area > 1000):
			x, y, w, h = cv2.boundingRect (contourblue)
			img = cv2.rectangle (img, (x, y), (x + w, y + h), (255, 0, 0), 2)
			cv2.putText (img, "Blue", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0))

	if len(contoursblue) > 0:
		
		# Find the biggest contour
		biggest_contour = max(contoursblue, key=cv2.contourArea)

		# Find center of contour and draw filled circle
		moments = cv2.moments(biggest_contour)
		centre_of_contour = (int(moments['m10'] / moments['m00']), int(moments['m01'] / moments['m00']))
		cv2.circle(img, centre_of_contour, 5, (255,0, 0), -1)
		# Save the center of contour so we draw line tracking it
		center_points2 = centre_of_contour
		r2 = center_points2[0]
		c2 = center_points2[1]
		
		
	#Tracking the yellow Color
	(_, contoursy, hierarchy) = cv2.findContours (yellow, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)[-3:]
	for pic, contoury in enumerate (contoursy):
		area = cv2.contourArea(contoury)
		if(area>1000):
			x,y,w,h = cv2.boundingRect(contoury)
			img =cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2) 
			cv2.putText (img, "Yellow", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0,(0, 0,255))

	if len(contoursy) > 0:
		# Find the biggest contour
		biggest_contour = max(contoursy, key=cv2.contourArea)

		# Find center of contour and draw filled circle
		moments = cv2.moments(biggest_contour)
		centre_of_contour = (int(moments['m10'] / moments['m00']), int(moments['m01'] / moments['m00']))
		cv2.circle(img, centre_of_contour, 5, (0,255, 255), -1)
		# Save the center of contour so we draw line tracking it
		center_points3 = centre_of_contour
		r3 = center_points3[0]
		c3 = center_points3[1]
		
	#Tracking the green Color
	(_, contoursgreen, hierarchy) =cv2.findContours (green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-3:]
	for pic, contourgreen in enumerate (contoursgreen):
		area = cv2.contourArea (contourgreen) 
		if (area > 1000):
			x, y, w, h = cv2.boundingRect (contourgreen)
			img = cv2.rectangle (img, (x, y), (x + w, y + h), (0, 255, 0), 2)
			cv2.putText(img,"Green",(x,y),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0))

	if len(contoursgreen) > 0:
		# Find the biggest contour
		biggest_contour = max(contoursgreen, key=cv2.contourArea)

		# Find center of contour and draw filled circle
		moments = cv2.moments(biggest_contour)
		centre_of_contour = (int(moments['m10'] / moments['m00']), int(moments['m01'] / moments['m00']))
		cv2.circle(img, centre_of_contour, 5, (0, 255, 0), -1)
		# Save the center of contour so we draw line tracking it
		center_points4 = centre_of_contour
		r4 = center_points4[0]
		c4 = center_points4[1]
	
	u_des1 = 261
	v_des1 = 203
	u_des2 = 318
	v_des2 = 203
	u_des3 = 262
	v_des3 = 259
	u_des4 = 317
	v_des4 = 259

	cv2.circle(img, (u_des1,v_des1), 5, (0, 0, 255), -1)
	cv2.circle(img, (u_des2,v_des2), 5, (255, 0, 0), -1)
	cv2.circle(img, (u_des3,v_des3), 5, (0, 255, 255), -1)
	cv2.circle(img, (u_des4,v_des4), 5, (0, 255, 0), -1)
	z1 = depth_img[r1][c1]
	z2 = depth_img[r2][c2]
	z3 = depth_img[r3][c3]
	z4 = depth_img[r4][c4]
	print(z1)
	print(z2)
	print(z3)
	print(z4)

	#--------------Finding Vcamera-----------------#
	error = -np.array([-r1 + u_des1, -c1 + v_des1, -r2 + u_des2, -c2 + v_des2,  -r3 + u_des3, -c3 + v_des3, -r4 + u_des4, -c4 + v_des4])
	print('error=',error)
	Lc = np.array([[-f/z1, 0, r1/z1, (r1*c1)/f, -(f*f + r1*r1)/f, c1], [0, -f/z1, c1/z1, (f*f + c1*c1)/f, -(r1*c1)/f, -r1], [-f/z2, 0, r2/z2, (r2*c2)/f, -(f*f + r2*r2)/f, c2], [0, -f/z2, c2/z2, (f*f + c2*c2)/f, -(r2*c2)/f, -r2], [-f/z3, 0, r3/z3, (r3*c3)/f, -(f*f + r3*r3)/f, c3], [0, -f/z3, c3/z3, (f*f + c3*c3)/f, -(r3*c3)/f, -r3], [-f/z4, 0, r4/z4, (r4*c4)/f, -(f*f + r4*r4)/f, c4], [0, -f/z4, c4/z4, (f*f + c4*c4)/f, -(r4*c4)/f, -r4]]) 
	

	Lc = np.linalg.pinv(Lc)
	Lc = -K*Lc
	
	Vcamera = np.matmul(Lc,error)
	Vcamera[0] = -Vcamera[0]
	Vcamera[1] = -Vcamera[1]
	Vcamera[2] = -Vcamera[2]
	Vcamera[3] = Vcamera[3]
	Vcamera[4] = Vcamera[4]
	Vcamera[5] = Vcamera[5]
	print('Vcamera= ', Vcamera)
	
#-------------------------Convert Vcamera to Base Frame----------------------#
		
	Vbase = Vcamera_to_Base.Vc_2_Base(Vcamera,th)

	#-----------------Finding Jacobian-------------#

	jacobian = jacobian_func.calc_jack(th1,th2,th3,th4,th5,th6)
	#jacobian = np.linalg.pinv(jacobian)
	Varm = 1*np.matmul(jacobian,Vbase)
	print(Varm)
	T = np.identity(4)
	trans = np.identity(4)
	for i1 in range (0,6):
		T = np.array([[m.cos(th[i1]) , -m.sin(th[i1])*m.cos(alp[i1]) , m.sin(th[i1])*m.sin(alp[i1]) , a[i1]*m.cos(th[i1])] , [m.sin(th[i1]) ,m.cos(th[i1])*m.cos(alp[i1]) , -m.cos(th[i1])*m.sin(alp[i1]) , a[i1]*m.sin(th[i1])] , [0 , m.sin(alp[i1]) , m.cos(alp[i1]) , d[i1]] , [0 , 0 , 0 , 1]])
		trans = np.matmul(trans,T)
	
	plot = Plots.plot(Varm,error,th,r1,c1,r2,c2,r3,c3,r4,c4,trans)
	




def main():
	global img
	global hsv
	global Varm
	global depth_img
	rospy.init_node('publish', anonymous = True)
	
	rospy.Subscriber('my_joint_states', joint_states, JS_callback)

	sub=rospy.Subscriber('camera/rgb/image_raw', Image, rgb_callback, queue_size=1)
	rospy.Subscriber('camera/depth/image_raw', Image, depth_callback, queue_size=1)
	
	pub=rospy.Publisher('joint_vel_cmd', joint_vel,  queue_size=10)
	pub1 = rospy.Publisher('joint_angles_cmd', joint_angles,  queue_size=10)
	jv = joint_vel()
	ja = joint_angles()
	
	while not rospy.is_shutdown():		
		jv.vel0.data = Varm.item(0)
		jv.vel1.data = Varm.item(1)
		jv.vel2.data = Varm.item(2)
		jv.vel3.data = Varm.item(3)
		jv.vel4.data = Varm.item(4)
		jv.vel5.data = Varm.item(5)
		cv2.imshow("Image window", img)
		# cv2.imshow("Depth Window", depth_img)
	#	print(jv)
		

		pub.publish(jv)
		cv2.waitKey(1)
		rospy.sleep(0.001)
	




if __name__=='__main__':
	try:
		main()
	except rospy.ROSInterruptException:
        	pass

