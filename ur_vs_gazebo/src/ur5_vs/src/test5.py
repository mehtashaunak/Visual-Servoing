#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from std_msgs.msg import Header
from ur5_vs.msg import joint_vel
from ur5_vs.msg import joint_angles
from ur5_vs.msg import joint_states
from sensor_msgs.msg import Image
import jacobian_func
import Smm_func
import Vcamera_to_Base
import Smm_error
import Smm_Vcamera
import Smm_Inter_Mat
import Smm_Inter_Mat1
import Smm_plots
import time
from PIL import Image as im
from cv_bridge import CvBridge, CvBridgeError
import math as m
import numpy.matlib as npmat
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.image as mpimg

bridge = CvBridge()
img_ht = 100
img_wt = 100
rgb_img   = np.zeros((480,640,3),np.uint8)
gray_img = np.zeros((480,640))
depth_img = np.zeros((480,640))
u0 = img_ht/2
v0 = img_wt/2
d = 2
i = 0
z = 4.7066
k = 0
smm_des = np.zeros((img_ht,img_wt))
smm_cur = np.zeros((img_ht,img_wt))

Varm = np.array([0,0,0,0,0,0])


th = np.array([0,0,0,0,0,0])
th1 = 0
th2 = 0
th3 = 0
th4 = 0
th5 = 0
th6 = 0
d11 = np.array([0.089459,0,0,0.10915,0.09465,0.0823])
a11 = np.array([0,0.42500,0.39225,0,0,0])
alp = np.array([3.142/2, 0, 0, 3.142/2, -3.142/2, 0])
d1 = 0.0895
d2 = 0
d3 = 0
d4 = 0.1092
d5 = 0.0947
d6 = 0.0823

a1 = 0
a2 = -0.425
a3 = -0.3923
a4 = 0
a5 = 0
a6 = 0
dt = 0.2
d = np.array([0.089459,0,0,0.10915,0.09465,0.0823])
a = np.array([0,0.42500,0.39225,0,0,0])
alp = np.array([3.142/2, 0, 0, 3.142/2, -3.142/2, 0])

counter = 0
count = 0


def JS_callback(js):
	global th,th1,th2,th3,th4,th5,th6
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
	# depth_img = cv2.resize(depth_img, dsize=(img_ht, img_wt), interpolation=cv2.INTER_CUBIC)
	#print('depth=',depth_img.item(25,25))
	

def rgb_callback(data):
	global rgb_img,gray_img,depth_img
	global i,d
	global smm_cur,smm_des
	global Varm
	
	global th,th1,th2,th3,th4,th5,th6
	global counter

#--------------------Getting Image from Kinect---------------------#
	
	rgb_img = bridge.imgmsg_to_cv2(data, "bgr8")
	# rgb_img = cv2.GaussianBlur(rgb_img, (11, 11), 0)
	cv2.imwrite('img_cur.png',rgb_img)
	# cur_img = cv2.imread('/home/shaunak/img_cur.png')
	gray_img = cv2.cvtColor(rgb_img, cv2.COLOR_BGR2GRAY)



	# gray_img = cv2.resize(gray_img, dsize=(img_ht, img_wt), interpolation=cv2.INTER_CUBIC)
	#print(np.shape(gray_img))

	# cv2.imshow("Image window", rgb_img)
	# cv2.waitKey(1)
	
def main():
	global rgb_img,gray_img,depth_img
	global i,d
	global smm_des,smm_cur
	global Varm
	
	global th,th1,th2,th3,th4,th5,th6
	global counter,count


	rospy.init_node('publish', anonymous=True)
	pub1=rospy.Publisher('joint_angles_cmd', joint_angles, queue_size=10)
	pub=rospy.Publisher('joint_vel_cmd', joint_vel, queue_size=10)
	
	sub=rospy.Subscriber('camera/rgb/image_raw', Image, rgb_callback, queue_size=1)
	rospy.Subscriber('camera/depth/image_raw', Image, depth_callback, queue_size=1)
	rospy.Subscriber('my_joint_states', joint_states, JS_callback)

#	raw_data = Image.data
	ja = joint_angles()
	jv = joint_vel()
	rospy.sleep(2)
	smm_des = np.load('/home/shaunak/smm_save1.npy')
#	print "before"
	while not rospy.is_shutdown():
            Th0 = np.load('/home/shaunak/TEST RESULTS/FINALS/3D/Closeup_1/Joint_ang/th0.npy')
            Th1 = np.load('/home/shaunak/TEST RESULTS/FINALS/3D/Closeup_1/Joint_ang/th1.npy')
            Th2 = np.load('/home/shaunak/TEST RESULTS/FINALS/3D/Closeup_1/Joint_ang/th2.npy')
            Th3 = np.load('/home/shaunak/TEST RESULTS/FINALS/3D/Closeup_1/Joint_ang/th3.npy')
            Th4 = np.load('/home/shaunak/TEST RESULTS/FINALS/3D/Closeup_1/Joint_ang/th4.npy')
            Th5 = np.load('/home/shaunak/TEST RESULTS/FINALS/3D/Closeup_1/Joint_ang/th5.npy')

            ja.ang0.data = Th0[i]
            ja.ang1.data = Th1[i]
            ja.ang2.data = Th2[i]
            ja.ang3.data = Th3[i]
            ja.ang4.data = Th4[i]
            ja.ang5.data = Th5[i]
            pub1.publish(ja)
            rospy.sleep(1)
            i = i + 1
		# if counter == 0:
		# 	smm_des = Smm_func.smm(gray_img)
		# 	np.save('smm_save1.npy', smm_des)
		# 	counter = counter + 1

		
            smm_cur = Smm_func.smm(gray_img)	
		# L_final = Smm_Inter_Mat.L_save(smm_cur,smm_des,gray_img,depth_img)

            L_final = Smm_Inter_Mat1.L_mat1(smm_cur,smm_des,gray_img,depth_img)
            error = Smm_error.error_func(smm_cur,smm_des)
            Vcamera = Smm_Vcamera.Vcam(L_final,error)
            Vbase = Vcamera_to_Base.Vc_2_Base(Vcamera,th)
            jacobian = jacobian_func.calc_jack(th1,th2,th3,th4,th5,th6)
            Varm = 1*np.matmul(jacobian,Vbase)

            T = np.identity(4)
            trans = np.identity(4)
            for i1 in range (0,6):
                T = np.array([[m.cos(th[i1]) , -m.sin(th[i1])*m.cos(alp[i1]) , m.sin(th[i1])*m.sin(alp[i1]) , a[i1]*m.cos(th[i1])] , [m.sin(th[i1]) ,m.cos(th[i1])*m.cos(alp[i1]) , -m.cos(th[i1])*m.sin(alp[i1]) , a[i1]*m.sin(th[i1])] , [0 , m.sin(alp[i1]) , m.cos(alp[i1]) , d[i1]] , [0 , 0 , 0 , 1]])
                trans = np.matmul(trans,T)

            
            Plot = Smm_plots.plot(error,Varm,th,trans,smm_cur)


		# jv.vel0.data = 0.01*Varm[0]
		# jv.vel1.data = 0.01*Varm[1]
		# jv.vel2.data = 0.01*Varm[2]
		# jv.vel3.data = 0.01*Varm[3]
		# jv.vel4.data = 0.01*Varm[4]
		# jv.vel5.data = 0.01*Varm[5]
		# pub.publish(jv)

		# ja.ang0.data = th1 + dt*Varm[0]
		# ja.ang1.data = th2 + dt*Varm[1]
		# ja.ang2.data = th3 + dt*Varm[2]
		# ja.ang3.data = th4 + dt*Varm[3]
		# ja.ang4.data = th5 + dt*Varm[4]
		# ja.ang5.data = th6 + dt*Varm[5]
		# pub1.publish(ja)
		# rospy.sleep(1)
#		print "after"


if __name__=='__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
