#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory
from ur5_vs.msg import joint_vel
from ur5_vs.msg import joint_angles
from ur5_vs.msg import joint_states
import time
from sensor_msgs.msg import Image
from PIL import Image as im
from cv_bridge import CvBridge, CvBridgeError
import math as m
import numpy.matlib as npmat
 
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.image as mpimg
counter = 0
i = 0


def main():
	
	global msg
	global vel
	global th1
	global counter,i
	rospy.init_node('publish', anonymous=True)
	pub=rospy.Publisher('joint_angles_cmd', joint_angles, queue_size=10)
	pub1=rospy.Publisher('joint_vel_cmd', joint_vel, queue_size=10)
	#rospy.Subscriber('my_joint_states', joint_states, JS_callback)
	#js = joint_states()
	#th1 = js.ang2.data
	
	ja = joint_angles()
	jv = joint_vel()

	#sub=rospy.Subscriber('camera/rgb/image_raw', Image, rgb_callback, queue_size=1)
	while not rospy.is_shutdown():
		
		#jv.ang0.data = 0.6
		#jv.ang1.data = -0.4
		#jv.ang2.data = 0.8
		#jv.ang3.data = 0
		#jv.ang4.data = 0.2
		#jv.ang5.data = 0
		#pub.publish(jv)
			#counter = counter+1

		# ja.ang0.data = 0
		# ja.ang1.data = 0
		# ja.ang2.data = 0
		# ja.ang3.data = 0
		# ja.ang4.data = 0
		# ja.ang5.data = 0
		#pub.publish(ja)
#Desired Working Position-----------------
		# ja.ang0.data = 1.57
		# ja.ang1.data = -1.2
		# ja.ang2.data = 1.4
		# ja.ang3.data = 0
		# ja.ang4.data = 1.57
		# ja.ang5.data = 0.0
		# pub.publish(ja)
#Current Position Working
		# ja.ang0.data = 1.4
		# ja.ang1.data = -1.2
		# ja.ang2.data = 1.2
		# ja.ang3.data = 0
		# ja.ang4.data = 1.57
		# ja.ang5.data = 0.5
		# pub.publish(ja)

#Current working position 2
		# ja.ang0.data = 1.4
		# ja.ang1.data = -1.6
		# ja.ang2.data = 1.55
		# ja.ang3.data = 0.15
		# ja.ang4.data = 1.57
		# ja.ang5.data = -0.5
		# pub.publish(ja)

#Current working position 3_dd
		 #ja.ang0.data = 1.42
		# ja.ang1.data = -1.0
		 #ja.ang2.data = 1.99
		 #ja.ang3.data = -1.0
		 #ja.ang4.data = 5.56
		 #ja.ang5.data = 3.55
		 #pub.publish(ja)

#SMM Desired Position
		# if counter == 0:
		
		#ja.ang0.data = 3.14
		#ja.ang1.data = -0.5
		#ja.ang2.data = 1.2
		#ja.ang3.data = 0.4
		#ja.ang4.data = 1.57
		#ja.ang5.data = -0.000
		#pub.publish(ja)
		
		 ja.ang0.data = 1.0
		 ja.ang1.data = -0.9
		 ja.ang2.data = 1.99
		 ja.ang3.data = -1.0
		 ja.ang4.data = 1.57
		 ja.ang5.data = 0
		 pub.publish(ja)
			

		# Th0 = np.load('/home/shaunak/TEST RESULTS/FINALS/3D/Posn1_Obstacle_New/Joint_ang/th0.npy')
		# Th1 = np.load('/home/shaunak/TEST RESULTS/FINALS/3D/Posn1_Obstacle_New/Joint_ang/th1.npy')
		# Th2 = np.load('/home/shaunak/TEST RESULTS/FINALS/3D/Posn1_Obstacle_New/Joint_ang/th2.npy')
		# Th3 = np.load('/home/shaunak/TEST RESULTS/FINALS/3D/Posn1_Obstacle_New/Joint_ang/th3.npy')
		# Th4 = np.load('/home/shaunak/TEST RESULTS/FINALS/3D/Posn1_Obstacle_New/Joint_ang/th4.npy')
		# Th5 = np.load('/home/shaunak/TEST RESULTS/FINALS/3D/Posn1_Obstacle_New/Joint_ang/th5.npy')

		# ja.ang0.data = Th0[i]
		# ja.ang1.data = Th1[i]
		# ja.ang2.data = Th2[i]
		# ja.ang3.data = Th3[i]
		# ja.ang4.data = Th4[i]
		# ja.ang5.data = Th5[i]
		# pub.publish(ja)
		# rospy.sleep(1)
		# i = i + 1
		# counter = counter + 1
		# jv.vel0.data = 0
		# jv.vel1.data = 0
		# jv.vel2.data = 0
		# jv.vel3.data = 0
		# jv.vel4.data = 0
		# jv.vel5.data = 0
		# pub1.publish(jv)
		
		#print(th1)
	
if __name__=='__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass

