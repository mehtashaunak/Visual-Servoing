#!/usr/bin/env python
import rospy
#import actionlib
import numpy as np
import matplotlib.pyplot as plot
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from control_msgs.msg import *
from trajectory_msgs.msg import *
#from ur_kin_py.kin import Kinematics
from geometry_msgs.msg import Point
from object_detection.msg import image_data

PI = 3.14
DEG2RAD = PI/180.0
RAD2DEG = 180.0/PI
VELMAX = 1.0
VELMIN = -1.0
JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
		'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

#KPX = 5*(0.01)
#KDX = 0*(0.001)
#KIX = 0*(0.0001)

KPX = 2.8*(0.0001)
KDX = 1*(0.001)
KIX = 5*(0.0001)
KPY = 2.8*(0.0001)
KDY = 1*(0.001)
KIY = 3*(0.0001)

KPZ = 2*(1.0)
KDZ = 5*(0.1)
KIZ = 0*(0.01)
#AMAX=[5.0,4.0,4.0,5.0,4.0,5.0]
AMAX=[5.0,4.0,4.0,4.0,3.0,5.0]

#KPX = 2.8*(0.001)
#KDX = 3.1*(0.00001)
#KIX = 0*(0.0001)
#KPY = 2.8*(0.001)
#KDY = 3.1*(0.00001)
#KIY = 0*(0.0001)


global end_eff_vel
global pre_point
#global joint_command
global q_dot
global preq_dot
global jacobian
global t
global pre_velocityx
global n
global intx
global inty
global accq

#global kin
#kin=Kinematics('ur5')

def ballCenterCallback(msg):
	global end_eff_vel
	global pre_point
	global t
	global pre_velocityx
	global n
	global intx
	global inty
	global accq
	global errorx
	global errory
        global x_detected
        global y_detected
        global x_meas
        global y_meas

	if(msg.detected_point_depth.data==0):
		end_eff_vel[0] = 0.00
		end_eff_vel[1] = 0.00
		end_eff_vel[2] = 0.00
		end_eff_vel[3] = 0.00
		end_eff_vel[4] = 0.00
		end_eff_vel[5] = 0.00
		#print t
	else:	
         	print("inside callback")

		#print(msg.detected_point_x.data)
                x_detected = msg.detected_point_x.data
                y_detected = msg.detected_point_y.data
                x_meas = (x_detected - 320)/(531.15/640)
                y_meas = (y_detected- 240)/(531.15/480)
                errorx=0-x_meas
                errory=0-y_meas
                print("x_d: ", x_detected, "y_d: ", y_detected, "x_m:", x_meas, "y_meas: ", y_meas)
		z=msg.detected_point_depth.data-1.0
		intx=(errorx+n*intx)
		inty=(errory+n*inty)
		end_eff_vel[0] = (z*KPZ + (z-pre_point.z)*KDZ)#
		end_eff_vel[1] = (msg.detected_point_depth.data/531.15)*(errorx*KPX + (errorx-pre_point.x)*KDX + intx*KIX)
		end_eff_vel[2] = (msg.detected_point_depth.data/531.15)*(errory*KPY + (errory-pre_point.y)*KDY + inty*KIY)
		pre_point.x = errorx
		pre_point.y = errory
		pre_point.z = z
#		pre_point.z = msg.z
		#pre_velocityx=end_eff_vel[0]

# m[0]= (u_c[0]-320)/(531.15/640);    //(u-u0)/px
 #   m[1]= (u_c[1]-240)/(531.15/480);    //(v-v0)/py
		
def jointStateCallback(joint_state):
	global end_eff_vel
	#global joint_command
        print("inside joint state callback")	
        global q_dot
	global preq_dot
	global jacobian
	global intx
	global inty
	global accq

	q1=joint_state.position[0]
	q2=joint_state.position[1]
	q3=joint_state.position[2]
	q4=joint_state.position[3]
	q5=joint_state.position[4]
	q6=joint_state.position[5]	
        print("inside joint state callback")
        print("q1 ", q1)
	#converting jpos to q1,q2,3,4,5--- check the robot at all jpos=0!!!!		
	jacobian=np.array([[ (2183*np.cos(q1))/20000 + (823*np.cos(q1)*np.cos(q5))/10000 + (17*np.cos(q2)*np.sin(q1))/40 - (1569*np.sin(q1)*np.sin(q2)*np.sin(q3))/4000 + (823*np.cos(q2 + q3 + q4)*np.sin(q1)*np.sin(q5))/10000 - (591*np.cos(q2 + q3)*np.sin(q1)*np.sin(q4))/6250 - (591*np.sin(q2 + q3)*np.cos(q4)*np.sin(q1))/6250 + (1569*np.cos(q2)*np.cos(q3)*np.sin(q1))/4000, np.cos(q1)*((1569*np.sin(q2 + q3))/4000 + (17*np.sin(q2))/40 + np.sin(q5)*((823*np.cos(q2 + q3)*np.sin(q4))/10000 + (823*np.sin(q2 + q3)*np.cos(q4))/10000) + (591*np.cos(q2 + q3)*np.cos(q4))/6250 - (591*np.sin(q2 + q3)*np.sin(q4))/6250),                         np.cos(q1)*((591*np.cos(q2 + q3 + q4))/6250 + (1569*np.sin(q2 + q3))/4000 + (823*np.sin(q2 + q3 + q4)*np.sin(q5))/10000),                         np.cos(q1)*((591*np.cos(q2 + q3 + q4))/6250 + (823*np.sin(q2 + q3 + q4)*np.sin(q5))/10000), (823*np.cos(q1)*np.cos(q2)*np.cos(q5)*np.sin(q3)*np.sin(q4))/10000 - (823*np.cos(q1)*np.cos(q2)*np.cos(q3)*np.cos(q4)*np.cos(q5))/10000 - (823*np.sin(q1)*np.sin(q5))/10000 + (823*np.cos(q1)*np.cos(q3)*np.cos(q5)*np.sin(q2)*np.sin(q4))/10000 + (823*np.cos(q1)*np.cos(q4)*np.cos(q5)*np.sin(q2)*np.sin(q3))/10000, 0],
	[ (2183*np.sin(q1))/20000 - (17*np.cos(q1)*np.cos(q2))/40 + (823*np.cos(q5)*np.sin(q1))/10000 - (823*np.cos(q2 + q3 + q4)*np.cos(q1)*np.sin(q5))/10000 + (591*np.cos(q2 + q3)*np.cos(q1)*np.sin(q4))/6250 + (591*np.sin(q2 + q3)*np.cos(q1)*np.cos(q4))/6250 - (1569*np.cos(q1)*np.cos(q2)*np.cos(q3))/4000 + (1569*np.cos(q1)*np.sin(q2)*np.sin(q3))/4000, np.sin(q1)*((1569*np.sin(q2 + q3))/4000 + (17*np.sin(q2))/40 + np.sin(q5)*((823*np.cos(q2 + q3)*np.sin(q4))/10000 + (823*np.sin(q2 + q3)*np.cos(q4))/10000) + (591*np.cos(q2 + q3)*np.cos(q4))/6250 - (591*np.sin(q2 + q3)*np.sin(q4))/6250),                         np.sin(q1)*((591*np.cos(q2 + q3 + q4))/6250 + (1569*np.sin(q2 + q3))/4000 + (823*np.sin(q2 + q3 + q4)*np.sin(q5))/10000),                         np.sin(q1)*((591*np.cos(q2 + q3 + q4))/6250 + (823*np.sin(q2 + q3 + q4)*np.sin(q5))/10000), (823*np.cos(q1)*np.sin(q5))/10000 - (823*np.cos(q2)*np.cos(q3)*np.cos(q4)*np.cos(q5)*np.sin(q1))/10000 + (823*np.cos(q2)*np.cos(q5)*np.sin(q1)*np.sin(q3)*np.sin(q4))/10000 + (823*np.cos(q3)*np.cos(q5)*np.sin(q1)*np.sin(q2)*np.sin(q4))/10000 + (823*np.cos(q4)*np.cos(q5)*np.sin(q1)*np.sin(q2)*np.sin(q3))/10000,                      0],
	[                                                                                                                                                                                                                                                                                            0,                                                      (591*np.sin(q2 + q3 + q4))/6250 - (823*np.sin(q2 + q3 + q4 + q5))/20000 - (1569*np.cos(q2 + q3))/4000 - (17*np.cos(q2))/40 + (823*np.sin(q2 + q3 + q4 - q5))/20000, (591*np.sin(q2 + q3 + q4))/6250 - (823*np.sin(q2 + q3 + q4 + q5))/20000 - (1569*np.cos(q2 + q3))/4000 + (823*np.sin(q2 + q3 + q4 - q5))/20000, (591*np.sin(q2 + q3 + q4))/6250 - (823*np.sin(q2 + q3 + q4 + q5))/20000 + (823*np.sin(q2 + q3 + q4 - q5))/20000,                                                                                                                                                                           - (823*np.sin(q2 + q3 + q4 + q5))/20000 - (823*np.sin(q2 + q3 + q4 - q5))/20000,                                                     0],
	[                                                                                                                                                                                                                                                                                            0,                                                                                                                                                                                                  np.sin(q1),                                                                                                                           np.sin(q1),                                                                                                np.sin(q1),                                                                                                                                                                                                                           np.sin(q2 + q3 + q4)*np.cos(q1),   np.cos(q5)*np.sin(q1) - np.cos(q2 + q3 + q4)*np.cos(q1)*np.sin(q5)],
	[                                                                                                                                                                                                                                                                                            0,                                                                                                                                                                                                 -np.cos(q1),                                                                                                                          -np.cos(q1),                                                                                               -np.cos(q1),                                                                                                                                                                                                                           np.sin(q2 + q3 + q4)*np.sin(q1), - np.cos(q1)*np.cos(q5) - np.cos(q2 + q3 + q4)*np.sin(q1)*np.sin(q5)],
	[                                                                                                                                                                                                                                                                                            1,                                                                                                                                                                                                        0,                                                                                                                                 0,                                                                                                      0,                                                                                                                                                                                                                                  -np.cos(q2 + q3 + q4),                            -np.sin(q2 + q3 + q4)*np.sin(q5)]])
	
	q_dot=np.linalg.pinv(jacobian).dot(end_eff_vel)
#	preq_dot=joint_state.velocity
	for i in range(6):
		accq[i]=100*(q_dot[i]-preq_dot[i])
		if accq[i]>AMAX[i]:
			accq[i]=AMAX[i]
		elif accq[i]<-1*AMAX[i]:
			accq[i]=-1*AMAX[i]
		preq_dot[i]=joint_state.velocity[i]
	
def sender():
	#Defining endEffectorVelocity as global variable to be calculated based on center of ball
	global end_eff_vel
	end_eff_vel = np.array([[0.0],[0.0],[0.0],[0.0],[0.0],[0.0]])
	#end_eff_vel = np.array([0.0,0.0,0.0])	
	global pre_point
	global joint_command
	global q_dot
	global preq_dot
	global jacobian
	global t
	global pre_velocityx
	global n
	global intx
	global inty
	global accq
	n=0
	intx=0
	inty=0

	pre_velocityx=0.0
	jacobian=np.zeros((6,6))
	joint_data1=np.array([0.0])
	joint_data2=np.array([0.0])
	joint_data3=np.array([0.0])
	joint_data4=np.array([0.0])
	joint_data5=np.array([0.0])
	joint_data6=np.array([0.0])
        print("Stage 1")

	det=np.array([0.0])
	q_dot=[0.0,0.0,0.0,0.0,0.0,0.0]	
	preq_dot=[0.0,0.0,0.0,0.0,0.0,0.0]	
	pre_point = Point(0,0,0)
	accq=[0.0,0.0,0.0,0.0,0.0,0.0]

	#Creating node, publisher and subscribers
	rospy.init_node('velocity_controler', anonymous=True)
	pub = rospy.Publisher('/ur_driver/joint_speed', JointTrajectory, queue_size=10)
	rospy.Subscriber('/object_points', image_data, ballCenterCallback)
        #rospy.spinOnce()
	rospy.Subscriber('/joint_states', JointState, jointStateCallback)
        #rospy.spinOnce()
        print("Stage 2")
	#Publishing Rate
	rate = rospy.Rate(100) # in hz

	#Defining joint_command
        print("Stage 3")
	joint_command = JointTrajectory()
	joint_command.header = Header()
	joint_command.header.seq=0
	joint_command.header.stamp = rospy.Time.now()
	#joint_command.header.frame_id=''
	#joint_command.joint_names=JOINT_NAMES
	#joint_command.points=JointTrajectoryPoint()
	#joint_command.points.time_from_start=[0.0]
	#joint_command.points.velocities=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
	#plot.axis([0,10,-300,300])
	#plot.ion()
	t=0.0
	time=np.array([0.0])
	speed=np.array([0.0])
	speed2=np.array([0.0])
	while not rospy.is_shutdown():
#	while t<10:
                print("Stage 4")
		joint_command.points=[JointTrajectoryPoint(velocities=q_dot, accelerations=accq, time_from_start=rospy.Duration(0.0))]
                print("stage 5")
                print("q_dot", q_dot)
		#Updating header for joint_command
		joint_command.header.seq=joint_command.header.seq+1
		joint_command.header.stamp = rospy.Time.now()	
		#plot.scatter(t,end_eff_vel[0])
		#plot.pause(0.001)
		#rospy.loginfo(joint_command)
		#Publishing Joint Commands
		# jstate=rospy.wait_for_message("/joint_states" , JointState)
		# print("while t<10")
		# Q1=jstate.velocity[0]
		# Q2=jstate.velocity[1]
		# Q3=jstate.velocity[2]
		# Q4=jstate.velocity[3]
		# Q5=jstate.velocity[4]
		# Q6=jstate.velocity[5]
		# jbian=np.array([[ (2183*np.cos(Q1))/20000 + (823*np.cos(Q1)*np.cos(Q5))/10000 + (17*np.cos(Q2)*np.sin(Q1))/40 - (1569*np.sin(Q1)*np.sin(Q2)*np.sin(Q3))/4000 + (823*np.cos(Q2 + Q3 + Q4)*np.sin(Q1)*np.sin(Q5))/10000 - (591*np.cos(Q2 + Q3)*np.sin(Q1)*np.sin(Q4))/6250 - (591*np.sin(Q2 + Q3)*np.cos(Q4)*np.sin(Q1))/6250 + (1569*np.cos(Q2)*np.cos(Q3)*np.sin(Q1))/4000, np.cos(Q1)*((1569*np.sin(Q2 + Q3))/4000 + (17*np.sin(Q2))/40 + np.sin(Q5)*((823*np.cos(Q2 + Q3)*np.sin(Q4))/10000 + (823*np.sin(Q2 + Q3)*np.cos(Q4))/10000) + (591*np.cos(Q2 + Q3)*np.cos(Q4))/6250 - (591*np.sin(Q2 + Q3)*np.sin(Q4))/6250),                         np.cos(Q1)*((591*np.cos(Q2 + Q3 + Q4))/6250 + (1569*np.sin(Q2 + Q3))/4000 + (823*np.sin(Q2 + Q3 + Q4)*np.sin(Q5))/10000),                         np.cos(Q1)*((591*np.cos(Q2 + Q3 + Q4))/6250 + (823*np.sin(Q2 + Q3 + Q4)*np.sin(Q5))/10000), (823*np.cos(Q1)*np.cos(Q2)*np.cos(Q5)*np.sin(Q3)*np.sin(Q4))/10000 - (823*np.cos(Q1)*np.cos(Q2)*np.cos(Q3)*np.cos(Q4)*np.cos(Q5))/10000 - (823*np.sin(Q1)*np.sin(Q5))/10000 + (823*np.cos(Q1)*np.cos(Q3)*np.cos(Q5)*np.sin(Q2)*np.sin(Q4))/10000 + (823*np.cos(Q1)*np.cos(Q4)*np.cos(Q5)*np.sin(Q2)*np.sin(Q3))/10000, 0],
		# [ (2183*np.sin(Q1))/20000 - (17*np.cos(Q1)*np.cos(Q2))/40 + (823*np.cos(Q5)*np.sin(Q1))/10000 - (823*np.cos(Q2 + Q3 + Q4)*np.cos(Q1)*np.sin(Q5))/10000 + (591*np.cos(Q2 + Q3)*np.cos(Q1)*np.sin(Q4))/6250 + (591*np.sin(Q2 + Q3)*np.cos(Q1)*np.cos(Q4))/6250 - (1569*np.cos(Q1)*np.cos(Q2)*np.cos(Q3))/4000 + (1569*np.cos(Q1)*np.sin(Q2)*np.sin(Q3))/4000, np.sin(Q1)*((1569*np.sin(Q2 + Q3))/4000 + (17*np.sin(Q2))/40 + np.sin(Q5)*((823*np.cos(Q2 + Q3)*np.sin(Q4))/10000 + (823*np.sin(Q2 + Q3)*np.cos(Q4))/10000) + (591*np.cos(Q2 + Q3)*np.cos(Q4))/6250 - (591*np.sin(Q2 + Q3)*np.sin(Q4))/6250),                         np.sin(Q1)*((591*np.cos(Q2 + Q3 + Q4))/6250 + (1569*np.sin(Q2 + Q3))/4000 + (823*np.sin(Q2 + Q3 + Q4)*np.sin(Q5))/10000),                         np.sin(Q1)*((591*np.cos(Q2 + Q3 + Q4))/6250 + (823*np.sin(Q2 + Q3 + Q4)*np.sin(Q5))/10000), (823*np.cos(Q1)*np.sin(Q5))/10000 - (823*np.cos(Q2)*np.cos(Q3)*np.cos(Q4)*np.cos(Q5)*np.sin(Q1))/10000 + (823*np.cos(Q2)*np.cos(Q5)*np.sin(Q1)*np.sin(Q3)*np.sin(Q4))/10000 + (823*np.cos(Q3)*np.cos(Q5)*np.sin(Q1)*np.sin(Q2)*np.sin(Q4))/10000 + (823*np.cos(Q4)*np.cos(Q5)*np.sin(Q1)*np.sin(Q2)*np.sin(Q3))/10000,                      0],
		# [                                                                                                                                                                                                                                                                                            0,                                                      (591*np.sin(Q2 + Q3 + Q4))/6250 - (823*np.sin(Q2 + Q3 + Q4 + Q5))/20000 - (1569*np.cos(Q2 + Q3))/4000 - (17*np.cos(Q2))/40 + (823*np.sin(Q2 + Q3 + Q4 - Q5))/20000, (591*np.sin(Q2 + Q3 + Q4))/6250 - (823*np.sin(Q2 + Q3 + Q4 + Q5))/20000 - (1569*np.cos(Q2 + Q3))/4000 + (823*np.sin(Q2 + Q3 + Q4 - Q5))/20000, (591*np.sin(Q2 + Q3 + Q4))/6250 - (823*np.sin(Q2 + Q3 + Q4 + Q5))/20000 + (823*np.sin(Q2 + Q3 + Q4 - Q5))/20000,                                                                                                                                                                           - (823*np.sin(Q2 + Q3 + Q4 + Q5))/20000 - (823*np.sin(Q2 + Q3 + Q4 - Q5))/20000,                                                     0],
		# [                                                                                                                                                                                                                                                                                            0,                                                                                                                                                                                                  np.sin(Q1),                                                                                                                           np.sin(Q1),                                                                                                np.sin(Q1),                                                                                                                                                                                                                           np.sin(Q2 + Q3 + Q4)*np.cos(Q1),   np.cos(Q5)*np.sin(Q1) - np.cos(Q2 + Q3 + Q4)*np.cos(Q1)*np.sin(Q5)],
		# [                                                                                                                                                                                                                                                                                            0,                                                                                                                                                                                                 -np.cos(Q1),                                                                                                                          -np.cos(Q1),                                                                                               -np.cos(Q1),                                                                                                                                                                                                                           np.sin(Q2 + Q3 + Q4)*np.sin(Q1), - np.cos(Q1)*np.cos(Q5) - np.cos(Q2 + Q3 + Q4)*np.sin(Q1)*np.sin(Q5)],
		# [                                                                                                                                                                                                                                                                                            1,                                                                                                                                                                                                        0,                                                                                                                                 0,                                                                                                      0,                                                                                                                                                                                                                                  -np.cos(Q2 + Q3 + Q4),                            -np.sin(Q2 + Q3 + Q4)*np.sin(Q5)]])
		# eff_real=jbian.dot(jstate.velocity)
		
		#dummy=10*np.linalg.det(jacobian)
		#det=np.append(det,[dummy])
		print("stage5")
		time=np.append(time,[t])

		#speed2=np.append(speed2, [eff_real[1]] )
		speed=np.append(speed, [end_eff_vel[1]] )

		#joint_data1=np.append(joint_data1,[jstate.position[0]])
		#joint_data2=np.append(joint_data2,[jstate.position[1]])
		#joint_data3=np.append(joint_data3,[jstate.position[2]])
		#joint_data4=np.append(joint_data4,[jstate.position[3]])
		#joint_data5=np.append(joint_data5,[jstate.position[4]])
		#joint_data6=np.append(joint_data6,[jstate.position[5]])
                #print(type(q_dot))
                #print(joint_data1)
		
		pub.publish(joint_command)
                raw_input()
		rate.sleep()
		t=t+0.01
		#i=i+1

#	plot.axis([0,10,-0.02	,-0.0])
#	plot.figure()	
#	plot.ion()
#	plot.plot(time,joint_data1)
#	plot.plot(time,joint_data2)
#	plot.plot(time,joint_data3)
#	plot.plot(time,joint_data4)
#	plot.plot(time,joint_data5)
#	plot.plot(time,joint_data6)

	#plot.figure()
	#plot.axis([0,10,-2.0,2.0])
	#plot.ion()
	#plot.plot(time,speed2)
	#plot.show()

	#plot.figure()
	#plot.axis([0,10,-2.0,2.0])
	#plot.ion()
	#plot.plot(time,speed)
	#plot.pause(2000.0)
if __name__ == '__main__':
        print("Code started")
	try:
		sender()
	except rospy.ROSInterruptException:
		pass


