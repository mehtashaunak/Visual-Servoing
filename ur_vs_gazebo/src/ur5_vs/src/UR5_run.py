#!/usr/bin/env python

import rospy
import numpy as np
import time


from visual_serv_ur5_nodes.msg import std_mssg
# from ur5_vs import msgs.joint_angles
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_msgs.msg import Float64MultiArray

from control_msgs.msg import *
from sensor_msgs.msg import *
from trajectory_msgs.msg import *

# joint_vel=[-0.0,-0.02,0,0,0,0] 
JOINT_NAMES=[ 'shoulder_pan_joint','shoulder_lift_joint','elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
# posn = [0,0,0,0,0,0]
start = (time.time())
iter = []
end = None
th1 =th2=th3=th4=th5=th6=0
th = np.array([0,0,0,0,0,0])
# main function
# Varm = np.array([-0.0,-0.1,0,0,0,0])


def run_ur5():
    global end,start,th
    rospy.init_node('run_ur5', anonymous=True)

    rospy.Subscriber('/joint_states', JointState, JS, queue_size = 1)

    while not rospy.is_shutdown():
        velocity=[0.0,0.0,0.0,0.0,0.0,0.03]
        pub = rospy.Publisher('/ur_driver/joint_speed', JointTrajectory, queue_size=1)

        rate = rospy.Rate(500) # 10hz
        hello_str = JointTrajectory()
        hello_str.header = Header()
        hello_str.joint_names=JOINT_NAMES
        hello_str.points=[JointTrajectoryPoint(velocities=velocity, time_from_start=rospy.Duration(0.0)),JointTrajectoryPoint(time_from_start=rospy.Duration(0.0))]
        hello_str.header.seq=hello_str.header.seq+1
        hello_str.header.stamp-rospy.Time.now()
        pub.publish(hello_str)
        rate.sleep()

       

def v2():
    global end,start,th
    rospy.init_node('run_ur5', anonymous=True)

    rospy.Subscriber('/joint_states', JointState, JS, queue_size = 1)

    while not rospy.is_shutdown():
        velocity=[0.05,-0.0,0,0,0,0]
        pub = rospy.Publisher('/ur_driver/joint_speed', JointTrajectory, queue_size=1)

        rate = rospy.Rate(500) # 10hz
        hello_str = JointTrajectory()
        hello_str.header = Header()
        hello_str.joint_names=JOINT_NAMES
        hello_str.points=[JointTrajectoryPoint(velocities=velocity, time_from_start=rospy.Duration(0.0)),JointTrajectoryPoint(time_from_start=rospy.Duration(0.0))]
        hello_str.header.seq=hello_str.header.seq+1
        hello_str.header.stamp-rospy.Time.now()
        pub.publish(hello_str)
        rate.sleep()

def JS(jss):
    global th,th1,th2,th3,th4,th5,th6
    js = np.array(jss.position)
    th1 = js[0]
    th2 = js[1]
    th3 = js[2]
    th4 = js[3]
    th5 = js[4]
    th6 = js[5]
    th  = np.array([th1,th2, th3,th4,th5,th6])


if __name__ == '__main__':
    try:
        run_ur5()
    except rospy.ROSInterruptException:
        pass
