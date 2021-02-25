#!/usr/bin/env python

import rospy
import numpy as np
import time


from visual_serv_ur5_nodes.msg import std_mssg

# from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_msgs.msg import Float64MultiArray

from control_msgs.msg import *
from trajectory_msgs.msg import *

joint_vel=[-0.0,-0.02,0,0,0,0] 
JOINT_NAMES=['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint','wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
posn = [0,0,0,0,0,0]
start = (time.time())
iter = []
end = None
# main function
Varm = np.array([0,0,0,0,0,0])


def run_ur5():
    global joint_vel,posn,end,start,Varm
    rospy.init_node('run_ur5_Z', anonymous=True)

    rospy.Subscriber('Arm_velocity', Float64MultiArray, Arm_vel, queue_size = 10)

    while not rospy.is_shutdown():
        pub = rospy.Publisher('/ur_driver/joint_speed', JointTrajectory, queue_size=10)
        rate = rospy.Rate(500) # 10hz
        hello_str = JointTrajectory()
        hello_str.header = Header()
        hello_str.joint_names=JOINT_NAMES 
        # hello_str.points=[JointTrajectoryPoint(positions=posn, time_from_start=rospy.Duration(0.0)),JointTrajectoryPoint(time_from_start=rospy.Duration(0.0))]
        end = (time.time())
        # print(end-start)
        # if ((end-start) <= 1):
        hello_str.points=[JointTrajectoryPoint(velocities=0.05*Varm)]
        # print(0.001*Varm)
        # if ((end-start)>=1):
        #     if ((end-start)<=2):
        #         hello_str.points=[JointTrajectoryPoint(velocities=v2())]
        #     else:
        #         hello_str.points=[JointTrajectoryPoint(velocities=v2())]
        #         start = end
            

        hello_str.header.seq=hello_str.header.seq+1
        hello_str.header.stamp-rospy.Time.now()
        pub.publish(hello_str)
        # print(time.time())
        # hello_str.points=[JointTrajectoryPoint(velocities=v2())]
        # pub.publish(hello_str)
        # rospy.sleep(2)
        rate.sleep()

        # joint_states()

def Arm_vel(vel):
    global Varm
    # print(rospy.Duration(2))
    joint_vel = np.asarray(vel.data)
    Varm = np.array([joint_vel[0], joint_vel[1], joint_vel[2], joint_vel[3], joint_vel[4], joint_vel[5]])
    # print(Varm)
    # return Varm

def v2():
    # rospy.Duration(10)
    joint_vel=[-0.0,-0.0,0,0,0,0] 
    return joint_vel

def stop_ur5():
    
    rospy.Subscriber('Arm_velocity', Float64MultiArray, Arm_vel, queue_size = 10)

    while not rospy.is_shutdown():
        pub = rospy.Publisher('/ur_driver/joint_speed', JointTrajectory, queue_size=10)
        rate = rospy.Rate(500) # 10hz
        hello_str = JointTrajectory()
        hello_str.header = Header()
        hello_str.joint_names=JOINT_NAMES 
        # hello_str.points=[JointTrajectoryPoint(positions=posn, time_from_start=rospy.Duration(0.0)),JointTrajectoryPoint(time_from_start=rospy.Duration(0.0))]
        end = (time.time())
        # print(end-start)
        # if ((end-start) <= 1):
        hello_str.points=[JointTrajectoryPoint(velocities=0.000*Varm)]
        print(0.000*Varm)
        # if ((end-start)>=1):
        #     if ((end-start)<=2):
        #         hello_str.points=[JointTrajectoryPoint(velocities=v2())]
        #     else:
        #         hello_str.points=[JointTrajectoryPoint(velocities=v2())]
        #         start = end
            

        hello_str.header.seq=hello_str.header.seq+1
        hello_str.header.stamp-rospy.Time.now()
        pub.publish(hello_str)
        # print(time.time())
        # hello_str.points=[JointTrajectoryPoint(velocities=v2())]
        # pub.publish(hello_str)
        # rospy.sleep(2)
        rate.sleep()

        # joint_states()
    





if __name__ == '__main__':
    try:
        run_ur5()
    except rospy.ROSInterruptException:
        stop_ur5()