#!/usr/bin/env python

import rospy
import numpy as np
import roslib
import sys
import cv2
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory
from ur5_vs.msg import joint_vel
from sensor_msgs.msg import Image
from PIL import Image as Im
import struct
from cv_bridge import CvBridge, CvBridgeError


global msg
global vel

def pubvel():
    global msg
    global vel
    global rgb_img
    rospy.init_node('publish', anonymous=True)
    pub=rospy.Publisher('joint_vel_cmd', joint_vel, queue_size=10)
#    r=rospy.Rate(100)
    jv = joint_vel()
#    i=0
    jv.vel0.data = 0
    jv.vel1.data = 0
    jv.vel2.data = 0
    jv.vel3.data = 0
    jv.vel4.data = 0
    jv.vel5.data = 0
#    vel=[0.1,-0.1,-0.1,0.2,0.2,0.2]
#    msg=joint_vel()
    while not rospy.is_shutdown():
        print "a"
        #vel[5]=-0.3
#        msg.points=[JointTrajectoryPoint(velocities=vel)]
#        rospy.loginfo(msg)
        pub.publish(jv)
        sub=rospy.Subscriber('camera/rgb/image_raw', Image, callback)
        rospy.sleep(1)


#def __init__(self):
#    self.image_pub = rospy.Publisher("image_topic_2",Image)
    
#    self.bridge = CvBridge()
#    self.image_sub = rospy.Subscriber('camera/rgb/image_raw',Image,self.callback)



def callback(data):
    bridge    = CvBridge()
    rgb_img   = np.zeros((480,640,3),np.uint8)
#    print data.height
#    print data.width
#    mat = np.empty((480,640,3))
#    for i in range(480):
#        for j in range(640):
#            testBytes = data.data[(j-1)*640+i]
#            testResult = struct.unpack('>B', testBytes)
#            mat[i][j] = testResult[0]
#    print mat
    
#    image_data = mat 
#    image = Im.frombytes('RGBA', (640,480), image_data)
#    image.save('im1.png')
#    image.show()
 
    print "b"
    try:
      rgb_img = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

#    print aa
    cv2.imshow("Image window", rgb_img)
    cv2.waitKey(0)


   


if __name__=='__main__':
    try:
        pubvel()
    except rospy.ROSInterruptException:
        pass

