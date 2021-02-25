import numpy as np
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from cv2 import aruco

bridge = CvBridge()

frame = np.zeros([480, 640, 3], np.uint8)

def rgb_callback(rgb_msg):
    global frame
    frame = bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
    # frame = np.flipud(frame)

def Object_detection():
    global key
    global mtx
    global dist

    mtx = np.array([[554.254691191187, 0.0, 320.5], [0.0, 554.254691191187, 240.5],[0.0, 0.0, 1.0]])
    # mtx = np.array([[554.254691191187, 0.0, 320.5], [0.0, 554.254691191187, 240.5],[0.0, 0.0, 1.0]])
    dist= np.array([0.0, 0.0, 0.0, 0.0, 0.0])

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_100)
    parameters = aruco.DetectorParameters_create()
    parameters.adaptiveThreshConstant = 10
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    print(corners)

    if np.all(ids != None):
        rvec, tvec ,_ = aruco.estimatePoseSingleMarkers(corners, 0.10, mtx, dist)

        for i in range(0, ids.size):
            aruco.drawAxis(frame, mtx, dist, rvec[i], tvec[i], 0.05)

        aruco.drawDetectedMarkers(frame, corners)
    
    cv2.imshow('frame',frame)
  

if __name__ == '__main__':
    global key
    try:
        # Initialization node
        rospy.init_node('Aruco_detection')

        # Subscribing
        rospy.Subscriber('/camera/color/image_raw', Image, rgb_callback)
        # rospy.Subscriber('/coppeliaSim/camera/depth_image', Float32MultiArray,
                        #  depth_callback)
        while not rospy.is_shutdown():
            key = cv2.waitKey(1) & 0xFF

            Object_detection()
            if key == ord('q'):
                break
        # KILL ALL WINDOWS
        cv2.destroyAllWindows()

    except rospy.ROSInterruptException:
        pass
