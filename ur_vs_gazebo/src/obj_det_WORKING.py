
import numpy as np
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
# from dd_ws.msg import custom_msg
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv2 import aruco

bridge = CvBridge()
# frame = np.zeros((480,640,3),np.uint8)
rgb_image = np.zeros([480, 640, 3], np.uint8)
# depth_img = np.zeros((480,640))
# depth_image = np.zeros([480, 640])


def rgb_callback(rgb_msg):
    global rgb_image
    rgb_image = bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
    # rgb_image = np.flipud(rgb_image)
    # cv2.imshow('rgb_image',rgb_image)


# def depth_callback(depth_msg):
    # global depth_image
    # depth_image = bridge.imgmsg_to_cv2(
        # depth_msg, desired_encoding="passthrough")


# def Feature_points(out_points):
#     global key, fix_points
#     # publish fixed points
#     if key == ord('s'):
#         fix_points = out_points[0]
#     pub_msg_fix.data = fix_points.flatten()
#     pubfix.publish(pub_msg_fix)

#     # publish current points
#     cur_points = out_points[0]
#     pub_msg_cur.data = cur_points.flatten()
#     pub_msg_cur.text = str(out_points[1])
#     pubcur.publish(pub_msg_cur)


def Object_detection():
    global key
    global cameraMatrix
    global distCoeffs

    # cameraMatrix = np.array([[554.254691191187, 0.0, 320.5], [0.0, 554.254691191187, 240.5],[0.0, 0.0, 1.0]])
    # print(cameraMatrix)
    # cameraMatrix = np.array([[607.4385986328125, 0.0, 315.3274841308594], [0.0, 606.0167846679688, 241.0012969970703],[0.0, 0.0, 1.0]])
    # cameraMatrix = np.array([[525.0, 0.0, 319.5], [0.0, 525.0, 239.5],[0.0, 0.0, 1.0]])
    cameraMatrix = np.array([[536.1547638382946, 0.0, 328.2989889116701], [0.0, 534.2242890469014, 263.4929383043271],[0.0, 0.0, 1.0]])
    # distCoeffs= np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    distCoeffs= np.array([0.16587981496803042, -0.3142580999644871, 0.0014280230760082854, 0.003129163390032306, 0.0])
    # distCoeffs= np.array([0.402, -0.497, 0.045, 0.019, 0.0])

    gray = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
    # cv2.imshow('gray',gray)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
    # print(aruco_dict)
    arucoParams = aruco.DetectorParameters_create()
    # corners, ids, rejected_imgpoints = aruco.detectMarkers(gray,aruco_dict, parameters=arucoParams)
    corners, ids, rejected_imgpoints = aruco.detectMarkers(gray,aruco_dict,cameraMatrix,distCoeffs, parameters=arucoParams)

    print(corners)
    # print(ids)
    
    # corners, ids, rejected_imgpoints = aruco.detectMarkers(rgb_image,aruco_dict, calibration_matrix=
    # np.array([[525.0, 0.0, 319.5], [0.0, 525.0, 239.5],[0.0, 0.0, 1.0]]), parameters=arucoParams)

    # while True:
    # ret, rgb_image = cap.read()

    # operation on the frame
    # Change to grayscale
    # gray = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)

    # choosing the dictionary, 5X5 dictionary to find markers
    # aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
    # create detector parameters
    # arucoParams = aruco.DetectorParameters_create()

    # list of ids and corners belonging to each id
    # (corners, ids, rejected_img_points) = aruco.detectMarkers(gray,
    #   aruco_dict, parameters=arucoParams, cameraMatrix=calibration_matrix)
    # , distcoeff=distortion_coefficients)
    # parameters of the fuctions are
    # gray:- grayscale image
    # aruco_dict:- dictionary we created
    # parameters:- detector parameters
    # cameraMatrix:- calibration matric from the camera calibration process
    # distCoeff:- Distortion coeff from camera calibration process

    # corners:- for every marker fucntion has found, we will get 4 corners.
    # ids:- ids of the marker.
    # rejected_img_points:- corner points of marker candidates rejected by function

    if np.all(ids is not None):  # if there are markers found by detector
        for i in range(0, len(ids)):  # iterate in markers

            # estimate pose of each marker and return the value of R_vec, T_vec ---
            # different from camera coefficeints

            R_vec,T_vec,_objPoints = aruco.estimatePoseSingleMarkers(corners[i], 0.05,cameraMatrix,distCoeffs)
            print("R_vec",R_vec)
            print("T_vec",T_vec)
            # R_vec, T_vec, markerpoints = aruco.estimatePoseSingleMarkers(
            # corners[i], 0.02, calibration_matrix)

            # , distortion_coefficients)
            c_x = (corners[i][0][0][0] + corners[i][0][1][0] + 
            corners[i][0][2][0] + corners[i][0][3][0]) / 4 # X coordinate of marker's center

            c_y = (corners[i][0][0][1] + corners[i][0][1][1] + 
            corners[i][0][2][1] + corners[i][0][3][1]) / 4 # Y coordinate of marker's center
            print("c_x = ",c_x)
            print("c_y = ",c_y)


            # get rid of numpy value array error
            (R_vec-T_vec).any()
            aruco.drawAxis(rgb_image,cameraMatrix,distCoeffs,R_vec, T_vec, 0.1)
            # cv2.imshow('axis',rgb_image)

            # Draw a sqaure around the markers
        det = aruco.drawDetectedMarkers(rgb_image, corners)
        cv2.imshow('image',det)

            # Draw axis
    # aruco.drawAxis(gray,cameraMatrix,distCoeffs,R_vec, T_vec, 0.01)

    # c_x = (corners[i][0][0][0] + corners[i][0][1][0] + corners[i][0][2][0] + corners[i][0][3][0]) / 4 # X coordinate of marker's center
    # c_y = (corners[i][0][0][1] + corners[i][0][1][1] + corners[i][0][2][1] + corners[i][0][3][1]) / 4 # Y coordinate of marker's center

    # print(c_x)
    # print(c_y)       
            # aruco.drawAxis(rgb_image, calibration_matrix, R_vec, T_vec, 0.01)

            # aruco.drawAxis(frame, calibration_matrix,
            # distortion_coefficients, R_vec, T_vec, 0.01)

    # c = np.zeros([4, 3])
    # if ids is not None:
    #     corners

# def main():
#     Ros_to_cv_object = Ros_to_cv()
#     rospy.init_node('Ros_to_cv_node',anonymous=True)
#     try:
#         rospy.spin()
#     except KeyboardInterrupt:
#         print("shutting down")


if __name__ == '__main__':
    global key
    try:
        # Initialization node
        rospy.init_node('Aruco_detection')

        # Subscribing
        # rospy.Subscriber('/camera/color/image_raw', Image, rgb_callback)
        rospy.Subscriber('/camera/rgb/image_raw', Image, rgb_callback)
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
