import numpy as np
import cv2
from cv2 import aruco

#Get the camera source
cap =cv2.VideoCapture(0)

def track(calibration_matrix, distortion_coefficients):
    while True:
        ret, frame = cap.read()

        # operation on the frame
        # Change to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        #choosing the dictionary, 5X5 dictionary to find markers
        aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
        #create detector parameters
        arucoParams = aruco.DetectorParameters_create()

        #list of ids and corners belonging to each id
        (corners, ids, rejected_img_points) = aruco.detectMarkers(gray, aruco_dict,parameters=
        arucoParams,cameraMatrix = calibration_matrix,distcoeff = distortion_coefficients)
        # parameters of the fuctions are
            # gray:- grayscale image
            # aruco_dict:- dictionary we created
            # parameters:- detector parameters
            # cameraMatrix:- calibration matric from the camera calibration process
            # distCoeff:- Distortion coeff from camera calibration process
        
        # corners:- for every marker fucntion has found, we will get 4 corners.
        # ids:- ids of the marker.
        # rejected_img_points:- corner points of marker candidates rejected by function

        if np.all(ids is not None): # if there are markers found by detector
            for i in range(0, len(ids)) # iterate in markers

            # estimate pose of each marker and return the value of R_vec, T_vec ---
            # different from camera coefficeints
            R_vec, T_vec, markerpoints = aruco.estimatePoseSingleMarkers(corners[i],0.02,
            calibration_matrix, distortion_coefficients)

            # get rid of numpy value array error
            (R_vec-T_vec).any()
            
            # Draw a sqaure around the markers
            aruco.drawDetectedMarkers(frame, corners)

            # Draw axis
            aruco.drawAxis(frame, calibration_matrix, distortion_coefficients, R_vec,T_vec,0.01)

            # Display the resulting frame
            cv2.imshow('frame',frame)
            key = cv2.waitKey(3) & 0xFF
            if key == ord('q'):
                break

cap.release()
cv2.distroyALLWindows()





















        




