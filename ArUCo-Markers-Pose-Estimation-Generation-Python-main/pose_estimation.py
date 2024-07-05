'''
Sample Usage:-
python pose_estimation.py --K_Matrix calibration_matrix.npy --D_Coeff distortion_coefficients.npy --type DICT_5X5_100
'''


import numpy as np
import cv2
import sys
from utils import ARUCO_DICT
import argparse
import time

taille_marqueur = 0.05
# Camera intrinsic parameters (already in pixels)
fx = 828.276
fy = 826.467
cx = 370.122
cy = 171.748
# Assuming no lens distortion
#dist_coeffs = np.zeros(5)
dist_coeffs = np.array([ 4.85659514e-02 , 1.34388697e+00  ,6.45864444e-03,  1.14625698e-03,-1.82063778e+01])
# Prepare the camera matrix
camera_matrix = np.array([[fx, 0, cx],[0, fy, cy],[0, 0, 1]])

def my_estimatePoseSingleMarkers(corners, tag_size, mtx, distortion):
    '''
    This will estimate the rvec and tvec for each of the marker corners detected by:
       corners, ids, rejectedImgPoints = detector.detectMarkers(image)
    corners - is an array of detected corners for each detected marker in the image
    marker_size - is the size of the detected markers
    mtx - is the camera matrix
    distortion - is the camera distortion matrix
    RETURN list of rvecs, tvecs, and trash (so that it corresponds to the old estimatePoseSingleMarkers())
    '''
    #marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
     #                         [marker_size / 2, marker_size / 2, 0],
      #                        [marker_size / 2, -marker_size / 2, 0],
       #                       [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
    
    marker_points = np.array([
            [-tag_size / 2, -tag_size / 2, 0],
            [tag_size / 2, -tag_size / 2, 0],
            [tag_size / 2, tag_size / 2, 0],
            [-tag_size / 2, tag_size / 2, 0]
            ], dtype=np.float32)
    #print(marker_points)
    trash = []
    rvecs = []
    tvecs = []
    corners2=[]
    for i in range (4) :
        corners2.append([corners[0][0][i][0] , corners[0][0][i][1]]) 
    corners2 = np.array(corners2,dtype=np.float32 )
    print ("Coordonnées sommets (en pxl)-------------")
    print (corners2)
    _, rvecs, tvecs = cv2.solvePnP(marker_points, corners2, mtx ,distortion)

    #for c in corners:
    #    nada, R, t = cv2.solvePnP(marker_points, c, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
    #    rvecs.append(R)
    #    tvecs.append(t)
    #    trash.append(nada)
    return rvecs, tvecs, trash



def pose_esitmation(frame, aruco_dict_type, matrix_coefficients, distortion_coefficients):

    '''
    frame - Frame from the video stream
    matrix_coefficients - Intrinsic matrix of the calibrated camera
    distortion_coefficients - Distortion coefficients associated with your camera

    return:-
    frame - The frame with the axis drawn on it
    '''

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #cv2.aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_type)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_1000)
    #parameters = cv2.aruco.DetectorParameters_create()
    parameters =  cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)


    #corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, cv2.aruco_dict,parameters=parameters,cameraMatrix=matrix_coefficients, distCoeff=distortion_coefficients)
    corners, ids , rejectedCandidates = detector.detectMarkers(gray)

        # If markers are detected
    if len(corners) > 0:
        for i in range(0, len(ids)):
            # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
            #rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, matrix_coefficients,distortion_coefficients)
            rvec, tvec, trash = my_estimatePoseSingleMarkers(corners, taille_marqueur, matrix_coefficients,distortion_coefficients)
            print("rvec :")
            print (rvec)
            print("tvec :")
            print(tvec)
            # Draw a square around the markers
            cv2.aruco.drawDetectedMarkers(frame, corners) 

            # Draw Axis
            #cv2.aruco.drawAxis(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01)  

    return frame

if __name__ == '__main__':

    ap = argparse.ArgumentParser()
    ap.add_argument("-k", "--K_Matrix", required=True, help="Path to calibration matrix (numpy file)")
    ap.add_argument("-d", "--D_Coeff", required=True, help="Path to distortion coefficients (numpy file)")
    ap.add_argument("-t", "--type", type=str, default="DICT_ARUCO_ORIGINAL", help="Type of ArUCo tag to detect")
    args = vars(ap.parse_args())

    
    if ARUCO_DICT.get(args["type"], None) is None:
        print(f"ArUCo tag type '{args['type']}' is not supported")
        sys.exit(0)

    aruco_dict_type = ARUCO_DICT[args["type"]]
    calibration_matrix_path = args["K_Matrix"]
    distortion_coefficients_path = args["D_Coeff"]
    
    #k = np.load(calibration_matrix_path)
    #d = np.load(distortion_coefficients_path)
    k = camera_matrix
    d  = dist_coeffs

    video = cv2.VideoCapture("/dev/video0")
    time.sleep(2.0)

    while True:
        ret, frame = video.read()

        if not ret:
            break
        
        output = pose_esitmation(frame, aruco_dict_type, k, d)

        cv2.imshow('Estimated Pose', output)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        time.sleep(0.5)


    video.release()
    cv2.destroyAllWindows()