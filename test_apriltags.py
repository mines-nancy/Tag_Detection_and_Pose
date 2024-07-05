import cv2
import numpy as np
import apriltag
import math
import streamlit as st
import time



# tag size 
tag_size = 0.05 # The size of the AprilTag in meters (adjust as necessary)


### PARAMETRES CAMERA  + STREAMLIT ###

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
# Streamlit
cap = cv2.VideoCapture("/dev/video0")
st.title("Webcam Live Feed")
run = st.checkbox('Run')
FRAME_WINDOW = st.image([])


# BOUCLE PRINCIPALE #

while run:
    _, image = cap.read()
    cv2.waitKey(60)
    cv2.imwrite("photo_AT.png",image)

    ### Detection AprilTag fenêtre entière  ###
    imageg = cv2.imread("//home/pc-techlab-ia-2/Documents/TOSELLO/projet_video/photo_AT.png", cv2.IMREAD_GRAYSCALE)

    # Initialize the AprilTag detector
    options = apriltag.DetectorOptions(families="tag36h11")
    detector = apriltag.Detector(options)

    # Detect AprilTags in the image
    results = detector.detect(imageg)


    # Process each detected AprilTag
    for r in reversed(results): # on inverse pour lire les AT absolus en priorité : ordre murs--> portes--> sols
        tag_id = r.tag_id
        # chercher les AT absolus murs et se positionner
        if tag_id >= 0 :
            #print(r.corners)
            (ptA, ptB, ptC, ptD) = r.corners
            ptA = (int(ptA[0]), int(ptA[1]))
            ptB = (int(ptB[0]), int(ptB[1]))
            ptC = (int(ptC[0]), int(ptC[1]))
            ptD = (int(ptD[0]), int(ptD[1]))

            # Draw the bounding box of the detection
            cv2.line(image, ptA, ptB, (0, 255, 0), 2)
            cv2.line(image, ptB, ptC, (0, 255, 0), 2)
            cv2.line(image, ptC, ptD, (0, 255, 0), 2)
            cv2.line(image, ptD, ptA, (0, 255, 0), 2)

            # Draw the center (x, y)-coordinates of the AprilTag
            (cX, cY) = (int(r.center[0]), int(r.center[1]))
            cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)

            # Estimate the pose of the AprilTag
            # Define the 3D coordinates of the AprilTag corners in its own coordinate system
            object_points = np.array([
            [-tag_size / 2, -tag_size / 2, 0],
            [tag_size / 2, -tag_size / 2, 0],
            [tag_size / 2, tag_size / 2, 0],
            [-tag_size / 2, tag_size / 2, 0]
            ], dtype=np.float32)

            # SolvePnP to get rotation and translation vectors
            _, rvec, tvec = cv2.solvePnP(object_points, r.corners, camera_matrix, dist_coeffs)

            # Print the tag family and ID
            tag_family = r.tag_family.decode("utf-8")
            print(f"Tag Family: {tag_family}, Tag ID: {tag_id}")
            print(f"Tag Corners: {r.corners}")
            print(f"Rotation Vector:\n{rvec}")
            print(f"Translation Vector:\n{tvec}")

    # Display the output image
    #cv2.imshow('AprilTag Detection', image)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    FRAME_WINDOW.image(image)
    time.sleep(0.5)

else:
    st.write('Stopped')

cv2.waitKey(0)
cv2.destroyAllWindows()


