import cv2
import numpy as np
import apriltag
import math
import streamlit as st
from fonctions_utiles import *

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

objet = [0.1,0.1] # on définie les dimensions de notre objet (ici un cibe de 10x10cm représenté en vue de dessus 2D par un carré)
origine = [0,0] # origine du repère en mètres 
origine_repere = [559,479] # origine du repère sur le moniteur (en pixels)


# BOUCLE PRINCIPALE #

while run:
    _, image = cap.read()
    cv2.waitKey(30)
    cv2.imwrite("photo_AT.png",image)

    ### Detecter une zone verte --> on a choisi (arbitrairement que les tags objets seraint encadrés en vert) ###


    ##  Filtrage couleur verte 
    b,g,r = cv2.split(image)
    ret,th_r=cv2.threshold(r,80,255,cv2.THRESH_BINARY) # seuillage manuel 
    th_r = cv2.bitwise_not(th_r) # logique NOT
    ret,th_g=cv2.threshold(g,100,255,cv2.THRESH_BINARY) # seuillage manuel 
    ret,th_b=cv2.threshold(b,100,255,cv2.THRESH_BINARY) # seuillage manuel 
    th_b = cv2.bitwise_not(th_b) # logique NOT

    th_fg = cv2.bitwise_and(th_b,th_r)
    th_fg = cv2.bitwise_and(th_fg,th_g)

    ## Detection de contours d'objets verts
    contours, hierarchy = cv2.findContours(th_fg,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE) #on extrait de l’image binaire , Ret_tree signifie que l’on veut tout les contours et chain_ approx_simple que les contours doivent être représentés par une suite de point
    hauteur_max = 0
    cont = -1
    for i in range (len(contours)) : # boucle opérant sur chaque contour
        x,y,w,h = cv2.boundingRect(contours[i]) # coo et dim du rectangle de taille minimum contenant le contours
        if h > hauteur_max : 
            hauteur_max = h
            cont = i 
            xm = x
            ym=y
            wm=w
            hm=h

    ## Objet(s) vert détecté(s) --> on cherche des tags dans le contour vert détecté
    if hauteur_max > 20 and cont>-1 :
        tag_size = 0.05 # Renseigner la taille des Apriltags (en mètres)
 
        #crop_img = image[0:0 , 100:100]
        crop_img = image[ym:ym+hm, xm:xm+wm]
        cv2.imwrite("photo_AT_crop.png",crop_img)
        imageg = cv2.imread("//home/pc-techlab-ia-2/Documents/TOSELLO/projet_video/photo_AT.png", cv2.IMREAD_GRAYSCALE)
        # Initialize the AprilTag detector
        options = apriltag.DetectorOptions(families="tag36h11")
        detector = apriltag.Detector(options)
        # Detect AprilTags in the image
        results = detector.detect(imageg)
        # Dessin de l'écran de contrôle (carte + position robot) sur l'image #
        cv2.rectangle(image, (479,359), (639, 479), (255,255,255), thickness=-1)
        cv2.line(image, (559,479), (479,359), (0,255,255), thickness=1)
        cv2.line(image, (559,479), (559,359), (0,255,255), thickness=1)
        cv2.line(image, (559,479), (639,359), (0,255,255), thickness=1)
                 

        for r in results :
            tag_id = r.tag_id
            # Dessin du centre de l'AT
            (cX, cY) = (int(r.center[0]), int(r.center[1]))
            cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)
            if cX>xm and cX<xm+wm and cY>ym and cY<ym+hm : # on vérifie si l'AT détecté est bien dans le contours
                # Récupération des coordonnées des sommets de l'AT
                (ptA, ptB, ptC, ptD) = r.corners
                ptA = (int(ptA[0]), int(ptA[1]))
                ptB = (int(ptB[0]), int(ptB[1]))
                ptC = (int(ptC[0]), int(ptC[1]))
                ptD = (int(ptD[0]), int(ptD[1]))
                couleur = (50,50,255)
                cv2.line(image, ptA, ptB, couleur, 2)
                cv2.line(image, ptB, ptC, couleur, 2)
                cv2.line(image, ptC, ptD, couleur, 2)
                cv2.line(image, ptD, ptA, couleur, 2)

                # Estimation de la position de l'AT (coordonnées en 3D dans son propre système de coordonnées (voir doc))
                object_points = np.array([
                [-tag_size / 2, -tag_size / 2, 0],
                [tag_size / 2, -tag_size / 2, 0],
                [tag_size / 2, tag_size / 2, 0],
                [-tag_size / 2, tag_size / 2, 0]
                ], dtype=np.float32)
                # Récupération des vecteurs de translation et de rotation de l'AT dans un repère de la caméra)
                _, rvec, tvec = cv2.solvePnP(object_points, r.corners, camera_matrix, dist_coeffs)
                # Impression de l'Id, des coordonnées et des vecteurs de rotation et de translation
                tag_family = r.tag_family.decode("utf-8")
                print(f"Tag Family: {tag_family}, Tag ID: {tag_id}")
                print(f"Tag Corners: {r.corners}")
                print(f"Rotation Vector:\n{rvec}")
                print(f"Translation Vector:\n{tvec}")
                # Calcul position du tag dans le repère de la caméra
                #xb,zb,omega = position_objet (tvec[0][0],tvec[2][0],rvec[1][0])
                # Postition de l'objet détecté représenté vu de dessus en 2D
                X1,Y1,X2,Y2,X3,Y3,X4,Y4 = placer_objet_2D(tvec[0][0],tvec[2][0],rvec[1][0],origine,objet)
                # dessin de l'objet sur l''écran de contrôle
                x1 = origine_repere[0] + meter_to_pixel(X1)
                y1 = origine_repere[1] + meter_to_pixel(Y1)
                x2 = origine_repere[0] + meter_to_pixel(X2)
                y2 = origine_repere[1] + meter_to_pixel(Y2)
                x3 = origine_repere[0] + meter_to_pixel(X3)
                y3 = origine_repere[1] + meter_to_pixel(Y3)
                x4 = origine_repere[0] + meter_to_pixel(X4)
                y4 = origine_repere[1] + meter_to_pixel(Y4)

                cv2.line(image, (x1,y1), (x2,y2), (0,0,255), thickness=1)
                cv2.line(image, (x3,y3), (x4,y4), (0,0,255), thickness=1)
                cv2.line(image, (x2,y2), (x4,y4), (0,0,255), thickness=1)
                cv2.line(image, (x3,y3), (x1,y1), (0,0,255), thickness=1)

    # Display the output image
    #cv2.imshow('AprilTag Detection', image)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    FRAME_WINDOW.image(image)




    



