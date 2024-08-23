import cv2
import numpy as np
import apriltag
import math
import streamlit as st
from fonctions_utiles import *




### PARAMETRAGE DE L'ENVIRONNEMENT ###



# Dimensions du robot #

dim_rob = [0.15,0.2] # renseigner ici largeur et longeur du robot(en m)
# on définiera les coordonnées du robot par 2 pts :  sa tête et sa queue (cette description est suffisante pour le placer sur une carte en 2 Dimensions)
delta = math.atan((dim_rob[0]/2)/dim_rob[1]) # delta correspond à l'angle entre la hauteur du triangle isocèle formé par la tête diu robot et ses pattes arrières et l'un de ses deux côté de même longueur
hypo = math.sqrt((dim_rob[0]/2)*(dim_rob[0]/2)+dim_rob[1]*dim_rob[1]) # hypo correspond à l'hypothenuse du demi triangle isocele defini par sa hauteur
# ces deux dernières variables servent uniquement à représenter le robot sur la carte, le robot est symnolisé par "V" dont la pointe est la tête du robot
# et les dux autres sommets les pattes arrières, on peu ainsi comprendre la position du robot et son orientation)

# Initialisation des matrices de position du robot et des variables de stockage #

mat_rob  = [-1,-1,0] # matrice position du robot (matrice en 3D, pour l'instant on fait du positionnement en 2D donc il n'y a pas d'information sur la 3e composante)
mat_rob_search= [[-1,-1,0],[-1,-1,0],[-1,-1,0]] # positions calculées en fonction des tags détectés
dist_tags=[0,0,0] # distance des tags détectés
coo_AT=[] # coordonnées de l'Apriltag détecté
AT_TARGETED = 0 # nombre d'Apriltags détectés
tag_size = 0.05 # Renseigner la taille des Apriltags (en mètres)

# Positionnement des AT sur la map #

# ces matrices contiennent les coordonnées (en m) des tags dans le repère de la maquette
# vous pouvez les copnstruire manuellement ou utiliser l'application map.py qui automatise la création de ces matrices
murs = [[0,0,0.75,0,'N'],[0.75,0,0.75,0.75,'E'],[0.75,0.75,0,0.75,'S'],[0,0.75,0,0,'W']]
MAP_rel = [[4 , [0, 0.075], 'W'],[2 , [0, 0.175], 'W'],[0 , [0, 0.275], 'W'],[1 , [0, 0.475], 'W'],[3 , [0, 0.575], 'W'],[5 , [0, 0.675], 'W'],
       [5 , [0.075, 0], 'N'],[3 , [0.175, 0], 'N'],[1 , [0.275, 0], 'N'],[0 , [0.475, 0], 'N'],[2 , [0.575, 0], 'N'],[4 , [0.675, 0], 'N']]
MAP_abs = [[101 , [0, 0.375], 'W'],[100 , [0.375, 0], 'N']]
MAP_abs_sol = [[104, [0.375, 0.375]]]
MAP_rel_sol = [[53, [0.125, 0.125]],[54, [0.375, 0.125]],[55, [0.625, 0.125]],[47, [0.125, 0.375]] ,[40, [0.125, 0.625]]]
PPSE = [0.75,0.75] # point le plus au sud-ouest de la MAP
origine_repere = [479+10,359+10] # position en pixel du repère sur la représentation graphique de la maquette dessinée sur le flux vidéo



### PARAMETRES CAMERA  + PARAMETRES STREAMLIT ###



# Paramètres intrinsèques de la caméra (en pxl), obtenus en après l'étape de calibration calibration de la caméra
# Renseigner ici les paramètres de la caméra que vous utilisez
fx = 828.276 
fy = 826.467
cx = 370.122
cy = 171.748
dist_coeffs = np.array([ 4.85659514e-02 , 1.34388697e+00  ,6.45864444e-03,  1.14625698e-03,-1.82063778e+01])
camera_matrix = np.array([[fx, 0, cx],[0, fy, cy],[0, 0, 1]])

# Streamlit #

cap = cv2.VideoCapture("/dev/video0") # création d'un flux vidéo (mettre en paramètre le port vidéo correspondant à votre caméra)
st.title("Webcam Live Feed") # création du stream 
run = st.checkbox('Run')
FRAME_WINDOW = st.image([])


### Traitement d'un  AT ###


def traiter_AT(r,image,type_AT) : 
    # Récupération des coordonnées des sommets de l'AT
    (ptA, ptB, ptC, ptD) = r.corners
    ptA = (int(ptA[0]), int(ptA[1]))
    ptB = (int(ptB[0]), int(ptB[1]))
    ptC = (int(ptC[0]), int(ptC[1]))
    ptD = (int(ptD[0]), int(ptD[1]))
    # Dessin des arêtes de l'AT
    couleur  = (0, 0, 0)
    if type_AT == "AT_absolu_mur" :
        couleur  = (0, 255, 0)
    if type_AT == "AT_relatif_mur":
        couleur  = (255, 0, 0)
    if type_AT == "AT_absolu_sol" :
        couleur  = (0, 0, 255)
    if type_AT == "AT_relatif_sol":
        couleur  = (255, 255, 0)
    cv2.line(image, ptA, ptB, couleur, 2)
    cv2.line(image, ptB, ptC, couleur, 2)
    cv2.line(image, ptC, ptD, couleur, 2)
    cv2.line(image, ptD, ptA, couleur, 2)
    # Dessin du centre de l'AT
    (cX, cY) = (int(r.center[0]), int(r.center[1]))
    cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)
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
    return(rvec,tvec)



### BOUCLE PRINCIPALE ### -------------------------------------------------------------------------------------------------------------



while run:
    _, image = cap.read() # lecture de l'image du flux vidéo 
    cv2.waitKey(30) # blocage du flux sur un nombre de frame
    cv2.imwrite("photo_AT.png",image) # enregistre l'image sur l'ordinateur

    # Detection des Apriltags sur l'image  #

    imageg = cv2.imread("//home/pc-techlab-ia-2/Documents/TOSELLO/projet_video/photo_AT.png", cv2.IMREAD_GRAYSCALE) #lecture de l'image enregistrée (renseigner le chemin du fichier)
    # initialisation du detecteur d'apriltags
    options = apriltag.DetectorOptions(families="tag36h11") #renseigner la famille d'apriltags à détecter
    detector = apriltag.Detector(options)
    results = detector.detect(imageg) #résultats de la détection

    # Dessin de l'écran de contrôle (carte + position robot) sur l'image #
    
    cv2.rectangle(image, (479,359), (639, 479), (255,255,255), thickness=-1)
    for m in murs :
        x1 = origine_repere[0] + meter_to_pixel(m[0])
        y1 = origine_repere[1] + meter_to_pixel(m[1])
        x2 = origine_repere[0] + meter_to_pixel(m[2])
        y2 = origine_repere[1] + meter_to_pixel(m[3])
        cv2.line(image, (x1,y1), (x2,y2), (0,0,255), thickness=1)


    ## Traitement de chaque résultat ##


    for r in reversed(results): # on inverse pour lire les AT absolus en priorité. Ordre de priorité :  murs--> portes--> sols
        tag_id = r.tag_id

        # Chercher les AT absolus murs et se positionner #

        if tag_id >= 100 and tag_id <= 103 and  AT_TARGETED < 3 : # Renseigner ici la plage des id correspondant à des AT_absolus_murs et le nombre de tags à traiter pour calculer la position
            #Traitement de l'AT (voir la fonction dédiée)
            rvec,tvec = traiter_AT(r,image,"AT_absolu_mur")
            # Calcul des positions des 2 pts du robot dans le repère du tag détecté
            xav,zav,xar, zar = position_relative_2pts (tvec[0][0],tvec[2][0],rvec[1][0],dim_rob[1])
            # Trouver et stocker le vecteur AT_absolu_mur qui correspond à l'Id l'inventaire des tags
            coo_AT = trouver_coo_abs (MAP_abs, tag_id)
            # Postition absolue dans le repère principal des pts avant et arrière du robot (en mètres)
            XAV,YAV,XAR,YAR = position_absolue_2pts(coo_AT, xav,zav,xar,zar)
            # Position absolue dans le repère principal des pts avant et arrière du robot (en pixels)
            XAV,YAV,XAR,YAR  = meter_to_pixel_2pts(XAV,YAV,XAR,YAR)
            # Si la position du robotest inconnue, prendre la position calculée comme point de départ
            if mat_rob [0] ==-1 :
                mat_rob = save_pos_rob(XAV,YAV,XAR,YAR) 
            # Si la position calculée n'est pas trop éloignée de la position actuelle, on stocke la position calculée
            if dist_2_pts (mat_rob[0], mat_rob[1],XAV,YAV) < 10: #ce paramètre (en pxl) est à ajuster, il permet d'éliminer les erreurs importantes dans le calcul de la position (bruit de mesure)
                mat_rob_search [AT_TARGETED] =  save_pos_rob(XAV,YAV,XAR,YAR)
                dist_tags [AT_TARGETED] = zav
                AT_TARGETED = AT_TARGETED +1
                print("AT Targeted :", coo_AT)

        # Chercher les AT relatifs murs et se positionner #

        if tag_id < 6 and AT_TARGETED < 3 and len(coo_AT) !=0: # Renseigner ici la plage des id correspondant à des AT_relatifs_murs et le nombre de tags à traiter pour calculer la position
            #Traitement de l'AT (voir la fonction dédiée)
            rvec,tvec = traiter_AT(r,image,"AT_relatif_mur")
            # Calcul des positions des 2 pts du robot dans le repère du tag détecté
            xav,zav,xar, zar = position_relative_2pts (tvec[0][0],tvec[2][0],rvec[1][0],dim_rob[1])
            # Trouver et stocker le vecteur AT_relatif_mur qui correspond à l'Id et qui minimise l'écart par rapport à la position actuelle dans l'inventaire des tags
            coo_AT = trouver_coo_rel_2pts (MAP_rel, tag_id, mat_rob, xav,zav,xar,zar)  
            # Position absolue dans le repère principal des pts avant et arrière du robot (en mètres)
            XAV,YAV,XAR,YAR = position_absolue_2pts(coo_AT, xav,zav,xar,zar)
            # Si la position calculée n'est pas trop éloignée de la position actuelle, on stocke la position calculée
            XAV,YAV,XAR,YAR  = meter_to_pixel_2pts(XAV,YAV,XAR,YAR)
            if dist_2_pts (mat_rob[0], mat_rob[1],XAV,YAV) < 10:
                mat_rob_search [AT_TARGETED] =  save_pos_rob(XAV,YAV,XAR,YAR)
                dist_tags [AT_TARGETED] = zav
                AT_TARGETED = AT_TARGETED +1
                print("AT Targeted :", coo_AT)

        # Chercher les AT absolus sols et se positionner # 

        if tag_id == 104 and  AT_TARGETED < 3 : # Renseigner ici la plage des id correspondant à des AT_absolus_sols et le nombre de tags à traiter pour calculer la position
            #Traitement de l'AT (voir la fonction dédiée)
            rvec,tvec = traiter_AT(r,image,"AT_absolu_sol")
            # Calcul des positions des 2 pts du robot dans le repère du tag détecté
            xav,zav,xar, zar = position_relative_sol_2pts (tvec[2][0],rvec[0][0],rvec[1][0],rvec[2][0],dim_rob[1])
            # Trouver et stocker le vecteur AT_absolu_sol qui correspond à l'Id l'inventaire des tags
            coo_AT = trouver_coo_abs (MAP_abs_sol, tag_id)
            # Postition absolue dans le repère principal des pts avant et arrière du robot (en mètres)
            XAV,YAV,XAR,YAR = position_absolue_2pts(coo_AT, xav,zav,xar,zar)
            # Postition absolue dans le repère principal des pts avant et arrière du robot (en pixels)
            XAV,YAV,XAR,YAR  = meter_to_pixel_2pts(XAV,YAV,XAR,YAR)
            # Si la position du robot est inconnue, prendre la position calculée comme point de départ
            if mat_rob [0] ==-1 :
                mat_rob = save_pos_rob(XAV,YAV,XAR,YAR) 
            # Si la position calculée n'est pas trop éloignée de la position actuelle, on stocke la position calculée
            if dist_2_pts (mat_rob[0], mat_rob[1],XAV,YAV) < 10:
                mat_rob_search [AT_TARGETED] =  save_pos_rob(XAV,YAV,XAR,YAR)
                dist_tags [AT_TARGETED] = zav
                AT_TARGETED = AT_TARGETED +1
                print("AT Targeted :", coo_AT)

        # Chercher les AT relatifs sol #

        if tag_id >= 40 and tag_id <= 55 and AT_TARGETED <3 : # Renseigner ici la plage des id correspondant à des AT_relatifs_sols et le nombre de tags à traiter pour calculer la position
            # Traitement de l'AT (voir la fonction dédiée)
            rvec,tvec = traiter_AT(r,image,"AT_relatif_sol")
            # Calcul des positions des 2 pts du robot dans le repère du tag détecté
            xav,zav,xar, zar = position_relative_sol_2pts (tvec[2][0],rvec[0][0],rvec[1][0],rvec[2][0],dim_rob[1])
            # Trouver et stocker le vecteur AT_relatif_sol qui correspond à l'Id et qui minimise l'écart par rapport à la position actuelle dans l'inventaire des tags
            coo_AT = trouver_coo_rel_2pts (MAP_rel_sol, tag_id, mat_rob, xav,zav,xar,zar)  
            # Postition absolue dans le repère principal des pts avant et arrière du robot (en mètres)
            XAV,YAV,XAR,YAR = position_absolue_2pts(coo_AT, xav,zav,xar,zar)
            # Position absolue dans le repère principal des pts avant et arrière du robot (en pixels)
            XAV,YAV,XAR,YAR  = meter_to_pixel_2pts(XAV,YAV,XAR,YAR)
            # Si la position calculée n'est pas trop éloignée de la position actuelle, on stocke la position calculée
            if dist_2_pts (mat_rob[0], mat_rob[1],XAV,YAV) < 10:
                mat_rob_search [AT_TARGETED] =  save_pos_rob(XAV,YAV,XAR,YAR)
                dist_tags [AT_TARGETED] = zav
                AT_TARGETED = AT_TARGETED +1
                print("AT Targeted :", coo_AT)

    ## Coix de la position la plus pertinente et affichage de la position du robot sur la carte ##


    if AT_TARGETED == 1 :
        mat_rob =  mat_rob_search[0]
        print("1 seul tag detecté")
        print("MAT ROB : ", mat_rob)
        print("--------------------------------")
    if AT_TARGETED == 2 :
        mini = min(dist_tags)
        for i in range (2) : 
            if dist_tags[i] == mini :
                mat_rob =  mat_rob_search[i]
        print("2 tags detectés : ",mat_rob_search[0], " || ", mat_rob_search[1] )
        print("MAT ROB : ", mat_rob)
        print("--------------------------------") 
    if AT_TARGETED == 3 :
        mini = min(dist_tags)
        for i in range (3) : 
            if dist_tags[i] == mini :
                mat_rob =  mat_rob_search[i]       
        print("3 tags detectés : ",mat_rob_search[0], " || ", mat_rob_search[1],  " || ", mat_rob_search[2] )
        print("MAT ROB : ", mat_rob)
        print("--------------------------------")    
    if coo_AT !=0 :
        draw_robot_2pts(mat_rob,origine_repere,hypo,delta,image)
    AT_TARGETED=0

    # Display the output image
    #cv2.imshow('AprilTag Detection', image)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    FRAME_WINDOW.image(image)

else:
    st.write('Stopped')

cv2.waitKey(0)
cv2.destroyAllWindows()



