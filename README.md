# Tag_Detection_and_Pose

Introduction : 
Ce dépot explique comment calibrer une caméra, identifier un Apriltag ou ArUcO tag sur un flux vidéo et obtenir la postition du tag dans le repère de la caméra.



1/ Calibration de la caméra
Le document .pdf "Camera_calibration_tutoriel" explique la méthode à suivre pour calibrer une caméra à partir du code camera_calibration.py
Le code python photos_damier.py permet de prendre les photos du damier avec la caméra à calibrer, ces photos sont nécessaires pour la calibration de la caméra.

2/ Repérage et positionnement des April tag
Le script python test_apriltags.py permet de repérer un Apriltag dans un flux vidéo et de donner l'Id et la position de ce tag dans le repère de la caméra.
Pour que les matrices de translations et rotations  donnent la position dans l'environnement réel à la bonne échelle il est nécessaire de renseigner les matrices de la caméra (focales + point centre, distortion) obtenues lors de la calibration de la caméra.

3/ Repérage et positionnement des tags ArUcO
Le script python pose_estimation.py (dans le dossier ARUCO-Markers-Pose-Estimation-Generation-Python-main permet de repérer un ArUcO tag dans un flux vidéo et de donner l'Id et la position de ce tag dans le repère de la caméra.
Pour que les matrices de translations et rotations  donnent la position dans l'environnement réel à la bonne échelle il est nécessaire de renseigner les matrices de la caméra (focales + point centre, distortion) obtenues lors de la calibration de la caméra.


4/ Résultats des tests de comparaison Apriltag/ArUcO
