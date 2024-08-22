# Tag_Detection_and_Pose

Introduction : 

Ce travail a été réalisé dans le cadre du projet Minisacr par Baptiste Tosello.

Ce dépot explique comment calibrer une caméra, identifier un Apriltag ou ArUcO tag sur un flux vidéo et obtenir la postition du tag dans le repère de la caméra.
Ensuite on peut utiliser ces tags pour décrire un environnement, calculer la position de notre robot dans cet environnement ou détecter des objets et les positionner dans le repère du robot.
On introduit également une méthode de filtrage par couleur d'un flux vidéo ainsi qu'une méthode de Streaming à distance d'un flux vidéo avec Streamlit.


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

5/ Définir un environnement avec des tags : 1 exemple dans le fichier "descrire_environnement.pdf" + 1 script qui automatise cette description "map.py"

6/ Calculer la position du robot dans un environnement décrit par des tags : 1 script qui permet d'afficher la position du robot dans son environnement et qui l'affiche en 2D "pose_robot_2D_MAP.py" + 1 script contenant les fonctions utiles pour développer ce genre d'algorithmes "fonctions_utiles.py" (nécessaire pour faire fonctionner le script précédent) + 1 document explicatif de la méthode et de ses applications "calcul_de_position.pdf"

7/ Repérer des objets avec du filtrage couleur et des tags et les positionner dans le repère du robot : 1 script qui permet de repérer un objet et de le positionner dans le repère du robot + 1 script contenant les fonctions utiles pour développer ce genre d'algorithmes "fonctions_utiles.py" (nécessaire pour faire fonctionner le script précédent) + 1 document explicatif de la méthode et de ses applications "positionnement_objet.pdf"

8/ Rapport de stage contenant toutes les avancées et réflexions sur le projet Miniscar, tous les schémas et tous les scripts nécessaire à mieux comprendre le travail réalisé. "Rapport_de_stage_Tosello.pdf"

