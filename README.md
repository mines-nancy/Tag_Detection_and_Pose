# Tag_Detection_and_Pose

Introduction : 

Ce travail a été réalisé dans le cadre du projet Minisacr par Baptiste Tosello.

Dans ce dépot vous trouverez des méthodes pour : 

- calibrer une caméra avec OpenCV et python
- filtrer des couleurs et segmenter un environnement avec OpenCV et python
- streamer en réseau un flux vidéo avec Streamlit
- identifier un Apriltag ou ArUcO tag sur un flux vidéo et obtenir la postition du tag dans le repère de la caméra.
- décrire un environnement avec des tags
- calculer la position de notre robot dans un environnement cartographier
- calculer la position d'éléments du l'environnement ou d'objets dans le repère du robot.
- représenter graphiquement le robot dans son environnement ou les objets dans l'environnement du robot


1/ Calibration de la caméra
Le document .pdf "Camera_calibration_tutoriel" explique la méthode à suivre pour calibrer une caméra à partir du code camera_calibration.py
Le code python "photos_damier.py" permet de prendre les photos du damier avec la caméra à calibrer, ces photos sont nécessaires pour la calibration de la caméra.

2/ Filtrage couleur et segmentation d'une image avec OpenCV
Le script "pos_objet_2D" comporte un exemple de filtrage des couleurs d'un flux vidéo et de la segmentation de l'image

3 / Streaming vidéo en réseau avec Streamlit et Python
Les scripts "pos_objet_2D" et "pose_robot_2D_MAP.py"comportent un exemple de streaminfg d'un flux vidéo à) distance avec Streamlit.

4/ Identifier un Apriltag ou un tag ArUco sur une image et obtenir les matrices de positions
Le script python "test_apriltags.py" permet de repérer un Apriltag dans un flux vidéo et de donner l'Id et la position de ce tag dans le repère de la caméra.
Pour que les matrices de translations et rotations  donnent la position dans l'environnement réel à la bonne échelle il est nécessaire de renseigner les matrices de la caméra (focales + point centre, distortion) obtenues lors de la calibration de la caméra.
Le script python "pose_estimation.py" (dans le dossier ARUCO-Markers-Pose-Estimation-Generation-Python-main) permet de repérer un ArUcO tag dans un flux vidéo et de donner l'Id et la position de ce tag dans le repère de la caméra.
Pour que les matrices de translations et rotations  donnent la position dans l'environnement réel à la bonne échelle il est nécessaire de renseigner les matrices de la caméra (focales + point centre, distortion) obtenues lors de la calibration de la caméra.

5/ Décrire un environnement avec des tags 
Le fichier "decrire_environnement.pdf" présente un exemple de description d'environnement avec des tags.  1 script qui automatise cette description "map.py"

6/ Calculer la position du robot dans un environnement décrit par des tags : 1 script qui permet d'afficher la position du robot dans son environnement et qui l'affiche en 2D "pose_robot_2D_MAP.py" + 1 script contenant les fonctions utiles pour développer ce genre d'algorithmes "fonctions_utiles.py" (nécessaire pour faire fonctionner le script précédent) + 1 document explicatif de la méthode et de ses applications "calcul_de_position.pdf"

7/ Repérer des objets avec du filtrage couleur et des tags et les positionner dans le repère du robot : 1 script qui permet de repérer un objet et de le positionner dans le repère du robot + 1 script contenant les fonctions utiles pour développer ce genre d'algorithmes "fonctions_utiles.py" (nécessaire pour faire fonctionner le script précédent) + 1 document explicatif de la méthode et de ses applications "positionnement_objet.pdf"

8/ Rapport de stage contenant toutes les avancées et réflexions sur le projet Miniscar, tous les schémas et tous les scripts nécessaire à mieux comprendre le travail réalisé. "Rapport_de_stage_Tosello.pdf"

