import math
import cv2 as cv
import numpy as np

# Cet algorithme a pour mission de donner à l'utilisateur la positionnement et les indices des tags à fixer dans la maquette

# fonctions utiles
def meter_to_pixel (mes) : # passe une mesure flottante en m à une mesure entière en pixel
    return(int(mes * 100)) # (*100 --> 1cm = 1pixel)s robot 

def taille_ecran(mes) : # cette fonction transforme une coordonnée en m dans le repère réel de la maquette en une coordonnée en pixel dans le repère de l'écranl'écran
    mes = meter_to_pixel(mes)
    return(mes*2+20)

def dist_2_pts (xg,yg,xd,yd) : # calcul la distance entre deux points à partir de leur coordonnées dans un repère 2D
    return (math.sqrt((xd-xg)*(xd-xg) + (yd-yg)*(yd-yg)))

# initialisation des listes 
button=1
murs=[]
pieces=[]
MAP_abs = []
MAP_rel = []
MAP_abs_sol=[]
MAP_rel_sol=[]

# initialisation et référence des indices
indice = 292
indice_sol = 72
indices_murs = [0,2,4,6,8,10,12,14,16,18,20,22,1,3,5,7,9,11,13,15,17,19,21,23]
indices_sols = [['nc',48,47,49,46,50,45], [54,55,53,56,52,57,51],[41,42,40,43,39,44,38],[61,62,60,63,59,64,58],[34,35,33,36,32,37,31],[68,69,67,70,66,71,65],[27,28,26,29,25,30,24]]

## Début du programme
# Rappel des consignes et entrée des dimensions de la maquette 

print("Bienvenue sur l'application de positionnement des AprilTag sur votre maquette.")
print("Assurez vous que votre maquette remplit les conditions précisées dans le document d'utilisation des algorithmes de positionnement ")
print ("Toutes les coordonnées demandées sont à exprimer en mètre dans le repère de votre maquette")
print("Origine du repère : point le plus Nord-Ouest")
print("L'axe x pointe vers l'Est, l'axe y vers le Sud")
print("----------------------------------------------")
print("Entrez l'abcisse du point le plus Sud Est :")
xSE = float(input())
print("entrez l'ordonnee du point le plus Sud Est :")
ySE = float(input())

# Création d'une image aux dimensions de la carte
blank = np.zeros((taille_ecran(ySE)+20,taille_ecran(xSE)+20,3), dtype='uint8')
blank[:] = 255,255,255

# Entrer les coordonnées des salles, définir et dessiner les murs
while button==1 :
    print ("ajouter une salle ? o/n") 
    vote = input()
    if vote != "N" and vote != "n" : #limiter la taille à 2.50 m dans le reère de la maquette --> si plus deux pièce sans mur entre les deux
        print ("entrez les cordonnées en mètres dans le repère de la maquette maquette du coin Nord-Ouest de la salle")
        print ("xNW = ")
        xNW=float(input())
        print ("yNW = ")
        yNW=float(input())
        print ("entrez les cordonnées en mètre dans le repère de la maquette maquette du coin Sud-Est de la salle")
        print ("xSE = ")
        xSE=float(input())
        print ("ySE = ")
        ySE=float(input())
        pieces.append([xNW,yNW,xSE,ySE])
        # Dessiner la pièce
        cv.rectangle(blank, (taille_ecran(xNW),taille_ecran(yNW)), (taille_ecran(xSE), taille_ecran(ySE)), (100,255,255), thickness=-1)
        # Ajouter les murs et les dessiner
        murs.append([xNW,yNW,xSE,yNW,"N"])  
        cv.line(blank, (taille_ecran(xNW),taille_ecran(yNW)), (taille_ecran(xSE),taille_ecran(yNW)), (0,0,0), thickness=1)
        murs.append([xSE,yNW,xSE,ySE,"E"])
        cv.line(blank, (taille_ecran(xSE),taille_ecran(yNW)), (taille_ecran(xSE),taille_ecran(ySE)), (0,0,0), thickness=1)
        murs.append([xSE,ySE,xNW,ySE,"S"])
        cv.line(blank, (taille_ecran(xSE),taille_ecran(ySE)), (taille_ecran(xNW),taille_ecran(ySE)), (0,0,0), thickness=1)
        murs.append([xNW,ySE,xNW,yNW,"W"])
        cv.line(blank, (taille_ecran(xNW),taille_ecran(ySE)), (taille_ecran(xNW),taille_ecran(yNW)), (0,0,0), thickness=1)
        # prevoir d'ajouter une option dans le cas ou la disposition des AT n'est pas régulière 
    if  vote == "N" or vote == "n" :
        button = 0

# Enregistrer les positions des AT au sol et les dessiner sur le moniteur
for p in pieces : 
    xc = p[0]+(p[2]-p[0])/2
    yc = p[1]+(p[3]-p[1])/2
    MAP_abs_sol.append([indice_sol, [xc,yc]])
    cv.putText(blank, str(indice_sol), (taille_ecran(xc)-7,taille_ecran(yc)), cv.FONT_HERSHEY_TRIPLEX, 0.25, (0,255,0), 1)
    indice_sol+=1
    i=0
    j=0
    while (xc+0.285+j*0.25) <= p[2] : # on traite la ligne N=0 (i =0) qui est speciale car le centre esr un AT absolu
        MAP_rel_sol.append([indices_sols[0][j*2+1], [xc+(j+1)*0.25,yc]])
        cv.putText(blank, str(indices_sols[0][j*2+1]), (taille_ecran(xc+(j+1)*0.25)-7,taille_ecran(yc)), cv.FONT_HERSHEY_TRIPLEX, 0.25, (0,0,0), 1)
        MAP_rel_sol.append([indices_sols[0][j*2+2], [xc-(j+1)*0.25,yc]])
        cv.putText(blank, str(indices_sols[0][j*2+2]), (taille_ecran(xc-(j+1)*0.25)-7,taille_ecran(yc)), cv.FONT_HERSHEY_TRIPLEX, 0.25, (0,0,0), 1)
        j+=1
    while (yc-0.285-i*0.25) >= p[1] : # on traite toutes les autres lignes (2*i+1)€[1...6]
        MAP_rel_sol.append([indices_sols[2*i+1][0], [xc,yc-(i+1)*0.25]])
        cv.putText(blank, str(indices_sols[2*i+1][0]), (taille_ecran(xc)-7,taille_ecran(yc-(i+1)*0.25)), cv.FONT_HERSHEY_TRIPLEX, 0.25, (0,0,0), 1)
        MAP_rel_sol.append([indices_sols[2*i+2][0], [xc,yc+(i+1)*0.25]])
        cv.putText(blank, str(indices_sols[2*i+2][0]), (taille_ecran(xc)-7,taille_ecran(yc+(i+1)*0.25)), cv.FONT_HERSHEY_TRIPLEX, 0.25, (0,0,0), 1)
        j=0
        while (xc+0.285+j*0.25) <= p[2] : 
            MAP_rel_sol.append([indices_sols[2*i+1][j*2+1], [xc+(j+1)*0.25,yc-(i+1)*0.25]])
            cv.putText(blank, str(indices_sols[2*i+1][j*2+1]), (taille_ecran(xc+(j+1)*0.25)-7,taille_ecran(yc-(i+1)*0.25)), cv.FONT_HERSHEY_TRIPLEX, 0.25, (0,0,0), 1)
            MAP_rel_sol.append([indices_sols[2*i+1][j*2+1], [xc-(j+1)*0.25,yc-(i+1)*0.25]])
            cv.putText(blank, str(indices_sols[2*i+1][j*2+1]), (taille_ecran(xc-(j+1)*0.25)-7,taille_ecran(yc-(i+1)*0.25)), cv.FONT_HERSHEY_TRIPLEX, 0.25, (0,0,0), 1)
            MAP_rel_sol.append([indices_sols[2*i+2][j*2+2], [xc+(j+1)*0.25,yc+(i+1)*0.25]])
            cv.putText(blank, str(indices_sols[2*i+2][j*2+2]), (taille_ecran(xc+(j+1)*0.25)-7,taille_ecran(yc+(i+1)*0.25)), cv.FONT_HERSHEY_TRIPLEX, 0.25, (0,0,0), 1)
            MAP_rel_sol.append([indices_sols[2*i+2][j*2+2], [xc-(j+1)*0.25,yc+(i+1)*0.25]])
            cv.putText(blank, str(indices_sols[2*i+2][j*2+2]), (taille_ecran(xc-(j+1)*0.25)-7,taille_ecran(yc+(i+1)*0.25)), cv.FONT_HERSHEY_TRIPLEX, 0.25, (0,0,0), 1)
            j+=1
        i+=1
        

for m in murs  :
    xg,yg,xd,yd,orientation = m[0],m[1],m[2],m[3], m[4]
    long = dist_2_pts(xg,yg,xd,yd)
    if long < 0.26 : # couloir avec 1 seul tag 
        mil = [(xg+xd)/2 , (yg+yd)/2] 
        MAP_abs.append([indice, mil , orientation])
        if orientation == "W":
            cv.putText(blank, str(indice), (taille_ecran(mil[0]),taille_ecran(mil[1])), cv.FONT_HERSHEY_TRIPLEX, 0.25, (0,255,0), 1)
        else :  
            cv.putText(blank, str(indice), (taille_ecran(mil[0])-10,taille_ecran(mil[1])), cv.FONT_HERSHEY_TRIPLEX, 0.25, (0,255,0), 1)
        indice +=1 
    else : # pièce normale
        mil = [(xg+xd)/2 , (yg+yd)/2] 
        MAP_abs.append([indice, mil , orientation])   
        indice +=1 

        if orientation == 'N' : 
            cv.putText(blank, str(indice), (taille_ecran(mil[0])-7,taille_ecran(mil[1])+7), cv.FONT_HERSHEY_TRIPLEX,0.25, (0,255,0), 1)
            for i in range (int((long -0.26)//0.20 + 1)):  
                x = mil[0]+(i+1)*0.10
                y = mil[1]
                MAP_rel.append([indices_murs[i], x,y, orientation])
                cv.putText(blank, str(indices_murs[i]), (taille_ecran(x)-3,taille_ecran(y)+7), cv.FONT_HERSHEY_TRIPLEX,0.25, (255,0,0), 1)
                x = mil[0]-(i+1)*0.10
                MAP_rel.append([indices_murs[i+12], x,y , orientation])
                cv.putText(blank, str(indices_murs[i+12]), (taille_ecran(x)-7,taille_ecran(y)+7), cv.FONT_HERSHEY_TRIPLEX,0.25, (255,0,0), 1)
        
        if orientation == 'S' :
            cv.putText(blank, str(indice), (taille_ecran(mil[0])-9,taille_ecran(mil[1])), cv.FONT_HERSHEY_TRIPLEX,0.25, (0,255,0), 1) 
            for i in range (int((long -0.26)//0.20 + 1)):
                x = mil[0]-(i+1)*0.10
                y = mil[1]
                MAP_rel.append([indices_murs[i], x,y , orientation])
                cv.putText(blank, str(indices_murs[i]), (taille_ecran(x)-3,taille_ecran(y)), cv.FONT_HERSHEY_TRIPLEX,0.25, (255,0,0), 1)
                x = mil[0]+(i+1)*0.10
                MAP_rel.append([indices_murs[i+12],x,y , orientation])
                cv.putText(blank, str(indices_murs[i+12]), (taille_ecran(x)-3,taille_ecran(y)), cv.FONT_HERSHEY_TRIPLEX,0.25, (255,0,0), 1)

        if orientation == 'E' : 
            cv.putText(blank, str(indice), (taille_ecran(mil[0])-14,taille_ecran(mil[1])), cv.FONT_HERSHEY_TRIPLEX,0.25, (0,255,0), 1) 
            for i in range (int((long -0.26)//0.20 + 1)):
                x = mil[0]
                y = mil[1]+(i+1)*0.10
                MAP_rel.append([indices_murs[i], mil[0],mil[1]+(i+1)*0.10 , orientation])
                cv.putText(blank, str(indices_murs[i]), (taille_ecran(x)-7,taille_ecran(y)), cv.FONT_HERSHEY_TRIPLEX,0.25, (255,0,0), 1)
                y = mil[1]-(i+1)*0.10
                MAP_rel.append([indices_murs[i+12], mil[0],mil[1]-(i+1)*0.10 , orientation])
                cv.putText(blank, str(indices_murs[i+12]), (taille_ecran(x)-7,taille_ecran(y)), cv.FONT_HERSHEY_TRIPLEX,0.25, (255,0,0), 1)

        
        if orientation == 'W' : 
            cv.putText(blank, str(indice), (taille_ecran(mil[0]),taille_ecran(mil[1])), cv.FONT_HERSHEY_TRIPLEX, 0.25, (0,255,0), 1)
            for i in range (int((long -0.26)//0.20 + 1)):
                x = mil[0]
                y = mil[1]-(i+1)*0.10 
                MAP_rel.append([indices_murs[i],x,y, orientation])
                cv.putText(blank, str(indices_murs[i]), (taille_ecran(x),taille_ecran(y)), cv.FONT_HERSHEY_TRIPLEX,0.25, (255,0,0), 1)
                y = mil[1]+(i+1)*0.10
                MAP_rel.append([indices_murs[i+12],x,y, orientation])
                cv.putText(blank, str(indices_murs[i+12]), (taille_ecran(x),taille_ecran(y)), cv.FONT_HERSHEY_TRIPLEX,0.25, (255,0,0), 1)
  

print("----------------------------")
print("MAP_abs = ")
print (MAP_abs)
print("MAP_rel = ")
print(MAP_rel)
print("MAP_abs_sol = ")
print(MAP_abs_sol)
print("MAP_rel_sol = ")
print(MAP_rel_sol)



cv.imshow('MAP', blank)
cv.waitKey(0)
cv.destroyAllWindows()


# fignoler l'app de map provisoire (portes, correcions de saisie, pièces supérieure à 2.5 m)
# faire une fiche sur les repères et les projetés

# faire le boitier qui ressemblera au boitier final --> avec la nouvelle caméra
# tableau position précision par distance, angle, sol/mur, taille tag , type tag --> il faut de la data
# définir le choix optimal pour la conception d'une maquette
# modifier, documenter et rendre lisibles et fonctionnels les  4 programmes 


    