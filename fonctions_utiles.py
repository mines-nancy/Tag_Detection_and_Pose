import cv2
import numpy as np
import apriltag
import math

def meter_to_pixel (mes) :
    return(int(mes * 100)) # (*100 --> 1cm = 1pixel)s robot 

def meter_to_pixel_coo_rob (X,Y,XAG,YAG,XAD,YAD):
    X = meter_to_pixel(X)
    Y = meter_to_pixel(Y)
    XAG = meter_to_pixel(XAG)
    YAG = meter_to_pixel(YAG)
    XAD = meter_to_pixel(XAD)
    YAD = meter_to_pixel(YAD)
    return(X,Y,XAG,YAG,XAD,YAD)

def meter_to_pixel_2pts(XAV,YAV,XAR,YAR):
    XAV = meter_to_pixel(XAV)
    YAV = meter_to_pixel(YAV)
    XAR = meter_to_pixel(XAR)
    YAR = meter_to_pixel(YAR)
    return(XAV,YAV,XAR,YAR)

def position_relative (x,z,theta,hypo,delta): # robot  = triangle
    xa = z*math.tan(theta)-x 
    za = z 
    xaG = xa - hypo * math.cos(math.pi/2+theta-delta)
    zaG = za + hypo * math.sin(math.pi/2+theta-delta)
    xaD = xa - hypo * math.cos(math.pi/2+theta+delta)
    zaD = za + hypo * math.sin(math.pi/2+theta+delta)
    return(xa,za,xaG,zaG,xaD,zaD)

def position_relative2 (x,z,theta,hypo,delta): # robot  = triangle (repère corrigé)
    sigma = math.pi/2 - theta
    za = z*math.sin(sigma) 
    xa = z*math.cos(sigma)
    
    xaG = xa - hypo * math.cos(math.pi/2+theta-delta)
    zaG = za + hypo * math.sin(math.pi/2+theta-delta)
    xaD = xa - hypo * math.cos(math.pi/2+theta+delta)
    zaD = za + hypo * math.sin(math.pi/2+theta+delta)
    return(xa,za,xaG,zaG,xaD,zaD)

def position_relative_2pts (x,z,ry,long_rob,): # robot  = segment (repère corrigé)
    rho = math.atan(x/z)
    sigma = math.pi/2 - ry + rho
    za = z*math.sin(sigma) 
    xa = z*math.cos(sigma)
    
    zar = (z+long_rob)*math.sin(sigma) 
    xar = (z+long_rob)*math.cos(sigma)

    return(xa,za,xar,zar)

def position_objet(x,z,ry): # robot  = segment (repère corrigé)
    rho = math.atan(x/z)
    xb = z*math.sin(rho) 
    zb = z*math.cos(rho)
    phi = math.acos(xb/x)
    omega = ry + phi
    return(xb, zb, omega)


def position_relative_sol (z,ang_x,ang_y, ang_z,hypo,delta): # robot  = triangle (repère corrigé)
    # on projète dans le répère du sol (rotation selon l'axe x --> on utilise la matrice de rotation en 3dim dans un repère euclidien autour de l'axe x d'un angle ang_x)
    thetas  = math.cos(math.pi/2 + ang_x)*ang_y + math.sin(math.pi/2 + ang_x)*ang_z
    zs = math.cos(math.pi/2 + ang_x)*ang_y + math.sin(math.pi/2 + ang_x)*ang_z
    beta = math.pi - thetas
    ya = -z*math.cos(beta) 
    xa = z*math.sin(beta)

    xaG = xa - hypo * math.cos(math.pi/2+thetas-delta)
    yaG = ya + hypo * math.sin(math.pi/2+thetas-delta)
    xaD = xa - hypo * math.cos(math.pi/2+thetas+delta)
    yaD = ya + hypo * math.sin(math.pi/2+thetas+delta)
    return(xa,ya,xaG,yaG,xaD,yaD)

def position_relative_sol_2pts (z,ang_x,ang_y, ang_z,long_rob): # robot  = triangle (repère corrigé)
    # on projète dans le répère du sol (rotation selon l'axe x --> on utilise la matrice de rotation en 3dim dans un repère euclidien autour de l'axe x d'un angle ang_x)
    thetas  = math.cos(math.pi/2 + ang_x)*ang_y + math.sin(math.pi/2 + ang_x)*ang_z
    zs = math.cos(math.pi/2 + ang_x)*ang_y + math.sin(math.pi/2 + ang_x)*ang_z
    beta = math.pi - thetas
    ya = -z*math.cos(beta) 
    xa = z*math.sin(beta)
    
    yar = -(z+long_rob)*math.cos(beta) 
    xar = (z+long_rob)*math.sin(beta)

    return(xa,ya,xar,yar)


def position_absolue(coo_AT, xa,za,xaG,zaG,xaD,zaD): # robot  = triangle (dans le repère !!)
    if coo_AT[2]== 'N' :
        X = coo_AT[1][0] + xa
        Y = coo_AT[1][1] + za
        XAG = coo_AT[1][0] + xaG
        YAG = coo_AT[1][1] + zaG
        XAD = coo_AT[1][0] + xaD
        YAD = coo_AT[1][1] + zaD
        return (X,Y,XAG,YAG,XAD,YAD)
    if coo_AT[2]== 'S' :
        X = coo_AT[1][0] - xa
        Y = coo_AT[1][1] - za
        XAG = coo_AT[1][0] - xaG
        YAG = coo_AT[1][1] - zaG
        XAD = coo_AT[1][0] - xaD
        YAD = coo_AT[1][1] - zaD
        return (X,Y,XAG,YAG,XAD,YAD)
    if coo_AT[2]== 'E' :
        X = coo_AT[1][0] - za
        Y = coo_AT[1][1] + xa
        XAG = coo_AT[1][0] - zaG
        YAG = coo_AT[1][1] + xaG
        XAD = coo_AT[1][0] - zaD
        YAD = coo_AT[1][1] + xaD
        return (X,Y,XAG,YAG,XAD,YAD)
    if coo_AT[2]== 'W' :
        X = coo_AT[1][0] + za
        Y = coo_AT[1][1] - xa
        XAG = coo_AT[1][0] + zaG
        YAG = coo_AT[1][1] - xaG
        XAD = coo_AT[1][0] + zaD
        YAD = coo_AT[1][1] - xaD
        return (X,Y,XAG,YAG,XAD,YAD)
    
def position_absolue_2pts(coo_AT, xav,zav,xar, zar): # robot  = triangle (dans le repère !!)
    if len (coo_AT)== 2 : 
        XAV = coo_AT[1][0] + xav
        YAV = coo_AT[1][1] + zav
        XAR = coo_AT[1][0] + xar
        YAR = coo_AT[1][1] + zar
        return (XAV,YAV,XAR,YAR)
    
    if coo_AT[2]== 'N' :
        XAV = coo_AT[1][0] + xav
        YAV = coo_AT[1][1] + zav
        XAR = coo_AT[1][0] + xar
        YAR = coo_AT[1][1] + zar
        return (XAV,YAV,XAR,YAR)
    if coo_AT[2]== 'S' :
        XAV = coo_AT[1][0] - xav
        YAV = coo_AT[1][1] - zav
        XAR = coo_AT[1][0] - xar
        YAR = coo_AT[1][1] - zar
        return (XAV,YAV,XAR,YAR)
    if coo_AT[2]== 'E' :
        XAV = coo_AT[1][0] - zav
        YAV = coo_AT[1][1] + xav
        XAR = coo_AT[1][0] - zar
        YAR = coo_AT[1][1] + xar
        return (XAV,YAV,XAR,YAR)
    if coo_AT[2]== 'W' :
        XAV = coo_AT[1][0] + zav
        YAV = coo_AT[1][1] - xav
        XAR = coo_AT[1][0] + zar
        YAR = coo_AT[1][1] - xar
        return (XAV,YAV,XAR,YAR)
    
def position_absolue_sol(coo_AT, xa,ya,xaG,yaG,xaD,yaD): # robot  = triangle (dans le repère !!) 
    print (coo_AT[1][1])
    X = coo_AT[1][0] + xa
    Y = coo_AT[1][1] + ya
    XAG = coo_AT[1][0] + xaG
    YAG = coo_AT[1][1] + yaG
    XAD = coo_AT[1][0] + xaD
    YAD = coo_AT[1][1] + yaD
    return (X,Y,XAG,YAG,XAD,YAD)


def position_absolue_light(coo_AT, xa,za): # robot  = point (dans le repère !!)
    if coo_AT[2]== 'N' :
        X = coo_AT[1][0] + xa
        Y = coo_AT[1][1] + za
        return (X,Y)
    if coo_AT[2]== 'S' :
        X = coo_AT[1][0] - xa
        Y = coo_AT[1][1] - za
        return (X,Y)
    if coo_AT[2]== 'E' :
        X = coo_AT[1][0] - za
        Y = coo_AT[1][1] + xa
        return (X,Y)
    if coo_AT[2]== 'W' :
        X = coo_AT[1][0] + za
        Y = coo_AT[1][1] - xa
        return (X,Y)
    
def trouver_coo_abs (MAP_abs, code) :
    for m in MAP_abs :
        if m[0] == code :
            return(m)

def coo_to_vector (x1,y1,x2,y2) : 
    return(x1-x2,y1-y2)


def angle_entre_2_vec (X1,Y1,X2,Y2): #retourne l'angle en degré entre deux vecteurs dont les coordonnées sont exprimées dans le même repère
    angle = math.acos((X1*X2 + Y1*Y2) / (math.sqrt(X1*X1+Y1*Y1)*math.sqrt(X2*X2+Y2*Y2)))
    angle_deg = 180*angle/math.pi
    angle_2 = math.asin((X1*Y2-X2*Y1)/(math.sqrt(X1*X1+Y1*Y1)*math.sqrt(X2*X2+Y2*Y2)))
    angle_2_deg = 180*angle_2/math.pi
    if angle_deg != 0 :
        return(angle_deg*angle_2_deg/math.fabs(angle_deg))
    return(0)


def trouver_coo_rel (MAP_rel,code,mat_rob,xa,za,xaG,zaG) : # on cherche à se positionner par rapport à l'AT qui minimise le déplacement du robot
    ecart_min =10000
    #print("----------------")
    #print(mat_rob)
    for m in MAP_rel :
        if m[0] == code :
            print(m)
            print('mat_rob : ', mat_rob)
            print('position relative : ', xa,za)
            Xp,Yp = position_absolue_light(m, xa,za)
            Xp = meter_to_pixel(Xp)
            Yp = meter_to_pixel(Yp)
            print(Xp,Yp)
            dist = dist_2_pts (mat_rob[0][0],mat_rob[0][1],Xp, Yp)
            print(dist)
            XpG,YpG = position_absolue_light(m, xaG,zaG)
            XpG = meter_to_pixel(XpG)
            YpG = meter_to_pixel(YpG)
            X1,Y1 = coo_to_vector (Xp,Yp,XpG,YpG)
            ang = angle_entre_2_vec (X1,Y1,0,-100)
            print(ang)
            if (dist+ math.fabs(mat_rob[2]-ang)) < ecart_min:
                AT_ecart_min = m
                ecart_min = (dist + math.fabs(mat_rob[2]-ang))
    return(AT_ecart_min)

def trouver_coo_rel_2pts (MAP_rel,ID,mat_rob,xav,zav,xar,zar) : # on cherche à se positionner par rapport à l'AT qui minimise le déplacement du robot
    ecart_min =100000
    #print("----------------")
    #print(mat_rob)
    for m in MAP_rel :
        if m[0] == ID :
            print('mat_rob : ', mat_rob)
            print('tag testé : ', m)
            XAV,YAV,XAR,YAR = position_absolue_2pts(m, xav,zav,xar,zar)
            XAV,YAV,XAR,YAR  = meter_to_pixel_2pts(XAV,YAV,XAR,YAR)
            dist = dist_2_pts (mat_rob[0],mat_rob[1],XAV, YAV)
            print("Distance entre tête robot actuelle  et tête robot estimée : ", dist)
            X1,Y1 = coo_to_vector (XAV,YAV,XAR,YAR)
            ang = angle_entre_2_vec (X1,Y1,0,-100)
            print("Angle position estimée robot : ", ang)
            print('position relative : ', xav,zav)
            if (dist+ math.fabs(mat_rob[2]-ang)) < ecart_min:
                AT_ecart_min = m
                ecart_min = (dist + math.fabs(mat_rob[2]-ang))
    return(AT_ecart_min)


def save_pos_rob (XAV,YAV,XAR,YAR) : 
    X1, Y1 = coo_to_vector (XAV,YAV,XAR,YAR)
    ang = angle_entre_2_vec (X1,Y1,0,-100) # calcule l'angle du robot par rapport à un vecteur Nord-->Sud
    return ([XAV,YAV,ang])

        



def dist_2_pts (xg,yg,xd,yd) :
    return (math.sqrt((xd-xg)*(xd-xg) + (yd-yg)*(yd-yg)))

def draw_robot (X,Y,XAG,YAG,XAD,YAD,origine,image) :
    cv2.line(image, (origine[0]+X,origine[1] + Y), (origine[0] + XAG, origine[1] + YAG), (255,0,0), thickness=2)
    cv2.line(image, (origine[0]+X,origine[1] + Y), (origine[0] + XAD, origine[1] + YAD), (255,0,0), thickness=2)

def draw_robot_2pts (mat_rob,origine,hypo,delta,image) :
    XAV = mat_rob[0]
    YAV = mat_rob[1]
    ang = math.pi * mat_rob[2] / 180
    hypo = meter_to_pixel(hypo)
    XAG = int(XAV + hypo * math.sin(ang-delta))
    YAG = int(YAV + hypo * math.cos(ang-delta))
    XAD = int(XAV + hypo * math.sin(ang+delta))
    YAD = int(YAV + hypo * math.cos(ang+delta))
    #print(XAV,YAV,XAG,YAG,XAD,YAD)
    cv2.line(image, (origine[0]+XAV,origine[1] + YAV), (origine[0] + XAG, origine[1] + YAG), (255,0,0), thickness=2)
    cv2.line(image, (origine[0]+XAV,origine[1] + YAV), (origine[0] + XAD, origine[1] + YAD), (255,0,0), thickness=2)

def moyenne_pos (mat_rob_search_1, mat_rob_search_2) :
    a = int((mat_rob_search_1[0]+mat_rob_search_2[0])/2)
    b = int((mat_rob_search_1[1]+mat_rob_search_2[1])/2)
    c= int((mat_rob_search_1[2]+mat_rob_search_2[2])/2)
    return( a,b,c)

def placer_objet_2D (xb,zb,omega,origine,dim_objet) :
    print(math.cos(omega))
    X1 = origine[0] + xb - dim_objet [0]/2*math.cos(omega)
    Y1 = origine[1] - zb - dim_objet[0]/2*math.sin(omega)
    X2 = origine[0] + xb + dim_objet [0]/2*math.cos(omega)
    Y2 = origine[1] - zb + dim_objet[0]/2*math.sin(omega)
    Xm = origine[0] + xb + dim_objet [1]*math.sin(omega)
    Ym = origine[1] - zb - dim_objet[0]*math.cos(omega)
    X3 = Xm - dim_objet [0]/2*math.cos(omega)
    Y3 = Ym - dim_objet[0]/2*math.sin(omega)
    X4 = Xm + dim_objet [0]/2*math.cos(omega)
    Y4 = Ym + dim_objet[0]/2*math.sin(omega)

    return(X1,Y1,X2,Y2,X3,Y3,X4,Y4)