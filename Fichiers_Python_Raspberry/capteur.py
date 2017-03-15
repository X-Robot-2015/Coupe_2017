# coding: utf8

import serial
import threading
from math import atan,pi


capteur = serial.Serial("/dev/pince",115200,timeout = 1)

distance_tab=[-1,-1,-1]
class capteurDist(threading.Thread):
    def run(self):
		while True:
			replyCommand = capteur.readline()
			#on identifie à quelle commande correspond la réponse
			if replyCommand != '':
				Targ = replyCommand.decode().split(',')
				#print(Targ)
				update_capt(Targ)
          

def update_capt(t):
    global distance_tab
    captIndex= int(t[0])
    timeStamp = int(t[1])
    rangeStatus = int(t[2])
    dist = int(t[3])
    if (rangeStatus==0):
        distance_tab[captIndex]=dist
        print(distance_tab)
        
    else:
        distance_tab[captIndex]=-500
        
    recherche_tube()


def recherche_tube():
    global distance_tab
    
    dPince = 100
    dCapteurs = 35
    dCentre = 210
    
    gauche = distance_tab[2]
    centre=distance_tab[1]
    droite = distance_tab[0]
    
    if min(distance_tab)>300:
        print("pas d'objet en face")
        return -1
    if centre>gauche+40:
        print("objet à gauche")
        angle = pi/2-atan((gauche+dCentre)/dCapteurs)
        #tourner(angle)
        return 0
    if centre>droite+40:
        print("objet à droite")
        angle = -(pi/2-atan((droite+dCentre)/dCapteurs))
        return 0
  
    if droite>centre+40:
        if gauche>centre+40:
            print("objet en face") 
            avancer((centre-dPince),80)
            return 1
        else:
            print("centre gauche")
            angle = (pi/2-atan((gauche+dCentre)/dCapteurs))/2
            return 0
    else:
        if gauche>centre+40:
            angle = -(pi/2-atan((droite+dCentre)/dCapteurs))/2
            print("centre droit")
            return 0
        else:
            print("gros objet")
            return 0

capteurDist().start()