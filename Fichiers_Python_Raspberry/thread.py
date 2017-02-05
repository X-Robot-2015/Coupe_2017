# coding: utf8
import fonctions
import serial
import threading,time

move = serial.Serial("/dev/ttyACM3",9600,timeout = 1)
## actionneur = serial.Serial("/dev/actionneur",9600,timeout = 1)

l=[]
finished = 1
position = (0,0,0)

class execution(threading.Thread):
	def run(self):
		global finished #il faut préciser qu'on se sert de la varaible globale
		while True:
			time.sleep(1)
			if finished and l:
				finished=0
				command=l.pop(0)
				if command[0]==1: #à terme il faudra créer un tableau du type t= ["avancer","tourner"] et regarder t[command]
					avancer(command[1])
				if command[0]==2:
					r()
				if command[0]==3:
					tourner(command[1])
				if command[0]==4:
					setNewTarget(command[1])
				if command[0]==5:
					aller(command[1])
				if command[0]==6:
					getpos()


class serialRead(threading.Thread):
	def run(self):
         
         replyTest = 0
         global finished
         global position
         while True:
             
             replyTest = move.readline()
             
             replyTest = replyTest[:-2]
             
             if(len(replyTest) > 0 and replyTest == "debut"):
                 replyCommand = move.readline()
                 replyCommand = replyCommand[:-2]
                 replyArgCount = move.readline()
                 
                 if(replyArgCount[0] == 'd'):
                     break
                 replyArgCount = int(replyArgCount[:-2])# une commande peut éventuellement renvoyer plusieurs valeurs
                 Targ = []
                 for i in range(replyArgCount):
                     Targ.append(move.readline()[:-2])
                 if replyCommand == "5":
                     finished = 1
                 elif replyCommand == "6":
    				print("Valeur :"+Targ[0]+"\n")
                 elif replyCommand == 140:
                     x = 256*ord(Targ[3])+ord(Targ[2])
                     y = 256*ord(Targ[5])+ord(Targ[4])
                     angle = 256*ord(Targ[1])+ord(Targ[0])
                     position = (x,y,angle)
             

def cmd(f,args):
	t=(f,args)
	l.append(t)
	print(l)

def deplacer(t):
    x_mm = t[0]
    y_mm = t[1]
    x_clicks = x_mm * cpmm
    y_clicks = y_mm * cpmm
    deplacer_aux((x_clicks, y_clicks, angle))

def deplacer_aux(t): ##x,y en clicks, angle en millième de radians. Déplacement rectiligne entre deux points
    x,y,angle = t
    readPos()

    ##calcul de l'angle à donner en consigne
    pos_x, pos_y, pos_angle = position
    delta_x = x - pos_x
    delta_y = y - pos_y
    norme = pow((delta_x**2+delta_y**2),1/2)
    if delta_y > 0 :
        cos_cons_angle = delta_x / norme
        cons_angle = arccos(cos_cons_angle)
    else :
        cos_cons_angle = delta_x / norme
        cons_angle = -arccos(cos_cons_angle)
    delta_angle = cons_angle - pos_angle
    ##fin du calcul

    cmd(3,delta_angle) ## le robot tourne de delta_angle
    cmd(1,(norme,4000)) ## le robot avance de norme à la vitesse 4000
    cmd(3,angle-cons_angle) ## le robot se met dans l'angle donné en consigne

def aller(t): ##distance en mm, angle en °, case6.
	distance,angle = t
	distance += 32768
	angle += 32768
	distance=min(distance,256**2-1)
	distance=max(0,distance)
	angle = min(angle,256**2-1)
	angle = max(0,angle)
	Arg1 = distance/256
	Arg0 = distance%256
	Arg3 = angle/256
	Arg2 = angle%256

	move.write(chr(6)+chr(4) +chr(Arg0) +chr(Arg1) +chr(Arg2) +chr(Arg3))


def avancer(t): ##case 1
	distance,speed = t
	distance += 32768
	speed += 32768
	distance=min(distance,256**2-1)
	distance=max(0,distance)
	speed=min(speed,256**2-1)
	speed=max(0,speed)
	Arg1 = distance/256
	#Arg0 = distance - 256*Arg1
	Arg0=distance%256
	Arg3 = speed/256
	#Arg2 = speed - 256*Arg3
	Arg2 = speed %256
	move.write(chr(1)+chr(4)+chr(Arg0)+chr(Arg1)+chr(Arg2)+chr(Arg3))

def r(): #case2
	move.write(chr(2)+chr(0))

def re(): #case999
	move.write(chr(999)+chr(0))

def tourner(angle): #case3
	angle += 32768
	if angle > 256**2 - 1:
		angle = 256**2 - 1
	if angle < 0 :
		angle = 0
	Arg1 = angle/256
	Arg0 = angle - 256*Arg1
	move.write(chr(3)+chr(2)+chr(Arg0)+chr(Arg1))

def setNewTarget(t): #case4, x et y en clicks
	x,y=t
	x += 32768
	if x > 256**2 - 1:
		x = 256**2 - 1
	if x < 0 :
		x = 0
	Arg1 = x/256
	Arg0 = x - 256*Arg1
	y += 32768
	if y > 256**2 - 1:
		y = 256**2 - 1
	if y < 0 :
		y = 0
	Arg3 = y/256
	Arg2 = y - 256*Arg3
	move.write(chr(4)+chr(4)+chr(Arg0)+chr(Arg1)+chr(Arg2)+chr(Arg3))

def hasArrived(): #case5
	move.write(chr(5)+chr(0))

def readPos(): #case140
	move.write(chr(140)+chr(0))
 
def test():
	fonctions.cmd(1,(300,400))
	fonctions.cmd(3,3140)
	fonctions.cmd(1,(300,400))
	fonctions.cmd(3,-3140)
	fonctions.cmd(6,(150,45))


serialRead().start()
execution().start()
