# coding: utf8

import serial
import threading,time

move = serial.Serial("/dev/moteur",9600,timeout = 1)
actionneur = serial.Serial("/dev/actionneur",9600,timeout = 1)

l=[]
finished = 1
position = (0,0,0)

class execution(threading.Thread):
	def run(self):
		global finished #il faut préciser qu'on se sert de la varaible globale
		while True:
			time.sleep(3)
			hasArrived()
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
		global finished
		global position
		while True:
			# time.sleep(.1) pas nécessaire car ser.read() attend 1 une seconde si il n'y a rien à lire
			replyCommand = move.read()  #on identifie à quelle commande correspond la réponse
			if replyCommand != '':
				replyArgCount = ord(move.read())  # une commande peut éventuellement renvoyer plusieurs valeurs
				Targ = []
				for i in range(replyArgCount):
					Targ.append(ord(move.read()))
				if ord(replyCommand) == 5:
					finished = Targ[0];
				elif ord(replyCommand) == 6:
					print("Valeur :"+string(256*ord(Targ[0])+ord(Targ[1]))+"\n")
				elif ord(replyCommand) == 140:
					x = 256*ord(Targ[3])+ord(Targ[2])
					y = 256*ord(Targ[5])+ord(Targ[4])
					angle = 256*ord(Targ[1])+ord(Targ[0])
					position = (x,y,angle)

def test():
	cmd(1,(300,400))
	cmd(3,3140)
	cmd(1,(300,400))
	cmd(3,-3140)
	cmd(6,(150,45))


serialRead().start()
execution().start()
