import serial
ser = serial.Serial("/dev/ttyACM0",9600,timeout = 1)
import threading,time

l=[]
finished=True





def cmd(f,args):
	t=(f,args)
	l.append(t)

def avancer(t): ##case 1
	distance,speed=t
	
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
	ser.write("11"+chr(1)+chr(4)+chr(Arg0)+chr(Arg1)+chr(Arg2)+chr(Arg3))

def r(): #case2
	ser.write("11"+chr(2)+chr(0))

def tourner(angle): #case3
	angle += 32768
	if angle > 256**2 - 1:
		angle = 256**2 - 1
	if angle < 0 :
		angle = 0
	Arg1 = angle/256
	Arg0 = angle - 256*Arg1
	ser.write("11"+chr(3)+chr(2)+chr(Arg0)+chr(Arg1))

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
	ser.write("11"+chr(4)+chr(4)+chr(Arg0)+chr(Arg1)+chr(Arg2)+chr(Arg3))

def hasArrived(): #case5
	ser.write("11"+chr(5)+chr(0))	

def maxSpeed(speed): #case 131
	
	speed += 32768
	
	
	speed=min(speed,256**2-1)
	speed=max(0,speed)
	
	Arg1 = speed/256
	
	Arg0 = speed %256
	ser.write("11"+chr(131)+chr(2)+chr(Arg0)+chr(Arg1))
	
def maxAccel(accel): #case 132
	
	accel += 32768
	
	
	accel=min(accel,256**2-1)
	accel=max(0,accel)
	
	Arg1 = accel/256
	
	Arg0 = accel %256
	ser.write("11"+chr(132)+chr(2)+chr(Arg0)+chr(Arg1)) 

def readPos(): #case140
	ser.write("11"+chr(140)+chr(0))


class execution(threading.Thread):
	def run(self):
		while True:
			hasArrived()
			
			if finished and l:
				finished=false
				command=l.pop(0)
				if command[0]==1:
					avancer(command[1])
				if command[0]==2:
					r()
				if command[0]==3:
					tourner(command[1])
				if command[0]==4:
					setNewTarget(command[1])
				
				if command[0]==131:
					maxSpeed(command[1])
				if command[0]==132:
					maxAccel(command[1])
			
				
class serialRead(threading.Thread):
	def run(self):
		while True:
			time.sleep(.1)
			read=ser.read()
			if read==129:
				finished=True
			
serialRead().start()
execution().start()
