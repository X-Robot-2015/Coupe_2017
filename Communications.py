import serial
ser = serial.Serial("/dev/ttyACM0",9600,timeout = 1)

def avancer(distance,speed): ##case 1
	distance += 32768
	speed += 32768
	if distance > 256**2-1 :
		distance = 256**2-1
	if distance < 0:
		distance = 0
	if speed > 256**2-1:
		speed = 256**2-1
	if speed < 0:
		speed = 0
	Arg1 = distance/256
	Arg0 = distance - 256*Arg1
	Arg3 = speed/256
	Arg2 = speed - 256*Arg3
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

def setNewTarget(x,y): #case4, x et y en clicks
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

def hasArrived():
	ser.write("11"+chr(5)+chr(0))	
