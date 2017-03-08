# -*- coding: utf-8 -*-

import fonctions
import serial
import threading,time

pince = serial.Serial("/dev/pince",9600,timeout = 1)

def attraper():
    pince.write(chr(0)+chr(0))

def reposer():
    pince.write(chr(1)+chr(0))