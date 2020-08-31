#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#author: XnibereiK, dr-baker

############# imports ###################
import struct
import socket
import time


import numpy as np

import Dynamics as dyn
from Sensor import Sensor
########### imports end #################


########### socket setup ################
def initSocket(ip, port):
	UDP_IP = ip
	UDP_PORT = port
	sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM) # UDP
	sock.bind((UDP_IP, UDP_PORT))
	tO = 2
	sock.settimeout(tO)
	mes= bytes("ok",'utf-8')
	
	print("Socket with {} second timeout".format(tO))

	return sock
######### socket setup end ##############
	

############ misc setup #################

def initFile():
	file = open("readings.txt", "w")
	file.write("time A1_xyz G1_xyz M1_xyx A2_xyz G2_xyz M2_xyz\n")
	return file

def initSensors():		
	s0 = Sensor(0, None)
	s1 = Sensor(1, s0)
	sensors = [s0, s1]
	return sensors

############ misc setup end #############

sigma = 0.0 # simulated socket
def getSimSample(sample):
	sensor_no = len(sample)
	for i in range(sensor_no):
		x = [np.random.normal(0, sigma), np.random.normal(0.1, sigma)]
		y = [np.random.normal(0, sigma), np.random.normal(0.1, sigma)]
		z = [np.random.normal(9.81, sigma), np.random.normal(0, sigma)]
		sample[i] = np.array([x,y,z])

def getTestSample(sample):
	sensor_no = len(sample)
	for i in range(sensor_no):
		accel_data = [(i+1)*100+11, (i+1)*100+12, (i+1)*100+13]
		gyro_data = [(i+1)*100+21, (i+1)*100+22, (i+1)*100+23]
		mag_data = [(i+1)*100+31, (i+1)*100+32, (i+1)*100+33]
		x = [accel_data[0], gyro_data[0], mag_data[0]]
		y = [accel_data[1], gyro_data[1], mag_data[1]]
		z = [accel_data[2], gyro_data[2], mag_data[2]]
		sample[i] = [x,y,z]

def getSample(sample, sock):
	sensor_no = len(sample)
	try:
		data = sock.recvfrom(1024) #buffer size is 1024 bytes
	except socket.timeout:
		print("Caught a timeout.")
		return None
	rec = struct.unpack('19f',data[0]) #19 float values {time+n(A_xyz+G_xyz+M_xyz)}

	for i in range(sensor_no):
		accel_data = rec[i*9+1:i*9+4]
		gyro_data = rec[i*9+4:i*9+7]
		mag_data = rec[i*9+7:i*9+10
		x = [accel_data[0], gyro_data[0], mag_data[0]]
		y = [accel_data[1], gyro_data[1], mag_data[1]]
		z = [accel_data[2], gyro_data[2], mag_data[2]]
		sample[i] = np.array([x,y,z])
	return rec

def closeFile(file):
	file.close()
	print('File closed.')

def closeSocket(sock):
	print('Closing socket.  ', end="")
	sock.close()
	print('Socket closed.')
