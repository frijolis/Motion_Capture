#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#author: XnibereiK

############# imports ###################
import argparse
import textwrap
import struct
import socket
import time
import sys
import pyrebase
import numpy as np
import pandas as pd
import Dynamics as dyn
import Firebase as fire
########### imports end #################

############## parser ###################
parser = argparse.ArgumentParser(formatter_class=argparse.RawDescriptionHelpFormatter,
description=textwrap.dedent('''This program is the socket for Motion Capture system. ESP32 Dev Module acquires data from IMUs and sends information to the computer over WiFi in UDP packages. The computer receives this data by listening on the port specified by variable UDP_PORT when the Socket.py code is running.

Samples sent to the Dynamics.py program are of the following format:
	
	#sample from n-th sensor
	sample[n],n =
	[[An_X, Gn_X],
	 [An_Y, Gn_X],
	 [An_Z, Gn_X]]
'''))
                                 
parser.add_argument('-t', action='store_true', default=False,
                    dest='text_file_enable',
                    help='makes a readings.txt file containing the master matrix information')
parser.add_argument('-f', action='store_true', default=False,
                    dest='firebase_enable',
                    help='sends position data to the firebase server')
parser.add_argument('-p', action='store_false', default=True,
                    dest='pandas_enable',
                    help='prints a master matrix containing positions, velocities and accelerations used for optimization (every i steps)')
parser.add_argument('-v', action='store_true', default=False,
                    dest='verbose_output_enable',
                    help='prints a verbose output of the program to the terminal')
results = parser.parse_args()
if results.verbose_output_enable:
	print("Verbose output enabled")
	time.sleep(1)
	if results.text_file_enable:
		print("Writing to readings.txt file enabled")
		time.sleep(1)
	if results.pandas_enable:
		print("Pandas enabled")
		time.sleep(1)
	if results.firebase_enable:
		print("Firebase enabled")
		time.sleep(1)
############ parser end #################

######### firebase setup ################
if results.firebase_enable:
	config = {
		"apiKey": "AIzaSyD1ajof65FRErV16r4b1A8JRqliPdJllJU", 
    		"authDomain": "cobey-2bbd1.firebaseapp.com",
    		"databaseURL": "https://cobey-2bbd1.firebaseio.com",
    		"storageBucket": "cobey-2bbd1.appspot.com"}
	pyrebase.pyrebase.quote = fire.noquote
	firebase = pyrebase.initialize_app(config)
	db = firebase.database()
	if results.verbose_output_enable:
		print("Firebase initialized")
		time.sleep(1)
####### firebase setup end ##############

############ misc setup #################
if results.text_file_enable:
	file = open("readings.txt", "w")
	file.write("time A1_xyz G1_xyz A2_xyz G2_xyz\n")
	if results.verbose_output_enable:
		print("File opened")
sensor_no = 2
timestep = 0
sigma = 0.35
sample = np.zeros([sensor_no,3,2],dtype = float)
if results.verbose_output_enable:
	print("Listening on port: %d" %UDP_PORT)
	time.sleep(1)
	print("Starting...")
	time.sleep(3)
start_time = time.time()
############ misc setup end #############

################# MAIN ##################
while True: #!!!should be listening for user input!!!
	if time.time()-start_time >= 0.01
		timestep += 1
		for i in range(sensor_no):
			x = [np.random.normal(0, sigma), np.random.normal(0, sigma)]
			y = [np.random.normal(0, sigma), np.random.normal(0, sigma)]
			z = [np.random.normal(9.81, sigma), np.random.normal(0, sigma)]
			sample[i] = np.array([x,y,z])
		if results.verbose_output_enable:
			print("Sample number: %d" %timestep)
			print("\n")
			print(sample)
			print("\n\n")

		if results.text_file_enable:
			file.write(str(rec)+"\n")

		if results.pandas_enable:
			for i in range(sensor_no):
				dyn.newState(sample[i],i)
			if timestep % 10 == 0:
				dyn.printStates()
				
		if results.firebase_enable:
			position = [[accel_data[0],accel_data[1],accel_data[2]],\
			[gyro_data[0], gyro_data[1], gyro_data[2]]]
			firedata = {"timestep %d" %timestep :{"time":(time.time()-start_time)/1000,\
			"sensor1":{"x":position[0][0], "y":position[0][1], "z":position[0][2]},\
			"sensor2":{"x":position[1][0], "y":position[1][1], "z":position[1][2]}}}
			fire.updateDB(db,firedata)

if results.text_file_enable:
	file.close()
