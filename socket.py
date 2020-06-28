#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#author: XnibereiK
import struct
import socket
import time
import sys
import pyrebase
import pandas as pd
import Dynamics as dyn
import Firebase as fire

UDP_IP = "0.0.0.0"
UDP_PORT = 8090
sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))
mes= bytes("ok",'utf-8')
A = pd.DataFrame([], columns=['Time', 'AX1', 'AY1', 'AZ1', 'GX1', 'GY1', 'GZ1'])
timestep = 0
i = 0

#Firebase setup
config = { #This is for Firebase
    "apiKey": "AIzaSyD1ajof65FRErV16r4b1A8JRqliPdJllJU", 
    "authDomain": "cobey-2bbd1.firebaseapp.com",
    "databaseURL": "https://cobey-2bbd1.firebaseio.com",
    "storageBucket": "cobey-2bbd1.appspot.com"}
pyrebase.pyrebase.quote = fire.noquote
firebase = pyrebase.initialize_app(config)
db = firebase.database()

#file = open("readings.txt", "w")
#file.write("time A1_xyz G1_xyz A2_xyz G2_xyz\n")
while i < 400: #should be listening for user input
   data = sock.recvfrom(1024) # buffer size is 1024 bytes

   if data:
      print('.', end ='')
      rec = struct.unpack('13f',data[0])
      
      c=1 #list of consecutive integers corresponding to IMU IDs
      accel_data = rec[c:c+3]
      gyro_data = rec[c+3:c+6]
      x = [accel_data[0], gyro_data[0]]
      y = [accel_data[1], gyro_data[1]]
      z = [accel_data[2], gyro_data[2]]
      sample = [x,y,z]
    
#####Pandas test#####
'''
      A = dyn.Panda_Matrix(rec[0],sample,A)
'''
#####Pandas test end#####
        
#####Firebase test#####
'''
      position = [accel_data[0],accel_data[1],accel_data[2]]
      print(position)
      firedata = {"timestep%d" %timestep :{"time":rec[0], "sensor1":{"x":position[0], "y":position[1], "z":position[2]}}}
      timestep += 1
      fire.updateDB(db,firedata)
'''
#####Firebase test end#####

#####Proper Pandas test#####
'''
      i += 1
      dyn.newState(sample)
dyn.printStates()
'''
#####Proper Pandas test end#####
sock.close()
