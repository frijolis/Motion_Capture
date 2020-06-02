#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#author: XnibereiK
import struct
import socket
import time
import sys
from Dynamics import newState

UDP_IP = "0.0.0.0"
UDP_PORT = 8090
sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))
mes= bytes("ok",'utf-8')

file = open("readings.txt", "w")
file.write("time A1_xyz G1_xyz A2_xyz G2_xyz\n");
while True:
   data = sock.recvfrom(1024) # buffer size is 1024 bytes
   if data:
      rec = struct.unpack('13f',data[0])
      file.write(str(rec)+"\n");
      row = []
      c=1; #list of consecutive integers corresponding to IMUs
      accel_data = rec[c:c+3]
      gyro_data = rec[c+3:c+6]
      x = [accel_data[0], gyro_data[0]];
      y = [accel_data[1], gyro_data[1]];
      z = [accel_data[2], gyro_data[2]];
      sample = [x,y,z];
      newState(sample);
      
#no fclose 
sock.close()
