#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#author: XnibereiK
import struct
import socket
import time
import sys
from Dynamics import newState
# from Dynamics.py import toArray

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
      #size = sys.getsizeof(data)
      #print(size)
      #print(rec)
      #f2 = open("rec.txt", "w");
      #f2.write(str(rec));
      #print(type(rec))
      #print("Jizz")
      #print(str(rec).split())
      file.write(str(rec)+"\n");
      #row = [for float(f) in str(rec).split()]
      row = []
      
      #print(row)
      #print(type(row))
      c=1;
      #accel_data = row[c:c+3]
      #gyro_data = row[c+3:c+6]
      accel_data = rec[1:1+3]
      gyro_data = rec[1+3:1+6]
      x = [accel_data[0], gyro_data[0]];
      y = [accel_data[1], gyro_data[1]];
      z = [accel_data[2], gyro_data[2]];
      #toArray(accel_data, gyro_data);
      #print("Semen")
      #print(list(accel_data));
      #print(list(gyro_data));
      sample = [x,y,z];
      #print("cum")
      newState(sample);
      c+=6;
      
      
      
      #sock.sendto(mes,data[1])
sock.close()
