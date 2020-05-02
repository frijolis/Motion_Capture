#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#author: XnibereiK
import struct
import socket
import time
import sys
UDP_IP = "0.0.0.0"
UDP_PORT = 8090
sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))
mes= bytes("ok",'utf-8')

file = open("readings.txt", "w")
while True:
   data = sock.recvfrom(1024) # buffer size is 1024 bytes
   if data:
      rec = struct.unpack('13f',data[0])
      #size = sys.getsizeof(data)
      #print(size)
      print(rec)
      file.write(rec);
      #sock.sendto(mes,data[1])
sock.close()
