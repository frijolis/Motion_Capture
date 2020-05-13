#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun May 10 18:27:46 2020

@author: xnibereik
"""
import csv
import numpy as np
time = []
data = []
data_time = {}
with open(r"readings.txt") as csv_file:
    csv_reader = csv.reader(csv_file, skipinitialspace=True)
    line_count = 0

    for row in csv_reader:
        if line_count == 0:
            pass
        else:
            row[0] = row[0].replace('(','')
            row[12] = row[12].replace(')','')
            row = [float(i) for i in row]
            time.append(row[0])
            data.append(row[1:])
        line_count += 1
#print(data)
csv_file.close()
data3D = np.reshape(data,(-1,4,3))
data_time = dict(zip(time, data3D))
print(data_time)
