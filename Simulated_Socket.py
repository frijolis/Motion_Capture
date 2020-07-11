#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#author: XnibereiK
'''
This code is the simulation of the samples sent to the socket.
'''
import time
import Dynamics as dyn
import numpy as np

#Standard deviation for the noise in measurements
sigma = 0.35
i = 0

while True:
	x = [np.random.normal(0, sigma), np.random.normal(0, sigma)]
	y = [np.random.normal(0, sigma), np.random.normal(0, sigma)]
	z = [np.random.normal(9.81, sigma), np.random.normal(0, sigma)]
	sample = [x,y,z]
	dyn.newState(sample)
	i += 1
	dyn.newState(sample)
	if i >= 400:
		dyn.printStates()
		i = 0
	time.sleep(0.01)
