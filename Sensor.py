import numpy as np
import quaternion
import math as m
import pandas as pd
import Dynamics as dyn

# Wrapper for state array
class State:
	def __init__(self, id, last_sensor):
		self.id = id 
		# xyz * a_body a_nav gyro v p
		self.states = [[[0 for axis in range(3)] for variables in range(5)]] # tx5x3
		# self.states = [[[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0]]]
		self.currentQ = np.quaternion(0,0,0,0)
		self.quats = [self.currentQ]
		self.sample_count = 0
		self.num_cal_samples = 10
		self.state_loc = 0
		self.cal_samples = np.zeros((self.num_cal_samples,6), dtype=float)
		self.offset = np.empty((3,2), dtype=float)
		self.g_weight = 0
		self.last_sensor = last_sensor


# instance variables:
#	id, sample, quaternions, states
class Sensor:
	""" Handles computation of sensor samples """

	def __init__(self, id, last_sensor):
		self.id = id 
		# xyz * a_body a_nav gyro v p
		self.states = [[[0 for axis in range(3)] for variables in range(5)]] # tx5x3
		# self.states = [[[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0]]]
		self.currentQ = np.quaternion(0,0,0,0)
		self.quats = [self.currentQ]
		self.sample_count = 0
		self.num_cal_samples = 10
		self.state_loc = 0
		self.cal_samples = np.zeros((self.num_cal_samples,6), dtype=float)
		self.offset = np.empty((3,2), dtype=float)
		self.g_weight = 0
		self.last_sensor = last_sensor
		# self.using_a = 0


		# [aX gX aY gY aZ gZ]
		# self.cal_samples = np.zeros((num_cal_samples,6), dtype=float) made local in newstate


	# averages t cal_samples and returns single sample, sets offest instance var
	# input: nxtx6 array
	#   0   [ [aX gX aY gY aZ gZ]
	#       ...
	#   t   [aX gX aY gY aZ gZ] ]
	#   per no_sensors
	# output: 3x2 sample
	# [aX gX aY gY aZ gZ] --> [aX gX aY][gY aZ gZ]
	def calibrate(self, calSample):
		# print("cal_sample:\n", calSample)
		n = self.num_cal_samples
		sample = np.empty((3,2), dtype=float)
		sample[0][0] = sum(calSample[:,0])/n
		sample[1][0] = sum(calSample[:,1])/n
		sample[2][0] = sum(calSample[:,2])/n
		sample[0][1] = sum(calSample[:,3])/n
		sample[1][1] = sum(calSample[:,4])/n
		sample[2][1] = sum(calSample[:,5])/n
		#print(sample)

		self.currentQ = dyn.makequaternion0(sample)

		a_body = np.quaternion(0,sample[0][0],sample[1][0],sample[2][0])
		a_nav = dyn.rotatef(self.currentQ, a_body) # accel in nav frame
		v = quaternion.as_float_array(a_nav)
		u = abs(v)
		x = np.where(u == np.amax(u))
		######## TODO
		# x = np.where(v == np.amax(abs(v)))
		# print("vx0 ", v[x[0]])
		# print(x)
		v[x[0]] = v[x[0]] + dyn.g
		# if v[x[0]] < 0:
		# 	print("vx < 0")
		# 	v[x[0]] = v[x[0]] + dyn.g
		# else:
		# 	print("vx >= 0")
		# 	v[x[0]] = v[x[0]] - dyn.g
		#print(v[x[0]])
		qNav = quaternion.as_quat_array(v)
		qBody = dyn.rotateb(self.currentQ, qNav)
		#print(qBody)
		Q = quaternion.as_float_array(qBody)
	

		self.offset[:,0] = Q[1:]
		self.offset[:,1] = sample[:,1]
		#print(offset)
		return sample

	# Processes sensor data from socket and stores state information in master array
	# input: 3x2
	# output: appends 5x3 state to states
	def getState(self, sample, using_a, prev_sens_pos):
		state = np.zeros((3,5), dtype=float)

		if(using_a):
			a_body = np.quaternion(0,sample[0,0],sample[1,0],sample[2,0])
			#changed angles to positive and used rotateb to get accel in nav frame 8/10/19
			a_nav = dyn.rotateb(self.currentQ, a_body) # accel in nav frame
			a_nav_subg = dyn.subtractg(a_nav)

			# Acceleration - body
			a_body = quaternion.as_float_array(a_body)
			state[:,4] = a_body[1:] # position
			# Acceleration - nav
			a_nav_subg = quaternion.as_float_array(a_nav_subg)
			state[0,2] = a_nav_subg[1]
			state[1,2] = a_nav_subg[2]
			state[2,2] = a_nav_subg[3]
			# Gyroscope
			state[0,3] = sample[0,1]
			state[1,3] = sample[1,1]
			state[2,3] = sample[2,1]
			# Velocity
			i = self.state_loc
			state[0,1] = dyn.speed(self.states[i-1][1][0], self.states[i-1][2][0], dyn.w) #speed(v0 a t)
			state[1,1] = dyn.speed(self.states[i-1][1][1], self.states[i-1][2][1], dyn.w)
			state[2,1] = dyn.speed(self.states[i-1][1][2], self.states[i-1][2][2], dyn.w)
			# Position
			state[0,0] = dyn.position(self.states[i-1][0][0], self.states[i-1][1][0], self.states[i-1][2][0], dyn.w) #position(s0,v0,a,t)
			state[1,0] = dyn.position(self.states[i-1][0][1], self.states[i-1][1][1], self.states[i-1][2][1], dyn.w)
			state[2,0] = dyn.position(self.states[i-1][0][2], self.states[i-1][1][2], self.states[i-1][2][2], dyn.w)
			
		else: # not using accelerometer values

			# Gyroscope
			state[0,3] = sample[0,1]
			state[1,3] = sample[1,1]
			state[2,3] = sample[2,1]

			# rotate old pos about prev sensor old position
			last_pos = self.states[self.state_loc][4]
			p = dyn.orbit( np.asarray(last_pos), np.asarray(prev_sens_pos), self.currentQ )
			print(p, end="\r", flush=True)
			state[:,4] = p
			# state[0,0] = p[0]
			# state[0,1] = p[1]
			# state[0,2] = p[2]

			# Velocity
			# i = self.state_loc
			# state[0,1] = dyn.speed(self.states[i-1][1][0], self.states[i-1][2][0], dyn.w) #speed(v0 a t)
			# state[1,1] = dyn.speed(self.states[i-1][1][1], self.states[i-1][2][1], dyn.w)
			# state[2,1] = dyn.speed(self.states[i-1][1][2], self.states[i-1][2][2], dyn.w)
			# # Position
			# state[0,0] = dyn.position(self.states[i-1][0][0], self.states[i-1][1][0], self.states[i-1][2][0], dyn.w) #position(s0,v0,a,t)
			# state[1,0] = dyn.position(self.states[i-1][0][1], self.states[i-1][1][1], self.states[i-1][2][1], dyn.w)
			# state[2,0] = dyn.position(self.states[i-1][0][2], self.states[i-1][1][2], self.states[i-1][2][2], dyn.w)

		return state.T

	# Returns true if calibrating
	def checkCali(self, sample):
		# collect samples for calibration
		if self.sample_count < self.num_cal_samples:
			self.cal_samples[self.sample_count, 0:3] = sample[:, 0]	# accel
			self.cal_samples[self.sample_count, 3:6] = sample[:, 1]	# gyro

			self.sample_count += 1
			return True
			
		elif self.sample_count == self.num_cal_samples:
			# TODO sample returned fromm cal unused
			sample = self.calibrate(self.cal_samples)
			print("Calibration Samples s{}:\n".format(self.id), self.cal_samples)
			
			mag_a = m.sqrt(pow(sample[0][0],2)+pow(sample[1][0],2)+pow(sample[2][0],2))
			self.g_weight = mag_a/dyn.g
			self.sample_count += 1
			return True

		return False


	# Called by socket.  Takes sample, calculates position and velocity,
	# composes in new state and appends to state array.
	# input: 
	#   sample: 3x2 np array
	#   n: sensor id
	# output: new 3x5 state
	def newState(self,sample, prev_sens):
		
		if(self.checkCali(sample)):
			return
		
			
		# else:
		# 	# TODO does this make sense??
		# 	return
			
		# state = [None]*sensor_no
		sample = np.subtract(sample, self.offset)
		
		sample[:][0] / self.g_weight
		# print("Sample: ", sample)

		prev_sens_pos = prev_sens.states[prev_sens.state_loc-1][4]
		state = self.getState(sample, 0, prev_sens_pos);
		#saveState
		self.states.append(state.tolist())
		self.state_loc += 1;
		
		# make quat
		self.currentQ = dyn.makequaternion(sample)
		self.quats.append(self.currentQ);

		if(self.state_loc % 100 == 0):
			self.printStates()

	# for non-0 sensor
	def gyroState(self, sample):

		# abg from cali
		dyn.sens_to_nav(sample[0,1],sample[1,1],sample[2,1], a, b, g)
		q = dyn.sensor_to_q(pos)
		# rotate elbow around shoulder
		q = orbit()
		# subtract shoulder rotate from elbow?
		# rotate wrist about elbow?


	# for shoulder sens
	def newStateS0(self,sample):
		
		if(self.checkCali(sample)):
			return
			
		# state = [None]*sensor_no
		sample = np.subtract(sample, self.offset)
		
		sample[:][0] / self.g_weight
		# print("Sample: ", sample)

		# init state array
		state = self.getState(sample, 1, 0); # use old getState using accel
		#saveState
		self.states.append(state.tolist())
		self.state_loc += 1;
		
		# use old quat method for s0
		self.currentQ = dyn.orientation(self.currentQ,sample)
		self.quats.append(self.currentQ);

		if(self.state_loc % 100 == 0):
			self.printStates()

			
	def printStates(self):
		# convert 3d array to 2d pandas
		mat = []
		for i in range(0,len(self.states)):
			s = [];
			for j in range(0, len(self.states[i])):
				s.extend(self.states[i][j]);
			mat.append(s);
		df = pd.DataFrame(data = mat, columns=['px','py','pz', 'vx','vy','vz', 'ax', 'ay', 'az', 'gx', 'gy', 'gz', 'abx', 'aby', 'abz'])
		
		
		#print("\n\nStates array, sensor#", self.id);
		pd.set_option('display.max_columns', None)
		#print(df);


