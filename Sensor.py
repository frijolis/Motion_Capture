import numpy as np
import quaternion
import math as m
import pandas as pd
import Dynamics as dyn
from Dynamics import vec3 as vec3
import logging
import os # For clearning terminal

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)
# handler.setLevel(logging.DEBUG)

ONLY_ACCEL = True
ONLY_GYRO = False


# Wrapper for state array
class State:

	def __init__(self):
		# https://science.ksc.nasa.gov/facts/acronyms.html
		self.a_body = vec3()
		self.a_nav = vec3()
		self.gyro = vec3()
		self.vel = vec3()
		self.pos = vec3()
		# self._state = [self.a_body, self.a_nav, self.gyro, self.vel, self.pos]

	def getStateArray(self):
		return [self.a_body, self.a_nav, self.gyro, self.vel, self.pos]

	def asNp(self):
		state = self.getStateArray()
		return np.array( [vec.asNp() for vec in state] )
		
	def  as1D(self):
		mat = []
		state = self.getStateArray()
		for vec in state:
			mat.append(vec.x); mat.append(vec.y); mat.append(vec.z)
		return mat


# instance variables:
#	id, sample, quaternions, states
class Sensor:
	""" Handles computation of sensor samples """

	def __init__(self, id, last_sensor):
		logger.info("Initiating s{}".format(id))
		self.id = id 
		# xyz * a_body a_nav gyro v p
		self.states = [State()] # tx5x3 
		self.currentQ = dyn.normalizeQ( np.quaternion(1,0,0,0) ) # Holds sensor orientation
		self.deltaQ = dyn.normalizeQ( np.quaternion(1,0,0,0) ) # Holds last rotation
		self.offsetQ = dyn.normalizeQ( np.quaternion(1,0,0,0) ) # Holds initial rotation from body to nav frame
		self.quats = [self.currentQ]
		self.sample_count = 0
		self.num_cal_samples = 100
		self.state_loc = 0
		self.cal_samples = np.zeros((self.num_cal_samples,6), dtype=float)
		self.offset = np.empty((3,2), dtype=float)
		self.g_weight = 0
		self.last_sensor = last_sensor
		self.limb_len = 1
		# self.using_a = 0


		# [aX gX aY gY aZ gZ]
		# self.cal_samples = np.zeros((num_cal_samples,6), dtype=float) made local in newstate


	# Returns true if calibrating
	def checkCali(self, sample):
		# collect samples for calibration
		if self.sample_count < self.num_cal_samples:

			# if(self.sample_count==0):
			# 	logger.info("Collecting samples for calibration:")
			# logger.info('\x1b[2K\r'+'\t{:.2%}'.format(self.sample_count/self.num_cal_samples))

			logger.info("Collecting samples for calibration s{}: \033[K{:.2%}".format(self.id, self.sample_count/self.num_cal_samples))

			self.cal_samples[self.sample_count, 0:3] = sample[:, 0]	# accel
			self.cal_samples[self.sample_count, 3:6] = sample[:, 1]	# gyro
			self.cal_samples[self.sample_count, 6:9] = sample[:, 2]	# mag

			self.sample_count += 1
			return True
			
		elif self.sample_count == self.num_cal_samples:
			# TODO sample returned from cal unused
			sample = self.calibrate(self.cal_samples)

			logger.info("s{} Calibration complete.".format(self.id) )
			logger.debug("Calibration samples: s{}:\n{}".format(self.id, self.cal_samples))
			
			mag_a = m.sqrt(pow(sample[0][0],2)+pow(sample[1][0],2)+pow(sample[2][0],2))
			self.g_weight = mag_a/dyn.g
			self.sample_count += 1
			return True

		return False

	"""
	# averages t cal_samples and returns single sample, sets offest instance var
	# input: nxtx9 array
	#   0   [ [aX aY aZ gX gY gZ mY mX mZ]
	#       ...
	#   t   [aX aY aZ gX gY gZ mY mX mZ] ]
	#   per no_sensors
	# output: 3x3 sample
	# [aX aY aZ gX gY gZ mY mX mZ] --> [[aX aY aZ][gX gY gZ][mX mY mZ]]
	"""
	def calibrate(self, calSample):

		# print("cal_sample:\n", calSample)
		n = self.num_cal_samples
		sample = np.empty((3,3), dtype=float)
		sample[0][0] = sum(calSample[:,0])/n
		sample[1][0] = sum(calSample[:,1])/n
		sample[2][0] = sum(calSample[:,2])/n
		sample[0][1] = sum(calSample[:,3])/n
		sample[1][1] = sum(calSample[:,4])/n
		sample[2][1] = sum(calSample[:,5])/n
		sample[0][2] = sum(calSample[:,6])/n
		sample[1][2] = sum(calSample[:,7])/n
		sample[2][2] = sum(calSample[:,8])/n
		#print(sample)

		## Quaternions
		self.currentQ = dyn.makequaternion0(sample)
		self.offsetQ = self.currentQ

		a_body = np.quaternion(0,sample[0][0],sample[1][0],sample[2][0])
		a_nav = dyn.rotatef(self.currentQ, a_body) # accel in nav frame
		v = quaternion.as_float_array(a_nav)
		u = abs(v)
		x = np.where(u == np.amax(u))
		######## TODO smart grav subtraction
		# x = np.where(v == np.amax(abs(v)))
		v[x[0]] = v[x[0]] + dyn.g

		qNav = quaternion.as_quat_array(v)
		qBody = dyn.rotateb(self.currentQ, qNav)
		#print(qBody)
		Q = quaternion.as_float_array(qBody)
	

		self.offset[:,0] = Q[1:]
		self.offset[:,1] = sample[:,1]
		#print(offset)
		return sample

	def stateFromGyro(self, sample):
		logger.debug("Generating state from gyroscope.")

			# state = np.zeros((3,5), dtype=float)
		state = State()

		## Update quaternions
		state.gyro = vec3(sample[:,1])
		# if(self.id == 1):
		# 	print(state.gyro.v, end="\r", flush=True)

		# self.deltaQ = dyn.deltaQ( state.gyro ) if dyn.deltaQ( state.gyro ) != None else self.deltaQ
		if dyn.deltaQ( state.gyro ) is None:
			# logger.info("DeltaQ not changing")
			self.deltaQ = self.deltaQ
		else:
			self.deltaQ = dyn.deltaQ( state.gyro ) 

		self.currentQ = self.currentQ*self.deltaQ

		# if(self.id == 1):
		# 	print(self.currentQ, end="\r", flush=True)

		## Project to nav frame
		nav_i = dyn.rotatef(self.currentQ, dyn.iq)
		nav_j = dyn.rotatef(self.currentQ, dyn.jq)
		nav_k = dyn.rotatef(self.currentQ, dyn.kq)
		## Rotate sensor inline with limb
		limb_i = dyn.rotateb(self.offsetQ, nav_i)
		limb_j = dyn.rotateb(self.offsetQ, nav_j)
		limb_k = dyn.rotateb(self.offsetQ, nav_k)
		## Scale by limb length
		limb_pos = (quaternion.as_float_array(limb_k)*self.limb_len)[1:]

		# if(self.id == 1):
		# 	print(limb_pos, end="\r", flush=True)

		## Add last sensor position
		if self.last_sensor == None:
			logger.debug("Trying to access last sensor but DNE")
			prev_sens_last_pos = vec3()
		else:
			psens = self.last_sensor
			prev_sens_last_pos = (psens.states[psens.state_loc]).pos
		# if self.state_loc == 0 or self.last_sensor.state_loc == 0:
		# 	logger.warn("Trying to access previous state but DNE")

		logger.debug("prev_sens_last_pos: {}".format(prev_sens_last_pos.asNp()) )
		limb_pos = limb_pos + prev_sens_last_pos.asNp()


		state.pos = vec3(limb_pos)
		# if(self.id == 1):
		# 	print(state.pos.v, end="\r", flush=True)

		return state

	## For non-0 sensor
	def gyroState(self, sample):

		## abg from cali
		dyn.sens_to_nav(sample[0,1],sample[1,1],sample[2,1], a, b, g)
		q = dyn.sensor_to_q(pos)
		## Rotate elbow around shoulder
		q = orbit()
		## Subtract shoulder rotate from elbow?
		## Rotate wrist about elbow?

	def stateFromAccel(self, sample):
		logger.debug("Generating state from acceleration.")

		# state = np.zeros((3,5), dtype=float)
		state = State()
		
		

		a_body = np.quaternion(0,sample[0,0],sample[1,0],sample[2,0])
		## Changed angles to positive and used rotateb to get accel in nav frame 8/10/19
		a_nav = dyn.rotateb(self.currentQ, a_body) # accel in nav frame
		a_nav_subg = dyn.subtractg(a_nav)

		## Acceleration - body
		a_body = quaternion.as_float_array(a_body)
			# state[:,4] = a_body[1:] # position
		state.a_body = vec3(a_body[1:])
		## Acceleration - nav
		a_nav_subg = quaternion.as_float_array(a_nav_subg)
		state.a_nav = vec3(a_nav_subg[1:])
		## Gyroscope
		state.gyro = vec3(sample[:,1])
		
		//////////////////////////////////////////////
		if dyn.deltaQ( state.gyro ) is None:
			# logger.info("DeltaQ not changing")
			self.deltaQ = self.deltaQ
		else:
			self.deltaQ = dyn.deltaQ( state.gyro ) 

		self.currentQ = self.currentQ*self.deltaQ
		///////////////////////////////////////////////


		## TODO make speed and position functions take vec3s
		last_state = self.states[self.state_loc-1]
		## Velocity
		ls = [dyn.speed(last_state.vel.x, last_state.a_nav.x, dyn.dt), 
				dyn.speed(last_state.vel.y, last_state.a_nav.y, dyn.dt), 
				dyn.speed(last_state.vel.z, last_state.a_nav.z, dyn.dt)]
		state.vel = vec3(ls)
		## Position
		ls = [ dyn.position(last_state.pos.x, last_state.vel.x, last_state.a_nav.x, dyn.dt),
				dyn.position(last_state.pos.y, last_state.vel.y, last_state.a_nav.y, dyn.dt),
				dyn.position(last_state.pos.z, last_state.vel.z, last_state.a_nav.z, dyn.dt)]		
		state.pos = vec3(ls);
		
		

		return state


	# Called by socket.  Takes sample, calculates position and velocity,
	# composes in new state and appends to state array.
	# input: 
	#   sample: 3x2 np array
	#   n: sensor id
	# output: new 3x5 state
	def getState(self,sample):
		
		if(self.checkCali(sample)):
			return
			
		# state = [None]*sensor_no
		sample = np.subtract(sample, self.offset)
		
		sample[:][0] / self.g_weight
		# print("Sample: ", sample)

		## Make and save state
		if ONLY_GYRO:
			state = self.stateFromGyro(sample);
		if(self.id == 0 or ONLY_ACCEL): # Root sensor uses accel calculations
			state = self.stateFromAccel(sample);
			#self.currentQ = dyn.orientation(self.currentQ,sample)
		else:
			state = self.stateFromGyro(sample);


		# logger.debug("s{} appending: {}".format(self.id, state.as1D()) )
		self.states.append(state)
		self.state_loc += 1;
		
		## Save quat
		self.quats.append(self.currentQ);

		# if(self.state_loc % 100 == 0):
		# 	self.printStates()

			
	def printStates(self):
		## Convert 3d array to 2d pandas
		mat = []
		for i in range(0,len(self.states)):
			mat.append(self.states[i].as1D());
		df = pd.DataFrame(data = mat, columns=['abx', 'aby', 'abz', 'anx', 'any', 'anz',
				'gx', 'gy', 'gz', 'vx','vy','vz', 'px','py','pz'])
		
		
		#print("\n\nStates array, sensor#", self.id);
		pd.set_option('display.max_columns', None)
		print("\ns{} states:\n".format(self.id), df);

	def printPos(self):
		## Clear last 3 lines
		# print('\b'+'\x1b[2K\r'+'\b'+'\x1b[2K\r'+'\b'+'\x1b[2K\r', end='')

		pos = self.states[self.state_loc].a_nav
		print("s{}: \t{:10.2f}\t{:10.2f}\t{:10.2f}".format(self.id, pos.x, pos.y, pos.z), end="\r")
