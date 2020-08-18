import numpy as np
import quaternion
import math as m
import pandas as pd
import Dynamics as dyn
import logging

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)
# handler.setLevel(logging.DEBUG)

class vec3:

	def __init__(self, ls=[0,0,0]):
		self.x, self.y, self.z = ls[0],ls[1],ls[2];
		self.v = [self.x, self.y, self.z]
		# logger.debug("Initiated vec3 with:\t{} \n\t\t\t\tfrom\t{}".format(ls, self.v))


	def asNp(self):
		return np.array(self.v)



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

	def stateFromGyro(self, sample):
			# state = np.zeros((3,5), dtype=float)
		state = State()

		## Gyroscope
			# state[0,3] = sample[0,1]
			# state[1,3] = sample[1,1]
			# state[2,3] = sample[2,1]
		state.gyro = vec3(sample[:,1])

		## Rotate old pos about prev sensor old position
			# last_pos = self.states[self.state_loc][4]
			# p = dyn.orbit( np.asarray(last_pos), np.asarray(prev_sens_pos), self.currentQ )
		# print("SENSOR ID: ", self.id)
		last_pos = (self.states[self.state_loc]).pos
		# last_state = self.states[self.state_loc]
		# print("last state: ", last_state.asArr())
		# print("lsat state.pos", last_state.pos.asNp())
		# print("last_pos: ", last_pos.v )
		prev_sens_last_pos = (self.last_sensor.states[self.last_sensor.state_loc]).pos
		# print("prev sens id: ", self.last_sensor.id)
		# print("prev_sens_last_pos: ", prev_sens_last_pos.asNp() )
		p = dyn.orbit( last_pos.asNp(), prev_sens_last_pos.asNp(), self.currentQ )
		# print(p, end="\r", flush=True)
		# state[:,4] = p
		state.pos = vec3(p)

		return state

	def stateFromAccel(self, sample):
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
			# state[0,2] = a_nav_subg[1]
			# state[1,2] = a_nav_subg[2]
			# state[2,2] = a_nav_subg[3]
		state.a_nav = vec3(a_nav_subg[1:])
		## Gyroscope
			# state[0,3] = sample[0,1]
			# state[1,3] = sample[1,1]
			# state[2,3] = sample[2,1]
		state.gyro = vec3(sample[:,1])

		last_state = self.states[self.state_loc-1]
		## TODO make speed and position functions take vec3s
		## Velocity
		ls = [dyn.speed(last_state.vel.x, last_state.a_nav.x, dyn.w), 
				dyn.speed(last_state.vel.y, last_state.a_nav.y, dyn.w), 
				dyn.speed(last_state.vel.z, last_state.a_nav.z, dyn.w)]
		state.vel = vec3(ls)
			# vel.x = dyn.speed(self.states[i-1][1][0], self.states[i-1][2][0], dyn.w) #speed(v0 a t)
			# vel.y = dyn.speed(self.states[i-1][1][1], self.states[i-1][2][1], dyn.w)
			# vel.z = dyn.speed(self.states[i-1][1][2], self.states[i-1][2][2], dyn.w)
		# vel.x = dyn.speed(last_state.vel.x, last_state.a_nav.x, dyn.w) #speed(v0 a t)
		# vel.y = dyn.speed(last_state.vel.y, last_state.a_nav.y, dyn.w)
		# vel.z = dyn.speed(last_state.vel.z, last_state.a_nav.z, dyn.w)

		## Position
		
		#position(s0,v0,a,t)

		ls = [ dyn.position(last_state.pos.x, last_state.vel.x, last_state.a_nav.x, dyn.w),
				dyn.position(last_state.pos.y, last_state.vel.y, last_state.a_nav.y, dyn.w),
				dyn.position(last_state.pos.z, last_state.vel.z, last_state.a_nav.z, dyn.w)]
		# pos.x = dyn.position(last_state.pos.x, last_state.vel.x, last_state.a_nav.x, dyn.w)
		# pos.y = dyn.position(last_state.pos.y, last_state.vel.y, last_state.a_nav.y, dyn.w)
		# pos.z = dyn.position(last_state.pos.z, last_state.vel.z, last_state.a_nav.z, dyn.w)
		state.pos = vec3(ls);

		return state

	# Returns true if calibrating
	def checkCali(self, sample):
		# collect samples for calibration
		if self.sample_count < self.num_cal_samples:

			if(self.sample_count==0):
				logger.info("Collecting samples for calibration:")
			logger.info('\x1b[2K\r'+'{:.2%}'.format(self.sample_count/self.num_cal_samples))

			self.cal_samples[self.sample_count, 0:3] = sample[:, 0]	# accel
			self.cal_samples[self.sample_count, 3:6] = sample[:, 1]	# gyro

			self.sample_count += 1
			return True
			
		elif self.sample_count == self.num_cal_samples:
			# TODO sample returned from cal unused
			sample = self.calibrate(self.cal_samples)

			logger.info("Calibration complete.")
			logger.debug("Calibration samples: s{}:\n{}".format(self.id, self.cal_samples))
			
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
	def getState(self,sample):
		
		if(self.checkCali(sample)):
			return
			
		# state = [None]*sensor_no
		sample = np.subtract(sample, self.offset)
		
		sample[:][0] / self.g_weight
		# print("Sample: ", sample)

			# prev_sens_pos = prev_sens.states[prev_sens.state_loc-1][4]
			# state = self.getState(sample, 0, prev_sens_pos);
		
		## Make and save state
		if(self.id == 0):
			state = self.stateFromAccel(sample);
			self.currentQ = dyn.orientation(self.currentQ,sample)
		else:
			state = self.stateFromGyro(sample);
			self.currentQ = dyn.makequaternion(sample)

		logger.debug("s{} appending: {}".format(self.id, state.as1D()) )
		self.states.append(state)
		self.state_loc += 1;
		
		## Make quat
		
		self.quats.append(self.currentQ);

		# if(self.state_loc % 100 == 0):
		# 	self.printStates()

	## For non-0 sensor
	def gyroState(self, sample):

		## abg from cali
		dyn.sens_to_nav(sample[0,1],sample[1,1],sample[2,1], a, b, g)
		q = dyn.sensor_to_q(pos)
		## Rotate elbow around shoulder
		q = orbit()
		## Subtract shoulder rotate from elbow?
		## Rotate wrist about elbow?
			
	def printStates(self):
		## Convert 3d array to 2d pandas
		mat = []
		for i in range(0,len(self.states)):
			# s = [];
			# for j in range(0, len(self.states[i])):
			# 	s.extend(self.states[i][j]);
			mat.append(self.states[i].as1D());
		df = pd.DataFrame(data = mat, columns=['abx', 'aby', 'abz', 'anx', 'any', 'anz',
				'gx', 'gy', 'gz', 'vx','vy','vz', 'px','py','pz'])
		
		
		#print("\n\nStates array, sensor#", self.id);
		pd.set_option('display.max_columns', None)
		print("\ns{} states:\n".format(self.id), df);


