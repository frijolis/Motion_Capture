import time
import argparse
import textwrap # for parser description

import Firebase as fire
import numpy as np

import Dynamics as dyn
import Firebase as fire
from Sensor import Sensor
import Socket

UDP_IP = "0.0.0.0"
UDP_PORT = 8090

start_time = time.time()
timestep_duration = 1/results.r
timestep = 0

def parseCLA():
	parser = argparse.ArgumentParser(formatter_class=argparse.RawDescriptionHelpFormatter,
	description=textwrap.dedent('''This program is the socket for Motion Capture system. ESP32 Dev Module acquires data from IMUs and sends information to the computer over WiFi in UDP packages. The computer receives this data by listening on the port specified by variable UDP_PORT when the Socket.py code is running.

	Samples sent to the Dynamics.py program are of the following format:
		
		#sample from n-th sensor
		sample[n],n =
		[[An_X, Gn_X, Mn_X],
		 [An_Y, Gn_Y, Mn_Y],
		 [An_Z, Gn_Z, Mn_Z]]
	'''))
	                                 
	parser.add_argument('-t', action='store_true', default=False,
	                    dest='text_file_enable',
	                    help='makes a readings.txt file containing the master matrix information')
	parser.add_argument('-f', action='store_true', default=False,
	                    dest='firebase_enable',
	                    help='sends position data and stores on the firebase server')
	parser.add_argument('-l', action='store_true', default=False,
	                    dest='firebase_live_enable',
	                    help='creates a live table on firebase server')
	parser.add_argument('-p', action='store_false', default=True,
	                    dest='pandas_enable',
	                    help='prints a master matrix containing positions, velocities and accelerations used for optimization (every i steps)')
	parser.add_argument('-v', action='store_true', default=False,
	                    dest='verbose_output_enable',
	                    help='prints a verbose output of the program to the terminal')
	parser.add_argument('-s', action='store_true', default=False,
	                    dest='simulated_socket',
	                    help='simulates working of the socket by feeding constant samples with randomly distributed noise')
	parser.add_argument('-S', action='store_true', default=False,
	                    dest='testing_sample',
	                    help='Disregarding socket. Sending encoded sample for testing purposes' )
	parser.add_argument('-r', type=int, default=100, help='data feedrate in sample/sec')
	parser.add_argument('-m', action='store_true', default=False, help='Only print position', dest='pos_print')
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
		if results.firebase_live_enable:
			print("Firebase live table enabled")
			time.sleep(1)
		if results.testing_sample:
			print("Disregarding socket. Sending encoded sample as: 'nnta'")
			print("\tnn - 2 digit sensor Number (1 innedxed)\n\tt - 1 digit sensor Type\t(1:accel, 2:gyro)\n\ta - one digit sensor Axis\t(1:x, 2:y, 3:z)")
			time.sleep(1)	
		if results.pos_print:
			print("Disregarding socket. Sending encoded sample as: 'nnta'")
			print("\tnn - 2 digit sensor Number (1 innedxed)\n\tt - 1 digit sensor Type\t(1:accel, 2:gyro)\n\ta - one digit sensor Axis\t(1:x, 2:y, 3:z)")
			time.sleep(1)	
	return results


############## General inits ###################
results = parseCLA()
sensors = Socket.initSensors()
sensor_no = len(sensors)
sock = Socket.initSocket(UDP_IP, UDP_PORT)

############## Optional inits ###################
if results.firebase_enable or results.firebase_live_enable:
	fire.initFB()
	if results.verbose_output_enable:
		print("Firebase initialized")
		time.sleep(1)

if results.text_file_enable:
	Socket.initFile()
	if results.verbose_output_enable:
		print("File opened")

if results.verbose_output_enable:
	print("Listening on port: %d" %UDP_PORT)
	time.sleep(1)
	print("Feedrate (Hz): %d" %results.r)
	time.sleep(1)
	print("Starting...")
	time.sleep(3)

	## TODO move elsewhere
	if results.pos_print:
		print("Pandas disabled.  Only printing position.")
		results.pandas_enable = False


############## Main loop ###################

try: ## Run until KeyboardInterrupt
	while True: 

		sample = np.zeros([sensor_no,3,3],dtype = float)


		if time.time()-start_time >= timestep*timestep_duration:
			timestep += 1

			########## Get sample ##########
			if results.simulated_socket:
				Socket.getSimSample(sample)
				if results.verbose_output_enable:
					for i in range(sensor_no):
						print("Simulated Sample s{} #{}: \n".format(i, timestep), sample)
			
			elif results.testing_sample:
				Socket.getTestSample(sample)
				if results.verbose_output_enable and timestep==1:
					for i in range(sensor_no):
						print("Encoded Sample s{} #{}: \n".format(i, timestep), sample)
			
			else:
				rec = Socket.getSample(sample, sock)

				if rec==None:
					continue
				
				if results.text_file_enable:
					file.write(str(rec)+"\n")
				

				if results.verbose_output_enable:
					if timestep%50 == 0:
						print("Sample number %d" %timestep)
						print("\n")			
						for i in range(1,sensor_no+1):
							
							print("sensor %d" %i)
							print(sample[i-1])
							print("\n")
			########## Sample populated ##########
			

			## Get state
			for i in range(sensor_no):
				sensors[i].getState(sample[i])

			## Print
			if results.pandas_enable:
				for i in range(sensor_no):
					if timestep % 200 == 0:
						sensors[i].printStates()
			if results.pos_print:
				sensors[i].printPos()
			
			## Update database		
			if results.firebase_live_enable:
				position = sample[0]
				firedata = {"time":(time.time()-start_time),\
				"sensor1":{"x":position[0,0], "y":position[1,0], "z":position[2,0]},\
				"sensor2":{"x":position[0,1], "y":position[1,1], "z":position[2,1]}}
				fire.liveDB(db,firedata)	
					
			elif results.firebase_enable:
				position = sample[0]
				firedata = {"timestep %d" %timestep :{"time":(time.time()-start_time),\
				"sensor1":{"x":position[0,0], "y":position[1,0], "z":position[2,0]},\
				"sensor2":{"x":position[0,1], "y":position[1,1], "z":position[2,1]}}}
				fire.updateDB(db,firedata)

## End loop
except KeyboardInterrupt:
	print('\n\nRecieved Keyboard Interrupt')

	if results.text_file_enable:
		Socket.closeFile(file)
		print('Closed file.')

	Socket.closeSocket(sock)
















