# Motion_Capture
Inertial Motion Capture System.
Active GitHub Users: Marcin, Edison, Wyatt, Daniel, Milo

The ESP32 board is connected to two LSM9DS1 9-axis IMUs. Motion_Capture_ESP32.ino file reads the data from accelerometers and gyroscopes and sends in UDP packets over WiFi connection
to the host. Given that the connection is established properly, the Socket.py python code receives and unpacks the packets and displays the data in the terminal, sends it to a firebase or saves it to a text file.

Max Frequency (adjustable through parser eg. Socket.py -r 40): 100Hz (10ms)
Accel Setting: 2[g] (default) at 952 [Hz]  Gyroscope Setting 245 [rad/s] (default) at 952 [Hz]
Port: 8090   Buffer Size: 1024   Data Type: float
Format: Time, gx1, gy1, gz1, gx2, gy2, gz2, ax1, ay1, az1, ax2, ay2, az2;

Functionality:
usage: Socket.py [-h] [-t] [-f] [-l] [-p] [-v] [-s] [-S] [-r R]

This program is the socket for Motion Capture system. ESP32 Dev Module acquires data from IMUs and sends information to the computer over WiFi in UDP packages. The computer receives this data by listening on the port specified by variable UDP_PORT when the Socket.py code is running.

Samples sent to the Dynamics.py program are of the following format:

	#sample from n-th sensor
	sample[n],n =
	[[An_X, Gn_X],
	 [An_Y, Gn_X],
	 [An_Z, Gn_X]]

optional arguments:
  -h, --help  show this help message and exit
  -t          makes a readings.txt file containing the master matrix
              information
  -f          sends position data and stores on the firebase server
  -l          creates a live table on firebase server
  -p          prints a master matrix containing positions, velocities and
              accelerations used for optimization (every i steps)
  -v          prints a verbose output of the program to the terminal
  -s          simulates working of the socket by feeding constant samples with
              randomly distributed noise
  -S          Disregarding socket. Sending encoded sample for testing purposes
  -r R        data feedrate in sample/sec
