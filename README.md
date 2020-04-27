# Motion_Capture
Inertial Motion Capture System.
Active GitHub Users:
Marcin,
Edison,
Wyatt,
Daniel,
\n The ESP32 board is connected to two LSM9DS1 9-axis IMUs. Motion_Capture_ESP32.ino file reads the data from accelerometers and gyroscopes and sends in UDP packets over wifi connection
\n to the host. Given that the connection is established properly, the socket_clientUDP.py python code receives and unpacks the packets and displays the data in the terminal.
\n Frequency: 55Hz (18ms)   Accel Setting: 2[g] (default)   Gyroscope Setting 245 [rad/s] (default)   Port: 8090   Buffer Size: 1024   Data Type: float 
\n Format: Time, gx1, gy1, gz1, gx2, gy2, gz2, ax1, ay1, az1, ax2, ay2, az2;
