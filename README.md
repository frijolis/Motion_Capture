# Motion_Capture
Inertial Motion Capture System.
Active GitHub Users: Marcin, Edison, Wyatt, Daniel, Milo
The ESP32 board is connected to two LSM9DS1 9-axis IMUs. Motion_Capture_ESP32.ino file reads the data from accelerometers and gyroscopes and sends in UDP packets over wifi connection
to the host. Given that the connection is established properly, the socket_clientUDP.py python code receives and unpacks the packets and displays the data in the terminal.

Frequency: 100Hz (10ms)
Accel Setting: 2[g] (default) at 952 [Hz]  Gyroscope Setting 245 [rad/s] (default) at 952 [Hz]
Port: 8090   Buffer Size: 1024   Data Type: float
Format: Time, gx1, gy1, gz1, gx2, gy2, gz2, ax1, ay1, az1, ax2, ay2, az2;
