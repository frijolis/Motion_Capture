//#include <WiFi.h>
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>

//const char* ssid = "MySpectrumWiFieb-2G";
//const char* password =  "outletwalnut604";
///const uint16_t port = 8090;
//const char * host = "192.168.1.232";

//Function definitions
void printGyro();
void printAccel();
void printMag();
void printAttitude(float ax, float ay, float az, float mx, float my, float mz);

//Adding definitions for new sensor on I2C
#define SDA_1 27
#define SCL_1 26
#define SDA_2 33
#define SCL_2 32

LSM9DS1 imu1;
LSM9DS1 imu2;

#define SENSITIVITY_ACCELEROMETER_2  0.000061
#define SENSITIVITY_ACCELEROMETER_4  0.000122
#define SENSITIVITY_ACCELEROMETER_8  0.000244
#define SENSITIVITY_ACCELEROMETER_16 0.000732
#define SENSITIVITY_GYROSCOPE_245    0.00875
#define SENSITIVITY_GYROSCOPE_500    0.0175
#define SENSITIVITY_GYROSCOPE_2000   0.07
//#define PRINT_CALCULATED
#define PRINT_RAW
#define PRINT_SPEED 250 // 250 ms between prints
static unsigned long lastPrint = 0; // Keep track of print time

// Earth's magnetic field varies by location. Add or subtract
// a declination to get a more accurate heading. Calculate
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION -8.58 // Declination (degrees) in Boulder, CO.
 
void setup()
{
 
  Serial.begin(115200);
  //WiFi.begin(ssid, password);
  //while (WiFi.status() != WL_CONNECTED) {
    //delay(500);
    //Serial.println("...");
  
  //Serial.print("WiFi connected with IP: ");
  //Serial.println(WiFi.localIP());
  Wire.begin(SDA_1, SCL_1, 100000); 
  Wire1.begin(SDA_2, SCL_2, 100000);
}
 
void loop()
{
    //WiFiClient client;
 
    //if (!client.connect(host, port)) {
 
      //  Serial.println("Connection to host failed");
 
        //delay(1000);
        //return;
    //}
 
    //Serial.println("Connected to server successful!");
///////////////CHANNEL OPEN////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      if (imu1.begin() == false || imu2.begin(0x6B, 0x1E, Wire1) == false) // with no arguments, this uses default addresses (AG:0x6B, M:0x1E) and i2c port (Wire).
        {
        Serial.println("Failed to communicate with LSM9DS1.");
        while (1);
        }
    //client.print("Hello from ESP32!");
    Serial.print("Time, gx1, gy1, gz1, gx2, gy2, gz2, ax1, ay1, az1, ax2, ay2, az2"); 
    // Update the sensor values whenever new data is available
    while(1){
      if ( imu1.gyroAvailable() || imu2.gyroAvailable())
      {
      // To read from the gyroscope,  first call the
      // readGyro() function. When it exits, it'll update the
      // gx, gy, and gz variables with the most current data.
      imu1.readGyro();
      imu2.readGyro();
      }
      if ( imu1.accelAvailable() || imu2.accelAvailable() )
      {
      // To read from the accelerometer, first call the
      // readAccel() function. When it exits, it'll update the
      // ax, ay, and az variables with the most current data.
      imu1.readAccel();
      imu2.readAccel();
      }
      if ( imu1.magAvailable() )
      {
      // To read from the magnetometer, first call the
      // readMag() function. When it exits, it'll update the
      // mx, my, and mz variables with the most current data.
      imu1.readMag();
      }

      if ((lastPrint) < millis())
      {
/////////PRINT GYRO////////////////
  Serial.println();
  Serial.print(millis());
  Serial.print(", ");
  Serial.print(imu1.gx * SENSITIVITY_GYROSCOPE_245);
  Serial.print(", ");
  Serial.print(imu1.gy * SENSITIVITY_GYROSCOPE_245);
  Serial.print(", ");
  Serial.print(imu1.gz * SENSITIVITY_GYROSCOPE_245);
  Serial.print(", ");
  Serial.print(imu2.gx * SENSITIVITY_GYROSCOPE_245);
  Serial.print(", ");
  Serial.print(imu2.gy * SENSITIVITY_GYROSCOPE_245);
  Serial.print(", ");
  Serial.print(imu2.gz * SENSITIVITY_GYROSCOPE_245);
//////////PRINT ACCEL//////////////
  Serial.print(", ");
  Serial.print(imu1.ax * SENSITIVITY_ACCELEROMETER_2);
  Serial.print(", ");
  Serial.print(imu1.ay * SENSITIVITY_ACCELEROMETER_2);
  Serial.print(", ");
  Serial.print(imu1.az * SENSITIVITY_ACCELEROMETER_2);
  Serial.print(", ");
  Serial.print(imu2.ax * SENSITIVITY_ACCELEROMETER_2);
  Serial.print(", ");
  Serial.print(imu2.ay * SENSITIVITY_ACCELEROMETER_2);
  Serial.print(", ");
  Serial.print(imu2.az * SENSITIVITY_ACCELEROMETER_2);
  Serial.print(", ");
      //printMag();   // Print "M: mx, my, mz"
      // Print the heading and orientation for fun!
      // Call print attitude. The LSM9DS1's mag x and y
      // axes are opposite to the accelerometer, so my, mx are
      // substituted for each other.
      //printAttitude(imu.ax, imu.ay, imu.az,
      //               -imu.my, -imu.mx, imu.mz);
     
      lastPrint = millis(); // Update lastPrint time
      }
     }
////////////////CHANNEL CLOSED//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   // Serial.println("Disconnecting...");//on keyboard detect, break and start disconnecting
//    client.stop();
//    while(1);
}

////////////////FUNCITONS//////////////////////////////////////////////////////////////////////
