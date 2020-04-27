#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>

//Adding definitions for new sensor on I2C
#define SDA_1 27
#define SCL_1 26
#define SDA_2 33
#define SCL_2 32
#define SENSITIVITY_ACCELEROMETER_2  0.000061
#define SENSITIVITY_ACCELEROMETER_4  0.000122
#define SENSITIVITY_ACCELEROMETER_8  0.000244
#define SENSITIVITY_ACCELEROMETER_16 0.000732
#define SENSITIVITY_GYROSCOPE_245    0.00875
#define SENSITIVITY_GYROSCOPE_500    0.0175
#define SENSITIVITY_GYROSCOPE_2000   0.07
#define PRINT_CALCULATED
//#define PRINT_RAW
#define PRINT_SPEED 250

// Keep track of print time
static unsigned long lastPrint = 0;

// Sensor library
LSM9DS1 imu1;
LSM9DS1 imu2;

//Function prototypes
void getData();
void buildPacket();
void printGyro();
void printAccel();
void printMag();
void printAttitude(float ax, float ay, float az, float mx, float my, float mz);

/* WiFi network name and password */
const char* ssid = "MySpectrumWiFieb-2G";
const char* password =  "outletwalnut604";
// IP address to send UDP data to.
// it can be ip address of the server or
// a network broadcast address
// here is broadcast address
const char * udpAddress = "192.168.1.232";
const int udpPort = 8090;
struct pkt {short Time, gx1, gy1, gz1, gx2, gy2, gz2, ax1, ay1, az1, ax2, ay2, az2;} pkt;
//create UDP instance
WiFiUDP udp;
char buffer[1024];

void setup(){
  Serial.begin(115200);
  Wire.begin(SDA_1, SCL_1, 100000); 
  Wire1.begin(SDA_2, SCL_2, 100000);
  //Connect to the WiFi network
  WiFi.begin(ssid, password);
  Serial.println("");
  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("\nStarting connection to server...");
  //This initializes udp and transfer buffer
  
  udp.begin(udpPort);
}

void loop(){
  if (imu1.begin() == false || imu2.begin(0x6B, 0x1E, Wire1) == false){ // with no arguments, this uses default addresses (AG:0x6B, M:0x1E) and i2c port (Wire).
     Serial.println("Failed to communicate with LSM9DS1.");
     Serial.println("Double-check wiring.");
     Serial.println("Default settings in this sketch will " \
                   "work for an out of the box LSM9DS1 " \
                   "Breakout, but may need to be modified " \
                   "if the board jumpers are.");
     while (1);
     }
  
  if ((lastPrint) < millis()){
      Serial.println("Checking for available data");
      getData();
      buildPacket();
      lastPrint = millis();
      }
  delay(100);
  Serial.println("Sending packet...");
  udp.beginPacket(udpAddress, udpPort);
  udp.write((byte *) &pkt, sizeof pkt);
  udp.endPacket();
  Serial.println("Packet sent...");
  //processing incoming packet, must be called before reading the buffer
  udp.parsePacket();
  //receive response from server, it will be HELLO WORLD
  if(udp.read(buffer, 50) > 0){
    Serial.print("Server to client: ");
    Serial.println((char *)buffer);
  }
  //Wait for 1 second
  delay(1000);
}

//////////////////////FUNCTIONS/////////////////////////////////
void buildPacket(){
  pkt.Time = (float)millis();
  pkt.gx1 = imu1.gx;
  pkt.gy1 = imu1.gy;
  pkt.gz1 = imu1.gz;
  pkt.gx2 = imu2.gx;
  pkt.gy2 = imu2.gy;
  pkt.gz2 = imu2.gz;
  pkt.ax1 = imu1.ax;
  pkt.ay1 = imu1.ay;
  pkt.az1 = imu1.az;
  pkt.ax2 = imu2.ax;
  pkt.ay2 = imu2.ay;
  pkt.az2 = imu2.az;
  }

void getData(){
          if ( imu1.gyroAvailable() || imu2.gyroAvailable()){
          // To read from the gyroscope,  first call the
          // readGyro() function. When it exits, it'll update the
          // gx, gy, and gz variables with the most current data.
          imu1.readGyro();
          imu2.readGyro();
      }
          if ( imu1.accelAvailable() || imu2.accelAvailable() ){
          // To read from the accelerometer, first call the
          // readAccel() function. When it exits, it'll update the
          // ax, ay, and az variables with the most current data.
          imu1.readAccel();
          imu2.readAccel();
          }
}
