#include <LiquidCrystal_I2C.h>
#include "WiFiEsp.h"
#include "MPU6050lib.h"
#include <TinyGPS.h>              //GPS
TinyGPS gps;                     //GPS
#define ALCOHOL;
LiquidCrystal_I2C lcd(0x3f, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
const int AOUTpin = A0; //the AOUT pin of the alcohol sensor
const int DOUTpin = 7; //the DOUT pin of the alcohol sensor
int limit = 0;
int value;
int status = WL_IDLE_STATUS;     // the Wifi radio's status
WiFiEspClient client;
#define NSSID        "SAU-wifi"
#define PASS    "12345678"
#define HOST_NAME "10.85.94.76"
#define HOST_PORT 5100
//************************************************************Accelerometer
int intPin = 12;  // This can be changed, 2 and 3 are the Arduinos ext int pins
int16_t accelCount[3];           // Stores the 16-bit signed accelerometer sensor output
float ax, ay, az;                // Stores the real accel value in g's
int16_t gyroCount[3];            // Stores the 16-bit signed gyro sensor output
float gx, gy, gz;                // Stores the real gyro value in degrees per seconds
float gyroBias[3], accelBias[3]; // Bias corrections for gyro and accelerometer
int16_t tempCount;               // Stores the internal chip temperature sensor output
float temperature;               // Scaled temperature in degrees Celsius
float aRes, gRes; // scale resolutions per LSB for the sensors
MPU6050lib mpu;
//************************************************************Accelerometer
void setup(void)
{
  pinMode(intPin, INPUT);//for accelerometer
  digitalWrite(intPin, LOW);//for accelerometer
  Serial.begin(9600);
  mpu.calibrateMPU6050(gyroBias, accelBias);
  mpu.initMPU6050(); Serial.println("MPU6050 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature

  // initialize serial for ESP module
  Serial1.begin(115200);
  WiFi.init(&Serial1);
  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(NSSID);
    // Connect to WPA/WPA2 network
    status = WiFi.begin(NSSID, PASS);
  }
  Serial.println("You're connected to the network");
  printWifiStatus();
  Serial.println("\nStarting connection to server...");
  if (client.connect(HOST_NAME, HOST_PORT))
    Serial.println("connected");
  else
    Serial.println("not connacted");
  Wire.begin();

  lcd.begin(16, 2);
  pinMode(DOUTpin, INPUT);



}

void loop(void)
{
  // If data ready bit set, all data registers have new data
  if (mpu.readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01) { // check if data ready interrupt
    mpu.readAccelData(accelCount);  // Read the x/y/z adc values
    aRes = mpu.getAres();

    // Now we'll calculate the accleration value into actual g's
    ax = (float)accelCount[0] * aRes; // get actual g value, this depends on scale being set
    ay = (float)accelCount[1] * aRes;
    az = (float)accelCount[2] * aRes;

    mpu.readGyroData(gyroCount);  // Read the x/y/z adc values
    gRes = mpu.getGres();

    // Calculate the gyro value into actual degrees per second
    gx = (float)gyroCount[0] * gRes; // get actual gyro value, this depends on scale being set
    gy = (float)gyroCount[1] * gRes;
    gz = (float)gyroCount[2] * gRes;

    tempCount = mpu.readTempData();  // Read the x/y/z adc values
    temperature = ((float) tempCount) / 340. + 36.53; // Temperature in degrees Centigrade
  }

  client.print("Ger X,Y,Z " + String(gx) + " " + String(gy) + " " + String(gz) + " Acc X,Y,Z " + String(ax) + " " + String(ay) + " " + String(az) + " " + String(temperature));

}

void printFloat(double number, int digits)    //GPS
{
  // Handle negative numbers
  if (number < 0.0)
  {
    Serial.print('-');
    number = -number;
  }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i = 0; i < digits; ++i)
    rounding /= 10.0;

  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;
  Serial.print(int_part);

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0)
    Serial.print(".");

  // Extract digits from the remainder one at a time
  while (digits-- > 0)
  {
    remainder *= 10.0;
    int toPrint = int(remainder);
    Serial.print(toPrint);
    remainder -= toPrint;
  }
}
void printWifiStatus()
{
  // print the SSID of the network you're attached to
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength
  long rssi = WiFi.RSSI();
  Serial.print("Signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

