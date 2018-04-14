#include <LiquidCrystal_I2C.h>
#include "WiFiEsp.h"
#include "MPU6050lib.h"
#include <TinyGPS.h> 
TinyGPS gps;                     
LiquidCrystal_I2C lcd(0x3f, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
WiFiEspClient client;
const int DOUTpin = 2; //the DOUT pin of the alcohol sensor
volatile byte drunk = false;
void drunkDriver();
int status = WL_IDLE_STATUS;     // the Wifi radio's status
#define NSSID "wifi"
#define PASS  "0123456789"
#define HOST_NAME "0.0.0.0"
#define HOST_PORT 5100
#define airbag 22
#define carid "1f5d8ea9s6"
//************************************************************Accelerometer
int intPin = 3;  // This can be changed, 2 and 3 are the Arduinos ext int pins
int16_t accelCount[3];           // Stores the 16-bit signed accelerometer sensor output
float ax, ay, az;                // Stores the real accel value in g's
int16_t gyroCount[3];            // Stores the 16-bit signed gyro sensor output
float gx, gy, gz;                // Stores the real gyro value in degrees per seconds
float gyroBias[3], accelBias[3]; // Bias corrections for gyro and accelerometer
int16_t tempCount;               // Stores the internal chip temperature sensor output
float temperature;               // Scaled temperature in degrees Celsius
float SelfTest[6];               // Gyro and accelerometer self-test sensor output
uint32_t count = 0;
float aRes, gRes; // scale resolutions per LSB for the sensors
MPU6050lib mpu;
//************************************************************Accelerometer
float flat, flon, fspeed;
int year;
byte month, day, hour, minute, second, hundredths;
unsigned long fix_age;
void setup(void)
{
  Wire.begin();
  pinMode(DOUTpin, INPUT);
  digitalWrite(DOUTpin, LOW);
  attachInterrupt(digitalPinToInterrupt(DOUTpin), drunkDriver, HIGH);
  pinMode(intPin, INPUT);
  digitalWrite(intPin, LOW);
  Serial.begin(9600);
  Serial1.begin(115200);
  Serial2.begin(9600);
  mpu.MPU6050SelfTest(SelfTest);
  mpu.calibrateMPU6050(gyroBias, accelBias);
  mpu.initMPU6050();

  if (SelfTest[0] < 1.0f && SelfTest[1] < 1.0f && SelfTest[2] < 1.0f && SelfTest[3] < 1.0f && SelfTest[4] < 1.0f && SelfTest[5] < 1.0f) {
    Serial.println("Pass Selftest!");

    mpu.calibrateMPU6050(gyroBias, accelBias);
    mpu.initMPU6050();
  }
  else
  {
    Serial.println("Could not connect to MPU6050");
    while (1) ; // Loop forever if communication doesn't happen
  }
  lcd.begin(16, 2);
  lcd.clear();
  lcd.home();
  WiFi.init(&Serial1);

  while ( status != WL_CONNECTED) {
    lcd.clear();
    lcd.home();
    lcd.print("trying to ");
    lcd.setCursor(0,1);
    lcd.print("connect to wifi");
    Serial.println(NSSID);
    status = WiFi.begin(NSSID, PASS);
  }
   lcd.clear();
  lcd.home();
  lcd.print("connected to:");
  lcd.setCursor(0, 1);
  lcd.print(WiFi.SSID());

  if (client.connect(HOST_NAME, HOST_PORT))
  {
    lcd.clear();
    lcd.home();
    lcd.print("Connected to");
    lcd.setCursor(0,1);
    lcd.print("server");
   }
  else
  {
    lcd.clear();
    lcd.home();
    lcd.print("Couldn't connect");
    lcd.setCursor(0,1);
    lcd.print("to the server!");
   // while(1){}
  }

  lcd.clear();
  lcd.home();
  lcd.print("      V2I!      ");
  lcd.setCursor(0, 1);
  lcd.print("     SYSTEM     ");
  Serial.println("setup done");
}

void loop(void)
{
  //reconnect if wifi is disconnected
  if (!client.connected()) {
    client.connectSSL(HOST_NAME, HOST_PORT);
  }
  //look for airbag signal
  if (digitalRead(airbag) == HIGH) {
    client.print("airbag," + String(flat) + ',' + String(flon));
    lcd.clear();
    lcd.home();
    lcd.print("AIRBAG");
   // while (1) {}

  }
  //check gyro values for rollover
    if ( mpu.readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01)  { // check if data ready interrupt
    mpu.readGyroData(gyroCount);  // Read the x/y/z adc values
    gRes = mpu.getGres();
    // Calculate the gyro value into actual degrees per second
    gx = (float)gyroCount[0] * gRes - gyroBias[0]; // get actual gyro value, this depends on scale being set
    gy = (float)gyroCount[1] * gRes - gyroBias[1];
    gz = (float)gyroCount[2] * gRes - gyroBias[2];
    if ((gx > 45 || gx < -45 || gy > 45 || gy < -45) && millis() > 400 ) {
      client.print("rollover," + String(flat) + ',' + String(flon));
      lcd.clear();
      lcd.home();
      lcd.print("ROLLOVER");
    //  while(1){}
    }
  }

  //check gps for speed and position
    if (Serial2.available()) {
    char c = Serial2.read();
    if (gps.encode(c))
    {
      gps.f_get_position(&flat, &flon);
      fspeed = gps.f_speed_kmph();
      gps.crack_datetime(&year, &month, &day,&hour, &minute, &second, &hundredths, &fix_age);
    }
  }
  //report if speed is high
  if (fspeed > 100) {
    lcd.clear();
    lcd.home();
    lcd.print("SPEED TICKET");
    lcd.setCursor(0, 1);
    lcd.print("AM CALLING POLICE!");
    client.print("speed," + String(flat) + ',' + String(flon));
   //while (1) {}
  }
  //report if drunk
    if (drunk) {
    lcd.clear();
    lcd.home();
    lcd.print("YOU ARE DRUNK");
    lcd.setCursor(0, 1);
    lcd.print("CALLING POLICE!");
    client.print("drunk," + String(flat) + ',' + String(flon));
   // while (1) {}
   // drunk = false;
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
void drunkDriver() {
  drunk = true;
}

