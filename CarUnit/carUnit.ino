#include <LiquidCrystal_I2C.h>
#include "ESP8266.h"
#include "freeram.h"
#include "mpu.h"
#include "I2Cdev.h"
#include <TinyGPS.h>              //GPS
TinyGPS gps;                     //GPS
#define SSID        ""
#define PASSWORD    ""
#define ALCOHOL;
LiquidCrystal_I2C lcd(0x3f, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE); 

const int AOUTpin=A0;//the AOUT pin of the alcohol sensor goes into analog pin A0 of the arduino
const int DOUTpin=7;//the DOUT pin of the alcohol sensor goes into digital pin D8 of the arduino
int limit=0;
int value;

ESP8266 wifi(Serial1);
void setup(void)
{
    wifi.setOprToStation()
    wifi.joinAP(SSID, PASSWORD) 
    
    lcd.begin(16,2);
    pinMode(DOUTpin, INPUT);
    Fastwire::setup(400,0);
    mympu_open(200);
    

}

void loop(void)
{
  mympu_update();

  Serial.print("\taX: "); Serial.print(mympu.accel[0]/2048);  
  Serial.print("g aY: "); Serial.print(mympu.accel[1]/2048);  
  Serial.print("g aZ: "); Serial.print(mympu.accel[2]/2048); Serial.println('g');
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
  for (uint8_t i=0; i<digits; ++i)
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
