#include <SparkFun_TMP117.h>
#include <SparkFun_TMP117_Registers.h>

#include "SparkFun_TMP117.h"
#include "SparkFun_TMP117_Registers.h"

/******************************************************************************
SparkFun_TMP117_Breakout_Example.ino
Example for the TMP117 I2C Temperature Sensor
Madison Chodikov @ SparkFun Electronics
May 16 2019
~

This sketch configures the TMP117 temperature sensor and prints the
temperature and alert state (both from the physical pin, as well as by
reading from the configuration register.)

Resources:
Wire.h (included with Arduino IDE)
SparkFunTMP117.h (included in the src folder)

Development environment specifics:
Arduino 1.8.9+
Hardware Version 1

This code is beerware; if you see me (or any other SparkFun employee) at
the local, and you've found our code helpful, please buy us a round!

Distributed as-is; no warranty is given.   
******************************************************************************/

/* 
 NOTE: Read for use for the most accurate readings from the sensor
  - Avoid heavy bypass traffic on the I2C bus for most accurate temperature readings
  - Use the highest available communication speeds
  - Use the minimal supply voltage acceptable for the system
  - Place device horizontally and out of any airflow when storing
  For more information on reaching the most accurate readings from the sensor, 
  reference the "Precise Temperature Measurements with TMP116" datasheet that is 
  linked on Page 35 of the TMP117's datasheet
*/

#include <Wire.h> // Used to establish serial communication on the I2C bus
#include <SparkFun_TMP117_Registers.h>
#include <Sparkfun_TMP117.h> // Used to send and recieve specific information from our sensor

// The initial value of the device is GND = 0x48, as declared above
// Sensor address can be changed with an external jumper to:
// VCC = 0x49
// SDA = 0x4A
// SCL = 0x4B
TMP117 sensor; // Initalize sensor at I2C address 0x48


void setup() 
{
  Wire.begin();
  Serial.begin(115200); // Start serial communication at 115200 baud
  if(sensor.begin() == true)
  {
    Serial.println("Begin");
  }
  else
  {
    Serial.println("Device failed to setup");
  }
}


void loop() 
{

  // Print temperature in C and F, and high and alert states.
//  Serial.print("Temperature in Celsius: ");
//  Serial.print(sensor.readTempC);
//  Serial.println("Temperature in Fahrenheit: ");
//  Serial.print(sensor.readTempF);
//  Serial.println("High Alert Status: ");
//  Serial.print(sensor.isHighAlert); // Prints true if the upper threshold is reached and false otherwise
//  Serial.println("Low Alert Status: ");
//  Serial.print(sensor.isLowAlert); // Prints true if the lower threshold is reached and false otherwise
//  Serial.println(); 
//  delay(1000); // Wait 1 second before reading the temperature and alert statuses again
}
