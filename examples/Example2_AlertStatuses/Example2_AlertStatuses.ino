/******************************************************************************
  SparkFun_TMP117_Breakout_Example.ino
  Example for the TMP117 I2C Temperature Sensor
  Madison Chodikov @ SparkFun Electronics
  May 29 2019
  ~

  This sketch configures the TMP117 temperature sensor and prints the
  alert state of the temperature sensor.

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
  NOTE: For the most accurate readings:
  - Avoid heavy bypass traffic on the I2C bus
  - Use the highest available communication speeds
  - Use the minimal supply voltage acceptable for the system
  - Place device horizontally and out of any airflow when storing
  For more information on reaching the most accurate readings from the sensor,
  reference the "Precise Temperature Measurements with TMP116" datasheet that is
  linked on Page 35 of the TMP117's datasheet
*/

#include <Wire.h> // Used to establish serial communication on the I2C bus
#include <SparkFun_TMP117.h> // Used to send and recieve specific information from our sensor

// The default address of the device is 0x48
// Sensor address can be changed with an external jumper to:
// VCC = 0x49
// SDA = 0x4A
// SCL = 0x4B
TMP117 sensor; // Initalize sensor


void setup() 
{
  Wire.begin();
  Serial.begin(115200); // Start serial communication at 115200 baud
  Serial.println("TMP117 Example 2: Alert Statuses");

  if (sensor.begin() == true)
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
  /*  Alert statuses below for really high or really low temperature reading possibilities */
  Serial.print("High Alert Status: ");
  Serial.println(sensor.isHighAlert()); // Prints true if the upper threshold is reached and false otherwise
  Serial.print("Low Alert Status: ");
  Serial.println(sensor.isLowAlert()); // Prints true if the lower threshold is reached and false otherwise
}
