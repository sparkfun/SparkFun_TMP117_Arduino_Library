/******************************************************************************
  SparkFun_TMP117_Breakout_Example.ino
  Example for the TMP117 I2C Temperature Sensor
  Madison Chodikov @ SparkFun Electronics
  June 11 2019
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

// The default address of the device is 0x48 (GND)
// Sensor address can be changed with an external jumper to:
// VCC = 0x49
// SDA = 0x4A
// SCL = 0x4B
TMP117 sensor; // Initalize sensor


void setup()
{
  Wire.begin();
  Serial.begin(115200); // Start serial communication at 115200 baud
  Wire.setClock(400000); // Set clock speed to be the fastest for better communication (fast mode)
  Serial.println("TMP117 Example 5: Setting High and Low Temperature Limits");
  if (sensor.begin() == true)
  {
    Serial.println("Begin");
  }
  else
  {
    Serial.println("Device failed to setup");
    while (1);
  }
}


void loop()
{
  if (sensor.begin() == true)
  {
    Serial.print("Current Low Limit: ");
    Serial.println(sensor.getConversionMode());
    Serial.print("Current High Limit: ");
    Serial.println("Enter which limit to change, 1 for Low Limit and 2 for High Limit: ");
    while (Serial.available() == 0); // Waits for the user input
    mode = Serial.read(); // Reads the input from the serial port
    if (mode == 1)
    {
      Serial.println("Please enter Low Limit Temperature (between -256 and 256): ");
    }
    else if (mode == 2)
    {
      while (Serial.available() == 0); // Waits for the user input
      mode = Serial.read(); // Reads the input from the serial port
    }
    else
    {
      Serial.println("Please enter 1 or 2");
    }


    // Left for reference 
    Serial.print("Number recieved: ");
    Serial.println(mode);
    delay(500);
    if (mode == '0' || mode == '1' || mode == '2' || mode == '3')
    {
      sensor.setConversionMode(mode);
      Serial.println();
      delay(500);
    }
    else
    {
      Serial.println("Conversion mode unsuccessfully set - Please enter a number 0 - 3");
    }
    delay(1000);
  }
