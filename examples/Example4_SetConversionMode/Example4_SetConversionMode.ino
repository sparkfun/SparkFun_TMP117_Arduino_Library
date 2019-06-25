/******************************************************************************
  SparkFun_TMP117_Breakout_Example.ino
  Example for the TMP117 I2C Temperature Sensor
  Madison Chodikov @ SparkFun Electronics
  May 29 2019
  ~

  This sketch can get and set the conversion mode that the temperature sensor can be in,
  which is Continuous Conversion, Shutdown, or One-Shot. The specific values for these
  are found below at the end of the comments section

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


  The default address of the device is 0x48 (GND)
  Sensor address can be changed with an external jumper to:
  VCC = 0x49
  SDA = 0x4A
  SCL = 0x4B


  There are 4 different modes
  Continuous Conversion (CC) = 0b00 = 0
  Shutdown (SD) = 0b01 = 1
  Continuous Conversion (CC), Same as 00 (Reads back = 00) = 0b10 = 2
  One-Shot Conversion (OS) = 0b11 = 3
*/

#include <Wire.h> // Used to establish serial communication on the I2C bus
#include <SparkFun_TMP117.h> // Used to send and recieve specific information from our sensor

TMP117 sensor; // Initalize sensor object
uint8_t mode = 0; // The conversion mode to be used (the sensor is set to 0 by default)

void setup()
{
  Wire.begin();
  Serial.begin(115200);    // Start serial communication at 115200 baud
  Wire.setClock(400000);   // Set clock speed to be the fastest for better communication (fast mode)
  sensor.setAddress(0x48); // Set the address of the device - see above address comments

  Serial.println("TMP117 Example 5: Setting High and Low Temperature Limits");
  if (sensor.begin() == true) // Function to check if the sensor will correctly self-identify with the proper Device ID/Address
  {
    Serial.println("Begin");
  }
  else
  {
    Serial.println("Device failed to setup.");
    while (1); // Runs forever if the sensor does not initialize correctly
  }
  
  Serial.print("Current Conversion Mode: ");
  Serial.println(sensor.getConversionMode());
}



// For function to work, make sure the Serial Monitor is set to "No Line Ending"
void loop()
{
  Serial.println("Enter your mode of Conversion (number 0 - 3): ");
  while (Serial.available() == 0); // Waits for the user input
  byte correctMode = Serial.parseInt(); // Reads the input from the serial port
  Serial.print("Number recieved: ");
  Serial.println(correctMode);
  delay(500);
  if (correctMode == 0 || correctMode == 1 || correctMode == 2 || correctMode == 3)
  {
    sensor.setConversionMode((uint8_t)mode);
    Serial.print("New Conversion Mode: ");
    Serial.println(sensor.getConversionMode());
    Serial.println(); // Create a whitespace for easier readings
    // delay(500);
  }
  else
  {
    Serial.println("Conversion mode unsuccessfully set - Please enter a number 0 - 3");
    Serial.println(); // Create a whitespace for easier readings
  }
  delay(1000);
}
