/******************************************************************************
  SparkFun_TMP117_Breakout_Example.ino
  Example for the TMP117 I2C Temperature Sensor
  Madison Chodikov @ SparkFun Electronics
  June 11 2019
  ~

  This sketch can set and get the temperature limits for the sensor. These
  limits can be set within +/- 256°C.

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
  linked on Page 35 of the TMP117's datasheet.
*/

#include <Wire.h> // Used to establish serial communication on the I2C bus
#include <SparkFun_TMP117.h> // Used to send and recieve specific information from the sensor

// The default address of the device is 0x48 (GND)
// Sensor address can be changed with an external jumper to:
// VCC = 0x49
// SDA = 0x4A
// SCL = 0x4B
TMP117 sensor; // Initalize sensor


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

  Serial.print("Current Low Limit: ");
  Serial.println(sensor.getLowLimit());
  Serial.print("Current High Limit: ");
  Serial.println(sensor.getHighLimit());
}


// For function to work, make sure the Serial Monitor is set to "No Line Ending"
void loop()
{
  float lowTemp, highTemp;
  Serial.println("Enter which limit to change, 1 for Low Limit and 2 for High Limit: ");
  while (Serial.available() == 0); // Waits for the user input
  int limit = Serial.parseInt(); // Reads the input from the serial port
  if (limit == 1)
  {
    Serial.println("Please enter Low Limit Temperature (between -256°C and 255.9°C): ");
    while (Serial.available() == 0); // Waits for the user input
    lowTemp = Serial.parseFloat();
    sensor.setLowLimit(lowTemp);
    delay(1000);
    Serial.print("New Low Limit (in °C): ");
    Serial.println(sensor.getLowLimit());
  }
  else if (limit == 2)
  {
    Serial.println("Please enter High Limit Temperature (between -256°C and 255.9°C): ");
    while (Serial.available() == 0); // Waits for the user input
    highTemp = Serial.parseFloat(); // Reads the input from the serial port
    sensor.setHighLimit(highTemp);
    delay(1000);
    Serial.print("New High Limit (in °C): ");
    Serial.println(sensor.getHighLimit());
  }
  else
  {
    Serial.println("Please enter 1 or 2");
  }

}
