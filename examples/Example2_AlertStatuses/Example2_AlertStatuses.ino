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

  Serial.println("TMP117 Example 2: Alert Statuses");
  if (sensor.begin() == true)
  {
    Serial.println("Begin");
  }
  else
  {
    Serial.println("Device failed to setup.");
    while (1);
  }
}

/* Alert statuses below for high or low temperature reading
   possibilities: High Alert = 256°C, Low Alert = -256°C*/
void loop()
{
  Serial.print("Current Temperature: ");
  Serial.println(sensor.readTempC());
  if(sensor.getHighAlert() == true)
  {
    Serial.println("High Alert");
  }
  else if(sensor.getLowAlert() == true)
  {
    Serial.println("Low Alert");
  }
  else
  {
    Serial.println("No Alert");
  }
  delay(1000); // Delay for 1 second before printing again
}
