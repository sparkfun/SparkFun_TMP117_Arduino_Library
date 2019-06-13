/******************************************************************************
  SparkFun_TMP117_Breakout_Example.ino
  Example for the TMP117 I2C Temperature Sensor
  Madison Chodikov @ SparkFun Electronics
  May 29 2019
  ~

  This sketch configures the TMP117 temperature sensor and prints the
  temperature.

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
  Serial.begin(115200); // Start serial communication at 115200 baud
  Wire.setClock(400000); // Set I2C bus to 400kHz
  Serial.println("TMP117 Example 4: Set Conversion Mode \r\n");

  //make sure the sensor is set up properly
  if(sensor.isAlive()) {
    Serial.println("Device found. I2C connections are good.");
  }
  
  else {
    Serial.println("Device not found. Check your connections and reset.");
    while(1); //hang forever
  }

  Serial.print("Current Conversion Mode: ");
  Serial.println(sensor.getConversionMode());
  Serial.println("Enter your mode of Conversion (number 0 - 3): ");

}

void loop()
{
  if(Serial.available()){
    uint8_t input = Serial.read();

    if(isdigit(input) && (input <= '3')){
      mode = input - '0';

      Serial.print("Updated Conversion Mode Received: ");
      Serial.println(mode);

      //update the conversion mode on the TMP117
      sensor.setConversionMode(mode);
    }  
  }

  if(sensor.dataReady()){
    Serial.print("Current Conversion Mode: ");
    Serial.print(sensor.getConversionMode());
    
    Serial.print(" Temp (C): ");
    Serial.println(sensor.readTempC());
  }
}

