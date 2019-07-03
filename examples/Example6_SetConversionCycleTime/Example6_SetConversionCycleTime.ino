/******************************************************************************
  Example6_SetConversionCycleTime.ino
  Example for the TMP117 I2C Temperature Sensor
  Madison Chodikov @ SparkFun Electronics
  July 3rd 2019
  ~

  This sketch can set and get the Conversion Times in Continuous Conversion mode
  for the Sensor. A chart for the averaging modes and the conversion times can
  be found in the comments below or in the Datasheet on Page 26, Table 7.

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


     Conversion Cycle Times in CC Mode (found in the datasheet page 26 table 6)
               AVG       0       1       2       3
       CONV  averaging  (0)     (8)     (32)   (64)
         0             15.5ms  125ms   500ms    1s
         1             125ms   125ms   500ms    1s
         2             250ms   250ms   500ms    1s
         3             500ms   500ms   500ms    1s
         4             1s      1s      1s       1s
         5             4s      4s      4s       4s
         6             8s      8s      8s       8s
         7             16s     16s     16s      16s

  CONV = Conversion Cycle Bit
  AVG = Conversion Averaging Mode
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

  Serial.print("Current Conversion Average Mode: ");
  Serial.println(sensor.getConversionAverageMode());
  Serial.print("Current Conversion Cycle Bit Value: ");
  Serial.println(sensor.getConversionCycleBit());
}


// Global variables declared for the loop
uint8_t cycleBit = 0;
uint8_t avMode = 0;

// For function to work, make sure the Serial Monitor is set to "No Line Ending"
void loop()
{
  Serial.println("Enter 1 for changing the Averaging Mode or 2 for changing the Cycle Bit Value: ");
  while (Serial.available() == 0); // Waits for the user input
  int mode = Serial.parseInt(); // Reads the input from the serial port
  if (mode == 1)
  {
    Serial.print("Current Averaging Mode: ");
    Serial.println(sensor.getConversionAverageMode());
    Serial.println("Please 0 - 3 for the New Averaging Mode: ");
    while (Serial.available() == 0); // Waits for the user input
    avMode = Serial.parseInt();
    if ((avMode > 0) || (avMode < 3))
    {
      sensor.setConversionAverageMode((uint8_t)avMode);
      Serial.print("New Conversion Average Mode: ");
      Serial.println(sensor.getConversionAverageMode());
    }
    else
    {
      Serial.println("Please Enter a Number 0 - 3");
    }
  }

  else if (mode == 2)
  {
    Serial.print("Current Cycle Bit Value: ");
    Serial.println(sensor.getConversionCycleBit());
    Serial.println("Please enter 0 - 7 for the Conversion Cycle Bit: ");
    while (Serial.available() == 0); // Waits for the user input
    cycleBit = Serial.parseInt(); // Reads the input from the serial port
    if ((cycleBit > 0) || (cycleBit < 7))
    {
      sensor.setConversionCycleBit((uint8_t)cycleBit);
      Serial.print("New Conversion Cycle Bit Value: ");
      Serial.println(sensor.getConversionCycleBit());
    }
    else
    {
      Serial.println("Please enter a Number 0 - 7");
    }
  }
  else
  {
    Serial.println("Please enter 1 or 2");
  }
  Serial.println(""); // Create a whitespace for easier readings
}
