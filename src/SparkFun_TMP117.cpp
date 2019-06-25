/******************************************************************************
SparkFunTMP117.cpp
SparkFunTMP117 Library Source File
Madison Chodikov @ SparkFun Electronics
Original Creation Date: April 22, 2019
https://github.com/sparkfunX/Qwiic_TMP117

This file implements all functions of the TMP117 class. Functions here range
from reading the temperature from the sensor, to reading and writing various
settings in the sensor.

Development environment specifics:
	IDE: Arduino 1.8.9
	Hardware Platform: Arduino Uno
	TMP117 Breakout Version: 1.0

This code is beerware; if you see me (or any other SparkFun employee) at the
local, and you've found our code helpful, please buy us a round!

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

#include <Arduino.h>
#include <Wire.h>
#include "Sparkfun_TMP117_Registers.h"
#include "SparkFun_TMP117.h"

/* CONSTRUCTOR
    This function, called when you initialize the class will write 
    the variable address into a private variable for future use. 
    The initial variable address should be 0x48. 
*/
TMP117::TMP117(TwoWire &wirePort, uint8_t addr)
{
	_deviceAddress = addr;
	_i2cPort = &wirePort; //use the main I2C port on the Arduino by default, but this is configurable with the setBus function
}

/* GET ADDRESS
	This function calls for the current address of the device to be
	set up. The addresses are 0x48 = GND, 0x49 = V+, 0x4A = ADD0, 0x4B = SCL
*/
uint8_t TMP117::getAddress()
{
	return _deviceAddress;
}

/* SET ADDRESS
	This function calls for the user to write the address of the 
	device with 0x48 = GND, 0x49 = V+, 0x4A = ADD0, 0x4B = SCL
	The sensor can be used to connect up to 4 devices if the addresses
	are called correctly (Found on Page 19, Table 2)
*/
void TMP117::setAddress(uint8_t addr)
{
	_deviceAddress = addr;
}

/* IS ALIVE
    This function checks if the TMP will ACK over I2C, and
	if the TMP will correctly self-identify with the proper
	device ID. Returns true if both checks pass.
*/
bool TMP117::begin()
{
	//make sure the TMP will acknowledge over I2C
	_i2cPort->beginTransmission(_deviceAddress);
	if (_i2cPort->endTransmission() != 0)
	{
		return false;
	}

	uint16_t deviceID = readRegister(TMP117_DEVICE_ID); // reads registers into rawData

	//make sure the device ID reported by the TMP is correct
	//should always be 0x0117
	if (deviceID != DEVICE_ID_VALUE)
	{
		return false;
	}

	return true; //returns true if all the checks pass
}

/* READ REGISTER
	This function reads the register bytes from the sensor when called upon.
	This reads 2 bytes of information from the 16-bit registers. 
*/
uint16_t TMP117::readRegister(uint8_t reg) // originally TMP117_Register reg
{
	_i2cPort->beginTransmission(_deviceAddress); // Originally cast (uint8_t)
	_i2cPort->write(reg);
	_i2cPort->endTransmission();					   // endTransmission but keep the connection active
	_i2cPort->requestFrom(_deviceAddress, (uint8_t)2); // Ask for 2 bytes, once done, bus is released by default

	uint8_t data[2] = {0};	 // Declares an array of length 2 to be empty
	int16_t datac = 0;		   // Declares the return variable to be 0
	if (Wire.available() <= 2) // Won't read more than 2 bits
	{
		data[0] = Wire.read();				// Reads the first set of bits (D15-D8)
		data[1] = Wire.read();				// Reads the second set of bits (D7-D0)
		datac = ((data[0] << 8) | data[1]); // Swap the LSB and the MSB
	}
	return datac;
}

/* WRITE REGISTER
    Wire data to a TMP117 register
*/
void TMP117::writeRegister(uint8_t reg, uint16_t data) // originally TMP117_Register reg
{
	_i2cPort->beginTransmission(_deviceAddress); // Originally cast uint8_t when a register value again
	_i2cPort->write(reg);
	_i2cPort->write(highByte(data)); // Write MSB (D15-D8)
	_i2cPort->write(lowByte(data));  // Write LSB (D7-D0)
	_i2cPort->endTransmission();	 // Stop transmitting data
}

/* READ TEMPERATURE CELSIUS
	This function reads the temperature reading from the sensor
	and returns the value in degrees celsius.

	NOTE: The data type of digitalTemp is a signed integer, meaning that the 
	value of the binary number being read will be negative if the MSB is 1,
	and positive if the bit is 0. 
*/
double TMP117::readTempC()
{
	int16_t digitalTempC = readRegister(TMP117_TEMP_RESULT); // Calls to read registers to pull all the bits to store in an array

	float finalTempC = digitalTempC * TMP117_RESOLUTION; // Multiplies by the resolution for digital to final temp

	return finalTempC;
}

/* READ TEMPERATURE FAHRENHEIT
	This function calculates the fahrenheit reading from the
	celsius reading initially found.
	The device reads in celsius unless called by this function
*/
double TMP117::readTempF()
{
	return readTempC() * 9.0 / 5.0 + 32.0; // Conversion from °C to °F
}

/* GET TEMPERATURE OFFSET
	This function reads the temperature offset. This reads from the register
	value 0x07 (TMP117_TEMP_OFFSET). This can be found on page 23 of the 
	datasheet. 
*/
float TMP117::getTemperatureOffset()
{
	int16_t _offset = 0;
	_offset = readRegister(TMP117_TEMP_OFFSET);				// Reads from the temperature offset register
	float finalOffset = (float)_offset * TMP117_RESOLUTION; // Multiplies by the resolution for correct offset temperature
	return finalOffset;
}

/* SET OFFSET TEMPERATURE
	This function sets the offset temperature of the device. The user
	can write to this to set any desired offset within the temperature range.
	This writes to the register value 0x07 (TMP117_TEMP_OFFSET)
*/
void TMP117::setTemperatureOffset(float offset)
{
	int16_t resolutionOffset = offset / TMP117_RESOLUTION; // Divides by the resolution to send the correct value to the sensor
	writeRegister(TMP117_TEMP_OFFSET, resolutionOffset);   // Writes to the offset temperature register with the new offset value
}

/* GET LOW LIMIT
	This function reads the low limit register that is set by the user.
	The values are signed integers since they can be negative.
*/
float TMP117::getLowLimit()
{
	int16_t lowLimit = 0;
	lowLimit = readRegister(TMP117_T_LOW_LIMIT);			// Calls to read register to pull all the bits to store in a variable
	float finalLimit = (float)lowLimit * TMP117_RESOLUTION; // Multiplies by the resolution for correct offset temperature
	return finalLimit;
}

/* SET LOW LIMIT
	This function allows the user to set the low limit register to whatever
	specified value, as long as in the range for the temperature sensor. This
	function can be used as a threshold for Therm mode and or Alert mode.
	The values are signed integers since they can be negative.
*/
void TMP117::setLowLimit(float lowLimit)
{
	int16_t finalLimit = lowLimit / TMP117_RESOLUTION; // Divides by the resolution to send the correct value to the sensor
	writeRegister(TMP117_T_LOW_LIMIT, finalLimit);	 // Writes to the low limit temperature register with the new limit value
}

/* GET HIGH LIMIT
	This function reads the high limit register that is set by the user.
	The values are signed integers since they can be negative.
*/
float TMP117::getHighLimit()
{
	int16_t highLimit = 0;
	highLimit = readRegister(TMP117_T_HIGH_LIMIT);			 // Calls to read registers to pull all the bits to store in an array
	float finalLimit = (float)highLimit * TMP117_RESOLUTION; // Multiplies by the resolution for correct offset temperature
	return finalLimit;
}

/* SET HIGH LIMIT
	This function allows the user to set the high limit register to whatever
	specified value, as long as in the range for the temperature sensor. This
	function can be used as a threshold for Therm mode and or Alert mode
	The values are signed integers since they can be negative.
*/
void TMP117::setHighLimit(float highLimit)
{
	int16_t finalLimit = highLimit / TMP117_RESOLUTION; // Divides by the resolution to send the correct value to the sensor
	writeRegister(TMP117_T_HIGH_LIMIT, finalLimit);		// Writes to the high limit temperature register with the new limit value
}

/* SOFTWARE RESET
	This function performs a software reset, loading all the default
	values into the configuration register. This uses the struct in 
	the SparkFun_TMP117.h file.
*/
void TMP117::softReset()
{
	CONFIGURATION_REG reg;
	reg.CONFIGURATION_COMBINED = readRegister(TMP117_CONFIGURATION);
	reg.CONFIGURATION_FIELDS.SOFT_RESET = 0b1;
	// Writes to the register to be 1 when this function is called upon
	writeRegister(TMP117_CONFIGURATION, reg.CONFIGURATION_FIELDS.SOFT_RESET);
}

/* SET CONVERSION MODE
	This function writes the mode for the conversions.
	This can be found in the datasheet on Page 25 Table 6.
	The TMP117 defaults to Continuous Conversion Mode on reset.
*/
void TMP117::setConversionMode(uint8_t cycle)
{
	CONFIGURATION_REG reg;
	reg.CONFIGURATION_COMBINED = cycle;
	if (cycle == 0)
	{
		writeRegister(TMP117_CONFIGURATION, 0b00);
	}
	else if (cycle == 1)
	{
		writeRegister(TMP117_CONFIGURATION, 0b01);
	}
	else if (cycle == 2)
	{
		writeRegister(TMP117_CONFIGURATION, 0b10);
	}
	else if (cycle == 3)
	{
		writeRegister(TMP117_CONFIGURATION, 0b11);
	}
}

/* GET CONVERSION MODE
	This function reads the mode for the conversions, then
	prints it to the Serial Monitor in the Arduino IDE.
	This can be found in the datasheet on Page 25 Table 6. 
*/
uint8_t TMP117::getConversionMode()
{
	CONFIGURATION_REG reg;
	reg.CONFIGURATION_COMBINED = readRegister(TMP117_CONFIGURATION);
	uint8_t mode = reg.CONFIGURATION_FIELDS.MOD; // Stores the information from the MOD register
	return mode;
}

/* GET CONVERSION CYCLE TIME
	This function gets the conversion cycle time of the device.
	This only works in Continuous Conversion mode, which was set 
	in the above function.
*/
uint8_t TMP117::getConversionCycleTime()
{
	uint16_t cycleTime = readRegister(TMP117_CONFIGURATION);
	if (cycleTime & 0 << 6 && cycleTime & 0 << 5) // Checks to see if bits 5 and 6 are 00
	{
		return 0;
	}
	else if (cycleTime & 0 << 6 && cycleTime & 1 << 5) // Checks to see if bits 5 and 6 are 01
	{
		return 1;
	}
	else if (cycleTime & 1 << 6 && cycleTime & 0 << 5) // Checks to see if bits 5 and 6 are 10
	{
		return 2;
	}
	else if (cycleTime & 1 << 6 && cycleTime & 1 << 5) // Checks to see if bits 5 and 6 are 11
	{
		return 3;
	}
}

/* SET CONVERSION CYCLE TIME
	This function sets the conversion cycle time of the device.
	This only works in Continuous Conversion mode, which was set 
	in an above function. The table of conversion cycle times can
	be seen below

	  Conversion Cycle Time in CC Mode (found on the datasheet page 26 table 6)
              AVG       0       1       2       3
      CONV  averaging  (0)     (8)     (32)   (64)
        0             15.5ms  125ms   500ms    1s     C15mS5
        1             125ms   125ms   500ms    1s     C125mS
        2             250ms   250ms   500ms    1s     C250mS
        3             500ms   500ms   500ms    1s     C500mS
        4             1s      1s      1s       1s     C1S
        5             4s      4s      4s       4s     C4S
        6             8s      8s      8s       8s     C8S
        7             16s     16s     16s      16s    C16S
*/
void TMP117::setConversionCycleTime(uint16_t cycle)
{
	CONFIGURATION_REG reg;
	reg.CONFIGURATION_COMBINED = readRegister(TMP117_CONFIGURATION);
	reg.CONFIGURATION_FIELDS.CONV = 0b000;
	cycle = reg.CONFIGURATION_FIELDS.CONV;
	writeRegister(TMP117_CONFIGURATION, cycle);
}

/* DATA READY
	This function checks to see if there is data ready to be sent
	from the TMP117. This can be found in Page 25 Table 6 of the 
	data sheet.
*/
bool TMP117::dataReady()
{
	uint16_t response = readRegister(TMP117_CONFIGURATION);

	// If statement to see if the 13th bit of the register is 1 or not
	if (response & 1 << 13)
	{
		return true;
	}
	else
	{
		return false;
	}
}
