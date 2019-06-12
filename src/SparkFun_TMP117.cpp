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
TMP117::TMP117(byte address)
{
	_address = address;
	alert_type = NOALERT;
}

/* GET ADDRESS
	This function calls for the current address of the device to be
	set up. The addresses are 0x48 = GND, 0x49 = V+, 0x4A = ADD0, 0x4B = SCL
*/
uint8_t TMP117::getAddress()
{
	return TMP117_I2C_ADDR;
}

/* SET ADDRESS
	This function calls for the user to write the address of the 
	device with 0x48 = GND, 0x49 = V+, 0x4A = ADD0, 0x4B = SCL
	The sensor can be used to connect up to 4 devices if the addresses
	are called correctly (Found on Page 19, Table 2)
*/
void TMP117::setAddress(uint8_t addr)
{
	addr = TMP117_I2C_ADDR;
}

/* BEGIN INITIALIZATION
    This function initalizes the TMP117 sensor and opens up the registers.
	This returns true when it was successfully set up and false otherwise.
*/
bool TMP117::begin(TwoWire &wirePort, uint8_t deviceAddress)
{
	// Set device address and wire port to private variable
	_address = deviceAddress;
	_i2cPort = &wirePort;
	byte rawData[2]; // Array created for values needed to be returned in for shifting
	int16_t digitalTempC = 0;
	if (isConnected() == false) // Returns false when not able to be connected
	{
		return false;
	}

	readRegisters(TMP117_DEVICE_ID, rawData, 2); // Calls to read registers to pull all the bits to store in an array
	int16_t deviceID = 0;						 // send to other function to get actual device id value
	byte MSB = rawData[0];
	byte LSB = rawData[1];
	deviceID = (MSB << 8) | (LSB & 0xFF); // Must be signed

	// DEVICE_ID should always be 0x0117
	// Checks to see if properly connected
	if (deviceID != DEVICE_ID_VALUE)
	{
		return false;
	}

	return true; // Returns true when all the checks are passed
}

/* IS CONNECTED
	This function returns true if the I2C Device acknowledges a connection.
    Otherwise returns false.
*/
bool TMP117::isConnected()
{
	_i2cPort->beginTransmission((uint8_t)_address);
	if (_i2cPort->endTransmission() == 0)
	{
		return (true);
	}
	else
	{
		Serial.println("Device unable to connect.");
		return (false);
	}
}

/* READ REGISTER
	This function reads the register bytes from the sensor when called upon.
*/
uint16_t TMP117::readRegister(TMP117_Register reg)
{
	_i2cPort->beginTransmission((uint8_t)_address);
	_i2cPort->write(reg);
	_i2cPort->endTransmission(false);			 // endTransmission but keep the connection active
	_i2cPort->requestFrom(_address, (uint8_t)2); // Ask for 2 bytes, once done, bus is released by default

	// Wait for the data to come back
	if (_i2cPort->available())
	{
		uint16_t response = _i2cPort->read() << 8; //Big endian (MSB)
		response |= _i2cPort->read();
		return response;
	}
	else
	{
		return 0;
	}
}

/* READ MULTIPLE REGISTERS
    Read "en" bytes from the TMP117, starting at register "reg." Bytes are 
    stored in "buffer" on exit.
*/
void TMP117::readRegisters(TMP117_Register reg, byte *buffer, byte len)
{
	_i2cPort->beginTransmission(_address);
	_i2cPort->write(reg);
	_i2cPort->endTransmission(false);	 // endTransmission but keep the connection active
	_i2cPort->requestFrom(_address, len); // Ask for bytes, once done, bus is released by default

	// Wait for data to come back
	if (_i2cPort->available() == len)
	{
		// Iterate through data from buffer
		for (uint8_t i = 0; i < len; i++)
			buffer[i] = _i2cPort->read();
	}
}

/* WRITE TO MULTIPLE REGISTERS
	Write an array of "len" bytes ("buffer"), starting at register "reg,"
    and auto-incrementing to the next.
*/
void TMP117::writeRegisters(TMP117_Register reg, byte *buffer, byte len)
{
	_i2cPort->beginTransmission((uint8_t)_address);
	_i2cPort->write(reg);
	for (int i = 0; i < len; i++) // Loop to run through all the bits requested
		_i2cPort->write(buffer[i]);
	_i2cPort->endTransmission(); // Stop transmitting
}

/* WRITE TO A SINGLE REGISTER
    Wire a single bit of data to a register in TMP117.
*/
void TMP117::writeRegister(TMP117_Register reg, byte data)
{
	writeRegisters(reg, &data, 1); // Only calls to write to one byte in the register
}

/* READ TEMPERATURE CELSIUS
	This function reads the temperature reading from the sensor
	and returns the value in degrees celsius.

	NOTE: The data type of digitalTemp is a signed integer, meaning that the 
	value of the binary number being read will be negative if the MSB is 1,
	and positive if the bit is 0. 
*/
float TMP117::readTempC()
{
	byte rawData[2] = {0}; // Sets the array of rawData equal to 0 initially
	int16_t digitalTempC = 0;
	readRegisters(TMP117_TEMP_RESULT, rawData, 2); // Calls to read registers to pull all the bits to store in an array
	byte MSB = rawData[0];
	byte LSB = rawData[1];
	digitalTempC = (MSB << 8) | (LSB & 0xFF);			 // Must be unsigned
	float finalTempC = digitalTempC * TMP117_RESOLUTION; // Multiplies by the resolution for digital to final temp

	return finalTempC;
}

/* READ TEMPERATURE FAHRENHEIT
	This function calculates the fahrenheit reading from the
	celsius reading initially found.
	The device reads in celsius unless called by this function
*/
float TMP117::readTempF()
{
	return readTempC() * 9.0 / 5.0 + 32.0;
}

/* GET TEMPERATURE OFFSET
	This function reads the temperature offset. This reads from the register
	value 0x07 (TMP117_TEMP_OFFSET)
*/
float TMP117::getTemperatureOffset() // Reads the temperature offset
{
	byte rawData[2];
	readRegisters(TMP117_TEMP_OFFSET, rawData, 2); // Calls to read registers to pull all the bits to store in an array
	byte MSB = rawData[0];
	byte LSB = rawData[1];
	uint16_t unsignedValue = (MSB << 8) | (LSB & 0xFF); // Must be unsigned
	int16_t finalOffset = unsignedValue * TMP117_RESOLUTION;
	return finalOffset;
}

/* SET OFFSET TEMPERATURE
	This function sets the offset temperature of the device. The user
	can write to this to set any desired offset within the temperature range.
	This writes to the register value 0x07 (TMP117_TEMP_OFFSET)
*/
void TMP117::setTemperatureOffset(uint16_t offset)
{
	CONFIGURATION_REG reg;
	reg.CONFIGURATION_COMBINED = offset;
	writeRegister(TMP117_CONFIGURATION, reg.CONFIGURATION_COMBINED);
}

/* GET ALERT
	This function returns the type of the alert being caused.
	Alerts are NOALERT, HIGHALERT, and LOWALERT.
*/
TMP117_ALERT TMP117::getAlert()
{
	return alert_type;
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
	Currently set in Continuous Conversion Mode.
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
	prints it to the Serial Monitor in the Arduino IDE
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
	in the above function
*/
uint8_t TMP117::getConversionCycleTime()
{
	CONFIGURATION_REG reg;
	reg.CONFIGURATION_COMBINED = readRegister(TMP117_CONFIGURATION);
	// reg.CONFIGURATION_FIELDS.CONV = 0b000;
	uint8_t cycle = reg.CONFIGURATION_FIELDS.CONV;
	return cycle;
}

/* SET CONVERSION CYCLE TIME
	This function sets the conversion cycle time of the device.
	This only works in Continuous Conversion mode, which was set 
	in an above function
*/
void TMP117::setConversionCycleTime(uint8_t cycle)
{
	CONFIGURATION_REG reg;
	reg.CONFIGURATION_COMBINED = readRegister(TMP117_CONFIGURATION);
	reg.CONFIGURATION_FIELDS.CONV = 0b000;
	// uint8_t cycle = reg.CONFIGURATION_FIELDS.CONV;
	writeRegister(TMP117_CONFIGURATION, reg.CONFIGURATION_COMBINED);

	// There is a chart of the conversion cycle times in the SparkFun_TMP117_Registers.h file
	// They can also be found in Table 7 on Page 26 of the datasheet
}

/* UNSIGNED WRITE REGISTER 16
	This function is used for converting all the values that have
	been read in 8 bit format to a 16 bit value.
	The functions getTemperatureOffset(), readTempC() call this.
*/
// uint16_t TMP117::unsignedWriteRegister16(byte rawData[2])
// {
// 	byte rawData[2];
// 	byte MSB = rawData[0];
// 	byte LSB = rawData[1];
// 	uint16_t unsignedValue = (MSB << 8) | (LSB & 0xFF); // Must be unsigned
// 	return unsignedValue;
// }

/* SIGNED WRITE REGISTER 16
	This function is used for converting all the values that have
	been read in 8 bit format to a 16 bit value.
	The function begin() calls this.
*/
// int16_t TMP117::signedWriteRegister16(byte rawData[2])
// {
// 	byte rawData[2];
// 	byte MSB = rawData[0];
// 	byte LSB = rawData[1];
// 	int16_t signedValue = (MSB << 8) | (LSB & 0xFF); // Must be signed
// 	return signedValue;
// }

/* DATA READY
	This function checks to see if there is data ready to be sent
	from the TMP117. This can be found in Page 25 Table 6 of the 
	data sheet.
*/
bool TMP117::dataReady()
{
	uint16_t response = readRegister(TMP117_CONFIGURATION);

	if (response & 1 << 13)
	{
		return true;
	}
	else
	{
		return false;
	}

	/* Code below not fully working yet */
	// CONFIGURATION_REG reg;
	// reg.CONFIGURATION_COMBINED = readRegister(TMP117_CONFIGURATION);
	// uint8_t ready = reg.CONFIGURATION_FIELDS.DATA_READY;
	// if (ready == 1)
	// {
	// 	Serial.println("Data Available");
	// 	return true;
	// }
	// else
	// {
	// 	Serial.println("Data not available");
	// 	return false;
	// }
}
