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


/* BEGIN INITIALIZATION
    This function initalizes the TMP117 sensor and opens up the registers.
*/
bool TMP117::begin(TwoWire &wirePort, uint16_t address) // originally uint8_t
{
	// Set device address and wire port to private variable
    _address = address;
    _i2cPort = &wirePort;
	byte rawData[2]; // Array created for 

    if (isConnected() == false) // Returns false when not able to be connected
    {
        return false;
    }

	readRegisters(DEVICE_ID, rawData, 2); // Calls to read registers to pull all the bits to store in an array
	byte MSB = rawData[0];
	byte LSB = rawData[1];

	uint16_t device_id_ = (MSB << 8) | (LSB & 0xFF);

    // DEVICE_ID should always be 0x0117
	// Checks to see if properly connected
    if (device_id_ != DEVICE_ID_VALUE)
    {
        return false;
    }

    return true; // Returns true when all the checks are passed
}


/* IS CONNECTED
	This function returns true if the I2C Device acknowledgs a connection.
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
    _i2cPort->endTransmission(false);     // endTransmission but keep the connection active
    _i2cPort->requestFrom(_address, (byte)1); // Ask for 1 byte, once done, bus is released by default

    // Wait for the data to come back
    if (_i2cPort->available())
    {
		return _i2cPort->read(); // Returns only one byte
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
    _i2cPort->endTransmission(false);           // endTransmission but keep the connection active
    _i2cPort->requestFrom(_address, len); // Ask for bytes, once done, bus is released by default

    // Wait for data to come back
    if (_i2cPort->available() == len)
    {
        // Iterate through data from buffer
        for (int i = 0; i < len; i++)
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
	readRegisters(TEMP_RESULT, rawData, 2); // Calls to read registers to pull all the bits to store in an array
	byte MSB = rawData[0]; // Stores the most significant bits
	byte LSB = rawData[1]; // Stores the least significant bits

	//Shifting the MSB to be in the MSB place from storing then changing the LSB values
	digitalTempC = (MSB << 8) | (LSB & 0xFF); // Must be signed

	float finalTempC = digitalTempC*TMP117_RESOLUTION; // Multiplies by the resolution for digital to final temp

	return finalTempC;
}


/* READ TEMPERATURE FAHRENHEIT
	This function calculates the fahrenheit reading from the
	celsius reading initially found.
	The device reads in celsius unless called by this function
*/
float TMP117::readTempF()	
{
	return readTempC()*9.0/5.0 + 32.0;
}


/* TEMPERATURE OFFSET
	This function reads the temperature offset.
	This register can also be written to, but is not included in this library.
*/
float TMP117::temperatureOffset() // Reads the temperature offset (for debugging purposes)
{
	byte rawData[2];
	readRegisters(TEMP_OFFSET, rawData, 2); // Calls to read registers to pull all the bits to store in an array
	byte MSB = rawData[0];
	byte LSB = rawData[1];
	int16_t tempOffset = (MSB << 8) | (LSB & 0xFF); // Must be signed
	int16_t finalOffset = tempOffset*TMP117_RESOLUTION;

}


/* GET ALERT
	This function returns the type of the alert being caused.
	Alerts are NOALERT, HIGHALERT, and LOWALERT.
*/
TMP117_ALERT TMP117::getAlert()
{
	return alert_type;
}


/* IS HIGH ALERT
	This function sets an alert when the temperature reading is too high
	for the device to handle. Returns true when Alert is high and false otherwise.
*/
bool TMP117::isHighAlert()
{
	CONFIGURATION_REG reg;
	// Read current configuration register value declared in SparkFun_TMP117.h file
	reg.CONFIGURATION_COMBINED = readRegister(1); // Reads 1 bit of information from the register
	uint8_t high_alert = reg.CONFIGURATION_FIELDS.HIGH_ALERT; // Picks which value to pull info from
	if(high_alert == 1)
	{
		return true;
	}
	else
	{
		return false;
	}
}


/* IS LOW ALERT
	This function sets an alert when the temperature reading is too low
	for the device to handle. Returns true when Alert is high and false otherwise.
*/
bool TMP117::isLowAlert()
{
	CONFIGURATION_REG reg;
	// Read current configuration register value declared in SparkFun_TMP117.h file
	reg.CONFIGURATION_COMBINED = readRegister(1); // Reads 1 bit from the register
	uint8_t low_alert = reg.CONFIGURATION_FIELDS.LOW_ALERT; 
	if(low_alert == 1) 
	{
		return true;
	}
	else
	{
		return false;
	}
}


/* SOFTWARE RESET
	This function performs a software reset, loading all the default
	values into the configuration register. This uses the struct in 
	the SparkFun_TMP117.h file.
*/
void TMP117::softReset()
{
	CONFIGURATION_REG reg;
	reg.CONFIGURATION_COMBINED = readRegister(1); // Reads 1 bit from the register
	uint8_t soft_rst = reg.CONFIGURATION_FIELDS.SOFT_RESET;
	writeRegister(soft_rst, 1); // Writes to the register SOFT_RESET to be 1 when called on
}
