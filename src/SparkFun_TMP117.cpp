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

    if (isConnected() == false)
    {
        return false;
    }
    // Read DEVICE_ID register
    uint16_t deviceIDValue = readRegister(DEVICE_ID);

    // DEVICE_ID should always be 0x0117
	// Checks to see if properly connected
    if (deviceIDValue != DEVICE_ID)
    {
        return false;
    }

    return true;
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
    _i2cPort->endTransmission(false);               // endTransmission but keep the connection active
    _i2cPort->requestFrom(_address, (byte)2); // Ask for 2 byte, once done, bus is released by default

    // Wait for the data to come back
    if (_i2cPort->available())
    {
        return _i2cPort->read(); // Return this one byte
    }
    else
    {
        return 0;
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
    for (int i = 0; i < len; i++)
        _i2cPort->write(buffer[i]);
    _i2cPort->endTransmission(); // Stop transmitting
}


/* WRITE TO A SINGLE REGISTER
    Wire a single bit of data to a register in TMP117.
*/
void TMP117::writeRegister(TMP117_Register reg, byte data)
{
    writeRegisters(reg, &data, 1);
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
	int16_t digitalTempC;      // Temperature stored in the TMP117 register (must be signed)

	digitalTempC = readRegister(TEMP_RESULT); //Reads the temperature from the sensor

	int16_t finalTempC = digitalTempC*TMP117_RESOLUTION;

	return finalTempC;
}


/* READ TEMPERATURE FAHRENHEIT
	This function calculates the fahrenheit reading from the
	celsius reading initially found.
*/
float TMP117::readTempF()	// Module reads in celcius unless called by this function
{
	return readTempC()*9.0/5.0 + 32.0;
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


// /* CONVERSION MODE
// 	This function sets the mode for the conversions.
// 	This can be found in the datasheet on Page 25 Table 6.
// 	Currently set in Continuous Conversion Mode.
// */
// void TMP117::conversionMode()
// {
// 	CONFIGURATION_REG reg;
// 	reg.CONFIGURATION_COMBINED = readRegister(2); // Reads 2 bits from the register
// 	uint8_t mode = reg.CONFIGURATION_FIELDS.MOD;
// 	writeRegisters(cycle, 0b00, 2); // Continuous Conversion (CC)
// 	// writeRegisters(cycle, 01, 2) // Shutdown (SD)
// 	// writeRegisters(cycle, 10, 2) // Continuous Conversion (CC), Same as 00 (reads back = 00)
// 	// writeRegisters(cycle, 11, 2) // One-Shot Conversion (OS)
// }


// /* CONVERSION CYCLE TIME
// 	This function sets the conversion cycle time of the device.
// 	This only works in Continuous Conversion mode, which was set 
// 	in the above function
// */
// void conversionCycleTime()
// {
// 	CONFIGURATION_REG reg;
// 	reg.CONFIGURATION_COMBINED = readRegister(3); // Reads 3 bits from the register
// 	uint8_t cycle = reg.CONFIGURATION_FIELDS.CONV;
// 	writeRegisters(cycle, 0b000, 3); // Sets the conversion cycle time to be between 15.5ms and 1s
// 	// There is a chart of the conversion cycle times in the SparkFun_TMP117_Registers.h file
// 	// They can also be found in Table 7 on Page 26 of the datasheet
// }

