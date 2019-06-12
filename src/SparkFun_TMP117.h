/******************************************************************************
SparkFunTMP117.h
SparkFunTMP117 Library Header File
Madison Chodikov @ SparkFun Electronics
Original Creation Date: April 29, 2016
https://github.com/sparkfunX/Qwiic_TMP117

This file prototypes the TMP102 class, implemented in SparkFunTMP117.cpp.

Development environment specifics:
	IDE: Arduino 1.8.9
	Hardware Platform: Arduino Uno
	TMP117 Breakout Version: 1.0

This code is beerware; if you see me (or any other SparkFun employee) at the
local, and you've found our code helpful, please buy us a round!

Distributed as-is; no warranty is given.
******************************************************************************/

#ifndef __SparkFun_TMP117_H__
#define __SparkFun_TMP117_H__

#include <Wire.h>
#include <Arduino.h>
#include "SparkFun_TMP117_Registers.h"

#define TMP117_I2C_ADDR 0x48 // Address found on Page 19 of data sheet (GND)

// Address found on page 23 Table 3 of the data sheet
#define DEVICE_ID_VALUE 0x0117

// Resolution of the device, found on page 1 of the data sheet
#define TMP117_RESOLUTION (double)0.0078125

enum TMP117_ALERT
{
	NOALERT = 0,
	HIGHALERT,
	LOWALERT
}; // Distinguishes the Alert type

// Configuration register found on page 25 Figure 26 and Table 6
typedef union {
	struct
	{
		uint8_t EMPTY : 1;		 // Empty bit in register
		uint8_t SOFT_RESET : 1;  // Software reset bit
		uint8_t DR_ALERT : 1;	// ALERT pin select bit
		uint8_t POL : 1;		 // ALERT pin polarity bit
		uint8_t T_NA : 1;		 // Therm/alert mode select
		uint8_t AVG : 2;		 // Conversion averaging modes
		uint8_t CONV : 3;		 // Conversion cycle bit
		uint8_t MOD : 2;		 // Set conversion mode
		uint8_t EEPROM_BUSY : 1; // EEPROM busy flag
		uint8_t DATA_READY : 1;  // Data ready flag
		uint8_t LOW_ALERT : 1;   // Low Alert flag
		uint8_t HIGH_ALERT : 1;  // High Alert flag
	} CONFIGURATION_FIELDS;
	uint8_t CONFIGURATION_COMBINED;
} CONFIGURATION_REG;

// Device ID Register used for checking if the device ID is the same as declared
// This register is found on Page 30 of the datasheet in Table 15 and Figure 34
typedef union {
	struct
	{
		uint16_t DID : 12; // Indicates the device ID
		uint8_t REV : 4;   // Indicates the revision number
	} DEVICE_ID_FIELDS;
	uint16_t DEVICE_ID_COMBINED;
} DEVICE_ID_REG;

class TMP117
{
public:
	// Constructor
	TMP117(byte address = TMP117_I2C_ADDR);

	bool begin(TwoWire &wirePort = Wire, uint8_t deviceAddress = TMP117_I2C_ADDR); //Initialize the TMP117 sensor at given address
	uint8_t getAddress();														   // Lets the user see the current address of the device
	void setAddress(uint8_t addr);												   // Lets the user set the address of the device
	bool begin_(uint8_t address, TwoWire &wirePort);							   // Initalizes sensor and opens registers
	bool isConnected();															   // Checks connection
	float readTempC();															   // Returns the temperature in degrees C
	float readTempF();															   // Converts readTempC result to degrees F
	float temperatureOffset();													   // Reads the offset temperature value from the register
	TMP117_ALERT getAlert();													   // Returns the type of alert being caused
	// bool isHighAlert();															   // Sets an alert when the temperature is too high for the device
	// bool isLowAlert();															   // Sets an alert when the temperature is too low for the device
	void softReset();						  // Performs a software reset on the Configuration Register Field bits
	float getTemperatureOffset();			  // Reads the temperature offset
	void setTemperatureOffset(uint16_t time); // Writes to the temperature offset
	// float getLowLimit();
	// void setLowLimit();
	// float getHighLimit();
	// void setHighLimit();
	uint8_t getConversionMode();				// Checks to see the Conversion Mode the device is currently in
	void setConversionMode(uint8_t cycle);		// Sets the Conversion Mode of the device (4 different types)
	uint8_t getConversionCycleTime();			// Read from the Conversion Cycle Time register
	void setConversionCycleTime(uint8_t cycle); // Write to the Conversion Cycle Time register
	bool dataReady();							// Checks to see if there is data ready from the device
												// uint16_t unsignedWriteRegister16(byte rawData[2]); // Register to simplify other functions with combining 16 bit numbers
												// int16_t signedWriteRegister16(byte rawData[2]); // Register to simplify other functions with combining 16 bit numbers

private:
	TwoWire *_i2cPort = NULL; //The generic connection to user's chosen I2C hardware
	uint8_t _address;		  // Address of Temperature sensor

	TMP117_ALERT alert_type;

	// Read and write to registers
	uint16_t readRegister(TMP117_Register reg);						  // Reads 2 register bytes from sensor
	void readRegisters(TMP117_Register reg, byte *buffer, byte len);  // Reads multiple bytes from a sensor
	void writeRegisters(TMP117_Register reg, byte *buffer, byte len); // Wires multiple bytes of data to the sensor
	void writeRegister(TMP117_Register reg, byte data);				  // Wires single byte of data to the sensor
};

#endif