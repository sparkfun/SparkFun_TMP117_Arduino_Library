/******************************************************************************
SparkFun_TMP117_Registers.h
TMP117 Library - TMP117 Register Map
Madison Chodikov @ SparkFun Electronics
Original Creation Date: April 19, 2019
https://github.com/sparkfunX/Qwiic_TMP117

This file defines all registers internal to the TMP117 sensor.

Development environment specifics:
	IDE: Arduino 1.8.9
	Hardware Platform: Arduino Uno
	TMP117 Breakout Version: 1.0

This code is beerware; if you see me (or any other SparkFun employee) at the
local, and you've found our code helpful, please buy us a round!

Distributed as-is; no warranty is given.
******************************************************************************/

#ifndef __SparkFun_TMP117_Registers_H__
#define __SparkFun_TMP117_Registers_H__
/*
  TMP117 Registers as defined in Table 3 from datasheet (pg 24)
  Features of the TMP117:
   - ±0.1°C (Maximum) From –20°C to +50°C
   - ±0.15°C (Maximum) From –40°C to +70°C
   - ±0.2°C (Maximum) From –40°C to +100°C
   - ±0.25°C (Maximum) From –55°C to +125°C
   - ±0.3°C (Maximum) From –55°C to +150°C
   -Low Power Consumption 3.5-µA, 1-Hz Conversion Cycle
*/

/*  Conversion Cycle Time in CC Mode (found on the datasheet page 26 table 6)
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

enum TMP117_Register
{
	TEMP_RESULT = 0X00,
	CONFIGURATION = 0x01,
	T_HIGH_LIMIT = 0X02,
	T_LOW_LIMIT = 0X03,
	EEPROM_UL = 0X04,
	EEPROM1 = 0X05,
	EEPROM2 = 0X06,
	TEMP_OFFSET = 0X07,
	EEPROM3 = 0X08,
	DEVICE_ID = 0X0F
};


#endif