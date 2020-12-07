////////////////////////////////////////////////////////////////////
//
//	AMX8XX Driver C Code
//
//  C Header file containing AMX8XX driver function prototypes
//	and defines
//
//	Copyright (C) 2012 Ambiq Micro Inc.
//
//  Version		Date		Release Notes
//  0.00		10/29/12	Initial release
//
//  File: amx8xx_v00.h
////////////////////////////////////////////////////////////////////

#ifndef __AMX8XX_H
#define __AMX8XX_H


#define AM18XX_I2C_ADDR     0xD2 // AM18XX I2C Address
#define AMX8XX_IF_I2C				     // Comment this line out for AMX8XX SPI device

typedef ErrorStatus STATUS_t;

typedef struct
{
	uint8_t hundredth;
	uint8_t second;
	uint8_t minute;
	uint8_t hour;
	uint8_t date;
	uint8_t weekday;
	uint8_t month;
	uint8_t year;
	uint8_t century;
	uint8_t mode;
} TIME_REG_STRUCT;

/*
typedef enum
{
    ERROR,
    SUCCESS
} STATUS_t;
*/

typedef enum
{
	SLEEP_SUCCESS,
	SLEEP_ERROR,
	SLEEP_INT_PENDING,
	SLEEP_NO_INT_ENABLED
} SLEEP_STATUS_t;


extern TIME_REG_STRUCT time_regs;


// AMX8XX Driver Function Prototypes ////////////////////////////////////////////////////

// Clear one or more bits in the selected register, selected by 1's in the mask
void clrreg(uint8_t address, uint8_t mask);

// Set one or more bits in the selected register, selected by 1's in the mask
void setreg(uint8_t address, uint8_t mask);

// Executes I2C or SPI commands required to read one byte of data at a particular address
uint8_t readreg(uint8_t address);

// Executes the I2C or SPI commands required to write one byte of data to a particular address
void writereg(uint8_t address, uint8_t data);

// Executes I2C or SPI commands required to implement a burst read of the data at a particular
// address of a particular length into a structure
void burstread(uint8_t address, uint8_t length, uint8_t * data);

// Executes I2C or SPI commands required to implement a burst write of the data in a structure to
// a particular address with a particular length
void burstwrite(uint8_t address, uint8_t length, uint8_t * data);

// Get the current time from the counter registers
void am_get_time(void);

// Sets the time in the counter registers
STATUS_t am_set_time(uint8_t protect);

// Configure the watch dog timer.
STATUS_t am_set_watchdog (uint32_t period, uint8_t pin);

// Configure sleep mode (AM18XX only).
SLEEP_STATUS_t am_set_sleep(uint8_t timeout, uint8_t mode);

// Configure the square wave output.
STATUS_t am_config_sqw(uint8_t sqfs, uint8_t pin);

// Select an oscillator mode.
STATUS_t am_osc_sel(uint8_t osc);

// Configure oscillator calibration mode.
STATUS_t am_set_calibration(uint8_t mode, int32_t adjust);

// Configure the alarm.
STATUS_t am_set_alarm(uint8_t repeat, uint8_t intmode, uint8_t pin);

// Configure the count-down function.
STATUS_t am_set_countdown(uint8_t range, int32_t period, uint8_t repeat, uint8_t pin);

// Set the repeat period for auto-calibration.
STATUS_t am_set_autocal(uint8_t period);

// Read a byte from local RAM
uint8_t am_read_ram(uint8_t address);

// Write a byte to local RAM
void am_write_ram(uint8_t address, uint8_t data);

// Get the RAM extension address
static uint8_t get_extension_address(uint8_t address);

// Support routines used by driver functions. /////////////////////////////////

// Converts a byte from decimal to binary coded decimal
uint8_t dec2bcd (uint8_t decno);

// Converts a byte from binary coded decimal to decimal
uint8_t bcd2dec(uint8_t bcdno);

// Checks a value against min/max limits. Returns ERROR if out of range.
STATUS_t check_param(int32_t param, int32_t min_param, int32_t max_param);

// Wait for a specified number of microseconds
//void waitms(uint32_t delay);

void am_init(void);
void PSWControl(uint8 state);
void RTCSetSleepTime(uint32 uiSlot, uint8 ucPin);
void am_clr_intflag(void);
void am_disable_alm(void);
#endif // __AMX8XX_H

