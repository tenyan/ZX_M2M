///////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	AMX8XX Driver C Code
//
//  C file containing AMX8XX driver functions
//
//	Copyright (C) 2012 Ambiq Micro Inc.
//
//  Version		Date		Release Notes
//  0.0			10/29/12	Initial release
//  0.1			01/28/13	Add option to enable autocalibration filter in am_set_autocal
//  0.2			04/04/13	Fixed bug in am_set_countdown function for RC mode period comparison values
//
//  File: amx8xx_v02.c
///////////////////////////////////////////////////////////////////////////////////////////////////////
#include<stdbool.h>
#include <stdio.h>
 #include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <malloc.h>
#include <sys/ioctl.h>
#include <string.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include "config.h"
#include "rtc.h"
#include "amx8xx.h"

#define HUNDREDTHS_REG      0x00
#define YEARS_REG			0x06
#define ALARM_HUNDRS_REG	0x08
#define STATUS_REG          0x0F
#define CONTROL_1_REG       0x10
#define CONTROL_2_REG		0x11
#define INT_MASK_REG		0x12
#define SQW_REG				0x13
#define CAL_XT_REG			0x14
#define CAL_RC_HI_REG		0x15
#define CAL_RC_LOW_REG		0x16
#define SLEEP_CTRL_REG		0x17
#define TIMER_CTRL_REG		0x18
#define TIMER_REG			0x19
#define TIMER_INITIAL_REG	0x1A
#define WDT_REG				0x1B
#define OSC_CONTROL_REG     0x1C
#define OSC_STATUS_REG		0x1D
#define CONFIG_KEY_REG      0x1F
#define ACAL_FLT_REG     	0x26
#define EXTENDED_ADDR_REG   0x3F

static uint8_t get_extension_address(uint8_t address);
TIME_REG_STRUCT time_regs;
void PSWControl(uint8 state);

////////////////////////////////////////////////////////////////////
//
// am_get_time - get the current AMX8XX time in the counters
//
// time_regs - a TIME_REGS_STRUCT that contains the time to be set
//
// This function loads the time_regs structure with the time
// from the AMX8XX
//
////////////////////////////////////////////////////////////////////
//SCL--PA8     SDA--PC9   I2C3

void am_init(void)
{
	Am_I2C_Config();
	am_osc_sel(0);
	PSWControl(0);
	clrreg(INT_MASK_REG,0x04);//Disable ALE
	clrreg(STATUS_REG, 0x7F);//clr interrupter flag
	setreg(CONTROL_1_REG, 0x12);//setreg(CONTROL_1_REG, 0x02);
}

////////////////////////////////////////////////////////////////////
//
// am_get_time - get the current AMX8XX time in the counters
//
// time_regs - a TIME_REGS_STRUCT that contains the time to be set
//
// This function loads the time_regs structure with the time
// from the AMX8XX
//
////////////////////////////////////////////////////////////////////

void am_get_time(void)
{
    uint8_t temp_buff[8];

    // Read the counters.
    burstread(HUNDREDTHS_REG, 8, temp_buff);

    time_regs.hundredth = bcd2dec(temp_buff[0]);
    time_regs.second = bcd2dec(temp_buff[1]);
    time_regs.minute = bcd2dec(temp_buff[2]);
    time_regs.hour = temp_buff[3];
    time_regs.date = bcd2dec(temp_buff[4]);
    time_regs.month = bcd2dec(temp_buff[5]);
    time_regs.year = bcd2dec(temp_buff[6]);
    time_regs.weekday = bcd2dec(temp_buff[7]);

    // Get the current hours format mode 12:24.
    temp_buff[0] = readreg(CONTROL_1_REG);
    if ((temp_buff[0] & 0x40) == 0)
    {
        // 24-hour mode.
        time_regs.mode = 2;
        time_regs.hour = time_regs.hour & 0x3F;           // Get tens:ones
    }
    else
    {
        // 12-hour mode.  Get PM:AM.
        time_regs.mode = (time_regs.hour & 0x20) ? 1 : 0;  // PM : AM
        time_regs.hour &= 0x1F;                            // Get tens:ones
    }
    time_regs.hour = bcd2dec(time_regs.hour);

    // Get the century bit.
    temp_buff[0] = readreg(STATUS_REG);
    time_regs.century = (temp_buff[0] & 0x80) ? 1 : 0;
}

void am_get_time_beijin(uint8_t* year, uint8_t* month, uint8_t* day, uint8_t* hour, uint8_t* min, uint8_t* sec)
{
	am_get_time();
	*year = time_regs.year;
	*month  = time_regs.month;
	*day  = time_regs.date;;
	*hour = time_regs.hour;
	*min  = time_regs.minute;
	*sec  = time_regs.second;
}
/////////////////////////////////////////////////////////////////////////
//
// am_set_time - set the time in the counters
//
// Inputs:
// protect - 0 => leave counters writable, 1 => leave counters unwritable
//
// time_regs - a time_reg_struct that contains the time to be set
//
// This function loads the AMX8XX counter registers with the current
// time_regs structure values.
//
// Returns 1 if illegal inputs, 0 otherwise
//
/////////////////////////////////////////////////////////////////////////

STATUS_t am_set_time(uint8_t protect)
{
	STATUS_t status = ERROR;
    uint8_t temp_buff[8];

	// First check the time parameters to ensure that they are within limits
// ERRCHK    if (check_param((int32_t)time_regs.hundredth, 0, 99) == ERROR) goto EXIT;
// ERRCHK    if (check_param((int32_t)time_regs.second, 0, 59) == ERROR) goto EXIT;
// ERRCHK    if (check_param((int32_t)time_regs.minute, 0, 59) == ERROR) goto EXIT;
// ERRCHK    if (check_param((int32_t)time_regs.weekday, 0, 6) == ERROR) goto EXIT;
// ERRCHK    if (check_param((int32_t)time_regs.month, 1, 12) == ERROR) goto EXIT;
// ERRCHK    if (check_param((int32_t)time_regs.year, 0, 99) == ERROR) goto EXIT;
// ERRCHK    if (check_param((int32_t)time_regs.century, 0, 1) == ERROR) goto EXIT;
// ERRCHK    if (check_param((int32_t)time_regs.mode, 0, 2) == ERROR) goto EXIT;

    //	 Make sure the date value is correct based on the month and year
// ERRCHK	if (time_regs.month == 2)
// ERRCHK	{
		// Month is February - check for leap year
// ERRCHK		if (is_leap_year(time_regs.year, time_regs.century))
// ERRCHK		{
// ERRCHK			if (check_param((int32_t)time_regs.date, 1, 29) == ERROR) goto EXIT;
// ERRCHK		}
// ERRCHK		else
// ERRCHK		{
// ERRCHK			if (check_param((int32_t)time_regs.date, 1, 28) == ERROR) goto EXIT;
// ERRCHK		}
// ERRCHK	}
// ERRCHK	else
// ERRCHK	{
		// Check dates for all other months.
// ERRCHK		if (check_param((int32_t)time_regs.date, 1, DAYS_OF_MONTH[time_regs.month]) == ERROR) goto EXIT;
// ERRCHK	}

	// Make sure the hours parameter is correct based on the mode
// ERRCHK	if (time_regs.mode == 2)
// ERRCHK	{
// ERRCHK		// 24 hour mode
// ERRCHK		if (check_param((int32_t)time_regs.hour, 0, 23) == ERROR) goto EXIT;
// ERRCHK	}
// ERRCHK	else
// ERRCHK	{
    // 12 hour mode
// ERRCHK		if (check_param((int32_t)time_regs.hour, 1, 12) == ERROR) goto EXIT;
// ERRCHK	}

    //Convert decimal to binary-coded decimal
    time_regs.hundredth = dec2bcd(time_regs.hundredth);
    time_regs.second = dec2bcd(time_regs.second);
    time_regs.minute = dec2bcd(time_regs.minute);
    time_regs.hour = dec2bcd(time_regs.hour);
    time_regs.date = dec2bcd(time_regs.date);
    time_regs.weekday = dec2bcd(time_regs.weekday);
    time_regs.month = dec2bcd(time_regs.month);
    time_regs.year = dec2bcd(time_regs.year);

    // Determine whether 12 or 24-hour timekeeping mode is being used
    // and set the 1224 bit appropriately
    if (time_regs.mode == 2)        // 24-hour day
    {
        clrreg(CONTROL_1_REG, 0x40);
    }
    else if (time_regs.mode == 1)   // 12-hour day PM
    {
        time_regs.hour |= 0x20;	    // Set AM/PM
        setreg(CONTROL_1_REG, 0x40);
    }
    else                            // 12-hour day AM
    {
        setreg(CONTROL_1_REG, 0x40);
    }

    // Set the WRTC bit to enable counter writes.
    setreg(CONTROL_1_REG, 0x01);

    // Set the correct century
    if (time_regs.century == 0)
    {
        clrreg(STATUS_REG, 0x80);
    }
    else
    {
        setreg(STATUS_REG, 0x80);
    }

    // Write all of the time counters
    temp_buff[0] = time_regs.hundredth;
    temp_buff[1] = time_regs.second;
    temp_buff[2] = time_regs.minute;
    temp_buff[3] = time_regs.hour;
    temp_buff[4] = time_regs.date;
    temp_buff[5] = time_regs.month;
    temp_buff[6] = time_regs.year;
    temp_buff[7] = time_regs.weekday;

    // Write the values to the AM18XX
    burstwrite(HUNDREDTHS_REG, 8, temp_buff);

    // Load the final value of the WRTC bit based on the value of protect
    temp_buff[0] = readreg(CONTROL_1_REG);
    temp_buff[0] &= 0x7E;                   // Clear the WRTC bit and the STOP bit
    temp_buff[0] |= (0x01 & (~protect));    // Invert the protect bit and update WRTC
    writereg(CONTROL_1_REG, temp_buff[0]);

	//Execution has succeeded
	status = SUCCESS;

// ERRCHKEXIT:
	return status;
}


void am_set_time_beijin(uint8_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec )
{
    if(year<19||(month<1||month>12)||(day<1||day>31)||hour>23||min>59||sec>59)  //Ê±¼äÒì³£ ÍË³ö
		return;
	time_regs.year   = year;
	time_regs.month  = month;
	time_regs.date   = day;
	time_regs.hour   = hour;
	time_regs.minute = min;
	time_regs.second = sec;
	am_set_time(1);
}

/////////////////////////////////////////////////////////////////////////////////////
//
// am_set_calibration(mode, adjust);
//
// mode -
//		0 => calibrate the XT oscillator
//		1 => calibrate the RC oscillator
// adjust - adjustment in ppm.  Adjustment limits are:
// 		mode = 0 => (-610 to +242)
// 		mode = 1 => (-65536 to +65520)
// 		An adjust value of zero resets the selected oscillator calibration value to 0
// error - returned value, 0 => adjustment successful, 1 => adjustment too large
//
/////////////////////////////////////////////////////////////////////////////////////

STATUS_t am_set_calibration(uint8_t mode, int32_t adjust)
{
	STATUS_t status = ERROR;
//	double adjval;
	int32_t adjint;
	uint8_t adjreg;
	uint8_t adjregu;
	uint8_t xtcal;

	// Check the input parameters to ensure that they are within limits
// ERRCHK	if (check_param((uint16_t)mode, 0, 1) == ERROR) goto EXIT;
// ERRCHK	if (mode == 0)
// ERRCHK	{
// ERRCHK		if (check_param(adjust, -610, 242) == ERROR) goto EXIT;
// ERRCHK	}
// ERRCHK	else
// ERRCHK	{
// ERRCHK		if (check_param(adjust, -124975,124975) == ERROR) goto EXIT;
// ERRCHK	}

	// Calculate current calibration value
//	adjval = (double)adjust;
//	adjint = (int16_t)round(adjval*1000/1907);
	if (adjust < 0 )
	{
	adjint = ((adjust)*1000 - 953);
	}
	else
	{
	adjint = ((adjust)*1000 + 953);
	}
	adjint = adjint/1907;

	if (mode == 0)
	{
		// XT adjust
		if (adjint > 63 )
		{
			// 64 to 127
			xtcal = 0;
			adjreg = ((adjint >> 1) & 0x3F) | 0x80;	// CMDX = 1
		}
		else if (adjint > -65)
		{
			// -64 to 63
			xtcal = 0;
			adjreg = (adjint & 0x7F);				// CMDX = 0
		}
		else if (adjint > -129)
		{
			// -128 to -65
			xtcal = 1;
			adjreg = ((adjint + 64) & 0x7F);		// CMDX = 0
		}
		else if (adjint > -193)
		{
			// -192 to -129
			xtcal = 2;
			adjreg = ((adjint + 128) & 0x7F);		// CMDX = 0
		}
		else if (adjint > -257)
		{
			// -256 to -193
			xtcal = 3;
			adjreg = ((adjint + 192) & 0x7F);		// CMDX = 0
		}
		else
		{
			// -320 to -257
			xtcal = 3;
			adjreg = ((adjint + 192) >> 1) & 0xFF;	// CMDX = 1
		}

		writereg(CAL_XT_REG, adjreg);				// Load the CALX register
		adjreg = readreg(OSC_STATUS_REG) & 0x3F;	// Mask XTCAL
		adjreg = adjreg | (xtcal << 6);				// Add XTCAL field
		writereg(OSC_STATUS_REG, adjreg);			// Write back
	}
	else
	{
		// RC adjust
		if (adjint > 32767 )
		{
			// 32768 to 65535
			adjreg = ((adjint >> 3) & 0xFF);		// Lower 8 bits
			adjregu = ((adjint >> 11) | 0xC0);		// CMDR = 3
		}
		else if (adjint > 16383 )
		{
			// 16384 to 32767
			adjreg = ((adjint >> 2) & 0xFF);		// Lower 8 bits
			adjregu = ((adjint >> 10) | 0x80);		// CMDR = 2
		}
		else if (adjint > 8191 )
		{
			// 8192 to 16383
			adjreg = ((adjint >> 1) & 0xFF);		// Lower 8 bits
			adjregu = ((adjint >> 9) | 0x40);		// CMDR = 2
		}
		else if (adjint >= 0 )
		{
			// 0 to 1023
			adjreg = ((adjint) & 0xFF);				// Lower 8 bits
			adjregu = (adjint >> 8);				// CMDR = 0
		}
		else if (adjint > -8193 )
		{
			// -8192 to -1
			adjreg = ((adjint) & 0xFF);				// Lower 8 bits
			adjregu = (adjint >> 8) & 0x3F;			// CMDR = 0
		}
		else if (adjint > -16385 )
		{
			// -16384 to -8193
			adjreg = ((adjint >> 1) & 0xFF);		// Lower 8 bits
			adjregu = (adjint >> 9) & 0x7F;			// CMDR = 1
		}
		else if (adjint > -32769 )
		{
			// -32768 to -16385
			adjreg = ((adjint >> 2) & 0xFF);		// Lower 8 bits
			adjregu = (adjint >> 10) & 0xBF;		// CMDR = 2
		}
		else
		{
			// -65536 to -32769
			adjreg = ((adjint >> 3) & 0xFF);		// Lower 8 bits
			adjregu = (adjint >> 11) & 0xFF;		// CMDR = 3
		}

		writereg(CAL_RC_HI_REG, adjregu);			// Load the CALRU register
		writereg(CAL_RC_LOW_REG, adjreg);			// Load the CALRL register
	}

	//Execution has succeeded
	status = SUCCESS;

	// ERRCHKEXIT:
	return status;
}

////////////////////////////////////////////////////////////////////
//
// am_set_alarm - set the alarm value
//
// Inputs:
//	repeat - the alarm repeat interval
//		0 => disable alarm
//		1 => once per year
//		2 => once per month
//		3 => once per week
//		4 => once per day
//		5 => once per hour
//		6 => once per minute
//		7 => once per second
//		8 => once per 10th of a second
//		9 => once per 100th of a second
//		NOTE: year and century are not used
//		NOTE: mode must match current 12/24 selection
//	intmode - define the interrupt mode
//		0 => level interrupt
//		1 => pulse of 1/8192s (XT) or 1/128 s (RC)
//		2 => pulse of 1/64 s
//		3 => pulse of 1/4 s
//	pin - pin on which to generate the interrupt
//		0 => internal flag only
//		1 => FOUT/nIRQ
//		2 => PSW/nIRQ2
//
//	time_regs - a time_reg_struct that contains the time to be set
//
// Returns 1 if illegal inputs, 0 otherwise
//
//////////////////////////////////////////////////////////////////////

STATUS_t am_set_alarm(uint8_t repeat, uint8_t intmode, uint8_t pin)
{

	STATUS_t status = ERROR;
// ERRCHK	uint8_t month, year, century;
	uint8_t temp;
    uint8_t temp_buff[8];
// ERRCHK    uint8_t h1224_bit;

	// First check the time parameters to ensure that they are within limits
// ERRCHK    if (check_param((int32_t)repeat, 0, 9) == ERROR) goto EXIT;
// ERRCHK    if (check_param((int32_t)pin, 0, 2) == ERROR) goto EXIT;
// ERRCHK    if (check_param((int32_t)intmode, 0, 3) == ERROR) goto EXIT;
// ERRCHK    if (check_param((int32_t)time_regs.mode, 0, 2) == ERROR) goto EXIT;

// ERRCHK	temp = readreg(OSC_STATUS_REG) & 0x10;		// Get OMODE bit
// ERRCHK	if (temp == 0)
// ERRCHK	{
		// XT mode
// ERRCHK		if (check_param((int32_t)time_regs.hundredth, 0, 99) == ERROR) goto EXIT;
// ERRCHK	}
// ERRCHK	else
// ERRCHK	{
// ERRCHK		// RC mode
// ERRCHK		if (repeat > 7) goto EXIT;				// No hundredth
// ERRCHK	}
// ERRCHK	if (repeat < 7)
// ERRCHK	{
		// Check seconds
// ERRCHK		if (check_param((int32_t)time_regs.second, 0, 59) == ERROR) goto EXIT;
// ERRCHK	}
// ERRCHK	if (repeat < 6)
// ERRCHK	{
		// Check minutes
// ERRCHK		if (check_param((int32_t)time_regs.minute, 0, 59) == ERROR) goto EXIT;
// ERRCHK	}

    // Make sure the hours parameter is correct based on the mode
// ERRCHK	h1224_bit = (readreg(CONTROL_1_REG) & 0x40) ? 1 : 0;
// ERRCHK	if (repeat < 5)
// ERRCHK	{
// ERRCHK		if (h1224_bit == 1)
// ERRCHK		{
			// 24 hour mode
// ERRCHK			if ((check_param(time_regs.hour, 0, 23) == ERROR) ||
// ERRCHK			    (time_regs.mode != 2)) goto EXIT;	// Must match
// ERRCHK		}
// ERRCHK		else
// ERRCHK		{
			// 12 hour mode
// ERRCHK			if ((check_param(time_regs.hour, 1, 12) == ERROR) ||
// ERRCHK			    (time_regs.mode == 2)) goto EXIT;	// Must match
// ERRCHK		}
// ERRCHK	}

	// Make sure the date value is correct based on the month and year
// ERRCHK	if (repeat = 1)											// Check date
// ERRCHK	{
		// Get current values
// ERRCHK		year = bcd2dec(readreg(YEARS_REG));			// Current year
// ERRCHK		century = (readreg(STATUS_REG) & 0x80) ? 1 : 0;	// Current century
// ERRCHK		month = time_regs.month;
// ERRCHK		if ((month < 1) || (month > 12)) goto EXIT;
// ERRCHK		if (month == 2)
// ERRCHK		{
// ERRCHK			// Month is February - check for leap year
// ERRCHK			if (is_leap_year(year, century))
// ERRCHK			{
// ERRCHK				if (check_param((int32_t)time_regs.date, 1, 29) == ERROR) goto EXIT;
// ERRCHK			}
// ERRCHK			else
// ERRCHK			{
// ERRCHK				if (check_param((int32_t)time_regs.date, 1, 28) == ERROR) goto EXIT;
// ERRCHK			}
// ERRCHK		}
// ERRCHK		else
// ERRCHK		{
// ERRCHK			// Check dates for all other months.
// ERRCHK			if (check_param((int32_t)time_regs.date, 1, DAYS_OF_MONTH[month]) == ERROR) goto EXIT;
// ERRCHK		}
// ERRCHK	}

// ERRCHK	if (repeat == 4)
// ERRCHK	{
		// Check weekday
// ERRCHK		if (check_param((int32_t)time_regs.weekday, 0, 6) == ERROR) goto EXIT;
// ERRCHK	}
// ERRCHK	if (repeat == 2)
// ERRCHK	{
// ERRCHK		// Check date (can still fail if date > 28)
// ERRCHK		if (check_param((int32_t)time_regs.date, 1, 31) == ERROR) goto EXIT;
// ERRCHK	}

	// Input is valid if we get here.
	// Next convert decimal to binary-coded decimal
	time_regs.hundredth = dec2bcd(time_regs.hundredth);
	time_regs.second = dec2bcd(time_regs.second);
	time_regs.minute = dec2bcd(time_regs.minute);
	time_regs.hour = dec2bcd(time_regs.hour);
	time_regs.date = dec2bcd(time_regs.date);
	time_regs.weekday = dec2bcd(time_regs.weekday);
	time_regs.month = dec2bcd(time_regs.month);

	// Determine whether a 12-hour or a 24-hour time keeping mode is being used
	if (time_regs.mode == 1)
	{
		// A 12-hour day PM
		time_regs.hour = time_regs.hour | 0x20;	  // Set AM/PM
	}

	// Write all of the time counters
	temp_buff[0] = time_regs.hundredth;
	temp_buff[1] = time_regs.second;
	temp_buff[2] = time_regs.minute;
	temp_buff[3] = time_regs.hour;
	temp_buff[4] = time_regs.date;
	temp_buff[5] = time_regs.month;
	temp_buff[6] = time_regs.weekday;

	clrreg(TIMER_CTRL_REG, 0x1C);			// Clear the RPT field
	clrreg(INT_MASK_REG, 0x64);				// Clear the AIE bit and IM field
	clrreg(STATUS_REG, 0x04);				// Clear the ALM flag

  	if (pin == 1)
	{
		// Interrupt on FOUT/nIRQ
		temp = readreg(CONTROL_2_REG);		// Get the Control2 Register
		temp = (temp & 0x03);				// Extract the OUT1S field
		if (temp != 0)						// Not already selecting nIRQ
		{
			setreg(CONTROL_2_REG, 0x03);	// Set OUT1S to 3
		}
	}
	if (pin == 2)
  	{
  		// Interrupt on PSW/nIRQ2
		temp = readreg(CONTROL_2_REG);		// Get the Control2 Register
		temp &= 0x1C;						// Extract the OUT2S field
		if (temp != 0)						// Not already selecting nIRQ
		{
			clrreg(CONTROL_2_REG, 0x1C);  	// Clear OUT2S
			setreg(CONTROL_2_REG, 0x0C);  	// Set OUT2S to 3
		}
	}

	if (repeat == 8)
	{
		// 10ths interrupt
		temp_buff[0] |= 0xF0;
		repeat = 7;						// Select correct RPT value
	}
	if (repeat == 9)
	{
		// 100ths interrupt
		temp_buff[0] = 0xFF;
		repeat = 7;						// Select correct RPT value
	}
	if (repeat != 0)								// Don't initiate if repeat = 0
	{

		temp = (repeat << 2);						// Set the RPT field to the value of repeat
		setreg(TIMER_CTRL_REG, temp);				// Was previously cleared
		setreg(INT_MASK_REG, (intmode << 5));		// Set the alarm interrupt mode
		burstwrite(ALARM_HUNDRS_REG, 7, temp_buff);	// Execute the burst write
		setreg(INT_MASK_REG, 0x04);					// Set the AIE bit
	}
	else
		setreg(INT_MASK_REG, 0x60);					// Set IM field to 0x3 (reset value) to minimize current draw

	//Execution has succeeded
	status = SUCCESS;

// ERRCHKEXIT:
	return status;
}

////////////////////////////////////////////////////////////////////////////////////////
//
// am_set_countdown(range, period, repeat,  pin);
//	range - 0 => period in us, 1 => period in seconds
//	period - the period of the countdown timer
//	repeat - configure the interrupt output type
//		0 => generate a single level interrupt
//		1 => generate a repeated pulsed interrupt, 1/4096 s (XT mode), 1/128 s (RC mode)
//				(range must be 0)
//		2 => generate a single pulsed interrupt, 1/4096 s (XT mode), 1/128 s (RC mode)
//				(range must be 0)
//		3 => generate a repeated pulsed interrupt, 1/128 s (range must be 0)
//		4 => generate a single pulsed interrupt, 1/128 s (range must be 0)
//		5 => generate a repeated pulsed interrupt, 1/64 s (range must be 1)
//		6 => generate a single pulsed interrupt, 1/64 s (range must be 1)
//	pin - select the pin to generate a countdown interrupt
//		0 => disable the countdown timer
//		1 => generate an interrupt on nTIRQ only, asserted low
//		2 => generate an interrupt on FOUT/nIRQ and nTIRQ, both asserted low
//		3 => generate an interrupt on PSW/nIRQ2 and nTIRQ, both asserted low
//		4 => generate an interrupt on CLKOUT/nIRQ3 and nTIRQ, both asserted low
//		5 => generate an interrupt on CLKOUT/nIRQ3 (asserted high) and nTIRQ (asserted low)
//
/////////////////////////////////////////////////////////////////////////////////////////

STATUS_t am_set_countdown(uint8_t range, int32_t period, uint8_t repeat, uint8_t pin)
{
	STATUS_t status = ERROR;
	uint8_t tm;
	uint8_t trpt;
	uint8_t tfs;
	uint8_t te;
	uint8_t temp;
	uint8_t tctrl;
	int32_t timer;
	uint8_t oscmode;

	// First check the time parameters to ensure that they are within limits
// ERRCHK	if (check_param((int32_t)repeat, 0, 6) == ERROR) goto EXIT;
// ERRCHK	if (check_param((int32_t)pin, 0, 5) == ERROR) goto EXIT;
// ERRCHK	if (check_param((int32_t)range, 0, 1) == ERROR) goto EXIT;

	oscmode = (readreg(OSC_STATUS_REG) & 0x10) ? 1 : 0;				// 0 = XT, 1 = RC

// ERRCHK	if ((repeat == 0) & (range == 0) & (oscmode == 0))
// ERRCHK	{
		// Microseconds, XT Mode
// ERRCHK		if (check_param((int32_t)period, 244, 2000000000) == ERROR) goto EXIT;
// ERRCHK	}
// ERRCHK	else if ((repeat == 0) & (range == 0) & (oscmode == 1))
// ERRCHK	{
		// Microseconds, RC Mode
// ERRCHK		if (check_param((int32_t)period, 7812, 2000000000) == ERROR) goto EXIT;
// ERRCHK	}
// ERRCHK	else if ((repeat == 0) & (range == 1))
// ERRCHK	{
		// Seconds
// ERRCHK		if (check_param((int32_t)period, 1, 15360) == ERROR) goto EXIT;
// ERRCHK	}
// ERRCHK	else if ((repeat < 3) & (range == 0) & (oscmode == 0))
// ERRCHK	{
		// Microseconds, XT mode
// ERRCHK		if (check_param((int32_t)period, 244, 62500) == ERROR) goto EXIT;
// ERRCHK	}
// ERRCHK	else if ((repeat < 3) & (range == 0) & (oscmode == 1))
// ERRCHK	{
// ERRCHK		// Microseconds, RC mode
// ERRCHK		if (check_param((int32_t)period, 7812, 2000000000) == ERROR) goto EXIT;
// ERRCHK	}
// ERRCHK	else if ((repeat < 3) & (range == 1))
// ERRCHK	{
// ERRCHK		goto EXIT; // Error
// ERRCHK	}
// ERRCHK	else if ((repeat < 5) & (range == 0))
// ERRCHK	{
		// Microseconds
// ERRCHK		if (check_param((int32_t)period, 7812, 2000000000) == ERROR) goto EXIT;
// ERRCHK	}
// ERRCHK	else if ((repeat < 5) & (range == 1))
// ERRCHK	{
		// Seconds
// ERRCHK		goto EXIT;  // Error
// ERRCHK	}
// ERRCHK	else if (range == 0)
// ERRCHK	{
		// Microseconds
// ERRCHK		goto EXIT;  // Error
// ERRCHK	}
// ERRCHK	else
// ERRCHK	{
		// Seconds
// ERRCHK		if (check_param((int32_t)period, 1, 15360) == ERROR) goto EXIT;
// ERRCHK	}

	if (pin == 0)
	{
		te = 0;
	}
	else
	{
		te = 1;
		if (repeat == 0)
		{
			// Level interrupt
			tm = 1;		 				// Level
			trpt = 0;					// No repeat
			if (range == 0)
			{
				// Microseconds
				if (oscmode == 0)
				{
					// XT Mode
					if (period <= 62500)				// Use 4K Hz
					{
						tfs = 0;
						timer = (period * 4096);
						timer = timer / 1000000;
						timer = timer - 1;
					}
					else if (period <= 16384000)		// Use 64 Hz
					{
						tfs = 1;
						timer = (period * 64);
						timer /= 1000000;
						timer = timer - 1;
					}
					else								// Use 1 Hz
					{
						tfs = 2;
						timer = period / 1000000;
						timer = timer - 1;
					}
				}
				else
				{
					// RC Mode
					if (period <= 2000000) {			// Use 128 Hz
						tfs = 0;
						timer = (period * 128);
						timer /= 1000000;
						timer = timer - 1;
					}
					else if (period <= 4000000) {		// Use 64 Hz
						tfs = 1;
						timer = (period * 64);
						timer /= 1000000;
						timer = timer - 1;
					}
					else {								// Use 1 Hz
						tfs = 2;
						timer = period / 1000000;
						timer = timer - 1;
					}
				}
			}
			else
			{
				// Seconds
				if (period <= 256)
				{
					// Use 1 Hz
					tfs = 2;
					timer = period - 1;
				}
				else
				{
					// Use 1/60 Hz
					tfs = 3;
					timer = period / 60;
					timer = timer - 1;
				}
			}
		}
		else
		{
			// Pulse interrupts
			tm = 0;					// Pulse
			trpt = repeat & 0x01;	// Set up repeat
			if (repeat < 3)
			{
				tfs = 0;
				if (oscmode == 0)
				{
						timer = (period * 4096);
						timer /= 1000000;
						timer = timer - 1;
				}
				else
				{
						timer = (period * 128);
						timer /= 1000000;
						timer = timer - 1;
				}
			}
			else if (repeat < 5)
			{
				tfs = 1;
				timer = (period * 128);
				timer /= 1000000;
				timer = timer - 1;
			}
			else if (period <= 256)
			{
				// Use 1 Hz
				tfs = 2;
				timer = period - 1;
			}
			else
			{
				// Use 1/60 Hz
				tfs = 3;
				timer = period / 60;
				timer = timer - 1;
			}
		}
	}

	tctrl = readreg(TIMER_CTRL_REG) & 0x1C;					// Get TCTRL, keep RPT, clear TE
	writereg(TIMER_CTRL_REG, tctrl);
	tctrl = tctrl | (te * 0x80) | (tm * 0x40) | (trpt * 0x20) | tfs;	// Merge the fields
	if (pin == 2)											// generate nTIRQ interrupt on FOUT/nIRQ (asserted low)
	{
		 clrreg(CONTROL_2_REG, 0x3);						// Clear OUT1S
	}
	if (pin == 3)											// generate nTIRQ interrupt on PSW/nIRQ2 (asserted low)
	{
		 temp = readreg(CONTROL_2_REG);						// Get OUT2S
		 if ((temp & 0x1C) != 0)
		 {
			 temp = (temp & 0xE3) | 0x14; 					// If OUT2S != 0, set OUT2S to 5
		 }
		 writereg(CONTROL_2_REG, temp);						// Write back
	}
	if (pin == 4)											// generate TIRQ interrupt on CLKOUT/nIRQ3 (asserted low)
	{
		writereg(SQW_REG, 0x9B);							// setup SQFS field and enable SQWE
	}
	if (pin == 5)											// generate TIRQ interrupt on CLKOUT/nIRQ3 (asserted high)
	{
		writereg(SQW_REG, 0x9A);							// setup SQFS field and enable SQWE
	}
	if (pin != 0)
	{
		clrreg(STATUS_REG,0x08);							// Clear TIM
		setreg(INT_MASK_REG,0x08);							// Set TIE
		writereg(TIMER_REG, timer);							// Initialize the timer
		writereg(TIMER_INITIAL_REG, timer);					// Initialize the timer repeat
		writereg(TIMER_CTRL_REG, tctrl);					// Start the timer
	}

	//Execution has succeeded
	status = SUCCESS;

// ERRCHKEXIT:
	return status;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
//
// am_osc_sel - select an oscillator mode
//
//	osc - the oscillator to select
//		0 => 32 KHz XT oscillator, no automatic oscillator switching
//		1 => 32 KHz XT oscillator, automatic oscillator switching to RC on switch to battery power
//		2 => 128 Hz RC oscillator
//
//////////////////////////////////////////////////////////////////////////////////////////////////

STATUS_t am_osc_sel(uint8_t osc)
{
	STATUS_t status = ERROR;
	uint8_t i;
	uint8_t temp;

	// Check the input parameter to ensure it is within limits
// ERRCHK	if (check_param((int32_t)osc, 0, 2) == ERROR) goto EXIT;

	// Read Oscillator Control register
	temp = readreg(OSC_CONTROL_REG);
	temp = temp & 0x67; 			    		// Clear OSEL, FOS, AOS

	// Enable Oscillator Register writes
	writereg(CONFIG_KEY_REG,0xA1);			    // Write the Key register

	switch (osc)
    {
		case 0:                                 // Do nothing, clear Key register
            writereg(OSC_CONTROL_REG,temp);
            break;

		case 1:
			temp = temp | 0x10;					// Set AOS
            writereg(OSC_CONTROL_REG,temp);
            break;

		default:
			temp = temp | 0x80;					// Set OSEL
            writereg(OSC_CONTROL_REG,temp);
            break;
	}

	// Wait to make sure switch occurred by testing OMODE

	for (i = 0; i < 100; i++)
	{
		//__delay_cycles(100000);					// Wait 100 ms
		//OSTimeDly(OS_TICKS_PER_SEC/100);
		usleep(1000*100);
		temp = readreg(OSC_STATUS_REG);			// Read OMODE
		temp = (temp & 0x10) >> 4;
		if (temp == (osc >> 1)) goto EXIT1;		// Successful switch
	}
	status = ERROR;
	goto EXIT;

	//Execution has succeeded
EXIT1:
	status = SUCCESS;

EXIT:
	return status;
}

////////////////////////////////////////////////////////////////
//
// am_config_sqw - configure the square wave output
//
// Inputs:
//	sqfs - square wave output select (0 to 31)
//	pin - output pin for SQW (may be ORed) in addition to CLKOUT
//		0 => disable SQW
//		1 => FOUT
//		2 => PSW/nIRQ2
//
////////////////////////////////////////////////////////////////

STATUS_t am_config_sqw(uint8_t sqfs, uint8_t pin)
{
	STATUS_t status = ERROR;
	uint8_t temp;

	// First check the input parameters to ensure that they are within limits
// ERRCHK	if (check_param((int32_t)sqfs, 0, 31) == ERROR) goto EXIT;
// ERRCHK	if (check_param((int32_t)pin, 0, 2) == ERROR) goto EXIT;

	// Set up SQW multiplexor
	temp = readreg(SQW_REG);				// Read the SQW register
	temp = (temp & 0x70) | sqfs | 0x80;		// Load SQFS, set SQWE
	if (pin == 0)
	{
		// Clear SQWE
		temp &= 0x7F;
	}

	if (pin & 0x1)
	{
		// Enable FOUT
		clrreg(CONTROL_2_REG,0x03);			// Clear OUT1S
		setreg(CONTROL_2_REG,0x01);			// Load OUT1S with 1
	}
	if (pin & 0x2)
	{
		// Enable PSW/nIRQ2
		clrreg(CONTROL_2_REG,0x1C);			// Clear OUT2S
		setreg(CONTROL_2_REG,0x04);			// Load OUT2S with 1
	}

	// Write the SQW register
	writereg(SQW_REG, temp);

	//Execution has succeeded
	status = SUCCESS;

// ERRCHKEXIT:
	return status;
}

////////////////////////////////////////////////////////////////////////////////////
//
// am_set_sleep - set up sleep mode (AM18xx only)
//
// Inputs:
//	timeout - minimum timeout period in 7.8 ms periods (0 to 7)
//	mode - sleep mode (nRST modes not available in AM08xx)
//		0 => nRST is pulled low in sleep mode
//		1 => PSW/nIRQ2 is pulled high on a sleep
//		2 => nRST pulled low and PSW/nIRQ2 pulled high on sleep
//	error – returned value of the attempted sleep command
//		0 => sleep request accepted, sleep mode will be initiated in timeout seconds
//		1 => illegal input values
//		2 => sleep request declined, interrupt is currently pending
//		3 => sleep request declined, no sleep trigger interrupt enabled
//
////////////////////////////////////////////////////////////////////////////////////

SLEEP_STATUS_t am_set_sleep(uint8_t timeout, uint8_t mode)
{
	uint8_t slres;
	uint8_t temp, temp1;
	SLEEP_STATUS_t status;

	// First check the input parameters to ensure that they are within limits
// ERRCHK	if ((check_param((int32_t)timeout, 0, 7) == ERROR) ||
// ERRCHK	   (check_param((int32_t)mode, 0, 2) == ERROR))
// ERRCHK		{
// ERRCHK			status = SLEEP_ERROR;
// ERRCHK			goto EXIT;
// ERRCHK}

	if (mode > 0)
	{
		// Sleep to PSW/nIRQ2
		temp = readreg(CONTROL_2_REG);		// Read OUT2S
		temp = (temp & 0xE3) | 0x18;		// MUST NOT WRITE OUT2S WITH 000
		writereg(CONTROL_2_REG, temp);		// Write value to OUT2S
		slres = 0;
	}

	if (mode != 1)
	{
		// Sleep to nRST
		slres = 1;
	}

	temp = timeout | (slres << 6) | 0x80;	// Assemble SLEEP register value
	writereg(SLEEP_CTRL_REG, temp);			// Write to the register

	// Determine if SLEEP was accepted
	temp = readreg(SLEEP_CTRL_REG) & 0x80;	// Get SLP bit
	if (temp == 0)
	{
		// SLEEP did not happen - determine why and return reason.
		temp = readreg(INT_MASK_REG) & 0x0F;	// Get status register interrupt enables
		temp1 = readreg(WDT_REG);				// Get WDT register
		if ((temp == 0) & (((temp1 & 0x7C) == 0) || ((temp1 & 0x80) == 0x80)))
		{
			status = SLEEP_NO_INT_ENABLED;	// No trigger interrupts enabled
		}
		else
		{
			status = SLEEP_INT_PENDING;		// Interrupt pending
		}
	}
	else
	{
		status = SLEEP_SUCCESS;				// SLEEP request successful
	}

// ERRCHKEXIT:
	return status;
}

////////////////////////////////////////////////////////////////////
//
// am_set_watchdog - set up the watchdog timer
//
// Inputs:
//	period - timeout period in ms (65 to 124,000)
//	pin - pin to generate the watchdog signal
//		0 => disable WDT
//		1 => generate an interrupt on FOUT/nIRQ
//		2 => generate an interrupt on PSW/nIRQ2
//		3 => generate a reset on nRST (AM18xx only)
//
// Returns 1 if illegal inputs, 0 otherwise
//
////////////////////////////////////////////////////////////////////

STATUS_t am_set_watchdog (uint32_t period, uint8_t pin)
{
	STATUS_t status = ERROR;
	uint8_t WDTreg;
	uint8_t wds;
	uint8_t bmb;
	uint8_t wrb;

	// First check the input parameters to ensure that they are within limits
// ERRCHK	if (check_param((int32_t)pin, 0, 3) == ERROR) goto EXIT;
// ERRCHK	if (pin != 0)
// ERRCHK	if (check_param((int32_t)period, 65, 124000) == ERROR) goto EXIT;

	writereg(WDT_REG, 0x00);			// Disable the WDT with BMB = 0
	clrreg(STATUS_REG, 0x20);			// Clear the WDT flag

	// Use the shortest clock interval which will allow the selected period
	if (period < (31000 / 16))
	{
		wrb = 0;						// Use 16 Hz
		bmb = (period * 16) / 1000;
	}
	else if (period < (31000 / 4))
	{
		wrb = 1;						// Use 4 Hz
		bmb = (period * 4) / 1000;
	}
	else if (period < 31000)
	{
		wrb = 2;						// Use 1 Hz
		bmb = period / 1000;
	}
	else
	{
		wrb = 3;						// Use 1/4 Hz
		bmb = period / 4000;
	}

	switch (pin)
	{
		case 0:							 // Disable WDT
			wds = 0;
			bmb = 0;
			break;
		case 1: 						 // Interrupt on FOUT/nIRQ
			wds = 0;					 // Select interrupt
			clrreg(CONTROL_2_REG, 0x03); // Clear the OUT1S field
			break;
		case 2:							 // Interrupt on PSW/nIRQ2
			wds = 0;					 // Select interrupt
			clrreg(CONTROL_2_REG, 0x1C); // Clear the OUT2S field
			break;
		case 3: 						 // Interrupt on nRST
		default:
			wds = 1;					 // Select reset out
			break;
	}

	WDTreg = (wds * 0x80) + (bmb * 0x4) + wrb;		// Create the correct value
	writereg(WDT_REG, WDTreg);						// Write the register

	//Execution has succeeded
	status = SUCCESS;

// ERRCHKEXIT:
	return status;
}

/////////////////////////////////////////////////////////////
//
// am_set_autocal(period);
//
//	period - the repeat period for autocalibration.
//		0 => disable autocalibration
//		1 => execute a single autocalibration cycle
//		2 => execute a cycle every 1024 seconds (~17 minutes)
//		3 => execute a cycle every 512 seconds (~8.5 minutes)
//
/////////////////////////////////////////////////////////////

STATUS_t am_set_autocal(uint8_t period)
{
	STATUS_t status = ERROR;
    uint8_t temp;

// ERRCHK    if (check_param((int32_t)period, 0, 3) == ERROR) goto EXIT;

/* Optional code to enable autocalibration filter capacitor
	writereg(CONFIG_KEY_REG,0x9D);		    			// Load the Key register
	if (period == 0)									// If autocalibration is disabled or not used
	{
	writereg(ACAL_FLT_REG, 0x00);						// Disable the autocalibration filter
	}
	else
	{
	writereg(ACAL_FLT_REG, 0xA0);						// Enable the optional autocalibration filter capacitor
	}
*/

	temp = readreg(OSC_CONTROL_REG);	    			// Read Oscillator Control, mask ACAL
    temp &= 0x9F;
	writereg(CONFIG_KEY_REG,0xA1);		   	 			// Load the Key register

	switch (period)
    {
		case 0:
            writereg(OSC_CONTROL_REG, temp);	        // Set ACAL to 0
            break;
		case 1:
			temp |= 0x40;
			writereg(OSC_CONTROL_REG, temp);		    // Set ACAL to 2
			//__delay_cycles(10000);						// Wait for initiation of autocal (10 ms)
			//OSTimeDly(OS_TICKS_PER_SEC/100);
			usleep(1000*10);
			writereg(CONFIG_KEY_REG,0xA1);				// Load the Key register
			temp = temp & 0x9F;							// Mask ACAL
			writereg(OSC_CONTROL_REG, temp);	 		// Set ACAL to 0
            break;
		case 2:
			temp = temp | 0x40;
			writereg(OSC_CONTROL_REG, temp);			// Set ACAL to 2
            break;
		case 3:
			temp = temp | 0x60;
			writereg(OSC_CONTROL_REG, temp);		    // Set ACAL to 3
            break;
     }

	//Execution has succeeded
	status = SUCCESS;

// ERRCHKEXIT:
	return status;
}

////////////////////////////////////////
//
// am_read_ram(address);
//
// Read a byte from the local AMX8XX RAM
//
////////////////////////////////////////

uint8_t am_read_ram(uint8_t address)
{
	uint8_t xadd;

    xadd = get_extension_address(address);                  // Calc XADDR value from address
	writereg(EXTENDED_ADDR_REG, xadd);					    // Load the XADDR register
	return readreg((address & 0x3F) | 0x40);				// Read the data
}

///////////////////////////////////////
//
// am_write_ram(address, data);
//
// Write a byte to the local AMX8XX RAM
//
///////////////////////////////////////

void am_write_ram(uint8_t address, uint8_t data)
{
	uint8_t xadd;

    xadd = get_extension_address(address);                  // Calc XADDR value from address
	writereg(EXTENDED_ADDR_REG, xadd);					    // Load the XADDR register
	writereg((address & 0x3F) | 0x40, data);   				// Write the data
}


// Support routines used by driver functions. /////////////////////////////////

uint8_t bcd2dec(uint8_t bcdno)
{
    return ((bcdno >> 4) * 10) + (bcdno & 0x0F);
}



uint8_t dec2bcd (uint8_t decno)
{
  return (((decno / 10) << 4) | (decno % 10));
}



STATUS_t check_param(int32_t param, int32_t min_param, int32_t max_param)
{
	return ((param < min_param) || (param > max_param)) ? ERROR : SUCCESS;
}


/*
void waitms(uint32_t delay)
{
    mcu_msec_delay(delay);
}
*/


static uint8_t get_extension_address(uint8_t address)
{
    uint8_t xadd, temp;

    temp = readreg(EXTENDED_ADDR_REG) & 0xC0;

    if (address < 64) { xadd = 0x8; }
	else if (address < 128) { xadd = 0x9; }
	else if (address < 192) { xadd = 0xA; }
	else { xadd = 0xB; }
    return (xadd | temp);
}



// ERRCHKstatic bool is_leap_year(uint8_t year, uint8_t century)
// ERRCHK{
    // The year is a leap year if it is evenly divisible by 4 or by 400 but not 100.
// ERRCHK    return ((year % 4 == 0) && ((year != 0) || (century != 0))) ? true : false;
// ERRCHK}


///////////////////////////////////////////////////////
//MCU Device-specific (SPI or I2C I/F) Functions     //
///////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// I2C I/O Implementation                                                     //
////////////////////////////////////////////////////////////////////////////////
#ifdef AMX8XX_IF_I2C
// Use the I2C versions of low-level AMX8XX register access routines.
#define AM1805_I2C_ADDR_W		0xd2
#define AM1805_I2C_ADDR_R		0xd3

static int fd = -1;
void Am_I2C_Config(void)
{
	
    //int adapter_nr = 2; /* probably dynamically determined */  
	char filename[20]="/dev/i2c-1";    
	
	fd = open(filename, O_RDWR);  
	if (fd < 0) {    
	 	/* ERROR HANDLING; you can check errno to see what went wrong */    
		//exit(1);
		printf("ERROR GPS write fd=%d\n", fd);
	}
}


void mcu_i2c_write(uint8 len, uint8 address, uint8 *data)
{
	unsigned char* write_data = malloc(len + 1);
	
	if(0==len)
		return;
    
    write_data[0] = address;
    memcpy(&write_data[1], data, len);
    
    ioctl(fd, I2C_SLAVE, AM1805_I2C_ADDR_W);
    ioctl(fd, I2C_TIMEOUT, 1);
    ioctl(fd, I2C_RETRIES, 1);
    
    write(fd, write_data, len + 1);
    
    printf("Write data success\n");
    
    if(write_data != NULL)
    {
        free(write_data);
        write_data = NULL;
    }
}

void mcu_i2c_read(uint8 len, uint8 address, uint8* buf)
{
	if(0==len)
		return;
	
	ioctl(fd, I2C_SLAVE, AM1805_I2C_ADDR_R);
    ioctl(fd, I2C_TIMEOUT, 1);
    ioctl(fd, I2C_RETRIES, 1);
    
    write(fd, &address, 1);
    
    read(fd, buf, len);
    
    printf("buf[0] = 0x%x\n", buf[0]);
    
    printf("Read data success\n");
}


// Clear one or more bits in the selected register, selected by 1's in the mask
void clrreg(uint8_t address, uint8_t mask)
{
    uint8_t temp;

    mcu_i2c_read(1, address, &temp);
	temp &= ~mask;
	mcu_i2c_write(1, address, &temp);
}


// Set one or more bits in the selected register, selected by 1's in the mask
void setreg(uint8_t address, uint8_t mask)
{
	uint8_t temp;

	mcu_i2c_read(1, address, &temp);
	temp |= mask;
	mcu_i2c_write(1, address, &temp);
}


// Executes I2C commands required to read one byte of data at a particular address
uint8_t readreg(uint8_t address)
{
    uint8_t reg_data;

    mcu_i2c_read(1,address, &reg_data);
    return reg_data;
}


// Executes the I2C commands required to write one byte of data to a particular address
void writereg(uint8_t address, uint8_t data)
{
    mcu_i2c_write(1, address, &data);
}


// Executes I2C commands required to implement a burst read of the data at a particular
// address of a particular length into a structure
void burstread(uint8_t address, uint8_t length, uint8_t * data)
{
   mcu_i2c_read(length, address, data);
}


// Executes I2C commands required to implement a burst write of the data in a structure to
// a particular address with a particular length
void burstwrite(uint8_t address, uint8_t length, uint8_t * data)
{
    mcu_i2c_write(length, address, data);
}


////////////////////////////////////////////////////////////////////////////////
// SPI I/O Implementation                                                     //
////////////////////////////////////////////////////////////////////////////////

#else

#endif



void PSWControl(uint8 state)
{
	//uint8 temp = 0;
	
	clrreg(OSC_STATUS_REG, 0x20);

	
	
	clrreg(CONTROL_2_REG, 0x1C);  	// Clear OUT2S
	setreg(CONTROL_2_REG, 0x1C);  	// Set OUT2S to 7
	if(0==state)
	{
		//clrreg(CONTROL_2_REG, 0x1C);  	// Clear OUT2S
		//setreg(CONTROL_2_REG, 0x1C);  	// Set OUT2S to 7
		clrreg(CONTROL_1_REG, 0x20);
	}
	else
	{
		//clrreg(CONTROL_2_REG, 0x1C);  	// Clear OUT2S
		//setreg(CONTROL_2_REG, 0x0C);  	// Set OUT2S to 6
		setreg(CONTROL_1_REG, 0x20);
	}
}

//ÉèÖÃ»½ÐÑµÄ¼ä¸ô²¢½øÈëÐÝÃß×´Ì¬
//usSlot:Ë¯Ãß³ÖÐøµÄÊ±¼ä,µ¥Î»:Ãë,¸ÃÊ±¼ä²»ÄÜ³¬¹ý24Ð¡Ê±
//ucPin: ÖÐ¶Ï½ÅÑ¡Ôñ, 0 => internal flag only
//		             1 => FOUT/nIRQ
//		             2 => PSW/nIRQ2
void RTCSetSleepTime(uint32 uiSlot, uint8 ucPin)
{
	uint32 rtcHour,rtcMin,rtcSec;
	
	if(uiSlot>=86400)
		uiSlot = 86399;
	clrreg(STATUS_REG, 0x7F);//clr interrupter flag
	am_get_time();

	rtcSec = time_regs.second + uiSlot;
	rtcMin = time_regs.minute + (rtcSec/60);
	rtcHour = time_regs.hour + (rtcMin/60);
	
	time_regs.hundredth = 0;
	time_regs.second = rtcSec%60;
	time_regs.minute = rtcMin%60;
	time_regs.hour   = rtcHour%24;
	if(1==ucPin)
	{
		am_set_alarm(4, 0, 1);//am_set_alarm(4, 3, 1);
	}
	else if(2==ucPin)
	{
		am_set_alarm(4, 0, 2);
	}
	else
	{
	}
}

void am_clr_intflag()
{
	clrreg(STATUS_REG, 0x7F);//clr interrupter flag
}

void am_disable_alm()
{
	clrreg(STATUS_REG, 0x7F);//clr interrupter flag
	clrreg(INT_MASK_REG,0x04);//Disable ALE
}

// End Device-specific Functions Group

