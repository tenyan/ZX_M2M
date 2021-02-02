/*******************************************************************************
 * Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
 * @FileName: am1805.c 
 * @Engineer: TenYan
 * @Company:  徐工信息智能硬件部
 * @version:  V1.0
 * @Date:     2020-12-12
 * @brief:    C file containing AM1805 driver functions
 *******************************************************************************/
#include "main.h"
#include "i2c1.h"
#include "am1805.h"
#include "types.h"
#include "cmsis_os2.h"

/******************************************************************************
 * Macros
 ******************************************************************************/ 
#define AM18X5_HUNDREDTHS_REG_ADDR      0x00
#define AM18X5_YEARS_REG_ADDR			      0x06
#define AM18X5_ALARM_HUNDRS_REG_ADDR	  0x08
#define AM18X5_STATUS_REG_ADDR          0x0F
#define AM18X5_CONTROL1_REG_ADDR        0x10
#define AM18X5_CONTROL2_REG_ADDR		    0x11
#define AM18X5_INT_MASK_REG_ADDR		    0x12
#define AM18X5_SQW_REG_ADDR				      0x13
#define AM18X5_CAL_XT_REG_ADDR			    0x14
#define AM18X5_CAL_RC_HI_REG_ADDR		    0x15
#define AM18X5_CAL_RC_LOW_REG_ADDR		  0x16
#define AM18X5_SLEEP_CTRL_REG_ADDR		  0x17
#define AM18X5_TIMER_CTRL_REG_ADDR		  0x18
#define AM18X5_TIMER_REG_ADDR			      0x19
#define AM18X5_TIMER_INITIAL_REG_ADDR	  0x1A
#define AM18X5_WDT_REG_ADDR				      0x1B
#define AM18X5_OSC_CONTROL_REG_ADDR     0x1C
#define AM18X5_OSC_STATUS_REG_ADDR		  0x1D
#define AM18X5_CONFIG_KEY_REG_ADDR      0x1F
#define AM18X5_ACAL_FLT_REG_ADDR     	  0x26
#define AM18X5_EXTENDED_ADDR_REG_ADDR   0x3F

#define am1805_delay(delay)		os_dly_wait(delay)
#define am1805_select()        AM1815_CS_GPIO_PORT->BRR = AM1815_CS_PIN   // FLASH_CS_L()
#define am1805_deselect()     AM1815_CS_GPIO_PORT->BSRR = AM1815_CS_PIN  // FLASH_CS_H()

/******************************************************************************
 * Data Types and Globals
 ******************************************************************************/
am18x5_time_regs_t am18x5_time_regs;
osSemaphoreId_t sid_am18x5_semaphore;  // am18x5信号量
extern osSemaphoreId_t sid_i2c1_semaphore;

/******************************************************************************
 * Support routines used by driver functions
 ******************************************************************************/
// Converts a byte from decimal to binary coded decimal
uint8_t bcd2dec(uint8_t bcdno)
{
  return ((bcdno >> 4) * 10) + (bcdno & 0x0F);
}

// Converts a byte from binary coded decimal to decimal
uint8_t dec2bcd (uint8_t decno)
{
  return (((decno / 10) << 4) | (decno % 10));
}

//===========================================================================
void mcu_i2c_write(uint8 len, uint8 address, uint8 *wrptr)
{
  uint8_t i = 0;

  osSemaphoreAcquire(sid_i2c1_semaphore, osWaitForever); // 等待信号量

  i2c1_start(); 	                      // Generate Start condition
  i2c1_write_byte((AM1805_I2C_ADDR & 0xFE)); // Write Control Byte MAKE SURE IT IS WRITE CONDITON
  i2c1_wait_ack();
  i2c1_write_byte(address);             // send low address
  i2c1_wait_ack();
  for (i=0;i<len;i++)
  {
    i2c1_write_byte(*wrptr); //write a single byte
    i2c1_wait_ack();
    wrptr++;
  }
  i2c1_stop(); 	                      // Generate Stop

  osSemaphoreRelease(sid_i2c1_semaphore);  // 释放信号量
}

//===========================================================================
void mcu_i2c_read(uint8 len, uint8 address, uint8* data)
{
  uint8_t i = 0;

  osSemaphoreAcquire(sid_i2c1_semaphore, osWaitForever); // 等待信号量

  i2c1_start(); 	                      // Generate Start Condition
  i2c1_write_byte((AM1805_I2C_ADDR & 0xFE)); // Write Control Byte MAKE SURE IT IS WRITE CONDITON
  i2c1_wait_ack();
  i2c1_write_byte(address); 		        // Write start address
  i2c1_wait_ack();
  i2c1_restart();                      // Generate restart condition
  i2c1_write_byte((AM1805_I2C_ADDR | 0x01)); // Write control byte for read
  i2c1_wait_ack();
  for (i=0;i<len-1;i++)            // read Length number of bytes
  {
    *data = i2c1_read_byte();
    i2c1_ack();
    data++;
  }
  *data = i2c1_read_byte();
  i2c1_not_ack(); 	                    // Send Not Ack
  i2c1_stop();    	                    // Generate Stop

	osSemaphoreRelease(sid_i2c1_semaphore);  // 释放信号量
}

// Clear one or more bits in the selected register, selected by 1's in the mask
void am1805_clrreg(uint8_t address, uint8_t mask)
{
  uint8_t temp;

  mcu_i2c_read(1, address, &temp);
  temp &= ~mask;
  mcu_i2c_write(1, address, &temp);
}

// Set one or more bits in the selected register, selected by 1's in the mask
void am1805_setreg(uint8_t address, uint8_t mask)
{
  uint8_t temp;

  mcu_i2c_read(1, address, &temp);
  temp |= mask;
  mcu_i2c_write(1, address, &temp);
}

// Executes I2C commands required to read one byte of data at a particular address
uint8_t am1805_readreg(uint8_t address)
{
  uint8_t reg_data;

  mcu_i2c_read(1,address, &reg_data);
  return reg_data;
}

// Executes the I2C commands required to write one byte of data to a particular address
void am1805_writereg(uint8_t address, uint8_t data)
{
  mcu_i2c_write(1, address, &data);
}

// Executes I2C commands required to implement a burst read of the data at a particular
// address of a particular length into a structure
void am1805_burst_read(uint8_t address, uint8_t length, uint8_t * data)
{
  mcu_i2c_read(length, address, data);
}

// Executes I2C commands required to implement a burst write of the data in a structure to
// a particular address with a particular length
void am1805_burst_write(uint8_t address, uint8_t length, uint8_t * data)
{
  mcu_i2c_write(length, address, data);
}

/*************************************************************************
* am18x5_GetTime - get the current am18x5 time in the counters
* am18x5_time_regs - a time_regs_t struct that contains the time to be set
* This function loads the am18x5_time_regs structure with the time from the am18x5
*************************************************************************/
void am18x5_GetTime(void)
{
  uint8_t temp_buff[8];

  osSemaphoreAcquire(sid_am18x5_semaphore, osWaitForever); // 等待信号量
  am1805_burst_read(AM18X5_HUNDREDTHS_REG_ADDR, 8, temp_buff); // Read the counters.
	
	// 将时间从BCD编码变成十进制
  am18x5_time_regs.hundredth = bcd2dec(temp_buff[0]); // 世纪
  am18x5_time_regs.second = bcd2dec(temp_buff[1]);  // 秒
  am18x5_time_regs.minute = bcd2dec(temp_buff[2]);  // 分
  am18x5_time_regs.hour = temp_buff[3];             // 时
  am18x5_time_regs.date = bcd2dec(temp_buff[4]);    // 日
  am18x5_time_regs.month = bcd2dec(temp_buff[5]);   // 月
  am18x5_time_regs.year = bcd2dec(temp_buff[6]);    // 年
  am18x5_time_regs.weekday = bcd2dec(temp_buff[7]); // 周

  // Get the current hours format mode 12/24.
  temp_buff[0] = am1805_readreg(AM18X5_CONTROL1_REG_ADDR);
  if ((temp_buff[0] & 0x40) == 0) // 24-hour mode.
  {
    am18x5_time_regs.mode = 2;
    am18x5_time_regs.hour = am18x5_time_regs.hour & 0x3F; // Get tens:ones
  }
  else // 12-hour mode.  Get PM:AM.
  {
    am18x5_time_regs.mode = (am18x5_time_regs.hour & 0x20) ? 1 : 0;  // PM : AM
    am18x5_time_regs.hour &= 0x1F;                            // Get tens:ones
  }
  am18x5_time_regs.hour = bcd2dec(am18x5_time_regs.hour);

  // Get the century bit.
  temp_buff[0] = am1805_readreg(AM18X5_STATUS_REG_ADDR);
  am18x5_time_regs.century = (temp_buff[0] & 0x80) ? 1 : 0;
	
  osSemaphoreRelease(sid_am18x5_semaphore);  // 释放信号量
}

/*************************************************************************
* am18x5_SetTime - set the time in the counters
* Inputs:
* protect - 0 => leave counters writable, 1 => leave counters unwritable
* am18x5_time_regs - a time_reg_struct that contains the time to be set
* This function loads the AMX8XX counter registers with the current
* am18x5_time_regs structure values.
*************************************************************************/
void am18x5_SetTime(uint8_t protect)
{
  uint8_t temp_buff[8];

  osSemaphoreAcquire(sid_am18x5_semaphore, osWaitForever); // 等待信号量
  //Convert decimal to binary-coded decimal
  am18x5_time_regs.hundredth = dec2bcd(am18x5_time_regs.hundredth);
  am18x5_time_regs.second = dec2bcd(am18x5_time_regs.second);
  am18x5_time_regs.minute = dec2bcd(am18x5_time_regs.minute);
  am18x5_time_regs.hour = dec2bcd(am18x5_time_regs.hour);
  am18x5_time_regs.date = dec2bcd(am18x5_time_regs.date);
  am18x5_time_regs.weekday = dec2bcd(am18x5_time_regs.weekday);
  am18x5_time_regs.month = dec2bcd(am18x5_time_regs.month);
  am18x5_time_regs.year = dec2bcd(am18x5_time_regs.year);

  // Determine whether 12 or 24-hour timekeeping mode is being used
  // and set the 12/24 bit appropriately
  if (am18x5_time_regs.mode == 2) // 24-hour day
  {
    am1805_clrreg(AM18X5_CONTROL1_REG_ADDR, 0x40);
  }
  else if (am18x5_time_regs.mode == 1) // 12-hour day PM
  {
    am18x5_time_regs.hour |= 0x20;	    // Set AM/PM
    am1805_setreg(AM18X5_CONTROL1_REG_ADDR, 0x40);
  }
  else // 12-hour day AM
  {
    am1805_setreg(AM18X5_CONTROL1_REG_ADDR, 0x40);
  }

  // Set the WRTC bit to enable counter writes.
  am1805_setreg(AM18X5_CONTROL1_REG_ADDR, 0x01);

  // Set the correct century
  if (am18x5_time_regs.century == 0)
  {  am1805_clrreg(AM18X5_STATUS_REG_ADDR, 0x80);}
  else
	{  am1805_setreg(AM18X5_STATUS_REG_ADDR, 0x80);}

  // Write all of the time counters
  temp_buff[0] = am18x5_time_regs.hundredth;
  temp_buff[1] = am18x5_time_regs.second;
  temp_buff[2] = am18x5_time_regs.minute;
  temp_buff[3] = am18x5_time_regs.hour;
  temp_buff[4] = am18x5_time_regs.date;
  temp_buff[5] = am18x5_time_regs.month;
  temp_buff[6] = am18x5_time_regs.year;
  temp_buff[7] = am18x5_time_regs.weekday;

  // Write the values to the AM18XX
  am1805_burst_write(AM18X5_HUNDREDTHS_REG_ADDR, 8, temp_buff);

  // Load the final value of the WRTC bit based on the value of protect
  temp_buff[0] = am1805_readreg(AM18X5_CONTROL1_REG_ADDR);
  temp_buff[0] &= 0x7E; // Clear the WRTC bit and the STOP bit
  temp_buff[0] |= (0x01 & (~protect));  // Invert the protect bit and update WRTC
  am1805_writereg(AM18X5_CONTROL1_REG_ADDR, temp_buff[0]);
  osSemaphoreRelease(sid_am18x5_semaphore);  // 释放信号量
}

/*************************************************************************
* am1805_set_alarm - set the alarm value
* Inputs:
*	repeat - the alarm repeat interval
*		0 => disable alarm
*		1 => once per year
*		2 => once per month
*		3 => once per week
*		4 => once per day
*		5 => once per hour
*		6 => once per minute
*		7 => once per second
*		8 => once per 10th of a second
*		9 => once per 100th of a second
*		NOTE: year and century are not used
*		NOTE: mode must match current 12/24 selection
*	intmode - define the interrupt mode
*		0 => level interrupt
*		1 => pulse of 1/8192s (XT) or 1/128 s (RC)
*		2 => pulse of 1/64 s
*		3 => pulse of 1/4 s
*	pin - pin on which to generate the interrupt
*		0 => internal flag only
*		1 => FOUT/nIRQ
*		2 => PSW/nIRQ2
*
*	am18x5_time_regs - a time_reg_struct that contains the time to be set
*************************************************************************/
void am1805_set_alarm(uint8_t repeat, uint8_t intmode, uint8_t pin)
{
	uint8_t temp;
	uint8_t temp_buff[8];

  osSemaphoreAcquire(sid_am18x5_semaphore, osWaitForever); // 等待信号量
	// Convert decimal to binary-coded decimal
	am18x5_time_regs.hundredth = dec2bcd(am18x5_time_regs.hundredth);
	am18x5_time_regs.second = dec2bcd(am18x5_time_regs.second);
	am18x5_time_regs.minute = dec2bcd(am18x5_time_regs.minute);
	am18x5_time_regs.hour = dec2bcd(am18x5_time_regs.hour);
	am18x5_time_regs.date = dec2bcd(am18x5_time_regs.date);
	am18x5_time_regs.weekday = dec2bcd(am18x5_time_regs.weekday);
	am18x5_time_regs.month = dec2bcd(am18x5_time_regs.month);

	// Determine whether a 12-hour or a 24-hour time keeping mode is being used
	if (am18x5_time_regs.mode == 1)
	{
		// A 12-hour day PM
		am18x5_time_regs.hour = am18x5_time_regs.hour | 0x20;	  // Set AM/PM
	}

	// Write all of the time counters
	temp_buff[0] = am18x5_time_regs.hundredth;
	temp_buff[1] = am18x5_time_regs.second;
	temp_buff[2] = am18x5_time_regs.minute;
	temp_buff[3] = am18x5_time_regs.hour;
	temp_buff[4] = am18x5_time_regs.date;
	temp_buff[5] = am18x5_time_regs.month;
	temp_buff[6] = am18x5_time_regs.weekday;

	am1805_clrreg(AM18X5_TIMER_CTRL_REG_ADDR, 0x1C);			// Clear the RPT field
	am1805_clrreg(AM18X5_INT_MASK_REG_ADDR, 0x64);				// Clear the AIE bit and IM field
	am1805_clrreg(AM18X5_STATUS_REG_ADDR, 0x04);				// Clear the ALM flag

  if (pin == 1)
	{
		// Interrupt on FOUT/nIRQ
		temp = am1805_readreg(AM18X5_CONTROL2_REG_ADDR);		// Get the Control2 Register
		temp = (temp & 0x03);				// Extract the OUT1S field
		if (temp != 0)						// Not already selecting nIRQ
		{
			am1805_setreg(AM18X5_CONTROL2_REG_ADDR, 0x03);	// Set OUT1S to 3
		}
	}
	if (pin == 2)
  {
  		// Interrupt on PSW/nIRQ2
		temp = am1805_readreg(AM18X5_CONTROL2_REG_ADDR);		// Get the Control2 Register
		temp &= 0x1C;						// Extract the OUT2S field
		if (temp != 0)						// Not already selecting nIRQ
		{
			am1805_clrreg(AM18X5_CONTROL2_REG_ADDR, 0x1C);  	// Clear OUT2S
			am1805_setreg(AM18X5_CONTROL2_REG_ADDR, 0x0C);  	// Set OUT2S to 3
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
		am1805_setreg(AM18X5_TIMER_CTRL_REG_ADDR, temp);				// Was previously cleared
		am1805_setreg(AM18X5_INT_MASK_REG_ADDR, (intmode << 5));		// Set the alarm interrupt mode
		am1805_burst_write(AM18X5_ALARM_HUNDRS_REG_ADDR, 7, temp_buff);	// Execute the burst write
		am1805_setreg(AM18X5_INT_MASK_REG_ADDR, 0x04);					// Set the AIE bit
	}
	else
	{	am1805_setreg(AM18X5_INT_MASK_REG_ADDR, 0x60);}					// Set IM field to 0x3 (reset value) to minimize current draw
	osSemaphoreRelease(sid_am18x5_semaphore);  // 释放信号量
}

/*************************************************************************
* am1805_osc_sel - select an oscillator mode
*	osc - the oscillator to select
*  0 => 32 KHz XT oscillator, no automatic oscillator switching
*	 1 => 32 KHz XT oscillator, automatic oscillator switching to RC on switch to battery power
*	 2 => 128 Hz RC oscillator
*************************************************************************/
uint8_t am1805_osc_sel(uint8_t osc)
{
	uint8_t i;
	uint8_t temp;

	// Read Oscillator Control register
	temp = am1805_readreg(AM18X5_OSC_CONTROL_REG_ADDR);
	temp = temp & 0x67; 			    		// Clear OSEL, FOS, AOS

	// Enable Oscillator Register writes
	am1805_writereg(AM18X5_CONFIG_KEY_REG_ADDR,0xA1);	// Write the Key register

	switch (osc)
  {
  case 0:   // Do nothing, clear Key register
    am1805_writereg(AM18X5_OSC_CONTROL_REG_ADDR,temp);
    break;

  case 1:
    temp = temp | 0x10;  // Set AOS
    am1805_writereg(AM18X5_OSC_CONTROL_REG_ADDR,temp);
    break;

  default:
    temp = temp | 0x80;  // Set OSEL
    am1805_writereg(AM18X5_OSC_CONTROL_REG_ADDR,temp);
    break;
	}

	// Wait to make sure switch occurred by testing OMODE
	for (i = 0; i < 100; i++)
	{
		osDelay(OS_TICKS_PER_SEC / 100);
		temp = am1805_readreg(AM18X5_OSC_STATUS_REG_ADDR);	// Read OMODE
		temp = (temp & 0x10) >> 4;
		if (temp == (osc >> 1)) return FALSE;		// Successful switch
	}
	
	return FALSE;
}

/*************************************************************************
 *
*************************************************************************/
void am18x5_PswControl(uint8_t state)
{
	osSemaphoreAcquire(sid_am18x5_semaphore, osWaitForever); // 等待信号量
	am1805_clrreg(AM18X5_OSC_STATUS_REG_ADDR, 0x20);
	am1805_clrreg(AM18X5_CONTROL2_REG_ADDR, 0x1C);  	// Clear OUT2S
	am1805_setreg(AM18X5_CONTROL2_REG_ADDR, 0x1C);  	// Set OUT2S to 7
	if(0==state)
	{  am1805_clrreg(AM18X5_CONTROL1_REG_ADDR, 0x20);}
	else
	{  am1805_setreg(AM18X5_CONTROL1_REG_ADDR, 0x20);}
	osSemaphoreRelease(sid_am18x5_semaphore);  // 释放信号量
}

/*************************************************************************
* 设置唤醒的间隔并进入休眠状态
* usSlot:睡眠持续的时间,单位:秒,该时间不能超过24小时
* ucPin: 中断脚选择
*   0 => internal flag only
*		1 => FOUT/nIRQ
*		2 => PSW/nIRQ2
*************************************************************************/
void am18x5_SetSleepTime(uint32_t uiSlot, uint8_t ucPin)
{
	uint32_t rtcHour,rtcMin,rtcSec;
	
	if(uiSlot>=86400)
		uiSlot = 86399;

	am1805_clrreg(AM18X5_STATUS_REG_ADDR, 0x7F); // clr interrupter flag
	am18x5_GetTime();

	rtcSec = am18x5_time_regs.second + uiSlot;
	rtcMin = am18x5_time_regs.minute + (rtcSec / 60);
	rtcHour = am18x5_time_regs.hour + (rtcMin / 60);
	
	am18x5_time_regs.hundredth = 0;
	am18x5_time_regs.second = rtcSec % 60;
	am18x5_time_regs.minute = rtcMin % 60;
	am18x5_time_regs.hour   = rtcHour % 24;
	
	if(1==ucPin)
	{
		am1805_set_alarm(4, 0, 1);//am1805_set_alarm(4, 3, 1);
	}
	else if(2==ucPin)
	{
		am1805_set_alarm(4, 0, 2);
	}
	else
	{
		
	}
}

/*************************************************************************
 * 
*************************************************************************/
void am18x5_ClearIntFlag(void)
{
	osSemaphoreAcquire(sid_am18x5_semaphore, osWaitForever); // 等待信号量
	am1805_clrreg(AM18X5_STATUS_REG_ADDR, 0x7F); // clr interrupter flag
	osSemaphoreRelease(sid_am18x5_semaphore);  // 释放信号量
}

/*************************************************************************
 * 
*************************************************************************/
void am18x5_DisableAlarm(void)
{
	osSemaphoreAcquire(sid_am18x5_semaphore, osWaitForever); // 等待信号量
	am1805_clrreg(AM18X5_STATUS_REG_ADDR, 0x7F);  // clr interrupter flag
	am1805_clrreg(AM18X5_INT_MASK_REG_ADDR,0x04); // Disable ALE
	osSemaphoreRelease(sid_am18x5_semaphore);  // 释放信号量
}

/*********************************************************************
 * 读取时间
 *********************************************************************/
void am18x5_GetDateTime(rtc_date_t *time)
{  
  am18x5_GetTime(); // 读取时间

  //将时间从BCD编码变成十进制
  time->second = am18x5_time_regs.second;		//秒
  time->minute = am18x5_time_regs.minute;		//分
  time->hour = am18x5_time_regs.hour;			  //时
  time->day = am18x5_time_regs.date;	      //日
  time->weekday = am18x5_time_regs.weekday; //星期
  time->month = am18x5_time_regs.month;		  //月
  time->year = am18x5_time_regs.year;			  //年
}

/*********************************************************************
 * 设置时间
 *********************************************************************/
void am18x5_SetDateTime(rtc_date_t *time)
{
  //将时间从十进制码转换成BCD码
  am18x5_time_regs.second = time->second;	//秒
  am18x5_time_regs.minute = time->minute;	//分
  am18x5_time_regs.hour = time->hour;	    //时
  am18x5_time_regs.date = time->day;	    //日
  am18x5_time_regs.month = time->month;		//月
  am18x5_time_regs.year = time->year;			//年
  
  am18x5_SetTime(1);
}

/*************************************************************************
 * 调此函数前,需先初始化I2C接口
*************************************************************************/
void am18x5_Initialize(void)
{	
	i2c1_Initialize();
  
  if(sid_am18x5_semaphore==NULL)
  {
    sid_am18x5_semaphore = osSemaphoreNew(1, 1, NULL);;   //创建AM18x5互斥信号量
  }  

	am1805_osc_sel(0);
	am18x5_PswControl(0); // 使能电池供电
	am1805_clrreg(AM18X5_INT_MASK_REG_ADDR,0x04); // Disable ALE
	am1805_clrreg(AM18X5_STATUS_REG_ADDR, 0x7F);  // clr interrupter flag
	am1805_setreg(AM18X5_CONTROL1_REG_ADDR, 0x12);// am1805_setreg(AM18X5_CONTROL1_REG_ADDR, 0x02);
}

//-----文件AM1805.c结束---------------------------------------------
