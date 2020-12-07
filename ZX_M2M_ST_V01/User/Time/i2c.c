/*****************************************************************************
* FileName:        i2c.c
* Engineer:        TenYan
* Date:            2019-2-19
* Driver for the I2C on STM32.
******************************************************************************/

/******************************************************************************
 *   Includes
 ******************************************************************************/
#include "config.h"

/******************************************************************************
 *   Macros
 ******************************************************************************/ 
#define I2C_SCL_PORT	GPIOB  // 开漏
#define I2C_SCL_PIN		GPIO_Pin_6
#define I2C_SCL_H()   I2C_SCL_PORT->BSRRL = I2C_SCL_PIN
#define I2C_SCL_L()   I2C_SCL_PORT->BSRRH  = I2C_SCL_PIN

#define I2C_SDA_PORT	GPIOB  // 开漏
#define I2C_SDA_PIN		GPIO_Pin_7
#define I2C_SDA_H()   I2C_SDA_PORT->BSRRL = I2C_SDA_PIN
#define I2C_SDA_L()   I2C_SDA_PORT->BSRRH  = I2C_SDA_PIN

#define I2C_SDA_IN    (I2C_SDA_PORT->IDR & I2C_SDA_PIN)

/******************************************************************************
* Data Types and Globals
******************************************************************************/
OS_EVENT *i2c1_semaphore = NULL;


/*******************************************************************************
 * 
 *******************************************************************************/
void i2c_Initialize(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	I2C_SCL_H();
	I2C_SDA_H();
	
  //PB7: I2C1_SDA 和 PB6: I2C1_SCL
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

	if(i2c1_semaphore == NULL)
		i2c1_semaphore = OSSemCreate(1);   //创建I2C通信的互斥信号量
}

/********************************************************				
 * 功能描述: 延时函数								 
 * 输    入：延时毫秒数
 * 输    出：无
********************************************************/
void delay_ms(uint32_t t)
{		
	uint32_t i;
	
  while(t--)
	{
		for(i = 0; i<12030; i++)
		{
			__nop();
			IWdtFeed();
		}
	}
}

/*******************************************************************************
 * 
 *******************************************************************************/
void delay_us(uint32_t nTimer)
{
	volatile uint32_t i = 0;
	for(i=0; i<nTimer; i++)
	{
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	//	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();

	}
	//IWdtFeed();
}

#define I2C_DELAY()  delay_us(10)
//#define I2C_DELAY10ms()  delay_us(10)

// 在执行任何一次i2c操作前和i2c操作后,应保证:
// 1: sda, 输出, 高电平(必须保证)
// 2: scl, 输出, 高电平()

/*********************************************************************
 * Waits for bus to become Idle
 ********************************************************************/
void i2c_idle(void)
{
	return;
}

/*********************************************************************
 * Generates an I2C Start Condition
 ********************************************************************/
void i2c_start(void)
{
  I2C_SCL_H();  // SCL高
	I2C_DELAY();  // 0.6uS
	
  I2C_SDA_H();	// SDA高 -> 低
	I2C_DELAY();	// tSU, Min:4.7us	  
	    	 
	I2C_SDA_L();      
  I2C_DELAY();  // tHD, Min:4.0us 
	
	I2C_SCL_L();  //SCL低(待写地址/数据)
  I2C_DELAY();  
}

/*********************************************************************
 * Generates a restart condition and optionally returns status
 ********************************************************************/
void i2c_restart(void)
{
  I2C_SCL_H();  // SCL高
	I2C_DELAY();
	
  I2C_SDA_H();	// SDA高 -> 低
	I2C_DELAY();	// tSU, Min:4.7us	  
	    	 
	I2C_SDA_L();      
  I2C_DELAY();  // tHD, Min:4.0us 
	
	I2C_SCL_L(); //SCL低(待写地址/数据)
  I2C_DELAY(); 
}

/*********************************************************************
 * Generates a bus stop condition
 ********************************************************************/
void i2c_stop(void)
{	
  I2C_SDA_L(); // SDA低 -> 高
  I2C_DELAY();
	
  I2C_SCL_H();
  I2C_DELAY(); 
	
  I2C_SDA_H();   
  I2C_DELAY();
}

/********************************************************************  
*
 ********************************************************************/  
void i2c_abort(void)
{
	return;
}
  
/********************************************************************  
* 等待应答信号
* I2C主机读取应答(或非应答)位
 ********************************************************************/  
uint8_t i2c_wait_ack(void)
{  
	uint16_t i2c_timeout = 0; 

  I2C_SCL_L();        // SCL低 -> 高
  I2C_DELAY();
	
  I2C_SDA_H();   	  	//释放SDA(开漏模式有效)
  I2C_DELAY();
	
  I2C_SCL_H();        // SCL高(读取应答位)
  I2C_DELAY();
  while(I2C_SDA_IN)
  {
     i2c_timeout++;   
     if(i2c_timeout > 8000)   
     {
       i2c_stop();
       return 0;
     }
  }
	  
  I2C_SCL_L();
  I2C_DELAY();
  return 1;   
}

/*********************************************************************
 * Generates an Acknowledge.
 ********************************************************************/
void i2c_ack(void)
{
	I2C_SCL_L();
  I2C_DELAY();
	
	I2C_SDA_L();  // 应答
	I2C_DELAY();
	
	I2C_SCL_H();  // SCL高 -> 低
	I2C_DELAY();
	I2C_SCL_L();
  I2C_DELAY();
}

/*********************************************************************
 * Generates a NO Acknowledge on the Bus
 ********************************************************************/
void i2c_not_ack(void)
{
	I2C_SCL_L();
  I2C_DELAY();
	
	I2C_SDA_H();  // 非应答
	I2C_DELAY();
	
	I2C_SCL_H();  // SCL高 -> 低
	I2C_DELAY();
	I2C_SCL_L();
	I2C_DELAY();
}

/*********************************************************************
 * Writes a byte out to the bus
 ********************************************************************/
void i2c_write_byte(uint8_t data)
{
	uint8_t i,temp_value; 
	temp_value = data;   			// data

	for(i=0; i<8; i++)
  {
		I2C_SCL_L();           // SCL低电平时,变化SDA有效
		I2C_DELAY();
		
		if(temp_value & 0x80)
			I2C_SDA_H();
		else
			I2C_SDA_L();
		
		temp_value <<= 1;	
		I2C_DELAY();
		
   	I2C_SCL_H();         //SCL高(发送数据)
		I2C_DELAY(); 
  }
	I2C_SCL_L();           // SCL低(等待应答信号)
	I2C_DELAY();
}

/*********************************************************************
 * Read a single byte from Bus
 ********************************************************************/
uint8_t i2c_read_byte(void)
{
	uint8_t i,temp_value = 0;
	
  I2C_SCL_L();        // 先SCL低,在释放SDA
	I2C_DELAY();

	I2C_SDA_H();        // 释放SDA(开漏模式有效)
  for(i=0; i<8; i++)     // SCL低 -> 高,读出SDA数据
  {
    I2C_SCL_H();      //SCL高(读取数据)
	  I2C_DELAY();
		
		temp_value <<= 1;
		if(I2C_SDA_IN) temp_value += 1;
		
		I2C_SCL_L(); 
		I2C_DELAY();
  }
	
	return (temp_value);	  //Return data
}

/*********************************************************************
 * Input: Control Byte, 8 - bit address, data.
 * Write a byte to low density device at address LowAdd
 ********************************************************************/
void i2c_LD_write_byte(uint8_t ControlByte, uint8_t Address, uint8_t data)
{
  uint8 err;
	
  OSSemPend(i2c1_semaphore, osWaitForever, &err);
  if(err != OS_ERR_NONE) { return;}
	
	i2c_start();                          // Generate Start COndition
	i2c_write_byte(ControlByte & 0xFE);   // Write Control byte
	i2c_wait_ack();
	i2c_write_byte(Address); 		          // Write start address
	i2c_wait_ack();
	i2c_write_byte(data);                 // Write Data
	i2c_wait_ack();
	i2c_stop();                           // Initiate Stop Condition
	
  OSSemPost(i2c1_semaphore);
}

/*********************************************************************
 * Input: Control Byte, Address, *Data, Length.
 * Performs a low density read a byte 
 ********************************************************************/
uint8_t i2c_LD_read_byte(uint8_t ControlByte, uint8_t Address)
{
	uint8_t dat;
  uint8 err;
	
  OSSemPend(i2c1_semaphore, osWaitForever, &err);

	i2c_start(); 	                      // Generate Start Condition
	i2c_write_byte(ControlByte & 0xFE); // Write Control Byte MAKE SURE IT IS WRITE CONDITON
  i2c_wait_ack();
	i2c_write_byte(Address); 		        // Write start address
  i2c_wait_ack();
	i2c_restart();                      // Generate restart condition
	i2c_write_byte(ControlByte | 0x01); // Write control byte for read
  i2c_wait_ack();
	dat = i2c_read_byte(); 	            // read a byte
	i2c_not_ack(); 	                    // Send Not Ack
	i2c_stop();    	                    // Generate Stop
	
  OSSemPost(i2c1_semaphore);
	
	return dat;
}

/*********************************************************************
 * Input: Control Byte, Address, *Data, Length.
 * Performs a low density read of Length bytes and stores in *Data 
 * array starting at Address.
 ********************************************************************/
void i2c_LD_read_bytes(uint8_t ControlByte, uint8_t Address, uint8_t *Data, uint8_t Length)
{
	uint8_t i = 0;
  uint8 err;
	
  OSSemPend(i2c1_semaphore, osWaitForever, &err);
  if(err != OS_ERR_NONE) { return;}
	
	i2c_start(); 	                      // Generate Start Condition
	i2c_write_byte(ControlByte & 0xFE); // Write Control Byte MAKE SURE IT IS WRITE CONDITON
  i2c_wait_ack();
	i2c_write_byte(Address); 		        // Write start address
  i2c_wait_ack();
	i2c_restart();                      // Generate restart condition
	i2c_write_byte(ControlByte | 0x01); // Write control byte for read
  i2c_wait_ack();
	for(i=0;i<Length-1;i++)             // read Length number of bytes
	{
		*Data = i2c_read_byte();
		i2c_ack();
		Data++;
	}
	*Data = i2c_read_byte();
	i2c_not_ack(); 	                    // Send Not Ack
	i2c_stop();    	                    // Generate Stop

  OSSemPost(i2c1_semaphore);
}

/*********************************************************************
 * Input: ControlByte, LowAdd, *wrptr ,Length.
 * Write a page of data from array pointed to be wrptr starting at LowAdd
 * Note:  LowAdd must start on a page boundary
 ********************************************************************/
void i2c_LD_write_bytes(uint8_t ControlByte, uint8_t LowAdd, uint8_t *wrptr,uint8_t Length)
{
	uint8_t i = 0;
  uint8 err;
	
  OSSemPend(i2c1_semaphore, osWaitForever, &err);
  if(err != OS_ERR_NONE) { return;}
	
	i2c_start(); 	                      // Generate Start condition
	i2c_write_byte(ControlByte & 0xFE); // Write Control Byte MAKE SURE IT IS WRITE CONDITON
  i2c_wait_ack();
	i2c_write_byte(LowAdd);             // send low address
  i2c_wait_ack();
	for(i=0;i<Length;i++)
	{
		i2c_write_byte(*wrptr); //write a single byte
    i2c_wait_ack();
		wrptr++;
	}
	i2c_stop(); 	                      // Generate Stop

  OSSemPost(i2c1_semaphore);
}

