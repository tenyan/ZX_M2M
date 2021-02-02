/*****************************************************************************
* FileName:        i2c1.c
* Engineer:        TenYan
* Date:            2020-10-2
* Driver for the I2C on STM32.
******************************************************************************/

/******************************************************************************
 *   Includes
 ******************************************************************************/
#include "config.h"

/******************************************************************************
 *   Macros
 ******************************************************************************/


/******************************************************************************
* Data Types and Globals
******************************************************************************/
osSemaphoreId_t sid_i2c1_semaphore = NULL;

/*******************************************************************************
 *
 *******************************************************************************/
void i2c1_Initialize(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

  I2C1_SCL_H();
  I2C1_SDA_H();

  //PB7: I2C1_SDA 和 PB6: I2C1_SCL
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  if(sid_i2c1_semaphore==NULL)
  {
    sid_i2c1_semaphore = osSemaphoreNew(1, 1, NULL);   //创建I2C通信的互斥信号量
  }
}


/*******************************************************************************
 *
 *******************************************************************************/
#define I2C1_DELAY()  delay_us(10)

// 在执行任何一次i2c操作前和i2c操作后,应保证:
// 1: sda, 输出, 高电平(必须保证)
// 2: scl, 输出, 高电平()

/*********************************************************************
 * Waits for bus to become Idle
 ********************************************************************/
void i2c1_idle(void)
{
  return;
}

/*********************************************************************
 * Generates an I2C Start Condition
 ********************************************************************/
void i2c1_start(void)
{
  I2C1_SCL_H();  // SCL高
  I2C1_DELAY();  // 0.6uS

  I2C1_SDA_H();	// SDA高 -> 低
  I2C1_DELAY();	// tSU, Min:4.7us

  I2C1_SDA_L();
  I2C1_DELAY();  // tHD, Min:4.0us

  I2C1_SCL_L();  //SCL低(待写地址/数据)
  I2C1_DELAY();
}

/*********************************************************************
 * Generates a restart condition and optionally returns status
 ********************************************************************/
void i2c1_restart(void)
{
  I2C1_SCL_H();  // SCL高
  I2C1_DELAY();

  I2C1_SDA_H();	// SDA高 -> 低
  I2C1_DELAY();	// tSU, Min:4.7us

  I2C1_SDA_L();
  I2C1_DELAY();  // tHD, Min:4.0us

  I2C1_SCL_L(); //SCL低(待写地址/数据)
  I2C1_DELAY();
}

/*********************************************************************
 * Generates a bus stop condition
 ********************************************************************/
void i2c1_stop(void)
{
  I2C1_SDA_L(); // SDA低 -> 高
  I2C1_DELAY();

  I2C1_SCL_H();
  I2C1_DELAY();

  I2C1_SDA_H();
  I2C1_DELAY();
}

/********************************************************************
*
 ********************************************************************/
void i2c1_abort(void)
{
  return;
}

/********************************************************************
* 等待应答信号
* I2C主机读取应答(或非应答)位
 ********************************************************************/
uint8_t i2c1_wait_ack(void)
{
  uint16_t i2c_timeout = 0;

  I2C1_SCL_L();        // SCL低 -> 高
  I2C1_DELAY();

  I2C1_SDA_H();   	  	//释放SDA(开漏模式有效)
  I2C1_DELAY();

  I2C1_SCL_H();        // SCL高(读取应答位)
  I2C1_DELAY();
  while (I2C1_SDA_IN)
  {
    i2c_timeout++;
    if (i2c_timeout > 2000)
    {
      i2c1_stop();
      return 0;
    }
  }

  I2C1_SCL_L();
  I2C1_DELAY();
  return 1;
}

/*********************************************************************
 * Generates an Acknowledge.
 ********************************************************************/
void i2c1_ack(void)
{
  I2C1_SCL_L();
  I2C1_DELAY();

  I2C1_SDA_L();  // 应答
  I2C1_DELAY();

  I2C1_SCL_H();  // SCL高 -> 低
  I2C1_DELAY();
  I2C1_SCL_L();
  I2C1_DELAY();
}

/*********************************************************************
 * Generates a NO Acknowledge on the Bus
 ********************************************************************/
void i2c1_not_ack(void)
{
  I2C1_SCL_L();
  I2C1_DELAY();

  I2C1_SDA_H();  // 非应答
  I2C1_DELAY();

  I2C1_SCL_H();  // SCL高 -> 低
  I2C1_DELAY();
  I2C1_SCL_L();
  I2C1_DELAY();
}

/*********************************************************************
 * Writes a byte out to the bus
 ********************************************************************/
void i2c1_write_byte(uint8_t data)
{
  uint8_t i,temp_value;
  temp_value = data;   			// data

  for (i=0; i<8; i++)
  {
    I2C1_SCL_L();           // SCL低电平时,变化SDA有效
    I2C1_DELAY();

    if (temp_value & 0x80)
      I2C1_SDA_H();
    else
      I2C1_SDA_L();

    temp_value <<= 1;
    I2C1_DELAY();

    I2C1_SCL_H();         //SCL高(发送数据)
    I2C1_DELAY();
  }
  I2C1_SCL_L();           // SCL低(等待应答信号)
  I2C1_DELAY();
}

/*********************************************************************
 * Read a single byte from Bus
 ********************************************************************/
uint8_t i2c1_read_byte(void)
{
  uint8_t i,temp_value = 0;

  I2C1_SCL_L();        // 先SCL低,在释放SDA
  I2C1_DELAY();

  I2C1_SDA_H();        // 释放SDA(开漏模式有效)
  for (i=0; i<8; i++)    // SCL低 -> 高,读出SDA数据
  {
    I2C1_SCL_H();      //SCL高(读取数据)
    I2C1_DELAY();

    temp_value <<= 1;
    if (I2C1_SDA_IN) temp_value += 1;

    I2C1_SCL_L();
    I2C1_DELAY();
  }

  return (temp_value);	  //Return data
}

/*********************************************************************
 * Input: Control Byte, 8 - bit address, data.
 * Write a byte to low density device at address LowAdd
 ********************************************************************/
void i2c1_LD_write_byte(uint8_t ControlByte, uint8_t Address, uint8_t data)
{
  //uint8_t err;

  osSemaphoreAcquire(sid_i2c1_semaphore, osWaitForever);

  i2c1_start();                          // Generate Start COndition
  i2c1_write_byte(ControlByte & 0xFE);   // Write Control byte
  i2c1_wait_ack();
  i2c1_write_byte(Address); 		          // Write start address
  i2c1_wait_ack();
  i2c1_write_byte(data);                 // Write Data
  i2c1_wait_ack();
  i2c1_stop();                           // Initiate Stop Condition

  osSemaphoreRelease(sid_i2c1_semaphore);
}

/*********************************************************************
 * Input: Control Byte, Address, *Data, Length.
 * Performs a low density read a byte
 ********************************************************************/
uint8_t i2c1_LD_read_byte(uint8_t ControlByte, uint8_t Address)
{
  uint8_t dat;
  //uint8 err;

  osSemaphoreAcquire(sid_i2c1_semaphore, osWaitForever);

  i2c1_start(); 	                      // Generate Start Condition
  i2c1_write_byte(ControlByte & 0xFE); // Write Control Byte MAKE SURE IT IS WRITE CONDITON
  i2c1_wait_ack();
  i2c1_write_byte(Address); 		        // Write start address
  i2c1_wait_ack();
  i2c1_restart();                      // Generate restart condition
  i2c1_write_byte(ControlByte | 0x01); // Write control byte for read
  i2c1_wait_ack();
  dat = i2c1_read_byte(); 	            // read a byte
  i2c1_not_ack(); 	                    // Send Not Ack
  i2c1_stop();    	                    // Generate Stop

  osSemaphoreRelease(sid_i2c1_semaphore);

  return dat;
}

/*********************************************************************
 * Input: Control Byte, Address, *Data, Length.
 * Performs a low density read of Length bytes and stores in *Data
 * array starting at Address.
 ********************************************************************/
void i2c1_LD_read_bytes(uint8_t ControlByte, uint8_t Address, uint8_t *Data, uint8_t Length)
{
  uint8_t i = 0;
  //uint8 err;

  osSemaphoreAcquire(sid_i2c1_semaphore, osWaitForever);

  i2c1_start(); 	                      // Generate Start Condition
  i2c1_write_byte(ControlByte & 0xFE); // Write Control Byte MAKE SURE IT IS WRITE CONDITON
  i2c1_wait_ack();
  i2c1_write_byte(Address); 		        // Write start address
  i2c1_wait_ack();
  i2c1_restart();                      // Generate restart condition
  i2c1_write_byte(ControlByte | 0x01); // Write control byte for read
  i2c1_wait_ack();
  for (i=0;i<Length-1;i++)            // read Length number of bytes
  {
    *Data = i2c1_read_byte();
    i2c1_ack();
    Data++;
  }
  *Data = i2c1_read_byte();
  i2c1_not_ack(); 	                    // Send Not Ack
  i2c1_stop();    	                    // Generate Stop

  osSemaphoreRelease(sid_i2c1_semaphore);
}

/*********************************************************************
 * Input: ControlByte, LowAdd, *wrptr ,Length.
 * Write a page of data from array pointed to be wrptr starting at LowAdd
 * Note:  LowAdd must start on a page boundary
 ********************************************************************/
void i2c1_LD_write_bytes(uint8_t ControlByte, uint8_t LowAdd, uint8_t *wrptr,uint8_t Length)
{
  uint8_t i = 0;
  //uint8 err;

  osSemaphoreAcquire(sid_i2c1_semaphore, osWaitForever);

  i2c1_start(); 	                      // Generate Start condition
  i2c1_write_byte(ControlByte & 0xFE); // Write Control Byte MAKE SURE IT IS WRITE CONDITON
  i2c1_wait_ack();
  i2c1_write_byte(LowAdd);             // send low address
  i2c1_wait_ack();
  for (i=0;i<Length;i++)
  {
    i2c1_write_byte(*wrptr); //write a single byte
    i2c1_wait_ack();
    wrptr++;
  }
  i2c1_stop(); 	                      // Generate Stop

  osSemaphoreRelease(sid_i2c1_semaphore);
}

