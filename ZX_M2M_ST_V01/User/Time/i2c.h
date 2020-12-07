/*****************************************************************************
* FileName: i2c.h
* Engineer: TenYan
* Date:     2018-11-13
******************************************************************************/
#ifndef I2C_H_
#define I2C_H_

#include  "includes.h"

/******************************************************************************
 *   Macros
 ******************************************************************************/ 
 
/******************************************************************************
* Data Types and Globals
******************************************************************************/
extern OS_EVENT *i2c1_semaphore;

/******************************************************************************
 *   Function prototypes
 ******************************************************************************/
void delay_us(uint32_t nTimer);
void delay_ms(uint32_t t);
void i2c_idle(void);
void i2c_abort(void);
void i2c_start(void);
void i2c_restart(void);
void i2c_stop(void);
uint8_t i2c_wait_ack(void);
void i2c_not_ack(void);
void i2c_ack(void);
void i2c_write_byte(uint8_t data);
uint8_t i2c_read_byte(void);
void i2c_LD_write_byte(uint8_t ControlByte, uint8_t Address, uint8_t data);
uint8_t i2c_LD_read_byte(uint8_t ControlByte, uint8_t Address);
void i2c_LD_read_bytes(uint8_t ControlByte, uint8_t Address, uint8_t *Data, uint8_t Length);
void i2c_LD_write_bytes(uint8_t ControlByte, uint8_t LowAdd, uint8_t *wrptr,uint8_t Length);

void i2c_Initialize(void);

#endif /* I2C_H_ */
