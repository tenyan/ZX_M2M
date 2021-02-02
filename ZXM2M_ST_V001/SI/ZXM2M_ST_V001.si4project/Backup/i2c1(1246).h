/*****************************************************************************
* FileName: i2c1.h
* Engineer: TenYan
* @Company:  徐工信息智能硬件部
* Date:     2020-09-30
******************************************************************************/
#ifndef I2C1_H_
#define I2C1_H_

//-----头文件调用-------------------------------------------------------------
#include "types.h"
#include "cmsis_os2.h"

/******************************************************************************
 *   Macros
 ******************************************************************************/

/******************************************************************************
* Data Types and Globals
******************************************************************************/
extern osSemaphoreId_t sid_i2c1_semaphore;

/******************************************************************************
 *   Function prototypes
 ******************************************************************************/
void delay_us(uint32_t nTimer);
void delay_ms(uint32_t t);
void i2c1_idle(void);
void i2c1_abort(void);
void i2c1_start(void);
void i2c1_restart(void);
void i2c1_stop(void);
uint8_t i2c1_wait_ack(void);
void i2c1_not_ack(void);
void i2c1_ack(void);
void i2c1_write_byte(uint8_t data);
uint8_t i2c1_read_byte(void);
void i2c1_LD_write_byte(uint8_t ControlByte, uint8_t Address, uint8_t data);
uint8_t i2c1_LD_read_byte(uint8_t ControlByte, uint8_t Address);
void i2c1_LD_read_bytes(uint8_t ControlByte, uint8_t Address, uint8_t *Data, uint8_t Length);
void i2c1_LD_write_bytes(uint8_t ControlByte, uint8_t LowAdd, uint8_t *wrptr,uint8_t Length);

void i2c1_Initialize(void);

#endif /* I2C1_H_ */
