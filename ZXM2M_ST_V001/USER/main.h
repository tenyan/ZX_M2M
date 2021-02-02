/*****************************************************************************
* Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
* @FileName: main.h
* @Engineer: TenYan
* @Company:  徐工信息智能硬件部
* @version   V1.0
* @Date:     2018-12-20
* @brief
*
* 用户名:tenyan     密码:yan111xgit
* 国六网址:http://10.80.0.108:1010/login
* 国六平台: 120.195.166.245  端口 10012
*
* 用户名:tenyan     密码:yan111xgit
* XM2M 网址: http://10.80.0.101:8010/Home/Login?returnUrl=%2F
* XM2M 平台: 58.218.196.200  端口 10004
******************************************************************************/
#ifndef MAIN_H_
#define MAIN_H_

/******************************************************************************
* Includes
******************************************************************************/
#include "types.h"
#include "stm32f2xx.h"

/******************************************************************************
 *   Pre-Processor Defines
 ******************************************************************************/
//#define _disable_interrupts() __ASM volatile("cpsid i") // 关闭所有中断
//#define _enable_interrupts()	 __ASM volatile("cpsie i") // 开启所有中断

#define _disable_interrupts()  __set_PRIMASK(1) // 关闭所有中断
#define _enable_interrupts()	 __set_PRIMASK(0) // 开启所有中断

//保证任务进程不被其他中断干扰
// taskENTER_CRITICAL(); // 进入临界区
// taskEXIT_CRITICAL();  // 退出临界区


// 保护一段代码不被其他任务打断，并不能约束中断
// vTaskSuspendAll(void);   //挂起调度器创建临界区(禁止任务调度 suspend the scheduler)
// vTaskResumeAll(void);  //（解除禁止任务调度 resuming the scheduler； 成对使用）
// 注意1：两个函数成对使用；
// 注意2：两者之间不能调用FreeRTOS系统API.

#define KR_KEY_Reload   ((uint16_t)0xAAAA)
#define ClrWdt()        do{IWDG->KR = KR_KEY_Reload;}while(0)
#define IWDG_Feed()     do{IWDG->KR = KR_KEY_Reload;}while(0)

/******************************************************************************
 *  Misc defines for clarity
 ******************************************************************************/
#define DTU_BUADRATE       115200
#define LCD_BUADRATE       115200

/*****************************************************************************
* Port pin defines
******************************************************************************/
/****************** A Register *******************/
//#define RTC_WAKE_UP     PA0   //input (INT)
//#define WIFI_LED        PA1   //output
//#define CAN_POWER_CTL   PA2   //output
//#define ETH_LED         PA3   //output
//#define MODEM_VBUS_CTL  PA4   //output
//#define GPS_LED         PA5   //output
//#define CAN1_LED        PA6   //output
//#define CAN2_LED        PA7   //output
//#define MODEM_RING      PA8   //input (INT)
//#define DBG_TXD1        PA9   //output
//#define DBG_RXD1        PA10  //input
//#define CAN2_RXD        PA11  //input
//#define CAN2_TXD        PA12  //output
//#define SWDIO           PA13  //output
//#define SWCLK           PA14  //output
//#define MODEM_POWER_CTL PA15  //output

// OUTPUT
#define GPIOA_OUTPUT_PIN      (GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | \
                                GPIO_Pin_9 | GPIO_Pin_12 | GPIO_Pin_15)

#define WIFI_LED_PORT   GPIOA
#define WIFI_LED_PIN    GPIO_Pin_1
#define WIFI_LED_OFF()  WIFI_LED_PORT->BSRRL = WIFI_LED_PIN  // 高电平灭
#define WIFI_LED_ON()   WIFI_LED_PORT->BSRRH = WIFI_LED_PIN  // 低电平亮

#define CAN_POWER_PORT   GPIOA
#define CAN_POWER_PIN    GPIO_Pin_2
#define CAN_POWER_ON()   CAN_POWER_PORT->BSRRL = CAN_POWER_PIN  // 高电平开电
#define CAN_POWER_OFF()  CAN_POWER_PORT->BSRRH = CAN_POWER_PIN // 低电平关电

#define ETH_LED_PORT   GPIOA
#define ETH_LED_PIN    GPIO_Pin_3
#define ETH_LED_OFF()  ETH_LED_PORT->BSRRL = ETH_LED_PIN  // 高电平灭
#define ETH_LED_ON()   ETH_LED_PORT->BSRRH = ETH_LED_PIN  // 低电平亮

#define MODEM_VBUS_PORT   GPIOA
#define MODEM_VBUS_PIN    GPIO_Pin_4
#define MODEM_VBUS_ON()   MODEM_VBUS_PORT->BSRRL = MODEM_VBUS_PIN  // 高电平开电
#define MODEM_VBUS_OFF()  MODEM_VBUS_PORT->BSRRH = MODEM_VBUS_PIN // 低电平关电

#define GPS_LED_PORT   GPIOA
#define GPS_LED_PIN    GPIO_Pin_5
#define GPS_LED_OFF()  GPS_LED_PORT->BSRRL = GPS_LED_PIN  // 高电平灭
#define GPS_LED_ON()   GPS_LED_PORT->BSRRH = GPS_LED_PIN  // 低电平亮

#define CAN1_LED_PORT   GPIOA
#define CAN1_LED_PIN    GPIO_Pin_6
#define CAN1_LED_OFF()  CAN1_LED_PORT->BSRRL = CAN1_LED_PIN  // 高电平灭
#define CAN1_LED_ON()   CAN1_LED_PORT->BSRRH = CAN1_LED_PIN  // 低电平亮

#define CAN2_LED_PORT   GPIOA
#define CAN2_LED_PIN    GPIO_Pin_7
#define CAN2_LED_OFF()  CAN2_LED_PORT->BSRRL = CAN2_LED_PIN  // 高电平灭
#define CAN2_LED_ON()   CAN2_LED_PORT->BSRRH = CAN2_LED_PIN  // 低电平亮

#define MODEM_PWR_PORT       GPIOA
#define MODEM_PWR_PIN        GPIO_Pin_15
#define ENABLE_MODEM_PWR()   MODEM_PWR_PORT->BSRRL = MODEM_PWR_PIN  // 拉高供电
#define DISABLE_MODEM_PWR()  MODEM_PWR_PORT->BSRRH = MODEM_PWR_PIN  // 拉低断电

//INPUT
#define GPIOA_INPUT_PIN     (GPIO_Pin_0 | GPIO_Pin_8 | GPIO_Pin_10 | GPIO_Pin_11)

#define RTC_ALARM_IN_PORT   GPIOA
#define RTC_ALARM_IN_PIN    GPIO_Pin_0
#define RTC_ALARM_IN()      (RTC_ALARM_IN_PORT->IDR & RTC_ALARM_IN_PIN)

/****************** B Register *******************/
//#define POWER_LED        PB0   //output
//#define DIP_IN           PB1   //input
//#define NC               PB2   //output BOOT1
//#define GPS_ANT_OPEN_IN  PB3   //input
//#define GPS_POWER_CTL    PB4   //output
//#define GPS_ANT_SHORT_IN PB5   //input
//#define RTC_SLC1         PB6   //output(开漏)
//#define RTC_SDA1         PB7   //output(开漏)
//#define MAIN_POWER_CTL   PB8   //output
//#define SPI2_CS          PB9   //output
//#define SPI2_SCK         PB10  //output
//#define BAT_CHARGE_CTL   PB11  //output
//#define CAN2_RX          PB12  //input
//#define CAN2_TX          PB13  //output
//#define MODEM_PWR_KEY    PB14  //output
//#define MODEM_STATUS_IN  PB15  //input

// OUTPUT
#define GPIOB_OUTPUT_PIN      (GPIO_Pin_0 | GPIO_Pin_2 | GPIO_Pin_4  | GPIO_Pin_6 | GPIO_Pin_7 |  GPIO_Pin_8 | GPIO_Pin_9 |\
                                            GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14)

#define POWER_LED_PORT       GPIOB
#define POWER_LED_PIN        GPIO_Pin_0
#define POWER_LED_OFF()      POWER_LED_PORT->BSRRL = POWER_LED_PIN  // 高电平灭
#define POWER_LED_ON()       POWER_LED_PORT->BSRRH = POWER_LED_PIN  // 低电平亮

#define MAIN_POWER_PORT      GPIOB
#define MAIN_POWER_PIN       GPIO_Pin_8
#define MAIN_POWER_ON()      MAIN_POWER_PORT->BSRRH = MAIN_POWER_PIN // 低电平,开主电
#define MAIN_POWER_OFF()     MAIN_POWER_PORT->BSRRL = MAIN_POWER_PIN // 高电平,关主电

#define FLASH_CS_PORT        GPIOB
#define FLASH_CS_PIN         GPIO_Pin_9
#define FLASH_CS_H()         FLASH_CS_PORT->BSRRL = FLASH_CS_PIN
#define FLASH_CS_L()         FLASH_CS_PORT->BSRRH  = FLASH_CS_PIN

#define BAT_CHARGE_PORT      GPIOB
#define BAT_CHARGE_PIN       GPIO_Pin_11
#define BAT_CHARGE_OFF()     BAT_CHARGE_PORT->BSRRL = BAT_CHARGE_PIN // 高电平,关闭充电
#define BAT_CHARGE_ON()      BAT_CHARGE_PORT->BSRRH = BAT_CHARGE_PIN // 低电平,开启充电

#define MODEM_PWRKEY_PORT    GPIOB
#define MODEM_PWRKEY_PIN     GPIO_Pin_14
//#define MODEM_PWRKEY_H()   MODEM_PWRKEY_PORT->BSRRH = MODEM_PWRKEY_PIN // 极性相反
//#define MODEM_PWRKEY_L()   MODEM_PWRKEY_PORT->BSRRL = MODEM_PWRKEY_PIN

#define MODEM_STATUS_IN_PORT   GPIOB
#define MODEM_STATUS_IN_PIN    GPIO_Pin_15
//#define MODEM_STATUS_IN()      (MODEM_STATUS_IN_PORT->IDR & MODEM_STATUS_IN_PIN)

//==================================================================
#define I2C1_SCL_PORT  GPIOB  // 开漏
#define I2C1_SCL_PIN   GPIO_Pin_6
#define I2C1_SCL_H()   I2C1_SCL_PORT->BSRRL = I2C1_SCL_PIN
#define I2C1_SCL_L()   I2C1_SCL_PORT->BSRRH  = I2C1_SCL_PIN

#define I2C1_SDA_PORT  GPIOB  // 开漏
#define I2C1_SDA_PIN   GPIO_Pin_7
#define I2C1_SDA_H()   I2C1_SDA_PORT->BSRRL = I2C1_SDA_PIN
#define I2C1_SDA_L()   I2C1_SDA_PORT->BSRRH  = I2C1_SDA_PIN

#define I2C1_SDA_IN    (I2C1_SDA_PORT->IDR & I2C1_SDA_PIN)

//INPUT
#define GPIOB_INPUT_PIN       (GPIO_Pin_1 | GPIO_Pin_3 | GPIO_Pin_5 | GPIO_Pin_12 | GPIO_Pin_15)

#define DIP_IN_PORT   GPIOB
#define DIP_IN_PIN    GPIO_Pin_1
#define DIP_IN()      (DIP_IN_PORT->IDR & DIP_IN_PIN)

#define GPS_ANT_OPEN_IN_PORT   GPIOB
#define GPS_ANT_OPEN_IN_PIN    GPIO_Pin_3
#define GPS_ANT_OPEN_IN()      (GPS_ANT_OPEN_IN_PORT->IDR & GPS_ANT_OPEN_IN_PIN)

#define GPS_ANT_SHORT_IN_PORT   GPIOB
#define GPS_ANT_SHORT_IN_PIN    GPIO_Pin_5
#define GPS_ANT_SHORT_IN()      (GPS_ANT_SHORT_IN_PORT->IDR & GPS_ANT_SHORT_IN_PIN)

/****************** C Register *******************/
//#define VBAT_MEAS     PC0   //input  ADC123IN10
//#define UBS_MEAS      PC1   //input  ADC123IN11
//#define SPI2_MISO     PC2   //input
//#define SPI2_MOSI     PC3   //output
//#define EMMC_POWER_CTL PC4   //output
//#define ACC_IN        PC5   //input
//#define MODEM_AUX_TXD6 PC6   //output
//#define MODEM_AUX_RXD6 PC7   //input
//#define MODEM_RESET   PC8   //output
//#define MODEM_DTR     PC9   //output
//#define MODEM_TXD3    PC10  //output
//#define MODEM_RXD3    PC11  //input
//#define GPS_TXD5      PC12  //output
//#define NC            PC13  //output
//#define NC            PC14  //output
//#define NC            PC15  //output

// OUTPUT
#define GPIOC_OUTPUT_PIN      ( GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_6 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 |\
                                              GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15)

#define MODEM_EMMC_PORT   GPIOC
#define MODEM_EMMC_PIN    GPIO_Pin_4
#define MODEM_EMMC_ON()   MODEM_EMMC_PORT->BSRRL = MODEM_EMMC_PIN // 高电平开电
#define MODEM_EMMC_OFF()  MODEM_EMMC_PORT->BSRRH = MODEM_EMMC_PIN // 低电平关电

#define GPS_POWER_PORT      GPIOC
#define GPS_POWER_PIN       GPIO_Pin_1
#define ENABLE_GPS_PWR()    GPS_POWER_PORT->BSRRL = GPS_POWER_PIN
#define DISABLE_GPS_PWR()   GPS_POWER_PORT->BSRRH = GPS_POWER_PIN

#define MODEM_RESET_PORT   GPIOC
#define MODEM_RESET_PIN    GPIO_Pin_8
//#define MODEM_RESET_H()    MODEM_RESET_PORT->BSRRH = MODEM_RESET_PIN // 极性相反
//#define MODEM_RESET_L()    MODEM_RESET_PORT->BSRRL = MODEM_RESET_PIN

// 拉低DTR可以唤醒模块
#define MODEM_DTR_PORT   GPIOC
#define MODEM_DTR_PIN    GPIO_Pin_9
//#define MODEM_DTR_H()   MODEM_DTR_PORT->BSRRL = MODEM_DTR_PIN
//#define MODEM_DTR_L()   MODEM_DTR_PORT->BSRRH = MODEM_DTR_PIN

//INPUT
#define GPIOC_INPUT_PIN      (GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_5 | GPIO_Pin_7 | GPIO_Pin_11)

#define ACC_IN_PORT   GPIOC
#define ACC_IN_PIN    GPIO_Pin_5
#define ACC_IN()      (ACC_IN_PORT->IDR & ACC_IN_PIN)

/******************************************************************************
 *   Data Types
 ******************************************************************************/


/******************************************************************************
 *   Application Specific Globals
 ******************************************************************************/


/******************************************************************************
 *   Function prototypes
 ******************************************************************************/
void gpio_Initialize(void);
void system_Initialize(void);

void delay_ms(uint32_t t);
void delay_us(uint32_t nTimer);

void IWDG_Init(void);

//void iRTC_Initialize(void);
//void iRTC_EnalbeWakeupCnt(void);
//void iRTC_DisalbeWakeupCnt(void);

#endif /* MAIN_H_ */

