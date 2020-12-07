/*
 * Copyright(c)2019, 徐工信息技术股份有限公司物联网事业部
 * All right reserved
 *
 * 文件名称: gpio_cfg.h
 * 版本号  : V1.0
 * 文件标识: 
 * 文件描述: 用户代码头文件包含
 *
 *-----------------------------------------------------------------------------
 * 修改记录
 *-----------------------------------------------------------------------------
 *
 * 2019-03-16,  创建本文件
 *
 */

#ifndef CFG_GPIO_H
#define CFG_GPIO_H

#define GSM_LED_GPIO   79 //sim7600.pin52
#define USER_LED_GPIO  77 //sim7600.pin87
#define MAX_BUF 128


#define GSM_LED_On()		 gpio_set_value(GSM_LED_GPIO,1)
#define GSM_LED_Off()		 gpio_set_value(GSM_LED_GPIO,0)

#define USER_LED_On() 		 gpio_set_value(USER_LED_GPIO,1)
#define USER_LED_Off()		 gpio_set_value(USER_LED_GPIO,0)


int GPIO_Chip_Init(void);
int gpio_set_value(unsigned int gpio, unsigned int value);
int gpio_set_dir(unsigned int gpio, unsigned int out_flag);
int gpio_file_create(int gpio);

//uint8 Get_Line_Num(uint8 GPIOx);
//int GPIO_Line_Init(uint8 GPIOx,uint32 direc,uint8 data);



#endif