/*
 * Copyright(c)2019, �칤��Ϣ�����ɷ����޹�˾��������ҵ��
 * All right reserved
 *
 * �ļ�����: gpio_cfg.h
 * �汾��  : V1.0
 * �ļ���ʶ: 
 * �ļ�����: �û�����ͷ�ļ�����
 *
 *-----------------------------------------------------------------------------
 * �޸ļ�¼
 *-----------------------------------------------------------------------------
 *
 * 2019-03-16,  �������ļ�
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