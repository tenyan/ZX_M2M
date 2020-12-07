

/*
 * Copyright(c)2011, �����칤��Ϣ�Ͼ��з�Ӳ����
 * All right reserved
 *
 * �ļ�����: gpio_cfg.c
 * �汾��  : V1.0
 * �ļ���ʶ:
 * �ļ�����: ���ļ�ΪT-box gpio ��ʼ���������ļ�
 *
 *-----------------------------------------------------------------------------
 * �޸ļ�¼
 *-----------------------------------------------------------------------------
 *
 * 2020-3-01, �������ļ�
 *
 */

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <sys/poll.h>
#include <sys/stat.h>
#include <string.h>
#include <errno.h>
#include <termios.h>
#include <linux/input.h>
#include <linux/ioctl.h>
#include <pthread.h>
//#include "GpioControl.h"
//#include "LedControl.h"
#include "config.h"


//#define GPIO77_LOCKNAME "GPIO77_Wakeup_modem"
//#define GPIO74_LOCKNAME "GPIO74_Wakeup_modem"
//#define GPIO_SEELP_MODEM_MS    200

/************************************************************
* Function Name  : gpio_set_dir
* Description    :  GPIO�ܽŷ�������
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*************************************************************/	
int gpio_set_dir(unsigned int gpio, unsigned int out_flag)
{
	int fd;
	char buf[MAX_BUF];
 
	snprintf(buf, sizeof(buf), "/sys/class/gpio/gpio%d/direction", gpio);
 
	fd = open(buf, O_WRONLY);
	if (fd < 0) {
		printf("gpio(%d)/direction\n",gpio);
		return fd;
	}
 
	if (out_flag)
		write(fd, "out", 4);
	else
		write(fd, "in", 3);
 
	close(fd);
	return 0;
}
/************************************************************
* Function Name  : gpio_set_dir
* Description    :  GPIO�ܽ������ֵ
* Input          : value  1:��  0:��
* Output         : None
* Return         : None
* Attention		 : None
*************************************************************/	
int gpio_set_value(unsigned int gpio, unsigned int value)
{
	int fd;
	char buf[MAX_BUF];
 
	snprintf(buf, sizeof(buf), "/sys/class/gpio/gpio%d/value", gpio);
 
	fd = open(buf, O_WRONLY);
	if (fd < 0) {
		perror("gpio/set-value\n");
		return fd;
	}
 
	if (value)
		write(fd, "1", 2);
	else
		write(fd, "0", 2);
 
	close(fd);
	return 0;
}
/************************************************************
* Function Name  : gpio_file_create
* Description    : gpio �����ļ���ʼ��
* Input          : value  1:��  0:��
* Output         : None
* Return         : None
* Attention		 : None
*************************************************************/	
int gpio_file_create(int gpio)
{
	int sfd = -1;
	char checkstr[50] = {0};
	char configstr[10] = {0};
	int reto,len;

	sprintf(checkstr,"/sys/class/gpio/gpio%d/value",gpio);
	if(0 == access(checkstr, F_OK)){
		return 0;
	}

	sfd = open("/sys/class/gpio/export",O_WRONLY);
	if(sfd < 0){
		printf("%s:%d,open file error,%d\n",__FUNCTION__,__LINE__,sfd);
		return sfd;
	}

	len = sprintf(configstr,"%d",gpio);
	reto = write(sfd,configstr,len);

	if(reto != len){
		printf("create gpio(%d) files:%d,%d,%d\n",gpio,__LINE__,len,reto);
		return reto;
	}
	usleep(1000);
	reto = access(checkstr, F_OK);
	if(0 > reto){
		printf("%s:%d,gpio file(%s)not exist\n",__FUNCTION__,__LINE__,checkstr);
	}

	close(sfd);
	return reto;
}
/************************************************************
* Function Name  : gpio_init
* Description    : gpio ��ʼ��
* Input          : value  1:��  0:��
* Output         : None
* Return         : None
* Attention		 : None
*************************************************************/	
int gpio_init(void)
{
	int ret;
	ret = gpio_file_create(GSM_LED_GPIO);
	ret += gpio_set_dir(GSM_LED_GPIO,1);
	ret += gpio_set_value(GSM_LED_GPIO,1);

	ret += gpio_file_create(USER_LED_GPIO);
	ret += gpio_set_dir(USER_LED_GPIO,1);
	ret += gpio_set_value(USER_LED_GPIO,0);
	//ret += gpio_set_isr(USER_LED_GPIO);
	//ret += gpio_can_wakeup(USER_LED_GPIO,1);
	return ret;
}	
#if 0
/************************************************************
* Function Name  : GPIO_SetBits
* Description    : GPIO �ܽ�����
* Input          : GPIOx :gpio�ܽű�� ��gpio_cfg.h�궨��
* Output         : None
* Return         : None
* Attention		 : None
*************************************************************/	
void GPIO_SetBits(uint8 GPIOx)
{
	struct gpiohandle_data data;
	uint8 num=0;
	int ret;
	
    num=Get_Line_Num(GPIOx);  
	data.values[0] = HIGH;
	ret =ioctl(req[num].fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data);

	if (ret < 0)
		printf("ERROR set line value ret=%d\n", ret);
}
/************************************************************
* Function Name  : GPIO_ResetBits
* Description    : GPIO �ܽ�����
* Input          : GPIOx :gpio�ܽű�� ��gpio_cfg.h�궨��
* Output         : None
* Return         : None
* Attention		 : None
*************************************************************/	
void GPIO_ResetBits(uint8 GPIOx)
{
   struct gpiohandle_data data;
	uint8 num=0;
	int ret;
	
    num=Get_Line_Num(GPIOx);  
	data.values[0] = LOW;
	ret =ioctl(req[num].fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data);

	if (ret < 0)
		printf("ERROR Reset line value ret=%d\n", ret);	
    
}

/************************************************************
* Function Name  : GPIO_ReadInputDataBit
* Description    : GPIO �ܽ�����ֵ��ȡ
* Input          : GPIOx :gpio�ܽű�� ��gpio_cfg.h�궨��
* Output         : None
* Return         : �ܽŵ�ƽ״̬ 0:�� 1:��
* Attention		 : None
*************************************************************/	
uint8 GPIO_ReadInputDataBit(uint8 GPIOx)
{
	struct gpiohandle_data data;
    int ret;
	uint8 num=0;
	
    num=Get_Line_Num(GPIOx); 	
	ret = ioctl(req[num].fd, GPIOHANDLE_GET_LINE_VALUES_IOCTL, &data);

	if (ret < 0)
		printf("ERROR GET line value ret=%d\n", ret);
	
	return  data.values[0];
	    
}


#endif

/************************************************************
* Function Name  : GPIO_Chip_Init
* Description    : MPU GPIO�ܽų�ʼ��
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*************************************************************/	
int GPIO_Chip_Init(void)
{
	gpio_init();
    
	return 1;
}
#if 0
int main(void)
{
    int i=0;
	GPIO_Chip_Init();

  while(1)
  {
      gpio_set_value(GSM_LED_GPIO,1);
	  gpio_set_value(USER_LED_GPIO,1);
	  sleep(2);
	 // printf("hello world i=%d\n",i++);
	  gpio_set_value(GSM_LED_GPIO,0);
	  gpio_set_value(USER_LED_GPIO,0);
	  sleep(2);
  }
   return 1;
}
#endif

