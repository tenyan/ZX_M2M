/*****************************************************************************
* @FileName: main.h
* @Engineer: TenYan
* @version   V1.0
* @Date:     2020-12-25
* @brief
*
* �û���:tenyan     ����:yan111xgit
* ������ַ:http://10.80.0.108:1010/login
* ����ƽ̨: 120.195.166.245  �˿� 10012
*
* �û���:tenyan     ����:yan111xgit
* XM2M ��ַ: http://10.80.0.101:8010/Home/Login?returnUrl=%2F
* XM2M ƽ̨: 58.218.196.200  �˿� 10004
******************************************************************************/
#ifndef MAIN_H_
#define MAIN_H_

/******************************************************************************
 *   Pre-Processor Defines
 ******************************************************************************/
#define GSM_LED_GPIO   79 //sim7600.pin52
#define USER_LED_GPIO  77 //sim7600.pin87
#define MAX_GPIO_BUF  128

#define GSM_LED_On()		 gpio_set_value(GSM_LED_GPIO,1)
#define GSM_LED_Off()		 gpio_set_value(GSM_LED_GPIO,0)

#define USER_LED_On() 	 gpio_set_value(USER_LED_GPIO,1)
#define USER_LED_Off()	 gpio_set_value(USER_LED_GPIO,0)

/******************************************************************************
 *   Data Types
 ******************************************************************************/


/******************************************************************************
 *   Application Specific Globals
 ******************************************************************************/
enum
{
  PTHREAD_GSM_SATRT_ID = 0x00,
  PTHREAD_GSM_READ_AT_ID,
  PTHREAD_COLLECT_FUNCTION,
  PTHREAD_GSM_RECV_ZX_DATA,
  PTHREAD_GSM_RECV_ZXEP_DATA,
  PTHREAD_GSM_SOCKET_ZX_SERVICE,
  PTHREAD_GSM_SOCKET_ZXEP_SERVICE,
  PTHREAD_MCU_FUNCTION,
  PTHREAD_PC_DEBUG_ID,
  PTHREAD_SYS_TIMER,
  PTHREAD_A5_UART0_FUNCTION,
  NUMBER_OF_PTHREADS
};
extern pthread_t pthreads[NUMBER_OF_PTHREADS];

/******************************************************************************
 *   Function prototypes
 ******************************************************************************/
void WDG_Feed(void);
int gpio_set_value(unsigned int gpio, unsigned int value);

#endif /* MAIN_H_ */

