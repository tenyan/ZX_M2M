/*****************************************************************************
* @FileName: main.h
* @Engineer: TenYan
* @version   V1.0
* @Date:     2020-12-25
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

#include <pthread.h>

/******************************************************************************
 *   Pre-Processor Defines
 ******************************************************************************/
#define NET_LED_GPIO   79 //sim7600.pin52
#define USER_LED_GPIO  77 //sim7600.pin87
#define MAX_GPIO_BUF   128

#define NET_LED_ON()     gpio_set_value(NET_LED_GPIO,1)
#define NET_LED_OFF()    gpio_set_value(NET_LED_GPIO,0)

#define USER_LED_ON()    gpio_set_value(USER_LED_GPIO,1)
#define USER_LED_OFF()   gpio_set_value(USER_LED_GPIO,0)

/******************************************************************************
 *   Data Types
 ******************************************************************************/
 

/******************************************************************************
 *   Application Specific Globals
 ******************************************************************************/
enum
{
  PTHREAD_PC_DEBUG_ID = 0x00,
  PTHREAD_GPS_PROCESS_ID,
  PTHREAD_MOMI_PROCESS_ID,
  PTHREAD_MOMI_PRODUCE_ID,
  PTHREAD_CELLURA_PROCESS_ID,
  PTHREAD_CELLURA_PRODUCE_ID,
  PTHREAD_ZXM2M_SOCKET_SERVICE_ID,
  PTHREAD_ZXM2M_PRODUCE_ID,
  PTHREAD_ZXM2M_PROCESS_ID,
  PTHREAD_HJEP_SOCKET_SERVICE_ID,
  PTHREAD_HJEP_PROCESS_ID,
  PTHREAD_HJEP_PRODUCE_ID,
  PTHREAD_COLLECT_ID,
  //PTHREAD_SYS_TIMER,
  //PTHREAD_A5_UART0_FUNCTION,
  NUMBER_OF_PTHREADS
};
extern pthread_t pthreads[NUMBER_OF_PTHREADS];

/******************************************************************************
 *   Function prototypes
 ******************************************************************************/
void WDG_Feed(void);
int gpio_set_value(unsigned int gpio, unsigned int value);

#endif /* MAIN_H_ */

