/*****************************************************************************
* Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
* @FileName: main.c
* @Engineer: TenYan
* @version   V1.0
* @Date:     2020-12-24
* @brief     Main program body.
* @Micro
******************************************************************************/

/******************************************************************************
* Includes
******************************************************************************/
#include "config.h"

/******************************************************************************
* Typedef
******************************************************************************/


/******************************************************************************
* Define
******************************************************************************/


/******************************************************************************
* Macros
******************************************************************************/


/******************************************************************************
* Data Types and Globals
******************************************************************************/
int32 fd_watchdog = -1;


/******************************************************************************
* Task Define
******************************************************************************/
pthread_t pthreads[NUMBER_OF_PTHREADS] = {0};


/******************************************************************************
* This is the standard entry point for C code.
* It is assumed that your code will call main() 
* once you have performed all necessary initialization.
******************************************************************************/
int main(void)
{
  int i = 0;
  printf("ZxM2mSysterm Start!\n");

  system_Initialize(); // IO和看门狗初始化

  //==线程任务初始化======================================
  PcDebug_ServiceInit();
  //Momi_ServiceInit();
  Cellura_ServiceInit();

  //==启动线程任务========================================
  PcDebug_ServiceStart();
  //Momi_ServiceStart();
  Cellura_ServiceStart();
  
#if 0
  pthread_create(&pthreads[PTHREAD_GSM_SATRT_ID],NULL,pthread_gsm_start,NULL); // 初始化GSM模块线程
  usleep(10);
  pthread_create(&pthreads[PTHREAD_GSM_READ_AT_ID],NULL,pthread_gsm_read_AT,NULL); // 读取GSM模块返回的AT指令响应数据线程(阻塞方式)
  usleep(10);
  pthread_create(&pthreads[PTHREAD_COLLECT_FUNCTION],NULL,pthread_Collect_Function,NULL); // 10ms时基(无阻塞)线程
  usleep(10);
  pthread_create(&pthreads[PTHREAD_GSM_RECV_ZX_DATA],NULL,pthread_gsm_RecvZxData,NULL); // 读取GSM模块重型服务器(10014)下行数据线程(阻塞方式)
  usleep(10);
  pthread_create(&pthreads[PTHREAD_GSM_RECV_ZXEP_DATA],NULL,pthread_gsm_RecvZxepData,NULL); // 读取GSM模块重型环保服务器(10012)下行数据线程(阻塞方式)
  usleep(10);
  pthread_create(&pthreads[PTHREAD_GSM_SOCKET_ZX_SERVICE],NULL,pthread_gsm_SocketZxService,NULL); // 维护Socket连接(10ms时基非阻塞方式)
  usleep(10);
  pthread_create(&pthreads[PTHREAD_GSM_SOCKET_ZXEP_SERVICE],NULL,pthread_gsm_SocketZxepService,NULL); // 维护Socket连接(10ms时基非阻塞方式)
  usleep(10);
  pthread_create(&pthreads[PTHREAD_MCU_FUNCTION],NULL,pthread_MCU_Function,NULL);  // 读取并处理协处理器ST发来的数据线程(阻塞方式)
  usleep(10);
  pthread_create(&pthreads[PTHREAD_PC_DEBUG_FUNCTION],NULL,pthread_PcDebug_Function,NULL); // 读取并处理RS232发来的数据线程(阻塞方式)
  usleep(10);
  pthread_create(&pthreads[PTHREAD_SYS_TIMER],NULL,pthread_SYS_Timer,NULL); // 100ms时基(阻塞)线程
  usleep(10);
  pthread_create(&pthreads[PTHREAD_A5_UART0_FUNCTION],NULL,pthread_A5Uart0_Function,NULL);
#endif
  
  usleep(100);
  for (i=0; i<NUMBER_OF_PTHREADS; i++)
  {
    pthread_join(pthreads[i],NULL);
  }

  return 0;
}

/******************************************************************************
* 看门狗
******************************************************************************/
//==喂狗初始化(1分钟不为狗系统复位)=================================================
int32 WDG_Initialize(void)
{

  fd_watchdog = open("/dev/watchdog", O_WRONLY);
  if (fd_watchdog == -1)
  {
    int err = errno;
    printf("\n!!! FAILED to open /dev/watchdog, errno: %d, %s\n", err, strerror(err));
    syslog(LOG_WARNING, "FAILED to open /dev/watchdog, errno: %d, %s", err, strerror(err));
  }
  return fd_watchdog;
}

//==喂狗===========================================================================
void WDG_Feed(void)
{
  if (fd_watchdog >= 0)
  {

    static uint8 food = 0;
    //ssize_t eaten = write(fd_watchdog, &food, 1);
    int32 eaten = write(fd_watchdog, &food, 1);
    if (eaten != 1)
    {
      puts("\n!!! FAILED feeding watchdog");
      syslog(LOG_WARNING, "FAILED feeding watchdog");
    }
  }
}

//==关闭喂狗功能==================================================================
void WDG_Close(void)
{
  close(fd_watchdog);
}

/******************************************************************************
* GPIO端口
******************************************************************************/
//==GPIO管脚方向设置(1:输出 0:输入)=================================================
int gpio_set_dir(unsigned int gpio, unsigned int out_flag)
{
  int fd;
  char buf[MAX_GPIO_BUF];

  snprintf(buf, sizeof(buf), "/sys/class/gpio/gpio%d/direction", gpio);

  fd = open(buf, O_WRONLY);
  if (fd < 0)
  {
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

//==GPIO管脚输出赋值(1:高  0:低)=================================================
int gpio_set_value(unsigned int gpio, unsigned int value)
{
  int fd;
  char buf[MAX_GPIO_BUF];

  snprintf(buf, sizeof(buf), "/sys/class/gpio/gpio%d/value", gpio);

  fd = open(buf, O_WRONLY);
  if (fd < 0)
  {
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

//==gpio控制文件初始化============================================================
int gpio_file_create(int gpio)
{
  int sfd = -1;
  char checkstr[50] = {0};
  char configstr[10] = {0};
  int reto,len;

  sprintf(checkstr,"/sys/class/gpio/gpio%d/value",gpio);
  if (0 == access(checkstr, F_OK)){
    return 0;
  }

  sfd = open("/sys/class/gpio/export",O_WRONLY);
  if (sfd < 0)
  {
    printf("%s:%d,open file error,%d\n",__FUNCTION__,__LINE__,sfd);
    return sfd;
  }

  len = sprintf(configstr,"%d",gpio);
  reto = write(sfd,configstr,len);

  if (reto != len)
  {
    printf("create gpio(%d) files:%d,%d,%d\n",gpio,__LINE__,len,reto);
    return reto;
  }
  usleep(1000);
  reto = access(checkstr, F_OK);
  if (0 > reto)
  {
    printf("%s:%d,gpio file(%s)not exist\n",__FUNCTION__,__LINE__,checkstr);
  }

  close(sfd);
  return reto;
}

//==IO初始化函数================================================================
int gpio_Initialize(void)
{
  int ret;

  ret = gpio_file_create(GSM_LED_GPIO);
  ret += gpio_set_dir(GSM_LED_GPIO,1);
  ret += gpio_set_value(GSM_LED_GPIO,1);

  ret += gpio_file_create(USER_LED_GPIO);
  ret += gpio_set_dir(USER_LED_GPIO,1);
  ret += gpio_set_value(USER_LED_GPIO,0);

  return ret;
}

/******************************************************************************
* IO和看门狗初始化
******************************************************************************/
void system_Initialize(void)
{
  gpio_Initialize();
  WDG_Initialize();
}


