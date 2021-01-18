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
int32_t fd_watchdog = -1;


/******************************************************************************
* Task Define
******************************************************************************/
pthread_t pthreads[NUMBER_OF_PTHREADS];

/******************************************************************************
* 看门狗
******************************************************************************/
//==喂狗初始化(1分钟不为狗系统复位)=================================================
int32_t WDG_Initialize(void)
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
    static uint8_t food = 0;
    //ssize_t eaten = write(fd_watchdog, &food, 1);
    int32_t eaten = write(fd_watchdog, &food, 1);
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

  ret = gpio_file_create(NET_LED_GPIO);
  ret += gpio_set_dir(NET_LED_GPIO,1);
  ret += gpio_set_value(NET_LED_GPIO,1);

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
  //WDG_Initialize();
}

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
  Momi_ServiceInit();
  GPS_ServiceInit();
  Cellura_ServiceInit();
  Collect_ServiceInit();
  NetSocket_Init();
  ZxM2m_ServiceInit();
  HJEP_ServiceInit();
  AuxCom_ServiceInit();

  //==启动线程任务========================================
  PcDebug_ServiceStart();
  Momi_ServiceStart();
  GPS_ServiceStart();
  Cellura_ServiceStart();
  Collect_ServiceStart();
  ZxM2m_ServiceStart();
  HJEP_ServiceStart();
  AuxCom_ServiceStart();
  
  usleep(100);
  for (i=0; i<NUMBER_OF_PTHREADS; i++)
  {
    pthread_join(pthreads[i],NULL);
  }

  return 0;
}


