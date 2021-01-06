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

  system_Initialize(); // IO�Ϳ��Ź���ʼ��

  //==�߳������ʼ��======================================
  PcDebug_ServiceInit();
  //Momi_ServiceInit();
  Cellura_ServiceInit();

  //==�����߳�����========================================
  PcDebug_ServiceStart();
  //Momi_ServiceStart();
  Cellura_ServiceStart();
  
#if 0
  pthread_create(&pthreads[PTHREAD_GSM_SATRT_ID],NULL,pthread_gsm_start,NULL); // ��ʼ��GSMģ���߳�
  usleep(10);
  pthread_create(&pthreads[PTHREAD_GSM_READ_AT_ID],NULL,pthread_gsm_read_AT,NULL); // ��ȡGSMģ�鷵�ص�ATָ����Ӧ�����߳�(������ʽ)
  usleep(10);
  pthread_create(&pthreads[PTHREAD_COLLECT_FUNCTION],NULL,pthread_Collect_Function,NULL); // 10msʱ��(������)�߳�
  usleep(10);
  pthread_create(&pthreads[PTHREAD_GSM_RECV_ZX_DATA],NULL,pthread_gsm_RecvZxData,NULL); // ��ȡGSMģ�����ͷ�����(10014)���������߳�(������ʽ)
  usleep(10);
  pthread_create(&pthreads[PTHREAD_GSM_RECV_ZXEP_DATA],NULL,pthread_gsm_RecvZxepData,NULL); // ��ȡGSMģ�����ͻ���������(10012)���������߳�(������ʽ)
  usleep(10);
  pthread_create(&pthreads[PTHREAD_GSM_SOCKET_ZX_SERVICE],NULL,pthread_gsm_SocketZxService,NULL); // ά��Socket����(10msʱ����������ʽ)
  usleep(10);
  pthread_create(&pthreads[PTHREAD_GSM_SOCKET_ZXEP_SERVICE],NULL,pthread_gsm_SocketZxepService,NULL); // ά��Socket����(10msʱ����������ʽ)
  usleep(10);
  pthread_create(&pthreads[PTHREAD_MCU_FUNCTION],NULL,pthread_MCU_Function,NULL);  // ��ȡ������Э������ST�����������߳�(������ʽ)
  usleep(10);
  pthread_create(&pthreads[PTHREAD_PC_DEBUG_FUNCTION],NULL,pthread_PcDebug_Function,NULL); // ��ȡ������RS232�����������߳�(������ʽ)
  usleep(10);
  pthread_create(&pthreads[PTHREAD_SYS_TIMER],NULL,pthread_SYS_Timer,NULL); // 100msʱ��(����)�߳�
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
* ���Ź�
******************************************************************************/
//==ι����ʼ��(1���Ӳ�Ϊ��ϵͳ��λ)=================================================
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

//==ι��===========================================================================
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

//==�ر�ι������==================================================================
void WDG_Close(void)
{
  close(fd_watchdog);
}

/******************************************************************************
* GPIO�˿�
******************************************************************************/
//==GPIO�ܽŷ�������(1:��� 0:����)=================================================
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

//==GPIO�ܽ������ֵ(1:��  0:��)=================================================
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

//==gpio�����ļ���ʼ��============================================================
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

//==IO��ʼ������================================================================
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
* IO�Ϳ��Ź���ʼ��
******************************************************************************/
void system_Initialize(void)
{
  gpio_Initialize();
  WDG_Initialize();
}


