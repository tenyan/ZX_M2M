/*
 * Copyright(c)2020, 江苏徐工信息技术股份有限公司-智能硬件事业部
 * All right reserved
 *
 * 文件名称: App.c
 * 版本号  : V1.0
 * 文件标识:
 * 文件描述: 
 *
 *-----------------------------------------------------------------------------
 * 修改记录
 *-----------------------------------------------------------------------------
 *
 * 2020-11-25, by  lxf, 创建本文件
 *
 */

/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#include "config.h"

/*
*********************************************************************************************************
*/


pthread_t pthreads[9]  ={0};



/*
*********************************************************************************************************
*                                                main()
*
* Description : This is the standard entry point for C code.  It is assumed that your code will call
*               main() once you have performed all necessary initialization.
*
* Arguments   : none
*
* Returns     : none
*********************************************************************************************************
*/


int main()
{
    int i  =0;


    printf("Sys Start\n");
	
	GPIO_Chip_Init();           //芯片GPIO初始化
	

    pthread_create(&pthreads[i++],NULL,pthread_gsm_init,NULL);
    usleep(10);
    pthread_create(&pthreads[i++],NULL,pthread_gsm_read_AT,NULL);
	usleep(10);
	pthread_create(&pthreads[i++],NULL,pthread_Collect_Function,NULL);
	usleep(10);
	pthread_create(&pthreads[i++],NULL,pthread_gsm_RecvData,NULL);
	usleep(10);
//	pthread_create(&pthreads[i++],NULL,pthread_Gps_Function,NULL);
	usleep(10);
	pthread_create(&pthreads[i++],NULL,pthread_MCU_Function,NULL);
	usleep(10);
	pthread_create(&pthreads[i++],NULL,pthread_PcDebug_Function,NULL);	
	usleep(10);
	pthread_create(&pthreads[i++],NULL,pthread_A5Uart0_Function,NULL);	
	usleep(10);
	pthread_create(&pthreads[i++],NULL,pthread_TaskSysTimer_Function,NULL);	
	usleep(100);
 //   pthread_create(&pthreads[i++],NULL,pthread_gsm_date,(void*)fd);


    for(i=0;i<8;i++)
        pthread_join(pthreads[i],NULL);

	
  //  pthread_mutex_destroy(&gsmLock);
  //  pthread_mutex_destroy(&pthreadLock);

    return 0;
}




