/*
*********************************************************************************************************
*                                              EXAMPLE CODE
*
*                   
*********************************************************************************************************
*/

/*
*********************************************************************************************************

*
* Filename      : app.c
* Version       : V1.00
* Programmer(s) : DC
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#include  <app_cfg.h>
#include  <includes.h>
#include "config.h"
#include "system_stm32f2xx.h"
/*
*********************************************************************************************************
*/

extern uint8 Public_Buf[];
/*
*********************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/

/* ----------------- APPLICATION GLOBALS ------------------ */
static OS_STK AppTaskStartStk[APP_TASK_START_STK_SIZE];
static OS_STK TaskGsmRecvStk[TASK_GSM_RECV_STK_SIZE];
static OS_STK TaskGsmServiceStk[TASK_GSM_SERVICE_STK_SIZE];
static OS_STK TaskSYSStk[TASK_SYSTEM_STK_SIZE];
//static OS_STK TaskGpsStk[TASK_GPS_STK_SIZE];
static OS_STK TaskPcDebugStk[TASK_PCDEBUG_STK_SIZE];		
static OS_STK TaskSysTimerStk[TASK_SYSTIMER_STK_SIZE];
static OS_STK TaskMCUStk[TASK_MCU_CAN_STK_SIZE];
static OS_STK TaskCollectStk[TASK_Collect_STK_SIZE];
static OS_STK TaskA5Com1Stk[TASK_A5COM1_STK_SIZE];
/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/
static  void  AppTaskStart              (void        *p_arg);


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

int main(void)
{
    //BSP_IntDisAll();                                            /* Disable all interrupts.                              */
	Delay(1000);
    CPU_Init();                                                 /* Initialize uC/CPU services.                          */
    
    OSInit();                                                   /* Initialize "uC/OS-II, The Real-Time Kernel"          */

    OSTaskCreateExt((void (*)(void *)) AppTaskStart,            /* Create the start task                                */
                    (void           *) 0,
                    (OS_STK         *)&AppTaskStartStk[APP_TASK_START_STK_SIZE - 1],
                    (INT8U           ) APP_TASK_START_PRIO,
                    (INT16U          ) APP_TASK_START_PRIO,
                    (OS_STK         *)&AppTaskStartStk[0],
                    (INT32U          ) APP_TASK_START_STK_SIZE,
                    (void           *) 0,
                    (INT16U          )(OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR));

#if (OS_TASK_NAME_EN > 0)
    OSTaskNameSet(APP_TASK_START_PRIO, "Start", &err);
#endif

    OSStart();                                                  /* Start multitasking (i.e. give control to uC/OS-II)   */
    return (1);
}


/*
*********************************************************************************************************
*                                          STARTUP TASK
*
* Description : This is an example of a startup task.  As mentioned in the book's text, you MUST
*               initialize the ticker only once multitasking has started.
*
* Arguments   : p_arg   is the argument passed to 'AppTaskStart()' by 'OSTaskCreate()'.
*
* Returns     : none
*
* Notes       : 1) The first line of code is used to prevent a compiler warning because 'p_arg' is not
*                  used.  The compiler should not generate any code for this statement.
*********************************************************************************************************
*/
static  void  AppTaskStart (void *p_arg)
{
	
   (void)p_arg;

    Delay(1000);
    BSP_Init();                                                 /* Init BSP fncts.                                          */
    CPU_Init();                                                 /* Init CPU name & int. dis. time measuring fncts.          */
    Mem_Init();                                                 /* Initialize Memory managment                              */
    BSP_CPU_TickInit();                                         /* Start Tick Initialization                                */
	DEBUG_Uart_Init();
	RTC_Onchip_Init();
	Collect_GPIO_Pin_Init();
	IWdtInit(2);  
    i2c_Initialize();
	Flash_SPI_Config();
	McuInit();
	OSTimeDly(OS_TICKS_PER_SEC);     //增加延时以等待RTC启动
	am_init();
	

    OSTaskCreate(TaskUart3Recv, 
		        (void *)0,
		        (OS_STK *)&TaskGsmRecvStk[TASK_GSM_RECV_STK_SIZE-1],
		        TASK_GSM_RECV_PRIO);
	
	#if 1
	OSTaskCreate(TaskSYS,
		        (void *)0,
		        (OS_STK *)&TaskSYSStk[TASK_SYSTEM_STK_SIZE-1],
		        TASK_SYSTEM_PRIO);
	#endif
	#if 1
	OSTaskCreate(TaskMCU, 
		        (void *)0,
		        (OS_STK *)&TaskMCUStk[TASK_MCU_CAN_STK_SIZE-1],  
		        TASK_MCU_CAN_PRIO);    
	#endif
	#if 1	
	OSTaskCreate(TaskGsmService,
		        (void *)0,
		        (OS_STK *)&TaskGsmServiceStk[TASK_GSM_SERVICE_STK_SIZE-1],
		        TASK_GSM_SERVICE_PRIO);
	#endif	
	#if 1
	OSTaskCreate(TaskPcDebug, 
		        (void *)0,
		        (OS_STK *)&TaskPcDebugStk[TASK_PCDEBUG_STK_SIZE-1],  
		        TASK_PCDEBUG_PRIO );
	#endif


	#if 1
	OSTaskCreate(TaskSysTimer,
        (void *)0,
        (OS_STK *)&TaskSysTimerStk[TASK_SYSTIMER_STK_SIZE-1],
        TASK_SYSTIMER_PRIO);
    #endif
	
    #if 1
	OSTaskCreate(TaskA5_Com1Function,
        (void *)0,
        (OS_STK *)&TaskA5Com1Stk[TASK_A5COM1_STK_SIZE-1],
        TASK_A5COM1_PRIO);
	#endif
	
	#if 1
	OSTaskCreate(TaskCollect,
		        (void *)0,
		        (OS_STK *)&TaskCollectStk[TASK_Collect_STK_SIZE-1],
		        TASK_Collect_PRIO);
	#endif   

    while (DEF_TRUE) {                                         
        OSTimeDlyHMSM(0, 0, 1, 0);
	}
	
}

