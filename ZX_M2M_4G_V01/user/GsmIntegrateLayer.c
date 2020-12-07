/* 
 * Copyright(c)2020, 江苏徐工信息技术股份有限公司-智能硬件事业部
 * All right reserved
 *
 * 文件名称: GsmIntegrateLayer.c
 * 版本号  : V1.0
 * 文件标识:
 * 文件描述: 本文件为GSM模块综合层的实现文件
 *
 *-----------------------------------------------------------------------------
 * 修改历史
 *-----------------------------------------------------------------------------
 *
 * 2020-11-30 by lxf, 创建本文件
 *
 */
#include "config.h"
#include "GsmInterfaceLayer.h" 
//#include "GsmHardWareLayer.h"
#include "GsmProtocolLayer.h"
#include "GsmIntegrateLayer.h"

//-----外部变量定义------------------------------------------------------------
extern STUGsmState f_stuGsmState;
extern stuSerAddr g_stuSerAddr;
extern int gGsm_socketfd;
extern int32 gGsmAT_fd;
extern STUSystem g_stuSystem;
extern STUSYSParamSet g_stuSYSParamSet;
//-----内部变量定义------------------------------------------------------------
STU_GsmOperate GsmOperate; 


uint8 GSM_AT_Recv_Buff[GSM_AT_RECV_BUFF_MAX_SIZE]={0};
int16 GSM_AT_Recv_Buff_Len = 0;
static uint8 f_ucAtSend;			//发送的AT指令
static uint8 f_GsmAtRespState = 0;  //发送AT指令后是否收到应答的标志 0-未收到或者未成,1-成功收到
uint8 GSM_Recv_Buff[GSM_RECV_BUFF_MAX_SIZE];
int16 GSM_Recv_Buff_Len = 0;
int16 GSM_Send_Buff_Len = 0;
uint8 GSM_BuffTemp[GSM_RECV_BUFF_MAX_SIZE*2];
uint8 ucOpenTimes = 0;   //记录开机次数

//int argc;
//char *argv[];


int gsm_read(int fd,char *buf,int size)
{
    int len = 0;
   // pthread_mutex_lock(&gsmLock);
    len=read(fd,buf,size);
   // pthread_mutex_unlock(&gsmLock);
    return len;
}

//-----内部函数定义------------------------------------------------------------

int WriteGsmUartData(int fd,uint8 *data,int size)
{
 //   pthread_mutex_lock(&gsmLock);	
    size=write(fd,data,size);
 //   pthread_mutex_unlock(&gsmLock);

  //  printf("%s\n",data);
 	PC_SendDebugData(data, size, DEBUG_AT);

    return size;
}

uint8 GSM_GetModemWorkingState(void)
{
    return f_stuGsmState.ucModemState;
}
/*GSM模块状态:0=启动,1=拨号,2=创建socket,3=创建成功,
  4=关闭拨号,5=关闭socket,6=快重连,7=休眠,8=重启模块*/
void GSM_SetModemWorkingState(uint8 ucModemState)
{
    f_stuGsmState.ucModemState = ucModemState;
}

void GSM_Variable_Init(void)
{
	uint8 i;
	
   	for(i=0; i<MAX_GPRS_LINK; i++)
		f_stuGsmState.ucLinkState[i] = GPRS_LINK_STATE_CLOSED;
	f_stuGsmState.ucCGATT = 0;
	f_stuGsmState.ucCSQ = 0;
	f_stuGsmState.ucCPIN = 0;
	f_stuGsmState.ucSmsRdy = 0;
	f_stuGsmState.ucGsmRegState = 0;
	f_stuGsmState.ucQMIWwanEnable = 0;
	f_stuGsmState.ucQMIWwanState = 0;
	
}

void GSM_Modem_RST(void)
{
    WriteGsmUartData(gGsmAT_fd, (uint8 *)"AT+CRESET\r", 10);
}


//采用AT+CGPADDR查询第一路ip地址,如果返回有效的IP地址怎进入socket流程
void GSM_Modem_Start(void)
{
    static uint8 ucAtErrCnt = 0;    //收不到AT响应计数
    static uint8 ucSimErrTimes = 0; //记录查询不到SIM卡的次数
    uint16 i = 0;
	uint8 j = 0;
    uint8 atemp[90]={0};
    
    GSM_Variable_Init();   //变量初始化

    if(gGsmAT_fd>=0)
    {
        close(gGsmAT_fd);
        printf("gGsmAT_fd Close: %d \n", gGsmAT_fd);
        gGsmAT_fd = -1; 
		sleep(1);
    }
    OpenGsmAT_DEV_TTY();   //打开AT设备节点
    if(gGsmAT_fd<0)
    {
        return ;
    }   
    
   //开机
    ucOpenTimes++;
    if(ucOpenTimes&&(ucOpenTimes%5)==0)
    {
       //发送AT
        f_GsmAtRespState = 0;
        for(i=0; i<3; i++)
        {
            SetAtSend(ATSEND_CONNECT_CFUN0);
            WriteGsmUartData(gGsmAT_fd, (uint8 *)"AT+CFUN=0\r", 10);    
            sleep(2);   //等待3s
            if(1==f_GsmAtRespState)
                break;
        }
        SetAtSend(ATSEND_IDLE);
        if(0==f_GsmAtRespState)
            return;

        f_GsmAtRespState = 0;
        for(i=0; i<3; i++)
        {
            SetAtSend(ATSEND_CONNECT_CFUN1);
            WriteGsmUartData(gGsmAT_fd, (uint8 *)"AT+CFUN=1\r", 10);    
            sleep(2);   //等待3s
            if(1==f_GsmAtRespState)
                break;
        }
        SetAtSend(ATSEND_IDLE);
        if(0==f_GsmAtRespState)
            return; 
        sleep(5);   //等待5s
    }
    else if(ucOpenTimes>20)   //重启模块   如果模块重启失败??
    {
        ucOpenTimes = 0;
        f_GsmAtRespState = 0;
        for(i=0; i<3; i++)
        {
            SetAtSend(ATSEND_CONNECT_CRESET);
            WriteGsmUartData(gGsmAT_fd, (uint8 *)"AT+CRESET\r", 10);    
            sleep(2);   //等待3s
            if(1==f_GsmAtRespState)
                break;
        }
        SetAtSend(ATSEND_IDLE);
        if(0==f_GsmAtRespState)
            return;     
        sleep(60);  //等待30s
    }
    sleep(3);   //等待3s
                    
   //发送AT
    f_GsmAtRespState = 0;
    for(i=0; i<5; i++)
    {
        SetAtSend(ATSEND_CONNECT_AT);
        WriteGsmUartData(gGsmAT_fd, (uint8 *)"at\r", 3);    
        usleep(1000*One_MilliSecond);   //等待1000ms
        if(1==f_GsmAtRespState)
            break;
    }
            
    SetAtSend(ATSEND_IDLE);
    if(0==f_GsmAtRespState)
    {
        if(ucAtErrCnt++ > 4)
        {
            f_stuGsmState.ucModuleState = 1;
            ucAtErrCnt = 0;
        }
        return;
    }
    else
    {
        f_stuGsmState.ucModuleState = 0;
        ucAtErrCnt = 0;
    }

    sleep(1);
    WriteGsmUartData(gGsmAT_fd, (uint8 *)"AT+CGPS=1\r", 10);    
    sleep(1);
    WriteGsmUartData(gGsmAT_fd, (uint8 *)"AT+CGPSXE=1\r", 12);  
    sleep(1);
    WriteGsmUartData(gGsmAT_fd, (uint8 *)"AT+CGPSINFOCFG=5,3\r", 20);   
    sleep(1);
    f_GsmAtRespState = 0;
    for(i=0; i<3; i++)
    {
        SetAtSend(ATSEND_CONNECT_ATE0);
        WriteGsmUartData(gGsmAT_fd, (uint8 *)"ate0\r", 5);  
        usleep(500*One_MilliSecond);   //等待20ms
        if(1==f_GsmAtRespState)
            break;
    }
    SetAtSend(ATSEND_IDLE);
    if(0==f_GsmAtRespState)
        return;
    

    f_GsmAtRespState = 0;
    for(i=0; i<3; i++)
    {
        SetAtSend(ATSEND_CONNECT_CPIN);
        WriteGsmUartData(gGsmAT_fd, (uint8 *)"at+cpin?\r", 9);  
        usleep(1000*One_MilliSecond);   //等待1s
        if(1==f_GsmAtRespState)
            break;
    }
    SetAtSend(ATSEND_IDLE);
    if(0==f_GsmAtRespState)
    {
        f_stuGsmState.ucCPIN = 0;
        if(ucSimErrTimes++ > 3)
            f_stuGsmState.ucSimErr = 1;
        return;
    }
    else
    {
        ucSimErrTimes = 0;
        f_stuGsmState.ucCPIN = 1;
        f_stuGsmState.ucSimErr = 0;
    }

    f_GsmAtRespState = 0;
    for(i=0; i<3; i++)
    {
        SetAtSend(ATSEND_CONNECT_CIMI);
        WriteGsmUartData(gGsmAT_fd, (uint8 *)"at+cimi\r", 8);   
        usleep(200*One_MilliSecond);   //等待200ms
        if(1==f_GsmAtRespState)
            break;
    }
    SetAtSend(ATSEND_IDLE);
    if(0==f_GsmAtRespState)
        return;

    f_GsmAtRespState = 0;
    for(i=0; i<3; i++)
    {
        SetAtSend(ATSEND_CONNECT_CCID);
        WriteGsmUartData(gGsmAT_fd, (uint8 *)"AT+CCID\r", 8);   
        usleep(200*One_MilliSecond);   //等待20ms
        if(1==f_GsmAtRespState)
            break;
    }
    SetAtSend(ATSEND_IDLE);
    if(0==f_GsmAtRespState)
        return;

#if 1
//	OSTimeDly(OS_TICKS_PER_SEC/10);
    f_GsmAtRespState = 0;
	for(i=0; i<5; i++)
	{
		SetAtSend(ATSEND_CONNECT_APN);
		memcpy(atemp,(uint8 *)"AT+CGDCONT=1,\"IP\",\"",19);
		memcpy(&atemp[19],g_stuSYSParamSet.aucAPN,g_stuSYSParamSet.ucAPNLen);
		atemp[19+g_stuSYSParamSet.ucAPNLen] = '\"';
        atemp[20+g_stuSYSParamSet.ucAPNLen] = '\r';
		WriteGsmUartData(gGsmAT_fd,atemp, 21+g_stuSYSParamSet.ucAPNLen);
		usleep(1000*One_MilliSecond);   //等待1s;
		if(1==f_GsmAtRespState)
			break;
	}
	SetAtSend(ATSEND_IDLE);
	#endif	

    f_GsmAtRespState = 0;
    for(i=0; i<10; i++)
    {
        SetAtSend(ATSEND_CONNECT_CSQ);
        WriteGsmUartData(gGsmAT_fd, (uint8 *)"at+csq\r", 7);    
        usleep(1000*One_MilliSecond);   //等待20ms
        if(1==f_GsmAtRespState)
            break;
    }
    SetAtSend(ATSEND_IDLE);
 //   if(0==f_GsmAtRespState)
 //       return;
    
    f_GsmAtRespState = 0;
    for(i=0; i<3; i++)
    {
        SetAtSend(ATSEND_CONNECT_CREG_SET);
        WriteGsmUartData(gGsmAT_fd, (uint8 *)"at+creg=2\r", 10);    
        usleep(200*One_MilliSecond);   //等待20ms
        if(1==f_GsmAtRespState)
            break;
    }
    SetAtSend(ATSEND_IDLE);
    if(0==f_GsmAtRespState)
        return;

    usleep(200*One_MilliSecond);
    f_GsmAtRespState = 0;
    for(i=0; i<20; i++)
    {
        SetAtSend(ATSEND_CONNECT_CREG);
        WriteGsmUartData(gGsmAT_fd, (uint8 *)"at+creg?\r", 9);  
        usleep(1000*One_MilliSecond);//等待1s
         
        if(1==f_GsmAtRespState)
            break;
    }
    SetAtSend(ATSEND_IDLE);
    if(0==f_GsmAtRespState)
    {   
        f_stuGsmState.ucGsmRegState = 0;
       // return;
    }
    else
    {
        f_stuGsmState.ucGsmRegState = 1;
        f_stuGsmState.ucSmsRdy = 1;
    }

    usleep(500*One_MilliSecond);
    f_GsmAtRespState = 0;
    for(i=0; i<200; i++)
    {
        SetAtSend(ATSEND_CONNECT_CGATT);
        WriteGsmUartData(gGsmAT_fd, (uint8 *)"at+cgatt?\r", 10);    
        usleep(1000*One_MilliSecond);//等待20ms
        if(1==f_GsmAtRespState)
            break;
    }
    SetAtSend(ATSEND_IDLE);
    if(0==f_GsmAtRespState)
        return;
	
    f_GsmAtRespState = 0;
    for(i=0; i<3; i++)
    {
        SetAtSend(ATSEND_CONNECT_CPSI);
        WriteGsmUartData(gGsmAT_fd, (uint8 *)"AT+CPSI?\r", 9);    
        usleep(1000*One_MilliSecond);//等待20ms
        if(1==f_GsmAtRespState)
            break;
    }
    SetAtSend(ATSEND_IDLE);
   // if(0==f_GsmAtRespState)
  //      return;
	usleep(100*One_MilliSecond);

    f_GsmAtRespState = 0;
    for(i=0; i<30; i++)
    {
        SetAtSend(ATSEND_CONNECT_CGPADDR);
        WriteGsmUartData(gGsmAT_fd, (uint8 *)"AT+CGPADDR\r", 11);    
        sleep(2);                 //等待2s
        if(1==f_GsmAtRespState)
            break;
    }
    SetAtSend(ATSEND_IDLE);
    if(0==f_GsmAtRespState)
        return;
	sleep(5);    //等待5S
//关闭WiFi
    f_GsmAtRespState = 0;
    for(i=0; i<2; i++)
    {
        SetAtSend(ATSEND_CONNECT_WiFIOFF);
        WriteGsmUartData(gGsmAT_fd, (uint8 *)"AT+CWMAP=0\r", 11); 
		for(j=0;j<10;j++)
		{
            sleep(1);                //等待1s
            if(1==f_GsmAtRespState)
				break;
		}        
        if(1==f_GsmAtRespState)
            break;
    }
    SetAtSend(ATSEND_IDLE);
    if(0==f_GsmAtRespState)
        return;	
	else
	{
        WriteGsmUartData(gGsmAT_fd, (uint8 *)"AT+CGPSINFOCFG=2,3\r", 20);  
		sleep(1); 
        GSM_SetModemWorkingState(1);    //下一步拨号
	}    
}


void GSM_Modem_QmiWwan_ON(void)
{
/*    
    f_stuGsmState.ucQMIWwanEnable = 1;  //通知模块拨号
    if (system("/usr/quectel-CM &") < 0)
    //if (system("/mnt/liuxf/quectel-CM &") < 0)
    {
        printf("quectel-CM_ERR ON \n");
	}
    sleep(5);    //延时等待killall quectel-CM
*/
    //判断拨号结果
    //成功 则转入建立socket步骤
    GSM_SetModemWorkingState(2); 
}
void GSM_Modem_QmiWwan_OFF(void)
{
    f_stuGsmState.ucQMIWwanEnable = 0;  //通知模块拨号
}

void GSM_Modem_Socket_ON(void)
{
    //创建socket成功，则转入发送网络数据步骤

    if(0==Gsm_CreatSocket())
        GSM_SetModemWorkingState(3); 
	else
		GSM_SetModemWorkingState(0);
}
//f_GsmAtRespState
void GSM_QueryGsmState(void)
{
	static uint8 state = 0;
	static uint8 sim_err_times = 0;
	static uint8 cgatt_err_times = 0;
	
	switch (state)
	{
		case 0:
			f_GsmAtRespState = 0;
	   		SetAtSend(ATSEND_CONNECT_CPIN);
			WriteGsmUartData(gGsmAT_fd, (uint8 *)"at+cpin?\r", 9);	
			OSTimeDly(OS_TICKS_PER_SEC/20);//等待50ms
			if(1==f_GsmAtRespState)
			{
				sim_err_times = 0;
				f_stuGsmState.ucCPIN = 1;
			}
			else
			{
				if(sim_err_times++ > 3)
				{
					sim_err_times = 0;
					f_stuGsmState.ucCPIN = 0;
					PC_SendDebugData((uint8 *)("GSM RST4"), 8, DEBUG_AT);
					//ResetGsm();  //考虑复位
					GSM_SetModemWorkingState(8);    //重启模块
				}
			}
			SetAtSend(ATSEND_IDLE);
			state = 1;
			break;
		case 1:
			f_GsmAtRespState = 0;
			SetAtSend(ATSEND_CONNECT_CSQ);
	   		WriteGsmUartData(gGsmAT_fd, (uint8 *)"at+csq\r", 7);	
			OSTimeDly(OS_TICKS_PER_SEC/20);//等待50ms
			SetAtSend(ATSEND_IDLE);
			state = 2;
			break;
		case 2:
			f_GsmAtRespState = 0;
			SetAtSend(ATSEND_CONNECT_CGATT);
	   		WriteGsmUartData(gGsmAT_fd, (uint8 *)"at+cgatt?\r", 10);	
			OSTimeDly(OS_TICKS_PER_SEC/20);//等待50ms
			if(1==f_GsmAtRespState)
			{
				cgatt_err_times = 0;
				f_stuGsmState.ucCGATT = 1;
			}
			else
			{
				if(cgatt_err_times++ > 3)
				{
					cgatt_err_times = 0;
					f_stuGsmState.ucCGATT = 0;
					GSM_SetModemWorkingState(8);   //重启模块
				}
			}
			SetAtSend(ATSEND_IDLE);
			state = 0;
			break;
		/*case 3:
			if((0==f_stuSmsRcv.flag) && (0==f_stuSmsSend.flag))
			{
				GsmAtCmdState.ucCMGL = 0;
				SetAtSend(ATSEND_READSMS_CMGL);
		   		WriteGsmUartData((uint8 *)"AT+CMGL=4\r", 10);
				OSTimeDly(OS_TICKS_PER_SEC/50);//等待20ms
				SetAtSend(ATSEND_IDLE);
			}
			state = 0;
			break;*/
		default:
			break;
	}
}
void GSM_Modem_NetWorkLayer(void)
{
    static uint16 usCount = 0;
	
    f_stuGsmState.ucLinkState[0] = GPRS_LINK_STATE_READY;
	//定时查询 模块状态
	if(usCount>500)    //定时5秒
	{
        usCount = 0;
	    GSM_QueryGsmState();
	}
	else
		usCount++;
	usleep(One_MilliSecond*10);  //10ms
}

void GSM_Modem_Socket_OFF(void)
{
    f_stuGsmState.ucLinkState[0] = GPRS_LINK_STATE_CLOSED;
//	g_stuSystem.ucResetModem = 1;
	close(gGsm_socketfd);
    gGsm_socketfd = -1;
	GSM_SetModemWorkingState(2);
}
//重启模块
void GSM_Modem_Reset(void)
{
    if(gGsm_socketfd >= 0)
    {
        shutdown(gGsm_socketfd,SHUT_RDWR);
    	sleep(1);	
		close(gGsm_socketfd);
		gGsm_socketfd = -1;
    }
	if(f_stuGsmState.ucQMIWwanState)
		f_stuGsmState.ucQMIWwanEnable =  0;

	sleep(2);    //延时等待拨号关闭
	GSM_SetModemWorkingState(0);     //重新初始化模块
}

void* pthread_gsm_init(void* data)
{ 

    uint8 ucModemState;	
    data = data;
    printf("Open gsm_init \n");
    GSM_DataSendMutexInit();
    while(1)
   	{	

		ucModemState = GSM_GetModemWorkingState();
        switch(ucModemState)
        {
            case 0:      //模块启动
                GSM_Modem_Start();
                break;
			case 1:      //拨号
			    GSM_Modem_QmiWwan_ON();
			    break;
			case 2:      //创建socket
				GSM_Modem_Socket_ON();
			    break;
			case 3:      //创建socket成功
				GSM_Modem_NetWorkLayer();  //
				break;
			case 4:      //关闭拨号
			    GSM_Modem_QmiWwan_OFF();
				break;	
			case 5:      //关闭socket
				GSM_Modem_Socket_OFF();
				break;
			case 7:      //休眠模块
			    break;
			case 8:      //重启模块
			    GSM_Modem_Reset();
				break;
            default:
				usleep(One_MilliSecond);
			    break;
		}	    

    }	
    return NULL;	
}


/***************************************************
//返回1-成功,0-失败
AT+CGPADDR

+CGPADDR: 1,10.65.13.231   //成功
+CGPADDR: 2,0.0.0.0,0.0.0.0.0.0.0.0.0.0.0.0.0.0.0.0
+CGPADDR: 3,0.0.0.0,0.0.0.0.0.0.0.0.0.0.0.0.0.0.0.0
+CGPADDR: 4,0.0.0.0,0.0.0.0.0.0.0.0.0.0.0.0.0.0.0.0
+CGPADDR: 5,0.0.0.0,0.0.0.0.0.0.0.0.0.0.0.0.0.0.0.0
+CGPADDR: 6,0.0.0.0,0.0.0.0.0.0.0.0.0.0.0.0.0.0.0.0
------------------------------------------------------
+CGPADDR: 1,0.0.0.0
+CGPADDR: 2,0.0.0.0,0.0.0.0.0.0.0.0.0.0.0.0.0.0.0.0
+CGPADDR: 3,0.0.0.0,0.0.0.0.0.0.0.0.0.0.0.0.0.0.0.0
+CGPADDR: 6,10.147.149.221
------------------------------------------------------
+CGPADDR: 1,0.0.0.0
+CGPADDR: 2,0.0.0.0,0.0.0.0.0.0.0.0.0.0.0.0.0.0.0.0
+CGPADDR: 3,0.0.0.0,0.0.0.0.0.0.0.0.0.0.0.0.0.0.0.0
+CGPADDR: 4,0.0.0.0,0.0.0.0.0.0.0.0.0.0.0.0.0.0.0.0
+CGPADDR: 5,0.0.0.0,0.0.0.0.0.0.0.0.0.0.0.0.0.0.0.0
+CGPADDR: 6,0.0.0.0,0.0.0.0.0.0.0.0.0.0.0.0.0.0.0.0
------------------------------------------------------
***************************************************/
#if 0
static char* strnstr( const char * src, uint16_t src_len, const char * substr, uint16_t substr_len )
{
	const char * p;
	const char * pend;

    if (src_len == 0 || substr_len == 0)
    {
        return NULL;
    }

	if (src == NULL || substr == NULL)
	{
		return NULL;
	}

	if (0==*src || 0==*substr)
	{
		return NULL;
	}

    if (src_len < substr_len)
    {
      return NULL;
    }

	p = src;
	pend = p + src_len - substr_len;
	while ( p < pend )
	{
		if ( *p == *substr )
		{
			if ( memcmp( p, substr, substr_len ) == 0)
			{
				return (char*)p;
			}
		}
		p++;
	}

	return NULL;
}

static char* str1str(const char *src, uint16_t src_len, const char *subch)
{
	const char * p;
	const char * pend;
	
  if (src_len == 0 || src == NULL || 0==*subch)
  {
    return NULL;
  }	
	
	p = src;
	pend = p + src_len;
	while ( p < pend )
	{
    if(*p == *subch)
		{
			return (char*)p;
		}
		p++;
	}

	return NULL;
}

#endif
//  SearchString(uint8_t * str, uint16_t strlen, char * substr, uint8_t substrlen)
//SearchString((uint8 *)pucSrc, usSrcLen, ",", 1) ;
uint8 GSM_GetNetIPAddressState(uint8 *ptr, uint16 usLen)
{
  uint8 buf[50];

  uint16_t str_len;
  uint8 *str1;
  uint8 *str2;
  uint8_t temp_val1;
  uint8_t temp_val2;
  uint8_t temp_val3;
  uint8_t temp_val4;
  uint8_t i;

  str_len = usLen;
  str1 = (char*)ptr;
  for (i=0; i<6; i++)
  {
    str2 = SearchString(str1,str_len,"+CGPADDR:",9);
    if (str2!=NULL)
    {
      temp_val1 = 0x00;
      temp_val2 = 0x00;
      temp_val3 = 0x00;
      temp_val4 = 0x00;

      str_len = usLen - (str2 - str1);
      str1 = str2;
      str2 = SearchString((uint8 *)str1,str_len,",",1);
      if(str2==NULL) {return FALSE;}
      str2 += 1;
      temp_val1 = atoi(str2);

      str_len = usLen - (str2 - str1);
      str1 = str2;
      str2 = SearchString((uint8 *)str1,str_len,".",1);
      if(str2==NULL) {return FALSE;}
      str2 += 1;
      temp_val2 = atoi(str2);

      str_len = usLen - (str2 - str1);
      str1 = str2;
      str2 = SearchString((uint8 *)str1,str_len,".",1);
      if(str2==NULL) {return FALSE;}
      str2 += 1;
      temp_val3 = atoi(str2);

      str_len = usLen - (str2 - str1);
      str1 = str2;
      str2 = SearchString((uint8 *)str1,str_len,".",1);
      if(str2==NULL) {return FALSE;}
      str2 += 1;
      temp_val4 = atoi(str2);

      str1 = str2;

      if (temp_val1 || temp_val2 || temp_val3 || temp_val4)
      {
        snprintf((char *)buf, sizeof(buf), "IP= %d.%d.%d.%d\r\n", temp_val1,temp_val2,temp_val3,temp_val4);
        PC_SendDebugData(buf, sizeof(buf), DEBUG_AT);
        return TRUE;
      }
    }
    else
    {
      return FALSE;
    }
  }
  
  return FALSE;
}





#if 0
uint8 GSM_GetNetIPAddressState(uint8 *ptr, uint16 usLen)
{
  uint8 buf[20];

  char *str;
  uint8_t temp_val1;
  uint8_t temp_val2;
  uint8_t temp_val3;
  uint8_t temp_val4;
  uint8_t i;

  str = (char*)ptr;
  for (i=0; i<6; i++)
  {
    str = strstr(str,"+CGPADDR:");
    if (str!=NULL)
    {
      temp_val1 = 0x00;
      temp_val2 = 0x00;
      temp_val3 = 0x00;
      temp_val4 = 0x00;

      str += 12;
      temp_val1 = atoi(str);

      str = strstr((char *)str,".");
      str += 1;

      temp_val2 = atoi(str);

      str = strstr((char *)str,".");
      str += 1;

      temp_val3 = atoi(str);

      str = strstr((char *)str,".");
      str += 1;

      temp_val4 = atoi(str);

      if (temp_val1 || temp_val2 || temp_val3 || temp_val4)
      {
        snprintf((char *)buf, sizeof(buf), "IP= %d.%d.%d.%d\r\n", temp_val1,temp_val2,temp_val3,temp_val4);
        PC_SendDebugData(buf, sizeof(buf), DEBUG_AT);
        return TRUE;
      }
    }
  }
  
  return FALSE;
}


uint8 GSM_GetNetIPAddressState(uint8 *ptr, uint16 usLen)
{
    uint8 *pTemp=NULL;
	uint8 *pTemp1=NULL;
	uint8 uclen = 0;
	uint8 ucIpValue[4] = {0};
	uint8 buf[20];
	
	pTemp = SearchString((uint8*)ptr, usLen, "CGPADDR: 1", 10);
    if(pTemp)
    {
        pTemp += 10;
		usLen -= 11;
		pTemp1 = GetStringMiddle(pTemp,usLen,',',1,'.',1,&uclen);
		if(pTemp1)
		{
           // pTemp1 += 1;    //IP第一个字段地址
    		ucIpValue[0] = atoi(pTemp1);    //10
		}
		pTemp = GetStringMiddle(pTemp1,usLen,'.',1,'.',2,&uclen);
		if(pTemp)
		{
           // pTemp += 1;    //IP第一个字段地址
    		ucIpValue[1] = atoi(pTemp);    //65
		}
		pTemp1 = GetStringMiddle(pTemp,usLen,'.',1,'.',2,&uclen);
		if(pTemp1)
		{
           // pTemp1 += 1;    //IP第一个字段地址
    		ucIpValue[2] = atoi(pTemp1); //13
		}	
		pTemp = GetStringMiddle(pTemp1,usLen,'.',1,'\r\n',1,&uclen);
		if(pTemp)
		{
           // pTemp += 1;    //IP第一个字段地址
    		ucIpValue[3] = atoi(pTemp);  //231
		}	
		snprintf(buf, sizeof(buf), "IP= %d.%d.%d.%d\r\n", ucIpValue[0],ucIpValue[1],ucIpValue[2],ucIpValue[3]);
        PC_SendDebugData(buf, sizeof(buf), DEBUG_AT);

		//printf("IP= %d.%d.%d.%d. ",ucIpValue[0],ucIpValue[1],ucIpValue[2],ucIpValue[3]);        
		if(ucIpValue[0]||ucIpValue[1]||ucIpValue[2]||ucIpValue[3])
		{
			return TRUE;
		}
		else
		    return FALSE;
	}
	return FALSE;
}
#endif

/******************************************************************************
** 函数名称: pthread_gsm_read_AT
** 功能描述: 读取GSM模块返回的AT指令响应数据线程,阻塞方式
** 
** 输    入: data
** 输    出: 无
** 返    回: 无
** 
** 作    者: lxf
** 日    期: 2019-03-25
**-----------------------------------------------------------------------------
*******************************************************************************/
void* pthread_gsm_read_AT(void *data)
{

    int usLen = 0;
    uint8 *pucData;
    uint8 temp = 0;

    printf("pthread_gsm_read_AT\n");

    while(1)
    {

        if(gGsmAT_fd<0)    //gGsmAT_fd 打开失败 继续等待
        {
            usleep(20*One_MilliSecond);
        //  continue;
        }
        else
        {
            memset(GSM_AT_Recv_Buff,0,GSM_AT_RECV_BUFF_MAX_SIZE);
            usLen=read(gGsmAT_fd,GSM_AT_Recv_Buff,GSM_AT_RECV_BUFF_MAX_SIZE-1);
           
            if(usLen>0&&usLen<GSM_AT_RECV_BUFF_MAX_SIZE)
            {
         
            //   printf("read: %d , %s\n", usLen,GSM_AT_Recv_Buff);
               PC_SendDebugData(GSM_AT_Recv_Buff, usLen, DEBUG_AT);

                pucData = &GSM_AT_Recv_Buff[0];
                
                switch(GetAtSend())
                {
                
                    case ATSEND_CONNECT_CFUN0:
                        if(SearchString((uint8*)pucData, usLen, "OK", 2))
                        {
                            //GsmAtCmdState.ucAt = 1;
                            f_GsmAtRespState =1;
                        }
                        break;
                    case ATSEND_CONNECT_CFUN1:
                        if(SearchString((uint8*)pucData, usLen, "OK", 2))
                        {
                            //GsmAtCmdState.ucAt = 1;
                            f_GsmAtRespState =1;
                        }
                        break;          
                    case ATSEND_CONNECT_CRESET:
                        if(SearchString((uint8*)pucData, usLen, "OK", 2))
                        {
                            //GsmAtCmdState.ucAt = 1;
                            f_GsmAtRespState =1;
                        }
                        break;      
                        
                    case ATSEND_CONNECT_AT:
                        if(SearchString((uint8*)pucData, usLen, "OK", 2))
                        {
                            //GsmAtCmdState.ucAt = 1;
                            f_GsmAtRespState =1;
                        }
                        break;
                    case ATSEND_CONNECT_ATE0:
                        if(SearchString((uint8*)pucData, usLen, "OK", 2))
                        {
                            //GsmAtCmdState.ucATE0= 1;
                            f_GsmAtRespState =1;
                        }
                        break;
                        /*
                    case ATSEND_CONNECT_CMEE:
                        if(SearchString((uint8*)pucData, usLen, "OK", 2))
                        {
                            //GsmAtCmdState.ucCMEE = 1;
                            f_GsmAtRespState =1;
                        }
                        break;
                        */
                    case ATSEND_CONNECT_CPIN:
                        temp = UnPackCPIN(pucData, usLen);
                        if(TRUE == temp)
                        {
                            //GsmAtCmdState.ucCPIN = 1;
                            f_GsmAtRespState =1;
                        }
                        break;
                        /*
                    case ATSEND_CONNECT_CNMI:
                        if(SearchString((uint8*)pucData, usLen, "OK", 2))
                        {
                            //GsmAtCmdState.ucCNMI= 1;
                            f_GsmAtRespState =1;
                        }
                        break;
                        */
                    case ATSEND_CONNECT_CIMI:

                        if(SearchString((uint8*)pucData, usLen, "OK", 2))
                        {
                            //GsmAtCmdState.ucATE0= 1;
                            f_GsmAtRespState =1;
                        }
                        break;						
						#if 0
                        temp = UnPackCIMI(pucData, usLen);
                        if(TRUE == temp)
                        {
                            //GsmAtCmdState.ucCIMI = 1;
                            f_GsmAtRespState =1;
                        }
                        break;
						#endif
                    case ATSEND_CONNECT_CCID:
                        temp = UnPackCCID(pucData, usLen);
                        if(TRUE == temp)
                        {
                            //GsmAtCmdState.ucCCID = 1;
                            f_GsmAtRespState =1;
                        }
                        break;  
                        
                    case ATSEND_CONNECT_CSQ:
                        temp = UnPackCSQ(pucData, usLen);
                        if((TRUE == temp) && (f_stuGsmState.ucCSQ>6))
                        {
                            //GsmAtCmdState.ucCSQ = 1;
                            f_GsmAtRespState =1;
                        }
                        break;
                    case ATSEND_CONNECT_CREG_SET:
                        if(SearchString((uint8*)pucData, usLen, "OK", 2))
                        {
                            //GsmAtCmdState.ucCREG_SET= 1;
                            f_GsmAtRespState =1;
                        }
                        break;
                    case ATSEND_CONNECT_CREG:
                        temp = UnPackCREG(pucData, usLen);
                        if(TRUE == temp)
                        {
                            f_GsmAtRespState = 1;
                        }
                        break;
                    
                    case ATSEND_CONNECT_CGATT:
                        temp = UnPackCGATT(pucData, usLen);
                        if(TRUE == temp)
                        {
                            //GsmAtCmdState.ucCGATT = 1;
                            f_GsmAtRespState =1;
                        }
                        break;
					case ATSEND_CONNECT_CPSI:
                        temp = UnPackCPSI(pucData, usLen);
                        if(TRUE == temp)
                        {                            
                            f_GsmAtRespState =1;
                        }
						break;
                    case ATSEND_CONNECT_APN:
                        if(SearchString((uint8*)pucData, usLen, "OK", 2))
                        {
                            f_GsmAtRespState =1;
                        }
                        break;      
                    case ATSEND_CONNECT_NETACT_SET:
                        if(SearchString((uint8*)pucData, usLen, "OK", 2))
                        {
                            f_GsmAtRespState =1;
                        }
                        break;  
                    case ATSEND_CONNECT_NETACT: 
                        if(SearchString((uint8*)pucData, usLen, "NETACT: 1", 9))
                        {
                            f_GsmAtRespState =1;
                        }
                        break; 
					case ATSEND_CONNECT_CGPADDR:
						if(GSM_GetNetIPAddressState((uint8*)pucData, usLen))
						{
                            f_GsmAtRespState =1;
						}
						break;
                    case ATSEND_CONNECT_WiFIOFF: 
                        if(SearchString((uint8*)pucData, usLen, "OK", 2))
                        {
                            f_GsmAtRespState =1;
                        }
                        break;						
                    default:
                        break;              
                }

                if(pucData[0]=='$')
                {
                    GPS_Function(pucData, usLen);
                }                
            }
        }
        usleep(20*One_MilliSecond);
    }
    return NULL;
}

#if 0
void* pthread_quectel_CM(char *argv[])
{

    printf("pthread_quectel_CM\n");

 //   quectel_CM(argc, argv);

}


void* pthread_gsm_RecvData(void *data)
{

   printf("pthread_gsm_RecvData\n");
   
   while(1)
   {
            
        //    printf("Gsm_RecvData\n");
            usleep(5*1000);
    	//读取数据
        	memset(GSM_Recv_Buff, 0, GSM_RECV_BUFF_MAX_SIZE);
		    memset(GSM_BuffTemp, 0, GSM_RECV_BUFF_MAX_SIZE);
        	if((GSM_Recv_Buff_Len = recv(gGsm_socketfd, GSM_Recv_Buff, GSM_RECV_BUFF_MAX_SIZE-1, 0)) < 0)
            {
        		if (errno != EAGAIN)   //等于EAGAIN时 不代表异常 徐继续查询  
        		{
                   printf("Recv Gsmdata Err\n");
        		}
            }
        	else
        	{
        	    if(GSM_Recv_Buff_Len)
        	    {
            		//printf("RecvLen = %d \n",GSM_Recv_Buff_Len);
            		DataToHexbuff(GSM_BuffTemp, GSM_Recv_Buff, GSM_Recv_Buff_Len);
    				PrintTime();
            		printf("Recv,Len= %d; %s\n",GSM_Recv_Buff_Len, GSM_BuffTemp);
                    DealSerCmd(GSM_Recv_Buff, GSM_Recv_Buff_Len,SRCDEVICE_ID_GSM,GSM_MSG_GPRS_RECV);    
				}
        	}
   	}
}
#endif

#if 1
/******************************************************************************
** 函数名称: pthread_gsm_RecvData
** 功能描述: 读取GSM模块服务器下行数据线程,阻塞方式
** 
** 输    入: data
** 输    出: 无
** 返    回: 无
** 
** 作    者: lxf
** 日    期: 2019-03-25
**-----------------------------------------------------------------------------
*******************************************************************************/
void* pthread_gsm_RecvData(void *data)
{

   printf("pthread_gsm_RecvData\n");
   
   while(1)
   {

	    if((GPRS_LINK_STATE_CLOSED==GSM_GetLinkState(0))||(gGsm_socketfd<0))
	    {
            usleep(5000);
		}
		else
		{
    	//读取数据
          //  printf("Recv Gsmdata test\n");
        	memset(GSM_Recv_Buff, 0, GSM_RECV_BUFF_MAX_SIZE);
		    memset(GSM_BuffTemp, 0, GSM_RECV_BUFF_MAX_SIZE);
        	if((GSM_Recv_Buff_Len = recv(gGsm_socketfd, GSM_Recv_Buff, GSM_RECV_BUFF_MAX_SIZE-1, 0)) < 0)
            {
        		if (errno != EAGAIN)   //等于EAGAIN时 不代表异常 徐继续查询  
        		{
                   printf("Recv Gsmdata Err\n");
        		}
            }
        	else
        	{
        	    if(GSM_Recv_Buff_Len>0&&GSM_Recv_Buff_Len<GSM_RECV_BUFF_MAX_SIZE)
        	    {
            		//printf("RecvLen = %d \n",GSM_Recv_Buff_Len);
            		if(SYS_GetDubugStatus()&BIT(0))
            		    DataToHexbuff(GSM_BuffTemp, GSM_Recv_Buff, GSM_Recv_Buff_Len);
    			//	PrintTime();
            	//	printf("Recv,Len= %d; %s\n",GSM_Recv_Buff_Len, GSM_BuffTemp);
            	    PC_SendDebugData(GSM_BuffTemp, GSM_Recv_Buff_Len*2, DEBUG_AT);
                    PC_SendDebugData(GSM_Recv_Buff, GSM_Recv_Buff_Len, DEBUG_GPRS);
                    DealSerCmd(GSM_Recv_Buff, GSM_Recv_Buff_Len,SRCDEVICE_ID_GSM,GSM_MSG_GPRS_RECV);    
                    //向协处理器转发参数设置命令
                    if(GSM_Recv_Buff[0]==0x05&&((GSM_Recv_Buff[15]=='P'&&GSM_Recv_Buff[16]=='W')||(GSM_Recv_Buff[15]=='R'&&GSM_Recv_Buff[16]=='C')))
                        A5_UART0_Write(GSM_Recv_Buff, GSM_Recv_Buff_Len);
				}
        	}
		}
        usleep(1000);
   	}
}

#endif

//-----内部函数定义------------------------------------------------------------ 


