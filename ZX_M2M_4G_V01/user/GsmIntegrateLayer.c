/* 
 * Copyright(c)2020, �����칤��Ϣ�����ɷ����޹�˾-����Ӳ����ҵ��
 * All right reserved
 *
 * �ļ�����: GsmIntegrateLayer.c
 * �汾��  : V1.0
 * �ļ���ʶ:
 * �ļ�����: ���ļ�ΪGSMģ���ۺϲ��ʵ���ļ�
 *
 *-----------------------------------------------------------------------------
 * �޸���ʷ
 *-----------------------------------------------------------------------------
 *
 * 2020-11-30 by lxf, �������ļ�
 *
 */
#include "config.h"
#include "GsmInterfaceLayer.h" 
//#include "GsmHardWareLayer.h"
#include "GsmProtocolLayer.h"
#include "GsmIntegrateLayer.h"

//-----�ⲿ��������------------------------------------------------------------
extern STUGsmState f_stuGsmState;
extern stuSerAddr g_stuSerAddr;
extern int gGsm_socketfd;
extern int32 gGsmAT_fd;
extern STUSystem g_stuSystem;
extern STUSYSParamSet g_stuSYSParamSet;
//-----�ڲ���������------------------------------------------------------------
STU_GsmOperate GsmOperate; 


uint8 GSM_AT_Recv_Buff[GSM_AT_RECV_BUFF_MAX_SIZE]={0};
int16 GSM_AT_Recv_Buff_Len = 0;
static uint8 f_ucAtSend;			//���͵�ATָ��
static uint8 f_GsmAtRespState = 0;  //����ATָ����Ƿ��յ�Ӧ��ı�־ 0-δ�յ�����δ��,1-�ɹ��յ�
uint8 GSM_Recv_Buff[GSM_RECV_BUFF_MAX_SIZE];
int16 GSM_Recv_Buff_Len = 0;
int16 GSM_Send_Buff_Len = 0;
uint8 GSM_BuffTemp[GSM_RECV_BUFF_MAX_SIZE*2];
uint8 ucOpenTimes = 0;   //��¼��������

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

//-----�ڲ���������------------------------------------------------------------

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
/*GSMģ��״̬:0=����,1=����,2=����socket,3=�����ɹ�,
  4=�رղ���,5=�ر�socket,6=������,7=����,8=����ģ��*/
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


//����AT+CGPADDR��ѯ��һ·ip��ַ,���������Ч��IP��ַ������socket����
void GSM_Modem_Start(void)
{
    static uint8 ucAtErrCnt = 0;    //�ղ���AT��Ӧ����
    static uint8 ucSimErrTimes = 0; //��¼��ѯ����SIM���Ĵ���
    uint16 i = 0;
	uint8 j = 0;
    uint8 atemp[90]={0};
    
    GSM_Variable_Init();   //������ʼ��

    if(gGsmAT_fd>=0)
    {
        close(gGsmAT_fd);
        printf("gGsmAT_fd Close: %d \n", gGsmAT_fd);
        gGsmAT_fd = -1; 
		sleep(1);
    }
    OpenGsmAT_DEV_TTY();   //��AT�豸�ڵ�
    if(gGsmAT_fd<0)
    {
        return ;
    }   
    
   //����
    ucOpenTimes++;
    if(ucOpenTimes&&(ucOpenTimes%5)==0)
    {
       //����AT
        f_GsmAtRespState = 0;
        for(i=0; i<3; i++)
        {
            SetAtSend(ATSEND_CONNECT_CFUN0);
            WriteGsmUartData(gGsmAT_fd, (uint8 *)"AT+CFUN=0\r", 10);    
            sleep(2);   //�ȴ�3s
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
            sleep(2);   //�ȴ�3s
            if(1==f_GsmAtRespState)
                break;
        }
        SetAtSend(ATSEND_IDLE);
        if(0==f_GsmAtRespState)
            return; 
        sleep(5);   //�ȴ�5s
    }
    else if(ucOpenTimes>20)   //����ģ��   ���ģ������ʧ��??
    {
        ucOpenTimes = 0;
        f_GsmAtRespState = 0;
        for(i=0; i<3; i++)
        {
            SetAtSend(ATSEND_CONNECT_CRESET);
            WriteGsmUartData(gGsmAT_fd, (uint8 *)"AT+CRESET\r", 10);    
            sleep(2);   //�ȴ�3s
            if(1==f_GsmAtRespState)
                break;
        }
        SetAtSend(ATSEND_IDLE);
        if(0==f_GsmAtRespState)
            return;     
        sleep(60);  //�ȴ�30s
    }
    sleep(3);   //�ȴ�3s
                    
   //����AT
    f_GsmAtRespState = 0;
    for(i=0; i<5; i++)
    {
        SetAtSend(ATSEND_CONNECT_AT);
        WriteGsmUartData(gGsmAT_fd, (uint8 *)"at\r", 3);    
        usleep(1000*One_MilliSecond);   //�ȴ�1000ms
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
        usleep(500*One_MilliSecond);   //�ȴ�20ms
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
        usleep(1000*One_MilliSecond);   //�ȴ�1s
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
        usleep(200*One_MilliSecond);   //�ȴ�200ms
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
        usleep(200*One_MilliSecond);   //�ȴ�20ms
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
		usleep(1000*One_MilliSecond);   //�ȴ�1s;
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
        usleep(1000*One_MilliSecond);   //�ȴ�20ms
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
        usleep(200*One_MilliSecond);   //�ȴ�20ms
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
        usleep(1000*One_MilliSecond);//�ȴ�1s
         
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
        usleep(1000*One_MilliSecond);//�ȴ�20ms
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
        usleep(1000*One_MilliSecond);//�ȴ�20ms
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
        sleep(2);                 //�ȴ�2s
        if(1==f_GsmAtRespState)
            break;
    }
    SetAtSend(ATSEND_IDLE);
    if(0==f_GsmAtRespState)
        return;
	sleep(5);    //�ȴ�5S
//�ر�WiFi
    f_GsmAtRespState = 0;
    for(i=0; i<2; i++)
    {
        SetAtSend(ATSEND_CONNECT_WiFIOFF);
        WriteGsmUartData(gGsmAT_fd, (uint8 *)"AT+CWMAP=0\r", 11); 
		for(j=0;j<10;j++)
		{
            sleep(1);                //�ȴ�1s
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
        GSM_SetModemWorkingState(1);    //��һ������
	}    
}


void GSM_Modem_QmiWwan_ON(void)
{
/*    
    f_stuGsmState.ucQMIWwanEnable = 1;  //֪ͨģ�鲦��
    if (system("/usr/quectel-CM &") < 0)
    //if (system("/mnt/liuxf/quectel-CM &") < 0)
    {
        printf("quectel-CM_ERR ON \n");
	}
    sleep(5);    //��ʱ�ȴ�killall quectel-CM
*/
    //�жϲ��Ž��
    //�ɹ� ��ת�뽨��socket����
    GSM_SetModemWorkingState(2); 
}
void GSM_Modem_QmiWwan_OFF(void)
{
    f_stuGsmState.ucQMIWwanEnable = 0;  //֪ͨģ�鲦��
}

void GSM_Modem_Socket_ON(void)
{
    //����socket�ɹ�����ת�뷢���������ݲ���

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
			OSTimeDly(OS_TICKS_PER_SEC/20);//�ȴ�50ms
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
					//ResetGsm();  //���Ǹ�λ
					GSM_SetModemWorkingState(8);    //����ģ��
				}
			}
			SetAtSend(ATSEND_IDLE);
			state = 1;
			break;
		case 1:
			f_GsmAtRespState = 0;
			SetAtSend(ATSEND_CONNECT_CSQ);
	   		WriteGsmUartData(gGsmAT_fd, (uint8 *)"at+csq\r", 7);	
			OSTimeDly(OS_TICKS_PER_SEC/20);//�ȴ�50ms
			SetAtSend(ATSEND_IDLE);
			state = 2;
			break;
		case 2:
			f_GsmAtRespState = 0;
			SetAtSend(ATSEND_CONNECT_CGATT);
	   		WriteGsmUartData(gGsmAT_fd, (uint8 *)"at+cgatt?\r", 10);	
			OSTimeDly(OS_TICKS_PER_SEC/20);//�ȴ�50ms
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
					GSM_SetModemWorkingState(8);   //����ģ��
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
				OSTimeDly(OS_TICKS_PER_SEC/50);//�ȴ�20ms
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
	//��ʱ��ѯ ģ��״̬
	if(usCount>500)    //��ʱ5��
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
//����ģ��
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

	sleep(2);    //��ʱ�ȴ����Źر�
	GSM_SetModemWorkingState(0);     //���³�ʼ��ģ��
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
            case 0:      //ģ������
                GSM_Modem_Start();
                break;
			case 1:      //����
			    GSM_Modem_QmiWwan_ON();
			    break;
			case 2:      //����socket
				GSM_Modem_Socket_ON();
			    break;
			case 3:      //����socket�ɹ�
				GSM_Modem_NetWorkLayer();  //
				break;
			case 4:      //�رղ���
			    GSM_Modem_QmiWwan_OFF();
				break;	
			case 5:      //�ر�socket
				GSM_Modem_Socket_OFF();
				break;
			case 7:      //����ģ��
			    break;
			case 8:      //����ģ��
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
//����1-�ɹ�,0-ʧ��
AT+CGPADDR

+CGPADDR: 1,10.65.13.231   //�ɹ�
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
           // pTemp1 += 1;    //IP��һ���ֶε�ַ
    		ucIpValue[0] = atoi(pTemp1);    //10
		}
		pTemp = GetStringMiddle(pTemp1,usLen,'.',1,'.',2,&uclen);
		if(pTemp)
		{
           // pTemp += 1;    //IP��һ���ֶε�ַ
    		ucIpValue[1] = atoi(pTemp);    //65
		}
		pTemp1 = GetStringMiddle(pTemp,usLen,'.',1,'.',2,&uclen);
		if(pTemp1)
		{
           // pTemp1 += 1;    //IP��һ���ֶε�ַ
    		ucIpValue[2] = atoi(pTemp1); //13
		}	
		pTemp = GetStringMiddle(pTemp1,usLen,'.',1,'\r\n',1,&uclen);
		if(pTemp)
		{
           // pTemp += 1;    //IP��һ���ֶε�ַ
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
** ��������: pthread_gsm_read_AT
** ��������: ��ȡGSMģ�鷵�ص�ATָ����Ӧ�����߳�,������ʽ
** 
** ��    ��: data
** ��    ��: ��
** ��    ��: ��
** 
** ��    ��: lxf
** ��    ��: 2019-03-25
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

        if(gGsmAT_fd<0)    //gGsmAT_fd ��ʧ�� �����ȴ�
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
    	//��ȡ����
        	memset(GSM_Recv_Buff, 0, GSM_RECV_BUFF_MAX_SIZE);
		    memset(GSM_BuffTemp, 0, GSM_RECV_BUFF_MAX_SIZE);
        	if((GSM_Recv_Buff_Len = recv(gGsm_socketfd, GSM_Recv_Buff, GSM_RECV_BUFF_MAX_SIZE-1, 0)) < 0)
            {
        		if (errno != EAGAIN)   //����EAGAINʱ �������쳣 �������ѯ  
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
** ��������: pthread_gsm_RecvData
** ��������: ��ȡGSMģ����������������߳�,������ʽ
** 
** ��    ��: data
** ��    ��: ��
** ��    ��: ��
** 
** ��    ��: lxf
** ��    ��: 2019-03-25
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
    	//��ȡ����
          //  printf("Recv Gsmdata test\n");
        	memset(GSM_Recv_Buff, 0, GSM_RECV_BUFF_MAX_SIZE);
		    memset(GSM_BuffTemp, 0, GSM_RECV_BUFF_MAX_SIZE);
        	if((GSM_Recv_Buff_Len = recv(gGsm_socketfd, GSM_Recv_Buff, GSM_RECV_BUFF_MAX_SIZE-1, 0)) < 0)
            {
        		if (errno != EAGAIN)   //����EAGAINʱ �������쳣 �������ѯ  
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
                    //��Э������ת��������������
                    if(GSM_Recv_Buff[0]==0x05&&((GSM_Recv_Buff[15]=='P'&&GSM_Recv_Buff[16]=='W')||(GSM_Recv_Buff[15]=='R'&&GSM_Recv_Buff[16]=='C')))
                        A5_UART0_Write(GSM_Recv_Buff, GSM_Recv_Buff_Len);
				}
        	}
		}
        usleep(1000);
   	}
}

#endif

//-----�ڲ���������------------------------------------------------------------ 


