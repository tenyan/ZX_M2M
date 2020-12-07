/*
 * Copyright(c)2019, 硬件研发部
 * All right reserved
 *
 * 文件名称: A5Uart0_MCU.c
 * 版本号  : V1.0
 * 文件标识:
 * 文件描述: 本文件为A5Uart0_MCU功能模块协议层处理的文件
 *           用于核心板uart0与协处理器MCU的uart6之间的通讯
 *-----------------------------------------------------------------------------
 * 修改记录
 *-----------------------------------------------------------------------------
 *
 * 2019-05-13, by  lxf, 创建本文件
 *
 */
#include "config.h"
//-----头文件调用------------------------------------------------------------
//#include "A5UART0_MCU.h"


//-----外部变量定义------------------------------------------------------------
extern STUExtMCUUpgrade stu_ExtMCUUpgrade;
extern stuFirmwareUpdate FirmwareUpdate;

//-----内部变量定义------------------------------------------------------------


#define A5_UART0_Rx_Buff_BSIZE    1200
uint8 A5_UART0_Rx_Buff[A5_UART0_Rx_Buff_BSIZE];
//uint16 DEBUG_UART_Rx_Cnt;

int fd_A5_UART0 = -1;
//-----内部函数声明------------------



void A5_Uart0_Init(void)  
{ 
    
	struct termios options;

	/* open uart */
	fd_A5_UART0 = open(DEV_TTYS4_UART0, O_RDWR|O_NOCTTY);
	if (fd_A5_UART0 < 0)
	{
		printf("ERROR open %s ret=%d\n\r", DEV_TTYS4_UART0, fd_A5_UART0);
        close(fd_A5_UART0);
		fd_A5_UART0 = -1;
		return ;
	}
	else
	{
    	
	}
    printf("fd_A5_UART0: %d \n", fd_A5_UART0);

	/* configure uart */
	tcgetattr(fd_A5_UART0, &options);
	options.c_cflag &= ~PARENB;
	options.c_cflag &= ~CSTOPB;
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;
	options.c_cc[VTIME] = 1; // read timeout 单位*100ms
	options.c_cc[VMIN]  = 0;
	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	options.c_oflag &= ~OPOST;
	options.c_iflag &= ~(ICRNL | IXON);
	cfsetispeed(&options, B115200);//波特率设置
	cfsetospeed(&options, B115200);//波特率设置
	options.c_cflag |= (CLOCAL | CREAD);
	tcflush(fd_A5_UART0, TCIFLUSH);
	tcsetattr(fd_A5_UART0, TCSANOW, &options);
}  

BOOL ReadA5Uart0Data(uint8 **data, uint16* len)
{
	int ret;
    memset(A5_UART0_Rx_Buff,0,A5_UART0_Rx_Buff_BSIZE);
	
	if(fd_A5_UART0<0)
		return FALSE;
		
	if((ret = read(fd_A5_UART0, A5_UART0_Rx_Buff, A5_UART0_Rx_Buff_BSIZE-1)) > 0)
	{
		*data = A5_UART0_Rx_Buff;
		*len = (uint16)ret;
		return TRUE;
	}
	else
		return FALSE;
}

uint16 A5_UART0_Write(uint8 *data, uint16 Len)
{
    int ret;

	if((0==Len) || (NULL==data)||(fd_A5_UART0<0))
		return 0;
	/* write uart */
	ret = write(fd_A5_UART0, data, Len);
	if (ret != Len)
	{
		    printf("ERROR Debug write ret=%d\n", ret);
			return 0;
	}
	else
		return Len;
}


/*
//串口发送打包函数
uslen :需要发送数据体长度 (不包含Command、SN、Checksum 3个字节长度)
*/
void A5Uart0_DataSend(uint8 ucCommand, uint16 uslen, uint8* pdata,uint8 ucSN)
{
    uint8 SendBuff[1200] = {0};

    	
	SendBuff[0] = 0x7E;
	SendBuff[1] = (uint8)((uslen+3)>>8);
	SendBuff[2] = (uint8)((uslen+3)&0xFF);
	SendBuff[3] = ucCommand;            //Command	
		
	SendBuff[4] = ucSN;
	memcpy(&SendBuff[5],pdata,uslen);

	SendBuff[5+uslen] = SumCalc(&SendBuff[3],uslen+2);
	
	SendBuff[6+uslen] = 0x0D;
	SendBuff[7+uslen] = 0x0A;
	
	A5_UART0_Write(SendBuff, 8+uslen);
	//PC_SendDebugData(SendBuff, 8+uslen, DEBUG_ANYDATA);
}

#if 0
//定时向协处理器MCU发送时钟信息,5分钟一次  ,每秒执行一次
void A5Uart0_TimingSendClock(void)
{
    STU_Date studate;
    uint8 SendBuff[10] = {0};
	static uint8 ucSN = 0;
	static uint16 usCount = 295;

	if(usCount>=300)
		usCount = 0;
	else
	{
		usCount++;
		return;
	}
	studate = GetRTCTime();      //读取RTC时钟
    memcpy(SendBuff,(uint8*)&studate,6);
    SendBuff[6] = 0;   //ms_H
	SendBuff[7] = 0;   //ms_L
    A5Uart0_DataSend(0x03, 8, SendBuff, uint8 ucSN);
    if(ucSN>254)
	    ucSN = 0;
}


void A5Uart0_Deal_MCUData(uint8* ptr, uint16 uslen)
{
    uint8 ucSumNum = 0;
	uint16 usDatalen = 0;
	uint8 ucCommand;
	uint8 ucSN,ucResult;


    if((ptr[0]==0x7E)&&(ptr[uslen-2]==0x0D)&&(ptr[uslen-1]==0x0A)&uslen>8)
    {
    
    	usDatalen = (uint16)(ptr[1]<<8) + ptr[2];
        ucCommand = ptr[3];
    	ucSN = ptr[4];
    	ucSumNum = SumCalc(&ptr[3], usDatalen-1);
    	if(ucSumNum!=ptr[uslen-3])
    		return;
    	
        if(ucCommand==0x83)      //MCU对核心板发送命令的应答
    	{
            ucResult = ptr[5];   // 0-成功；1-异常。 失败可以考虑重发
    	}	
    }
	else   //考虑透传数据 设置程序
	{

	}
}
#endif
void* pthread_A5Uart0_Function(void* data)
{
	uint16 usLen = 0;    //缓冲区接收到的数据长度 
	uint8 *p=NULL;


    A5_Uart0_Init();
   // stu_ExtMCUUpgrade.ucUpgradeStep = 1;    //20191212 测试
  //  FirmwareUpdate.usLastPacketLen = 536;
    while(1)
    {
        if(ReadA5Uart0Data(&p, &usLen))
        {     
          //  A5Uart0_Deal_MCUData(p,usLen);
           Ext_Mcu_RecvUartData(p,usLen);
           PC_SendDebugData(p,usLen, DEBUG_GPRS);
           
        }
		else
		{
          //  usleep(20*One_MilliSecond);
		}
    }
}



//-----文件结束---------------------------------------------
