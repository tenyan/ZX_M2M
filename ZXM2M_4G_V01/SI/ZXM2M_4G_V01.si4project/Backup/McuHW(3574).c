/*
 * Copyright(c)2020, 江苏徐工信息技术股份有限公司-智能硬件事业部
 * All right reserved
 *
 * 文件名称: McuHW.c
 * 版本号  : V1.0
 * 文件标识:
 * 文件描述: 本文件为
 *
 *-----------------------------------------------------------------------------
 * 修改记录
 *-----------------------------------------------------------------------------
 *
 * 2019-3-22, 创建本文件
 *
 */

//-----头文件调用------------------------------------------------------------
#include "config.h"
#include "McuHW.h"

//-----外部变量定义------------------------------------------------------------

//-----内部变量定义------------------------------------------------------------
stu_McuCan_Tx stuMcuCanTx;
stu_McuCan_Rx stuMcuCanRx;
pthread_mutex_t gMcuCanSendMutex;	/* 发送GPRS数据锁 */

MessageDetail MessageCAN1_0;      // 引用CAN0通道帧变量
MessageDetail MessageCAN1_1;

extern void DealCan1Message(MessageDetail msg);
extern STU_CommState m_stuCommState;

void McuIOPinInit()
{
#if 0

  CAN1_LED_ON();
  CAN2_LED_ON();
  BLU_LED_ON();
#endif
}
/*
------------------------------------------------------------------------------
can时钟是RCC_APB1PeriphClock，你要注意CAN时钟频率
CAN波特率 = RCC_APB1PeriphClock/CAN_SJW+CAN_BS1+CAN_BS2/CAN_Prescaler;
如果CAN时钟为8M， CAN_SJW = 1，CAN_BS1 = 8，CAN_BS2 = 7，CAN_Prescaler = 2
那么波特率就是=8M/(1+8+7)/2=250K
--------------------------------------------------------------------------------_
*/
#define USE_CAN1				1
#define USE_CAN2				0


#define DEV_TTY_MCU_CAN "/dev/ttyHS0"

#define MCU_CAN_RECV_BUFF_LEN         1500    //实际从MCU发来的包最长为1029Byte
uint8 MCU_Can_Recv_Buff[MCU_CAN_RECV_BUFF_LEN]={0};

int32 McuCan_fd;

void MCU_CAN_Uart_Init(void)
{

  struct termios options;

  pthread_mutex_init(&gMcuCanSendMutex, NULL);   //

  /* open uart */
  McuCan_fd = open(DEV_TTY_MCU_CAN, O_RDWR|O_NOCTTY);

  if (McuCan_fd < 0)
  {
    printf("ERROR open %s ret=%d\n\r", DEV_TTY_MCU_CAN, McuCan_fd);
    close(McuCan_fd);
  }
  else
  {

  }
  printf("McuCan_fd: %d \n", McuCan_fd);

  /* configure uart */
  tcgetattr(McuCan_fd, &options);
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
  tcflush(McuCan_fd, TCIFLUSH);
  tcsetattr(McuCan_fd, TCSANOW, &options);
}

BOOL ReadMCU_CAN_UartData(uint8 **data, uint16* len)
{
  int ret;
  memset(MCU_Can_Recv_Buff,0,MCU_CAN_RECV_BUFF_LEN);
  if ((ret = read(McuCan_fd, MCU_Can_Recv_Buff, MCU_CAN_RECV_BUFF_LEN-1)) > 0)
  {
    *data = MCU_Can_Recv_Buff;
    *len = (uint16)ret;
    return TRUE;
  }
  else
    return FALSE;
}

uint16 WriteMCU_CAN_UartData(uint8 *data, uint16 Len)
{
  int ret;

  if ((0==Len) || (NULL==data))
    return 0;
  /* write uart */
  pthread_mutex_lock(&gMcuCanSendMutex);       //互斥锁
  ret = write(McuCan_fd, data, Len);
  pthread_mutex_unlock(&gMcuCanSendMutex);     //互斥锁解锁

  if (ret != Len)
  {
    printf("ERROR MCU_CAN write ret=%d\n", ret);
    return 0;
  }
  else
    return Len;
}

/*
*********************************************************************************************************
*Function name	:CANWrite
*Description	:通过CAN1发送数据
*Arguments  	:len-数据长度
*				 FF -帧格式，STD_FRAME=标准帧，EXT_FRAME:扩展帧
*                ID -帧ID
*                data-数据指针
*                ch - CAN编号  CAN_CHANNEL1\CAN_CHANNEL2
*Returns    	:none
*Author			:lxf
*Date			:2019-04-02
*Modified		:
*********************************************************************************************************
*/
#if 0
void CanWrite(uint8 ch, unsigned char FF, unsigned int ID, unsigned char len, unsigned char *data)
{
  uint8 SendBuff[25] = {0};

  SendBuff[0] = 0x7E;
  SendBuff[1] = 0;
  SendBuff[2] = 17;
  SendBuff[3] = 0x02;         //Command
  if (stuMcuCanTx.SN>=255)
    stuMcuCanTx.SN = 0;
  else
    stuMcuCanTx.SN++;

  SendBuff[4] = stuMcuCanTx.SN;
  SendBuff[5] = ch;
  SendBuff[6] = FF;
  SendBuff[7] = (uint8)(ID>>24);
  SendBuff[8] = (uint8)(ID>>16);
  SendBuff[9] = (uint8)(ID>>8);
  SendBuff[10] = (uint8)(ID&0xFF);
  memcpy(&SendBuff[11],data,8);
  SendBuff[19] = SumCalc(&SendBuff[3],16);
  SendBuff[20] = 0x0D;
  SendBuff[21] = 0x0A;

  WriteMCU_CAN_UartData(SendBuff, 22);
}
#endif

void CanWrite(uint8 ch, unsigned char FF, unsigned int ID, unsigned char len, unsigned char *data)
{
  uint8 SendBuff[30] = {0};

  SendBuff[0] = 0x7E;
  SendBuff[1] = 0;
  SendBuff[2] = 22;
  SendBuff[3] = 0x02;         //Command
  if (stuMcuCanTx.SN>=255)
    stuMcuCanTx.SN = 0;
  else
    stuMcuCanTx.SN++;
  SendBuff[4] = stuMcuCanTx.SN;

  SendBuff[5] = 1;      //TLVnum
  SendBuff[6] = 0x06;   //T=0x06 CAN透传
  SendBuff[7] = 0;      //L-LenH
  SendBuff[8] = 15;     //L-LenL

  SendBuff[9] = ch;
  SendBuff[10] = FF;
  SendBuff[11] = (uint8)(ID>>24);
  SendBuff[12] = (uint8)(ID>>16);
  SendBuff[13] = (uint8)(ID>>8);
  SendBuff[14] = (uint8)(ID&0xFF);
  SendBuff[15] = 8;      //Candatalen
  memcpy(&SendBuff[16],data,8);
  SendBuff[24] = SumCalc(&SendBuff[3],21);
  SendBuff[25] = 0x0D;
  SendBuff[26] = 0x0A;

  WriteMCU_CAN_UartData(SendBuff, 27);
}
