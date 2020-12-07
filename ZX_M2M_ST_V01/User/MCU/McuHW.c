/*
 * Copyright(c)2019, 硬件研发部
 * All right reserved
 *
 * 文件名称: can.c
 * 版本号  : V1.0
 * 文件标识:
 * 文件描述: 本文件为can
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
OS_EVENT *CAN1ReadSem;			// 收到CAN数据的信号量	
//-----内部变量定义------------------------------------------------------------

MessageDetail MessageCAN1_0;      // 引用CAN0通道帧变量
MessageDetail MessageCAN1_1;
MessageDetail MessageCAN2_0;      // 引用CAN0通道帧变量
MessageDetail MessageCAN2_1;
void CAN1_RX0_IRQHandler(void);
void CAN1_RX1_IRQHandler(void);
extern void DealCan1Message(MessageDetail msg);
extern STU_CommState m_stuCommState;
//PA2--CAN_PWR_EN
void McuIOPinInit()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/* Enable GPIOG's AHB interface clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	/* Configure PG6 in output pushpull mode */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	/* Set PA2 to high level */
	POWERON_CAN();
/*
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	//POWEROFF_485();
*/
	//PA5--CAN1_LED PA6--GPS_LED   PA7--CAN2_LED 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//CAN1_LED_ON();
	//CAN2_LED_ON();

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

#define USE_CAN1_BRT_250K		1
//#define USE_CAN1_BRT_500K
//#define USE_CAN1_BRT_1000K

#define USE_CAN2_BRT_250K		1
//#define USE_CAN2_BRT_500K
//#define USE_CAN2_BRT_1000K

void McuHwInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	CAN_InitTypeDef CAN_InitStructure;
	CAN_FilterInitTypeDef CAN_FilterInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	McuIOPinInit();

#if	1
	/* CAN GPIOs configuration ***********************************/
	/* Enable GPIOD clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	/* Connect PA12 to CAN1_Tx pin */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1);
	/* Connect PA11 to CAN1_Rx pin */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1);
	/* Configure CAN1_Rx(PD0) and CAN1_Tx(PD1) pins */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	/* CAN configuration *****************************************/
	/* Enable CAN1 clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	/* CAN cell init */
	CAN_InitStructure.CAN_TTCM = DISABLE;
	CAN_InitStructure.CAN_ABOM = ENABLE;//DISABLE;
	CAN_InitStructure.CAN_AWUM = DISABLE;
	CAN_InitStructure.CAN_NART = DISABLE;
	CAN_InitStructure.CAN_RFLM = DISABLE;
	CAN_InitStructure.CAN_TXFP = DISABLE;
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;//CAN_Mode_LoopBack;
	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;

	#if 0
	//USE_CAN1_BRT_1000K
	CAN_InitStructure.CAN_BS1 = CAN_BS1_6tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_8tq;
	CAN_InitStructure.CAN_Prescaler = 2;
	#endif

	#if 0 
	//USE_CAN1_BRT_500K
	CAN_InitStructure.CAN_BS1 = CAN_BS1_4tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_1tq;
	CAN_InitStructure.CAN_Prescaler = 10;
	#endif

	#if 0 
	//USE_CAN1_BRT_500K_1
	CAN_InitStructure.CAN_BS1 = CAN_BS1_3tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_1tq;
	CAN_InitStructure.CAN_Prescaler = 12;
	#endif

	#if 0
	CAN_InitStructure.CAN_BS1 = CAN_BS1_4tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_3tq;
	CAN_InitStructure.CAN_Prescaler = 15;
	#endif

	#if 0
	//USE_CAN1_BRT_250K_1
	CAN_InitStructure.CAN_BS1 = CAN_BS1_7tq
	CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
	CAN_InitStructure.CAN_Prescaler = 12;
	#endif

	#if 1
	//USE_CAN1_BRT_250K_2
	CAN_InitStructure.CAN_BS1 = CAN_BS1_6tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_1tq;
	CAN_InitStructure.CAN_Prescaler = 15;
	#endif
    CAN_DeInit(CAN1);	
	CAN_Init(CAN1, &CAN_InitStructure);

	/* CAN filter init */
	/*
	CAN_FilterInitStructure.CAN_FilterNumber = 0;
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);
	*/
	#if 1
	/* Enable CAN1 RX0 interrupt IRQ channel  */
  	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX1_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x1;
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
	#if 0
	NVIC_InitStructure.NVIC_IRQChannel = CAN1_SCE_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x2;
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
	#endif
	
	CAN_ITConfig(CAN1,CAN_IT_FMP0, ENABLE);
    CAN_ITConfig(CAN1,CAN_IT_FMP1, ENABLE);
	#if 0
	CAN_ITConfig(CAN1,CAN_IT_EPV|CAN_IT_EWG|CAN_IT_ERR|CAN_IT_BOF|CAN_IT_LEC, ENABLE);
	CAN_ClearFlag(CAN1, CAN_FLAG_LEC);
	#endif
	#endif
#endif

#if	1
	/* CAN GPIOs configuration ***********************************/
	/* Enable GPIOD clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	/* Connect PB13 to CAN2_Tx pin */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2);
	/* Connect PB12 to CAN2_Rx pin */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_CAN2);
	/* Configure CAN1_Rx(PB12) and CAN1_Tx(PB13) pins */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	/* CAN configuration *****************************************/
	/* Enable CAN2 clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);
	/* CAN cell init */
	CAN_InitStructure.CAN_TTCM = DISABLE;
	CAN_InitStructure.CAN_ABOM = ENABLE;
	CAN_InitStructure.CAN_AWUM = DISABLE;
	CAN_InitStructure.CAN_NART = DISABLE;
	CAN_InitStructure.CAN_RFLM = DISABLE;
	CAN_InitStructure.CAN_TXFP = DISABLE;
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;//CAN_Mode_LoopBack;
	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;

	#if 0 
	//USE_CAN2_BRT_1000K
	/* CAN Baudrate = 1MBps (CAN clocked at 30 MHz) */
	CAN_InitStructure.CAN_BS1 = CAN_BS1_6tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_8tq;
	CAN_InitStructure.CAN_Prescaler = 2;
	#endif

	#if 0 
	//USE_CAN2_BRT_500K
	CAN_InitStructure.CAN_BS1 = CAN_BS1_4tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_1tq;
	CAN_InitStructure.CAN_Prescaler = 10;
	#endif

	#if 0 
	//USE_CAN2_BRT_500K_1
	CAN_InitStructure.CAN_BS1 = CAN_BS1_3tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_1tq;
	CAN_InitStructure.CAN_Prescaler = 12;
	#endif

	#if 0 
	//USE_CAN2_BRT_250K
	CAN_InitStructure.CAN_BS1 = CAN_BS1_4tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_3tq;
	CAN_InitStructure.CAN_Prescaler = 15;
	#endif

	#if 0 
	//USE_CAN2_BRT_250K_1
	CAN_InitStructure.CAN_BS1 = CAN_BS1_7tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
	CAN_InitStructure.CAN_Prescaler = 12;
	#endif

	#if 1
	//USE_CAN1_BRT_250K_2
	CAN_InitStructure.CAN_BS1 = CAN_BS1_6tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_1tq;
	CAN_InitStructure.CAN_Prescaler = 15;
	#endif
    CAN_DeInit(CAN2);	

	CAN_Init(CAN2, &CAN_InitStructure);

	 /* Enable CAN2 RX0 interrupt IRQ channel  */
  	NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x2;
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX1_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x3;
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);

	CAN_ITConfig(CAN2,CAN_IT_FMP0, ENABLE);
    CAN_ITConfig(CAN2,CAN_IT_FMP1, ENABLE);
#endif

	/* CAN filter init */
	CAN_FilterInitStructure.CAN_FilterNumber = 0;
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);

	CAN_FilterInitStructure.CAN_FilterNumber = 14;
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);	
	if(NULL==CAN1ReadSem)
		CAN1ReadSem = OSSemCreate(0);
}

/*
*********************************************************************************************************
*Function name	:CAN1Write
*Description	:通过CAN1发送数据
*Arguments  	:len-数据长度
*				 FF -帧格式，STD_FRAME=标准帧，EXT_FRAME:扩展帧
*                ID -帧ID
*                data-数据指针
*Returns    	:none				
*Author			:hhm
*Date			:2011-6-13
*Modified		:                 
*********************************************************************************************************
*/
void CanWrite(uint8 ch, unsigned char FF, unsigned int ID, unsigned char len, unsigned char *data)
{
	CanTxMsg TxMessage;

	if(FF)
   	{
      	TxMessage.ExtId=ID;
	  	TxMessage.IDE=CAN_ID_EXT;
   	}
   	else
   	{
   	  	TxMessage.StdId=ID;
	  	TxMessage.IDE=CAN_ID_STD;
	}
    TxMessage.RTR=CAN_RTR_DATA;
    TxMessage.DLC=len;
    TxMessage.Data[0]=data[0];
    TxMessage.Data[1]=data[1];
	TxMessage.Data[2]=data[2];
    TxMessage.Data[3]=data[3];
	TxMessage.Data[4]=data[4];
    TxMessage.Data[5]=data[5];
	TxMessage.Data[6]=data[6];
    TxMessage.Data[7]=data[7];
	if(CAN_CHANNEL1==ch)
    	CAN_Transmit(CAN1, &TxMessage);
	else if(CAN_CHANNEL2==ch)
		CAN_Transmit(CAN2, &TxMessage);
	else
	{
	}
}




/*******************************************************************************
* Function Name  : USB_LP_CAN1_RX0_IRQHandler
* Description    : This function handles USB Low Priority or CAN RX0 interrupts 
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void CAN1_RX0_IRQHandler(void)
{
    CanRxMsg RxMessage;
	uint32 id;
	uint8 ff,temp;
	
    if(CAN_GetITStatus(CAN1,CAN_IT_FMP0))
    {
        CAN_Receive(CAN1,CAN_FIFO0, &RxMessage);
        CAN_ClearITPendingBit(CAN1,CAN_IT_FMP0);

        if(RxMessage.IDE==CAN_Id_Extended)
		{
			id = RxMessage.ExtId;
		   	ff = EXT_FRAME;    
		}                                
        else
    	{
    		id = RxMessage.StdId;
		    ff = STD_FRAME;
    	}

		temp = CanFrameFilter(id);
       	switch(temp)									//FF为1，ID为29位
       	{
       		case 3:   // 3=该帧只需要上传
				A5_AddCanFrameToBuff(id,ff,RxMessage.DLC, RxMessage.Data);
				break;
			case 2:   // 2=只需要特殊处理
			  	MessageCAN1_0.CANID = id;
	      		MessageCAN1_0.FF = ff;
		   		MessageCAN1_0.LEN = RxMessage.DLC ;      
	         	MessageCAN1_0.CANRE[0]=RxMessage.Data[0];
	         	MessageCAN1_0.CANRE[1]=RxMessage.Data[1]; 
	         	MessageCAN1_0.CANRE[2]=RxMessage.Data[2];
	         	MessageCAN1_0.CANRE[3]=RxMessage.Data[3];
	         	MessageCAN1_0.CANRE[4]=RxMessage.Data[4];
	         	MessageCAN1_0.CANRE[5]=RxMessage.Data[5];
	         	MessageCAN1_0.CANRE[6]=RxMessage.Data[6];
	        	MessageCAN1_0.CANRE[7]=RxMessage.Data[7];
				A5_AddCanFrameToBuff(id,ff,RxMessage.DLC, RxMessage.Data);
			 	OSSemPost(CAN1ReadSem);
				break;
			case 1:
				MessageCAN1_0.CANID = id;
	      		MessageCAN1_0.FF = ff;
		   		MessageCAN1_0.LEN = RxMessage.DLC ;      
	         	MessageCAN1_0.CANRE[0]=RxMessage.Data[0];
	         	MessageCAN1_0.CANRE[1]=RxMessage.Data[1]; 
	         	MessageCAN1_0.CANRE[2]=RxMessage.Data[2];
	         	MessageCAN1_0.CANRE[3]=RxMessage.Data[3];
	         	MessageCAN1_0.CANRE[4]=RxMessage.Data[4];
	         	MessageCAN1_0.CANRE[5]=RxMessage.Data[5];
	         	MessageCAN1_0.CANRE[6]=RxMessage.Data[6];
	        	MessageCAN1_0.CANRE[7]=RxMessage.Data[7];
				A5_AddCanFrameToBuff(id,ff,RxMessage.DLC, RxMessage.Data);
			 	OSSemPost(CAN1ReadSem);
				break; 
			default:
				break;
       	}	
		
		if(id!=0x18ff0001)   //去除CAN诊断仪心跳帧
		{
    		m_stuCommState.usCan1RcvErrTime = 0;
    	 	m_stuCommState.ucCan1CommState = 0;
    	 	m_stuCommState.ucRcvCan1DataFlag = 1;
		}			
    }
}

/*******************************************************************************
* Function Name  : USB_LP_CAN1_RX0_IRQHandler
* Description    : This function handles USB Low Priority or CAN RX0 interrupts 
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void CAN1_RX1_IRQHandler(void)
{
    CanRxMsg RxMessage;
  	uint32 id;
	uint8 ff,temp;
	
    if(CAN_GetITStatus(CAN1,CAN_IT_FMP1))//增加fi1接受327
    {
        CAN_Receive(CAN1,CAN_FIFO1, &RxMessage);
        CAN_ClearITPendingBit(CAN1,CAN_IT_FMP1);

		if(RxMessage.IDE==CAN_Id_Extended)
		{
			id = RxMessage.ExtId;
		   	ff = EXT_FRAME;    
		}                                
        else
    	{
    		id = RxMessage.StdId;
		    ff = STD_FRAME;
    	}
		
       	temp = CanFrameFilter(id);
       	switch(temp)												//FF为1，ID为29位
       	{
       		case 3:
				A5_AddCanFrameToBuff(id,ff,RxMessage.DLC, RxMessage.Data);
				break;
			case 2:
			  	MessageCAN1_1.CANID = id;
	      		MessageCAN1_1.FF = ff;
		   		MessageCAN1_1.LEN = RxMessage.DLC ;      
	         	MessageCAN1_1.CANRE[0]=RxMessage.Data[0];
	         	MessageCAN1_1.CANRE[1]=RxMessage.Data[1]; 
	         	MessageCAN1_1.CANRE[2]=RxMessage.Data[2];
	         	MessageCAN1_1.CANRE[3]=RxMessage.Data[3];
	         	MessageCAN1_1.CANRE[4]=RxMessage.Data[4];
	         	MessageCAN1_1.CANRE[5]=RxMessage.Data[5];
	         	MessageCAN1_1.CANRE[6]=RxMessage.Data[6];
	        	MessageCAN1_1.CANRE[7]=RxMessage.Data[7];
				A5_AddCanFrameToBuff(id,ff,RxMessage.DLC, RxMessage.Data);
			 	OSSemPost(CAN1ReadSem);
				break;
			case 1:
				//AddCanFrameToBuff(id, RxMessage.Data);
				MessageCAN1_1.CANID = id;
	      		MessageCAN1_1.FF = ff;
		   		MessageCAN1_1.LEN = RxMessage.DLC ;      
	         	MessageCAN1_1.CANRE[0]=RxMessage.Data[0];
	         	MessageCAN1_1.CANRE[1]=RxMessage.Data[1]; 
	         	MessageCAN1_1.CANRE[2]=RxMessage.Data[2];
	         	MessageCAN1_1.CANRE[3]=RxMessage.Data[3];
	         	MessageCAN1_1.CANRE[4]=RxMessage.Data[4];
	         	MessageCAN1_1.CANRE[5]=RxMessage.Data[5];
	         	MessageCAN1_1.CANRE[6]=RxMessage.Data[6];
	        	MessageCAN1_1.CANRE[7]=RxMessage.Data[7];
				A5_AddCanFrameToBuff(id,ff,RxMessage.DLC, RxMessage.Data);
			 	OSSemPost(CAN1ReadSem);
				break; 
			default:
				break;
       	}	
		
		if(id!=0x18ff0001)   //去除CAN诊断仪心跳帧
		{
    		m_stuCommState.usCan1RcvErrTime = 0;
    	 	m_stuCommState.ucCan1CommState = 0;
    	 	m_stuCommState.ucRcvCan1DataFlag = 1;
		}			
    }
}

/*
void CAN1_SCE_IRQHandler()
{
	if(CAN_GetITStatus(CAN1, CAN_IT_EWG))
		CAN_ClearITPendingBit(CAN1, CAN_IT_EWG);
	if(CAN_GetITStatus(CAN1, CAN_IT_EPV))
		CAN_ClearITPendingBit(CAN1, CAN_IT_EPV);
	if(CAN_GetITStatus(CAN1, CAN_IT_BOF))
		CAN_ClearITPendingBit(CAN1, CAN_IT_BOF);
	if(CAN_GetITStatus(CAN1, CAN_IT_LEC))
		CAN_ClearITPendingBit(CAN1, CAN_IT_LEC);
	if(CAN_GetITStatus(CAN1, CAN_IT_ERR))
		CAN_ClearITPendingBit(CAN1, CAN_IT_ERR);
	
	CAN_ClearFlag(CAN1, CAN_FLAG_LEC);
}
*/

/*******************************************************************************
* Function Name  : USB_LP_CAN1_RX0_IRQHandler
* Description    : This function handles USB Low Priority or CAN RX0 interrupts 
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void CAN2_RX0_IRQHandler(void)
{
    CanRxMsg RxMessage;
	uint32 id;
	uint8 ff,temp;
	
    if(CAN_GetITStatus(CAN2,CAN_IT_FMP0))
    {
        CAN_Receive(CAN2,CAN_FIFO0, &RxMessage);
        CAN_ClearITPendingBit(CAN2,CAN_IT_FMP0);
		
        if(RxMessage.IDE==CAN_Id_Extended)
		{
			id = RxMessage.ExtId;
		   	ff = EXT_FRAME;    
		}                                
        else
    	{
    		id = RxMessage.StdId;
		    ff = STD_FRAME;
    	}

		temp = CanFrameFilter(id);
       	switch(temp)												//FF为1，ID为29位
       	{
       	
       		case 3:
				A5_AddCanFrameToBuff(id,ff,RxMessage.DLC, RxMessage.Data);
				break;
			case 2:
			//	MessageCAN2_0.Channel = CAN_CHANNEL2;
			  	MessageCAN2_0.CANID = id;
	      		MessageCAN2_0.FF = ff;
		   		MessageCAN2_0.LEN = RxMessage.DLC ;      
	         	MessageCAN2_0.CANRE[0]=RxMessage.Data[0];
	         	MessageCAN2_0.CANRE[1]=RxMessage.Data[1]; 
	         	MessageCAN2_0.CANRE[2]=RxMessage.Data[2];
	         	MessageCAN2_0.CANRE[3]=RxMessage.Data[3];
	         	MessageCAN2_0.CANRE[4]=RxMessage.Data[4];
	         	MessageCAN2_0.CANRE[5]=RxMessage.Data[5];
	         	MessageCAN2_0.CANRE[6]=RxMessage.Data[6];
	        	MessageCAN2_0.CANRE[7]=RxMessage.Data[7];
				A5_AddCanFrameToBuff(id,ff,RxMessage.DLC, RxMessage.Data);
			 	OSSemPost(CAN1ReadSem);
				break;
			case 1:
			//	MessageCAN2_0.Channel = CAN_CHANNEL2;
				MessageCAN2_0.CANID = id;
	      		MessageCAN2_0.FF = ff;
		   		MessageCAN2_0.LEN = RxMessage.DLC ;      
	         	MessageCAN2_0.CANRE[0]=RxMessage.Data[0];
	         	MessageCAN2_0.CANRE[1]=RxMessage.Data[1]; 
	         	MessageCAN2_0.CANRE[2]=RxMessage.Data[2];
	         	MessageCAN2_0.CANRE[3]=RxMessage.Data[3];
	         	MessageCAN2_0.CANRE[4]=RxMessage.Data[4];
	         	MessageCAN2_0.CANRE[5]=RxMessage.Data[5];
	         	MessageCAN2_0.CANRE[6]=RxMessage.Data[6];
	        	MessageCAN2_0.CANRE[7]=RxMessage.Data[7];
				A5_AddCanFrameToBuff(id,ff,RxMessage.DLC, RxMessage.Data);
			 	OSSemPost(CAN1ReadSem);
				break; 
			default:
				break;
       	}	
		if(id!=0x18ff0001)   //去除CAN诊断仪心跳帧
		{
    		m_stuCommState.usCan2RcvErrTime = 0;
    		m_stuCommState.ucCan2CommState = 0;
            m_stuCommState.ucRcvCan2DataFlag = 1;
		}
    }
}

/*******************************************************************************
* Function Name  : USB_LP_CAN1_RX0_IRQHandler
* Description    : This function handles USB Low Priority or CAN RX0 interrupts 
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void CAN2_RX1_IRQHandler(void)
{
  	CanRxMsg RxMessage;
   	uint32 id;
	uint8 ff,temp;
	
    if(CAN_GetITStatus(CAN2,CAN_IT_FMP1))//增加fi1接受327
    {
        CAN_Receive(CAN2,CAN_FIFO1, &RxMessage);
        CAN_ClearITPendingBit(CAN2,CAN_IT_FMP1);

		if(RxMessage.IDE==CAN_Id_Extended)
		{
			id = RxMessage.ExtId;
		   	ff = EXT_FRAME;    
		}                                
        else
    	{
    		id = RxMessage.StdId;
		    ff = STD_FRAME;
    	}
		
       	temp = CanFrameFilter(id);
       	switch(temp)												//FF为1，ID为29位
       	{
           
			case 3:
				A5_AddCanFrameToBuff(id,ff,RxMessage.DLC, RxMessage.Data);
				break;
			case 2:
			//	MessageCAN2_0.Channel = CAN_CHANNEL2;
			  	MessageCAN2_1.CANID = id;
	      		MessageCAN2_1.FF = ff;
		   		MessageCAN2_1.LEN = RxMessage.DLC ;      
	         	MessageCAN2_1.CANRE[0]=RxMessage.Data[0];
	         	MessageCAN2_1.CANRE[1]=RxMessage.Data[1]; 
	         	MessageCAN2_1.CANRE[2]=RxMessage.Data[2];
	         	MessageCAN2_1.CANRE[3]=RxMessage.Data[3];
	         	MessageCAN2_1.CANRE[4]=RxMessage.Data[4];
	         	MessageCAN2_1.CANRE[5]=RxMessage.Data[5];
	         	MessageCAN2_1.CANRE[6]=RxMessage.Data[6];
	        	MessageCAN2_1.CANRE[7]=RxMessage.Data[7];
				A5_AddCanFrameToBuff(id,ff,RxMessage.DLC, RxMessage.Data);
			 	OSSemPost(CAN1ReadSem);
				break;
			case 1:
			//	MessageCAN2_0.Channel = CAN_CHANNEL2;
				MessageCAN2_1.CANID = id;
	      		MessageCAN2_1.FF = ff;
		   		MessageCAN2_1.LEN = RxMessage.DLC ;      
	         	MessageCAN2_1.CANRE[0]=RxMessage.Data[0];
	         	MessageCAN2_1.CANRE[1]=RxMessage.Data[1]; 
	         	MessageCAN2_1.CANRE[2]=RxMessage.Data[2];
	         	MessageCAN2_1.CANRE[3]=RxMessage.Data[3];
	         	MessageCAN2_1.CANRE[4]=RxMessage.Data[4];
	         	MessageCAN2_1.CANRE[5]=RxMessage.Data[5];
	         	MessageCAN2_1.CANRE[6]=RxMessage.Data[6];
	        	MessageCAN2_1.CANRE[7]=RxMessage.Data[7];
				A5_AddCanFrameToBuff(id,ff,RxMessage.DLC, RxMessage.Data);
			 	OSSemPost(CAN1ReadSem);
				break;
			default:
				break;
       	}	
		if(id!=0x18ff0001)   //去除CAN诊断仪心跳帧
		{
    		m_stuCommState.usCan2RcvErrTime = 0;
    		m_stuCommState.ucCan2CommState = 0;
    		m_stuCommState.ucRcvCan2DataFlag = 1;
		}
    }
}

#if 0
void CAN2_RX1_IRQHandler(void)
{
    CanRxMsg RxMessage;
   
    if(CAN_GetITStatus(CAN2,CAN_IT_FMP1))//增加fi1接受327
    {
        CAN_Receive(CAN2,CAN_FIFO1, &RxMessage);
        CAN_ClearITPendingBit(CAN2,CAN_IT_FMP1);

		MessageCAN1_1.LEN =RxMessage.DLC ;                           //获取帧长度
        if(RxMessage.IDE==CAN_Id_Extended)
		{
		   	MessageCAN1_1.FF =EXT_FRAME;    
		}                                
        else
    	{
		   	MessageCAN1_1.FF =STD_FRAME;
    	}
	    if(MessageCAN1_1.FF)												//FF为1，ID为29位
	    {
			MessageCAN1_1.CANID = RxMessage.ExtId ;
	    }
	    else                                                             //FF为0 ，ID为11位
	    {	
			MessageCAN1_1.CANID = RxMessage.StdId;
	    }
	   
        MessageCAN1_1.CANRE[0]=RxMessage.Data[0] ;
        MessageCAN1_1.CANRE[1]=RxMessage.Data[1] ; 
        MessageCAN1_1.CANRE[2]=RxMessage.Data[2] ;
        MessageCAN1_1.CANRE[3]=RxMessage.Data[3] ;
        MessageCAN1_1.CANRE[4]=RxMessage.Data[4] ;
        MessageCAN1_1.CANRE[5]=RxMessage.Data[5] ;
        MessageCAN1_1.CANRE[6]=RxMessage.Data[6] ;
        MessageCAN1_1.CANRE[7]=RxMessage.Data[7] ;
		// OSSemPost(CAN1ReadSem);
		DealCan1Message(MessageCAN1_1);
		m_stuCommState.usCan2RcvErrTime = 0;
		m_stuCommState.ucCan2CommState = 0;
		m_stuCommState.ucRcvCan2DataFlag = 1;
    }
}
#endif

/*
*********************************************************************************************************
*Function name	:WaitForCAN1DataFrame
*Description	:等待来自CAN1的数据帧
*Arguments  	:timeout -> 等待超时的时间
*Returns    	:CAN1_DATA_OK -> 收到数据
*                CAN1_DATA_TIMEOUT -> 等待数据超时
*Author			:hhm
*Date			:2011-6-13
*Modified		:                 
*********************************************************************************************************
*/
uint8 WaitForCAN1Message(uint16 timeout)
{
	uint8 err;
	
	OSSemPend(CAN1ReadSem, timeout, &err);
	if(err == OS_ERR_NONE)
		return CAN1_DATA_OK;
	else
		return CAN1_DATA_TIMEOUT;
}


/*
*********************************************************************************************************
*Function name	:Sleep_Can_Io
*Description	:系统休眠CAN管脚配置为IO下拉输入
*Arguments  	:
*Returns    	:
*                
*Author			:yzj
*Date			:2014-12-11
*Modified		:                 
*********************************************************************************************************
*/
void Sleep_Can_Io(void)
{
     GPIO_InitTypeDef       GPIO_InitStructure;


	 /* Configure CAN pin: RX */
     GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
     GPIO_Init(GPIOD, &GPIO_InitStructure);

  
      /* Configure CAN pin: TX */
     GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
     GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
     GPIO_Init(GPIOD, &GPIO_InitStructure);
}
