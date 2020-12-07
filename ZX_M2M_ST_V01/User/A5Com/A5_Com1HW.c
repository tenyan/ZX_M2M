/*
 * Copyright(c)2019, Ӳ���з���
 * All right reserved
 *
 * �ļ�����: A5_Com1HW.c
 * �汾��  : V1.0
 * �ļ���ʶ:
 * �ļ�����: ���ļ�Ϊuart6
 *
 *-----------------------------------------------------------------------------
 * �޸ļ�¼
 *-----------------------------------------------------------------------------
 *
 * 2019-3-22, �������ļ�
 *
 */

//-----ͷ�ļ�����------------------------------------------------------------
#include "config.h"


//-----�ⲿ��������------------------------------------------------------------
#define RS232_BRT		115200
#define RS485_BRT		RS232_BRT

#define RS232_DMA_UART_Tx_BSIZE        1300
uint8 RS232_DMA_UART_Tx_Buff[RS232_DMA_UART_Tx_BSIZE];

#define RS232_DMA_UART_Rx_BSIZE        1300
uint8 RS232_DMA_UART_Rx_Buff[RS232_DMA_UART_Rx_BSIZE];

uint8 RS232_UART_Rx_Buff[RS232_DMA_UART_Rx_BSIZE];
uint16 RS232_UART_Rx_Cnt;


OS_EVENT   *UartRS232ReadSem;    //���ڽ����ź���
OS_EVENT   *UartRS232WriteSem;   //���ڷ����ź���


BOOL ReadRS232UartData(uint8 **data, uint16* len)
{
	uint8 err = 0;
	
	OSSemPend(UartRS232ReadSem, 0, &err);
	if(0==err)
	{
		*data = RS232_UART_Rx_Buff;
		*len = RS232_UART_Rx_Cnt;
		return TRUE;
	}
	else
		return FALSE;
}

void RS232_UART_Write(uint8 *Data, uint16 len)
{
    uint8 err = 0;
    if((0==len) || (len>RS232_DMA_UART_Tx_BSIZE))
		return;
    OSSemPend(UartRS232WriteSem, 0, &err);
    memcpy(RS232_DMA_UART_Tx_Buff, Data, len);
   	DMA_SetCurrDataCounter(DMA2_Stream6, len); 	// ����Ҫ���͵��ֽ���Ŀ
	DMA_Cmd(DMA2_Stream6, ENABLE);        		//��ʼDMA����
}
/*
void RS485_UART_Write(uint8 *Data, uint16 len)
{
	RS485_CON_SEND();
	OSTimeDly(OS_TICKS_PER_SEC/1000);
	RS232_UART_Write(Data, len);
	OSTimeDly((OS_TICKS_PER_SEC/1000) * (2+len*10*1000/RS485_BRT));
	RS485_CON_RCV();
}
*/
void USART6_DMA_Config(void)  
{  
    DMA_InitTypeDef DMA_InitStructure;  
  
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);//����DMAʱ�� 

	//����DMA
	DMA_DeInit(DMA2_Stream1);
    DMA_InitStructure.DMA_Channel = DMA_Channel_5;//ͨ��5   
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART6->DR);
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)RS232_DMA_UART_Rx_Buff; //�ڴ��ַ  
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;//���赽�ڴ�  
    DMA_InitStructure.DMA_BufferSize = RS232_DMA_UART_Rx_BSIZE; //�����С  
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//�����ַ����  
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//�ڴ��ַ��  
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//�������ݴ�С1byte  
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//�ڴ����ݴ�С1byte  
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//ѭ��ģʽ  
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;//���ȼ���  
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;// ����fifo  
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull; //  
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//  
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//  
    DMA_Init(DMA2_Stream1, &DMA_InitStructure);//��ʼ��dma2��1  
    DMA_Cmd(DMA2_Stream1, ENABLE);//����dma2��1  

	//����DMA����
	DMA_InitStructure.DMA_Channel =DMA_Channel_5;         
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; /* Specifies whether the Peripheral address register should be incremented or not */
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; /* Specifies whether the memory address register should be incremented or not */
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; 
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; 
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal; 
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh; 
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable; 
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull; 
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single; 
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single; 
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral; 
	        
	DMA_DeInit(DMA2_Stream6);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART6->DR); 
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)RS232_DMA_UART_Tx_Buff;;   
	DMA_InitStructure.DMA_BufferSize = RS232_DMA_UART_Tx_BSIZE;         
	DMA_Init(DMA2_Stream6,&DMA_InitStructure);
	//DMA_Cmd(UART3_TX_DMAy_Streamx, ENABLE);/* move to rt_serial_enable_dma() by RTT */
	        
	DMA_ITConfig(DMA2_Stream6, DMA_IT_TC | DMA_IT_TE, ENABLE);
	DMA_ClearFlag(DMA2_Stream6, DMA_FLAG_TCIF6| DMA_FLAG_TEIF6);
}  

//����1������ü������ж����ã�
void UART6_Config(void)  
{  
    GPIO_InitTypeDef GPIO_InitStructure;  
    NVIC_InitTypeDef   NVIC_InitStructure;  
    USART_InitTypeDef USART_InitStructure;   
    /* Enable GPIO clock */  
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);  
    /* Enable UART clock */  
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);  
    /* Connect PXx to USARTx_Tx*/  
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);  
    /* Connect PXx to USARTx_Rx*/  
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);  

	/*USART_DeInit(USART6); */
	
    /* Configure USART Tx as alternate function  */  
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
    GPIO_Init(GPIOC, &GPIO_InitStructure);  
    /* Configure USART Rx as alternate function  */  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;  
    GPIO_Init(GPIOC, &GPIO_InitStructure);  
    USART_InitStructure.USART_BaudRate = RS232_BRT;  
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  
    USART_InitStructure.USART_StopBits = USART_StopBits_1;  
    USART_InitStructure.USART_Parity = USART_Parity_No;  
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  
    /* USART configuration */  
    USART_Init(USART6, &USART_InitStructure);  
	
    NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 13;  
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
    NVIC_Init(&NVIC_InitStructure); 

	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream6_IRQn;  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 14;  
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
    NVIC_Init(&NVIC_InitStructure); 
	
    USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);  
    /* Enable USART */  
    USART_Cmd(USART6, ENABLE);  
}  
//������������ܺ�����
void RS232_Uart_Init(void)  
{ 

	if(NULL==UartRS232ReadSem)
		UartRS232ReadSem = OSSemCreate(0);
	if(NULL==UartRS232WriteSem)
		UartRS232WriteSem = OSSemCreate(1);

  	UART6_Config();  
  	USART6_DMA_Config();//��Ҫ�����������ַ���ڴ��ַ�Լ���������С�����úú�DMA�ͻ��Զ��İѴ������ݴ浽��Ӧ���ڴ��ַ��  
  	USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE);	//���ô���6DMA����  
	USART_DMACmd(USART6, USART_DMAReq_Tx, ENABLE);  //��������DMA����
}  
//����1�����жϷ�������
void USART6_IRQHandler(void)  
{  
    u16 i;  
    if(USART_GetITStatus(USART6, USART_IT_IDLE) != RESET)//���Ϊ���������ж�  
    {  
        i = USART6->SR;  
        i = USART6->DR;  
        USART_ClearITPendingBit(USART6, USART_IT_IDLE);  
        DMA_Cmd(DMA2_Stream1, DISABLE);//�ر�DMA,��ֹ�������������  
        //USART_DMACmd(USART1, USART_DMAReq_Rx, DISABLE);	//���ô���1DMA����  
        RS232_UART_Rx_Cnt = RS232_DMA_UART_Rx_BSIZE - DMA_GetCurrDataCounter(DMA2_Stream1);
		memcpy(RS232_UART_Rx_Buff, RS232_DMA_UART_Rx_Buff, RS232_UART_Rx_Cnt);
		DMA_SetCurrDataCounter(DMA2_Stream1, RS232_DMA_UART_Rx_BSIZE);
		DMA_Cmd(DMA2_Stream1, ENABLE);//����DMA
        //USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);	//���ô���1DMA����  
        OSSemPost(UartRS232ReadSem);
    }  
}  
/*********************DMA�жϷ�����****************************/
void DMA2_Stream6_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA2_Stream6,DMA_IT_TCIF6))
    {
     	OSSemPost(UartRS232WriteSem);
    }
    /* clear DMA flag */
    DMA_ClearFlag(DMA2_Stream6,DMA_FLAG_TCIF6 | DMA_FLAG_TEIF6);
}

#if 0
void McuIOPinInit()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	//PD12-- PWRC_485  PD15--485_DE
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	/* Configure PD12 PD15 in output pushpull mode */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure); 
	POWERON_485();
	RS485_CON_RCV();

	//PE4--CAN_BUS1_LED PE5--CAN_BUS2_LED
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	/* Configure PE4 in output pushpull mode */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5|GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure); 
	CAN1_LED_ON();
	CAN2_LED_ON();
	BLU_LED_ON();	
}

void McuHwInit(void)
{
	//McuIOPinInit();
	RS232_Uart_Init();
}

// UART ����ΪGPIO����,��UARTʱ�ӣ���ҪΪ������ʱ���͹���
void Mcu_Uart_Deinit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;  
	
	/*USART_DeInit(USART6);*/
	/* Configure USART Tx as alternate function  */  
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;  
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
    GPIO_Init(GPIOC, &GPIO_InitStructure);  
	
    /* Configure USART Rx as alternate function  */  
  //  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;  
  //  GPIO_Init(GPIOD, &GPIO_InitStructure);  
}
#endif



