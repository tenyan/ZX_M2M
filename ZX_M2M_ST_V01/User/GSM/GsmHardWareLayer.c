/*
 * Copyright(c)2019, Ӳ���з���
 * All right reserved
 *
 * �ļ�����: PcDebug.c
 * �汾��  : V1.0
 * �ļ���ʶ:
 * �ļ�����: ���ļ�ΪPcDebug����ģ��Э��㴦����ļ�
 *
 *-----------------------------------------------------------------------------
 * �޸ļ�¼
 *-----------------------------------------------------------------------------
 *
 * 2019-05-1, by  lxf, �������ļ�
 *
 */

//-----ͷ�ļ�����------------------------------------------------------------
#include "config.h"
#include "GsmHardWareLayer.h"
#include "App_cfg.h"
//-----�ⲿ��������------------------------------------------------------------

#define GSM_DMA_UART_Tx_BSIZE        1500
uint8 GSM_DMA_UART_Tx_Buff[GSM_DMA_UART_Tx_BSIZE];

#define GSM_DMA_UART_Rx_BSIZE        500
uint8 GSM_DMA_UART_Rx_Buff[GSM_DMA_UART_Rx_BSIZE];

uint8 GSM_UART_Rx_Buff[GSM_DMA_UART_Rx_BSIZE];
uint16 GSM_UART_Rx_Cnt;

OS_EVENT   *UartGSMReadSem;   //gps���ڽ����ź���
OS_EVENT   *UartGSMWriteSem;   //gps���ڷ����ź���



BOOL ReadGsmUartData(uint8 **data, uint16* len)
{
	uint8 err = 0;
	
	OSSemPend(UartGSMReadSem, 0, &err);
	if(0==err)
	{
		*data = GSM_UART_Rx_Buff;
		*len = GSM_UART_Rx_Cnt;
		return TRUE;
	}
	else
		return FALSE;
}


void GSM_UART_Write(uint8 *Data, uint16 len)
{
    uint8 err = 0;

	
	if((0==len) || (len>GSM_DMA_UART_Tx_BSIZE))
		return;
    OSSemPend(UartGSMWriteSem, 0, &err);
    memcpy(GSM_DMA_UART_Tx_Buff, Data, len);
   	DMA_SetCurrDataCounter(DMA1_Stream3, len); 	// ����Ҫ���͵��ֽ���Ŀ
	DMA_Cmd(DMA1_Stream3, ENABLE);        		//��ʼDMA����
}



void USART3_DMA_Config(void)  
{  
    DMA_InitTypeDef DMA_InitStructure;  
  
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);//����DMAʱ�� 

	//����DMA
    DMA_InitStructure.DMA_Channel = DMA_Channel_4;//ͨ��4   
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART3->DR);
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)GSM_DMA_UART_Rx_Buff; //�ڴ��ַ  
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;//���赽�ڴ�  
    DMA_InitStructure.DMA_BufferSize = GSM_DMA_UART_Rx_BSIZE; //�����С  
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
    DMA_Init(DMA1_Stream1, &DMA_InitStructure);//��ʼ��dma2��5  
    DMA_Cmd(DMA1_Stream1, ENABLE);//����dma2��5  

	//����DMA����
	DMA_InitStructure.DMA_Channel =DMA_Channel_4;         
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
	        
	DMA_DeInit(DMA1_Stream3);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART3->DR); 
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)GSM_DMA_UART_Tx_Buff;;   
	DMA_InitStructure.DMA_BufferSize = GSM_DMA_UART_Tx_BSIZE;         
	DMA_Init(DMA1_Stream3,&DMA_InitStructure);
	//DMA_Cmd(UART3_TX_DMAy_Streamx, ENABLE);/* move to rt_serial_enable_dma() by RTT */
	        
	DMA_ITConfig(DMA1_Stream3, DMA_IT_TC | DMA_IT_TE, ENABLE);
	DMA_ClearFlag(DMA1_Stream3, DMA_FLAG_TCIF3| DMA_FLAG_TEIF3);
}  

//����1������ü������ж����ã�
void UART3_Config(void)  
{  
    GPIO_InitTypeDef GPIO_InitStructure;  
    NVIC_InitTypeDef   NVIC_InitStructure;  
    USART_InitTypeDef USART_InitStructure;   
    /* Enable GPIO clock */  
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);  
    /* Enable UART clock */  
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);  
    /* Connect PD8 to USART3_Tx*/  
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_USART3);  
    /* Connect PD9 to USART3_Rx*/  
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_USART3);  
    /* Configure USART Tx as alternate function  */  
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
    GPIO_Init(GPIOC, &GPIO_InitStructure);  
    /* Configure USART Rx as alternate function  */  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;  
    GPIO_Init(GPIOC, &GPIO_InitStructure);  
	
	 /* USART configuration */  
    USART_InitStructure.USART_BaudRate = 115200;  
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  
    USART_InitStructure.USART_StopBits = USART_StopBits_1;  
    USART_InitStructure.USART_Parity = USART_Parity_No;  
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  
    USART_Init(USART3, &USART_InitStructure);  
	
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 7;  
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
    NVIC_Init(&NVIC_InitStructure); 

	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream3_IRQn;  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 8;  
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
    NVIC_Init(&NVIC_InitStructure); 
	
    USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);  
    /* Enable USART */  
    USART_Cmd(USART3, ENABLE);  
}  
//������������ܺ�����
void GSM_Uart_Init(void)  
{  
	if(NULL==UartGSMReadSem)
		UartGSMReadSem = OSSemCreate(0);
	if(NULL==UartGSMWriteSem)
		UartGSMWriteSem = OSSemCreate(1);
  	UART3_Config();  
  	USART3_DMA_Config();//��Ҫ�����������ַ���ڴ��ַ�Լ���������С�����úú�DMA�ͻ��Զ��İѴ������ݴ浽��Ӧ���ڴ��ַ��  
  	USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);	//���ô���1DMA����  
	USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);  //��������DMA����
}  
//����1�����жϷ�������
void USART3_IRQHandler(void)  
{  
    u16 i;  
    if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)//���Ϊ���������ж�  
    {  
        i = USART3->SR;  
        i = USART3->DR;  
        USART_ClearITPendingBit(USART3, USART_IT_IDLE);  
        DMA_Cmd(DMA1_Stream1, DISABLE);//�ر�DMA,��ֹ�������������  
        //USART_DMACmd(USART3, USART_DMAReq_Rx, DISABLE);	//���ô���1DMA���� 
        GSM_UART_Rx_Cnt = GSM_DMA_UART_Rx_BSIZE - DMA_GetCurrDataCounter(DMA1_Stream1);
		memcpy(GSM_UART_Rx_Buff, GSM_DMA_UART_Rx_Buff, GSM_UART_Rx_Cnt);
		DMA_SetCurrDataCounter(DMA1_Stream1, GSM_DMA_UART_Rx_BSIZE);
		DMA_Cmd(DMA1_Stream1, ENABLE);//����DMA
        //USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);	//���ô���1DMA���� 
        OSSemPost(UartGSMReadSem);
    }  
}  
/*********************DMA�жϷ�����****************************/
void DMA1_Stream3_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA1_Stream3,DMA_IT_TCIF3))
    {
     	OSSemPost(UartGSMWriteSem);
    }
    /* clear DMA flag */
    DMA_ClearFlag(DMA1_Stream3,DMA_FLAG_TCIF3 | DMA_FLAG_TEIF3);

}

 
uint8 WriteGsmUartData(uint8 *Data, uint16 Len)
{
	GSM_UART_Write(Data, Len);
//	PC_SendDebugData(Data, Len, DEBUG_AT);
    PC_SendDebugData(Data, Len, DEBUG_GPSMODULE);
	return Len;
}
 

#if 1
/*GSMģ����ؿ��ƹܽ�
  PA8--RING_WAKE PC6--PWR_CTRL 
  PC6--GSM_PWR_EN PC8--GSM_RESET 
  PC9--GSM_WAKEUP PB14--GSM_PWR_KEY
  PB15-GSM_STATUS

*/
void GSM_GPIO_Pin_Init()
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);

  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  	GPIO_Init(GPIOC, &GPIO_InitStructure);
	ModemDTRLow();                         //����GSM
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  	GPIO_Init(GPIOB, &GPIO_InitStructure);

/*
	//GSM_LED
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  	GPIO_Init(GPIOE, &GPIO_InitStructure);


	//����EN  PC4
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_SetBits(GPIOC, GPIO_Pin_4);
	*/
}
#endif


//-----�ļ�PcDebug.c����---------------------------------------------
