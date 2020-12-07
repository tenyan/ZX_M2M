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
 * 2019-03-11, by  lxf, �������ļ�
 *
 */

//-----ͷ�ļ�����------------------------------------------------------------
#include "config.h"
//#define DEV_TTY "/dev/ttyUSB2"
#define DEV_TTY "/dev/smd8"

//-----�ⲿ��������------------------------------------------------------------

#define GSM_DMA_UART_Tx_BSIZE        1500
uint8 GSM_DMA_UART_Tx_Buff[GSM_DMA_UART_Tx_BSIZE];

#define GSM_DMA_UART_Rx_BSIZE        1500
uint8 GSM_DMA_UART_Rx_Buff[GSM_DMA_UART_Rx_BSIZE];

uint8 GSM_UART_Rx_Buff[GSM_DMA_UART_Rx_BSIZE];
uint16 GSM_UART_Rx_Cnt;
int32 gGsmAT_fd = -1;


int32 OpenGsmAT_DEV_TTY(void)
{

	struct termios options;
	
//��ʧ�ܣ���δ���?	   
	gGsmAT_fd = open(DEV_TTY, O_RDWR|O_NOCTTY);
	if (gGsmAT_fd < 0) {
		printf("ERROR open %s ret=%d\n\r", DEV_TTY, gGsmAT_fd);
		//return -1;
		return gGsmAT_fd;
	}
    printf("gGsmAT_fd: %d \n", gGsmAT_fd);

	tcgetattr(gGsmAT_fd, &options);
	options.c_cflag &= ~PARENB;
	options.c_cflag &= ~CSTOPB;
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;
	options.c_cc[VTIME] = 10; // read timeout 10*100ms
	options.c_cc[VMIN]  = 0;
	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	options.c_oflag &= ~OPOST;
	options.c_iflag &= ~(ICRNL | IXON);
	cfsetispeed(&options, B115200);
	cfsetospeed(&options, B115200);
	options.c_cflag |= (CLOCAL | CREAD);
	tcflush(gGsmAT_fd, TCIFLUSH);
	tcsetattr(gGsmAT_fd, TCSANOW, &options);
	return gGsmAT_fd;
}



//-----�ļ�GsmHardWareLayer.c����---------------------------------------------
