/*
 * Copyright(c)2019,硬件研发部
 * All right reserved
 *
 * 文件名称: PcDebug.c
 * 版本号  : V1.0
 * 文件标识:
 * 文件描述: 本文件为PcDebug功能模块协议层处理的文件
 *
 *-----------------------------------------------------------------------------
 * 修改记录
 *-----------------------------------------------------------------------------
 *
 * 2019-03-11, by  lxf, 创建本文件
 *
 */

//-----头文件调用------------------------------------------------------------
#include "config.h"
#include "PcHardware.h"

//-----外部变量定义------------------------------------------------------------


//#define DEBUG_DMA_UART_Tx_BSIZE        1500
//uint8 DEBUG_DMA_UART_Tx_Buff[DEBUG_DMA_UART_Tx_BSIZE];

uint8 DEBUG_DMA_UART_Rx_Buff[DEBUG_DMA_UART_Rx_BSIZE];

//uint8 DEBUG_UART_Rx_Buff[DEBUG_DMA_UART_Rx_BSIZE];
//uint16 DEBUG_UART_Rx_Cnt;


int fd_DEBUG = -1;


static int set_opt(int fd, int nSpeed, int nBits, char nEvent, int nStop,unsigned int cflag)
{
	struct termios newtio, oldtio;
	if(tcgetattr(fd, &oldtio) != 0)
	{
		perror("setupserial 1\n");
		printf("errno=%d\n", errno);
		//printf("errno=%s\n", strerror(errno));
		return -1;
	}
	bzero(&newtio, sizeof(newtio));
	newtio.c_cflag |=CLOCAL | CREAD |cflag;
	//newtio.c_cflag &= ~CSIZE;
	switch(nBits)
	{
		case 7:
			newtio.c_cflag |=CS7;
			break;
		case 8:
			newtio.c_cflag |=CS8;
			break;
	}
	switch(nEvent)
	{
		case 'O':
			newtio.c_cflag |=PARENB;
			newtio.c_cflag |=PARODD;
			newtio.c_iflag |=(INPCK);
			break;
		case 'P':
			newtio.c_cflag |=PARENB;
			newtio.c_cflag |=CMSPAR;
			newtio.c_iflag |=(INPCK);
			break;
		case 'E':
			newtio.c_iflag |=(INPCK);
			newtio.c_cflag |= PARENB;
			newtio.c_cflag &= ~ (PARODD|CMSPAR);
			break;
		case 'N':
			newtio.c_cflag &= ~PARENB;
			break;
	}

	switch(nSpeed)
	{
		case 300:
			cfsetispeed(&newtio, B300);
			cfsetospeed(&newtio, B300);
			break;
		case 600:
			cfsetispeed(&newtio, B600);
			cfsetospeed(&newtio, B600);
			break;
		case 1200:
			cfsetispeed(&newtio, B1200);
			cfsetospeed(&newtio, B1200);
			break;
		case 2400:
			cfsetispeed(&newtio, B2400);
			cfsetospeed(&newtio, B2400);
			break;
		case 4800:
			cfsetispeed(&newtio, B4800);
			cfsetospeed(&newtio, B4800);
			break;
		case 9600:
			cfsetispeed(&newtio, B9600);
			cfsetospeed(&newtio, B9600);
			break;
		case 19200:
			cfsetispeed(&newtio, B19200);
			cfsetospeed(&newtio, B19200);
			break;
		case 38400:
			cfsetispeed(&newtio, B38400);
			cfsetospeed(&newtio, B38400);
			break;
		case 57600:
			cfsetispeed(&newtio, B57600);
			cfsetospeed(&newtio, B57600);
			break;
		case 115200:
			cfsetispeed(&newtio, B115200);
			cfsetospeed(&newtio, B115200);
			break;
		case 230400:
			cfsetispeed(&newtio, B230400);
			cfsetospeed(&newtio, B230400);
			break;
		case 460800:
			cfsetispeed(&newtio, B460800);
			cfsetospeed(&newtio, B460800);
			break;
		case 921600:
			cfsetispeed(&newtio, B921600);
			cfsetospeed(&newtio, B921600);
			break;
		case 1000000:
			cfsetispeed(&newtio, B1000000);
			cfsetospeed(&newtio, B1000000);
			break;
		case 1152000:
			cfsetispeed(&newtio, B1152000);
			cfsetospeed(&newtio, B1152000);
			break;
		case 1500000:
			cfsetispeed(&newtio, B1500000);
			cfsetospeed(&newtio, B1500000);
			break;
		case 2000000:
			cfsetispeed(&newtio, B2000000);
			cfsetospeed(&newtio, B2000000);
			break;
		case 3000000:
			cfsetispeed(&newtio, B3000000);
			cfsetospeed(&newtio, B3000000);
			break;
		case 3200000:
			cfsetispeed(&newtio, B200);
			cfsetospeed(&newtio, B200);
			break;
		case 3686400:
			cfsetispeed(&newtio, B150);
			cfsetospeed(&newtio, B150);
			break;
		default:
			cfsetispeed(&newtio, B115200);
			cfsetospeed(&newtio, B115200);
			break;
	}
	if( nStop == 1)
		newtio.c_cflag &= ~CSTOPB;
	else if(nStop == 2)
		newtio.c_cflag |=CSTOPB;

	newtio.c_cc[VTIME] = 0;
	newtio.c_cc[VMIN] = 1;
	tcflush(fd, TCIFLUSH);
	tcflush(fd, TCOFLUSH);
	if((tcsetattr(fd,TCSANOW,&newtio))!=0)
	{
		perror("com set error\n");
		return -1;
	}

	return 0;
}

#if 0
void DEBUG_Uart_Init(void)  
{
	/* open uart */
	//open(devpath, O_RDWR|O_NOCTTY | O_NDELAY |O_NONBLOCK);
	fd_DEBUG = open(DEV_TTYGS0_CDC_DEBUG, O_RDWR|O_NOCTTY | O_NDELAY |O_NONBLOCK);
	if (fd_DEBUG < 0)
	{
		printf("ERROR open %s ret=%d\n\r", DEV_TTYGS0_CDC_DEBUG, fd_DEBUG);
        close(fd_DEBUG);
		fd_DEBUG = -1;
		return ;
	}
	else
	{
    	
	}
    printf("fd_DEBUG: %d \n", fd_DEBUG);

    set_opt(fd_DEBUG, 115200, 8, 'N', 1,0);
	
}
#endif
#if 1
void DEBUG_Uart_Init(void)  
{ 
    
	struct termios options;

	/* open uart */
	fd_DEBUG = open(DEV_TTYGS0_CDC_DEBUG, O_RDWR|O_NOCTTY);
	if (fd_DEBUG < 0)
	{
		printf("ERROR open %s ret=%d\n\r", DEV_TTYGS0_CDC_DEBUG, fd_DEBUG);
        close(fd_DEBUG);
		fd_DEBUG = -1;
		return ;
	}
	else
	{
    	
	}
    printf("fd_DEBUG: %d \n", fd_DEBUG);

	/* configure uart */
	tcgetattr(fd_DEBUG, &options);
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
	tcflush(fd_DEBUG, TCIFLUSH);
	tcsetattr(fd_DEBUG, TCSANOW, &options);
}  
#endif
BOOL ReadDebugUartData(uint8 **data, uint16* len)
{
	int ret;
    memset(DEBUG_DMA_UART_Rx_Buff,0,DEBUG_DMA_UART_Rx_BSIZE);
	
	if(fd_DEBUG<0)
		return FALSE;
		
	if((ret = read(fd_DEBUG, DEBUG_DMA_UART_Rx_Buff, DEBUG_DMA_UART_Rx_BSIZE-1)) > 0)
	{
		*data = DEBUG_DMA_UART_Rx_Buff;
		*len = (uint16)ret;
		return TRUE;
	}
	else
		return FALSE;
}
uint16 DEBUG_UART_Write(uint8 *data, uint16 Len)
{
 //   printf("%s\n", data);
 //   return 0;
#if 1
    int ret;

	if((0==Len) || (NULL==data)||(fd_DEBUG<0))
		return 0;
	/* write uart */
	ret = write(fd_DEBUG, data, Len);
	if (ret != Len)
	{
		    printf("ERROR Debug write ret=%d\n", ret);
			close(fd_DEBUG);
			DEBUG_Uart_Init();
			return 0;
	}
	else
		return Len;
	#endif
}




//-----文件PcDebug.c结束---------------------------------------------
