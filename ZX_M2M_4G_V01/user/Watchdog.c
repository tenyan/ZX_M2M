/*
 * Copyright(c)2019, 硬件研发部
 * All right reserved
 *
 * 文件名称: Watchdog.c
 * 版本号  : V1.0
 * 文件标识:
 * 文件描述: 此文件为喂狗文件
 *
 *-------------------------------------------------------
 * 修改记录
 *-------------------------------------------------------
 *
 * 2019-04-1, by lxf, 创建本文件
 *
 */
#include "config.h"

int32 fd_watchdog = -1;

//喂狗初始化 ,目前喂狗复位时间为1分钟不为狗,系统复位
int32 Watchdog_Init(void)
{
	
	fd_watchdog = open("/dev/watchdog", O_WRONLY);
    if(fd_watchdog == -1)
    {
        int err = errno;
        printf("\n!!! FAILED to open /dev/watchdog, errno: %d, %s\n", err, strerror(err));	
        syslog(LOG_WARNING, "FAILED to open /dev/watchdog, errno: %d, %s", err, strerror(err));
    }
	return fd_watchdog;
}
//喂狗
void Watchdog_Feed(void)
{
    if(fd_watchdog >= 0) 
    {	
        
        static uint8 food = 0;	
    	//ssize_t eaten = write(fd_watchdog, &food, 1);
    	int32 eaten = write(fd_watchdog, &food, 1);
    	if(eaten != 1)
    	{
    	    puts("\n!!! FAILED feeding watchdog");
    		syslog(LOG_WARNING, "FAILED feeding watchdog");
    	}
    }
}

//关闭喂狗功能
void Watchdog_Close(void)
{
    close(fd_watchdog);
}


#if 0
#include <stdio.h>
#include <string.h>
#include "tcp_client.h"
typedef unsigned short U16;
const char srv_ip[] = "8.8.8.8";
#define R_OK    0
#define R_ERROR -1
/*typedef struct _DNS_HDR
{  
    U16 id;
    U16 tag;
    U16 numq;
    U16 numa;
    U16 numa1;
    U16 numa2;
}DNS_HDR;*/
typedef struct
{
unsigned short id;       // identification number
unsigned char rd :1;     // recursion desired
unsigned char tc :1;     // truncated message
unsigned char aa :1;     // authoritive answer
unsigned char opcode :4; // purpose of message
unsigned char qr :1;     // query/response flag
unsigned char rcode :4;  // response code
unsigned char cd :1;     // checking disabled
unsigned char ad :1;     // authenticated data
unsigned char z :1;      // its z! reserved
unsigned char ra :1;     // recursion available
unsigned short q_count;  // number of question entries
unsigned short ans_count; // number of answer entries
unsigned short auth_count; // number of authority entries
unsigned short add_count; // number of resource entries
}DNS_HDR;
/*typedef struct _DNS_QER
{
    U16 type;
    U16 classes;
}DNS_QER;*/
typedef struct
{
unsigned short type;
unsigned short classes;
}DNS_QES;
int main(int argc, char **argv)
{
    unsigned char buff[1024];
    unsigned char *buf = buff + 2;
    unsigned char *p;
    int len, i;
    DNS_HDR  *dnshdr = (DNS_HDR *)buf;
    DNS_QES  *dnsqes = NULL;
   
    if (R_ERROR == tcp_client_init(argv[2], 53))
    {
        printf("Conn Error!\n");
        return -1;
    }
    else
    {
        printf("Conn OK!\n");
    }
   
    memset(buff, 0, 1024);
    dnshdr->id = htons(0x2000);//(U16)1;
    dnshdr->qr = 0;
    dnshdr->opcode = 0;
    dnshdr->aa = 0;
    dnshdr->tc = 0;
    dnshdr->rd = 1;
    dnshdr->ra = 1;
    dnshdr->z  = 0;
    dnshdr->ad = 0;
    dnshdr->cd = 0;
    dnshdr->rcode = 0;
    dnshdr->q_count = htons(1);
    dnshdr->ans_count = 0;
    dnshdr->auth_count = 0;
    dnshdr->add_count = 0;
    strcpy(buf + sizeof(DNS_HDR) + 1, argv[1]);
    p = buf + sizeof(DNS_HDR) + 1; i = 0;
    while (p < (buf + sizeof(DNS_HDR) + 1 + strlen(argv[1])))
    {
        if ( *p == '.')
        {
            *(p - i - 1) = i;
            i = 0;
        }
        else
        {
            i++;
        }
        p++;
    }
    *(p - i - 1) = i;
                                   
    dnsqes = (DNS_QES *)(buf + sizeof(DNS_HDR) + 2 + strlen(argv[1]));
    dnsqes->classes = htons(1);
    dnsqes->type = htons(1);
    buff[0] = 0; buff[1] = sizeof(DNS_HDR) + sizeof(DNS_QES) + strlen(argv[1]) + 2;
    if (R_ERROR == tcp_client_send(buff, sizeof(DNS_HDR) + sizeof(DNS_QES) + strlen(argv[1]) + 4))
    {
        printf("Send Error!\n");
        return -1;
    }
    else
    {
        printf("Send OK!\n");
    }
   
    len = tcp_client_recv(buff, 1024);
    if (len < 0)
    {
        printf("Recv Error!\n");
        return -1;
    }
    else
    {
        printf("Recv OK!\n");
    }
   
    if (dnshdr->rcode !=0 || dnshdr->ans_count == 0)
    {
        printf("Ack Error\n");
        return -1;
    }
    p = buff + 2 + sizeof(DNS_HDR) + sizeof(DNS_QES) + strlen(argv[1]) + 2;
    printf("Ans Count = %d\n", ntohs(dnshdr->ans_count));
    for (i = 0; i < ntohs(dnshdr->ans_count); i++)
    {
        p = p + 12;
        printf("%s ==> %u.%u.%u.%u\n", argv[1], (unsigned char)*p, (unsigned char)*(p + 1), (unsigned char)*(p + 2), (unsigned char)*(p + 3));
        p = p + 4;
    }
    tcp_client_close();
    return 0;
}


#endif



