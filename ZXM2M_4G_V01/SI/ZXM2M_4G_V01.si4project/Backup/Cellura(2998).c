/*******************************************************************************
 * Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
 * @FileName: CelluraCore.c
 * @Engineer: TenYan
 * @version:  V1.0
 * @Date:     2020-10-2
 * @brief:
 *******************************************************************************/
//-----头文件调用------------------------------------------------------------
//#include "main.h"
#include "config.h"

/******************************************************************************
 *   Macros
 ******************************************************************************/
#define MODEM_DEV_TTY              "/dev/smd8"
#define MODEM_UART_RX_BUFFER_SIZE  1500

#define CELLURA_Receive    Modem_UartReceiveData

#define CELLULAR_MAX_RXBUF_SIZE  MODEM_UART_RX_BUFFER_SIZE
#define cellular_rx_buffer       usart3_rx_buffer
#define cellular_rx_size         usart3_rx_size

#define AT_transaction(p_buffer,size)  do { Cellura_TransmitData(p_buffer,size); } while (0) // 通过串口DMA发送AT指令到模块
#define AT_SET_CMD_ID(cmd)   (current_atcmd_id = cmd)
#define AT_GET_CMD_ID()      (current_atcmd_id)
#define ATCMD_IS_CUR(cmd)    (current_atcmd_id == (cmd))

#define AT_DELAY(ms)         do { usleep(1000*ms); } while(0)
#define MODEM_DELAY_MS(ms)   do { usleep(1000*ms); } while(0)
#define CELLURA_DELAY(ms)    do { usleep(1000*ms); } while(0)

#define CELLURA_CHARISNUM(x)        ((x) >= '0' && (x) <= '9')
#define CELLURA_CHARTONUM(x)        ((x) - '0')
#define CELLURA_CHARISHEXNUM(x)     (((x) >= '0' && (x) <= '9') || ((x) >= 'a' && (x) <= 'f') || ((x) >= 'A' && (x) <= 'F'))
#define CELLURA_CHARHEXTONUM(x)     (((x) >= '0' && (x) <= '9') ? ((x) - '0') : (((x) >= 'a' && (x) <= 'f') ? ((x) - 'a' + 10) : (((x) >= 'A' && (x) <= 'F') ? ((x) - 'A' + 10) : 0)))
#define CELLURA_ISVALIDASCII(x)     (((x) >= 32 && (x) <= 126) || (x) == '\r' || (x) == '\n')

/******************************************************************************
 * Data Types and Globals
 ******************************************************************************/
 int fd_modem = -1;
uint8_t modem_uart_rx_buffer[MODEM_UART_RX_BUFFER_SIZE];
modem_info_t modem_info;
at_cmd_t current_atcmd_id;
atStatus_t atcmd_rsp_status;
skt_context_t skt_context[NUMBER_OF_SOCKET_LINKS];

/******************************************************************************
* AT指令定义
******************************************************************************/
const char AT_CFUN0_SET[]="AT+CFUN=0\r\n"; // 模块进入飞行模式
#define SIZE_OF_AT_CFUN0_SET  (sizeof(AT_CFUN0_SET)/sizeof(AT_CFUN0_SET[0]) - 1)

const char AT_CFUN1_SET[]="AT+CFUN=1\r\n"; // 模块为全功能(online mode)
#define SIZE_OF_AT_CFUN1_SET  (sizeof(AT_CFUN1_SET)/sizeof(AT_CFUN1_SET[0]) - 1)

const char AT_CRESET[]="AT\r\n"; // 模块软件复位
#define SIZE_OF_AT_CRESET  (sizeof(AT_CRESET)/sizeof(AT_CRESET[0]) - 1)

const char AT_AT[]="AT\r\n"; // AT测试
#define SIZE_OF_AT_AT  (sizeof(AT_AT)/sizeof(AT_AT[0]) - 1)

const char AT_ATE0[]="ATE0\r\n"; // 关闭回显
#define SIZE_OF_AT_ATE0  (sizeof(AT_ATE0)/sizeof(AT_ATE0[0]) - 1)

const char AT_CGPS_SET[]="AT+CGPS=1\r\n"; // 开启GPS会话
#define SIZE_OF_AT_CGPS_SET  (sizeof(AT_CGPS_SET)/sizeof(AT_CGPS_SET[0]) - 1)

const char AT_CGPSXE_SET[]="AT+CGPSXE=1\r\n"; // 使能GPS XTRA
#define SIZE_OF_AT_CGPSXE_SET  (sizeof(AT_CGPSXE_SET)/sizeof(AT_CGPSXE_SET[0]) - 1)

const char AT_CGPSINFOCFG_SET[]="AT+CGPSINFOCFG=5,3\r\n"; // 配置NMEA-0183输出周期5S,输出GPGGA和GPRMC信息
#define SIZE_OF_AT_CGPSINFOCFG_SET  (sizeof(AT_CGPSINFOCFG_SET)/sizeof(AT_CGPSINFOCFG_SET[0]) - 1)

const char AT_CGMR[]="AT+GMR\r\n"; // 查询模块软件版本
#define SIZE_OF_AT_CGMR  (sizeof(AT_CGMR)/sizeof(AT_CGMR[0]) - 1)

const char AT_CMEE[]="AT+CMEE=1\r\n"; // 用数字报告移动设备的错误
#define SIZE_OF_AT_CMEE  (sizeof(AT_CMEE)/sizeof(AT_CMEE[0]) - 1)

const char AT_CPIN[]="AT+CPIN?\r\n"; // 查询SIM卡是否READY
#define SIZE_OF_AT_CPIN  (sizeof(AT_CPIN)/sizeof(AT_CPIN[0]) - 1)

const char AT_CIMI[]="AT+CIMI\r\n"; // 获取SIM卡的IMSI
#define SIZE_OF_AT_CIMI  (sizeof(AT_CIMI)/sizeof(AT_CIMI[0]) - 1)

const char AT_CICCID[]="AT+CICCID\r\n"; // 获取SIM卡的ICCID
#define SIZE_OF_AT_CICCID  (sizeof(AT_CICCID)/sizeof(AT_CICCID[0]) - 1)

const char AT_CMGF[]="AT+CMGF=1\r\n";  // Message Format
#define SIZE_OF_AT_CMGF (sizeof(AT_CMGF)/sizeof(AT_CMGF[0]) - 1)

const char AT_CPMS[]="AT+CPMS=\"ME\",\"ME\",\"ME\"\r\n";  // 短信存储设置(存储在移动设备上)
#define SIZE_OF_AT_CPMS (sizeof(AT_CPMS)/sizeof(AT_CPMS[0]) - 1)

const char AT_CNMI[]="AT+CNMI=2,2\r\n";  // SMS Event Reporting Configuration
#define SIZE_OF_AT_CNMI (sizeof(AT_CNMI)/sizeof(AT_CNMI[0]) - 1)

const char AT_CMGD[]="AT+CMGD=1,4\r\n";  // 删除所有短信(包括未读)
#define SIZE_OF_AT_CMGD (sizeof(AT_CMGD)/sizeof(AT_CMGD[0]) - 1)

const char AT_CMGL[]="AT+CMGL=\"ALL\"\r\n";  // 列出所有短信
#define SIZE_OF_AT_CMGL (sizeof(AT_CMGL)/sizeof(AT_CMGL[0]) - 1)

const char AT_CNMP_SET[]="AT+CNMP=2\r\n"; // 设置网络搜索模式为Automatic(LTE/WCDMA/GSM)
#define SIZE_OF_AT_CNMP_SET (sizeof(AT_CNMP_SET)/sizeof(AT_CNMP_SET[0]) - 1)

const char AT_CSQ[]="AT+CSQ\r\n"; // 查询无线信号质量
#define SIZE_OF_AT_CSQ  (sizeof(AT_CSQ)/sizeof(AT_CSQ[0]) - 1)

const char AT_CREG0_SET[]="AT+CREG=0\r\n"; // 关闭+CREG: URC上报
#define SIZE_OF_AT_CREG0_SET  (sizeof(AT_CREG0_SET)/sizeof(AT_CREG0_SET[0]) - 1)

const char AT_CREG2_SET[]="AT+CREG=2\r\n"; // 使能+CREG: URC上报
#define SIZE_OF_AT_CREG2_SET  (sizeof(AT_CREG2_SET)/sizeof(AT_CREG2_SET[0]) - 1)

const char AT_CREG_GET[]="AT+CREG?\r\n"; // CS业务:网络注册状态
#define SIZE_OF_AT_CREG_GET  (sizeof(AT_CREG_GET)/sizeof(AT_CREG_GET[0]) - 1)

const char AT_CGREG_GET[]="AT+CGREG?\r\n"; // PS业务:GPRS网络注册状态
#define SIZE_OF_AT_CGREG_GET  (sizeof(AT_CGREG_GET)/sizeof(AT_CGREG_GET[0]) - 1)

const char AT_CEREG_GET[]="AT+CEREG?\r\n"; // PS业务:LTE网络注册状态
#define SIZE_OF_AT_CEREG_GET  (sizeof(AT_CGREG_GET)/sizeof(AT_CEREG_GET[0]) - 1)

const char AT_CGATT_SET[]="AT+CGATT=1\r\n"; // 使能PS附着
#define SIZE_OF_AT_CGATT_SET  (sizeof(AT_CGATT_SET)/sizeof(AT_CGATT_SET[0]) - 1)

const char AT_CGATT_GET[]="AT+CGATT?\r\n"; // 查询PS附着
#define SIZE_OF_AT_CGATT_GET  (sizeof(AT_CGATT_GET)/sizeof(AT_CGATT_GET[0]) - 1)

const char AT_CGDCONT[]="AT+CGDCONT=1,\"IP\",\"cmmtm\"\r\n"; // 定义PDP上下文
#define SIZE_OF_AT_CGDCONT  (sizeof(AT_CGDCONT)/sizeof(AT_CGDCONT[0]) - 1)

const char AT_CGACT[]="AT+CGACT=1\r\n"; // 激活PDP上下文
#define SIZE_OF_AT_CGACT  (sizeof(AT_CGACT)/sizeof(AT_CGACT[0]) - 1)

const char AT_CGPADDR[]="AT+CGPADDR=1\r\n"; // 查询PDP地址
#define SIZE_OF_AT_CGPADDR  (sizeof(AT_CGPADDR)/sizeof(AT_CGPADDR[0]) - 1)

const char AT_IPADDR[]="AT+IPADDR\r\n"; // 查询Socket PDP地址
#define SIZE_OF_AT_IPADDR  (sizeof(AT_IPADDR)/sizeof(AT_IPADDR[0]) - 1)

const char AT_CSCLK[]="AT+CSCLK=1\r\n"; // 控制模块休眠
#define SIZE_OF_AT_CSCLK  (sizeof(AT_CSCLK)/sizeof(AT_CSCLK[0]) - 1)


/*************************************************************************
 *
*************************************************************************/
void Modem_UartInitialize(uint32_t baudrate)
{
  struct termios options;

  // 打开失败，如何处理?
  fd_modem = open(MODEM_DEV_TTY, O_RDWR|O_NOCTTY);
  if (fd_modem < 0)
  {
    printf("ERROR open %s ret=%d\n\r", MODEM_DEV_TTY, fd_modem);
    return ;
  }
  printf("fd_modem=%d\n", fd_modem);

  tcgetattr(fd_modem, &options);
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;
  options.c_cc[VTIME] = 10; // read timeout 10*100ms
  options.c_cc[VMIN]  = 0;
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  options.c_oflag &= ~OPOST;
  options.c_iflag &= ~(ICRNL | IXON);
  cfsetispeed(&options, baudrate);
  cfsetospeed(&options, baudrate);
  options.c_cflag |= (CLOCAL | CREAD);
  tcflush(fd_modem, TCIFLUSH);
  tcsetattr(fd_modem, TCSANOW, &options);
}

//===================================================================================
int Modem_UartReceiveData(uint8_t **data, uint16_t* size)
{
  int ret = 0;

  if (fd_modem<0)
  {
    return FALSE;
  }

  ret = read(fd_modem, modem_uart_rx_buffer, (MODEM_UART_RX_BUFFER_SIZE-1)); // 阻塞等待数据
  if (ret > 0)
  {
    *data = modem_uart_rx_buffer;
    *size = (uint16_t)ret;
    
    return TRUE;
  }

  return FALSE;
}


//===================================================================================
int Modem_UartTransmitData(uint8_t *data, uint16_t size)
{
  int retVal;

  if ((0==size) || (NULL==data) || (fd_modem<0))
  {
    return 0;
  }

  retVal = write(fd_modem, data, size);  // 写串口数据
  if (retVal != size)
  {
    printf("ERROR Modem write ret=%d\n", retVal);
    close(fd_modem);
    Modem_UartInitialize(B115200);
    return 0;
  }
  
  PcDebug_SendData(data, size, DBG_MSG_TYPE_AT); // 调试信息

  return size;
}


/******************************************************************************
 *
*******************************************************************************/
void Cellura_TransmitData(uint8_t *pdata, uint16_t size)
{
  Modem_UartTransmitData(pdata, size);
  PcDebug_SendData(pdata, size, DBG_MSG_TYPE_AT);
}

/******************************************************************************
* 
******************************************************************************/
modem_state_t Modem_GetState(void)
{
  return modem_info.modem_state;
}

void Modem_SetState(modem_state_t state)
{
  modem_info.modem_state = state;
}

/******************************************************************************
* 创建AT命令并发送(每10ms检查AT是否响应)
******************************************************************************/
atStatus_t AT_SendCmd(at_cmd_t atcmdId, uint8_t retries, uint16_t timeout)
{
  uint8_t it;
  uint16_t chk_cnt = 0;
  uint16_t chk_period;
  const char* at_buf;
  uint8_t at_size;

  switch (atcmdId) // 通过atcmdId获取AT指令
  {
  case AT_CMD_RESET:
    break;

  case AT_CMD_AT: // AT测试
    at_buf = AT_AT;
    at_size = SIZE_OF_AT_AT;
    break;

  case AT_CMD_ATE0: // 关闭回显
    at_buf = AT_ATE0;
    at_size = SIZE_OF_AT_ATE0;
    break;

  case AT_CMD_CGMR: // 查询模块软件版本
    at_buf = AT_CGMR;
    at_size = SIZE_OF_AT_CGMR;
    break;

  case AT_CMD_CMEE_SET: // 用数字表示错误消息
    at_buf = AT_CMEE;
    at_size = SIZE_OF_AT_CMEE;
    break;

  case AT_CMD_CPIN_GET: // 查询SIM卡是否READY
    at_buf = AT_CPIN;
    at_size = SIZE_OF_AT_CPIN;
    break;

  case AT_CMD_CIMI_GET: // 获取SIM卡的IMSI
    at_buf = AT_CIMI;
    at_size = SIZE_OF_AT_CIMI;
    break;

  case AT_CMD_CICCID_GET: // 获取SIM卡的ICCID
    at_buf = AT_CICCID;
    at_size = SIZE_OF_AT_CICCID;
    break;

  case AT_CMD_CMGF_SET: // 设置短信格式
    at_buf = AT_CMGF;
    at_size = SIZE_OF_AT_CMGF;
    break;

  case AT_CMD_CPMS_SET: // 设置短信存储位置
    at_buf = AT_CPMS;
    at_size = SIZE_OF_AT_CPMS;
    break;

  case AT_CMD_CNMI_SET: // SMS事件上报配置:不上报
    at_buf = AT_CNMI;
    at_size = SIZE_OF_AT_CNMI;
    break;

  case AT_CMD_QCFG3_SET: // 设置网络搜索模式为Automatic(LTE/WCDMA/GSM)
    at_buf = AT_QCFG3;
    at_size = SIZE_OF_AT_QCFG3;
    break;

  case AT_CMD_CSQ_GET: // 查询无线信号质量
    at_buf = AT_CSQ;
    at_size = SIZE_OF_AT_CSQ;
    break;
  
  case AT_CMD_CREG0_SET: // 使能+CREG: URC上报
    at_buf = AT_CREG0_SET;
    at_size = SIZE_OF_AT_CREG0_SET;
    break;
  
  case AT_CMD_CREG2_SET: // 使能+CREG: URC上报
    at_buf = AT_CREG2_SET;
    at_size = SIZE_OF_AT_CREG2_SET;
    break;

  case AT_CMD_CREG_GET: // CS业务:网络注册状态
    at_buf = AT_CREG_GET;
    at_size = SIZE_OF_AT_CREG_GET;
    break;

  case AT_CMD_CGREG_GET: // PS业务:GPRS网络注册状态
    at_buf = AT_CGREG_GET;
    at_size = SIZE_OF_AT_CGREG_GET;
    break;

  case AT_CMD_CEREG_GET: // PS业务:LTE网络注册状态
    at_buf = AT_CEREG_GET;
    at_size = SIZE_OF_AT_CEREG_GET;
    break;

  case AT_CMD_CGATT_SET: // 使能PS附着
    at_buf = AT_CGATT_SET;
    at_size = SIZE_OF_AT_CGATT_SET;
    break;

  case AT_CMD_CGATT_GET: // 查询PS附着
    at_buf = AT_CGATT_GET;
    at_size = SIZE_OF_AT_CGATT_GET;
    break;

  case AT_CMD_CGDCONT_SET: // 定义PDP上下文
    at_buf = AT_CGDCONT;
    at_size = SIZE_OF_AT_CGDCONT;
    break;

  case AT_CMD_CGACT_SET: // 激活PDP上下文
    at_buf = AT_CGACT;
    at_size = SIZE_OF_AT_CGACT;
    break;

  case AT_CMD_QIACT_SET: // 激活PDP场景
    at_buf = AT_QIACT;
    at_size = SIZE_OF_AT_QIACT;
    break;

  case AT_CMD_QIDEACT_SET: // 去激活PDP场景
    at_buf = AT_QIDEACT;
    at_size = SIZE_OF_AT_QIDEACT;
    break;

  case AT_CMD_CGPADDR_GET: // 查询PDP地址
    at_buf = AT_CGPADDR;
    at_size = SIZE_OF_AT_CGPADDR;
    break;
  
  case AT_CMD_QSCLK: // 设置模块进入休眠状态
    at_buf = AT_QSCLK;
    at_size = SIZE_OF_AT_QSCLK;
    break;

  default:
    break;
  }

  chk_period = timeout/50 + 1;
  atcmd_rsp_status = ATCMD_RSP_NONE;
  for (it=0; it<retries; it++)
  {
    AT_SET_CMD_ID(atcmdId);
    AT_transaction((uint8_t*)at_buf,at_size); // 通过串口DMA发送AT指令到模块
    for (chk_cnt=0; chk_cnt<chk_period; chk_cnt++)
    {
      AT_DELAY(50);  // 每50m检查一次AT响应状态
      if (ATCMD_RSP_OK == atcmd_rsp_status)
      {
        AT_SET_CMD_ID(AT_CMD_IDLE);
        return ATCMD_RSP_OK;
      }
    }
  }

  AT_SET_CMD_ID(AT_CMD_IDLE);
  return ATCMD_RSP_NOK;
}

/******************************************************************************
 * 功能: CAT1初始化及上线操作
*******************************************************************************/
void Modem_Reset(void)
{
	ucResetGsmModuleFlag = 1;
	sleep(3);  // 等待3s,让其他进程去写入参数
	ucResetGsmModuleFlag = 0;
	system("sync"); // 将缓存写入FLASH中
	PC_SendDebugData((uint8 *)("LTE-CRESET\r\n"), 10, DEBUG_ANYDATA);
	WriteGsmUartData(gGsmAT_fd, (uint8 *)"AT+CRESET\r", 10);
}

/******************************************************************************
 * 功能: CAT1初始化及上线操作
*******************************************************************************/
uint8_t Modem_Init(void)
{
  atStatus_t at_rsp;

  modem_info.modem_init_fail_cnt++;

  /***********************************模块开机,进行初始化操作**************************************/
  // 测试模块串口AT是否初始化完成
  at_rsp = AT_SendCmd(AT_CMD_AT, 30, OS_TICKS_PER_SEC);
  if (at_rsp == ATCMD_RSP_NOK)
  {
    modem_info.at_fail_cnt++;
    return AT_FALSE; // 退出重试
  }
  else
  {
    modem_info.modem_err_flag = 0;
    modem_info.at_fail_cnt = 0;
  }

  // 关闭回显
  AT_SendCmd(AT_CMD_ATE0, 3, OS_TICKS_PER_SEC);
  
  // 查询模块软件版本
  AT_SendCmd(AT_CMD_CGMR, 1, OS_TICKS_PER_SEC);
  
  // 设置错误码带数字标识
  AT_SendCmd(AT_CMD_CMEE_SET, 3, OS_TICKS_PER_SEC);

  // 获取SIM卡状态
  at_rsp = AT_SendCmd(AT_CMD_CPIN_GET, 20, OS_TICKS_PER_SEC);
  if (at_rsp == ATCMD_RSP_NOK)
  {
    modem_info.sim_fail_cnt++;
    return AT_FALSE; // 退出重试
  }
  else
  {
    modem_info.sim_err_flag = 0;
    modem_info.sim_fail_cnt = 0;
  }

  // 获取手机卡号
  at_rsp = AT_SendCmd(AT_CMD_CIMI_GET, 3, OS_TICKS_PER_SEC);
  if (at_rsp == ATCMD_RSP_NOK)
  {
    return AT_FALSE; // 退出重试
  }

  // 获取ICCID
  at_rsp = AT_SendCmd(AT_CMD_QCCID_GET, 3, OS_TICKS_PER_SEC);
  if (at_rsp == ATCMD_RSP_NOK)
  {
    return AT_FALSE; // 退出重试
  }

  // 设置模块短信消息模式为TXT
  at_rsp = AT_SendCmd(AT_CMD_CMGF_SET, 5, OS_TICKS_PER_SEC);
  if (at_rsp == ATCMD_RSP_NOK)
  {
    return AT_FALSE; // 退出重试
  }

  // 设置短信消息存储
  at_rsp = AT_SendCmd(AT_CMD_CPMS_SET, 3, OS_TICKS_PER_SEC);
  if (at_rsp == ATCMD_RSP_NOK)
  {
    return AT_FALSE; // 退出重试
  }

  // 设置短信消息上报方式
  at_rsp = AT_SendCmd(AT_CMD_CNMI_SET, 5, OS_TICKS_PER_SEC);
  if (at_rsp == ATCMD_RSP_NOK)
  {
    return AT_FALSE; // 退出重试
  }

  // 设置模块网络制式
  AT_SendCmd(AT_CMD_QCFG3_SET, 3, OS_TICKS_PER_SEC); // 自动模式
  if (at_rsp == ATCMD_RSP_NOK)
  {
    return AT_FALSE; // 退出重试
  }

  // 查询蜂窝无线信号强度
  at_rsp = AT_SendCmd(AT_CMD_CSQ_GET, 30, OS_TICKS_PER_SEC);
  if (at_rsp == ATCMD_RSP_NOK)
  {
    return AT_FALSE; // 退出重试
  }

  // 使能网络状态和小区码自动上报
  at_rsp = AT_SendCmd(AT_CMD_CREG2_SET, 3, OS_TICKS_PER_SEC);
  if (at_rsp == ATCMD_RSP_NOK)
  {
    return AT_FALSE; // 退出重试
  }

  AT_DELAY(OS_TICKS_PER_SEC*2);

  // 查询CS网络注册状态
  at_rsp = AT_SendCmd(AT_CMD_CREG_GET, 120, OS_TICKS_PER_SEC); // 给模块两分钟网络注册时间
  if (at_rsp == ATCMD_RSP_NOK)
  {
    modem_info.cs_network_regist_status = CS_CS_DETACHED;
    // return; // 不要返回，继续往下走
  }
  else
  {
    modem_info.cs_network_regist_status = CS_CS_ATTACHED;
    modem_info.sms_ready_flag = 1;
  }

  // 查询PS网络注册状态
  at_rsp = AT_SendCmd(AT_CMD_CEREG_GET, 30, OS_TICKS_PER_SEC); // 给模块30秒钟网络注册时间
  if (at_rsp == ATCMD_RSP_NOK)
  {
    modem_info.ps_network_regist_status = CS_PS_DETACHED;
    // return; // 不要返回，继续往下走
  }
  else
  {
    modem_info.ps_network_regist_status = CS_PS_ATTACHED;
  }

  AT_DELAY(OS_TICKS_PER_SEC);

  // 使能模块网络附着
  AT_SendCmd(AT_CMD_CGATT_SET, 3, OS_TICKS_PER_SEC);

  // 查询网络附着状态
  at_rsp = AT_SendCmd(AT_CMD_CGATT_GET, 240, OS_TICKS_PER_SEC); // 给模块四分钟附着网络时间
  if (at_rsp == ATCMD_RSP_NOK)
  {
    modem_info.network_attach_status = CS_PS_DETACHED;
    return AT_FALSE; // 退出重试
  }
  else
  {
    modem_info.network_attach_status = CS_PS_ATTACHED;
  }

  // 定义PDP(Packet Data Protocol传输数据)上下文
  at_rsp = AT_SendCmd(AT_CMD_CGDCONT_SET, 3, OS_TICKS_PER_SEC);
  if (at_rsp == ATCMD_RSP_NOK)
  {
    return AT_FALSE; // 退出重试
  }

  // 设置模块网络制式状态自动上报
  // AT_SendCmd(AT_CMD_CNSMOD_SET, 3, (OS_TICKS_PER_SEC/2));

  // 激活PDP,只发一次
  at_rsp = AT_SendCmd(AT_CMD_QIACT_SET, 1, (25*OS_TICKS_PER_SEC));
  if (at_rsp == ATCMD_RSP_NOK)
  {
    return AT_FALSE; // 退出重试
  }

  // 查询Socket PDP的IP地址
  at_rsp = AT_SendCmd(AT_CMD_CGPADDR_GET, 3, OS_TICKS_PER_SEC);
  if (at_rsp == ATCMD_RSP_NOK)
  {
    return AT_FALSE; // 退出重试
  }

  modem_info.network_ready_flag = 1;
  modem_info.modem_init_fail_cnt = 0;
  PcDebug_SendString("Modem Data Ready!\n");

  return AT_TRUE;
}

#define CST_QNS_STATE_SIMCARD  0x01  // 查询SIM卡状态
#define CST_QNS_STATE_RFSQ     0x02  // 查询无线信号质量
#define CST_QNS_STATE_NETWORK  0x03  // 查询网络附着状态
/******************************************************************************
 *功能: 查询CAT1模块网络状态(SIM卡、信号质量、网络附着状态)
*******************************************************************************/
void Modem_QueryNetworkState(void)
{
  static uint8_t qns_state = CST_QNS_STATE_SIMCARD;
  atStatus_t at_rsp;

  switch (qns_state)
  {
  case CST_QNS_STATE_SIMCARD:  // 获取SIM卡状态
    at_rsp = AT_SendCmd(AT_CMD_CPIN_GET, 1, (OS_TICKS_PER_SEC/20));  // 等待50ms
    if (at_rsp == ATCMD_RSP_OK)
    {
      modem_info.sim_err_flag = 0;
      modem_info.sim_fail_cnt = 0;
    }
    else
    {
      modem_info.sim_fail_cnt++;
    }
    qns_state = CST_QNS_STATE_RFSQ;
    break;

  case CST_QNS_STATE_RFSQ: // 查询蜂窝无线信号强度
    AT_SendCmd(AT_CMD_CSQ_GET, 1, (OS_TICKS_PER_SEC/20));
    qns_state = CST_QNS_STATE_NETWORK;
    break;

  case CST_QNS_STATE_NETWORK: // 查询网络附着状态
    at_rsp = AT_SendCmd(AT_CMD_CGATT_GET, 1, (OS_TICKS_PER_SEC/20)); 
    if (at_rsp == ATCMD_RSP_OK)
    {
      modem_info.cgatt_fail_cnt = 0;
      modem_info.network_attach_status = CS_PS_ATTACHED;
    }
    else
    {
      modem_info.cgatt_fail_cnt++;
    }
    qns_state = CST_QNS_STATE_SIMCARD;
    break;

#if 0
  case 3:
    if((0==f_stuSmsRcv.flag) && (0==f_stuSmsSend.flag))
    {
      GsmAtCmdState.ucCMGL = 0;
      SetAtSend(ATSEND_READSMS_CMGL);
      WriteGsmUartData((uint8 *)"AT+CMGL=\"ALL\"\r", 14); // 列出SIM卡中的短消息(全部的)
      OSTimeDly(OS_TICKS_PER_SEC/50);//等待20ms
      SetAtSend(ATSEND_IDLE);
    }
    state = 0;
    break;
#endif

  default:
      qns_state = CST_QNS_STATE_SIMCARD;
      break;
    }
}

/******************************************************************************
* Modem 故障处理
******************************************************************************/
void Modem_ProcessError(void)
{
  // at指令未响应,模块故障
  if (modem_info.at_fail_cnt > 5)
  {
    modem_info.at_fail_cnt = 0;
    modem_info.modem_err_flag = 1;
    PcDebug_SendString("ModemRst:AT!\n");
    Modem_SetState(MODEM_STATE_REPOWER_OFF); // 重启模块
  }

  // SIM卡故障
  if (modem_info.sim_fail_cnt > 5)
  {
    modem_info.sim_fail_cnt = 0;
    modem_info.sim_err_flag = 1;
    PcDebug_SendString("ModemRst:SimCard!\n");
    Modem_SetState(MODEM_STATE_REPOWER_OFF); // 重启模块
  }

  // 网络未附着
  if (modem_info.cgatt_fail_cnt > 5)
  {
    modem_info.cgatt_fail_cnt = 0;
    modem_info.network_attach_status = CS_PS_DETACHED;
    PcDebug_SendString("ModemRst:CGATT!\n");
    Modem_SetState(MODEM_STATE_REPOWER_OFF); // 重启模块
  }
  
  if(modem_info.pdpdeact_cnt > 0)
  {
    modem_info.pdpdeact_cnt = 0;
    modem_info.pdp_actived_flag = 0;
    PcDebug_SendString("ModemRst:PDP!\n");
    Modem_SetState(MODEM_STATE_REPOWER_OFF); // 重启模块
  }
}

/******************************************************************************
 * 模块状态机(10ms调用一次)
*******************************************************************************/
void Modem_StateManageService(void)               /// 阻塞
{
  uint8_t modem_state;
  uint8_t retval;
  static uint16_t qns_timer_10ms = 500; // 每5秒查询一次

  modem_state = Modem_GetState();
  switch (modem_state)
  {
  case MODEM_STATE_OFF:  // 关机后,无操作
    break;

  case MODEM_STATE_INIT:  // 模块初始化
    retval = Modem_Init();
    if (retval == AT_TRUE)
    {
      Modem_SetState(MODEM_STATE_DATA_READY); // 初始化成功,转数据模式
    }
    else
    {
      Modem_SetState(MODEM_STATE_REPOWER_OFF); // 初始化失败,转模块关机
    }
    break;

  case MODEM_STATE_REPOWER_OFF:
    Modem_PowerOff();
    if (modem_info.modem_init_fail_cnt > 5)  // 反复入网不成功休息10分钟
    {
      modem_info.modem_init_fail_cnt = 0;
      Modem_SetState(MODEM_STATE_SILENCE);
    }
    else // 重新开机
    {
      Modem_SetState(MODEM_STATE_POWER_ON); // 重新开机
    }
    break;

  case MODEM_STATE_DATA_READY:  // 模块可上网
    if (qns_timer_10ms)
      qns_timer_10ms--;
    else
    {
      qns_timer_10ms = 500; // 每5秒查询一次
      Modem_QueryNetworkState(); // 查询模块状态
    }
    Modem_ProcessError();
    break;

  case MODEM_STATE_SILENCE:  // 模块静默10分钟
    PcDebug_SendString("Modem Rest 10min\n");
    MODEM_DELAY_MS((600*OS_TICKS_PER_SEC));
    Modem_SetState(MODEM_STATE_POWER_ON); // 重新开机
    break;

  default:
    Modem_SetState(MODEM_STATE_REPOWER_OFF);
    break;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 短消息管理
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/******************************************************************************
* 功能: 发送短消息处理函数
*******************************************************************************/
#if 0
void Modem_SendSMS(void)
{
  uint8_t at_buff[300]={0};
  uint8_t i;
  uint16_t ucPackLen = 0;
  uint8_t* pucData = NULL;

  if (f_stuSmsSend.flag != 1)
    return;

  if (!strncmp((char*)f_stuSmsSend.aucDesNumber, "10", 2)) //10打头号码不加86
  {
    ucPackLen=CalcSmsPduLen(f_stuSmsSend.usDataLen, f_stuSmsSend.ucNumberLen);
  }
  else
  {
    ucPackLen=CalcSmsPduLen(f_stuSmsSend.usDataLen, 2+f_stuSmsSend.ucNumberLen);
  }

  sprintf((char*)at_buff, "AT+CMGS=%d\r", ucPackLen);
  atcmd_rsp_status = ATCMD_RSP_NONE;
  AT_SET_CMD_ID(AT_CMD_CMGS);
  AT_transaction(at_buff, strlen(at_buff));
  for (i=0; i<10; i++)
  {
    AT_DELAY(OS_TICKS_PER_SEC/100); // 每10ms检查一次
    if (ATCMD_RSP_OK == atcmd_rsp_status)
    {
      break;
    }
  }
  AT_SET_CMD_ID(AT_CMD_IDLE);
  if (ATCMD_RSP_OK == atcmd_rsp_status)
  {
    memcpy(at_buff, f_stuSmsSend.aucSmsSendBuff, f_stuSmsSend.usDataLen);
    pucData = at_buff;
    PackSmsPDU(pucData, &ucPackLen);//将包封转成PDU格式,数据长度放前两字节
    atcmd_rsp_status = ATCMD_RSP_NONE;
    AT_SET_CMD_ID(AT_CMD_CMGS_DATA);
    AT_transaction(at_buff, ucPackLen);

    at_buff[0] = 0x1a;
    AT_transaction(at_buff, 1);
  }
  AT_SET_CMD_ID(AT_CMD_IDLE);
}
#endif


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// socket状态机
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//=============================================================================================
void Net_SocketInvalid(skt_context_t* pThis)
{
  if (pThis->state_transition==SOCKET_TRUE)
  {
    pThis->state_transition = SOCKET_FALSE;
    pThis->started_delay = SOCKET_FALSE;
#if SOCKET_DEBUG
    PcDebug_Printf("InvSkt:Id=%d\n",pThis->conId);
#endif
  }

  if (pThis->started_delay==SOCKET_FALSE)
  {
    pThis->started_delay = SOCKET_TRUE;

    pThis->link_state = SOCKET_LINK_STATE_CLOSED;
    pThis->timer_10ms = 100; // 1秒
  }
  else
  {
    if (pThis->timer_10ms == 0)
    {
      if ((Modem_GetState()==MODEM_STATE_DATA_READY) && (pThis->enable_flag==SOCKET_TRUE)) // 4G模块网络已就绪
      {
        pThis->state = NET_SOCKET_STATE_CREATE;
        pThis->state_transition = SOCKET_TRUE;
      }
    }
  }
}

//=============================================================================================
void Net_SocketCreate(skt_context_t* pThis)
{
  if (pThis->state_transition==SOCKET_TRUE)
  {
    pThis->state_transition = SOCKET_FALSE;
    pThis->started_delay = SOCKET_FALSE;
#if SOCKET_DEBUG
    PcDebug_Printf("CrtSkt:Id=%d\n",pThis->conId);
#endif
  }

  if (pThis->started_delay==SOCKET_FALSE)
  {
    pThis->started_delay = SOCKET_TRUE;
    
    // 使用手动分配socket
    (*pThis->reinit_server_addr)(); // 重新获取服务器地址和端口
    
    pThis->timer_10ms = 100; // 1秒
  }
  else
  {
    if (pThis->timer_10ms == 0)
    {
      pThis->state = NET_SOCKET_STATE_CONNECT;
      pThis->state_transition = SOCKET_TRUE;
    }
  }
}

//=============================================================================================
void Net_SocketConnect(skt_context_t* pThis)
{
  int retval = 0;
  int8_t sbuff[50] = {NULL};

  if (pThis->state_transition==SOCKET_TRUE)
  {
    pThis->state_transition = SOCKET_FALSE;
    pThis->started_delay = SOCKET_FALSE;
#if SOCKET_DEBUG
    PcDebug_Printf("ConSkt:Id=%d\n",pThis->conId);
#endif
  }

  if (pThis->started_delay==SOCKET_FALSE)
  {
    pThis->started_delay = SOCKET_TRUE;
    pThis->link_state = SOCKET_LINK_STATE_CONNECTING;
    
    retval = Modem_SocketConnect(pThis);  // 阻塞,如果服务器无效,时间大概2分钟
    if (retval == 1) // 连接失败
    {
#if SOCKET_DEBUG
      PcDebug_Printf("ConSktErr:Id=%d\n",pThis->conId);
#endif
      pThis->error_cnt++;
      pThis->state = NET_SOCKET_STATE_CLOSE;
      pThis->state_transition = SOCKET_TRUE;
      return;
    }
    sprintf((char *)sbuff, "SktId=%d IP-%s:%d\n",pThis->conId,pThis->srv_ip,pThis->srv_port); // 显示链接id、服务器地址和端口
    PcDebug_Printf((const char *)sbuff);

    pThis->timer_10ms = 50; // 0.5秒
  }
  else
  {
    if (pThis->timer_10ms == 0)
    {
      pThis->state = NET_SOCKET_STATE_READY;
      pThis->state_transition = SOCKET_TRUE;
    }
  }
}

//=============================================================================================
void Net_SocketReady(skt_context_t* pThis)
{
  if (pThis->state_transition)
  {
    pThis->state_transition = SOCKET_FALSE;
    pThis->started_delay = SOCKET_FALSE;
#if SOCKET_DEBUG
    PcDebug_Printf("RdySkt:Id=%d\n",pThis->conId);
#endif
  }

  if (pThis->started_delay==SOCKET_FALSE)
  {
    pThis->started_delay = SOCKET_TRUE;
    pThis->error_cnt = 0x00; // 清除错误计数
    //pThis->error_flag = FALSE;
    pThis->link_state = SOCKET_LINK_STATE_READY;
  }
  else
  {
    // 4G模块判断
    if (Modem_GetState()!= MODEM_STATE_DATA_READY) // 4G模块处于初始化
    {
      pThis->link_state = SOCKET_LINK_STATE_CLOSED;
      pThis->state = NET_SOCKET_STATE_CLOSE;
      pThis->state_transition = SOCKET_TRUE;
      return;
    }

    if (pThis->link_state == SOCKET_LINK_STATE_CLOSED)
    {
      pThis->state = NET_SOCKET_STATE_CLOSE;
      pThis->state_transition = SOCKET_TRUE;
      return;
    }

    // 发送数据
    Modem_SocketSend(pThis);

    if (pThis->error_cnt > 3) // 发送3次都失败
    {
      pThis->state = NET_SOCKET_STATE_CLOSE; // 重连
      pThis->state_transition = SOCKET_TRUE;
    }
  }
}

//=============================================================================================
void Net_SocketClose(skt_context_t* pThis)
{
  if (pThis->state_transition==SOCKET_TRUE)
  {
    pThis->state_transition = SOCKET_FALSE;
    pThis->started_delay = SOCKET_FALSE;
#if SOCKET_DEBUG
    PcDebug_Printf("CloSkt:Id=%d\n",pThis->conId);
#endif
  }

  if (pThis->started_delay==SOCKET_FALSE)
  {
    pThis->started_delay = SOCKET_TRUE;
    pThis->link_state = SOCKET_LINK_STATE_CLOSED;

    Modem_SocketClose(pThis); // 正常关闭,保证通信双方都能够收到应用程序发出的所有数据
    pThis->timer_10ms = 500;  // 5秒
  }
  else
  {
    if (pThis->timer_10ms == 0)
    {
      if (pThis->error_cnt > 3) // 连接3次都失败,暂停1分钟后再连
      {
        pThis->state = NET_SOCKET_STATE_SILENCE; // 重连
        pThis->state_transition = SOCKET_TRUE;
      }
      else
      {
        pThis->state = NET_SOCKET_STATE_INVALID; // 重连
        pThis->state_transition = SOCKET_TRUE;
      }
    }
  }
}

//=============================================================================================
void Net_SocketSilence(skt_context_t* pThis)
{
  if (pThis->state_transition==SOCKET_TRUE)
  {
    pThis->state_transition = SOCKET_FALSE;
    pThis->started_delay = SOCKET_FALSE;
#if SOCKET_DEBUG
    PcDebug_Printf("SilSkt:Id=%d\n",pThis->conId);
#endif
  }

  if (pThis->started_delay==SOCKET_FALSE)
  {
    pThis->started_delay = SOCKET_TRUE;

    pThis->link_state = SOCKET_LINK_STATE_CLOSED;
    pThis->error_flag = SOCKET_TRUE;
    pThis->timer_10ms = 12000; // 2分钟
  }
  else
  {
    if (pThis->timer_10ms == 0)
    {
      pThis->error_cnt = 0x00; // 清除错误计数
      pThis->error_flag = SOCKET_FALSE;

      pThis->state = NET_SOCKET_STATE_INVALID; // 重连
      pThis->state_transition = SOCKET_TRUE;
    }
  }
}

/******************************************************************************
* 此函数负责维护Socket链接,每10ms调用一次(无阻塞)
*******************************************************************************/
void Net_SocketManageService(skt_context_t* pThis)
{
  switch (pThis->state)
  {
  case NET_SOCKET_STATE_INVALID: // 等待4G网络READY
    Net_SocketInvalid(pThis);
    break;

  case NET_SOCKET_STATE_CREATE: // 创建SOCKET通信端点
    Net_SocketCreate(pThis);
    break;

  case NET_SOCKET_STATE_CONNECT: // 连接服务器
    Net_SocketConnect(pThis);
    break;

  case NET_SOCKET_STATE_READY: // 已连接可通信
    Net_SocketReady(pThis);
    break;

  case NET_SOCKET_STATE_CLOSE: // 关闭SOCKET
    Net_SocketClose(pThis);
    break;

  case NET_SOCKET_STATE_SILENCE: // 静默一段时间
    Net_SocketSilence(pThis);
    break;

  default:
    pThis->state = NET_SOCKET_STATE_CLOSE;
    pThis->state_transition = TRUE;
    break;
  }

  if (pThis->timer_10ms)
    pThis->timer_10ms--;
}


/******************************************************************************
 * 功能: CAT1模块网络数据发送接口
 * 输入: pucData=待发送数据指针; usLen=待发送数据长度
 * 返回: 0 :成功返回 -1:发送失败
*******************************************************************************/
int32_t net_socket_send(skt_context_t* pThis, uint8_t *pData, uint16_t len)
{
  uint8_t it;

  if((Modem_GetState()!=MODEM_STATE_DATA_READY))
  {
    return -1;
  }

  if ((pData==NULL) || (len==0) || (len > SOCKET_CONN_MAX_DATA_LEN))
  {
#if SOCKET_DEBUG
    PC_DebugPrintf("SktSendParaErr:Id=%d\n",pThis->conId);
#endif
    return -1;
  }

  for(it=0; it<200; it++)    //如果该链接忙,最多等待2s
  {
    if(pThis->link_state==SOCKET_LINK_STATE_READY)
    {
      break;
    }
    CELLURA_DELAY(OS_TICKS_PER_SEC/100);  // 等待10ms
  }

  if (pThis->link_state==SOCKET_LINK_STATE_READY)  // 在线状态
  {
    memcpy(pThis->tx_buff, pData, len); // 拷贝数据
    pThis->tx_len = len; // 数据长度
    pThis->new_tx_data_flag = 1;  // 有新数据要发送
    pThis->link_state = SOCKET_LINK_STATE_BUSY; // 连接忙碌

    return 0;
  }
  else
  {
    PcDebug_Printf("SktSendBusy:Id=%d\n",pThis->conId);
    return -1;
  }
}

/******************************************************************************
*
*******************************************************************************/
uint8_t net_socket_GetLinkState(skt_context_t* pThis)
{
  return pThis->link_state;
}

void net_socket_SetLinkState(skt_context_t* pThis,uint8_t link_state)
{
  pThis->link_state = link_state;
}

/******************************************************************************
* 下面函数由用户编写
*******************************************************************************/
//=========================================================================
void Net_SocketInit(void)
{
  M2M_NetSocketInit();  // M2M数据初始化
  HJEP_NetSocketInit(); // 国六环保数据初始化
}

//int32_t net_socket_send(skt_context_t* pThis, uint8_t *pData, uint16_t len);

//=========================================================================
void net_socket_recv(uint8_t conId, uint8_t* pData, uint16_t recv_size)
{
  if(recv_size > 0)
  {
    SbusMsg_Gprs.data_size = recv_size;
    SbusMsg_Gprs.data = pData;
    SbusMsg_Gprs.resv = conId;
    SYSBUS_PutMbox(SbusMsg_Gprs);
  }
}

//=========================================================================
void net_socket_close(uint8_t conId)
{
  if(SOCKET_LINK_M2M == conId)
  {
    net_socket_SetLinkState(&m2m_socket, SOCKET_LINK_STATE_CLOSED);
  }
  else if(SOCKET_LINK_HJ == conId)
  {
    net_socket_SetLinkState(&hjep_socket, SOCKET_LINK_STATE_CLOSED);
  }
  else if(SOCKET_LINK_GB == conId)
  {
    net_socket_SetLinkState(&gbep_socket, SOCKET_LINK_STATE_CLOSED);
  }
}

//=========================================================================
uint8_t net_IsAllSocketClosed(void)
{
  if((hjep_socket.state==NET_SOCKET_STATE_INVALID) && (m2m_socket.state==NET_SOCKET_STATE_INVALID))
  {
    return SOCKET_TRUE;
  }
  else
  {
    return SOCKET_FALSE;
  }
}

// 复位socket
void net_socket_reset(void)
{
	if (gGsm_ZxSocketFd >= 0)
	{
		shutdown(gGsm_ZxSocketFd,SHUT_RDWR);
		sleep(1);
		close(gGsm_ZxSocketFd);
		gGsm_ZxSocketFd = -1;
	}

	if (gGsm_ZxepSocketFd >= 0)
	{
		shutdown(gGsm_ZxepSocketFd,SHUT_RDWR);
		sleep(1);
		close(gGsm_ZxepSocketFd);
		gGsm_ZxepSocketFd = -1;
	}
  
	sleep(1);
}


//==网络管理===============================================================
void Net_StateManageService(void)
{
  Net_SocketManageService(&m2m_socket);  // m2m链接管理
  Net_SocketManageService(&hjep_socket);  // ep链接管理
  //Net_SocketManageService(&gbep_socket);  // ep链接管理
}

/******************************************************************************
 * CAT1模块主进程,负责对模块的所有操作
*******************************************************************************/
void pthread_CelluraProduce(void *argument)
{
  while (1)  // 10m执行一次
  {
    CELLURA_DELAY(OS_TICKS_PER_SEC/100); // 10ms执行一次
    Modem_StateManageService();   // modem管理
    Net_StateManageService();  // 网络管理
    //if (f_stuGsmState.ucSmsRdy)  // 短信管理
    //{
    //  SendSms();
    //}
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// AT指令处理函数
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/******************************************************************************
 * strnstr - 反向查找s2在s1第一次的指针地址; 若找不到,指针地址为NULL
 * @src: The string to be searched
 * @substr: The string to search for
 * @src_len: 在src的前src_len字符中查找
 ******************************************************************************/
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
      if ( memcmp(p, substr, substr_len ) == 0)
      {
        return (char*)p;
      }
    }
    p++;
  }

  return NULL;
}

/******************************************************************************
 * \brief : Parse number from string
 * \note   Input string pointer is changed and number is skipped
 * \param[in,out]   str: Pointer to pointer to string to parse
 * \return  Parsed number
 *****************************************************************************/
int32_t AT_ParseNumber(const char** str)
{
  int32_t val = 0;
  uint8_t minus = 0;
  const char* p = *str;

  if (*p == '"'){    // Skip leading quotes
    ++p;
  }
  if (*p == ',') {  // Skip leading comma
    ++p;
  }
  if (*p == '"') {  // Skip leading quotes
    ++p;
  }
  if (*p == '/') {  // Skip '/' character, used in datetime
    ++p;
  }
  if (*p == ':') {  // Skip ':' character, used in datetime
    ++p;
  }
  if (*p == '+') {  // Skip '+' character, used in datetime
    ++p;
  }
  if (*p == '-') {  // Check negative number
    minus = 1;
    ++p;
  }

  while (CELLURA_CHARISNUM(*p)) // Parse until character is valid number
  {
    val = val * 10 + CELLURA_CHARTONUM(*p);
    ++p;
  }
  if (*p == '"') {  // Skip trailling quotes
    ++p;
  }
  *str = p;        // Save new pointer with new offset

  return minus ? -val : val;
}

/*******************************************************************************
 * \brief : Parse number from string as hex
 * \note Input string pointer is changed and number is skipped
 * \param[in,out] str: Pointer to pointer to string to parse
 * \return  Parsed number
 *******************************************************************************/
uint32_t AT_ParseHexNumber(const char** str)
{
  int32_t val = 0;
  const char* p = *str;

  if (*p == '"') // Skip leading quotes
  {
    ++p;
  }

  if (*p == ',') // Skip leading comma
  {
    ++p;
  }

  if (*p == '"') // Skip leading quotes
  {
    ++p;
  }

  while (CELLURA_CHARISHEXNUM(*p)) // Parse until character is valid number
  {
    val = val * 16 + CELLURA_CHARHEXTONUM(*p);
    ++p;
  }

  if (*p == ',') // Go to next entry if possible
  {
    ++p;
  }
  *str = p;   // Save new pointer with new offset
  return val;
}

/* ================================================================== */
uint8_t AT_RspAnalyze_CSQ(uint8_t *pdata, uint16_t size)
{
  /* 3GP TS27.007
    *  format: +CSQ: <rssi>,<ber>
    *
    *  <rssi>: integer type
    *          0  -113dBm or less
    *          1  -111dBm
    *          2...30  -109dBm to -53dBm
    *          31  -51dBm or greater
    *          99  unknown or not detectable
    *  <ber>: integer type (channel bit error rate in percent)
    *          0...7  as RXQUAL values in the table 3GPP TS 45.008
    *          99     not known ot not detectable
    */
  const char *str;

  str = strnstr((char *)cellular_rx_buffer, cellular_rx_size, "+CSQ", 4);
  if (str!=NULL)
  {
    str += 6;
    modem_info.csq_rssi= AT_ParseNumber(&str);
    if(modem_info.csq_rssi==99)
    {  modem_info.csq_rssi = 0;}
    return 1;
  }
  else
  {
    return 0;
  }
}


/* ================================================================== */
uint8_t AT_RspAnalyze_CCID(uint8_t *pdata, uint16_t size)
{
  /*
     * 发→◇AT+QCCID□
        收←◆
       +QCCID: 89860436101892671308
       OK
  */
  char *str;

  str = strnstr((char *)pdata, size, "+QCCID", 6);
  if (str!=NULL)
  {
    str += 8;
    (void)memcpy((void *)modem_info.iccid, (const void *)str, 20);
    return 1;
  }
  else
  {
    return 0;
  }
}

/* ================================================================== */
uint8_t AT_RspAnalyze_CIMI(uint8_t *pdata, uint16_t size)
{
  /*
     发→◇AT+CIMI□
     收←◆
     460045621801309
     OK
    */
  /*
     * IMSI是15位的十进制数。其结构如下：MCC(3位)+MNC(2位)+MSIN(10位)
     * MCC（Mobile Country Code，移动国家码）,中国为460。
     * MNC（Mobile Network Code，移动网络号码）,
     * 中国移动使用00、02、04、07，
     * 中国联通使用01、06、09，
     * 中国电信使用03、05、11，
     * 中国铁通使用20
     */
  char *str;

  if (size <= 15)
    return 0;

  str = strnstr((char *)pdata, size, "OK", 2);
  if (str != NULL)
  {
    str = ((char *)pdata) + 2;
    (void)memcpy((void *)modem_info.imsi, (const void *)str, 15);
    return 1;
  }
  else
  {
    return 0;
  }
}

/* ================================================================== */
uint8_t AT_RspAnalyze_CREG(uint8_t *pdata, uint16_t size)
{
  /* analyze parameters for +CREG
  *  Different cases to consider (as format is different)
  *  1/ answer to CREG read command
  *     +CREG: <n>,<stat>[,[<lac>],[<ci>],[<AcT>[,<cause_type>,<reject_cause>]]]
  *  2/ URC:
  *     +CREG: <stat>[,[<lac>],[<ci>],[<AcT>]]
  */
  const char *str;
  uint8_t n;
  uint8_t stat = 0;
  //uint32_t lac;
  //uint32_t ci;
  uint8_t retval;

  str = strnstr((char *)pdata, size, "+CREG", 5);
  if (str!=NULL)
  {
    str += 7; // 指向第一个参数
    if(ATCMD_IS_CUR(AT_CMD_CREG_GET))  // AT查询响应 +CREG: <n>,<stat>[,[<lac>],[<ci>]
    {
      n=n; // 编译警告
      n = AT_ParseNumber(&str);
      stat = AT_ParseNumber(&str);
      modem_info.location_area_code = AT_ParseHexNumber(&str);  
      modem_info.cell_id = AT_ParseHexNumber(&str);
    }
    else // URC +CREG: <stat>[,[<lac>],[<ci>]
    {
      stat = AT_ParseNumber(&str);
      modem_info.location_area_code = AT_ParseHexNumber(&str);
      modem_info.cell_id = AT_ParseHexNumber(&str);
    }

    // 1 - registered, home network
    // 5 - registered, roaming
    if(stat==1 || stat==5)  
    {
      retval = 1;
    }
    else
    {
      retval = 0;
    }
  }
  else
  {
    retval = 0;
  }

  if(ATCMD_IS_CUR(AT_CMD_CREG_GET))
  {
    if(retval)
    {
      atcmd_rsp_status = ATCMD_RSP_OK;
    }
  }

  return retval;
}


/* ================================================================== */
uint8_t AT_RspAnalyze_CEREG(uint8_t *pdata, uint16_t size)
{
  /* 
  * +CEREG: <n>,<stat>[,[<lac>],[<ci>],[<AcT>[,<cause_type>,<reject_cause>]]]
  */
  const char *str;
  uint8_t n;
  uint8_t stat = 0;
  uint8_t retval;

  str = strnstr((char *)pdata, size, "+CEREG", 6);
  if (str!=NULL)
  {
    str += 8; // 指向第一个参数
    n=n; // 编译警告
    n = AT_ParseNumber(&str);
    stat = AT_ParseNumber(&str);

    // 1 - registered, home network
    // 5 - registered, roaming
    if(stat==1 || stat==5)  
    {
      retval = 1;
    }
    else
    {
      retval = 0;
    }
  }
  else
  {
    retval = 0;
  }

  return retval;
}

/* ================================================================== */
uint8_t AT_RspAnalyze_CGATT(uint8_t *pdata, uint16_t size)
{
  // +CGATT: 1
  const char *str;

  str = strnstr((char *)pdata, size, "+CGATT", 6);
  if (str!=NULL)
  {
    str += 8;
    if (str[0] == '1')
    {
      modem_info.network_attach_status = 1;
    }
    else if (str[0] == '0')
    {
      modem_info.network_attach_status = 0;
    }
    return 1;
  }
  else
  {
    return 0;
  }
}

/* ================================================================== */
uint8_t AT_RspAnalyze_CGPADDR(uint8_t *pdata, uint16_t size)
{
  // +CGPADDR: 1,“10.76.51.180”
  const char *str;
  uint8_t ip[4]; // IPv4 address
  uint8_t buf[40];

  str = strnstr((char *)pdata, size, "+CGPADDR", 8);
  if (str!=NULL)
  {
    str += 12;  // 指向逗号

    if (*str == ','){
      str++;
    }
    if (*str == '"') {
      str++;
    }

    if (CELLURA_CHARISNUM(*str))
    {
      ip[0] = AT_ParseNumber(&str);
      str++;
      ip[1] = AT_ParseNumber(&str);
      str++;
      ip[2] = AT_ParseNumber(&str);
      str++;
      ip[3] = AT_ParseNumber(&str);
    }
    
    if (ip[0] || ip[1] || ip[2] || ip[3])
    {
      snprintf((char *)buf, sizeof(buf), "IP= %d.%d.%d.%d\r\n", ip[0],ip[1],ip[2],ip[3]);
      PcDebug_SendData(buf, strlen((const char *)buf), DBG_MSG_TYPE_AT);
      return 1;
    }
  }

  return 0;
}

/******************************************************************************
 * 返回: 1-字符串匹配, 0-不匹配
******************************************************************************/
uint8_t AT_RspAnalyzeURC(const char *rsp)
{
  const char *strx;
  uint8_t rsp_size;

  rsp_size = strlen(rsp);
  strx = strnstr((char *)cellular_rx_buffer, cellular_rx_size, rsp, rsp_size);
  if (strx != NULL)
    return 1;
  else
    return 0;
}

/******************************************************************************
 * 解析QIURC
******************************************************************************/
uint8_t fRspAnalyze_QIURC(uint8_t *pdata, uint16_t size)
{
  /*IP AT Commands manual - LTE Module Series - V1.0
  * URC
  * +QIURC: "closed",<connectID> : URC of connection closed
  * +QIURC: "recv",<connectID> : URC of incoming data
  * +QIURC: "pdpdeact",<contextID> : URC of PDP deactivation
  * +QIURC: "closed",1
  */
  const char *str1;
  const char *str2;
  uint8_t retval = 0;
  uint8_t connectID;
  //uint8_t contextID;
  uint16_t recv_size = 0x00;
  uint8_t* pData;
  str1 = strnstr((char *)pdata, size, "+QIURC:", 7);
  if (str1!=NULL)
  {
    str1 += 9; // 指向"

    if ((str2 = strnstr((char *)str1, (size-7), "recv", 4)) != NULL) // 数据接收通知(必须最先判断)
    {
      // +QIURC: "recv",<connectID>,<currentrecvlength><CR><LF><data>
      //PRINT_DBG("+QIURC recv info received")
      retval = 1;
      str2 += 6; // 指向,
      connectID = AT_ParseNumber(&str2);
      recv_size = AT_ParseNumber(&str2);
      pData = (uint8_t*)(str2+2);
      net_socket_recv(connectID, pData, recv_size);
    }
    else if ((str2 = strnstr((char *)str1, (size-7), "pdpdeact", 8)) != NULL) // PDP 去激活通知
    {
      //PRINT_DBG("+QIURC pdpdeact info received")
      // +QIURC: "pdpdeact",<contextID>
      retval = 1;
      str2 += 10; // 指向,
      //contextID = AT_ParseNumber(&str2);
      modem_info.pdpdeact_cnt++;
    }
    else if ((str2 = strnstr((char *)str1, (size-7), "closed", 6)) != NULL) // 连接断开通知
    { // 这个最好放在最后面判断,远程升级BIN文件可能含有相应的字符串
      //PRINT_DBG("+QIURC closed info received")
      // +QIURC: "closed",<connectID>
      retval = 1;
      str2 += 8; // 指向,
      connectID = AT_ParseNumber(&str2);
      net_socket_close(connectID);
    }
  }

  return retval;
}

/******************************************************************************
 * 提取SMS数据包裸数据
 *******************************************************************************/
 #if 0
uint8_t AT_ParseSms(uint8_t *pdata, uint16_t size)
{
  uint8  ucMasFlag = 0;
  uint16 ucLen = 0;
  uint8  ucPackHeadLen=0;                     //中心号码长度
  uint8  ucIndex=4;                           //索引值
  uint8  ucType=0;                            //数据编码方式
  uint8  *pucPoint1 = (uint8*)pucSrc;
  uint8  *pucPoint2 = 0;
  uint8  *pucPoint3 = (uint8*)pucSrc;
  uint8  ucaComNum[]="13973100157";         	//中联万能号
  uint8  aucGimisSimNum[]="13809046648";		//XGIT硬件研发部万能号
  uint8  aucSrcNumber[15];						//接收到的短信的源号码
  uint8  aucTemp[12];

  // 删除所有短信
  AT_SendCmd(AT_CMD_CMGD, 1, (OS_TICKS_PER_SEC/3));  // 等待300ms

  if (NULL==pdata || 0==size)
  {
    sprintf((char*)aucTemp, "AT+CMGD=,4\r");
    WriteGsmUartData((uint8*)aucTemp, strlen((char*)aucTemp));
    return FALSE;
  }

  //+CMT: "+8613501154105"，，"01/09/13，11:04:09+32"
  //+CMT: "1064899110196","","19/10/17,14:45:00+32"
  //WakeUpforRC
  //+CMT: "1064899110196","","19/10/17,14:40:47+32"
  //*PW,0006:139.23.123.245,0008:10012#
  //公田卡:
  //+CMT: "1064899110196","","20/06/28,09:25:32+32"
  //*PW,0006:139.23.123.245,0008:10012#
  //先搜索 *
  pucPoint1 = SearchString((uint8*)pucSrc+3, usSrcLen, "*", 1);//3为随机数
  if (NULL==pucPoint1)
  {
    sprintf((char*)aucTemp, "AT+CMGD=,4\r");
    WriteGsmUartData((uint8*)aucTemp, strlen((char*)aucTemp));
    return FALSE;
  }
  pucPoint2 = SearchString((uint8*)pucSrc+3, usSrcLen, "#", 1);//3为随机数
  if (NULL==pucPoint1)
  {
    sprintf((char*)aucTemp, "AT+CMGD=,4\r");
    WriteGsmUartData((uint8*)aucTemp, strlen((char*)aucTemp));
    return FALSE;
  }
  //计算长度
  if (pucPoint2>pucPoint1)
  {
    f_stuRecvSmsMsg.pMsgPacket = pucPoint1;
    f_stuRecvSmsMsg.usSize =  pucPoint2-pucPoint1+1;
    SYS_PutDataQ((void *)&f_stuRecvSmsMsg);
  }
  else
  {
    sprintf((char*)aucTemp, "AT+CMGD=,4\r");
    WriteGsmUartData((uint8*)aucTemp, strlen((char*)aucTemp));
    return FALSE;
  }

  sprintf((char*)aucTemp, "AT+CMGD=,4\r");
  WriteGsmUartData((uint8*)aucTemp, strlen((char*)aucTemp));
  return TRUE;
}
#endif

/******************************************************************************
 * 解析CMGL
******************************************************************************/
uint8_t AT_RspAnalyze_CMGL(uint8_t *pdata, uint16_t size)
{
  const char *str1;
  uint8_t retVal;

  str1 = strnstr((char *)pdata, size, "+CMGL:", 6);  // 周期性列出所有短消息
  if (str1!=NULL)
  {
    //retVal = AT_ParseSms(pdata, size);
    return retVal;
  }

  str1 = strnstr((char *)pdata, size, "+CMT: ", 6); // 新短消息提醒
  if (str1!=NULL)
  {
    //retVal = AT_ParseSms(pdata, size);
    return retVal;
  }

  return FALSE;
}

/******************************************************************************
 * 功能: 处理收到Cellura模块的响应和URC
 * 输入: pData-数据包指针;  size-数据包长度
*******************************************************************************/
static void Cellura_ProcessRecvData(uint8_t *pdata, uint16_t size)
{
  uint8_t retval = 0;
  uint8_t urc_flag = 0;

  // POWERED DOWN
  if(fRspAnalyze_QIURC(pdata,size)) // 接收数据,链接断开,承载关闭
  {
    urc_flag = CELLULAR_TRUE;
  }
  else if (AT_RspAnalyze_CREG(pdata, size)) // 处理基站ID
  {  
    urc_flag = CELLULAR_TRUE;
  }
  else if (AT_RspAnalyze_CMGL(pdata, size))
  {  
    urc_flag = CELLULAR_TRUE;
  }

  if (urc_flag == CELLULAR_FALSE)
  {
    switch (AT_GET_CMD_ID())
    {
    //====================================================================================
    case AT_CMD_QISEND:
      if (AT_RspAnalyzeURC(">"))
      {
        atcmd_rsp_status = ATCMD_RSP_OK;
      }
      break;
    case AT_CMD_QISEND_DATA:
      if (current_send_conId >= CELLULAR_MAX_SOCKETS)
      {  break;}
      if (AT_RspAnalyzeURC("SEND OK"))
      {
        atcmd_rsp_status = ATCMD_RSP_OK;
      }
      else if(AT_RspAnalyzeURC("ERROR"))
      {
        atcmd_rsp_status = ATCMD_RSP_NOK;
      }
      else if(AT_RspAnalyzeURC("SEND FAIL"))
      {
        atcmd_rsp_status = ATCMD_RSP_NOK;
      }
      break;
    case AT_CMD_QIOPEN:
      if (current_open_conId >= CELLULAR_MAX_SOCKETS)
      {  break;}
      retval = AT_RspAnalyze_QIOPEN(pdata, size);
      if (retval==1)
      {
        atcmd_rsp_status = ATCMD_RSP_OK;
      }
      else if (retval==2)
      {
        atcmd_rsp_status = ATCMD_RSP_NOK;
      }
      break;
    case AT_CMD_QICLOSE:
      if (AT_RspAnalyzeURC("OK"))
      {
        atcmd_rsp_status = ATCMD_RSP_OK;
      }
      break;
      
    //====================================================================================
    case AT_CMD_CPIN_GET:
      if (AT_RspAnalyzeURC("+CPIN: READY"))
      {
        atcmd_rsp_status = ATCMD_RSP_OK;
      }
      break;
    case AT_CMD_CSQ_GET:
      retval = AT_RspAnalyze_CSQ(pdata, size);
      if ((retval == 1) && (modem_info.csq_rssi > 6))
      {
        atcmd_rsp_status = ATCMD_RSP_OK;
      }
      break;
    case AT_CMD_CGATT_GET:
      retval = AT_RspAnalyze_CGATT(pdata, size);
      if ((retval == 1) && (modem_info.network_attach_status == 1))
      {
        atcmd_rsp_status = ATCMD_RSP_OK;
      }
      break;
      
    //====================================================================================
    case AT_CMD_AT:
      if (AT_RspAnalyzeURC("OK"))
      {
        atcmd_rsp_status = ATCMD_RSP_OK;
      }
      break;
    case AT_CMD_ATE0:
      if (AT_RspAnalyzeURC("OK"))
      {
        atcmd_rsp_status = ATCMD_RSP_OK;
      }
      break;
    case AT_CMD_CGMR:
      if (AT_RspAnalyzeURC("OK"))
      {
        atcmd_rsp_status = ATCMD_RSP_OK;
      }
      break;
    case AT_CMD_CMEE_SET:
      if (AT_RspAnalyzeURC("OK"))
      {
        atcmd_rsp_status = ATCMD_RSP_OK;
      }
      break;
    case AT_CMD_CMGF_SET:
      if (AT_RspAnalyzeURC("OK"))
      {
        atcmd_rsp_status = ATCMD_RSP_OK;
      }
      break;
    case AT_CMD_CPMS_SET:
      if (AT_RspAnalyzeURC("OK"))
      {
        atcmd_rsp_status = ATCMD_RSP_OK;
      }
      break;
    case AT_CMD_CNMI_SET:
      if (AT_RspAnalyzeURC("OK"))
      {
        atcmd_rsp_status = ATCMD_RSP_OK;
      }
      break;
    case AT_CMD_CIMI_GET:
      retval = AT_RspAnalyze_CIMI(pdata, size);
      if (retval==1)
      {
        atcmd_rsp_status = ATCMD_RSP_OK;
      }
      break;
    case AT_CMD_QCCID_GET:
      retval = AT_RspAnalyze_CCID(pdata, size);
      if (retval == 1)
      {
        atcmd_rsp_status = ATCMD_RSP_OK;
      }
      break;
    case AT_CMD_QCFG3_SET:
      if (AT_RspAnalyzeURC("OK"))
      {
        atcmd_rsp_status = ATCMD_RSP_OK;
      }
      break;
    case AT_CMD_CREG0_SET:
      if (AT_RspAnalyzeURC("OK"))
      {
        atcmd_rsp_status = ATCMD_RSP_OK;
      }
      break;
    case AT_CMD_CREG2_SET:
      if (AT_RspAnalyzeURC("OK"))
      {
        atcmd_rsp_status = ATCMD_RSP_OK;
      }
      break;
    case AT_CMD_CEREG_GET:
      retval = AT_RspAnalyze_CEREG(pdata, size);
      if (retval == 1)
      {
        atcmd_rsp_status = ATCMD_RSP_OK;
      }
      break;
    case AT_CMD_CGATT_SET:
      if (AT_RspAnalyzeURC("OK"))
      {
        atcmd_rsp_status = ATCMD_RSP_OK;
      }
      break;
    case AT_CMD_CGDCONT_SET:
      if (AT_RspAnalyzeURC("OK"))
      {
        atcmd_rsp_status = ATCMD_RSP_OK;
      }
      break;
    case AT_CMD_QIACT_SET:
      if (AT_RspAnalyzeURC("OK"))
      {
        modem_info.pdpdeact_cnt = 0;
        modem_info.pdp_actived_flag = 1;
        atcmd_rsp_status = ATCMD_RSP_OK;
      }
      break;
    case AT_CMD_QIDEACT_SET:
      if (AT_RspAnalyzeURC("OK"))
      {
        modem_info.pdpdeact_cnt = 0;
        modem_info.pdp_actived_flag = 0;
        atcmd_rsp_status = ATCMD_RSP_OK;
      }
      break;
    case AT_CMD_CGPADDR_GET:  // Show PDP Address
      if (AT_RspAnalyze_CGPADDR(pdata, size))
      {
        atcmd_rsp_status = ATCMD_RSP_OK;
      }
      break;
    case AT_CMD_QSCLK:
      if (AT_RspAnalyzeURC("OK"))
      {
        atcmd_rsp_status = ATCMD_RSP_OK;
      }
      break;
      
    //====================================================================================
      /*
    case ATSEND_READSMS_CMGL:
      temp = UnPackSms(pucData, usLen);
      if(TRUE == temp)
      {
       GsmAtCmdState.ucCMGL = 1;
      }
      break;
      */
    case AT_CMD_CMGD:
      if (AT_RspAnalyzeURC("OK"))
      {
        atcmd_rsp_status = ATCMD_RSP_OK;
      }
      break;
    case AT_CMD_CMGS:
      if (AT_RspAnalyzeURC(">"))
      {
        atcmd_rsp_status = ATCMD_RSP_OK;
      }
      break;
    case AT_CMD_CMGS_DATA:
      //if (AT_RspAnalyzeURC(">"))
      //{
      //  atcmd_rsp_status = ATCMD_RSP_OK;
      //}
      break;

    default:
      break;
    }
  }
}

/******************************************************************************
 * CAT1模块数据接收进程
*******************************************************************************/
void pthread_CelluraProcess(void *argument)
{
  uint8_t *pdata = NULL;
  uint16_t len;

  while (1)
  {
    if (CELLURA_Receive(&pdata, &len)) // 等待信号量(阻塞)
    {
      PcDebug_SendData(pdata, len, DBG_MSG_TYPE_AT);
      Cellura_ProcessRecvData(pdata, len);
    }
  }
}

/*************************************************************************
 *
*************************************************************************/
void Cellura_Init(void)
{
  Modem_SetState(MODEM_STATE_OFF);
  Net_SocketInit();
}

/*************************************************************************
 *
*************************************************************************/
void Cellura_ServiceInit(void)
{
  Modem_GpioInitialize();
  USART3_Initialize(MODEM_UART_BAUDRATE);
  Cellura_Init();
}

/*************************************************************************
 *
*************************************************************************/
void Cellura_ServiceStart(void)
{
  pthread_create(&pthreads[PTHREAD_CELLURA_PROCESS_ID], NULL, pthread_CelluraProcess, NULL);
  usleep(10);
   pthread_create(&pthreads[PTHREAD_CELLURA_PRODUCE_ID], NULL, pthread_CelluraProduce, NULL);
  usleep(10);
}


////////////////////////////////////////////////////////////////////////////////////////////
// 外部信息接口
////////////////////////////////////////////////////////////////////////////////////////////
//==获取SIM卡的ICCID============================================================
uint8_t *Cellura_GetIccid(void)
{
  return modem_info.iccid;
}

//==获取移动网信号强度==========================================================
uint8_t Cellura_GetCsq(void)
{
  return modem_info.csq_rssi;
}

//==获取小区ID==================================================================
uint32 Cellura_GetLacCellID(void)
{
  return modem_info.cell_id;
}

//==获取模块故障状态:1=故障, 0=正常=============================================
uint8_t Cellura_GetModemStatus(void)
{
  return modem_info.modem_err_flag;
}

//==获取SIM卡是否故障: 1=故障; 0,正常===========================================
uint8_t Cellura_GetSimCardState(void)
{
  if(modem_info.sim_card_ok_flag)
    return 0;
  else
    return 1;
}

//==获取模块CS域注册状态========================================================
uint8_t Cellura_GetCsRegistState(void)
{
  return modem_info.cs_network_regist_status;
}

//==获取模块PS域注册状态========================================================
uint8_t Cellura_GetPsRegistState(void)
{
  return modem_info.ps_network_regist_status;
}

//-----文件CelluraCore.c结束---------------------------------------------
