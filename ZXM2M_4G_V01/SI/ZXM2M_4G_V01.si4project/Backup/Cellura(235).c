/*******************************************************************************
 * Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
 * @FileName: CelluraCore.c
 * @Engineer: TenYan
 * @version:  V1.0
 * @Date:     2020-10-2
 * @brief:
 *******************************************************************************/
//-----ͷ�ļ�����------------------------------------------------------------
#include "config.h"

/******************************************************************************
 *   Macros
 ******************************************************************************/
#define MODEM_DEV_TTY              "/dev/smd8"
#define MODEM_UART_RX_BUFFER_SIZE  1500

#define CELLURA_Receive    Modem_UartReceiveData

#define CELLULAR_MAX_RXBUF_SIZE  MODEM_UART_RX_BUFFER_SIZE
//#define cellular_rx_buffer       usart3_rx_buffer
//#define cellular_rx_size         usart3_rx_size

#define AT_transaction(p_buffer,size)  do { Cellura_TransmitData(p_buffer,size); } while (0) // ͨ������DMA����ATָ�ģ��
#define AT_SET_CMD_ID(cmd)   (current_atcmd_id = cmd)
#define AT_GET_CMD_ID()      (current_atcmd_id)
#define ATCMD_IS_CUR(cmd)    (current_atcmd_id == (cmd))

#define AT_DELAY(ms)         do { usleep(1000*(ms)); } while(0)
#define MODEM_DELAY_MS(ms)   do { usleep(1000*(ms)); } while(0)
#define CELLURA_DELAY(ms)    do { usleep(1000*(ms)); } while(0)

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
* ATָ���
******************************************************************************/
const char AT_CFUN0_SET[]="AT+CFUN=0\r\n"; // ģ��������ģʽ
#define SIZE_OF_AT_CFUN0_SET  (sizeof(AT_CFUN0_SET)/sizeof(AT_CFUN0_SET[0]) - 1)

const char AT_CFUN1_SET[]="AT+CFUN=1\r\n"; // ģ��Ϊȫ����(online mode)
#define SIZE_OF_AT_CFUN1_SET  (sizeof(AT_CFUN1_SET)/sizeof(AT_CFUN1_SET[0]) - 1)

const char AT_CRESET[]="AT\r\n"; // ģ�������λ
#define SIZE_OF_AT_CRESET  (sizeof(AT_CRESET)/sizeof(AT_CRESET[0]) - 1)

const char AT_AT[]="AT\r\n"; // AT����
#define SIZE_OF_AT_AT  (sizeof(AT_AT)/sizeof(AT_AT[0]) - 1)

const char AT_ATE0[]="ATE0\r\n"; // �رջ���
#define SIZE_OF_AT_ATE0  (sizeof(AT_ATE0)/sizeof(AT_ATE0[0]) - 1)

const char AT_CGPS_SET[]="AT+CGPS=1\r\n"; // ����GPS�Ự
#define SIZE_OF_AT_CGPS_SET  (sizeof(AT_CGPS_SET)/sizeof(AT_CGPS_SET[0]) - 1)

const char AT_CGPSXE_SET[]="AT+CGPSXE=1\r\n"; // ʹ��GPS XTRA
#define SIZE_OF_AT_CGPSXE_SET  (sizeof(AT_CGPSXE_SET)/sizeof(AT_CGPSXE_SET[0]) - 1)

const char AT_CGPSINFOCFG_SET[]="AT+CGPSINFOCFG=2,3\r\n"; // ����NMEA-0183�������5S,���GPGGA��GPRMC��Ϣ
#define SIZE_OF_AT_CGPSINFOCFG_SET  (sizeof(AT_CGPSINFOCFG_SET)/sizeof(AT_CGPSINFOCFG_SET[0]) - 1)

const char AT_CGMR[]="AT+GMR\r\n"; // ��ѯģ������汾
#define SIZE_OF_AT_CGMR  (sizeof(AT_CGMR)/sizeof(AT_CGMR[0]) - 1)

const char AT_CMEE[]="AT+CMEE=1\r\n"; // �����ֱ����ƶ��豸�Ĵ���
#define SIZE_OF_AT_CMEE  (sizeof(AT_CMEE)/sizeof(AT_CMEE[0]) - 1)

const char AT_CPIN[]="AT+CPIN?\r\n"; // ��ѯSIM���Ƿ�READY
#define SIZE_OF_AT_CPIN  (sizeof(AT_CPIN)/sizeof(AT_CPIN[0]) - 1)

const char AT_CIMI[]="AT+CIMI\r\n"; // ��ȡSIM����IMSI
#define SIZE_OF_AT_CIMI  (sizeof(AT_CIMI)/sizeof(AT_CIMI[0]) - 1)

const char AT_CICCID[]="AT+CICCID\r\n"; // ��ȡSIM����ICCID
#define SIZE_OF_AT_CICCID  (sizeof(AT_CICCID)/sizeof(AT_CICCID[0]) - 1)

const char AT_CMGF[]="AT+CMGF=1\r\n";  // Message Format
#define SIZE_OF_AT_CMGF (sizeof(AT_CMGF)/sizeof(AT_CMGF[0]) - 1)

const char AT_CPMS[]="AT+CPMS=\"ME\",\"ME\",\"ME\"\r\n";  // ���Ŵ洢����(�洢���ƶ��豸��)
#define SIZE_OF_AT_CPMS (sizeof(AT_CPMS)/sizeof(AT_CPMS[0]) - 1)

const char AT_CNMI[]="AT+CNMI=2,2\r\n";  // SMS Event Reporting Configuration
#define SIZE_OF_AT_CNMI (sizeof(AT_CNMI)/sizeof(AT_CNMI[0]) - 1)

const char AT_CMGD[]="AT+CMGD=1,4\r\n";  // ɾ�����ж���(����δ��)
#define SIZE_OF_AT_CMGD (sizeof(AT_CMGD)/sizeof(AT_CMGD[0]) - 1)

const char AT_CMGL[]="AT+CMGL=\"ALL\"\r\n";  // �г����ж���
#define SIZE_OF_AT_CMGL (sizeof(AT_CMGL)/sizeof(AT_CMGL[0]) - 1)

const char AT_CNMP_SET[]="AT+CNMP=2\r\n"; // ������������ģʽΪAutomatic(LTE/WCDMA/GSM)
#define SIZE_OF_AT_CNMP_SET (sizeof(AT_CNMP_SET)/sizeof(AT_CNMP_SET[0]) - 1)

const char AT_CSQ[]="AT+CSQ\r\n"; // ��ѯ�����ź�����
#define SIZE_OF_AT_CSQ  (sizeof(AT_CSQ)/sizeof(AT_CSQ[0]) - 1)

const char AT_CREG0_SET[]="AT+CREG=0\r\n"; // �ر�+CREG: URC�ϱ�
#define SIZE_OF_AT_CREG0_SET  (sizeof(AT_CREG0_SET)/sizeof(AT_CREG0_SET[0]) - 1)

const char AT_CREG2_SET[]="AT+CREG=2\r\n"; // ʹ��+CREG: URC�ϱ�
#define SIZE_OF_AT_CREG2_SET  (sizeof(AT_CREG2_SET)/sizeof(AT_CREG2_SET[0]) - 1)

const char AT_CREG_GET[]="AT+CREG?\r\n"; // CSҵ��:����ע��״̬
#define SIZE_OF_AT_CREG_GET  (sizeof(AT_CREG_GET)/sizeof(AT_CREG_GET[0]) - 1)

const char AT_CGREG_GET[]="AT+CGREG?\r\n"; // PSҵ��:GPRS����ע��״̬
#define SIZE_OF_AT_CGREG_GET  (sizeof(AT_CGREG_GET)/sizeof(AT_CGREG_GET[0]) - 1)

const char AT_CEREG_GET[]="AT+CEREG?\r\n"; // PSҵ��:LTE����ע��״̬
#define SIZE_OF_AT_CEREG_GET  (sizeof(AT_CGREG_GET)/sizeof(AT_CEREG_GET[0]) - 1)

const char AT_CGATT_SET[]="AT+CGATT=1\r\n"; // ʹ��PS����
#define SIZE_OF_AT_CGATT_SET  (sizeof(AT_CGATT_SET)/sizeof(AT_CGATT_SET[0]) - 1)

const char AT_CGATT_GET[]="AT+CGATT?\r\n"; // ��ѯPS����
#define SIZE_OF_AT_CGATT_GET  (sizeof(AT_CGATT_GET)/sizeof(AT_CGATT_GET[0]) - 1)

const char AT_CGDCONT[]="AT+CGDCONT=1,\"IP\",\"cmmtm\"\r\n"; // ����PDP������
#define SIZE_OF_AT_CGDCONT  (sizeof(AT_CGDCONT)/sizeof(AT_CGDCONT[0]) - 1)

const char AT_CGACT[]="AT+CGACT=1\r\n"; // ����PDP������
#define SIZE_OF_AT_CGACT  (sizeof(AT_CGACT)/sizeof(AT_CGACT[0]) - 1)

const char AT_CGPADDR[]="AT+CGPADDR=1\r\n"; // ��ѯPDP��ַ
#define SIZE_OF_AT_CGPADDR  (sizeof(AT_CGPADDR)/sizeof(AT_CGPADDR[0]) - 1)

const char AT_CSCLK[]="AT+CSCLK=1\r\n"; // ����ģ���������ģʽ
#define SIZE_OF_AT_CSCLK  (sizeof(AT_CSCLK)/sizeof(AT_CSCLK[0]) - 1)

const char AT_CWMAP0_SET[]="AT+CWMAP=0\r\n"; // �ر�WIFIģ��
#define SIZE_OF_AT_CWMAP0_SET  (sizeof(AT_CWMAP0_SET)/sizeof(AT_CWMAP0_SET[0]) - 1)

/*************************************************************************
 *
*************************************************************************/
void Modem_UartInitialize(uint32_t baudrate)
{
  struct termios options;

  // ��ʧ�ܣ���δ���?
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
    usleep((20*1000)); // �ȴ�20ms
    return FALSE;
  }

  ret = read(fd_modem, modem_uart_rx_buffer, (MODEM_UART_RX_BUFFER_SIZE-1)); // �����ȴ�����
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

  retVal = write(fd_modem, data, size);  // д��������
  if (retVal != size)
  {
    printf("ERROR Modem write ret=%d\n", retVal);
    close(fd_modem);
    Modem_UartInitialize(B115200);
    return 0;
  }
  
  PcDebug_SendData(data, size, DBG_MSG_TYPE_AT); // ������Ϣ

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
* ����AT�������(ÿ10ms���AT�Ƿ���Ӧ)
******************************************************************************/
atStatus_t AT_SendCmd(at_cmd_t atcmdId, uint8_t retries, uint16_t timeout)
{
  uint8_t it;
  uint16_t chk_cnt = 0;
  uint16_t chk_period;
  const char* at_buf;
  uint8_t at_size;

  switch (atcmdId) // ͨ��atcmdId��ȡATָ��
  {
  case AT_CMD_CRESET: // ģ�������λ
    at_buf = AT_CRESET;
    at_size = SIZE_OF_AT_CRESET;
    break;

   case AT_CMD_CFUN0_SET: // ģ��������ģʽ
     at_buf = AT_CFUN0_SET;
     at_size = SIZE_OF_AT_CFUN0_SET;
     break;
     
   case AT_CMD_CFUN1_SET: // ģ��Ϊȫ����(online mode)
     at_buf = AT_CFUN1_SET;
     at_size = SIZE_OF_AT_CFUN1_SET;
     break;

  case AT_CMD_AT: // AT����
    at_buf = AT_AT;
    at_size = SIZE_OF_AT_AT;
    break;

  case AT_CMD_ATE0: // �رջ���
    at_buf = AT_ATE0;
    at_size = SIZE_OF_AT_ATE0;
    break;

  case AT_CMD_CGPS_SET: // ����GPS�Ự
    at_buf = AT_CGPS_SET;
    at_size = SIZE_OF_AT_CGPS_SET;
    break;

  case AT_CMD_CGPSXE_SET: // ʹ��GPS XTRA
    at_buf = AT_CGPSXE_SET;
    at_size = SIZE_OF_AT_CGPSXE_SET;
    break;

  case AT_CMD_CGPSINFOCFG_SET: // ����NMEA-0183�������5S,���GPGGA��GPRMC��Ϣ
    at_buf = AT_CGPSINFOCFG_SET;
    at_size = SIZE_OF_AT_CGPSINFOCFG_SET;
    break;

  case AT_CMD_CGMR: // ��ѯģ������汾
    at_buf = AT_CGMR;
    at_size = SIZE_OF_AT_CGMR;
    break;

  case AT_CMD_CMEE_SET: // �����ֱ�ʾ������Ϣ
    at_buf = AT_CMEE;
    at_size = SIZE_OF_AT_CMEE;
    break;

  case AT_CMD_CPIN_GET: // ��ѯSIM���Ƿ�READY
    at_buf = AT_CPIN;
    at_size = SIZE_OF_AT_CPIN;
    break;

  case AT_CMD_CIMI_GET: // ��ȡSIM����IMSI
    at_buf = AT_CIMI;
    at_size = SIZE_OF_AT_CIMI;
    break;

  case AT_CMD_CICCID_GET: // ��ȡSIM����ICCID
    at_buf = AT_CICCID;
    at_size = SIZE_OF_AT_CICCID;
    break;

  case AT_CMD_CMGF_SET: // ���ö��Ÿ�ʽ
    at_buf = AT_CMGF;
    at_size = SIZE_OF_AT_CMGF;
    break;

  case AT_CMD_CPMS_SET: // ���ö��Ŵ洢λ��
    at_buf = AT_CPMS;
    at_size = SIZE_OF_AT_CPMS;
    break;

  case AT_CMD_CNMI_SET: // SMS�¼��ϱ�����:���ϱ�
    at_buf = AT_CNMI;
    at_size = SIZE_OF_AT_CNMI;
    break;

  case AT_CMD_CNMP_SET: // ������������ģʽΪAutomatic(LTE/WCDMA/GSM)
    at_buf = AT_CNMP_SET;
    at_size = SIZE_OF_AT_CNMP_SET;
    break;

  case AT_CMD_CSQ_GET: // ��ѯ�����ź�����
    at_buf = AT_CSQ;
    at_size = SIZE_OF_AT_CSQ;
    break;
  
  case AT_CMD_CREG0_SET: // ʹ��+CREG: URC�ϱ�
    at_buf = AT_CREG0_SET;
    at_size = SIZE_OF_AT_CREG0_SET;
    break;
  
  case AT_CMD_CREG2_SET: // ʹ��+CREG: URC�ϱ�
    at_buf = AT_CREG2_SET;
    at_size = SIZE_OF_AT_CREG2_SET;
    break;

  case AT_CMD_CREG_GET: // CSҵ��:����ע��״̬
    at_buf = AT_CREG_GET;
    at_size = SIZE_OF_AT_CREG_GET;
    break;

  case AT_CMD_CGREG_GET: // PSҵ��:GPRS����ע��״̬
    at_buf = AT_CGREG_GET;
    at_size = SIZE_OF_AT_CGREG_GET;
    break;

  case AT_CMD_CEREG_GET: // PSҵ��:LTE����ע��״̬
    at_buf = AT_CEREG_GET;
    at_size = SIZE_OF_AT_CEREG_GET;
    break;

  case AT_CMD_CGATT_SET: // ʹ��PS����
    at_buf = AT_CGATT_SET;
    at_size = SIZE_OF_AT_CGATT_SET;
    break;

  case AT_CMD_CGATT_GET: // ��ѯPS����
    at_buf = AT_CGATT_GET;
    at_size = SIZE_OF_AT_CGATT_GET;
    break;

  case AT_CMD_CGDCONT_SET: // ����PDP������
    at_buf = AT_CGDCONT;
    at_size = SIZE_OF_AT_CGDCONT;
    break;

  case AT_CMD_CGACT_SET: // ����PDP������
    at_buf = AT_CGACT;
    at_size = SIZE_OF_AT_CGACT;
    break;

  case AT_CMD_CGPADDR_GET: // ��ѯPDP��ַ
    at_buf = AT_CGPADDR;
    at_size = SIZE_OF_AT_CGPADDR;
    break;

  case AT_CMD_CSCLK: // ����ģ���������״̬
    at_buf = AT_CSCLK;
    at_size = SIZE_OF_AT_CSCLK;
    break;

  case AT_CMD_CWMAP0_SET: // �ر�WIFIģ��
    at_buf = AT_CWMAP0_SET;
    at_size = SIZE_OF_AT_CWMAP0_SET;
    break;

  default:
    break;
  }

  chk_period = timeout/50 + 1;
  atcmd_rsp_status = ATCMD_RSP_NONE;
  for (it=0; it<retries; it++)
  {
    AT_SET_CMD_ID(atcmdId);
    AT_transaction((uint8_t*)at_buf,at_size); // ͨ������DMA����ATָ�ģ��
    for (chk_cnt=0; chk_cnt<chk_period; chk_cnt++)
    {
      AT_DELAY(50);  // ÿ50m���һ��AT��Ӧ״̬
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
 * ����: CAT1��ʼ�������߲���
*******************************************************************************/
//==ģ�������С����ģʽ====================================================
void Modem_MiniFun(void)
{
  atStatus_t at_rsp;

  PcDebug_SendString("ModemMiniFun!\n");
  
  at_rsp = AT_SendCmd(AT_CMD_CFUN0_SET, 3, (OS_TICKS_PER_SEC*2)); 
  if (at_rsp == ATCMD_RSP_OK)
  {
  
  }

  MODEM_DELAY_MS(OS_TICKS_PER_SEC*2);
}

//==ģ���Ը�λ==============================================================
void Modem_Reset(void)
{
  atStatus_t at_rsp;

  PcDebug_SendString("ModemReset!\n");
  //ucResetGsmModuleFlag = 1;
  //sleep(3);  // �ȴ�3s,����������ȥд�����
  //ucResetGsmModuleFlag = 0;
  //system("sync"); // ������д��FLASH��

  at_rsp = AT_SendCmd(AT_CMD_CRESET, 3, (OS_TICKS_PER_SEC*2)); 
  if (at_rsp == ATCMD_RSP_OK)
  {
  
  }

  MODEM_DELAY_MS(OS_TICKS_PER_SEC*10);
}

/******************************************************************************
 * ����: CAT1��ʼ�������߲���
*******************************************************************************/
uint8_t Modem_Init(void)
{
  atStatus_t at_rsp;

  modem_info.modem_init_fail_cnt++;

  /***********************************ģ�鿪��,���г�ʼ������**************************************/
  // ����ģ�鴮��AT�Ƿ��ʼ�����
  at_rsp = AT_SendCmd(AT_CMD_AT, 30, OS_TICKS_PER_SEC);
  if (at_rsp == ATCMD_RSP_NOK)
  {
    modem_info.at_fail_cnt++;
    return AT_FALSE; // �˳�����
  }
  else
  {
    modem_info.modem_err_flag = 0;
    modem_info.at_fail_cnt = 0;
  }
  
  // �رջ���
  AT_SendCmd(AT_CMD_ATE0, 3, OS_TICKS_PER_SEC);
  sleep(1);
  
  // ����GPS�Ự
  AT_SendCmd(AT_CMD_CGPS_SET, 2, OS_TICKS_PER_SEC);
  sleep(1);

  // ʹ��GPS XTRA
  AT_SendCmd(AT_CMD_CGPSXE_SET, 2, OS_TICKS_PER_SEC);
  sleep(1);

  // ����NMEA-0183�������5S,���GPGGA��GPRMC��Ϣ
  AT_SendCmd(AT_CMD_CGPSINFOCFG_SET, 2, OS_TICKS_PER_SEC);
  sleep(1);

  // ��ѯģ������汾
  AT_SendCmd(AT_CMD_CGMR, 1, OS_TICKS_PER_SEC);
  
  // ���ô���������ֱ�ʶ
  AT_SendCmd(AT_CMD_CMEE_SET, 3, OS_TICKS_PER_SEC);

  // ��ȡSIM��״̬
  at_rsp = AT_SendCmd(AT_CMD_CPIN_GET, 20, OS_TICKS_PER_SEC);
  if (at_rsp == ATCMD_RSP_NOK)
  {
    modem_info.sim_fail_cnt++;
    return AT_FALSE; // �˳�����
  }
  else
  {
    modem_info.sim_err_flag = 0;
    modem_info.sim_fail_cnt = 0;
  }

  // ��ȡ�ֻ�����
  at_rsp = AT_SendCmd(AT_CMD_CIMI_GET, 3, OS_TICKS_PER_SEC);
  if (at_rsp == ATCMD_RSP_NOK)
  {
    return AT_FALSE; // �˳�����
  }

  // ��ȡICCID
  at_rsp = AT_SendCmd(AT_CMD_CICCID_GET, 3, OS_TICKS_PER_SEC);
  if (at_rsp == ATCMD_RSP_NOK)
  {
    return AT_FALSE; // �˳�����
  }

  // ����ģ�������ϢģʽΪTXT
  at_rsp = AT_SendCmd(AT_CMD_CMGF_SET, 5, OS_TICKS_PER_SEC);
  if (at_rsp == ATCMD_RSP_NOK)
  {
    return AT_FALSE; // �˳�����
  }

  // ���ö�����Ϣ�洢
  at_rsp = AT_SendCmd(AT_CMD_CPMS_SET, 3, OS_TICKS_PER_SEC);
  if (at_rsp == ATCMD_RSP_NOK)
  {
    return AT_FALSE; // �˳�����
  }

  // ���ö�����Ϣ�ϱ���ʽ
  at_rsp = AT_SendCmd(AT_CMD_CNMI_SET, 5, OS_TICKS_PER_SEC);
  if (at_rsp == ATCMD_RSP_NOK)
  {
    return AT_FALSE; // �˳�����
  }

  // ����ģ��������ʽ
  AT_SendCmd(AT_CMD_CNMP_SET, 3, OS_TICKS_PER_SEC); // �Զ�ģʽ
  if (at_rsp == ATCMD_RSP_NOK)
  {
    return AT_FALSE; // �˳�����
  }

  // ��ѯ���������ź�ǿ��
  at_rsp = AT_SendCmd(AT_CMD_CSQ_GET, 30, OS_TICKS_PER_SEC);
  if (at_rsp == ATCMD_RSP_NOK)
  {
    return AT_FALSE; // �˳�����
  }

  // ʹ������״̬��С�����Զ��ϱ�
  at_rsp = AT_SendCmd(AT_CMD_CREG2_SET, 3, OS_TICKS_PER_SEC);
  if (at_rsp == ATCMD_RSP_NOK)
  {
    return AT_FALSE; // �˳�����
  }

  AT_DELAY(OS_TICKS_PER_SEC*2);

  // ��ѯCS����ע��״̬
  at_rsp = AT_SendCmd(AT_CMD_CREG_GET, 120, OS_TICKS_PER_SEC); // ��ģ������������ע��ʱ��
  if (at_rsp == ATCMD_RSP_NOK)
  {
    modem_info.cs_network_regist_status = CS_CS_DETACHED;
    // return; // ��Ҫ���أ�����������
  }
  else
  {
    modem_info.cs_network_regist_status = CS_CS_ATTACHED;
    modem_info.sms_ready_flag = 1;
  }

  // ��ѯPS����ע��״̬
  at_rsp = AT_SendCmd(AT_CMD_CEREG_GET, 30, OS_TICKS_PER_SEC); // ��ģ��30��������ע��ʱ��
  if (at_rsp == ATCMD_RSP_NOK)
  {
    modem_info.ps_network_regist_status = CS_PS_DETACHED;
    // return; // ��Ҫ���أ�����������
  }
  else
  {
    modem_info.ps_network_regist_status = CS_PS_ATTACHED;
  }

  AT_DELAY(OS_TICKS_PER_SEC);

  // ʹ��ģ�����總��
  AT_SendCmd(AT_CMD_CGATT_SET, 3, OS_TICKS_PER_SEC);

  // ��ѯ���總��״̬
  at_rsp = AT_SendCmd(AT_CMD_CGATT_GET, 240, OS_TICKS_PER_SEC); // ��ģ���ķ��Ӹ�������ʱ��
  if (at_rsp == ATCMD_RSP_NOK)
  {
    modem_info.network_attach_status = CS_PS_DETACHED;
    return AT_FALSE; // �˳�����
  }
  else
  {
    modem_info.network_attach_status = CS_PS_ATTACHED;
  }

  // ����PDP(Packet Data Protocol��������)������
  at_rsp = AT_SendCmd(AT_CMD_CGDCONT_SET, 3, OS_TICKS_PER_SEC);
  if (at_rsp == ATCMD_RSP_NOK)
  {
    return AT_FALSE; // �˳�����
  }

  // ����ģ��������ʽ״̬�Զ��ϱ�
  // AT_SendCmd(AT_CMD_CNSMOD_SET, 3, (OS_TICKS_PER_SEC/2));

  // ����PDP,ֻ��һ��
  //at_rsp = AT_SendCmd(AT_CMD_CGACT_SET, 1, (25*OS_TICKS_PER_SEC));
  //if (at_rsp == ATCMD_RSP_NOK)
  //{
  //  return AT_FALSE; // �˳�����
  //}

  // ��ѯSocket PDP��IP��ַ
  at_rsp = AT_SendCmd(AT_CMD_CGPADDR_GET, 20, OS_TICKS_PER_SEC);
  if (at_rsp == ATCMD_RSP_NOK)
  {
    return AT_FALSE; // �˳�����
  }

  // �ر�WIFIģ��
  at_rsp = AT_SendCmd(AT_CMD_CWMAP0_SET, 2, (OS_TICKS_PER_SEC*8));
  if (at_rsp == ATCMD_RSP_NOK)
  {
    //return AT_FALSE; // �˳�����
  }

  //sleep(3);   // ģ�鳧�ҽ���鵽IP��,�ȴ�����������

  modem_info.network_ready_flag = 1;
  modem_info.modem_init_fail_cnt = 0;
  PcDebug_SendString("Modem Data Ready!\n");

  return AT_TRUE;
}

#define CST_QNS_STATE_SIMCARD  0x01  // ��ѯSIM��״̬
#define CST_QNS_STATE_RFSQ     0x02  // ��ѯ�����ź�����
#define CST_QNS_STATE_NETWORK  0x03  // ��ѯ���總��״̬
/******************************************************************************
 *����: ��ѯCAT1ģ������״̬(SIM�����ź����������總��״̬)
*******************************************************************************/
void Modem_QueryNetworkState(void)
{
  static uint8_t qns_state = CST_QNS_STATE_SIMCARD;
  atStatus_t at_rsp;

  switch (qns_state)
  {
  case CST_QNS_STATE_SIMCARD:  // ��ȡSIM��״̬
    at_rsp = AT_SendCmd(AT_CMD_CPIN_GET, 1, (OS_TICKS_PER_SEC/20));  // �ȴ�50ms
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

  case CST_QNS_STATE_RFSQ: // ��ѯ���������ź�ǿ��
    AT_SendCmd(AT_CMD_CSQ_GET, 1, (OS_TICKS_PER_SEC/20));
    qns_state = CST_QNS_STATE_NETWORK;
    break;

  case CST_QNS_STATE_NETWORK: // ��ѯ���總��״̬
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
      WriteGsmUartData((uint8 *)"AT+CMGL=\"ALL\"\r", 14); // �г�SIM���еĶ���Ϣ(ȫ����)
      OSTimeDly(OS_TICKS_PER_SEC/50);//�ȴ�20ms
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
* Modem ���ϴ���
******************************************************************************/
void Modem_ProcessError(void)
{
  // atָ��δ��Ӧ,ģ�����
  if (modem_info.at_fail_cnt > 5)
  {
    modem_info.at_fail_cnt = 0;
    modem_info.modem_err_flag = 1;
    PcDebug_SendString("ModemRst:AT!\n");
    Modem_SetState(MODEM_STATE_MINI_FUN); // ����ģ��
  }

  // SIM������
  if (modem_info.sim_fail_cnt > 5)
  {
    modem_info.sim_fail_cnt = 0;
    modem_info.sim_err_flag = 1;
    PcDebug_SendString("ModemRst:SimCard!\n");
    Modem_SetState(MODEM_STATE_MINI_FUN); // ����ģ��
  }

  // ����δ����
  if (modem_info.cgatt_fail_cnt > 5)
  {
    modem_info.cgatt_fail_cnt = 0;
    modem_info.network_attach_status = CS_PS_DETACHED;
    PcDebug_SendString("ModemRst:CGATT!\n");
    Modem_SetState(MODEM_STATE_MINI_FUN); // ����ģ��
  }
  
  if(modem_info.pdpdeact_cnt > 0)
  {
    modem_info.pdpdeact_cnt = 0;
    modem_info.pdp_actived_flag = 0;
    PcDebug_SendString("ModemRst:PDP!\n");
    Modem_SetState(MODEM_STATE_MINI_FUN); // ����ģ��
  }
}

/******************************************************************************
 * ģ��״̬��(10ms����һ��)
*******************************************************************************/
void Modem_StateManageService(void)               /// ����
{
  uint8_t modem_state;
  uint8_t retval;
  static uint16_t qns_timer_10ms = 500; // ÿ5���ѯһ��

  modem_state = Modem_GetState();
  switch (modem_state)
  {
  case MODEM_STATE_INIT:  // ģ���ʼ��
    retval = Modem_Init();
    if (retval == AT_TRUE)
    {
      Modem_SetState(MODEM_STATE_DATA_READY); // ��ʼ���ɹ�,ת����ģʽ
    }
    else
    {
      Modem_SetState(MODEM_STATE_MINI_FUN); // ��ʼ��ʧ��,תģ��ػ�
    }
    break;

  case MODEM_STATE_MINI_FUN:
    Modem_MiniFun(); // ģ�������С����
    if (modem_info.modem_init_fail_cnt > 5)  // �����������ɹ���Ϣ10����
    {
      modem_info.modem_init_fail_cnt = 0;
      Modem_SetState(MODEM_STATE_SILENCE);
    }
    else // ���¿���
    {
      Modem_SetState(MODEM_STATE_INIT); // ���¿���
    }
    break;

  case MODEM_STATE_RESET:
    Modem_Reset();  // ģ�������λ
    Modem_SetState(MODEM_STATE_INIT); // ���¿���
    break;

  case MODEM_STATE_DATA_READY:  // ģ�������
    if (qns_timer_10ms)
      qns_timer_10ms--;
    else
    {
      qns_timer_10ms = 500; // ÿ5���ѯһ��
      Modem_QueryNetworkState(); // ��ѯģ��״̬
    }
    Modem_ProcessError();
    break;

  case MODEM_STATE_SILENCE:  // ģ�龲Ĭ5����
    PcDebug_SendString("Modem Rest 10min\n");
    MODEM_DELAY_MS((600*OS_TICKS_PER_SEC));
    Modem_SetState(MODEM_STATE_INIT); // ���¿���
    break;

  default:
    Modem_SetState(MODEM_STATE_MINI_FUN);
    break;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ����Ϣ����
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/******************************************************************************
* ����: ���Ͷ���Ϣ������
*******************************************************************************/
#if 0
void Modem_SendSMS(void)
{

}
#endif

/******************************************************************************
 * CAT1ģ��������,�����ģ������в���
*******************************************************************************/
void* pthread_CelluraProduce(void *argument)
{
  while (1)  // 10mִ��һ��
  {
    CELLURA_DELAY(OS_TICKS_PER_SEC/100); // 10msִ��һ��
    Modem_StateManageService();   // modem����
    //Net_StateManageService();  // �������
    //if (f_stuGsmState.ucSmsRdy)  // ���Ź���
    //{
    //  SendSms();
    //}
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ATָ�����
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/******************************************************************************
 * strnstr - �������s2��s1��һ�ε�ָ���ַ; ���Ҳ���,ָ���ַΪNULL
 * @src: The string to be searched
 * @substr: The string to search for
 * @src_len: ��src��ǰsrc_len�ַ��в���
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

  str = strnstr((char *)pdata, size, "+CSQ", 4);
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
uint8_t AT_RspAnalyze_ICCID(uint8_t *pdata, uint16_t size)
{
  /*
     * ������AT+CICCID��
        �ա���
       +ICCID: 898600700907A6019125
       OK
  */
  char *str;

  str = strnstr((char *)pdata, size, "+ICCID", 6);
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
     ������AT+CIMI��
     �ա���
     460045621801309
     OK
    */
  /*
     * IMSI��15λ��ʮ����������ṹ���£�MCC(3λ)+MNC(2λ)+MSIN(10λ)
     * MCC��Mobile Country Code���ƶ������룩,�й�Ϊ460��
     * MNC��Mobile Network Code���ƶ�������룩,
     * �й��ƶ�ʹ��00��02��04��07��
     * �й���ͨʹ��01��06��09��
     * �й�����ʹ��03��05��11��
     * �й���ͨʹ��20
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
  uint8_t retval;

  str = strnstr((char *)pdata, size, "+CREG", 5);
  if (str!=NULL)
  {
    str += 7; // ָ���һ������
    if(ATCMD_IS_CUR(AT_CMD_CREG_GET))  // AT��ѯ��Ӧ +CREG: <n>,<stat>[,[<lac>],[<ci>]
    {
      UNUSED(n);  // ���뾯��
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
    str += 8; // ָ���һ������
    UNUSED(n);  // ���뾯��
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
  // +CGPADDR: 1,��10.76.51.180��
  const char *str;
  uint8_t ip[4]; // IPv4 address
  uint8_t buf[40];

  str = strnstr((char *)pdata, size, "+CGPADDR", 8);
  if (str!=NULL)
  {
    str += 12;  // ָ�򶺺�

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

/* ================================================================== */
uint8_t AT_RspAnalyze_IPADDR(uint8_t *pdata, uint16_t size)
{
  // +CGPADDR: 1,��10.76.51.180��
  // +IPADDR: 10.71.155.118

  const char *str;
  uint8_t ip[4]; // IPv4 address
  uint8_t buf[40];

  str = strnstr((char *)pdata, size, "+IPADDR", 7);
  if (str!=NULL)
  {
    str += 7;  // ָ��ð��

    if (*str == ':'){
      str++;
    }
    if (*str == ' ') {
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
 * ����: 1-�ַ���ƥ��, 0-��ƥ��
******************************************************************************/
uint8_t AT_RspAnalyzeURC(uint8_t *pdata, uint16_t size, const char *rsp)
{
  const char *strx;
  uint8_t rsp_size;

  rsp_size = strlen(rsp);
  strx = strnstr((char *)pdata, size, rsp, rsp_size);
  if (strx != NULL)
    return 1;
  else
    return 0;
}

/******************************************************************************
 * ��ȡSMS���ݰ�������
 *******************************************************************************/
 #if 0
uint8_t AT_ParseSms(uint8_t *pdata, uint16_t size)
{

  return TRUE;
}
#endif

/******************************************************************************
 * ����CMGL
******************************************************************************/
uint8_t AT_RspAnalyze_CMGL(uint8_t *pdata, uint16_t size)
{
  const char *str1;
  uint8_t retVal = FALSE;

  str1 = strnstr((char *)pdata, size, "+CMGL:", 6);  // �������г����ж���Ϣ
  if (str1!=NULL)
  {
    //retVal = AT_ParseSms(pdata, size);
    return retVal;
  }

  str1 = strnstr((char *)pdata, size, "+CMT: ", 6); // �¶���Ϣ����
  if (str1!=NULL)
  {
    //retVal = AT_ParseSms(pdata, size);
    return retVal;
  }

  return retVal;
}

/******************************************************************************
 * ����: �����յ�Celluraģ�����Ӧ��URC
 * ����: pData-���ݰ�ָ��;  size-���ݰ�����
*******************************************************************************/
static void Cellura_ProcessRecvData(uint8_t *pdata, uint16_t size)
{
  uint8_t retval = 0;
  uint8_t urc_flag = 0;

  // +CGPSXD: 1  // GPS��λ
  if(pdata[0]=='$')
  {
    // GPS_Function(pucData, usLen);
    urc_flag = CELLULAR_TRUE;
  } 
  else if (AT_RspAnalyze_CREG(pdata, size)) // �����վID
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
    case AT_CMD_CPIN_GET:
      if (AT_RspAnalyzeURC(pdata, size, "+CPIN: READY"))
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
      if (AT_RspAnalyzeURC(pdata, size, "OK"))
      {
        atcmd_rsp_status = ATCMD_RSP_OK;
      }
      break;
    case AT_CMD_ATE0:
      if (AT_RspAnalyzeURC(pdata, size, "OK"))
      {
        atcmd_rsp_status = ATCMD_RSP_OK;
      }
      break;
    case AT_CMD_CGPS_SET:
      if (AT_RspAnalyzeURC(pdata, size, "OK"))
      {
        atcmd_rsp_status = ATCMD_RSP_OK;
      }
      break;
    case AT_CMD_CGPSXE_SET:
      if (AT_RspAnalyzeURC(pdata, size, "OK"))
      {
        atcmd_rsp_status = ATCMD_RSP_OK;
      }
      break;
    case AT_CMD_CGPSINFOCFG_SET:
      if (AT_RspAnalyzeURC(pdata, size, "OK"))
      {
        atcmd_rsp_status = ATCMD_RSP_OK;
      }
      break;
    case AT_CMD_CGMR:
      if (AT_RspAnalyzeURC(pdata, size, "OK"))
      {
        atcmd_rsp_status = ATCMD_RSP_OK;
      }
      break;
    case AT_CMD_CMEE_SET:
      if (AT_RspAnalyzeURC(pdata, size, "OK"))
      {
        atcmd_rsp_status = ATCMD_RSP_OK;
      }
      break;
    case AT_CMD_CMGF_SET:
      if (AT_RspAnalyzeURC(pdata, size, "OK"))
      {
        atcmd_rsp_status = ATCMD_RSP_OK;
      }
      break;
    case AT_CMD_CPMS_SET:
      if (AT_RspAnalyzeURC(pdata, size, "OK"))
      {
        atcmd_rsp_status = ATCMD_RSP_OK;
      }
      break;
    case AT_CMD_CNMI_SET:
      if (AT_RspAnalyzeURC(pdata, size, "OK"))
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
    case AT_CMD_CICCID_GET:
      retval = AT_RspAnalyze_ICCID(pdata, size);
      if (retval == 1)
      {
        atcmd_rsp_status = ATCMD_RSP_OK;
      }
      break;
    case AT_CMD_CNMP_SET:
      if (AT_RspAnalyzeURC(pdata, size, "OK"))
      {
        atcmd_rsp_status = ATCMD_RSP_OK;
      }
      break;
    case AT_CMD_CREG0_SET:
      if (AT_RspAnalyzeURC(pdata, size, "OK"))
      {
        atcmd_rsp_status = ATCMD_RSP_OK;
      }
      break;
    case AT_CMD_CREG2_SET:
      if (AT_RspAnalyzeURC(pdata, size, "OK"))
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
      if (AT_RspAnalyzeURC(pdata, size, "OK"))
      {
        atcmd_rsp_status = ATCMD_RSP_OK;
      }
      break;
    case AT_CMD_CGDCONT_SET:
      if (AT_RspAnalyzeURC(pdata, size, "OK"))
      {
        atcmd_rsp_status = ATCMD_RSP_OK;
      }
      break;
    case AT_CMD_CGACT_SET:
      if (AT_RspAnalyzeURC(pdata, size, "OK"))
      {
        modem_info.pdpdeact_cnt = 0;
        modem_info.pdp_actived_flag = 1;
        atcmd_rsp_status = ATCMD_RSP_OK;
      }
      break;
    case AT_CMD_CGPADDR_GET:  // ��ѯPDP��ַ
      if (AT_RspAnalyze_CGPADDR(pdata, size))
      {
        atcmd_rsp_status = ATCMD_RSP_OK;
      }
      break;
    case AT_CMD_CSCLK:
      if (AT_RspAnalyzeURC(pdata, size, "OK"))
      {
        atcmd_rsp_status = ATCMD_RSP_OK;
      }
      break;
    case AT_CMD_CWMAP0_SET:
      if (AT_RspAnalyzeURC(pdata, size, "OK"))
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
      if (AT_RspAnalyzeURC(pdata, size, "OK"))
      {
        atcmd_rsp_status = ATCMD_RSP_OK;
      }
      break;
    case AT_CMD_CMGS:
      if (AT_RspAnalyzeURC(pdata, size, ">"))
      {
        atcmd_rsp_status = ATCMD_RSP_OK;
      }
      break;
    case AT_CMD_CMGS_DATA:
      //if (AT_RspAnalyzeURC(pdata, size, ">"))
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
 * CAT1ģ�����ݽ��ս���
*******************************************************************************/
void* pthread_CelluraProcess(void *argument)
{
  uint8_t *pdata = NULL;
  uint16_t len;

  while (1)
  {
    if (CELLURA_Receive(&pdata, &len)) // �ȴ��ź���(����)
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
  Modem_SetState(MODEM_STATE_INIT);
  NetSocket_Init();
}

/*************************************************************************
 *
*************************************************************************/
void Cellura_ServiceInit(void)
{

  Modem_UartInitialize(MODEM_UART_BAUDRATE);
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
// �ⲿ��Ϣ�ӿ�
////////////////////////////////////////////////////////////////////////////////////////////
//==��ȡSIM����ICCID============================================================
uint8_t *Cellura_GetIccid(void)
{
  return modem_info.iccid;
}

//==��ȡ�ƶ����ź�ǿ��==========================================================
uint8_t Cellura_GetCsq(void)
{
  return modem_info.csq_rssi;
}

//==��ȡС��ID==================================================================
uint32 Cellura_GetLacCellID(void)
{
  return modem_info.cell_id;
}

//==��ȡģ�����״̬:1=����, 0=����=============================================
uint8_t Cellura_GetModemStatus(void)
{
  return modem_info.modem_err_flag;
}

//==��ȡSIM���Ƿ����: 1=����; 0,����===========================================
uint8_t Cellura_GetSimCardState(void)
{
  if(modem_info.sim_card_ok_flag)
    return 0;
  else
    return 1;
}

//==��ȡģ��CS��ע��״̬========================================================
uint8_t Cellura_GetCsRegistState(void)
{
  return modem_info.cs_network_regist_status;
}

//==��ȡģ��PS��ע��״̬========================================================
uint8_t Cellura_GetPsRegistState(void)
{
  return modem_info.ps_network_regist_status;
}

//-----�ļ�CelluraCore.c����---------------------------------------------
