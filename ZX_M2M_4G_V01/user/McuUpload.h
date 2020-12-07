/*
 * Copyright(c)2018, XXXXX��˾Ӳ���з���
 * All right reserved
 *
 * �ļ�����: McuUpload.h
 * �汾��  : V1.0
 * �ļ���ʶ: 
 * �ļ�����: ���ļ�ΪMcuUpload����ģ��Э����ͷ�ļ�
 *
 *-----------------------------------------------------------------------------
 * �޸ļ�¼
 *-----------------------------------------------------------------------------
 *
 * 2018-02-07, by lxf, �������ļ�
 *
 */

#ifndef _MCUUPLOAD_H
#define _MCUUPLOAD_H

//-----��������----------------------------------------------------------------

#define Mcu_UPdateTimerOut   240

typedef struct _MCUFirmware
{
    uint8 ucLoadStep;        //0-Ĭ��״̬;1-���Ϳ�����;2-���Ϳ�����;3-�������Ϳ��������;
                               //4-�յ�MCU�����������ؽ�����Ӧ��;5-�������
    uint8 ucRcvPackflag;     //0-�հ�δ�ɹ�����У�����,1-�հ��ɹ�����У��ͨ��
    uint8 ucMcuRespflag;     //�������տ�����Ӧ���־0-δӦ��,1-Ӧ��ʧ��(��Ӧ��ʱ15����ط�)
    uint8 ucUpDeviceType;    //��Ҫ�������豸���� 0-�նˣ�1-��������2-�Ǳ�
    uint8 ucMcuCmdTimerOut;  //�ն���MUCͨѶ�ǳ�ʱ������,���ʱ15S
    uint16 usMcuUPdateTimerOut;//���ն˿�ʼ��MCU������������ʼ��ʱ�� 200k=150S
	uint8 aSendbuff[300];    //��MCU����SDO���
	uint16 usReadFlashSN;    //��ȡ�洢��ҳ���(�߼����)��0��ʼ
	uint16 usTotalPackets;   //��mcu���͵������ļ��ܰ���
	uint8 ucUploadResult;    //�������������0-ʧ�� 1-�ɹ�
	uint16 uscheck;
	uint8 ucCANFrameFormat;  //���յ�MCU����֡��ʽ: 0-��׼֡,1-��չ֡
	
}STUMCUFirmware;

typedef struct _ExTMCUUpgrade
{
    uint8 ucUpgradeStep;     //�����ļ����ز���  0-����������;1-֪ͨExtMcu����;2-��������������
                             //3-�������Ӧ��,0xFF-�м䲽��(��ϣ���ط�?)
    uint16 ucTimer;           //���ж�ʱ��
    uint16 usRequestPacketSn; //�������������
    uint8 ucDev;             //�����豸���
    uint8 ucResult;          //�豸�������
    uint16 usTimeOut;        //�����ļ������ܳ�ʱʱ��

}STUExtMCUUpgrade;



void McuUploadSend(void);
void McuUpload_CreatePackage(void);
void McuFirmwareTimerOut(void);
void Mcu_FirmwareDownload_Function(void);
void Ext_Mcu_Upgrade_Function(void);
void Ext_Mcu_Timeout_Function(void);
void Ext_Mcu_RecvUartData(uint8* ptr, uint16 uslen);
#endif
