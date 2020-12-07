/*
 * Copyright(c)2020, XXXXX��˾Ӳ���з���
 * All right reserved
 *
 * �ļ�����: McuChQiUpload.h
 * �汾��  : V1.0
 * �ļ���ʶ: 
 * �ļ�����: ���ļ�Ϊ���������������������ģ��Э����ͷ�ļ�
 *
 *-----------------------------------------------------------------------------
 * �޸ļ�¼
 *-----------------------------------------------------------------------------
 *
 * 2020-01-17, by lxf, �������ļ�
 *
 */

#ifndef _MCUCHQIUPLOAD_H
#define _MCUCHQIUPLOAD_H

//-----��������----------------------------------------------------------------

#define KCMCU_RePeat_MAXCOUNT              10
#define KCMUC_SingleCMD_TIMEOUT            10          //����CAN���ʱ 10s

typedef struct _STU_KCMCUDownload
{
    uint8 ucLoadStep;        /*0-Ĭ��״̬;
                               1-����������������-Update Start Command ;
                               2-���͹̼���������-Update Imfomation ;
                               3-���͹̼����ݰ�����-Update Data(1-50);
                               4-���Ͳ�����������-Update Imfomation;
                               5-���Ͳ������ݰ�����-Update Data(1-50);
                               6-�������-Data Complete Acknowledge */
    uint8 ucRcvPackflag;     //B0:0-����̼��ļ��հ�δ�ɹ�����У�����,1-�հ��ɹ�����У��ͨ��
                             //B1:0-��������ļ��հ�δ�ɹ�����У�����,1-�հ��ɹ�����У��ͨ��
  //  uint8 ucMcuRespflag;     //�������տ�����Ӧ���־0-δӦ��,1-Ӧ��ʧ��(��Ӧ��ʱ15����ط�)
    uint8 ucUpDeviceType;    //��Ҫ�������豸���� 0-�նˣ�1-������������2-�Ǳ�,4-���������
    uint8 ucMcuCmdTimerOut;  //�ն���MUCͨѶ�ǳ�ʱ������,���ʱ10S
    uint16 usMcuUPdateTimerOut;//���ն˿�ʼ��MCU������������ʼ��ʱ�� 200k=150S
	uint8 aSendbuff[512];    //��MCU����������ԭʼ�������
	uint16 usReadFlashSN;    //��ȡ�洢��ҳ���(�߼����)��0��ʼ
	uint16 usTotalPackets;   //��mcu���͵������ļ��ܰ���
	uint8 ucUploadResult;    //�������������0-ʧ�� 1-�ɹ�
	uint16 uscheck;
	uint32 uiProgramTotPackets;   //�����ܰ���
	uint32 uiParameterTotPackets; //�����ܰ���
	uint32 uiOffsetAddr;      //��������ȡ����ָ���ƫ����
	uint8 ucRepeatSendCount;     //�ط�������
    uint32 uiSoftwareWSize;       //��������ļ���С
	uint32 uiParamSize;           //��������ļ���С
	
}STUKCMCUDownload;


void KCMCU_RecVCan_ACK(uint32 uiID, uint8 *arr);
void KCMcuFirmwareTimerOut(void);

#endif
