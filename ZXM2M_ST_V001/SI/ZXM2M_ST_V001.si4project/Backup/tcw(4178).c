/*****************************************************************************
* Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
* @FileName: tcw.c
* @Engineer: TenYan
* @Company:  �칤��Ϣ����Ӳ����
* @version   V1.0
* @Date:     2019-10-29
* @brief     �칤�������ػ�CANͨ��Э��(�ϳ�(CAN2)���³�(CAN1))
* @Device ID: 2181193799(ȫ����)  2181193800(��ʽKXCT)
******************************************************************************/

/******************************************************************************
* Includes
******************************************************************************/
#include "config.h"
#include "tcw.h"


/******************************************************************************
* Define
******************************************************************************/


/******************************************************************************
 *   Function prototypes
 ******************************************************************************/
void ZxSts_SetNumberFlag(zxsts_context_t* pThis);
void ZxSts_SetStartFlag(zxsts_context_t* pThis);
void ZxSts_SetStopFlag(zxsts_context_t* pThis);
void ZxSts_ResetCanMsgTimer(zxsts_context_t* pThis);

/******************************************************************************
* Data Types and Globals
******************************************************************************/
// TLV״̬��Ч��־
bittype2 zxinfo_tlv_flag;
bittype2 zxup_tlv_flag1;
bittype2 zxup_tlv_flag2;
bittype2 zxup_tlv_flag3;
bittype2 zxdown_tlv_flag1;
bittype2 zxdown_tlv_flag2;
bittype2 zxengine_tlv_flag;
bittype2 zxversion_tlv_flag;
bittype2 zxstatistics_tlv_flag;

// ���ݻ���
uint8_t zxinfo_buffer[SIZE_OF_ZXINFO_BUFFER]; /// ������Ϣ
uint8_t zxup_buffer[SIZE_OF_ZXUP_BUFFER]; /// �ϳ����ݻ���
uint8_t zxdown_buffer[SIZE_OF_ZXDOWN_BUFFER]; /// �³��������ݻ���
uint8_t zxengine_buffer[SIZE_OF_ZXENGINE_BUFFER]; /// �³����������ݻ���
uint8_t zxstatistics_buffer[SIZE_OF_ZXSTATISTICS_BUFFER]; ///ͳ�����ݻ���
uint8_t zxversion_buffer[SIZE_OF_ZXVERSION_BUFFER]; /// �汾��Ϣ����

zxsts_context_t zxsts_context[NUMBER_OF_ZXSTS_TYPES]; /// Ƶ��ͳ��
bittype2 zxsts_flag1,zxsts_flag2;

/*************************************************************************
 * ������յ���uCAN����(�ϳ�)
*************************************************************************/
uint8_t CAN_ProcessRecvUpMsg(uint32_t canId, uint8_t *pdata, uint8_t size)
{
  bittype temp_flag;
  uint8_t retval = 0x00;
  uint16_t torque_percent;
  //uint32_t tempVal;

  switch (canId) // �ϳ�ͨ��
  {
  //==TAG-A504�ɼ�Э����Ϣ==========================================================
  case 0x574:  // �ϳ���������˵��
   zxinfo_buffer_a504[ZXINFO_A504_POS1_ADDR] = pdata[0]; // ��������
   zxinfo_buffer_a504[ZXINFO_A504_POS2_ADDR] = pdata[1]; // �ϳ�CANЭ��
   break;

  //==A5A0====================================================================================
  case 0x565:
    zxup_buffer_a5a0[ZXUP_A5A0_POS1] = pdata[0]; // ����������
    zxup_buffer_a5a0[ZXUP_A5A0_POS1+1] = pdata[1];
    break;

  case 0x285:
    zxup_buffer_a5a0[ZXUP_A5A0_POS2] = pdata[1]; // ��������(ABCDEFGH)
    zxup_buffer_a5a0[ZXUP_A5A0_POS2+1] = pdata[3];
    zxup_buffer_a5a0[ZXUP_A5A0_POS2+2] = pdata[2];
    zxup_buffer_a5a0[ZXUP_A5A0_POS3] = pdata[0]; // ����
    zxup_buffer_a5a0[ZXUP_A5A0_POS6] = pdata[4]; // ���۳���
    zxup_buffer_a5a0[ZXUP_A5A0_POS6+1] = pdata[5];
    zxup_buffer_a5a0[ZXUP_A5A0_POS10] = pdata[6]; // ���۳���
    zxup_buffer_a5a0[ZXUP_A5A0_POS10+1] = pdata[7];
    tlv_a5a0_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x195:
    zxup_buffer_a5a0[ZXUP_A5A0_POS4] = pdata[4]; // ����ͷ���Ƕ�
    zxup_buffer_a5a0[ZXUP_A5A0_POS4+1] = pdata[5];
    zxup_buffer_a5a0[ZXUP_A5A0_POS5] = pdata[2]; // ���۸����Ƕ�
    zxup_buffer_a5a0[ZXUP_A5A0_POS5+1] = pdata[3];
    zxup_buffer_a5a0[ZXUP_A5A0_POS15] = pdata[6]; // ��ͷ�߶�
    zxup_buffer_a5a0[ZXUP_A5A0_POS15+1] = pdata[7];
    tlv_a5a0_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x183:
    memcpy(&zxup_buffer_a5a0[ZXUP_A5A0_POS8], pdata, 8); // �ڱ۰ٷֱ�
    tlv_a5a0_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x1D1:
    //==���ٳ���=============================================
    temp_flag.byte = pdata[0];
    if(temp_flag.b.bit0)  // ���ٳ���
    {  zxsts_lmi_wind_speed_flag = ZXSTS_TRUE;}
    else
    {  zxsts_lmi_wind_speed_flag = ZXSTS_FALSE;}
    ZxSts_ResetCanMsgTimer(&zxsts_context[ZXSTS_TYPE_FSCX]);
    //=======================================================
    retval = 0x01;
    break;

  case 0x283:
    zxup_buffer_a5a0[ZXUP_A5A0_POS9] = pdata[0]; // �ױ�������
    tlv_a5a0_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x295:
    zxup_buffer_a5a0[ZXUP_A5A0_POS11] = pdata[0]; // ���۽Ƕ�
    zxup_buffer_a5a0[ZXUP_A5A0_POS11+1] = pdata[1];
    zxup_buffer_a5a0[ZXUP_A5A0_POS12] = pdata[2]; // ����/���۸����Ƕ�
    zxup_buffer_a5a0[ZXUP_A5A0_POS12+1] = pdata[3];
    zxup_buffer_a5a0[ZXUP_A5A0_POS13] = pdata[4]; // ����/����ͷ���Ƕ�
    zxup_buffer_a5a0[ZXUP_A5A0_POS13+1] = pdata[5];
    zxup_buffer_a5a2[ZXUP_A5A2_POS1] = pdata[6]; // ����������
    zxup_buffer_a5a2[ZXUP_A5A2_POS1+1] = pdata[7];
    tlv_a5a0_valid_flag = 1;
    tlv_a5a2_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x1D5:
    memcpy(&zxup_buffer_a5a0[ZXUP_A5A0_POS14], pdata, 8); // ǻѹ��
    tlv_a5a0_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x185:
    //==Ƶ��ͳ��:����==========================
    torque_percent = ((uint16_t)(pdata[5]<<8) + pdata[4])/10;
    if(torque_percent > 100) // ����ֵ�ж�
    {  zxsts_overload_flag = ZXSTS_TRUE;}
    else
    {  zxsts_overload_flag = ZXSTS_FALSE;}
    ZxSts_ResetCanMsgTimer(&zxsts_context[ZXSTS_TYPE_LMI]);
    //=========================================
    
    zxup_buffer_a5a0[ZXUP_A5A0_POS16] = pdata[0]; // �����
    zxup_buffer_a5a0[ZXUP_A5A0_POS16+1] = pdata[1];
    zxup_buffer_a5a0[ZXUP_A5A0_POS17] = pdata[2]; // ʵ������
    zxup_buffer_a5a0[ZXUP_A5A0_POS17+1] = pdata[3];
    zxup_buffer_a5a0[ZXUP_A5A0_POS18] = pdata[4]; // ���ذٷֱ�
    zxup_buffer_a5a0[ZXUP_A5A0_POS18+1] = pdata[5];
    zxup_buffer_a5a0[ZXUP_A5A0_POS19] = pdata[6]; // ��������
    zxup_buffer_a5a0[ZXUP_A5A0_POS19+1] = pdata[7];
    tlv_a5a0_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x385:
    memcpy(&zxup_buffer_a5a0[ZXUP_A5A0_POS20], pdata, 8); // LMI���ϴ���1-4
    tlv_a5a0_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x395:
    memcpy(&zxup_buffer_a5a0[ZXUP_A5A0_POS21], pdata, 8); // LMI���ϴ���5-8
    tlv_a5a0_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x3F5:
    memcpy(&zxup_buffer_a5a0[ZXUP_A5A0_POS22], pdata, 4); // �ǶԳƹ��ϴ���
    tlv_a5a0_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x3E5:
    memcpy(&zxup_buffer_a5a0[ZXUP_A5A0_POS23], pdata, 4); // LMI����ʱ��
    tlv_a5a0_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x1C5:
    if (pdata[4]&BIT(0)) // �߶���λ
      zxup_buffer_a5a0[ZXUP_A5A0_POS24] |= BIT(0); // ��1
    else
      zxup_buffer_a5a0[ZXUP_A5A0_POS24] &= ~BIT(0); // ��0

    zxup_buffer_a5a2[ZXUP_A5A2_POS2] = pdata[0]; // ����������
    zxup_buffer_a5a2[ZXUP_A5A2_POS2+1] = pdata[1];
    tlv_a5a0_valid_flag = 1;
    tlv_a5a2_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x463:
    //==Ƶ��ͳ��:��װ���ء�����ǿ�ơ������ǿ�ơ���Ȧǿ�ơ���ǿ��====
    temp_flag.byte = pdata[2];
    if(temp_flag.b.bit0)  // ��װ����
    {  zxsts_setup_flag = ZXSTS_TRUE;}
    else
    {  zxsts_setup_flag = ZXSTS_FALSE;}
    ZxSts_ResetCanMsgTimer(&zxsts_context[ZXSTS_TYPE_CZKG]);

    if(temp_flag.b.bit1)  // ����ǿ��
    {  zxsts_a2b_force_flag = ZXSTS_TRUE;}
    else
    {  zxsts_a2b_force_flag = ZXSTS_FALSE;}
    ZxSts_ResetCanMsgTimer(&zxsts_context[ZXSTS_TYPE_GXQZ]);

    if(temp_flag.b.bit2)  // �����ǿ��
    {  zxsts_luff_up_force_flag = ZXSTS_TRUE;}
    else
    {  zxsts_luff_up_force_flag = ZXSTS_FALSE;}
    ZxSts_ResetCanMsgTimer(&zxsts_context[ZXSTS_TYPE_BFQQZ]);

    if(temp_flag.b.bit3)  // ��Ȧǿ��
    {  zxsts_od_force_flag = ZXSTS_TRUE;}
    else
    {  zxsts_od_force_flag = ZXSTS_FALSE;}
    ZxSts_ResetCanMsgTimer(&zxsts_context[ZXSTS_TYPE_SQQZ]);

    if(temp_flag.b.bit4)  // ��ǿ��
    {  zxsts_lmi_force_flag = ZXSTS_TRUE;}
    else
    {  zxsts_lmi_force_flag = ZXSTS_FALSE;}
    ZxSts_ResetCanMsgTimer(&zxsts_context[ZXSTS_TYPE_ZQZ]);
    //========================================================
    
    if (pdata[2]&BIT(4)) // LMIǿ��
      zxup_buffer_a5a0[ZXUP_A5A0_POS24] |= BIT(1); // ��1
    else
      zxup_buffer_a5a0[ZXUP_A5A0_POS24] &= ~BIT(1); // ��0

    if (pdata[2]&BIT(0)) // ��װ����
      zxup_buffer_a5a0[ZXUP_A5A0_POS24] |= BIT(2); // ��1
    else
      zxup_buffer_a5a0[ZXUP_A5A0_POS24] &= ~BIT(2); // ��0

    if (pdata[2]&BIT(2)) // �����ǿ��
      zxup_buffer_a5a0[ZXUP_A5A0_POS24] |= BIT(3); // ��1
    else
      zxup_buffer_a5a0[ZXUP_A5A0_POS24] &= ~BIT(3); // ��0

    if (pdata[2]&BIT(3)) // ����ǿ��
      zxup_buffer_a5a0[ZXUP_A5A0_POS24] |= BIT(4); // ��1
    else
      zxup_buffer_a5a0[ZXUP_A5A0_POS24] &= ~BIT(4); // ��0

    if (pdata[2]&BIT(1)) // �߶���λǿ��
      zxup_buffer_a5a0[ZXUP_A5A0_POS24] |= BIT(5); // ��1
    else
      zxup_buffer_a5a0[ZXUP_A5A0_POS24] &= ~BIT(5); // ��0

    if (pdata[7]&BIT(7)) // ������������
      zxup_buffer_a5b3[ZXUP_A5B3_POS3] |= BIT(0); // ��1
    else
      zxup_buffer_a5b3[ZXUP_A5B3_POS3] &= ~BIT(0); // ��0

    if (pdata[7]&BIT(6)) // ���Ͷ�������
      zxup_buffer_a5b3[ZXUP_A5B3_POS3] |= BIT(1); // ��1
    else
      zxup_buffer_a5b3[ZXUP_A5B3_POS3] &= ~BIT(1); // ��0

    tlv_a5b3_valid_flag = 1;
    tlv_a5a0_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x263:
    zxup_buffer_a5a0[ZXUP_A5A0_POS25] = pdata[0]; // ˮƽ��X
    zxup_buffer_a5a0[ZXUP_A5A0_POS25+1] = pdata[1];
    zxup_buffer_a5a0[ZXUP_A5A0_POS26] = pdata[2]; // ˮƽ��Y
    zxup_buffer_a5a0[ZXUP_A5A0_POS26+1] = pdata[3];
    zxup_buffer_a5a0[ZXUP_A5A0_POS27] = pdata[4]; // ����
    zxup_buffer_a5a0[ZXUP_A5A0_POS27+1] = pdata[5];
    zxup_buffer_a5a0[ZXUP_A5A0_POS28] = pdata[6]; // ��ת�Ƕ�
    zxup_buffer_a5a0[ZXUP_A5A0_POS28+1] = pdata[7];
    tlv_a5a0_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x203:
    zxup_buffer_a5a0[ZXUP_A5A0_POS29] = pdata[1]; // ǿ�����Ԯ
    tlv_a5a0_valid_flag = 1;
    retval = 0x01;
    break;


  //==A5A1====================================================================================
  case 0x1E3:
    zxup_buffer_a5a1[ZXUP_A5A1_POS1] = pdata[0]; // �������Ƕ�
    zxup_buffer_a5a1[ZXUP_A5A1_POS1+1] = pdata[1];
    zxup_buffer_a5a1[ZXUP_A5A1_POS5] = pdata[2]; // �ҳ������Ƕ�
    zxup_buffer_a5a1[ZXUP_A5A1_POS5+1] = pdata[3];
    zxup_buffer_a5a1[ZXUP_A5A1_POS3] = pdata[4]; // ����չ���Ƕ�
    zxup_buffer_a5a1[ZXUP_A5A1_POS3+1] = pdata[5];
    zxup_buffer_a5a1[ZXUP_A5A1_POS7] = pdata[6]; // �ҳ���չ���Ƕ�
    zxup_buffer_a5a1[ZXUP_A5A1_POS7+1] = pdata[7];
    tlv_a5a1_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x2E3:
    zxup_buffer_a5a1[ZXUP_A5A1_POS2] = pdata[0]; // ��������
    zxup_buffer_a5a1[ZXUP_A5A1_POS2+1] = pdata[1];
    zxup_buffer_a5a1[ZXUP_A5A1_POS6] = pdata[2]; // �ҳ�������
    zxup_buffer_a5a1[ZXUP_A5A1_POS6+1] = pdata[3];
    zxup_buffer_a5a1[ZXUP_A5A1_POS13] = pdata[4]; // �����Ž��׳���
    zxup_buffer_a5a1[ZXUP_A5A1_POS13+1] = pdata[5];
    zxup_buffer_a5a1[ZXUP_A5A1_POS14] = pdata[6]; // �ҳ����Ž��׳���
    zxup_buffer_a5a1[ZXUP_A5A1_POS14+1] = pdata[7];
    tlv_a5a1_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x4E3:
    zxup_buffer_a5a1[ZXUP_A5A1_POS4] = pdata[2]; // �������Ƕ�
    zxup_buffer_a5a1[ZXUP_A5A1_POS4+1] = pdata[3];
    zxup_buffer_a5a1[ZXUP_A5A1_POS8] = pdata[4]; // �ҳ������Ƕ�
    zxup_buffer_a5a1[ZXUP_A5A1_POS8+1] = pdata[5];
    zxup_buffer_a5b1[ZXUP_A5B1_POS3] = pdata[6]; // �����Ž��׳��� *********
    zxup_buffer_a5b1[ZXUP_A5B1_POS3+1] = pdata[7];
    tlv_a5a1_valid_flag = 1;
    tlv_a5b1_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x3E3:
    if (pdata[5]&BIT(0)) // ���������ֹ
      zxup_buffer_a5a1[ZXUP_A5A1_POS9] |= BIT(0); // ��1
    else
      zxup_buffer_a5a1[ZXUP_A5A1_POS9] &= ~BIT(0); // ��0

    if (pdata[5]&BIT(1)) // ����������
      zxup_buffer_a5a1[ZXUP_A5A1_POS9] |= BIT(1); // ��1
    else
      zxup_buffer_a5a1[ZXUP_A5A1_POS9] &= ~BIT(1); // ��0

    if (pdata[5]&BIT(4)) // ����������λ
      zxup_buffer_a5a1[ZXUP_A5A1_POS9] |= BIT(2); // ��1
    else
      zxup_buffer_a5a1[ZXUP_A5A1_POS9] &= ~BIT(2); // ��0

    if (pdata[6]&BIT(1)) // �����Ž��׽���
      zxup_buffer_a5a1[ZXUP_A5A1_POS9] |= BIT(5); // ��1
    else
      zxup_buffer_a5a1[ZXUP_A5A1_POS9] &= ~BIT(5); // ��0

    if (pdata[6]&BIT(0)) // �����Ž�����ֹ
      zxup_buffer_a5a1[ZXUP_A5A1_POS9] |= BIT(6); // ��1
    else
      zxup_buffer_a5a1[ZXUP_A5A1_POS9] &= ~BIT(6); // ��0

    if (pdata[6]&BIT(4)) // �����Ž���ȫ������
      zxup_buffer_a5a1[ZXUP_A5A1_POS9] |= BIT(7); // ��1
    else
      zxup_buffer_a5a1[ZXUP_A5A1_POS9] &= ~BIT(7); // ��0

    if (pdata[5]&BIT(2)) // �ҳ��������ֹ
      zxup_buffer_a5a1[ZXUP_A5A1_POS10] |= BIT(0); // ��1
    else
      zxup_buffer_a5a1[ZXUP_A5A1_POS10] &= ~BIT(0); // ��0

    if (pdata[5]&BIT(3)) // �ҳ���������
      zxup_buffer_a5a1[ZXUP_A5A1_POS10] |= BIT(1); // ��1
    else
      zxup_buffer_a5a1[ZXUP_A5A1_POS10] &= ~BIT(1); // ��0

    if (pdata[5]&BIT(5)) // �ҳ���������λ
      zxup_buffer_a5a1[ZXUP_A5A1_POS10] |= BIT(2); // ��1
    else
      zxup_buffer_a5a1[ZXUP_A5A1_POS10] &= ~BIT(2); // ��0

    if (pdata[6]&BIT(3)) // �ҳ����Ž��׽���
      zxup_buffer_a5a1[ZXUP_A5A1_POS10] |= BIT(5); // ��1
    else
      zxup_buffer_a5a1[ZXUP_A5A1_POS10] &= ~BIT(5); // ��0

    if (pdata[6]&BIT(2)) // �ҳ����Ž�����ֹ
      zxup_buffer_a5a1[ZXUP_A5A1_POS10] |= BIT(6); // ��1
    else
      zxup_buffer_a5a1[ZXUP_A5A1_POS10] &= ~BIT(6); // ��0

    if (pdata[6]&BIT(5)) // �ҳ����Ž���ȫ������
      zxup_buffer_a5a1[ZXUP_A5A1_POS10] |= BIT(7); // ��1
    else
      zxup_buffer_a5a1[ZXUP_A5A1_POS10] &= ~BIT(7); // ��0

    zxup_buffer_a5a1[ZXUP_A5A1_POS11] = pdata[0]; // �������
    zxup_buffer_a5a1[ZXUP_A5A1_POS12] = pdata[1]; // �ҳ������

    if (pdata[7]&BIT(2)) // ���Ž�������С*********
      zxup_buffer_a5b1[ZXUP_A5B1_POS4] |= BIT(0); // ��1
    else
      zxup_buffer_a5b1[ZXUP_A5B1_POS4] &= ~BIT(0); // ��0

    if (pdata[7]&BIT(3)) // ���Ž���������*********
      zxup_buffer_a5b1[ZXUP_A5B1_POS4] |= BIT(1); // ��1
    else
      zxup_buffer_a5b1[ZXUP_A5B1_POS4] &= ~BIT(1); // ��0

    if (pdata[7]&BIT(4)) // ���Ž�������С*********
      zxup_buffer_a5b1[ZXUP_A5B1_POS4] |= BIT(2); // ��1
    else
      zxup_buffer_a5b1[ZXUP_A5B1_POS4] &= ~BIT(2); // ��0

    if (pdata[7]&BIT(5)) // ���Ž���������*********
      zxup_buffer_a5b1[ZXUP_A5B1_POS4] |= BIT(3); // ��1
    else
      zxup_buffer_a5b1[ZXUP_A5B1_POS4] &= ~BIT(3); // ��0

    zxup_buffer_a5b1[ZXUP_A5B1_POS2] = pdata[2]; // ����Ŀ���Ž��Ƕ�*********
    zxup_buffer_a5b1[ZXUP_A5B1_POS2+1] = pdata[3];
    tlv_a5a1_valid_flag = 1;
    tlv_a5b1_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5A2====================================================================================
  case 0x3F3:
    zxup_buffer_a5a2[ZXUP_A5A2_POS3] = pdata[4]; // ������ѹ��
    zxup_buffer_a5a2[ZXUP_A5A2_POS3+1] = pdata[5];
    zxup_buffer_a5a2[ZXUP_A5A2_POS4] = pdata[4]; // ǰ֧�ܽǶ�
    zxup_buffer_a5a2[ZXUP_A5A2_POS4+1] = pdata[5];
    zxup_buffer_a5be[ZXUP_A5BE_POS2] = pdata[6]; // ��Ͳת��*****
    zxup_buffer_a5be[ZXUP_A5BE_POS2+1] = pdata[7];
    tlv_a5a2_valid_flag = 1;
    tlv_a5be_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x1F3:
    if (pdata[0]&BIT(3)) // �ҳ��������ֹ
      zxup_buffer_a5a2[ZXUP_A5A2_POS5] |= BIT(0); // ��1
    else
      zxup_buffer_a5a2[ZXUP_A5A2_POS5] &= ~BIT(0); // ��0

    if (pdata[2]&BIT(0)) // �ƶ���ŷ�
      zxup_buffer_a5be[ZXUP_A5BE_POS3] |= BIT(1); // ��1
    else
      zxup_buffer_a5be[ZXUP_A5BE_POS3] &= ~BIT(1); // ��0

    if (pdata[2]&BIT(1)) // ѹ���̵���
      zxup_buffer_a5be[ZXUP_A5BE_POS3] |= BIT(2); // ��1
    else
      zxup_buffer_a5be[ZXUP_A5BE_POS3] &= ~BIT(2); // ��0

    zxup_buffer_a5be[ZXUP_A5BE_POS1] = pdata[6]; // ������
    zxup_buffer_a5be[ZXUP_A5BE_POS1+1] = pdata[7];
    tlv_a5a2_valid_flag = 1;
    tlv_a5be_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5A5====================================================================================
  case 0x35A:
    zxup_buffer_a5a5[ZXUP_A5A5_POS1] = pdata[3]; // ������ת��
    zxup_buffer_a5a5[ZXUP_A5A5_POS1+1] = pdata[4];
    zxup_buffer_a5a5[ZXUP_A5A5_POS2] = pdata[2]; // ʵ��Ť�ذٷֱ�

    if (pdata[0]&BIT(0)) // ������Ť��ģʽ(����������)1
      zxup_buffer_a5a5[ZXUP_A5A5_POS16] |= BIT(0); // ��1
    else
      zxup_buffer_a5a5[ZXUP_A5A5_POS16] &= ~BIT(0); // ��0
      
    if (pdata[0]&BIT(1)) // ������Ť��ģʽ(����������)2
      zxup_buffer_a5a5[ZXUP_A5A5_POS16] |= BIT(1); // ��1
    else
      zxup_buffer_a5a5[ZXUP_A5A5_POS16] &= ~BIT(1); // ��0

    if (pdata[0]&BIT(2)) // ������Ť��ģʽ(����������)3
      zxup_buffer_a5a5[ZXUP_A5A5_POS16] |= BIT(2); // ��1
    else
      zxup_buffer_a5a5[ZXUP_A5A5_POS16] &= ~BIT(2); // ��0

    if (pdata[0]&BIT(3)) // ������Ť��ģʽ(����������)4
      zxup_buffer_a5a5[ZXUP_A5A5_POS16] |= BIT(3); // ��1
    else
      zxup_buffer_a5a5[ZXUP_A5A5_POS16] &= ~BIT(3); // ��0

    tlv_a5a5_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x35B:
    zxup_buffer_a5a5[ZXUP_A5A5_POS3] = pdata[0]; // Ħ��Ť�ذٷֱ�
    tlv_a5a5_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x35C:
    zxup_buffer_a5a5[ZXUP_A5A5_POS4] = pdata[1]; // �������ѹ��
    zxup_buffer_a5a5[ZXUP_A5A5_POS5] = pdata[2]; // ��������¶�
    tlv_a5a5_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x35D:
    zxup_buffer_a5a5[ZXUP_A5A5_POS6] = pdata[0]; // ��ȴҺ�¶�
    zxup_buffer_a5a5[ZXUP_A5A5_POS7] = pdata[2]; // �����¶�
    zxup_buffer_a5a5[ZXUP_A5A5_POS7+1] = pdata[3];
    tlv_a5a5_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x35E:
    if (pdata[0]&BIT(0)) // ������ˮָʾ��1
      zxup_buffer_a5a5[ZXUP_A5A5_POS11] |= BIT(0); // ��1
    else
      zxup_buffer_a5a5[ZXUP_A5A5_POS11] &= ~BIT(0); // ��0

    if (pdata[0]&BIT(1)) // ������ˮָʾ��2
      zxup_buffer_a5a5[ZXUP_A5A5_POS11] |= BIT(1); // ��1
    else
      zxup_buffer_a5a5[ZXUP_A5A5_POS11] &= ~BIT(1); // ��0

    zxup_buffer_a5a5[ZXUP_A5A5_POS8] = pdata[2]; // ����Һλ
    zxup_buffer_a5a5[ZXUP_A5A5_POS9] = pdata[3]; // ����ѹ��
    tlv_a5a5_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x35F:
    zxup_buffer_a5a5[ZXUP_A5A5_POS10] = pdata[0]; // ������������ʱ��
    zxup_buffer_a5a5[ZXUP_A5A5_POS10+1] = pdata[1];
    zxup_buffer_a5a5[ZXUP_A5A5_POS10+2] = pdata[2];
    zxup_buffer_a5a5[ZXUP_A5A5_POS10+3] = pdata[3];
    tlv_a5a5_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x36A:
    zxup_buffer_a5a5[ZXUP_A5A5_POS12] = pdata[1]; // ����̤��ٷֱ�
    tlv_a5a5_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x36B:
    zxup_buffer_a5a5[ZXUP_A5A5_POS13] = pdata[0]; // ������ȼ��������
    zxup_buffer_a5a5[ZXUP_A5A5_POS13+1] = pdata[1];
    zxup_buffer_a5a5[ZXUP_A5A5_POS14] = pdata[4]; // ������ƽ��ȼ��������
    zxup_buffer_a5a5[ZXUP_A5A5_POS14+1] = pdata[5];
    tlv_a5a5_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x36C:
    zxup_buffer_a5a5[ZXUP_A5A5_POS15] = pdata[1]; // ȼ��Һλ
    tlv_a5a5_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5A6====================================================================================
  case 0x464:
    //==��ҵͳ��:������������䡢�����ͱ���䡢������͸����䡢���ת���һ�ת==
    temp_flag.byte = pdata[3];
    if(temp_flag.b.bit0)  // ���������
    {  zxsts_main_hoist_up_flag = ZXSTS_TRUE;}
    else
    {  zxsts_main_hoist_up_flag = ZXSTS_FALSE;}

    if(temp_flag.b.bit1)  // ���������
    {  zxsts_main_hoist_down_flag = ZXSTS_TRUE;}
    else
    {  zxsts_main_hoist_down_flag = ZXSTS_FALSE;}
    ZxSts_ResetCanMsgTimer(&zxsts_context[ZXSTS_TYPE_ZJUP]);
    ZxSts_ResetCanMsgTimer(&zxsts_context[ZXSTS_TYPE_ZJDW]);

    if(temp_flag.b.bit2)  // ���ת����
    {  zxsts_slew_left_flag = ZXSTS_TRUE;}
    else
    {  zxsts_slew_left_flag = ZXSTS_FALSE;}

    if(temp_flag.b.bit3)  // �һ�ת����
    {  zxsts_slew_right_flag = ZXSTS_TRUE;}
    else
    {  zxsts_slew_right_flag = ZXSTS_FALSE;}
    ZxSts_ResetCanMsgTimer(&zxsts_context[ZXSTS_TYPE_ZHR]);
    ZxSts_ResetCanMsgTimer(&zxsts_context[ZXSTS_TYPE_YHR]);

    if(temp_flag.b.bit4)  // ��������
    {  zxsts_luff_up_flag = ZXSTS_TRUE;}
    else
    {  zxsts_luff_up_flag = ZXSTS_FALSE;}

    if(temp_flag.b.bit5)  // ��������
    {  zxsts_luff_down_flag = ZXSTS_TRUE;}
    else
    {  zxsts_luff_down_flag = ZXSTS_FALSE;}
    ZxSts_ResetCanMsgTimer(&zxsts_context[ZXSTS_TYPE_BFUP]);
    ZxSts_ResetCanMsgTimer(&zxsts_context[ZXSTS_TYPE_BFDW]);

    if(temp_flag.b.bit6)  // ���������
    {  zxsts_deputy_hoist_up_flag = ZXSTS_TRUE;}
    else
    {  zxsts_deputy_hoist_up_flag = ZXSTS_FALSE;}

    if(temp_flag.b.bit7)  // ���������
    {  zxsts_deputy_hoist_down_flag = ZXSTS_TRUE;}
    else
    {  zxsts_deputy_hoist_down_flag = ZXSTS_FALSE;}
    ZxSts_ResetCanMsgTimer(&zxsts_context[ZXSTS_TYPE_FJUP]);
    ZxSts_ResetCanMsgTimer(&zxsts_context[ZXSTS_TYPE_FJDW]);
    //=========================================================

    if (pdata[1]&BIT(0)) // ���ֱ���λ
      zxup_buffer_a5a6[ZXUP_A5A6_POS1] |= BIT(0); // ��1
    else
      zxup_buffer_a5a6[ZXUP_A5A6_POS1] &= ~BIT(0); // ��0

    if (pdata[1]&BIT(1)) // ���ֱ��󿪹�
      zxup_buffer_a5a6[ZXUP_A5A6_POS1] |= BIT(5); // ��1
    else
      zxup_buffer_a5a6[ZXUP_A5A6_POS1] &= ~BIT(5); // ��0

    if (pdata[1]&BIT(2)) // ���ֱ��ҿ���
      zxup_buffer_a5a6[ZXUP_A5A6_POS1] |= BIT(6); // ��1
    else
      zxup_buffer_a5a6[ZXUP_A5A6_POS1] &= ~BIT(6); // ��0

    if (pdata[1]&BIT(3)) // ���ֱ��ȵ�����
      zxup_buffer_a5a6[ZXUP_A5A6_POS1] |= BIT(7); // ��1
    else
      zxup_buffer_a5a6[ZXUP_A5A6_POS1] &= ~BIT(7); // ��0

    if (pdata[1]&BIT(4)) // ���ֱ���λ
      zxup_buffer_a5a6[ZXUP_A5A6_POS4] |= BIT(0); // ��1
    else
      zxup_buffer_a5a6[ZXUP_A5A6_POS4] &= ~BIT(0); // ��0

    if (pdata[1]&BIT(5)) // ���ֱ��󿪹�
      zxup_buffer_a5a6[ZXUP_A5A6_POS4] |= BIT(5); // ��1
    else
      zxup_buffer_a5a6[ZXUP_A5A6_POS4] &= ~BIT(5); // ��0

    if (pdata[1]&BIT(6)) // ���ֱ��ҿ���
      zxup_buffer_a5a6[ZXUP_A5A6_POS4] |= BIT(6); // ��1
    else
      zxup_buffer_a5a6[ZXUP_A5A6_POS4] &= ~BIT(6); // ��0

    if (pdata[1]&BIT(7)) // ���ֱ��ȵ�����
      zxup_buffer_a5a6[ZXUP_A5A6_POS4] |= BIT(7); // ��1
    else
      zxup_buffer_a5a6[ZXUP_A5A6_POS4] &= ~BIT(7); // ��0

    zxup_buffer_a5b3[ZXUP_A5B3_POS2] = pdata[2];
    tlv_a5a6_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x4E4:
    if (pdata[0]&BIT(4)) // ���ֱ�X����
      zxup_buffer_a5a6[ZXUP_A5A6_POS1] |= BIT(1); // ��1
    else
      zxup_buffer_a5a6[ZXUP_A5A6_POS1] &= ~BIT(1); // ��0

    if (pdata[0]&BIT(5)) // ���ֱ�X����
      zxup_buffer_a5a6[ZXUP_A5A6_POS1] |= BIT(2); // ��1
    else
      zxup_buffer_a5a6[ZXUP_A5A6_POS1] &= ~BIT(2); // ��0

    if (pdata[0]&BIT(6)) // ���ֱ�Y����
      zxup_buffer_a5a6[ZXUP_A5A6_POS1] |= BIT(3); // ��1
    else
      zxup_buffer_a5a6[ZXUP_A5A6_POS1] &= ~BIT(3); // ��0

    if (pdata[0]&BIT(7)) // ���ֱ�Y����
      zxup_buffer_a5a6[ZXUP_A5A6_POS1] |= BIT(4); // ��1
    else
      zxup_buffer_a5a6[ZXUP_A5A6_POS1] &= ~BIT(4); // ��0

    if (pdata[0]&BIT(0)) // ���ֱ�X����
      zxup_buffer_a5a6[ZXUP_A5A6_POS4] |= BIT(1); // ��1
    else
      zxup_buffer_a5a6[ZXUP_A5A6_POS4] &= ~BIT(1); // ��0

    if (pdata[0]&BIT(1)) // ���ֱ�X����
      zxup_buffer_a5a6[ZXUP_A5A6_POS4] |= BIT(2); // ��1
    else
      zxup_buffer_a5a6[ZXUP_A5A6_POS4] &= ~BIT(2); // ��0

    if (pdata[0]&BIT(2)) // ���ֱ�Y����
      zxup_buffer_a5a6[ZXUP_A5A6_POS4] |= BIT(3); // ��1
    else
      zxup_buffer_a5a6[ZXUP_A5A6_POS4] &= ~BIT(3); // ��0

    if (pdata[0]&BIT(3)) // ���ֱ�Y����
      zxup_buffer_a5a6[ZXUP_A5A6_POS4] |= BIT(4); // ��1
    else
      zxup_buffer_a5a6[ZXUP_A5A6_POS4] &= ~BIT(4); // ��0

    tlv_a5a6_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x363:
    zxup_buffer_a5a6[ZXUP_A5A6_POS2] = pdata[4]; // ���ֱ�X���
    zxup_buffer_a5a6[ZXUP_A5A6_POS2+1] = pdata[5];
    zxup_buffer_a5a6[ZXUP_A5A6_POS3] = pdata[6]; // ���ֱ�Y���
    zxup_buffer_a5a6[ZXUP_A5A6_POS3+1] = pdata[7];
    zxup_buffer_a5a6[ZXUP_A5A6_POS5] = pdata[0]; // ���ֱ�X���
    zxup_buffer_a5a6[ZXUP_A5A6_POS5+1] = pdata[1];
    zxup_buffer_a5a6[ZXUP_A5A6_POS6] = pdata[2]; // ���ֱ�Y���
    zxup_buffer_a5a6[ZXUP_A5A6_POS6+1] = pdata[3];
    tlv_a5a6_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5A7====================================================================================
  case 0x561:
    zxup_buffer_a5a7[ZXUP_A5A7_POS1] = pdata[0]; // ��Ʒ����(1-4)
    zxup_buffer_a5a7[ZXUP_A5A7_POS1+1] = pdata[1];
    zxup_buffer_a5a7[ZXUP_A5A7_POS1+2] = pdata[2];
    zxup_buffer_a5a7[ZXUP_A5A7_POS1+3] = pdata[3];
    zxversion_buffer_a505[ZXVERSION_A505_POS3] = pdata[4];  // ��ʾ���ײ�汾
    zxversion_buffer_a505[ZXVERSION_A505_POS3+1] = pdata[5];
    zxversion_buffer_a505[ZXVERSION_A505_POS3+2] = pdata[6];
    tlv_a505_valid_flag = 1;
    tlv_a5a7_valid_flag = 1;
    retval = 0x01;
    break;
  
  case 0x181:
    zxup_buffer_a5a7[ZXUP_A5A7_POS2] = pdata[1]; // ��������(ABCDEF)
    zxup_buffer_a5a7[ZXUP_A5A7_POS2+1] = pdata[3];
    zxup_buffer_a5a7[ZXUP_A5A7_POS2+2] = pdata[2];
    zxup_buffer_a5a7[ZXUP_A5A7_POS11] = pdata[4]; // Ŀ�����
    zxup_buffer_a5a7[ZXUP_A5A7_POS14] = pdata[5]; // ����ģʽ
    tlv_a5a7_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x3A1:
    zxup_buffer_a5a7[ZXUP_A5A7_POS3] = pdata[0]; // �������ٶ�
    zxup_buffer_a5a7[ZXUP_A5A7_POS4] = pdata[6]; // �������ٶ�
    zxup_buffer_a5a7[ZXUP_A5A7_POS5] = pdata[4]; // �������ٶ�
    zxup_buffer_a5a7[ZXUP_A5A7_POS6] = pdata[5]; // �������ٶ�
    zxup_buffer_a5a7[ZXUP_A5A7_POS7] = pdata[2]; // ������ٶ�
    zxup_buffer_a5a7[ZXUP_A5A7_POS8] = pdata[3]; // ������ٶ�
    zxup_buffer_a5a7[ZXUP_A5A7_POS9] = pdata[1]; // ���ת�ٶ�
    zxup_buffer_a5a7[ZXUP_A5A7_POS10] = pdata[7]; // �һ�ת�ٶ�
    tlv_a5a7_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x191:
    if (pdata[0]&BIT(0)) // �����涯1
      zxup_buffer_a5a7[ZXUP_A5A7_POS12] |= BIT(1); // ��1
    else
      zxup_buffer_a5a7[ZXUP_A5A7_POS12] &= ~BIT(1); // ��0

    if (pdata[0]&BIT(1)) // �����涯2
      zxup_buffer_a5a7[ZXUP_A5A7_POS12] |= BIT(2); // ��1
    else
      zxup_buffer_a5a7[ZXUP_A5A7_POS12] &= ~BIT(2); // ��0

    if (pdata[0]&BIT(4)) // ��/�����涯
      zxup_buffer_a5a7[ZXUP_A5A7_POS12] |= BIT(3); // ��1
    else
      zxup_buffer_a5a7[ZXUP_A5A7_POS12] &= ~BIT(3); // ��0

    if (pdata[0]&BIT(3)) // �������
      zxup_buffer_a5a7[ZXUP_A5A7_POS12] |= BIT(4); // ��1
    else
      zxup_buffer_a5a7[ZXUP_A5A7_POS12] &= ~BIT(4); // ��0

    tlv_a5a7_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x482:
    if (pdata[2]&BIT(0)) // ȡ��ʹ��
      zxup_buffer_a5a7[ZXUP_A5A7_POS12] |= BIT(0); // ��1
    else
      zxup_buffer_a5a7[ZXUP_A5A7_POS12] &= ~BIT(0); // ��0

    tlv_a5a7_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x291:
    if (pdata[0]&BIT(0)) // ��ת����
      zxup_buffer_a5a7[ZXUP_A5A7_POS12] |= BIT(5); // ��1
    else
      zxup_buffer_a5a7[ZXUP_A5A7_POS12] &= ~BIT(5); // ��0

    tlv_a5a7_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x381:
    if (pdata[0]&BIT(0)) // ǿ��1
      zxup_buffer_a5a7[ZXUP_A5A7_POS13] |= BIT(3); // ��1
    else
      zxup_buffer_a5a7[ZXUP_A5A7_POS13] &= ~BIT(3); // ��0

    if (pdata[0]&BIT(1)) // ǿ��2
      zxup_buffer_a5a7[ZXUP_A5A7_POS13] |= BIT(4); // ��1
    else
      zxup_buffer_a5a7[ZXUP_A5A7_POS13] &= ~BIT(4); // ��0

    if (pdata[0]&BIT(2)) // ���Ͼ�Ԯ
      zxup_buffer_a5a7[ZXUP_A5A7_POS13] |= BIT(5); // ��1
    else
      zxup_buffer_a5a7[ZXUP_A5A7_POS13] &= ~BIT(5); // ��0
    
    if (pdata[3]&BIT(0)) // ���۲�װ����1
      zxup_buffer_a5a7[ZXUP_A5A7_POS12] |= BIT(6); // ��1
    else
      zxup_buffer_a5a7[ZXUP_A5A7_POS12] &= ~BIT(6); // ��0

    if (pdata[3]&BIT(1)) // ���۲�װ����2
      zxup_buffer_a5a7[ZXUP_A5A7_POS12] |= BIT(7); // ��1
    else
      zxup_buffer_a5a7[ZXUP_A5A7_POS12] &= ~BIT(7); // ��0

    if (pdata[3]&BIT(2)) // ���۹����任1
      zxup_buffer_a5a7[ZXUP_A5A7_POS13] |= BIT(0); // ��1
    else
      zxup_buffer_a5a7[ZXUP_A5A7_POS13] &= ~BIT(0); // ��0

    if (pdata[3]&BIT(3)) // ���۹����任2
      zxup_buffer_a5a7[ZXUP_A5A7_POS13] |= BIT(1); // ��1
    else
      zxup_buffer_a5a7[ZXUP_A5A7_POS13] &= ~BIT(1); // ��0

    tlv_a5a7_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x391:
    if (pdata[0]&BIT(0)) // �����Զ�
      zxup_buffer_a5a7[ZXUP_A5A7_POS13] |= BIT(2); // ��1
    else
      zxup_buffer_a5a7[ZXUP_A5A7_POS13] &= ~BIT(2); // ��0

    tlv_a5a7_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5A8====================================================================================
  case 0x382:
    memcpy(&zxup_buffer_a5a8[ZXUP_A5A8_POS1], pdata, 8); // ����ٿ�
    zxup_buffer_a5b1[ZXUP_A5B1_POS1] = pdata[3]; // ����Ŀ��չ���Ƕ�
    tlv_a5a8_valid_flag = 1;
    tlv_a5b1_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x4C1:
    zxup_buffer_a5a8[ZXUP_A5A8_POS2] = pdata[0]; // ����ά��1+2
    zxup_buffer_a5a8[ZXUP_A5A8_POS2+1] = pdata[1];
    tlv_a5a8_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5A9====================================================================================
  case 0x1B1:
    zxup_buffer_a5a9[ZXUP_A5A9_POS1] = pdata[0]; // ���1
    zxup_buffer_a5a9[ZXUP_A5A9_POS1+1] = pdata[1];
    tlv_a5a9_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x1B2:
    zxup_buffer_a5a9[ZXUP_A5A9_POS2] = pdata[0]; // ���2
    zxup_buffer_a5a9[ZXUP_A5A9_POS2+1] = pdata[1];
    tlv_a5a9_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x1B3:
    zxup_buffer_a5a9[ZXUP_A5A9_POS3] = pdata[0]; // ���3
    zxup_buffer_a5a9[ZXUP_A5A9_POS3+1] = pdata[1];
    tlv_a5a9_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5AA====================================================================================
  case 0x188:
    memcpy(&zxup_buffer_a5aa[ZXUP_A5AA_POS1], pdata, 8); // Msg1
    tlv_a5aa_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x288:
    memcpy(&zxup_buffer_a5aa[ZXUP_A5AA_POS2], pdata, 8); // Msg2
    tlv_a5aa_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x388:
    memcpy(&zxup_buffer_a5aa[ZXUP_A5AA_POS3], pdata, 8); // Msg3
    tlv_a5aa_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x488:
    memcpy(&zxup_buffer_a5aa[ZXUP_A5AA_POS4], pdata, 8); // Msg4
    tlv_a5aa_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5AB====================================================================================
  case 0x343:
    memcpy(&zxup_buffer_a5ab[ZXUP_A5AB_POS1], pdata, 8); // �ڵ�״̬
    tlv_a5ab_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5AC====================================================================================
  case 0x493:
    memcpy(&zxup_buffer_a5ac[ZXUP_A5AC_POS1], pdata, 6); // �������ƺͽ��
    zxup_buffer_a5b2[ZXUP_A5B2_POS3] = pdata[6]; // �����������***********
    zxup_buffer_a5b2[ZXUP_A5B2_POS4] = pdata[7]; // ������������***********
    tlv_a5ac_valid_flag = 1;
    tlv_a5b2_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5AD====================================================================================
  case 0x4C3:
    memcpy(&zxup_buffer_a5ad[ZXUP_A5AD_POS1], pdata, 8); // �������ƺͽ��
    zxup_buffer_a5b2[ZXUP_A5B2_POS5] = pdata[6]; // �������Ʊ����***********
    zxup_buffer_a5b2[ZXUP_A5B2_POS6] = pdata[7]; // �������Ʊ����***********
    tlv_a5ad_valid_flag = 1;
    tlv_a5b2_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5AE====================================================================================
  case 0x4B3:
    zxup_buffer_a5ae[ZXUP_A5AE_POS1] = pdata[0]; // �������1
    zxup_buffer_a5ae[ZXUP_A5AE_POS2] = pdata[4]; // �������2
    zxup_buffer_a5ae[ZXUP_A5AE_POS3] = pdata[1]; // ��ؽ��
    zxup_buffer_a5ae[ZXUP_A5AE_POS4] = pdata[2]; // �һ�����1
    zxup_buffer_a5ae[ZXUP_A5AE_POS5] = pdata[5]; // �һ�����2
    zxup_buffer_a5ae[ZXUP_A5AE_POS6] = pdata[3]; // �һؽ��
    tlv_a5ae_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5AF====================================================================================
  case 0x4A3:
    memcpy(&zxup_buffer_a5af[ZXUP_A5AF_POS1], pdata, 5); // �������������ƺͽ��
    zxup_buffer_a5b2[ZXUP_A5B2_POS7] = pdata[5]; // ��������������***********
    zxup_buffer_a5b2[ZXUP_A5B2_POS8] = pdata[6]; // ��������������***********
    tlv_a5af_valid_flag = 1;
    tlv_a5b2_valid_flag = 1;
    retval = 0x01;
    break;

    //==A5B0====================================================================================
  case 0x4D3:
    memcpy(&zxup_buffer_a5b0[ZXUP_A5B0_POS1], &pdata[3], 4); // �������������ƺͽ��
    tlv_a5b0_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5B1====================================================================================
  // zxup_buffer

  //==A5B2====================================================================================
  case 0x4F3:
    zxup_buffer_a5b2[ZXUP_A5B2_POS1] = pdata[0]; // ����������
    zxup_buffer_a5b2[ZXUP_A5B2_POS1+1] = pdata[1];
    zxup_buffer_a5b2[ZXUP_A5B2_POS2] = pdata[2]; // ����������
    zxup_buffer_a5b2[ZXUP_A5B2_POS2+1] = pdata[3];
    tlv_a5b2_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5B3====================================================================================
  case 0x564:
    zxup_buffer_a5b3[ZXUP_A5B3_POS1] = pdata[0]; // Һѹ���¶�
    zxup_buffer_a5b3[ZXUP_A5B3_POS1+1] = pdata[1];
    zxup_buffer_a5b5[ZXUP_A5B5_POS10] = pdata[4]; // LS1ѹ��*****
    zxup_buffer_a5b5[ZXUP_A5B5_POS10+1] = pdata[5];
    zxup_buffer_a5b5[ZXUP_A5B5_POS12] = pdata[6]; // LS2ѹ��*****
    zxup_buffer_a5b5[ZXUP_A5B5_POS12+1] = pdata[7];
    tlv_a5b3_valid_flag = 1;
    tlv_a5b5_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5B4====================================================================================
  case 0x1A3:
    zxup_buffer_a5b4[ZXUP_A5B4_POS1] = pdata[0]; // 1#���õ�ŷ�
    zxup_buffer_a5b4[ZXUP_A5B4_POS1+1] = pdata[1];
    zxup_buffer_a5b4[ZXUP_A5B4_POS2] = pdata[2]; // 2#���õ�ŷ�
    zxup_buffer_a5b4[ZXUP_A5B4_POS2+1] = pdata[3];

    zxup_buffer_a5b5[ZXUP_A5B5_POS5] = pdata[0]; // �������ŷ�*****
    zxup_buffer_a5b5[ZXUP_A5B5_POS5+1] = pdata[1];
    zxup_buffer_a5b5[ZXUP_A5B5_POS6] = pdata[2]; // �������ŷ�*****
    zxup_buffer_a5b5[ZXUP_A5B5_POS6+1] = pdata[3];

    zxup_buffer_a5bb[ZXUP_A5BB_POS1] = pdata[0]; // ������ŷ�*****
    zxup_buffer_a5bb[ZXUP_A5BB_POS1+1] = pdata[1];
    zxup_buffer_a5bb[ZXUP_A5BB_POS2] = pdata[2]; // �����ŷ�*****
    zxup_buffer_a5bb[ZXUP_A5BB_POS2+1] = pdata[3];

    zxup_buffer_a5bc[ZXUP_A5BC_POS1] = pdata[4]; // ������
    zxup_buffer_a5bc[ZXUP_A5BC_POS1+1] = pdata[5];
    zxup_buffer_a5bc[ZXUP_A5BC_POS2] = pdata[6]; // ��Ͳת��
    zxup_buffer_a5bc[ZXUP_A5BC_POS2+1] = pdata[7];
    tlv_a5b4_valid_flag = 1;
    tlv_a5b5_valid_flag = 1;
    tlv_a5bb_valid_flag = 1;
    tlv_a5bc_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x2A3:
    zxup_buffer_a5b4[ZXUP_A5B4_POS3] = pdata[0]; // ����ѹ��
    zxup_buffer_a5b4[ZXUP_A5B4_POS3+1] = pdata[1];

    zxup_buffer_a5b5[ZXUP_A5B5_POS9] = pdata[0]; // MP1ѹ��*****
    zxup_buffer_a5b5[ZXUP_A5B5_POS9+1] = pdata[1];

    zxup_buffer_a5bb[ZXUP_A5BB_POS3] = pdata[0]; // �ͱ�ѹ��*****
    zxup_buffer_a5bb[ZXUP_A5BB_POS3+1] = pdata[1];

    if (pdata[6]&BIT(0)) // �ƶ���ŷ�*****
      zxup_buffer_a5bc[ZXUP_A5BC_POS3] |= BIT(1); // ��1
    else
      zxup_buffer_a5bc[ZXUP_A5BC_POS3] &= ~BIT(1); // ��0

    if (pdata[6]&BIT(1)) // ѹ���̵���*****
      zxup_buffer_a5bc[ZXUP_A5BC_POS3] |= BIT(2); // ��1
    else
      zxup_buffer_a5bc[ZXUP_A5BC_POS3] &= ~BIT(2); // ��0

    tlv_a5b4_valid_flag = 1;
    tlv_a5b5_valid_flag = 1;
    tlv_a5bb_valid_flag = 1;
    tlv_a5bc_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5B5====================================================================================
  case 0x393:
    zxup_buffer_a5b5[ZXUP_A5B5_POS1] = pdata[0]; // ���ŷ�
    zxup_buffer_a5b5[ZXUP_A5B5_POS1+1] = pdata[1];
    zxup_buffer_a5b5[ZXUP_A5B5_POS2] = pdata[2]; // ����ŷ�
    zxup_buffer_a5b5[ZXUP_A5B5_POS2+1] = pdata[3];
    zxup_buffer_a5b5[ZXUP_A5B5_POS11] = pdata[4]; // MP2ѹ��
    zxup_buffer_a5b5[ZXUP_A5B5_POS11+1] = pdata[5];
    zxup_buffer_a5b7[ZXUP_A5B7_POS1] = pdata[6]; // ������ѹ��*******
    zxup_buffer_a5b7[ZXUP_A5B7_POS1+1] = pdata[7];
    tlv_a5b5_valid_flag = 1;
    tlv_a5b7_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x1C3:
    zxup_buffer_a5b5[ZXUP_A5B5_POS3] = pdata[0]; // ������ŷ�
    zxup_buffer_a5b5[ZXUP_A5B5_POS3+1] = pdata[1];
    zxup_buffer_a5b5[ZXUP_A5B5_POS4] = pdata[2]; // ������ŷ�
    zxup_buffer_a5b5[ZXUP_A5B5_POS4+1] = pdata[3];
    tlv_a5b5_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x3D3:
    zxup_buffer_a5b5[ZXUP_A5B5_POS7] = pdata[0]; // �������ŷ�
    zxup_buffer_a5b5[ZXUP_A5B5_POS7+1] = pdata[1];
    zxup_buffer_a5b5[ZXUP_A5B5_POS8] = pdata[2]; // �������ŷ�
    zxup_buffer_a5b5[ZXUP_A5B5_POS8+1] = pdata[3];
    tlv_a5b5_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x443:
    //==Ƶ��ͳ��:����͸�����Ȧ���������ۺ͸��۸��޴���======
    temp_flag.byte = pdata[0];
    if(temp_flag.b.bit3)  // ������Ȧ����
    {  zxsts_od_main_hoist_flag = ZXSTS_TRUE;}
    else
    {  zxsts_od_main_hoist_flag = ZXSTS_FALSE;}
    ZxSts_ResetCanMsgTimer(&zxsts_context[ZXSTS_TYPE_ZJSQ]);

    if(temp_flag.b.bit4)  // ������Ȧ����
    {  zxsts_od_deputy_hoist_flag = ZXSTS_TRUE;}
    else
    {  zxsts_od_deputy_hoist_flag = ZXSTS_FALSE;}
    ZxSts_ResetCanMsgTimer(&zxsts_context[ZXSTS_TYPE_FJSQ]);

    temp_flag.byte = pdata[1];
    if(temp_flag.b.bit0)  // ���۸��޴���
    {  zxsts_a2b_main_arm_flag = ZXSTS_TRUE;}
    else
    {  zxsts_a2b_main_arm_flag = ZXSTS_FALSE;}
    ZxSts_ResetCanMsgTimer(&zxsts_context[ZXSTS_TYPE_ZBGX]);
    
    if(temp_flag.b.bit3)  // ���۸��޴���
    {  zxsts_a2b_deputy_arm_flag = ZXSTS_TRUE;}
    else
    {  zxsts_a2b_deputy_arm_flag = ZXSTS_FALSE;}
    ZxSts_ResetCanMsgTimer(&zxsts_context[ZXSTS_TYPE_FBGX]);
    //===================================================

    if (pdata[3]&BIT(3)) // ������ŷ�
      zxup_buffer_a5b5[ZXUP_A5B5_POS13] |= BIT(0); // ��1
    else
      zxup_buffer_a5b5[ZXUP_A5B5_POS13] &= ~BIT(0); // ��0

    if (pdata[0]&BIT(0)) // ���ʧ�ټ��*****
      zxup_buffer_a5bc[ZXUP_A5BC_POS3] |= BIT(0); // ��1
    else
      zxup_buffer_a5bc[ZXUP_A5BC_POS3] &= ~BIT(0); // ��0

    if (pdata[0]&BIT(1)) // ���ʧ�ټ��*****
      zxup_buffer_a5be[ZXUP_A5BE_POS3] |= BIT(0); // ��1
    else
      zxup_buffer_a5be[ZXUP_A5BE_POS3] &= ~BIT(0); // ��0

    tlv_a5b5_valid_flag = 1;
    tlv_a5bc_valid_flag = 1;
    tlv_a5be_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5B6====================================================================================
  case 0x264:
    if (pdata[0]&BIT(0)) // 32MPaѹ��
      zxup_buffer_a5b6[ZXUP_A5B6_POS1] |= BIT(0); // ��1
    else
      zxup_buffer_a5b6[ZXUP_A5B6_POS1] &= ~BIT(0); // ��0

    if (pdata[0]&BIT(1)) // 24MPaѹ��1
      zxup_buffer_a5b6[ZXUP_A5B6_POS1] |= BIT(1); // ��1
    else
      zxup_buffer_a5b6[ZXUP_A5B6_POS1] &= ~BIT(1); // ��0
      
    if (pdata[0]&BIT(2)) // 24MPaѹ��2
      zxup_buffer_a5b6[ZXUP_A5B6_POS1] |= BIT(3); // ��1
    else
      zxup_buffer_a5b6[ZXUP_A5B6_POS1] &= ~BIT(3); // ��0
      
    if (pdata[0]&BIT(3)) // 1Mpaѹ��
      zxup_buffer_a5b6[ZXUP_A5B6_POS1] |= BIT(4); // ��1
    else
      zxup_buffer_a5b6[ZXUP_A5B6_POS1] &= ~BIT(4); // ��0

    tlv_a5b6_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x193:
    //==Ƶ��ͳ��:�ո���Ϳո�����������ʹ�����===============
    temp_flag.byte = pdata[4];
    if((temp_flag.b.bit0==0) && (temp_flag.b.bit4==0) && (temp_flag.b.bit5==1)) // �ո��ж�
    {  zxsts_empty_cylinder_flag = ZXSTS_TRUE;}
    else
    {  zxsts_empty_cylinder_flag = ZXSTS_FALSE;}

    temp_flag.byte = pdata[4];
    if((temp_flag.b.bit0==1) && (temp_flag.b.bit1==0)) // ���۱�־
    {  zxsts_arm_work_flag = ZXSTS_TRUE;}
    else
    {  zxsts_arm_work_flag = ZXSTS_FALSE;}
    
    temp_flag.byte = pdata[3];
    if(temp_flag.b.bit0)  // ������������
    {  zxsts_cylinder_shrink_flag = ZXSTS_TRUE;}
    else
    {  zxsts_cylinder_shrink_flag = ZXSTS_FALSE;}
    
    if(temp_flag.b.bit1)  // �����������
    {  zxsts_cylinder_extend_flag = ZXSTS_TRUE;}
    else
    {  zxsts_cylinder_extend_flag = ZXSTS_FALSE;}

    ZxSts_ResetCanMsgTimer(&zxsts_context[ZXSTS_TYPE_KGEX]);
    ZxSts_ResetCanMsgTimer(&zxsts_context[ZXSTS_TYPE_KGS]);
    ZxSts_ResetCanMsgTimer(&zxsts_context[ZXSTS_TYPE_DBEX]);
    ZxSts_ResetCanMsgTimer(&zxsts_context[ZXSTS_TYPE_DBS]);
    //=========================================
  
    if (pdata[5]&BIT(5)) // ���ŷ�
      zxup_buffer_a5b6[ZXUP_A5B6_POS1] |= BIT(2); // ��1
    else
      zxup_buffer_a5b6[ZXUP_A5B6_POS1] &= ~BIT(2); // ��0

    if (pdata[5]&BIT(6)) // ����ŷ�
      zxup_buffer_a5b6[ZXUP_A5B6_POS1] |= BIT(5); // ��1
    else
      zxup_buffer_a5b6[ZXUP_A5B6_POS1] &= ~BIT(5); // ��0

    ///////////////////////////////////////////////////////////////////////
    if (pdata[0]&BIT(0)) // ���������
      zxup_buffer_a5b8[ZXUP_A5B8_POS1] |= BIT(0); // ��1
    else
      zxup_buffer_a5b8[ZXUP_A5B8_POS1] &= ~BIT(0); // ��0

    if (pdata[0]&BIT(1)) // ���������
      zxup_buffer_a5b8[ZXUP_A5B8_POS1] |= BIT(1); // ��1
    else
      zxup_buffer_a5b8[ZXUP_A5B8_POS1] &= ~BIT(1); // ��0

    if (pdata[0]&BIT(5)) // ���������
      zxup_buffer_a5b8[ZXUP_A5B8_POS1] |= BIT(2); // ��1
    else
      zxup_buffer_a5b8[ZXUP_A5B8_POS1] &= ~BIT(2); // ��0

    if (pdata[0]&BIT(4)) // ���������
      zxup_buffer_a5b8[ZXUP_A5B8_POS1] |= BIT(3); // ��1
    else
      zxup_buffer_a5b8[ZXUP_A5B8_POS1] &= ~BIT(3); // ��0

    if (pdata[4]&BIT(0)) // ǰ��������
      zxup_buffer_a5b8[ZXUP_A5B8_POS1] |= BIT(4); // ��1
    else
      zxup_buffer_a5b8[ZXUP_A5B8_POS1] &= ~BIT(4); // ��0

    if (pdata[4]&BIT(1)) // ǰ��������
      zxup_buffer_a5b8[ZXUP_A5B8_POS1] |= BIT(5); // ��1
    else
      zxup_buffer_a5b8[ZXUP_A5B8_POS1] &= ~BIT(5); // ��0

    if (pdata[4]&BIT(5)) // ǰ��������
      zxup_buffer_a5b8[ZXUP_A5B8_POS1] |= BIT(6); // ��1
    else
      zxup_buffer_a5b8[ZXUP_A5B8_POS1] &= ~BIT(6); // ��0

    if (pdata[4]&BIT(4)) // ǰ��������
      zxup_buffer_a5b8[ZXUP_A5B8_POS1] |= BIT(7); // ��1
    else
      zxup_buffer_a5b8[ZXUP_A5B8_POS1] &= ~BIT(7); // ��0

    if (pdata[0]&BIT(6)) // ǰ��ͷ���־
      zxup_buffer_a5b8[ZXUP_A5B8_POS2] |= BIT(0); // ��1
    else
      zxup_buffer_a5b8[ZXUP_A5B8_POS2] &= ~BIT(0); // ��0

    if (pdata[0]&BIT(7)) // ���ͷ���־
      zxup_buffer_a5b8[ZXUP_A5B8_POS2] |= BIT(1); // ��1
    else
      zxup_buffer_a5b8[ZXUP_A5B8_POS2] &= ~BIT(1); // ��0

    zxup_buffer_a5b8[ZXUP_A5B8_POS5] = pdata[1]; // ��λ

    if (pdata[5]&BIT(0)) // �ױ������͵�ŷ�
      zxup_buffer_a5b9[ZXUP_A5B9_POS1] |= BIT(0); // ��1
    else
      zxup_buffer_a5b9[ZXUP_A5B9_POS1] &= ~BIT(0); // ��0

    if (pdata[5]&BIT(1)) // �ױ����л���ŷ�
      zxup_buffer_a5b9[ZXUP_A5B9_POS1] |= BIT(1); // ��1
    else
      zxup_buffer_a5b9[ZXUP_A5B9_POS1] &= ~BIT(1); // ��0

    tlv_a5b6_valid_flag = 1;
    tlv_a5b8_valid_flag = 1;
    tlv_a5b9_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x2C3:
    if (pdata[2]&BIT(0)) // ������ŷ�
      zxup_buffer_a5b6[ZXUP_A5B6_POS1] |= BIT(6); // ��1
    else
      zxup_buffer_a5b6[ZXUP_A5B6_POS1] &= ~BIT(6); // ��0

    if (pdata[2]&BIT(3)) // ����������䷧
      zxup_buffer_a5b6[ZXUP_A5B6_POS1] |= BIT(7); // ��1
    else
      zxup_buffer_a5b6[ZXUP_A5B6_POS1] &= ~BIT(7); // ��0

    zxup_buffer_a5ba[ZXUP_A5BA_POS1] = pdata[6]; // ����ƽ�ⷧ����
    zxup_buffer_a5ba[ZXUP_A5BA_POS1+1] = pdata[7];
    tlv_a5b6_valid_flag = 1;
    tlv_a5ba_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5B7====================================================================================
  case 0x293:
    zxup_buffer_a5b7[ZXUP_A5B7_POS2] = pdata[2]; // �����׳���
    zxup_buffer_a5b7[ZXUP_A5B7_POS2+1] = pdata[3];
    tlv_a5b7_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x394:
    zxup_buffer_a5b7[ZXUP_A5B7_POS3] = pdata[6]; // ���ƽ�ⷧ
    zxup_buffer_a5b7[ZXUP_A5B7_POS3+1] = pdata[7];
    tlv_a5b7_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5B8====================================================================================
  case 0x1E9:
    zxup_buffer_a5b8[ZXUP_A5B8_POS3] = pdata[0]; // ǰ��ͷ���⿪��8
    zxup_buffer_a5b8[ZXUP_A5B8_POS4] = pdata[1]; // ���ͷ���⿪��8
    zxup_buffer_a5b9[ZXUP_A5B9_POS2] = pdata[6]; // ������ѹ��
    zxup_buffer_a5b9[ZXUP_A5B9_POS2+1] = pdata[7];
    tlv_a5b8_valid_flag = 1;
    tlv_a5b9_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5B9====================================================================================

  //==A5BA====================================================================================
  case 0x3C3:
    zxup_buffer_a5ba[ZXUP_A5BA_POS2] = pdata[0]; // �ұ��ƽ�ⷧ����
    zxup_buffer_a5ba[ZXUP_A5BA_POS2+1] = pdata[1];
    tlv_a5ba_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5BB====================================================================================
  //==A5BC====================================================================================

  //==A5BD====================================================================================
  case 0x2F3:
    zxup_buffer_a5bd[ZXUP_A5BD_POS1] = pdata[0]; // ������ŷ�
    zxup_buffer_a5bd[ZXUP_A5BD_POS1+1] = pdata[1];
    zxup_buffer_a5bd[ZXUP_A5BD_POS2] = pdata[2]; // �����ŷ�
    zxup_buffer_a5bd[ZXUP_A5BD_POS2+1] = pdata[3];
    zxup_buffer_a5bd[ZXUP_A5BD_POS3] = pdata[4]; // �ͱ�ѹ��
    zxup_buffer_a5bd[ZXUP_A5BD_POS3+1] = pdata[5];
    tlv_a5bd_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5BE====================================================================================
  // zxup_buffer_a5be[ZXUP_A5BE_POS1]

  //==A5BF====================================================================================
  case 0x2B3:
    zxup_buffer_a5bf[ZXUP_A5BF_POS1] = pdata[0]; // ���ת��ŷ�
    zxup_buffer_a5bf[ZXUP_A5BF_POS1+1] = pdata[1];
    zxup_buffer_a5bf[ZXUP_A5BF_POS2] = pdata[2]; // �һ�ת��ŷ�
    zxup_buffer_a5bf[ZXUP_A5BF_POS2+1] = pdata[3];
    zxup_buffer_a5bf[ZXUP_A5BF_POS4] = pdata[6]; // �ͱ�ѹ������תѹ����⣩
    zxup_buffer_a5bf[ZXUP_A5BF_POS4+1] = pdata[7];
    tlv_a5bf_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5C0====================================================================================
  case 0x3B3:
    if (pdata[0]&BIT(0)) // �ƶ����Ʒ�
      zxup_buffer_a5c0[ZXUP_A5C0_POS1] |= BIT(0); // ��1
    else
      zxup_buffer_a5c0[ZXUP_A5C0_POS1] &= ~BIT(0); // ��0

    if (pdata[0]&BIT(1)) // ѹ�����
      zxup_buffer_a5c0[ZXUP_A5C0_POS1] |= BIT(1); // ��1
    else
      zxup_buffer_a5c0[ZXUP_A5C0_POS1] &= ~BIT(1); // ��0

    zxup_buffer_a5c0[ZXUP_A5C0_POS2] = pdata[6]; // ��ת�ƶ���-����ѹ��
    zxup_buffer_a5c0[ZXUP_A5C0_POS2+1] = pdata[7];
    tlv_a5c0_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5C1====================================================================================
  case 0x364:
    zxup_buffer_a5c1[ZXUP_A5C1_POS1] = pdata[4];
    zxup_buffer_a5c1[ZXUP_A5C1_POS2] = pdata[5];
    tlv_a5c1_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5C2====================================================================================
  case 0x3E4:
    zxup_buffer_a5c2[ZXUP_A5C2_POS1] = pdata[6];  // ѹ��ѡ��
    zxup_buffer_a5c2[ZXUP_A5C2_POS1+1] = pdata[7];

    if (pdata[5]&BIT(0)) // �л�����Y35��
      zxup_buffer_a5c2[ZXUP_A5C2_POS2] |= BIT(0); // ��1
    else
      zxup_buffer_a5c2[ZXUP_A5C2_POS2] &= ~BIT(0); // ��0

    if (pdata[5]&BIT(1)) // �л�����(Y33A)
      zxup_buffer_a5c2[ZXUP_A5C2_POS2] |= BIT(1); // ��1
    else
      zxup_buffer_a5c2[ZXUP_A5C2_POS2] &= ~BIT(1); // ��0

    if (pdata[5]&BIT(2)) // �л�����(Y33B)
      zxup_buffer_a5c2[ZXUP_A5C2_POS2] |= BIT(2); // ��1
    else
      zxup_buffer_a5c2[ZXUP_A5C2_POS2] &= ~BIT(2); // ��0
    
    tlv_a5c2_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5C3====================================================================================
  //==A5C4====================================================================================
  case 0x273:
    if (pdata[0]&BIT(0)) // ��-��������
      zxup_buffer_a5c3[ZXUP_A5C3_POS4] |= BIT(0); // ��1
    else
      zxup_buffer_a5c3[ZXUP_A5C3_POS4] &= ~BIT(0); // ��0

    if (pdata[0]&BIT(1)) // ��-��������
      zxup_buffer_a5c3[ZXUP_A5C3_POS4] |= BIT(1); // ��1
    else
      zxup_buffer_a5c3[ZXUP_A5C3_POS4] &= ~BIT(1); // ��0
    
    if (pdata[0]&BIT(2)) // ��-����չ��
      zxup_buffer_a5c3[ZXUP_A5C3_POS4] |= BIT(2); // ��1
    else
      zxup_buffer_a5c3[ZXUP_A5C3_POS4] &= ~BIT(2); // ��0
    
    if (pdata[0]&BIT(3)) // ��-�����ջ�
      zxup_buffer_a5c3[ZXUP_A5C3_POS4] |= BIT(3); // ��1
    else
      zxup_buffer_a5c3[ZXUP_A5C3_POS4] &= ~BIT(3); // ��0
    
    zxup_buffer_a5c3[ZXUP_A5C3_POS1] = pdata[2]; // ��-������
    zxup_buffer_a5c3[ZXUP_A5C3_POS1+1] = pdata[3];
    zxup_buffer_a5c3[ZXUP_A5C3_POS2] = pdata[6]; // ��-������
    zxup_buffer_a5c3[ZXUP_A5C3_POS2+1] = pdata[7];
    zxup_buffer_a5c3[ZXUP_A5C3_POS7] = pdata[4]; // ��-�Ž�����
    zxup_buffer_a5c3[ZXUP_A5C3_POS7+1] = pdata[5];
    tlv_a5c3_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x373:
    if (pdata[0]&BIT(7)) // ��-����ѹ��ѡ��
      zxup_buffer_a5c3[ZXUP_A5C3_POS4] |= BIT(4); // ��1
    else
      zxup_buffer_a5c3[ZXUP_A5C3_POS4] &= ~BIT(4); // ��0

    if (pdata[1]&BIT(3)) // ��-�����ƶ�
      zxup_buffer_a5c3[ZXUP_A5C3_POS4] |= BIT(5); // ��1
    else
      zxup_buffer_a5c3[ZXUP_A5C3_POS4] &= ~BIT(5); // ��0
    
    if (pdata[1]&BIT(1)) // ��-���ֽ���
      zxup_buffer_a5c3[ZXUP_A5C3_POS4] |= BIT(6); // ��1
    else
      zxup_buffer_a5c3[ZXUP_A5C3_POS4] &= ~BIT(6); // ��0
    
    if (pdata[1]&BIT(0)) // ��-��������
      zxup_buffer_a5c3[ZXUP_A5C3_POS4] |= BIT(7); // ��1
    else
      zxup_buffer_a5c3[ZXUP_A5C3_POS4] &= ~BIT(7); // ��0
    
    if (pdata[1]&BIT(2)) // ��-�Ž��׽���
      zxup_buffer_a5c3[ZXUP_A5C3_POS5] |= BIT(0); // ��1
    else
      zxup_buffer_a5c3[ZXUP_A5C3_POS5] &= ~BIT(0); // ��0
    
    if (pdata[1]&BIT(4)) // ��-������︡��
      zxup_buffer_a5c3[ZXUP_A5C3_POS5] |= BIT(1); // ��1
    else
      zxup_buffer_a5c3[ZXUP_A5C3_POS5] &= ~BIT(1); // ��0
    
    zxup_buffer_a5c4[ZXUP_A5C4_POS1] = pdata[2]; // ��-������
    zxup_buffer_a5c4[ZXUP_A5C4_POS1+1] = pdata[3];
    zxup_buffer_a5c4[ZXUP_A5C4_POS2] = pdata[6]; // ��-������
    zxup_buffer_a5c4[ZXUP_A5C4_POS2+1] = pdata[7];
    zxup_buffer_a5c4[ZXUP_A5C4_POS7] = pdata[4]; // ��-�Ž�����
    zxup_buffer_a5c4[ZXUP_A5C4_POS7+1] = pdata[5];
    
    if (pdata[0]&BIT(0)) // ��-��������
      zxup_buffer_a5c4[ZXUP_A5C4_POS4] |= BIT(0); // ��1
    else
      zxup_buffer_a5c4[ZXUP_A5C4_POS4] &= ~BIT(0); // ��0

    if (pdata[0]&BIT(1)) // ��-��������
      zxup_buffer_a5c4[ZXUP_A5C4_POS4] |= BIT(1); // ��1
    else
      zxup_buffer_a5c4[ZXUP_A5C4_POS4] &= ~BIT(1); // ��0
    
    if (pdata[0]&BIT(2)) // ��-����չ��
      zxup_buffer_a5c4[ZXUP_A5C4_POS4] |= BIT(2); // ��1
    else
      zxup_buffer_a5c4[ZXUP_A5C4_POS4] &= ~BIT(2); // ��0
    
    if (pdata[0]&BIT(3)) // ��-�����ջ�
      zxup_buffer_a5c4[ZXUP_A5C4_POS4] |= BIT(3); // ��1
    else
      zxup_buffer_a5c4[ZXUP_A5C4_POS4] &= ~BIT(3); // ��0

    if (pdata[0]&BIT(7)) // ��-����ѹ��ѡ��
      zxup_buffer_a5c4[ZXUP_A5C4_POS4] |= BIT(4); // ��1
    else
      zxup_buffer_a5c4[ZXUP_A5C4_POS4] &= ~BIT(4); // ��0

    if (pdata[1]&BIT(3)) // ��-�����ƶ�
      zxup_buffer_a5c4[ZXUP_A5C4_POS4] |= BIT(5); // ��1
    else
      zxup_buffer_a5c4[ZXUP_A5C4_POS4] &= ~BIT(5); // ��0
    
    if (pdata[1]&BIT(1)) // ��-���ֽ���
      zxup_buffer_a5c4[ZXUP_A5C4_POS4] |= BIT(6); // ��1
    else
      zxup_buffer_a5c4[ZXUP_A5C4_POS4] &= ~BIT(6); // ��0
    
    if (pdata[1]&BIT(0)) // ��-��������
      zxup_buffer_a5c4[ZXUP_A5C4_POS4] |= BIT(7); // ��1
    else
      zxup_buffer_a5c4[ZXUP_A5C4_POS4] &= ~BIT(7); // ��0
    
    if (pdata[1]&BIT(2)) // ��-�Ž��׽���
      zxup_buffer_a5c4[ZXUP_A5C4_POS5] |= BIT(0); // ��1
    else
      zxup_buffer_a5c4[ZXUP_A5C4_POS5] &= ~BIT(0); // ��0
    
    if (pdata[1]&BIT(4)) // ��-������︡��
      zxup_buffer_a5c4[ZXUP_A5C4_POS5] |= BIT(1); // ��1
    else
      zxup_buffer_a5c4[ZXUP_A5C4_POS5] &= ~BIT(1); // ��0
    
    tlv_a5c3_valid_flag = 1;
    tlv_a5c4_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x504:
    zxup_buffer_a5c3[ZXUP_A5C3_POS3] = pdata[0]; // ��-����������
    zxup_buffer_a5c3[ZXUP_A5C3_POS3+1] = pdata[1];
    zxup_buffer_a5c4[ZXUP_A5C4_POS3] = pdata[2]; // ��-����������
    zxup_buffer_a5c4[ZXUP_A5C4_POS3+1] = pdata[3];
    tlv_a5c3_valid_flag = 1;
    tlv_a5c4_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x473:
    zxup_buffer_a5c3[ZXUP_A5C3_POS6] = pdata[0]; // ��-�Ž�����
    zxup_buffer_a5c3[ZXUP_A5C3_POS6+1] = pdata[1];
    zxup_buffer_a5c4[ZXUP_A5C4_POS6] = pdata[2]; // ��-�Ž�����
    zxup_buffer_a5c4[ZXUP_A5C4_POS6+1] = pdata[3];
    tlv_a5c3_valid_flag = 1;
    tlv_a5c4_valid_flag = 1;
    retval = 0x01;
    break;

  //==�ϳ�ϵͳ�汾====================================================================================
  case 0x562:
    zxversion_buffer_a505[ZXVERSION_A505_POS7] = pdata[0];  // ��ʾ��2�ײ�
    zxversion_buffer_a505[ZXVERSION_A505_POS7+1] = pdata[1];
    zxversion_buffer_a505[ZXVERSION_A505_POS7+2] = pdata[2];
    tlv_a505_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x571:
    zxversion_buffer_a505[ZXVERSION_A505_POS2] = pdata[4];  // ��ʾ��1
    zxversion_buffer_a505[ZXVERSION_A505_POS2+1] = pdata[5];
    zxversion_buffer_a505[ZXVERSION_A505_POS2+2] = pdata[6];
    tlv_a505_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x572:
    zxversion_buffer_a505[ZXVERSION_A505_POS6] = pdata[4];  // ��ʾ��2
    zxversion_buffer_a505[ZXVERSION_A505_POS6+1] = pdata[5];
    zxversion_buffer_a505[ZXVERSION_A505_POS6+2] = pdata[6];
    tlv_a505_valid_flag = 1;
    retval = 0x00;
    break;

  case 0x573:
    zxversion_buffer_a505[ZXVERSION_A505_POS5] = pdata[4];  // ������
    zxversion_buffer_a505[ZXVERSION_A505_POS5+1] = pdata[5];
    zxversion_buffer_a505[ZXVERSION_A505_POS5+2] = pdata[6];
    tlv_a505_valid_flag = 1;
    retval = 0x00;
    break;
    
  case 0x575:
    zxversion_buffer_a505[ZXVERSION_A505_POS1] = pdata[4];  // ����������
    zxversion_buffer_a505[ZXVERSION_A505_POS1+1] = pdata[5];
    zxversion_buffer_a505[ZXVERSION_A505_POS1+2] = pdata[6];
    tlv_a505_valid_flag = 1;
    retval = 0x01;
    break;

  default:
    break;
  }

  return retval;
}

/*************************************************************************
 * ������յ���dCAN����(�³�)(����ʽ���ػ�Autocrane )
*************************************************************************/
uint8_t CAN_ProcessRecvDownMsg_AC(uint32_t canId, uint8_t *pdata, uint8_t size)
{
  uint8_t retval = 0x00;
  //uint32_t tempVal;

  switch (canId) // �³�ͨ��
  {
  //==A5E0===================================================================================
  case 0x1CF23B21:
    zxdown_buffer_a5e0[ZXDOWN_A5E0_POS1] = pdata[0]; // �ڵ�״̬1
    
    if (pdata[1]&BIT(0)) // ����������ź�
      zxdown_buffer_a5e0[ZXDOWN_A5E0_POS2] |= BIT(0); // ��1
    else
      zxdown_buffer_a5e0[ZXDOWN_A5E0_POS2] &= ~BIT(0); // ��0

    if (pdata[1]&BIT(1)) // ABS�����ź�
      zxdown_buffer_a5e0[ZXDOWN_A5E0_POS2] |= BIT(1); // ��1
    else
      zxdown_buffer_a5e0[ZXDOWN_A5E0_POS2] &= ~BIT(1); // ��0
    
    if (pdata[1]&BIT(4)) // �������¶ȸ߼��
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS8] |= BIT(1); // ��1
    else
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS8] &= ~BIT(1); // ��0
    
    if (pdata[1]&BIT(3)) // �ƶ���ĥ����
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS8] |= BIT(2); // ��1
    else
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS8] &= ~BIT(2); // ��0
    
    if (pdata[1]&BIT(3)) // �ƶ���Ƭĥ���⿪�� ?????????????????????
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS8] |= BIT(3); // ��1
    else
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS8] &= ~BIT(3); // ��0
    
    if (pdata[1]&BIT(2)) // ABS����״̬
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS8] |= BIT(6); // ��1
    else
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS8] &= ~BIT(6); // ��0
    
    tlv_a5e0_valid_flag = 1;
    tlv_a5e6_valid_flag = 1;
    retval = 0x01;
    break;
    
  //==A5E1====================================================================================
  case 0x1CF21D21:
    zxdown_buffer_a5e1[ZXDOWN_A5E1_POS1] = pdata[7];
    if (pdata[3]&BIT(4)) // ������յ���� У��
      zxdown_buffer_a5e1[ZXDOWN_A5E1_POS2] |= BIT(0); // ��1
    else
      zxdown_buffer_a5e1[ZXDOWN_A5E1_POS2] &= ~BIT(0); // ��0
    
    if (pdata[3]&BIT(5)) // �����䵹����� У��
      zxdown_buffer_a5e1[ZXDOWN_A5E1_POS2] |= BIT(1); // ��1
    else
      zxdown_buffer_a5e1[ZXDOWN_A5E1_POS2] &= ~BIT(1); // ��0
    
    if (pdata[1]&BIT(0)) // ����������� У��
      zxdown_buffer_a5e1[ZXDOWN_A5E1_POS2] |= BIT(2); // ��1
    else
      zxdown_buffer_a5e1[ZXDOWN_A5E1_POS2] &= ~BIT(2); // ��0
    
    if (pdata[1]&BIT(1)) // �ֶ���ߵ����
      zxdown_buffer_a5e1[ZXDOWN_A5E1_POS2] |= BIT(3); // ��1
    else
      zxdown_buffer_a5e1[ZXDOWN_A5E1_POS2] &= ~BIT(3); // ��0
    
    if (pdata[1]&BIT(2)) // �ֶ���͵����
      zxdown_buffer_a5e1[ZXDOWN_A5E1_POS2] |= BIT(4); // ��1
    else
      zxdown_buffer_a5e1[ZXDOWN_A5E1_POS2] &= ~BIT(4); // ��0

    if (pdata[1]&BIT(3)) // �ּ���ټ��
      zxdown_buffer_a5e1[ZXDOWN_A5E1_POS2] |= BIT(5); // ��1
    else
      zxdown_buffer_a5e1[ZXDOWN_A5E1_POS2] &= ~BIT(5); // ��0
    
    if (pdata[1]&BIT(4)) // �����ټ��
      zxdown_buffer_a5e1[ZXDOWN_A5E1_POS2] |= BIT(6); // ��1
    else
      zxdown_buffer_a5e1[ZXDOWN_A5E1_POS2] &= ~BIT(6); // ��0

    if (pdata[6]&BIT(6)) // ��ɲ���(���ƶ���⿪��)
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS8] |= BIT(0); // ��1
    else
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS8] &= ~BIT(0); // ��0
    
    if (pdata[3]&BIT(6)) // �г��ƶ���⿪��
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS8] |= BIT(4); // ��1
    else
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS8] &= ~BIT(4); // ��0
    
    if (pdata[2]&BIT(5)) // ����ѹ����
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS8] |= BIT(5); // ��1
    else
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS8] &= ~BIT(5); // ��0
    
    if (pdata[6]&BIT(7)) // PTO���
      zxdown_buffer_a5e7[ZXDOWN_A5E7_POS1] |= BIT(0); // ��1
    else
      zxdown_buffer_a5e7[ZXDOWN_A5E7_POS1] &= ~BIT(0); // ��0

    tlv_a5e1_valid_flag = 1;
    tlv_a5e6_valid_flag = 1;
    tlv_a5e7_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x1CF25E21:
    zxdown_buffer_a5e1[ZXDOWN_A5E1_POS3] = pdata[0];  // ����������
    zxdown_buffer_a5e1[ZXDOWN_A5E1_POS3+1] = pdata[1];
    zxdown_buffer_a5e1[ZXDOWN_A5E1_POS4] = pdata[3];  // �����������ѹ
    zxdown_buffer_a5e1[ZXDOWN_A5E1_POS5] = pdata[4];  // �����λ�� 
    
    if (pdata[5]&BIT(0)) // �������ƶ�
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS8] |= BIT(7); // ��1
    else
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS8] &= ~BIT(7); // ��0
    
    if (pdata[5]&BIT(1)) // �������ƶ�1��
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS9] |= BIT(0); // ��1
    else
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS9] &= ~BIT(0); // ��0
    
    if (pdata[5]&BIT(2)) // �������ƶ�2��
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS9] |= BIT(1); // ��1
    else
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS9] &= ~BIT(1); // ��0
    
    if (pdata[5]&BIT(3)) // �������ƶ�3��
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS9] |= BIT(2); // ��1
    else
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS9] &= ~BIT(2); // ��0
    
    if (pdata[5]&BIT(4)) // �������ƶ�4��
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS9] |= BIT(3); // ��1
    else
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS9] &= ~BIT(3); // ��0
    
    if (pdata[5]&BIT(5)) // ������Ϩ������
      zxdown_buffer_a5e7[ZXDOWN_A5E7_POS1] |= BIT(1); // ��1
    else
      zxdown_buffer_a5e7[ZXDOWN_A5E7_POS1] &= ~BIT(1); // ��0
    
    if (pdata[5]&BIT(6)) // ������Ϩ�����
      zxdown_buffer_a5e7[ZXDOWN_A5E7_POS1] |= BIT(2); // ��1
    else
      zxdown_buffer_a5e7[ZXDOWN_A5E7_POS1] &= ~BIT(2); // ��0
    
    zxdown_buffer_a5e6[ZXDOWN_A5E6_POS5] = pdata[0];  // ����������
    zxdown_buffer_a5e6[ZXDOWN_A5E6_POS5+1] = pdata[1];
    zxdown_buffer_a5e6[ZXDOWN_A5E6_POS6] = pdata[2];  // �������ذٷֱ�

    tlv_a5e1_valid_flag = 1;
    tlv_a5e6_valid_flag = 1;
    tlv_a5e7_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5E5====================================================================================
  case 0x1CF22D21:
    zxdown_buffer_a5e5[ZXDOWN_A5E5_POS1] = pdata[0];  // ��ǰһ��ת��
    zxdown_buffer_a5e5[ZXDOWN_A5E5_POS1+1] = pdata[1];
    tlv_a5e5_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x1CF28B21:
    if (pdata[1]&BIT(0)) // ת��ϵͳ����ָʾ��E101
      zxdown_buffer_a5e5[ZXDOWN_A5E5_POS6] |= BIT(0); // ��1
    else
      zxdown_buffer_a5e5[ZXDOWN_A5E5_POS6] &= ~BIT(0); // ��0

    if (pdata[1]&BIT(1)) // ��λָʾ��
      zxdown_buffer_a5e5[ZXDOWN_A5E5_POS6] |= BIT(1); // ��1
    else
      zxdown_buffer_a5e5[ZXDOWN_A5E5_POS6] &= ~BIT(1); // ��0
    
    zxdown_buffer_a5e5[ZXDOWN_A5E5_POS2] = pdata[4];  // ת������ѹ��(bar)
    zxdown_buffer_a5e5[ZXDOWN_A5E5_POS2+1] = pdata[5];
    zxdown_buffer_a5e5[ZXDOWN_A5E5_POS4] = pdata[2];  // ������λ
    zxdown_buffer_a5e5[ZXDOWN_A5E5_POS5] = pdata[0];  // ���ֽ�״̬λ1
    tlv_a5e5_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x1CF25C21:
    zxdown_buffer_a5e5[ZXDOWN_A5E5_POS3] = pdata[0];  // Ŀ��ת��ģʽ
    zxdown_buffer_a5e5[ZXDOWN_A5E5_POS3+1] = pdata[1];// ��ǰת��ģʽ
    tlv_a5e5_valid_flag = 1;

    zxdown_buffer_a5e9[ZXDOWN_A5E9_POS2] = pdata[6]; // Һѹϵͳѹ��
    zxdown_buffer_a5e9[ZXDOWN_A5E9_POS2+1] = pdata[7];
    tlv_a5e9_valid_flag = 1;
    retval = 0x01;
    break;
  
  //==A5E6====================================================================================
  case 0x1CF25D21:
    zxdown_buffer_a5e6[ZXDOWN_A5E6_POS1] = pdata[0];  // ��·һ��ѹ
    zxdown_buffer_a5e6[ZXDOWN_A5E6_POS1+1] = pdata[1];
    zxdown_buffer_a5e6[ZXDOWN_A5E6_POS2] = pdata[2];  // ��·һ��ѹ
    zxdown_buffer_a5e6[ZXDOWN_A5E6_POS2+1] = pdata[3];
    zxdown_buffer_a5e6[ZXDOWN_A5E6_POS3] = pdata[4];  // ��·һ��ѹ
    zxdown_buffer_a5e6[ZXDOWN_A5E6_POS3+1] = pdata[5];
    zxdown_buffer_a5e6[ZXDOWN_A5E6_POS4] = pdata[6];  // ��·һ��ѹ
    zxdown_buffer_a5e6[ZXDOWN_A5E6_POS4+1] = pdata[7];
    tlv_a5e6_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x1CF23A21:
    zxdown_buffer_a5e6[ZXDOWN_A5E6_POS7] = pdata[4];  // ���������
    zxdown_buffer_a5e6[ZXDOWN_A5E6_POS7+1] = pdata[5];
    tlv_a5e6_valid_flag = 1;
    retval = 0x01;
    break;
    
  //==A5E7====================================================================================
  // ��ɢ

  //==A5E9====================================================================================
  case 0x1CF24B21:
    zxdown_buffer_a5e9[ZXDOWN_A5E9_POS1] = pdata[6]; // Һѹ���¶�
    tlv_a5e9_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5EB====================================================================================
  case 0x1CF28C21:
    if(pdata[0]>0 && pdata[0]<19) // 1����̥��18����̥
    {
      zxdown_buffer_a5eb[ZXDOWN_A5EB_POS1+pdata[0]-1] = pdata[1];
      tlv_a5eb_valid_flag = 1;
    }
    retval = 0x01;
    break;

  //==�³�ϵͳ�汾====================================================================================
  case 0x1CF20F21:
    zxversion_buffer_a506[ZXVERSION_A506_POS3] = pdata[0];  // P1Ӧ�ò�
    zxversion_buffer_a506[ZXVERSION_A506_POS3+1] = pdata[1];
    zxversion_buffer_a506[ZXVERSION_A506_POS3+2] = pdata[2];
    zxversion_buffer_a506[ZXVERSION_A506_POS5] = pdata[3];  // P2Ӧ�ò�
    zxversion_buffer_a506[ZXVERSION_A506_POS5+1] = pdata[4];
    zxversion_buffer_a506[ZXVERSION_A506_POS5+2] = pdata[5];
    zxversion_buffer_a506[ZXVERSION_A506_POS7] = pdata[6];  // P3Ӧ�ò�
    zxversion_buffer_a506[ZXVERSION_A506_POS7+1] = pdata[7];
    tlv_a506_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x1CF21F21:
    zxversion_buffer_a506[ZXVERSION_A506_POS7+2] = pdata[0];  // P3Ӧ�ò�
    zxversion_buffer_a506[ZXVERSION_A506_POS9] = pdata[1];    // P4Ӧ�ò�
    zxversion_buffer_a506[ZXVERSION_A506_POS9+1] = pdata[2];
    zxversion_buffer_a506[ZXVERSION_A506_POS9+2] = pdata[3];
    zxversion_buffer_a506[ZXVERSION_A506_POS11] = pdata[4];   // P5Ӧ�ò�
    zxversion_buffer_a506[ZXVERSION_A506_POS11+1] = pdata[5];
    zxversion_buffer_a506[ZXVERSION_A506_POS11+2] = pdata[6];
    zxversion_buffer_a506[ZXVERSION_A506_POS17] = pdata[7];   // P8Ӧ�ò�
    tlv_a506_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x1CF22F21:
    zxversion_buffer_a506[ZXVERSION_A506_POS17+1] = pdata[0];   // P8Ӧ�ò�
    zxversion_buffer_a506[ZXVERSION_A506_POS17+2] = pdata[1];
    tlv_a506_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x1CF29F21:
    if(pdata[0]==1)
    {
      zxversion_buffer_a506[ZXVERSION_A506_POS4] = pdata[1];  // P1�ײ�
      zxversion_buffer_a506[ZXVERSION_A506_POS4+1] = pdata[2];
      zxversion_buffer_a506[ZXVERSION_A506_POS4+2] = pdata[3];
      tlv_a506_valid_flag = 1;
    }
    else if(pdata[0]==2)
    {
      zxversion_buffer_a506[ZXVERSION_A506_POS6] = pdata[1];  // P2�ײ�
      zxversion_buffer_a506[ZXVERSION_A506_POS6+1] = pdata[2];
      zxversion_buffer_a506[ZXVERSION_A506_POS6+2] = pdata[3];
      tlv_a506_valid_flag = 1;
    }
    else if(pdata[0]==3)
    {
      zxversion_buffer_a506[ZXVERSION_A506_POS8] = pdata[1];  // P3�ײ�
      zxversion_buffer_a506[ZXVERSION_A506_POS8+1] = pdata[2];
      zxversion_buffer_a506[ZXVERSION_A506_POS8+2] = pdata[3];
      tlv_a506_valid_flag = 1;
    }
    else if(pdata[0]==4)
    {
      zxversion_buffer_a506[ZXVERSION_A506_POS10] = pdata[1];  // P4�ײ�
      zxversion_buffer_a506[ZXVERSION_A506_POS10+1] = pdata[2];
      zxversion_buffer_a506[ZXVERSION_A506_POS10+2] = pdata[3];
      tlv_a506_valid_flag = 1;
    }
    else if(pdata[0]==5)
    {
      zxversion_buffer_a506[ZXVERSION_A506_POS12] = pdata[1];  // P5�ײ�
      zxversion_buffer_a506[ZXVERSION_A506_POS12+1] = pdata[2];
      zxversion_buffer_a506[ZXVERSION_A506_POS12+2] = pdata[3];
      tlv_a506_valid_flag = 1;
    }
    else if(pdata[0]==6)
    {
      zxversion_buffer_a506[ZXVERSION_A506_POS14] = pdata[1];  // P6�ײ�
      zxversion_buffer_a506[ZXVERSION_A506_POS14+1] = pdata[2];
      zxversion_buffer_a506[ZXVERSION_A506_POS14+2] = pdata[3];
      tlv_a506_valid_flag = 1;
    }
    else if(pdata[0]==7)
    {
      zxversion_buffer_a506[ZXVERSION_A506_POS16] = pdata[1];  // P7�ײ�
      zxversion_buffer_a506[ZXVERSION_A506_POS16+1] = pdata[2];
      zxversion_buffer_a506[ZXVERSION_A506_POS16+2] = pdata[3];
      tlv_a506_valid_flag = 1;
    }
    else if(pdata[0]==8)
    {
      zxversion_buffer_a506[ZXVERSION_A506_POS18] = pdata[1];  // P8�ײ�
      zxversion_buffer_a506[ZXVERSION_A506_POS18+1] = pdata[2];
      zxversion_buffer_a506[ZXVERSION_A506_POS18+2] = pdata[3];
      tlv_a506_valid_flag = 1;
    }
    retval = 0x01;
    break;


  default:
    retval = 0x00;
    break;
  }
  
  return retval;
}

/*************************************************************************
 * ������յ���dCAN����(�³�)(ȫ�������ػ�All-Terrain Crane)
*************************************************************************/
uint8_t CAN_ProcessRecvDownMsg_AG(uint32_t canId, uint8_t *pdata, uint8_t size)
{
  uint8_t retval = 0x00;
  //uint32_t tempVal;

  switch (canId) // �³�ͨ��
  {
  //==A5A3====================================================================================
  case 0x24D:
    memcpy(&zxdown_buffer_a5a3[ZXDOWN_A5A3_POS1], pdata, 8); // ˮƽ֧�ȳ���
    tlv_a5a3_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x22A:
    zxdown_buffer_a5a3[ZXDOWN_A5A3_POS2] = pdata[0];   // ��ǰ֧��ѹ��
    zxdown_buffer_a5a3[ZXDOWN_A5A3_POS2+1] = pdata[1];
    zxdown_buffer_a5a3[ZXDOWN_A5A3_POS2+2] = pdata[2]; // ��ǰ֧��ѹ��
    zxdown_buffer_a5a3[ZXDOWN_A5A3_POS2+3] = pdata[3];
    zxdown_buffer_a5a3[ZXDOWN_A5A3_POS2+4] = pdata[4]; // ���֧��ѹ��
    zxdown_buffer_a5a3[ZXDOWN_A5A3_POS2+5] = pdata[5];
    zxdown_buffer_a5a3[ZXDOWN_A5A3_POS2+6] = pdata[6]; // �Һ�֧��ѹ��
    zxdown_buffer_a5a3[ZXDOWN_A5A3_POS2+7] = pdata[7];
    tlv_a5a3_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x24E:
    memcpy(&zxdown_buffer_a5a3[ZXDOWN_A5A3_POS3], pdata, 8); // �ڸ׳���
    tlv_a5a3_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5A4====================================================================================
  case 0x24B:
    if (pdata[1]&BIT(4)) // Һѹ�����״̬
      zxdown_buffer_a5ea[ZXDOWN_A5EA_POS2] |= BIT(1); // ��1
    else
      zxdown_buffer_a5ea[ZXDOWN_A5EA_POS2] &= ~BIT(1); // ��0
    
    zxdown_buffer_a5a4[ZXDOWN_A5A4_POS1] = pdata[0]; // ����֧��״̬
    zxdown_buffer_a5e9[ZXDOWN_A5E9_POS1] = pdata[6]; // Һѹ���¶�
    zxdown_buffer_a5ea[ZXDOWN_A5EA_POS8] = pdata[2]; // ɢ����Һѹ�����ÿ��Ƶ���
    zxdown_buffer_a5ea[ZXDOWN_A5EA_POS8+1] = pdata[3];
    tlv_a5a4_valid_flag = 1;
    tlv_a5e9_valid_flag = 1;
    tlv_a5ea_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x28D:
    zxdown_buffer_a5a4[ZXDOWN_A5A4_POS2] = pdata[0]; // ��ǰ����֧��ѹ��
    zxdown_buffer_a5a4[ZXDOWN_A5A4_POS2+1] = pdata[1];
    zxdown_buffer_a5a4[ZXDOWN_A5A4_POS3] = pdata[2]; // ��ǰ����֧��ѹ��
    zxdown_buffer_a5a4[ZXDOWN_A5A4_POS3+1] = pdata[3]; 
    tlv_a5a4_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5E0===================================================================================
  case 0x23B:
    zxdown_buffer_a5e0[ZXDOWN_A5E0_POS1] = pdata[0]; // �ڵ�״̬1
    
    if (pdata[1]&BIT(0)) // ����������ź�
      zxdown_buffer_a5e0[ZXDOWN_A5E0_POS2] |= BIT(0); // ��1
    else
      zxdown_buffer_a5e0[ZXDOWN_A5E0_POS2] &= ~BIT(0); // ��0

    if (pdata[1]&BIT(1)) // ABS�����ź�
      zxdown_buffer_a5e0[ZXDOWN_A5E0_POS2] |= BIT(1); // ��1
    else
      zxdown_buffer_a5e0[ZXDOWN_A5E0_POS2] &= ~BIT(1); // ��0
    
    if (pdata[1]&BIT(4)) // �������¶ȸ߼��
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS8] |= BIT(1); // ��1
    else
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS8] &= ~BIT(1); // ��0
    
    if (pdata[1]&BIT(3)) // �ƶ���ĥ����
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS8] |= BIT(2); // ��1
    else
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS8] &= ~BIT(2); // ��0
    
    if (pdata[1]&BIT(3)) // �ƶ���Ƭĥ���⿪�� ?????????????????????
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS8] |= BIT(3); // ��1
    else
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS8] &= ~BIT(3); // ��0
    
    if (pdata[1]&BIT(2)) // ABS����״̬
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS8] |= BIT(6); // ��1
    else
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS8] &= ~BIT(6); // ��0
    
    tlv_a5e0_valid_flag = 1;
    tlv_a5e6_valid_flag = 1;
    retval = 0x01;
    break;
    
  //==A5E1====================================================================================
  case 0x21D:
    zxdown_buffer_a5e1[ZXDOWN_A5E1_POS1] = pdata[7];
    if (pdata[0]&BIT(0)) // �ֶ���յ�
      zxdown_buffer_a5e8[ZXDOWN_A5E8_POS2] |= BIT(0); // ��1
    else
      zxdown_buffer_a5e8[ZXDOWN_A5E8_POS2] &= ~BIT(0); // ��0
    
    if (pdata[0]&BIT(1)) // ȡ��standby
      zxdown_buffer_a5e8[ZXDOWN_A5E8_POS2] |= BIT(1); // ��1
    else
      zxdown_buffer_a5e8[ZXDOWN_A5E8_POS2] &= ~BIT(1); // ��0
    
    if (pdata[3]&BIT(0)) // bit0:����ECOģʽ
      zxdown_buffer_a5e8[ZXDOWN_A5E8_POS2] |= BIT(2); // ��1
    else
      zxdown_buffer_a5e8[ZXDOWN_A5E8_POS2] &= ~BIT(2); // ��0
    
    if (pdata[0]&BIT(2)) // ȡ������
      zxdown_buffer_a5e8[ZXDOWN_A5E8_POS2] |= BIT(3); // ��1
    else
      zxdown_buffer_a5e8[ZXDOWN_A5E8_POS2] &= ~BIT(3); // ��0
    
    if (pdata[3]&BIT(4)) // ������յ���� У��
      zxdown_buffer_a5e1[ZXDOWN_A5E1_POS2] |= BIT(0); // ��1
    else
      zxdown_buffer_a5e1[ZXDOWN_A5E1_POS2] &= ~BIT(0); // ��0
    
    if (pdata[3]&BIT(5)) // �����䵹����� У��
      zxdown_buffer_a5e1[ZXDOWN_A5E1_POS2] |= BIT(1); // ��1
    else
      zxdown_buffer_a5e1[ZXDOWN_A5E1_POS2] &= ~BIT(1); // ��0
    
    if (pdata[1]&BIT(0)) // ����������� У��
      zxdown_buffer_a5e1[ZXDOWN_A5E1_POS2] |= BIT(2); // ��1
    else
      zxdown_buffer_a5e1[ZXDOWN_A5E1_POS2] &= ~BIT(2); // ��0
    
    if (pdata[1]&BIT(1)) // �ֶ���ߵ����
      zxdown_buffer_a5e1[ZXDOWN_A5E1_POS2] |= BIT(3); // ��1
    else
      zxdown_buffer_a5e1[ZXDOWN_A5E1_POS2] &= ~BIT(3); // ��0
    
    if (pdata[1]&BIT(2)) // �ֶ���͵����
      zxdown_buffer_a5e1[ZXDOWN_A5E1_POS2] |= BIT(4); // ��1
    else
      zxdown_buffer_a5e1[ZXDOWN_A5E1_POS2] &= ~BIT(4); // ��0

    if (pdata[1]&BIT(3)) // �ּ���ټ��
      zxdown_buffer_a5e1[ZXDOWN_A5E1_POS2] |= BIT(5); // ��1
    else
      zxdown_buffer_a5e1[ZXDOWN_A5E1_POS2] &= ~BIT(5); // ��0
    
    if (pdata[1]&BIT(4)) // �����ټ��
      zxdown_buffer_a5e1[ZXDOWN_A5E1_POS2] |= BIT(6); // ��1
    else
      zxdown_buffer_a5e1[ZXDOWN_A5E1_POS2] &= ~BIT(6); // ��0
    
    if (pdata[6]&BIT(2)) // �ֶ���͵����
      zxdown_buffer_a5ea[ZXDOWN_A5EA_POS2] |= BIT(0); // ��1
    else
      zxdown_buffer_a5ea[ZXDOWN_A5EA_POS2] &= ~BIT(0); // ��0
    
    if (pdata[6]&BIT(6)) // ��ɲ���(���ƶ���⿪��)
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS8] |= BIT(0); // ��1
    else
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS8] &= ~BIT(0); // ��0
    
    if (pdata[3]&BIT(6)) // �г��ƶ���⿪��
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS8] |= BIT(4); // ��1
    else
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS8] &= ~BIT(4); // ��0
    
    if (pdata[2]&BIT(5)) // ����ѹ����
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS8] |= BIT(5); // ��1
    else
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS8] &= ~BIT(5); // ��0
    
    if (pdata[6]&BIT(7)) // PTO���
      zxdown_buffer_a5e7[ZXDOWN_A5E7_POS1] |= BIT(0); // ��1
    else
      zxdown_buffer_a5e7[ZXDOWN_A5E7_POS1] &= ~BIT(0); // ��0

    zxdown_buffer_a5e8[ZXDOWN_A5E8_POS1] = pdata[2]; // ȡ���ҽ�(�ֽ�״̬λ)
    tlv_a5e1_valid_flag = 1;
    tlv_a5e6_valid_flag = 1;
    tlv_a5e7_valid_flag = 1;
    tlv_a5e8_valid_flag = 1;
    tlv_a5ea_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x25E:
    zxdown_buffer_a5e1[ZXDOWN_A5E1_POS3] = pdata[0];  // ����������
    zxdown_buffer_a5e1[ZXDOWN_A5E1_POS3+1] = pdata[1];
    zxdown_buffer_a5e1[ZXDOWN_A5E1_POS4] = pdata[3];  // �����������ѹ
    zxdown_buffer_a5e1[ZXDOWN_A5E1_POS5] = pdata[4];  // �����λ�� 
    
    if (pdata[5]&BIT(0)) // �������ƶ�
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS8] |= BIT(7); // ��1
    else
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS8] &= ~BIT(7); // ��0
    
    if (pdata[5]&BIT(1)) // �������ƶ�1��
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS9] |= BIT(0); // ��1
    else
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS9] &= ~BIT(0); // ��0
    
    if (pdata[5]&BIT(2)) // �������ƶ�2��
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS9] |= BIT(1); // ��1
    else
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS9] &= ~BIT(1); // ��0
    
    if (pdata[5]&BIT(3)) // �������ƶ�3��
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS9] |= BIT(2); // ��1
    else
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS9] &= ~BIT(2); // ��0
    
    if (pdata[5]&BIT(4)) // �������ƶ�4��
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS9] |= BIT(3); // ��1
    else
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS9] &= ~BIT(3); // ��0
    
    if (pdata[5]&BIT(5)) // ������Ϩ������
      zxdown_buffer_a5e7[ZXDOWN_A5E7_POS1] |= BIT(1); // ��1
    else
      zxdown_buffer_a5e7[ZXDOWN_A5E7_POS1] &= ~BIT(1); // ��0
    
    if (pdata[5]&BIT(6)) // ������Ϩ�����
      zxdown_buffer_a5e7[ZXDOWN_A5E7_POS1] |= BIT(2); // ��1
    else
      zxdown_buffer_a5e7[ZXDOWN_A5E7_POS1] &= ~BIT(2); // ��0
    
    zxdown_buffer_a5e6[ZXDOWN_A5E6_POS5] = pdata[0];  // ����������
    zxdown_buffer_a5e6[ZXDOWN_A5E6_POS5+1] = pdata[1];
    zxdown_buffer_a5e6[ZXDOWN_A5E6_POS6] = pdata[2];  // �������ذٷֱ�
    zxdown_buffer_a5ea[ZXDOWN_A5EA_POS7] = pdata[6]; // �����Ƶ���
    zxdown_buffer_a5ea[ZXDOWN_A5EA_POS7+1] = pdata[7];
    tlv_a5e1_valid_flag = 1;
    tlv_a5e6_valid_flag = 1;
    tlv_a5e7_valid_flag = 1;
    tlv_a5ea_valid_flag = 1;
    retval = 0x01;
    break;
    
  //==A5E2====================================================================================
  case 0x24F:
    zxdown_buffer_a5e2[ZXDOWN_A5E2_POS1] = pdata[7]; // ֧�ȴ�ֱ״̬λ
    zxdown_buffer_a5e2[ZXDOWN_A5E2_POS2] = pdata[5]; // ֧��ˮƽ״̬λ
    
    if (pdata[6]&BIT(2)) // ��ǰ�ڱ���
      zxdown_buffer_a5e2[ZXDOWN_A5E2_POS3] |= BIT(0); // ��1
    else
      zxdown_buffer_a5e2[ZXDOWN_A5E2_POS3] &= ~BIT(0); // ��0
    
    if (pdata[6]&BIT(3)) // ��ǰ�ڱ���
      zxdown_buffer_a5e2[ZXDOWN_A5E2_POS3] |= BIT(1); // ��1
    else
      zxdown_buffer_a5e2[ZXDOWN_A5E2_POS3] &= ~BIT(1); // ��0
    
    if (pdata[6]&BIT(0)) // ��ǰ�ڱ���
      zxdown_buffer_a5e2[ZXDOWN_A5E2_POS3] |= BIT(2); // ��1
    else
      zxdown_buffer_a5e2[ZXDOWN_A5E2_POS3] &= ~BIT(2); // ��0
    
    if (pdata[6]&BIT(1)) // ��ǰ�ڱ���
      zxdown_buffer_a5e2[ZXDOWN_A5E2_POS2] |= BIT(3); // ��1
    else
      zxdown_buffer_a5e2[ZXDOWN_A5E2_POS2] &= ~BIT(3); // ��0
    
    if (pdata[6]&BIT(6)) // �Һ�ڱ���
      zxdown_buffer_a5e2[ZXDOWN_A5E2_POS2] |= BIT(4); // ��1
    else
      zxdown_buffer_a5e2[ZXDOWN_A5E2_POS2] &= ~BIT(4); // ��0
    
    if (pdata[6]&BIT(7)) // �Һ�ڱ���
      zxdown_buffer_a5e2[ZXDOWN_A5E2_POS2] |= BIT(5); // ��1
    else
      zxdown_buffer_a5e2[ZXDOWN_A5E2_POS2] &= ~BIT(5); // ��0
    
    if (pdata[6]&BIT(4)) // ���ڱ���
      zxdown_buffer_a5e2[ZXDOWN_A5E2_POS2] |= BIT(6); // ��1
    else
      zxdown_buffer_a5e2[ZXDOWN_A5E2_POS2] &= ~BIT(6); // ��0
    
    if (pdata[6]&BIT(5)) // ���ڱ���
      zxdown_buffer_a5e2[ZXDOWN_A5E2_POS2] |= BIT(7); // ��1
    else
      zxdown_buffer_a5e2[ZXDOWN_A5E2_POS2] &= ~BIT(7); // ��0
    
    tlv_a5e2_valid_flag = 1;
    retval = 0x01;
    break;
    
  //==A5E3====================================================================================
  case 0x22B:
    memcpy(&zxdown_buffer_a5e3[ZXDOWN_A5E3_POS1], pdata, 8); // ����ѹ��
    tlv_a5e3_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x22C:
    memcpy(&zxdown_buffer_a5e3[ZXDOWN_A5E3_POS2], pdata, 8); // �����г�
    tlv_a5e3_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x21E:
    memcpy(&zxdown_buffer_a5e3[ZXDOWN_A5E3_POS3], pdata, 4); // ��������
    tlv_a5e3_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x25F:
    if (pdata[1]&BIT(0)) // ��ǰ�ڱ���
      zxdown_buffer_a5e3[ZXDOWN_A5E3_POS4] |= BIT(0); // ��1
    else
      zxdown_buffer_a5e3[ZXDOWN_A5E3_POS4] &= ~BIT(0); // ��0

    if (pdata[1]&BIT(1)) // ��ǰ�ڱ���
      zxdown_buffer_a5e3[ZXDOWN_A5E3_POS4] |= BIT(1); // ��1
    else
      zxdown_buffer_a5e3[ZXDOWN_A5E3_POS4] &= ~BIT(1); // ��0
    
    if (pdata[0]&BIT(0)) // ��ǰ�ڱ���
      zxdown_buffer_a5e3[ZXDOWN_A5E3_POS4] |= BIT(2); // ��1
    else
      zxdown_buffer_a5e3[ZXDOWN_A5E3_POS4] &= ~BIT(2); // ��0

    if (pdata[0]&BIT(1)) // ��ǰ�ڱ���
      zxdown_buffer_a5e3[ZXDOWN_A5E3_POS4] |= BIT(3); // ��1
    else
      zxdown_buffer_a5e3[ZXDOWN_A5E3_POS4] &= ~BIT(3); // ��0
    
    if (pdata[0]&BIT(2)) // ��ǰ�ڱ���
      zxdown_buffer_a5e3[ZXDOWN_A5E3_POS4] |= BIT(4); // ��1
    else
      zxdown_buffer_a5e3[ZXDOWN_A5E3_POS4] &= ~BIT(4); // ��0
    
    if (pdata[0]&BIT(3)) // ��ǰ�ڱ���
      zxdown_buffer_a5e3[ZXDOWN_A5E3_POS4] |= BIT(5); // ��1
    else
      zxdown_buffer_a5e3[ZXDOWN_A5E3_POS4] &= ~BIT(5); // ��0
    
    if (pdata[0]&BIT(4)) // ��ǰ�ڱ���
      zxdown_buffer_a5e3[ZXDOWN_A5E3_POS4] |= BIT(6); // ��1
    else
      zxdown_buffer_a5e3[ZXDOWN_A5E3_POS4] &= ~BIT(6); // ��0
    
    if (pdata[0]&BIT(5)) // ��ǰ�ڱ���
      zxdown_buffer_a5e3[ZXDOWN_A5E3_POS4] |= BIT(7); // ��1
    else
      zxdown_buffer_a5e3[ZXDOWN_A5E3_POS4] &= ~BIT(7); // ��0
    
    if (pdata[0]&BIT(6)) // ��ǰ�ڱ���
      zxdown_buffer_a5e3[ZXDOWN_A5E3_POS5] |= BIT(0); // ��1
    else
      zxdown_buffer_a5e3[ZXDOWN_A5E3_POS5] &= ~BIT(0); // ��0
    
    if (pdata[0]&BIT(7)) // ��ǰ�ڱ���
      zxdown_buffer_a5e3[ZXDOWN_A5E3_POS5] |= BIT(1); // ��1
    else
      zxdown_buffer_a5e3[ZXDOWN_A5E3_POS5] &= ~BIT(1); // ��0
    
    tlv_a5e3_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5E4====================================================================================
  case 0x22D:
    memcpy(&zxdown_buffer_a5e4[ZXDOWN_A5E4_POS1], pdata, 8); // һ����ת��Ƕ�
    tlv_a5e4_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x22E:
    zxdown_buffer_a5e4[ZXDOWN_A5E4_POS2] = pdata[0];  // �������ת��Ƕ�
    zxdown_buffer_a5e4[ZXDOWN_A5E4_POS2+1] = pdata[1];
    zxdown_buffer_a5e4[ZXDOWN_A5E4_POS2+2] = pdata[2]; // �������ת��Ƕ�
    zxdown_buffer_a5e4[ZXDOWN_A5E4_POS2+3] = pdata[3];
    tlv_a5e4_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x25A:
    zxdown_buffer_a5e4[ZXDOWN_A5E4_POS2+4] = pdata[0]; // �������ת��Ƕ�
    zxdown_buffer_a5e4[ZXDOWN_A5E4_POS2+5] = pdata[1];
    zxdown_buffer_a5e4[ZXDOWN_A5E4_POS2+6] = pdata[2]; // �������ת��Ƕ�
    zxdown_buffer_a5e4[ZXDOWN_A5E4_POS2+7] = pdata[3];
    tlv_a5e4_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x2BB:
    memcpy(&zxdown_buffer_a5e4[ZXDOWN_A5E4_POS3], pdata, 8); // һ���ᴫ��������
    tlv_a5e4_valid_flag = 1;
    retval = 0x01;
    break;    

  case 0x2BC:
    memcpy(&zxdown_buffer_a5e4[ZXDOWN_A5E4_POS4], pdata, 8); // ���������ᴫ��������
    tlv_a5e4_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x25C:
    zxdown_buffer_a5e4[ZXDOWN_A5E4_POS5] = pdata[1];  // ��ǰת��ģʽ
    zxdown_buffer_a5e4[ZXDOWN_A5E4_POS6] = pdata[0];  // Ŀ��ת��ģʽ
    zxdown_buffer_a5e4[ZXDOWN_A5E4_POS7] = pdata[2];  // ת��ϵͳѹ��
    zxdown_buffer_a5e4[ZXDOWN_A5E4_POS7+1] = pdata[3];
    zxdown_buffer_a5e4[ZXDOWN_A5E4_POS8] = pdata[4];  // ������ѹ������
    zxdown_buffer_a5e4[ZXDOWN_A5E4_POS8+1] = pdata[5];
    zxdown_buffer_a5e9[ZXDOWN_A5E9_POS2] = pdata[6]; // Һѹϵͳѹ��
    zxdown_buffer_a5e9[ZXDOWN_A5E9_POS2+1] = pdata[7];
    tlv_a5e4_valid_flag = 1;
    tlv_a5e9_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x2BD:
    zxdown_buffer_a5e4[ZXDOWN_A5E4_POS11] = pdata[0];  // ����ֹ��
    memcpy(&zxdown_buffer_a5e4[ZXDOWN_A5E4_POS9], &pdata[2], 6); // 123������ת��ռ�ձ�
    tlv_a5e4_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x2BE:
    memcpy(&zxdown_buffer_a5e4[ZXDOWN_A5E4_POS10], pdata, 6); // 456������ת��ռ�ձ�
    tlv_a5e4_valid_flag = 1;
    retval = 0x01;
    break;
  
  //==A5E6====================================================================================
  case 0x25D:
    zxdown_buffer_a5e6[ZXDOWN_A5E6_POS1] = pdata[0];  // ��·һ��ѹ
    zxdown_buffer_a5e6[ZXDOWN_A5E6_POS1+1] = pdata[1];
    zxdown_buffer_a5e6[ZXDOWN_A5E6_POS2] = pdata[2];  // ��·һ��ѹ
    zxdown_buffer_a5e6[ZXDOWN_A5E6_POS2+1] = pdata[3];
    zxdown_buffer_a5e6[ZXDOWN_A5E6_POS3] = pdata[4];  // ��·һ��ѹ
    zxdown_buffer_a5e6[ZXDOWN_A5E6_POS3+1] = pdata[5];
    zxdown_buffer_a5e6[ZXDOWN_A5E6_POS4] = pdata[6];  // ��·һ��ѹ
    zxdown_buffer_a5e6[ZXDOWN_A5E6_POS4+1] = pdata[7];
    tlv_a5e6_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x23A:
    zxdown_buffer_a5e6[ZXDOWN_A5E6_POS7] = pdata[4];  // ���������
    zxdown_buffer_a5e6[ZXDOWN_A5E6_POS7+1] = pdata[5];
    tlv_a5e6_valid_flag = 1;
    retval = 0x01;
    break;
    
  //==A5E7====================================================================================
  // ��ɢ
  //==A5E8====================================================================================
  // ��ɢ
  //==A5E9====================================================================================
  // ��ɢ
    
  //==A5EA====================================================================================
  case 0x26A:
    if (pdata[0]&BIT(0)) // �ϳ�Һѹ����ȷ����Ϣ
      zxdown_buffer_a5ea[ZXDOWN_A5EA_POS1] |= BIT(0); // ��1
    else
      zxdown_buffer_a5ea[ZXDOWN_A5EA_POS1] &= ~BIT(0); // ��0
    
    zxdown_buffer_a5ea[ZXDOWN_A5EA_POS3] = pdata[1]; // �ϳ�������ˮ��
    zxdown_buffer_a5ea[ZXDOWN_A5EA_POS4] = pdata[2]; // �ϳ�����������ѹ��
    zxdown_buffer_a5ea[ZXDOWN_A5EA_POS4+1] = pdata[3];
    zxdown_buffer_a5ea[ZXDOWN_A5EA_POS5] = pdata[4]; // �ϳ�������ת��
    zxdown_buffer_a5ea[ZXDOWN_A5EA_POS5+1] = pdata[5];
    zxdown_buffer_a5ea[ZXDOWN_A5EA_POS6] = pdata[6]; // �ϳ��ó���ѹ��
    zxdown_buffer_a5ea[ZXDOWN_A5EA_POS6+1] = pdata[7];
    tlv_a5ea_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5EB====================================================================================
  case 0x28C:
    if(pdata[0]>0 && pdata[0]<19) // 1����̥��18����̥
    {
      zxdown_buffer_a5eb[ZXDOWN_A5EB_POS1+pdata[0]-1] = pdata[1];
      tlv_a5eb_valid_flag = 1;
    }
    retval = 0x01;
    break;

  //==A5EC====================================================================================
  case 0x2AB:
    zxdown_buffer_a5ec[ZXDOWN_A5EC_POS1] = pdata[0]; // ��֧����� - �ֽ�״̬λ1
    zxdown_buffer_a5ec[ZXDOWN_A5EC_POS2] = pdata[1]; // ��֧����� - �ֽ�״̬λ2
    zxdown_buffer_a5ec[ZXDOWN_A5EC_POS3] = pdata[2]; // ��֧����� - �ֽ�״̬λ3
    zxdown_buffer_a5ec[ZXDOWN_A5EC_POS4] = pdata[3]; // ��֧����� - �ֽ�״̬λ4
    zxdown_buffer_a5ec[ZXDOWN_A5EC_POS5] = pdata[4]; // ��֧����� - �ֽ�״̬λ5
    zxdown_buffer_a5ec[ZXDOWN_A5EC_POS6] = pdata[5]; // ��֧����� - Ԥ��
    zxdown_buffer_a5ec[ZXDOWN_A5EC_POS7] = pdata[6]; // ��֧����� - DIR
    zxdown_buffer_a5ec[ZXDOWN_A5EC_POS8] = pdata[7]; // ��֧����� - ��ƽ������
    tlv_a5ec_valid_flag = 1;
    retval = 0x01;
    break;
    
  //==A5ED====================================================================================
  case 0x2AC:
    zxdown_buffer_a5ed[ZXDOWN_A5ED_POS1] = pdata[0]; // ��֧����� - �ֽ�״̬λ1
    zxdown_buffer_a5ed[ZXDOWN_A5ED_POS2] = pdata[1]; // ��֧����� - �ֽ�״̬λ2
    zxdown_buffer_a5ed[ZXDOWN_A5ED_POS3] = pdata[2]; // ��֧����� - �ֽ�״̬λ3
    zxdown_buffer_a5ed[ZXDOWN_A5ED_POS4] = pdata[3]; // ��֧����� - �ֽ�״̬λ4
    zxdown_buffer_a5ed[ZXDOWN_A5ED_POS5] = pdata[4]; // ��֧����� - �ֽ�״̬λ5
    zxdown_buffer_a5ed[ZXDOWN_A5ED_POS6] = pdata[5]; // ��֧����� - Ԥ��
    zxdown_buffer_a5ed[ZXDOWN_A5ED_POS7] = pdata[6]; // ��֧����� - DIR
    zxdown_buffer_a5ed[ZXDOWN_A5ED_POS8] = pdata[7]; // ��֧����� - ��ƽ������
    tlv_a5ed_valid_flag = 1;
    retval = 0x01;
    break;
    
  //==A5EE====================================================================================
  case 0x2AD:
    zxdown_buffer_a5ee[ZXDOWN_A5EE_POS1] = pdata[0]; // �п�̨������Ϣ - �ֽ�״̬λ1
    zxdown_buffer_a5ee[ZXDOWN_A5EE_POS2] = pdata[1]; // �п�̨������Ϣ - �ֽ�״̬λ2
    tlv_a5ee_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x2AE:
    zxdown_buffer_a5ee[ZXDOWN_A5EE_POS3] = pdata[0]; // �п�̨������Ϣ - �ֽ�״̬λ3
    zxdown_buffer_a5ee[ZXDOWN_A5EE_POS4] = pdata[1]; // �п�̨������Ϣ - �ֽ�״̬λ4
    tlv_a5ee_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x2AF:
    zxdown_buffer_a5ee[ZXDOWN_A5EE_POS5] = pdata[0]; // �п�̨������Ϣ - �ֽ�״̬λ5
    zxdown_buffer_a5ee[ZXDOWN_A5EE_POS6] = pdata[1]; // �п�̨������Ϣ - �ֽ�״̬λ6
    zxdown_buffer_a5ee[ZXDOWN_A5EE_POS7] = pdata[2]; // �п�̨������Ϣ - �ֽ�״̬λ7
    zxdown_buffer_a5ee[ZXDOWN_A5EE_POS8] = pdata[3]; // ����ת��Ƕ�
    zxdown_buffer_a5ee[ZXDOWN_A5EE_POS9] = pdata[4]; // �ֽ�״̬λ8
    tlv_a5ee_valid_flag = 1;
    retval = 0x01;
    break;

  //==�³�ϵͳ�汾====================================================================================
  case 0x20F:
    zxversion_buffer_a506[ZXVERSION_A506_POS3] = pdata[0];  // P1Ӧ�ò�
    zxversion_buffer_a506[ZXVERSION_A506_POS3+1] = pdata[1];
    zxversion_buffer_a506[ZXVERSION_A506_POS3+2] = pdata[2];
    zxversion_buffer_a506[ZXVERSION_A506_POS5] = pdata[3];  // P2Ӧ�ò�
    zxversion_buffer_a506[ZXVERSION_A506_POS5+1] = pdata[4];
    zxversion_buffer_a506[ZXVERSION_A506_POS5+2] = pdata[5];
    zxversion_buffer_a506[ZXVERSION_A506_POS7] = pdata[6];  // P3Ӧ�ò�
    zxversion_buffer_a506[ZXVERSION_A506_POS7+1] = pdata[7];
    tlv_a506_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x21F:
    zxversion_buffer_a506[ZXVERSION_A506_POS7+2] = pdata[0];  // P3Ӧ�ò�
    zxversion_buffer_a506[ZXVERSION_A506_POS9] = pdata[1];    // P4Ӧ�ò�
    zxversion_buffer_a506[ZXVERSION_A506_POS9+1] = pdata[2];
    zxversion_buffer_a506[ZXVERSION_A506_POS9+2] = pdata[3];
    zxversion_buffer_a506[ZXVERSION_A506_POS11] = pdata[4];   // P5Ӧ�ò�
    zxversion_buffer_a506[ZXVERSION_A506_POS11+1] = pdata[5];
    zxversion_buffer_a506[ZXVERSION_A506_POS11+2] = pdata[6];
    zxversion_buffer_a506[ZXVERSION_A506_POS17] = pdata[7];   // P8Ӧ�ò�
    tlv_a506_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x22F:
    zxversion_buffer_a506[ZXVERSION_A506_POS17+1] = pdata[0];   // P8Ӧ�ò�
    zxversion_buffer_a506[ZXVERSION_A506_POS17+2] = pdata[1];
    tlv_a506_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x29F:
    if(pdata[0]==1)
    {
      zxversion_buffer_a506[ZXVERSION_A506_POS4] = pdata[1];  // P1�ײ�
      zxversion_buffer_a506[ZXVERSION_A506_POS4+1] = pdata[2];
      zxversion_buffer_a506[ZXVERSION_A506_POS4+2] = pdata[3];
      tlv_a506_valid_flag = 1;
    }
    else if(pdata[0]==2)
    {
      zxversion_buffer_a506[ZXVERSION_A506_POS6] = pdata[1];  // P2�ײ�
      zxversion_buffer_a506[ZXVERSION_A506_POS6+1] = pdata[2];
      zxversion_buffer_a506[ZXVERSION_A506_POS6+2] = pdata[3];
      tlv_a506_valid_flag = 1;
    }
    else if(pdata[0]==3)
    {
      zxversion_buffer_a506[ZXVERSION_A506_POS8] = pdata[1];  // P3�ײ�
      zxversion_buffer_a506[ZXVERSION_A506_POS8+1] = pdata[2];
      zxversion_buffer_a506[ZXVERSION_A506_POS8+2] = pdata[3];
      tlv_a506_valid_flag = 1;
    }
    else if(pdata[0]==4)
    {
      zxversion_buffer_a506[ZXVERSION_A506_POS10] = pdata[1];  // P4�ײ�
      zxversion_buffer_a506[ZXVERSION_A506_POS10+1] = pdata[2];
      zxversion_buffer_a506[ZXVERSION_A506_POS10+2] = pdata[3];
      tlv_a506_valid_flag = 1;
    }
    else if(pdata[0]==5)
    {
      zxversion_buffer_a506[ZXVERSION_A506_POS12] = pdata[1];  // P5�ײ�
      zxversion_buffer_a506[ZXVERSION_A506_POS12+1] = pdata[2];
      zxversion_buffer_a506[ZXVERSION_A506_POS12+2] = pdata[3];
      tlv_a506_valid_flag = 1;
    }
    else if(pdata[0]==6)
    {
      zxversion_buffer_a506[ZXVERSION_A506_POS14] = pdata[1];  // P6�ײ�
      zxversion_buffer_a506[ZXVERSION_A506_POS14+1] = pdata[2];
      zxversion_buffer_a506[ZXVERSION_A506_POS14+2] = pdata[3];
      tlv_a506_valid_flag = 1;
    }
    else if(pdata[0]==7)
    {
      zxversion_buffer_a506[ZXVERSION_A506_POS16] = pdata[1];  // P7�ײ�
      zxversion_buffer_a506[ZXVERSION_A506_POS16+1] = pdata[2];
      zxversion_buffer_a506[ZXVERSION_A506_POS16+2] = pdata[3];
      tlv_a506_valid_flag = 1;
    }
    else if(pdata[0]==8)
    {
      zxversion_buffer_a506[ZXVERSION_A506_POS18] = pdata[1];  // P8�ײ�
      zxversion_buffer_a506[ZXVERSION_A506_POS18+1] = pdata[2];
      zxversion_buffer_a506[ZXVERSION_A506_POS18+2] = pdata[3];
      tlv_a506_valid_flag = 1;
    }
    retval = 0x01;
    break;
  
  default:
    retval = 0x00;
    break;
  }
  
  return retval;
}

/*************************************************************************
 * ������յ���CAN����(�������̷�����)
*************************************************************************/
uint8_t CAN_ProcessRecvEngineMsg_AC(uint32_t canId, uint8_t *pdata, uint8_t size)
{
  uint8_t retval = 0x00;
  //uint32_t tempVal;

  switch (canId) // �ϳ�ͨ��
  {
  //==A5EF===================================================================================
  case 0x0CF00300:
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS11] = pdata[1]; // ����̤��ٷֱ�
    tlv_a5ef_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x0CF00400:
    if (pdata[0]&BIT(0)) // ������Ť��ģʽ1
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS22] |= BIT(0); // ��1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS22] &= ~BIT(0); // ��0
    
    if (pdata[0]&BIT(1)) // ������Ť��ģʽ2
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS22] |= BIT(1); // ��1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS22] &= ~BIT(1); // ��0
    
    if (pdata[0]&BIT(2)) // ������Ť��ģʽ3
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS22] |= BIT(2); // ��1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS22] &= ~BIT(2); // ��0
    
    if (pdata[0]&BIT(3)) // ������Ť��ģʽ4
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS22] |= BIT(3); // ��1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS22] &= ~BIT(3); // ��0

    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS1] = pdata[3]; // ������ת��
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS1+1] = pdata[4];
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS2] = pdata[2]; // ʵ��Ť�ذٷֱ�
    tlv_a5ef_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x18FEEE00:
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS6] = pdata[0]; // ��ȴҺ�¶�
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS7] = pdata[2]; // �����¶�
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS7+1] = pdata[3];
    tlv_a5ef_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x18FEEF00:
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS8] = pdata[2]; // ����Һλ
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS9] = pdata[3]; // ����ѹ��
    tlv_a5ef_valid_flag = 1;
    retval = 0x01;
    break;
 
  case 0x18FEF600:
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS4] = pdata[1]; // �������ѹ��
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS5] = pdata[2]; // ��������¶�
    tlv_a5ef_valid_flag = 1;
    retval = 0x01;
    break;
   
  case 0x18FEE500:
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS10] = pdata[0]; // ������������ʱ��
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS10+1] = pdata[1];
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS10+2] = pdata[2];
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS10+3] = pdata[3];
    tlv_a5ef_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x18FEDF00:
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS3] = pdata[0]; // Ħ��Ť�ذٷֱ�
    tlv_a5ef_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x18FEF100:
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS13] = pdata[3];
    
    if (pdata[4]&BIT(0)) // Ѳ�����ÿ���1
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS14] |= BIT(0); // ��1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS14] &= ~BIT(0); // ��0
    
    if (pdata[4]&BIT(1)) // Ѳ�����ÿ���1
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS14] |= BIT(1); // ��1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS14] &= ~BIT(1); // ��0
    
    if (pdata[4]&BIT(2)) // Ѳ�����ٿ���1
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS14] |= BIT(2); // ��1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS14] &= ~BIT(2); // ��0
    
    if (pdata[4]&BIT(3)) // Ѳ�����ٿ���2
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS14] |= BIT(3); // ��1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS14] &= ~BIT(3); // ��0
    
    if (pdata[4]&BIT(6)) // Ѳ�����Ƽ��ٿ���1
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS14] |= BIT(4); // ��1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS14] &= ~BIT(4); // ��0
    
    if (pdata[4]&BIT(7)) // Ѳ�����Ƽ��ٿ���2
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS14] |= BIT(5); // ��1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS14] &= ~BIT(5); // ��0
    
    if (pdata[6]&BIT(5)) // Ѳ������״̬1
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS23] |= BIT(4); // ��1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS23] &= ~BIT(4); // ��0

    if (pdata[6]&BIT(6)) // Ѳ������״̬2
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS23] |= BIT(5); // ��1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS23] &= ~BIT(5); // ��0

    if (pdata[6]&BIT(7)) // Ѳ������״̬3
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS23] |= BIT(6); // ��1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS23] &= ~BIT(6); // ��0

    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS12] = pdata[1]; // ����
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS12+1] = pdata[2];
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS15] = pdata[5]; // Ѳ���趨�ٶ�
    tlv_a5ef_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x18FEF200:
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS16] = pdata[0]; // ������ȼ��������
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS16+1] = pdata[1];
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS17] = pdata[4]; // ������ƽ��ȼ��������
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS17+1] = pdata[5];
    tlv_a5ef_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x18FEE900:
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS18] = pdata[4]; // ȼ�����ͺ���
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS18+1] = pdata[5];
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS18+2] = pdata[6];
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS18+3] = pdata[7];
    tlv_a5ef_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x18FEE000:
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS19] = pdata[4]; // ����ʻ���
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS19+1] = pdata[5];
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS19+2] = pdata[6];
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS19+3] = pdata[7];
    tlv_a5ef_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x18FEFC17:
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS20] = pdata[1]; // ȼ��Һλ1
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS21] = pdata[6]; // ȼ��Һλ2
    tlv_a5ef_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x18FD0700:
    if (pdata[0]&BIT(2)) // ��ʻԱ������DWL-1
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS23] |= BIT(0); // ��1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS23] &= ~BIT(0); // ��0
    
    if (pdata[0]&BIT(3)) // ��ʻԱ������DWL-2
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS23] |= BIT(1); // ��1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS23] &= ~BIT(1); // ��0
    
    tlv_a5ef_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x18FEFF00:
    if (pdata[0]&BIT(0)) // ������ˮָʾ��-1
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS23] |= BIT(2); // ��1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS23] &= ~BIT(2); // ��0

    if (pdata[0]&BIT(1)) // ������ˮָʾ��-2
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS23] |= BIT(3); // ��1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS23] &= ~BIT(3); // ��0

    tlv_a5ef_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x18F00A00:
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS24] = pdata[2];  // ��������
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS24+1] = pdata[3];
    tlv_a5ef_valid_flag = 1;
    retval = 0x01;
    break;
  
  //==A5F0===================================================================================
  //GPS���㣬���ݷ�����PTO�������³�����

  //==A502===================================================================================
  case 0x18FEF500:
    zxengine_buffer_a502[ZXENGINE_A502_POS1_ADDR] = pdata[0];// ����ѹ��
    zxengine_buffer_a502[ZXENGINE_A502_POS2_ADDR] = pdata[3];// �����¶�
    zxengine_buffer_a502[ZXENGINE_A502_POS2_ADDR+1] = pdata[4];
    tlv_a502_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5F1===================================================================================
  case 0x18FF203D:
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS1] = pdata[0]; // ��������״̬
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS2] = pdata[5]; // T15_DCU
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS3] = pdata[6]; // ���ر�ѹ��
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS3+1] = pdata[7];
    tlv_a5f1_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x18FE563D:
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS4] = pdata[0]; // ������Һλ
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS5] = pdata[1]; // �������¶�
    tlv_a5f1_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x0CF0233D:
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS6] = pdata[6]; // ����������
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS6+1] = pdata[7];
    tlv_a5f1_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x18F00E51:
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS7] = pdata[0]; // SCR����NOxŨ��
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS7+1] = pdata[1];
    tlv_a5f1_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x18F00F52:
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS8] = pdata[0]; // SCR����NOxŨ��
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS8+1] = pdata[1];
    tlv_a5f1_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x18FD3E3D:
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS9] = pdata[0]; // SCR���������¶�(T6�¶�)
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS9+1] = pdata[1];
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS10] = pdata[3]; // SCR���������¶�(T7�¶�)
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS10+1] = pdata[4];
    tlv_a5f1_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x18FD9BA3:
    if (pdata[3]&BIT(0)) // Ʒ���¶ȴ�����FMI (SPN 3519)1
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS14] |= BIT(0); // ��1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS14] &= ~BIT(0); // ��0

    if (pdata[3]&BIT(1)) // Ʒ���¶ȴ�����FMI (SPN 3519)2
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS14] |= BIT(1); // ��1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS14] &= ~BIT(1); // ��0

    if (pdata[3]&BIT(2)) // Ʒ���¶ȴ�����FMI (SPN 3519)3
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS14] |= BIT(2); // ��1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS14] &= ~BIT(2); // ��0

    if (pdata[3]&BIT(3)) // Ʒ���¶ȴ�����FMI (SPN 3519)4
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS14] |= BIT(3); // ��1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS14] &= ~BIT(3); // ��0

    if (pdata[3]&BIT(4)) // Ʒ���¶ȴ�����FMI (SPN 3519)5
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS14] |= BIT(4); // ��1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS14] &= ~BIT(4); // ��0

    if (pdata[4]&BIT(0)) // Ʒ�ʴ�����FMI (SPN3520)1
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS15] |= BIT(0); // ��1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS15] &= ~BIT(0); // ��0

    if (pdata[4]&BIT(1)) // Ʒ�ʴ�����FMI (SPN3520)2
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS15] |= BIT(1); // ��1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS15] &= ~BIT(1); // ��0

    if (pdata[4]&BIT(2)) // Ʒ�ʴ�����FMI (SPN3520)3
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS15] |= BIT(2); // ��1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS15] &= ~BIT(2); // ��0

    if (pdata[4]&BIT(3)) // Ʒ�ʴ�����FMI (SPN3520)4
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS15] |= BIT(3); // ��1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS15] &= ~BIT(3); // ��0

    if (pdata[4]&BIT(4)) // Ʒ�ʴ�����FMI (SPN3520)5
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS15] |= BIT(4); // ��1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS15] &= ~BIT(4); // ��0

    if (pdata[5]&BIT(0)) // �߻����Լ�����(SPN3521)1
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS16] |= BIT(0); // ��1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS16] &= ~BIT(0); // ��0

    if (pdata[5]&BIT(1)) // �߻����Լ�����(SPN3521)2
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS16] |= BIT(1); // ��1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS16] &= ~BIT(1); // ��0

    if (pdata[5]&BIT(2)) // �߻����Լ�����(SPN3521)3
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS16] |= BIT(2); // ��1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS16] &= ~BIT(2); // ��0

    if (pdata[5]&BIT(3)) // �߻����Լ�����(SPN3521)4
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS16] |= BIT(3); // ��1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS16] &= ~BIT(3); // ��0

    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS13] = pdata[1]; // ����Ʒ�ʴ������¶�(SPN 3515)
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS11] = pdata[1]; // ����Ũ��(SPN 3516)
    tlv_a5f1_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x18FCBD3D:
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS12] = pdata[4]; // �ۼ�����������
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS12+1] = pdata[5];
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS12+2] = pdata[6];
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS12+3] = pdata[7];
    tlv_a5f1_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x18FE56A3:
    if (pdata[4]&BIT(0)) // ������Һλ������ʧЧģʽFMI-1
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS17] |= BIT(0); // ��1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS17] &= ~BIT(0); // ��0

    if (pdata[4]&BIT(1)) // ������Һλ������ʧЧģʽFMI-2
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS17] |= BIT(1); // ��1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS17] &= ~BIT(1); // ��0

    if (pdata[4]&BIT(2)) // ������Һλ������ʧЧģʽFMI-3
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS17] |= BIT(2); // ��1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS17] &= ~BIT(2); // ��0

    if (pdata[4]&BIT(3)) // ������Һλ������ʧЧģʽFMI-4
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS17] |= BIT(3); // ��1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS17] &= ~BIT(3); // ��0

    if (pdata[4]&BIT(4)) // ������Һλ������ʧЧģʽFMI-5
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS17] |= BIT(4); // ��1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS17] &= ~BIT(4); // ��0

    if (pdata[5]&BIT(0)) // �������¶ȴ�����ʧЧģʽFMI-1
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS18] |= BIT(0); // ��1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS18] &= ~BIT(0); // ��0

    if (pdata[5]&BIT(1)) // �������¶ȴ�����ʧЧģʽFMI-2
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS18] |= BIT(1); // ��1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS18] &= ~BIT(1); // ��0

    if (pdata[5]&BIT(2)) // �������¶ȴ�����ʧЧģʽFMI-3
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS18] |= BIT(2); // ��1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS18] &= ~BIT(2); // ��0

    if (pdata[5]&BIT(3)) // �������¶ȴ�����ʧЧģʽFMI-4
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS18] |= BIT(3); // ��1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS18] &= ~BIT(3); // ��0

    if (pdata[5]&BIT(4)) // �������¶ȴ�����ʧЧģʽFMI-5
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS18] |= BIT(4); // ��1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS18] &= ~BIT(4); // ��0
    
    tlv_a5f1_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x18FEDF3D:
    if (pdata[7]&BIT(0)) // Nox������¶��״̬1
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS19] |= BIT(0); // ��1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS19] &= ~BIT(0); // ��0

    if (pdata[7]&BIT(1)) // Nox������¶��״̬2
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS19] |= BIT(1); // ��1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS19] &= ~BIT(1); // ��0

    tlv_a5f1_valid_flag = 1;
    retval = 0x01;
    break;
  
  //==A5F2===================================================================================
  case 0x18FD2000:
    zxengine_buffer_a5f2[ZXENGINE_A5F2_POS1] = pdata[0]; // DOC���������¶�
    zxengine_buffer_a5f2[ZXENGINE_A5F2_POS1+1] = pdata[1];
    zxengine_buffer_a5f2[ZXENGINE_A5F2_POS2] = pdata[2]; // DPF���������¶�
    zxengine_buffer_a5f2[ZXENGINE_A5F2_POS2+1] = pdata[3];
    tlv_a5f2_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x18FF1400:
    if (pdata[1]&BIT(0)) // DPF��������״̬1
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS5] |= BIT(0); // ��1
    else
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS5] &= ~BIT(0); // ��0

    if (pdata[1]&BIT(1)) // DPF��������״̬2
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS5] |= BIT(1); // ��1
    else
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS5] &= ~BIT(1); // ��0

    zxengine_buffer_a5f2[ZXENGINE_A5F2_POS3] = pdata[0]; // DPF̼����������
    tlv_a5f2_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x18FDB200:
    zxengine_buffer_a5f2[ZXENGINE_A5F2_POS4] = pdata[0]; // DPFѹ��
    zxengine_buffer_a5f2[ZXENGINE_A5F2_POS4+1] = pdata[1];
    tlv_a5f2_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x18FD7C00:
    if (pdata[0]&BIT(0)) // DPF����ָʾ��״̬1
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS6] |= BIT(0); // ��1
    else
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS6] &= ~BIT(0); // ��0

    if (pdata[0]&BIT(1)) // DPF����ָʾ��״̬2
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS6] |= BIT(1); // ��1
    else
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS6] &= ~BIT(1); // ��0

    if (pdata[0]&BIT(2)) // DPF����ָʾ��״̬3
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS6] |= BIT(2); // ��1
    else
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS6] &= ~BIT(2); // ��0

    if (pdata[2]&BIT(0)) // DPF������ֹ״̬1
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS7] |= BIT(0); // ��1
    else
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS7] &= ~BIT(0); // ��0

    if (pdata[2]&BIT(1)) // DPF������ֹ״̬2
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS7] |= BIT(1); // ��1
    else
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS7] &= ~BIT(1); // ��0
      
    tlv_a5f2_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x18E00021:
    if (pdata[5]&BIT(0)) // DPF��������״̬1
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS8] |= BIT(0); // ��1
    else
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS8] &= ~BIT(0); // ��0

    if (pdata[5]&BIT(1)) // DPF��������״̬2
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS8] |= BIT(1); // ��1
    else
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS8] &= ~BIT(1); // ��0

    if (pdata[5]&BIT(2)) // DPF������ֹ����״̬1
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS8] |= BIT(2); // ��1
    else
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS8] &= ~BIT(2); // ��0

    if (pdata[5]&BIT(3)) // DPF������ֹ����״̬2
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS8] |= BIT(3); // ��1
    else
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS8] &= ~BIT(3); // ��0

    tlv_a5f2_valid_flag = 1;
    retval = 0x01;
    break;
  
  default:
    retval = 0x00;
    break;
  }
  
  return retval;
}

/*************************************************************************
 * ������յ���CAN����(ȫ������̷�����)
*************************************************************************/
uint8_t CAN_ProcessRecvEngineMsg_AG(uint32_t canId, uint8_t *pdata, uint8_t size)
{
  uint8_t retval = 0x00;
  //uint32_t tempVal;

  switch (canId) // �³�ͨ��
  {
  //==A5EF���������в���(���ġ����塢����)===================================================
  case 0x20B:
    if (pdata[0]&BIT(0)) // ������Ť��ģʽ1
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS22] |= BIT(0); // ��1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS22] &= ~BIT(0); // ��0
    
    if (pdata[0]&BIT(1)) // ������Ť��ģʽ2
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS22] |= BIT(1); // ��1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS22] &= ~BIT(1); // ��0
    
    if (pdata[0]&BIT(2)) // ������Ť��ģʽ3
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS22] |= BIT(2); // ��1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS22] &= ~BIT(2); // ��0
    
    if (pdata[0]&BIT(3)) // ������Ť��ģʽ4
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS22] |= BIT(3); // ��1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS22] &= ~BIT(3); // ��0

    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS1] = pdata[3]; // ������ת��
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS1+1] = pdata[4];
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS2] = pdata[2]; // ʵ��Ť�ذٷֱ�
    tlv_a5ef_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x40D:
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS3] = pdata[7]; // Ħ��Ť�ذٷֱ�
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS16] = pdata[0]; // ������ȼ��������
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS16+1] = pdata[1];
    zxengine_buffer_a502[ZXENGINE_A502_POS1_ADDR] = pdata[4];// ����ѹ��
    zxengine_buffer_a502[ZXENGINE_A502_POS2_ADDR] = pdata[5];// �����¶�
    zxengine_buffer_a502[ZXENGINE_A502_POS2_ADDR+1] = pdata[6];
    tlv_a502_valid_flag = 1;
    tlv_a5ef_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x20E:
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS4] = pdata[1]; // �������ѹ��
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS5] = pdata[2]; // ��������¶�
    tlv_a5ef_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x20C:
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS6] = pdata[0]; // ��ȴҺ�¶�
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS7] = pdata[2]; // �����¶�
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS7+1] = pdata[3];
    tlv_a5ef_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x20D:
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS8] = pdata[2]; // ����Һλ
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS9] = pdata[3]; // ����ѹ��
    tlv_a5ef_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x40B:
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS10] = pdata[0]; // ������������ʱ��
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS10+1] = pdata[1];
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS10+2] = pdata[2];
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS10+3] = pdata[3];
    tlv_a5ef_valid_flag = 1;
    retval = 0x01;
    break;   
    
  case 0x20A:
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS11] = pdata[1]; // ����̤��ٷֱ�
    tlv_a5ef_valid_flag = 1;
    retval = 0x01;
    break; 
    
  case 0x40C:
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS13] = pdata[7]; // Ѳ��/ɲ��/���״̬
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS12] = pdata[4]; // ����
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS12+1] = pdata[5];
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS18] = pdata[0]; // ȼ�����ͺ���
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS18+1] = pdata[1];
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS18+2] = pdata[2];
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS18+3] = pdata[3];
    tlv_a5ef_valid_flag = 1;
    retval = 0x01;
    break;
 
  case 0x41A:
    if (pdata[4]&BIT(0)) // Ѳ�����ÿ���1
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS14] |= BIT(0); // ��1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS14] &= ~BIT(0); // ��0
    
    if (pdata[4]&BIT(1)) // Ѳ�����ÿ���2
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS14] |= BIT(1); // ��1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS14] &= ~BIT(1); // ��0
    
    if (pdata[4]&BIT(2)) // Ѳ�����ٿ���1
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS14] |= BIT(2); // ��1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS14] &= ~BIT(2); // ��0
    
    if (pdata[4]&BIT(3)) // Ѳ�����ٿ���2
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS14] |= BIT(3); // ��1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS14] &= ~BIT(3); // ��0
    
    if (pdata[4]&BIT(6)) // Ѳ�����Ƽ��ٿ���1
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS14] |= BIT(4); // ��1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS14] &= ~BIT(4); // ��0
    
    if (pdata[4]&BIT(7)) // Ѳ�����Ƽ��ٿ���2
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS14] |= BIT(5); // ��1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS14] &= ~BIT(5); // ��0
    
    if (pdata[6]&BIT(5)) // Ѳ������״̬1
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS23] |= BIT(4); // ��1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS23] &= ~BIT(4); // ��0

    if (pdata[6]&BIT(6)) // Ѳ������״̬2
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS23] |= BIT(5); // ��1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS23] &= ~BIT(5); // ��0

    if (pdata[6]&BIT(7)) // Ѳ������״̬3
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS23] |= BIT(6); // ��1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS23] &= ~BIT(6); // ��0
    
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS15] = pdata[5]; // Ѳ���趨�ٶ� 
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS20] = pdata[0]; // ȼ��Һλ1
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS21] = pdata[1]; // ȼ��Һλ2
    
    //===========================================================================
    if (pdata[2]&BIT(0)) // ������Һλ������ʧЧģʽFMI-1
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS17] |= BIT(0); // ��1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS17] &= ~BIT(0); // ��0

    if (pdata[2]&BIT(1)) // ������Һλ������ʧЧģʽFMI-2
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS17] |= BIT(1); // ��1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS17] &= ~BIT(1); // ��0

    if (pdata[2]&BIT(2)) // ������Һλ������ʧЧģʽFMI-3
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS17] |= BIT(2); // ��1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS17] &= ~BIT(2); // ��0

    if (pdata[2]&BIT(3)) // ������Һλ������ʧЧģʽFMI-4
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS17] |= BIT(3); // ��1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS17] &= ~BIT(3); // ��0

    if (pdata[2]&BIT(4)) // ������Һλ������ʧЧģʽFMI-5
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS17] |= BIT(4); // ��1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS17] &= ~BIT(4); // ��0

    if (pdata[3]&BIT(0)) // �������¶ȴ�����ʧЧģʽFMI-1
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS18] |= BIT(0); // ��1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS18] &= ~BIT(0); // ��0

    if (pdata[3]&BIT(1)) // �������¶ȴ�����ʧЧģʽFMI-2
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS18] |= BIT(1); // ��1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS18] &= ~BIT(1); // ��0

    if (pdata[3]&BIT(2)) // �������¶ȴ�����ʧЧģʽFMI-3
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS18] |= BIT(2); // ��1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS18] &= ~BIT(2); // ��0

    if (pdata[3]&BIT(3)) // �������¶ȴ�����ʧЧģʽFMI-4
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS18] |= BIT(3); // ��1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS18] &= ~BIT(3); // ��0

    if (pdata[3]&BIT(4)) // �������¶ȴ�����ʧЧģʽFMI-5
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS18] |= BIT(4); // ��1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS18] &= ~BIT(4); // ��0
    
    tlv_a5ef_valid_flag = 1;
    tlv_a5f1_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x42C:
    if (pdata[0]&BIT(0)) // ������ˮָʾ��-1
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS23] |= BIT(2); // ��1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS23] &= ~BIT(2); // ��0

    if (pdata[0]&BIT(1)) // ������ˮָʾ��-2
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS23] |= BIT(3); // ��1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS23] &= ~BIT(3); // ��0
    
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS17] = pdata[4]; // ������ƽ��ȼ��������
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS17+1] = pdata[5];
    tlv_a5ef_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x42B:
    if (pdata[0]&BIT(2)) // ��ʻԱ������DWL-1
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS23] |= BIT(0); // ��1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS23] &= ~BIT(0); // ��0
    
    if (pdata[0]&BIT(3)) // ��ʻԱ������DWL-2
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS23] |= BIT(1); // ��1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS23] &= ~BIT(1); // ��0
    
    tlv_a5ef_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x46C:
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS24] = pdata[2];  // ��������
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS24+1] = pdata[3];
    tlv_a5ef_valid_flag = 1;
    retval = 0x01;
    break;
  
  //==A5F0===================================================================================
  //GPS���㣬���ݷ�����PTO�������³�����
  
  //==A5F1(SCR����)==========================================================================
  case 0x41B:
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS1] = pdata[0]; // ��������״̬
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS2] = pdata[5]; // T15_DCU
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS3] = pdata[6]; // ���ر�ѹ��
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS3+1] = pdata[7];
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS12] = pdata[1]; // �ۼ�����������
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS12+1] = pdata[2];
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS12+2] = pdata[3];
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS12+3] = pdata[4];
    tlv_a5f1_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x40A:
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS4] = pdata[0]; // ������Һλ
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS5] = pdata[1]; // �������¶�
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS6] = pdata[2]; // ����������
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS6+1] = pdata[3];
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS19] = pdata[4]; // ����ʻ���
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS19+1] = pdata[5];
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS19+2] = pdata[6];
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS19+3] = pdata[7];
    tlv_a5ef_valid_flag = 1;
    tlv_a5f1_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x46A:
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS7] = pdata[0]; // SCR����NOxŨ��
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS7+1] = pdata[1];
    tlv_a5f1_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x46B:
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS8] = pdata[0]; // SCR����NOxŨ��
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS8+1] = pdata[1];
    tlv_a5f1_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x46D:
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS9] = pdata[0]; // SCR���������¶�(T6�¶�)
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS9+1] = pdata[1];
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS10] = pdata[3]; // SCR���������¶�(T7�¶�)
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS10+1] = pdata[4];
    tlv_a5f1_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x41C:
    if (pdata[3]&BIT(0)) // Ʒ���¶ȴ�����FMI (SPN 3519)1
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS14] |= BIT(0); // ��1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS14] &= ~BIT(0); // ��0

    if (pdata[3]&BIT(1)) // Ʒ���¶ȴ�����FMI (SPN 3519)2
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS14] |= BIT(1); // ��1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS14] &= ~BIT(1); // ��0

    if (pdata[3]&BIT(2)) // Ʒ���¶ȴ�����FMI (SPN 3519)3
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS14] |= BIT(2); // ��1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS14] &= ~BIT(2); // ��0

    if (pdata[3]&BIT(3)) // Ʒ���¶ȴ�����FMI (SPN 3519)4
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS14] |= BIT(3); // ��1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS14] &= ~BIT(3); // ��0

    if (pdata[3]&BIT(4)) // Ʒ���¶ȴ�����FMI (SPN 3519)5
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS14] |= BIT(4); // ��1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS14] &= ~BIT(4); // ��0

    if (pdata[4]&BIT(0)) // Ʒ�ʴ�����FMI (SPN3520)1
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS15] |= BIT(0); // ��1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS15] &= ~BIT(0); // ��0

    if (pdata[4]&BIT(1)) // Ʒ�ʴ�����FMI (SPN3520)2
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS15] |= BIT(1); // ��1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS15] &= ~BIT(1); // ��0

    if (pdata[4]&BIT(2)) // Ʒ�ʴ�����FMI (SPN3520)3
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS15] |= BIT(2); // ��1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS15] &= ~BIT(2); // ��0

    if (pdata[4]&BIT(3)) // Ʒ�ʴ�����FMI (SPN3520)4
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS15] |= BIT(3); // ��1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS15] &= ~BIT(3); // ��0

    if (pdata[4]&BIT(4)) // Ʒ�ʴ�����FMI (SPN3520)5
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS15] |= BIT(4); // ��1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS15] &= ~BIT(4); // ��0

    if (pdata[5]&BIT(0)) // �߻����Լ�����(SPN3521)1
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS16] |= BIT(0); // ��1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS16] &= ~BIT(0); // ��0

    if (pdata[5]&BIT(1)) // �߻����Լ�����(SPN3521)2
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS16] |= BIT(1); // ��1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS16] &= ~BIT(1); // ��0

    if (pdata[5]&BIT(2)) // �߻����Լ�����(SPN3521)3
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS16] |= BIT(2); // ��1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS16] &= ~BIT(2); // ��0

    if (pdata[5]&BIT(3)) // �߻����Լ�����(SPN3521)4
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS16] |= BIT(3); // ��1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS16] &= ~BIT(3); // ��0

    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS13] = pdata[1]; // ����Ʒ�ʴ������¶�(SPN 3515)
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS11] = pdata[1]; // ����Ũ��(SPN 3516)
    tlv_a5f1_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5F2===================================================================================
  case 0x41D:
    if (pdata[5]&BIT(0)) // DPF��������״̬1
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS5] |= BIT(0); // ��1
    else
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS5] &= ~BIT(0); // ��0

    if (pdata[5]&BIT(1)) // DPF��������״̬2
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS5] |= BIT(1); // ��1
    else
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS5] &= ~BIT(1); // ��0
    
    zxengine_buffer_a5f2[ZXENGINE_A5F2_POS1] = pdata[0]; // DOC���������¶�
    zxengine_buffer_a5f2[ZXENGINE_A5F2_POS1+1] = pdata[1];
    zxengine_buffer_a5f2[ZXENGINE_A5F2_POS2] = pdata[2]; // DPF���������¶�
    zxengine_buffer_a5f2[ZXENGINE_A5F2_POS2+1] = pdata[3];
    zxengine_buffer_a5f2[ZXENGINE_A5F2_POS3] = pdata[4]; // DPF̼����������
    zxengine_buffer_a5f2[ZXENGINE_A5F2_POS4] = pdata[6]; // DPFѹ��
    zxengine_buffer_a5f2[ZXENGINE_A5F2_POS4+1] = pdata[7];
    tlv_a5f2_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x42A:
    if (pdata[7]&BIT(0)) // Nox������¶��״̬1
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS19] |= BIT(0); // ��1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS19] &= ~BIT(0); // ��0

    if (pdata[7]&BIT(1)) // Nox������¶��״̬2
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS19] |= BIT(1); // ��1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS19] &= ~BIT(1); // ��0

    tlv_a5f1_valid_flag = 1;
  
    if (pdata[0]&BIT(0)) // DPF����ָʾ��״̬1
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS6] |= BIT(0); // ��1
    else
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS6] &= ~BIT(0); // ��0

    if (pdata[0]&BIT(1)) // DPF����ָʾ��״̬2
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS6] |= BIT(1); // ��1
    else
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS6] &= ~BIT(1); // ��0

    if (pdata[0]&BIT(2)) // DPF����ָʾ��״̬3
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS6] |= BIT(2); // ��1
    else
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS6] &= ~BIT(2); // ��0
 
    if (pdata[2]&BIT(0)) // DPF������ֹ״̬1
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS7] |= BIT(0); // ��1
    else
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS7] &= ~BIT(0); // ��0

    if (pdata[2]&BIT(1)) // DPF������ֹ״̬2
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS7] |= BIT(1); // ��1
    else
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS7] &= ~BIT(1); // ��0
    
    if (pdata[5]&BIT(0)) // DPF������ֹ����״̬1
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS8] |= BIT(0); // ��1
    else
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS8] &= ~BIT(0); // ��0

    if (pdata[5]&BIT(1)) // DPF������ֹ����״̬2
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS8] |= BIT(1); // ��1
    else
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS8] &= ~BIT(1); // ��0
    
    if (pdata[5]&BIT(2)) // DPF��������״̬1
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS8] |= BIT(2); // ��1
    else
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS8] &= ~BIT(2); // ��0

    if (pdata[5]&BIT(3)) // DPF��������״̬2
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS8] |= BIT(3); // ��1
    else
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS8] &= ~BIT(3); // ��0

    tlv_a5f2_valid_flag = 1;
    retval = 0x01;
    break;
  
  default:
    retval = 0x00;
    break;
  }
  
  return retval;
}

#if (PART("��ҵͳ��"))
/*************************************************************************
 * ��ҵͳ��ʵ�ֻ���
*************************************************************************/
//==���ü�����־==============================================================
void ZxSts_SetNumberFlag(zxsts_context_t* pThis)
{
  pThis->number_flag = 0x01; // ��Ҫ����
}

//==���ÿ�ʼ��ʱ��־==========================================================
void ZxSts_SetStartFlag(zxsts_context_t* pThis)
{
  pThis->work_flag = 0x01; // ��ʼ
  pThis->debounce_timer = ZXSTS_DEBOUNCE_TIME_SP;
}

//==����ֹͣ��ʱ��־==========================================================
void ZxSts_SetStopFlag(zxsts_context_t* pThis)
{
  pThis->work_flag = 0x00; // ֹͣ
  pThis->debounce_timer = ZXSTS_DEBOUNCE_TIME_SP;
}

//==����CAN֡��ʧȥ��ʱ��=====================================================
void ZxSts_ResetCanMsgTimer(zxsts_context_t* pThis)
{
  pThis->debounce_timer = ZXSTS_DEBOUNCE_TIME_SP;
}

//==�ۼƹ���ʱ��=============================================================
void ZxSts_Accumulate(zxsts_context_t* pThis)
{
  //==ʱ���ۼ�===================================
  if(pThis->work_flag)  // �豸������
  {
    pThis->timer_100ms++;  // ʱ���ۼ�
    if(pThis->timer_100ms >= ZXSTS_STEP_SP)
    {
      pThis->timer_100ms -= ZXSTS_STEP_SP;
      pThis->total_work_time++; // ����ʱ���ۼ�0.05h(3����)
    }
  }
  //==Ƶ���ۼ�===================================
  if(pThis->number_flag)  // �����ۼ�
  {
    pThis->number_flag = ZXSTS_FALSE;
    pThis->total_work_number++;  // Ƶ�μ�1
  }
}

//==��ҵͳ��״̬��============================================================
void ZxSts_AccumulateAll(void)
{
  uint8_t it;

  for(it=0x00; it<NUMBER_OF_ZXSTS_TYPES; it++)
  {
    ZxSts_Accumulate(&zxsts_context[it]);  // ��ҵͳ��
  }
}

//==״̬�ж�==================================================================
void ZxSts_ServiceInput(zxsts_context_t* pThis, uint8_t current_state)
{
  pThis->current_state = current_state;  // ��ȡ��ǰ״̬

  if(pThis->debounce_timer)  // CAN֡��ʧȥ��ʱ��
    pThis->debounce_timer--;
  else
  {
    pThis->current_state = ZXSTS_FALSE;
  }

  if(pThis->current_state) // ��ʼ��ʱ  
  {  pThis->work_flag = ZXSTS_TRUE;}
  else // ֹͣ��ʱ
  {  pThis->work_flag = ZXSTS_FALSE;}
  
  if((pThis->previous_state == ZXSTS_FALSE)&&(pThis->current_state == ZXSTS_TRUE)) // ��һ����
  {
    pThis->number_flag = ZXSTS_TRUE;
  }
  
  pThis->previous_state = pThis->current_state;
}

//==��ҵͳ��״̬��(100ms����)=======================================================
void ZxSts_StateMachine(void)
{
  uint8_t current_state;

  //==�ո���===========================================
  if((zxsts_empty_cylinder_flag==ZXSTS_TRUE) && (zxsts_cylinder_extend_flag==ZXSTS_TRUE)) // ��ȡ��ǰ״̬
  {  current_state = ZXSTS_TRUE;}
  else
  {  current_state = ZXSTS_FALSE;}
  ZxSts_ServiceInput(&zxsts_context[ZXSTS_TYPE_KGEX], current_state);
  ZxSts_Accumulate(&zxsts_context[ZXSTS_TYPE_KGEX]);
    
  //==�ո���===========================================
  if((zxsts_empty_cylinder_flag==ZXSTS_TRUE) && (zxsts_cylinder_shrink_flag==ZXSTS_TRUE)) // ��ȡ��ǰ״̬
  {  current_state = ZXSTS_TRUE;}
  else
  {  current_state = ZXSTS_FALSE;}
  ZxSts_ServiceInput(&zxsts_context[ZXSTS_TYPE_KGS], current_state);
  ZxSts_Accumulate(&zxsts_context[ZXSTS_TYPE_KGS]);

  //==������===========================================
  if((zxsts_arm_work_flag==ZXSTS_TRUE) && (zxsts_cylinder_extend_flag==ZXSTS_TRUE)) // ��ȡ��ǰ״̬
  {  current_state = ZXSTS_TRUE;}
  else
  {  current_state = ZXSTS_FALSE;}
  ZxSts_ServiceInput(&zxsts_context[ZXSTS_TYPE_DBEX], current_state);
  ZxSts_Accumulate(&zxsts_context[ZXSTS_TYPE_DBEX]);

  //==������===========================================
  if((zxsts_arm_work_flag==ZXSTS_TRUE) && (zxsts_cylinder_shrink_flag==ZXSTS_TRUE)) // ��ȡ��ǰ״̬
  {  current_state = ZXSTS_TRUE;}
  else
  {  current_state = ZXSTS_FALSE;}
  ZxSts_ServiceInput(&zxsts_context[ZXSTS_TYPE_DBS], current_state);
  ZxSts_Accumulate(&zxsts_context[ZXSTS_TYPE_DBS]);

  //==�����===========================================
  if(zxsts_luff_up_flag==ZXSTS_TRUE) // ��ȡ��ǰ״̬
  {  current_state = ZXSTS_TRUE;}
  else
  {  current_state = ZXSTS_FALSE;}
  ZxSts_ServiceInput(&zxsts_context[ZXSTS_TYPE_BFUP], current_state);
  ZxSts_Accumulate(&zxsts_context[ZXSTS_TYPE_BFUP]);

  //==�����===========================================
  if(zxsts_luff_down_flag==ZXSTS_TRUE) // ��ȡ��ǰ״̬
  {  current_state = ZXSTS_TRUE;}
  else
  {  current_state = ZXSTS_FALSE;}
  ZxSts_ServiceInput(&zxsts_context[ZXSTS_TYPE_BFDW], current_state);
  ZxSts_Accumulate(&zxsts_context[ZXSTS_TYPE_BFDW]);

  //==��������=========================================
  if(zxsts_main_hoist_up_flag==ZXSTS_TRUE) // ��ȡ��ǰ״̬
  {  current_state = ZXSTS_TRUE;}
  else
  {  current_state = ZXSTS_FALSE;}
  ZxSts_ServiceInput(&zxsts_context[ZXSTS_TYPE_ZJUP], current_state);
  ZxSts_Accumulate(&zxsts_context[ZXSTS_TYPE_ZJUP]);

  //==��������=========================================
  if(zxsts_main_hoist_down_flag==ZXSTS_TRUE) // ��ȡ��ǰ״̬
  {  current_state = ZXSTS_TRUE;}
  else
  {  current_state = ZXSTS_FALSE;}
  ZxSts_ServiceInput(&zxsts_context[ZXSTS_TYPE_ZJDW], current_state);
  ZxSts_Accumulate(&zxsts_context[ZXSTS_TYPE_ZJDW]);

  //==��������=========================================
  if(zxsts_deputy_hoist_up_flag==ZXSTS_TRUE) // ��ȡ��ǰ״̬
  {  current_state = ZXSTS_TRUE;}
  else
  {  current_state = ZXSTS_FALSE;}
  ZxSts_ServiceInput(&zxsts_context[ZXSTS_TYPE_FJUP], current_state);
  ZxSts_Accumulate(&zxsts_context[ZXSTS_TYPE_FJUP]);

  //==��������=========================================
  if(zxsts_deputy_hoist_down_flag==ZXSTS_TRUE) // ��ȡ��ǰ״̬
  {  current_state = ZXSTS_TRUE;}
  else
  {  current_state = ZXSTS_FALSE;}
  ZxSts_ServiceInput(&zxsts_context[ZXSTS_TYPE_FJDW], current_state);
  ZxSts_Accumulate(&zxsts_context[ZXSTS_TYPE_FJDW]);
  
  //==���ת===========================================
  if(zxsts_slew_left_flag==ZXSTS_TRUE) // ��ȡ��ǰ״̬
  {  current_state = ZXSTS_TRUE;}
  else
  {  current_state = ZXSTS_FALSE;}
  ZxSts_ServiceInput(&zxsts_context[ZXSTS_TYPE_ZHR], current_state);
  ZxSts_Accumulate(&zxsts_context[ZXSTS_TYPE_ZHR]);
  
  //==�һ�ת===========================================
  if(zxsts_slew_right_flag==ZXSTS_TRUE) // ��ȡ��ǰ״̬
  {  current_state = ZXSTS_TRUE;}
  else
  {  current_state = ZXSTS_FALSE;}
  ZxSts_ServiceInput(&zxsts_context[ZXSTS_TYPE_YHR], current_state);
  ZxSts_Accumulate(&zxsts_context[ZXSTS_TYPE_YHR]);

  ///////////////////////////////////////////////////////////////////////////
  // ��ȫͳ��
  ///////////////////////////////////////////////////////////////////////////
  //==����==============================================
  if(zxsts_overload_flag==ZXSTS_TRUE) // ��ȡ��ǰ״̬
  {  current_state = ZXSTS_TRUE;}
  else
  {  current_state = ZXSTS_FALSE;}
  ZxSts_ServiceInput(&zxsts_context[ZXSTS_TYPE_LMI], current_state);
  ZxSts_Accumulate(&zxsts_context[ZXSTS_TYPE_LMI]);

  //==������Ȧ==========================================
  if(zxsts_od_main_hoist_flag==ZXSTS_TRUE) // ��ȡ��ǰ״̬
  {  current_state = ZXSTS_TRUE;}
  else
  {  current_state = ZXSTS_FALSE;}
  ZxSts_ServiceInput(&zxsts_context[ZXSTS_TYPE_ZJSQ], current_state);
  ZxSts_Accumulate(&zxsts_context[ZXSTS_TYPE_ZJSQ]);

  //==������Ȧ(����)====================================
  if(zxsts_od_deputy_hoist_flag==ZXSTS_TRUE) // ��ȡ��ǰ״̬
  {  current_state = ZXSTS_TRUE;}
  else
  {  current_state = ZXSTS_FALSE;}
  ZxSts_ServiceInput(&zxsts_context[ZXSTS_TYPE_FJSQ], current_state);
  ZxSts_Accumulate(&zxsts_context[ZXSTS_TYPE_FJSQ]);

  //==���۸���==========================================
  if(zxsts_a2b_main_arm_flag==ZXSTS_TRUE) // ��ȡ��ǰ״̬
  {  current_state = ZXSTS_TRUE;}
  else
  {  current_state = ZXSTS_FALSE;}
  ZxSts_ServiceInput(&zxsts_context[ZXSTS_TYPE_ZBGX], current_state);
  ZxSts_Accumulate(&zxsts_context[ZXSTS_TYPE_ZBGX]);

  //==���۸���(����)====================================
  if(zxsts_a2b_deputy_arm_flag==ZXSTS_TRUE) // ��ȡ��ǰ״̬
  {  current_state = ZXSTS_TRUE;}
  else
  {  current_state = ZXSTS_FALSE;}
  ZxSts_ServiceInput(&zxsts_context[ZXSTS_TYPE_FBGX], current_state);
  ZxSts_Accumulate(&zxsts_context[ZXSTS_TYPE_FBGX]);

  //==��ǿ��============================================
  if(zxsts_lmi_force_flag==ZXSTS_TRUE) // ��ȡ��ǰ״̬
  {  current_state = ZXSTS_TRUE;}
  else
  {  current_state = ZXSTS_FALSE;}
  ZxSts_ServiceInput(&zxsts_context[ZXSTS_TYPE_ZQZ], current_state);
  ZxSts_Accumulate(&zxsts_context[ZXSTS_TYPE_ZQZ]);

  //==��װ����==========================================
  if(zxsts_setup_flag==ZXSTS_TRUE) // ��ȡ��ǰ״̬
  {  current_state = ZXSTS_TRUE;}
  else
  {  current_state = ZXSTS_FALSE;}
  ZxSts_ServiceInput(&zxsts_context[ZXSTS_TYPE_CZKG], current_state);
  ZxSts_Accumulate(&zxsts_context[ZXSTS_TYPE_CZKG]);
  
  //==�����ǿ��========================================
  if(zxsts_luff_up_force_flag==ZXSTS_TRUE) // ��ȡ��ǰ״̬
  {  current_state = ZXSTS_TRUE;}
  else
  {  current_state = ZXSTS_FALSE;}
  ZxSts_ServiceInput(&zxsts_context[ZXSTS_TYPE_BFQQZ], current_state);
  ZxSts_Accumulate(&zxsts_context[ZXSTS_TYPE_BFQQZ]);

  //==����ǿ��==========================================
  if(zxsts_a2b_force_flag==ZXSTS_TRUE) // ��ȡ��ǰ״̬
  {  current_state = ZXSTS_TRUE;}
  else
  {  current_state = ZXSTS_FALSE;}
  ZxSts_ServiceInput(&zxsts_context[ZXSTS_TYPE_GXQZ], current_state);
  ZxSts_Accumulate(&zxsts_context[ZXSTS_TYPE_GXQZ]);

  //==��Ȧǿ��==========================================
  if(zxsts_od_force_flag==ZXSTS_TRUE) // ��ȡ��ǰ״̬
  {  current_state = ZXSTS_TRUE;}
  else
  {  current_state = ZXSTS_FALSE;}
  ZxSts_ServiceInput(&zxsts_context[ZXSTS_TYPE_SQQZ], current_state);
  ZxSts_Accumulate(&zxsts_context[ZXSTS_TYPE_SQQZ]);

  //==���ٳ���==========================================
  if(zxsts_lmi_wind_speed_flag==ZXSTS_TRUE) // ��ȡ��ǰ״̬
  {  current_state = ZXSTS_TRUE;}
  else
  {  current_state = ZXSTS_FALSE;}
  ZxSts_ServiceInput(&zxsts_context[ZXSTS_TYPE_FSCX], current_state);
  ZxSts_Accumulate(&zxsts_context[ZXSTS_TYPE_FSCX]);
}

//==��ʼ����ҵͳ�Ʋ���=======================================================
void ZxSts_Initialize(void)
{
  uint8_t it;

  for(it=0x00; it<NUMBER_OF_ZXSTS_TYPES; it++)
  {
    zxsts_context[it].previous_state = 0x00;
    zxsts_context[it].current_state = 0x00;
    zxsts_context[it].number_flag = 0x00;
    zxsts_context[it].work_flag = 0x00;
    zxsts_context[it].debounce_timer = ZXSTS_DEBOUNCE_TIME_SP;
  }
  zxsts_flag1.word = 0x00;
  zxsts_flag2.word = 0x00;
}

#endif

#if (PART("����������ͳ��"))
/*************************************************************************
 * ����������ͳ��ʵ�ֻ���(ACC���ҷ�����ת�ٴ���300)
*************************************************************************/
//==����������ͳ��״̬��(1s����)==========================================
void ZxStsEngine_StateMachine(zxsts_engine_context_t* pThis)
{
  uint8_t acc_state;
  bittype temp_flag;
  static uint8_t divide_for_1min = 59;
  static uint8_t divide_for_5min = 4;
  static uint8_t DO_1MIN_FLAG = ZXSTS_FALSE;
  static uint8_t DO_5MIN_FLAG = ZXSTS_FALSE;

  //==״̬��ȡ==========
  acc_state = COLT_GetAccStatus();
  temp_flag.byte = zxdown_buffer_a5e7[ZXDOWN_A5E7_POS1]; // ����ϵͳ: bit0-PTO���
  if(temp_flag.b.bit0)
  {  pThis->pto_status = 0x01;}  // �ϳ�����
  else
  {  pThis->pto_status = 0x02;}  // �³�����

  temp_flag.byte = zxinfo_buffer_a504[ZXINFO_A504_POS2_ADDR]; // 0x574.byte1.bit3
  if(temp_flag.b.bit3)
  {  pThis->engine_type = 0x02;}  // ˫������
  else
  {  pThis->engine_type = 0x01;}  // ��������

  //==ȷ�Ϸ���������״̬=======
  if (acc_state==1) // ACC���ͷ���������
  {
    if (pThis->engine_type==0x01) // ����=0x01
    {
      if (pThis->pto_status==0x01) // 0x01=��ҵ
      {
        pThis->up_work_flag = 0x01;
        pThis->down_work_flag = 0x00;
      }
      else if (pThis->pto_status==0x02) // 0x02=��ʻ
      {
        pThis->up_work_flag = 0x00;
        pThis->down_work_flag = 0x01;
      }
      else
      {
        pThis->up_work_flag = 0x00;
        pThis->down_work_flag = 0x00;
      }
    }
    else if (pThis->engine_type==0x02)// ˫��=0x02
    {
      pThis->up_work_flag = 0x01;
      pThis->down_work_flag = 0x01;
    }
    else
    {
      return;
    }
    
    //==ʱ���ʱ��==========
    if (!divide_for_1min)
    {
      divide_for_1min = 59;  // ����1s
      DO_1MIN_FLAG = ZXSTS_TRUE;

      if (!divide_for_5min)
      {
        divide_for_5min = 4;
        DO_5MIN_FLAG = ZXSTS_TRUE;
      }
      else divide_for_5min--;
    }
    else divide_for_1min--;
  }
  else
  {
    divide_for_1min = 59;
    divide_for_5min = 4;
    DO_1MIN_FLAG = ZXSTS_FALSE;
    DO_5MIN_FLAG = ZXSTS_FALSE;
  }

  //==һ��������================================
  if (DO_1MIN_FLAG==ZXSTS_TRUE)
  {
    DO_1MIN_FLAG = ZXSTS_FALSE;

    //==�ϳ�����==============================
    if (pThis->up_work_flag)
    {
      //==��ʱͳ��=====
      pThis->current_twt_up = CAN_GetUpEngineTwt(); // �ϳ��ܹ���ʱ��
      if (pThis->current_twt_up > pThis->previous_twt_up)
      {
        pThis->tdv_up = pThis->current_twt_up - pThis->previous_twt_up;
      }
      else
      {
        pThis->tdv_up = 0;
      }
      pThis->twt_up += pThis->tdv_up;  // �ۼƹ�ʱ
      pThis->previous_twt_up = pThis->current_twt_up;

      //==�ͺ�ͳ��=====
      pThis->current_tfc_up = CAN_GetUpEngineTfc(); // �ϳ����ͺ�
      if (pThis->current_tfc_up > pThis->previous_tfc_up)
      {
        pThis->fdv_up = pThis->current_tfc_up - pThis->previous_tfc_up;
      }
      else
      {
        pThis->fdv_up = 0;
      }
      pThis->tfc_up += pThis->fdv_up;  // �ۼ��ͺ�
      pThis->previous_tfc_up = pThis->current_tfc_up;
    } // end if (pThis->up_work_flag)

    //==�³�����==============================
    if (pThis->down_work_flag)
    {
      //==��ʱͳ��=====
      pThis->current_twt_down = CAN_GetDownEngineTwt(); // �³��ܹ���ʱ��
      if (pThis->current_twt_down > pThis->previous_twt_down)
      {
        pThis->tdv_down = pThis->current_twt_down - pThis->previous_twt_down;
      }
      else
      {
        pThis->tdv_down = 0;
      }
      pThis->twt_down += pThis->tdv_down;  // �ۼƹ�ʱ
      pThis->previous_twt_down = pThis->current_twt_down;

      //==�ͺ�ͳ��=====
      pThis->current_tfc_down = CAN_GetDownEngineTfc(); // �³����ͺ�
      if (pThis->current_tfc_down > pThis->previous_tfc_down)
      {
        pThis->fdv_down = pThis->current_tfc_down - pThis->previous_tfc_down;
      }
      else
      {
        pThis->fdv_down = 0;
      }
      pThis->tfc_down += pThis->fdv_down;  // �ۼ��ͺ�
      pThis->previous_tfc_down = pThis->current_tfc_down;
    } // end if (pThis->down_work_flag)
  } // end if (DO_1MIN_FLAG==ZXSTS_TRUE)

  //==5��������===========================
  if (DO_5MIN_FLAG==ZXSTS_TRUE)
  {
    DO_5MIN_FLAG = ZXSTS_FALSE;

    //==�ϳ�ƽ���ͺ�=====
    if (pThis->twt_up > 0)
    {
      pThis->afc_up = (uint16_t)(pThis->tfc_up / pThis->twt_up);
    }
    //==�³�ƽ���ͺ�=====
    if (pThis->twt_down > 0)
    {
      pThis->afc_down = (uint16_t)(pThis->tfc_down / pThis->twt_down);
    }
  }
}

//==��ʼ������������ͳ�Ʋ���=================================================
void ZxStsEngine_Initialize(zxsts_engine_context_t* pThis)
{
  pThis->pto_status = 0x00;
  pThis->up_work_flag = 0x00;
  pThis->down_work_flag = 0x00;
  pThis->tdv_down = 0x00;
  pThis->tdv_up = 0x00;
  pThis->fdv_up = 0x00;
  pThis->fdv_down = 0x00;

  // ��EEPROM�ڻ�ȡ
  //pThis->engine_type = ;
  //pThis->twt_up = ;
  //pThis->twt_down = ;
  //pThis->tfc_up = ;
  //pThis->tfc_down = ;
  //pThis->afc_up = ;
  //pThis->afc_down = ;
  
  pThis->current_twt_up = pThis->twt_up;
  pThis->previous_twt_up = pThis->twt_up;
  pThis->current_twt_down = pThis->twt_down;
  pThis->previous_twt_down = pThis->twt_down;
  pThis->current_tfc_up = pThis->tfc_up;
  pThis->previous_tfc_up = pThis->tfc_up;
  pThis->current_tfc_down = pThis->tfc_down;
  pThis->previous_tfc_down = pThis->tfc_down;
}

#endif


