/*****************************************************************************
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



/******************************************************************************
* Data Types and Globals
******************************************************************************/
// TLV״̬��Ч��־
bittype2 zxup_tlv_flag1;
bittype2 zxup_tlv_flag2;
bittype2 zxup_tlv_flag3;
bittype2 zxdown_tlv_flag1;
bittype2 zxdown_tlv_flag2;
bittype2 zxengine_tlv_flag;
bittype2 zxversion_tlv_flag;
bittype2 zxstatistics_tlv_flag;

// ���ݻ���
uint8_t zxup_buffer[SIZE_OF_ZXUP_BUFFER]; /// �ϳ����ݻ���
uint8_t zxdown_buffer[SIZE_OF_ZXDOWN_BUFFER]; /// �³��������ݻ���
uint8_t zxengine_buffer[SIZE_OF_ZXENGINE_BUFFER]; /// �³����������ݻ���
uint8_t zxstatistics_buffer[SIZE_OF_ZXSTATISTICS_BUFFER]; ///ͳ�����ݻ���
uint8_t zxversion_buffer[SIZE_OF_ZXVERSION_BUFFER]; /// �汾��Ϣ����

/*************************************************************************
 * ������յ���uCAN����(�ϳ�)
*************************************************************************/
uint8_t CAN_ProcessRecvUpMsg(uint32_t canId, uint8_t *pdata, uint8_t size)
{
  uint8_t retval = 0x00;
  //uint32_t tempVal;

  switch (canId) // �ϳ�ͨ��
  {
  //==A5A0====================================================================================
  case 0x565:
    zxup_buffer[ZXUP_A5A0_POS1_ADDR] = pdata[0]; // ����������
    zxup_buffer[ZXUP_A5A0_POS1_ADDR+1] = pdata[1];
    break;

  case 0x285:
    zxup_buffer[ZXUP_A5A0_POS2_ADDR] = pdata[1]; // ��������(ABCDEFGH)
    zxup_buffer[ZXUP_A5A0_POS2_ADDR+1] = pdata[3];
    zxup_buffer[ZXUP_A5A0_POS2_ADDR+2] = pdata[2];
    zxup_buffer[ZXUP_A5A0_POS3_ADDR] = pdata[0]; // ����
    zxup_buffer[ZXUP_A5A0_POS6_ADDR] = pdata[4]; // ���۳���
    zxup_buffer[ZXUP_A5A0_POS6_ADDR+1] = pdata[5];
    zxup_buffer[ZXUP_A5A0_POS10_ADDR] = pdata[6]; // ���۳���
    zxup_buffer[ZXUP_A5A0_POS10_ADDR+1] = pdata[7];
    tlv_a5a0_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x195:
    zxup_buffer[ZXUP_A5A0_POS4_ADDR] = pdata[4]; // ����ͷ���Ƕ�
    zxup_buffer[ZXUP_A5A0_POS4_ADDR+1] = pdata[5];
    zxup_buffer[ZXUP_A5A0_POS5_ADDR] = pdata[2]; // ���۸����Ƕ�
    zxup_buffer[ZXUP_A5A0_POS5_ADDR+1] = pdata[3];
    zxup_buffer[ZXUP_A5A0_POS15_ADDR] = pdata[6]; // ��ͷ�߶�
    zxup_buffer[ZXUP_A5A0_POS15_ADDR+1] = pdata[7];
    tlv_a5a0_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x183:
    memcpy(&zxup_buffer[ZXUP_A5A0_POS8_ADDR], pdata, 8); // �ڱ۰ٷֱ�
    tlv_a5a0_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x283:
    zxup_buffer[ZXUP_A5A0_POS9_ADDR] = pdata[0]; // �ױ�������
    tlv_a5a0_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x295:
    zxup_buffer[ZXUP_A5A0_POS11_ADDR] = pdata[0]; // ���۽Ƕ�
    zxup_buffer[ZXUP_A5A0_POS11_ADDR+1] = pdata[1];
    zxup_buffer[ZXUP_A5A0_POS12_ADDR] = pdata[2]; // ����/���۸����Ƕ�
    zxup_buffer[ZXUP_A5A0_POS12_ADDR+1] = pdata[3];
    zxup_buffer[ZXUP_A5A0_POS13_ADDR] = pdata[4]; // ����/����ͷ���Ƕ�
    zxup_buffer[ZXUP_A5A0_POS13_ADDR+1] = pdata[5];
    zxup_buffer[ZXUP_A5A2_POS1_ADDR] = pdata[6]; // ����������
    zxup_buffer[ZXUP_A5A2_POS1_ADDR+1] = pdata[7];
    tlv_a5a0_valid_flag = 1;
    tlv_a5a2_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x1D5:
    memcpy(&zxup_buffer[ZXUP_A5A0_POS14_ADDR], pdata, 8); // ǻѹ��
    tlv_a5a0_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x185:
    zxup_buffer[ZXUP_A5A0_POS16_ADDR] = pdata[0]; // �����
    zxup_buffer[ZXUP_A5A0_POS16_ADDR+1] = pdata[1];
    zxup_buffer[ZXUP_A5A0_POS17_ADDR] = pdata[2]; // ʵ������
    zxup_buffer[ZXUP_A5A0_POS17_ADDR+1] = pdata[3];
    zxup_buffer[ZXUP_A5A0_POS18_ADDR] = pdata[4]; // ���ذٷֱ�
    zxup_buffer[ZXUP_A5A0_POS18_ADDR+1] = pdata[5];
    zxup_buffer[ZXUP_A5A0_POS19_ADDR] = pdata[6]; // ��������
    zxup_buffer[ZXUP_A5A0_POS19_ADDR+1] = pdata[7];
    tlv_a5a0_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x385:
    memcpy(&zxup_buffer[ZXUP_A5A0_POS20_ADDR], pdata, 8); // LMI���ϴ���1-4
    tlv_a5a0_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x395:
    memcpy(&zxup_buffer[ZXUP_A5A0_POS21_ADDR], pdata, 8); // LMI���ϴ���5-8
    tlv_a5a0_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x3F5:
    memcpy(&zxup_buffer[ZXUP_A5A0_POS22_ADDR], pdata, 4); // �ǶԳƹ��ϴ���
    tlv_a5a0_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x3E5:
    memcpy(&zxup_buffer[ZXUP_A5A0_POS23_ADDR], pdata, 4); // LMI����ʱ��
    tlv_a5a0_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x1C5:
    if (pdata[4]&BIT(0)) // �߶���λ
      zxup_buffer[ZXUP_A5A0_POS24_ADDR] |= BIT(0); // ��1
    else
      zxup_buffer[ZXUP_A5A0_POS24_ADDR] &= ~BIT(0); // ��0

    zxup_buffer[ZXUP_A5A2_POS2_ADDR] = pdata[0]; // ����������
    zxup_buffer[ZXUP_A5A2_POS2_ADDR+1] = pdata[1];
    tlv_a5a0_valid_flag = 1;
    tlv_a5a2_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x463:
    if (pdata[2]&BIT(4)) // LMIǿ��
      zxup_buffer[ZXUP_A5A0_POS24_ADDR] |= BIT(1); // ��1
    else
      zxup_buffer[ZXUP_A5A0_POS24_ADDR] &= ~BIT(1); // ��0

    if (pdata[2]&BIT(0)) // ��װ����
      zxup_buffer[ZXUP_A5A0_POS24_ADDR] |= BIT(2); // ��1
    else
      zxup_buffer[ZXUP_A5A0_POS24_ADDR] &= ~BIT(2); // ��0

    if (pdata[2]&BIT(2)) // �����ǿ��
      zxup_buffer[ZXUP_A5A0_POS24_ADDR] |= BIT(3); // ��1
    else
      zxup_buffer[ZXUP_A5A0_POS24_ADDR] &= ~BIT(3); // ��0

    if (pdata[2]&BIT(3)) // ����ǿ��
      zxup_buffer[ZXUP_A5A0_POS24_ADDR] |= BIT(4); // ��1
    else
      zxup_buffer[ZXUP_A5A0_POS24_ADDR] &= ~BIT(4); // ��0

    if (pdata[2]&BIT(1)) // �߶���λǿ��
      zxup_buffer[ZXUP_A5A0_POS24_ADDR] |= BIT(5); // ��1
    else
      zxup_buffer[ZXUP_A5A0_POS24_ADDR] &= ~BIT(5); // ��0

    if (pdata[7]&BIT(7)) // ������������
      zxup_buffer[ZXUP_A5B3_POS3_ADDR] |= BIT(0); // ��1
    else
      zxup_buffer[ZXUP_A5B3_POS3_ADDR] &= ~BIT(0); // ��0

    if (pdata[7]&BIT(6)) // ���Ͷ�������
      zxup_buffer[ZXUP_A5B3_POS3_ADDR] |= BIT(1); // ��1
    else
      zxup_buffer[ZXUP_A5B3_POS3_ADDR] &= ~BIT(1); // ��0

    tlv_a5b3_valid_flag = 1;
    tlv_a5a0_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x263:
    zxup_buffer[ZXUP_A5A0_POS25_ADDR] = pdata[0]; // ˮƽ��X
    zxup_buffer[ZXUP_A5A0_POS25_ADDR+1] = pdata[1];
    zxup_buffer[ZXUP_A5A0_POS26_ADDR] = pdata[2]; // ˮƽ��Y
    zxup_buffer[ZXUP_A5A0_POS26_ADDR+1] = pdata[3];
    zxup_buffer[ZXUP_A5A0_POS27_ADDR] = pdata[4]; // ����
    zxup_buffer[ZXUP_A5A0_POS27_ADDR+1] = pdata[5];
    zxup_buffer[ZXUP_A5A0_POS28_ADDR] = pdata[6]; // ��ת�Ƕ�
    zxup_buffer[ZXUP_A5A0_POS28_ADDR+1] = pdata[7];
    tlv_a5a0_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5A1====================================================================================
  case 0x1E3:
    zxup_buffer[ZXUP_A5A1_POS1_ADDR] = pdata[0]; // �������Ƕ�
    zxup_buffer[ZXUP_A5A1_POS1_ADDR+1] = pdata[1];
    zxup_buffer[ZXUP_A5A1_POS5_ADDR] = pdata[2]; // �ҳ������Ƕ�
    zxup_buffer[ZXUP_A5A1_POS5_ADDR+1] = pdata[3];
    zxup_buffer[ZXUP_A5A1_POS3_ADDR] = pdata[4]; // ����չ���Ƕ�
    zxup_buffer[ZXUP_A5A1_POS3_ADDR+1] = pdata[5];
    zxup_buffer[ZXUP_A5A1_POS7_ADDR] = pdata[6]; // �ҳ���չ���Ƕ�
    zxup_buffer[ZXUP_A5A1_POS7_ADDR+1] = pdata[7];
    tlv_a5a1_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x2E3:
    zxup_buffer[ZXUP_A5A1_POS2_ADDR] = pdata[0]; // ��������
    zxup_buffer[ZXUP_A5A1_POS2_ADDR+1] = pdata[1];
    zxup_buffer[ZXUP_A5A1_POS6_ADDR] = pdata[2]; // �ҳ�������
    zxup_buffer[ZXUP_A5A1_POS6_ADDR+1] = pdata[3];
    zxup_buffer[ZXUP_A5A1_POS13_ADDR] = pdata[4]; // �����Ž��׳���
    zxup_buffer[ZXUP_A5A1_POS13_ADDR+1] = pdata[5];
    zxup_buffer[ZXUP_A5A1_POS14_ADDR] = pdata[6]; // �ҳ����Ž��׳���
    zxup_buffer[ZXUP_A5A1_POS14_ADDR+1] = pdata[7];
    tlv_a5a1_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x4E3:
    zxup_buffer[ZXUP_A5A1_POS4_ADDR] = pdata[2]; // �������Ƕ�
    zxup_buffer[ZXUP_A5A1_POS4_ADDR+1] = pdata[3];
    zxup_buffer[ZXUP_A5A1_POS8_ADDR] = pdata[4]; // �ҳ������Ƕ�
    zxup_buffer[ZXUP_A5A1_POS8_ADDR+1] = pdata[5];
    zxup_buffer[ZXUP_A5B1_POS3_ADDR] = pdata[6]; // �����Ž��׳��� *********
    zxup_buffer[ZXUP_A5B1_POS3_ADDR+1] = pdata[7];
    tlv_a5a1_valid_flag = 1;
    tlv_a5b1_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x3E3:
    if (pdata[5]&BIT(0)) // ���������ֹ
      zxup_buffer[ZXUP_A5A1_POS9_ADDR] |= BIT(0); // ��1
    else
      zxup_buffer[ZXUP_A5A1_POS9_ADDR] &= ~BIT(0); // ��0

    if (pdata[5]&BIT(1)) // ����������
      zxup_buffer[ZXUP_A5A1_POS9_ADDR] |= BIT(1); // ��1
    else
      zxup_buffer[ZXUP_A5A1_POS9_ADDR] &= ~BIT(1); // ��0

    if (pdata[5]&BIT(4)) // ����������λ
      zxup_buffer[ZXUP_A5A1_POS9_ADDR] |= BIT(2); // ��1
    else
      zxup_buffer[ZXUP_A5A1_POS9_ADDR] &= ~BIT(2); // ��0

    if (pdata[6]&BIT(1)) // �����Ž��׽���
      zxup_buffer[ZXUP_A5A1_POS9_ADDR] |= BIT(5); // ��1
    else
      zxup_buffer[ZXUP_A5A1_POS9_ADDR] &= ~BIT(5); // ��0

    if (pdata[6]&BIT(0)) // �����Ž�����ֹ
      zxup_buffer[ZXUP_A5A1_POS9_ADDR] |= BIT(6); // ��1
    else
      zxup_buffer[ZXUP_A5A1_POS9_ADDR] &= ~BIT(6); // ��0

    if (pdata[6]&BIT(4)) // �����Ž���ȫ������
      zxup_buffer[ZXUP_A5A1_POS9_ADDR] |= BIT(7); // ��1
    else
      zxup_buffer[ZXUP_A5A1_POS9_ADDR] &= ~BIT(7); // ��0

    if (pdata[5]&BIT(2)) // �ҳ��������ֹ
      zxup_buffer[ZXUP_A5A1_POS10_ADDR] |= BIT(0); // ��1
    else
      zxup_buffer[ZXUP_A5A1_POS10_ADDR] &= ~BIT(0); // ��0

    if (pdata[5]&BIT(3)) // �ҳ���������
      zxup_buffer[ZXUP_A5A1_POS10_ADDR] |= BIT(1); // ��1
    else
      zxup_buffer[ZXUP_A5A1_POS10_ADDR] &= ~BIT(1); // ��0

    if (pdata[5]&BIT(5)) // �ҳ���������λ
      zxup_buffer[ZXUP_A5A1_POS10_ADDR] |= BIT(2); // ��1
    else
      zxup_buffer[ZXUP_A5A1_POS10_ADDR] &= ~BIT(2); // ��0

    if (pdata[6]&BIT(3)) // �ҳ����Ž��׽���
      zxup_buffer[ZXUP_A5A1_POS10_ADDR] |= BIT(5); // ��1
    else
      zxup_buffer[ZXUP_A5A1_POS10_ADDR] &= ~BIT(5); // ��0

    if (pdata[6]&BIT(2)) // �ҳ����Ž�����ֹ
      zxup_buffer[ZXUP_A5A1_POS10_ADDR] |= BIT(6); // ��1
    else
      zxup_buffer[ZXUP_A5A1_POS10_ADDR] &= ~BIT(6); // ��0

    if (pdata[6]&BIT(5)) // �ҳ����Ž���ȫ������
      zxup_buffer[ZXUP_A5A1_POS10_ADDR] |= BIT(7); // ��1
    else
      zxup_buffer[ZXUP_A5A1_POS10_ADDR] &= ~BIT(7); // ��0

    zxup_buffer[ZXUP_A5A1_POS11_ADDR] = pdata[0]; // �������
    zxup_buffer[ZXUP_A5A1_POS12_ADDR] = pdata[1]; // �ҳ������

    if (pdata[7]&BIT(2)) // ���Ž�������С*********
      zxup_buffer[ZXUP_A5B1_POS4_ADDR] |= BIT(0); // ��1
    else
      zxup_buffer[ZXUP_A5B1_POS4_ADDR] &= ~BIT(0); // ��0

    if (pdata[7]&BIT(3)) // ���Ž���������*********
      zxup_buffer[ZXUP_A5B1_POS4_ADDR] |= BIT(1); // ��1
    else
      zxup_buffer[ZXUP_A5B1_POS4_ADDR] &= ~BIT(1); // ��0

    if (pdata[7]&BIT(4)) // ���Ž�������С*********
      zxup_buffer[ZXUP_A5B1_POS4_ADDR] |= BIT(2); // ��1
    else
      zxup_buffer[ZXUP_A5B1_POS4_ADDR] &= ~BIT(2); // ��0

    if (pdata[7]&BIT(5)) // ���Ž���������*********
      zxup_buffer[ZXUP_A5B1_POS4_ADDR] |= BIT(3); // ��1
    else
      zxup_buffer[ZXUP_A5B1_POS4_ADDR] &= ~BIT(3); // ��0

    zxup_buffer[ZXUP_A5B1_POS2_ADDR] = pdata[2]; // ����Ŀ���Ž��Ƕ�*********
    zxup_buffer[ZXUP_A5B1_POS2_ADDR+1] = pdata[3];
    tlv_a5a1_valid_flag = 1;
    tlv_a5b1_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5A2====================================================================================
  case 0x3F3:
    zxup_buffer[ZXUP_A5A2_POS3_ADDR] = pdata[4]; // ������ѹ��
    zxup_buffer[ZXUP_A5A2_POS3_ADDR+1] = pdata[5];
    zxup_buffer[ZXUP_A5A2_POS4_ADDR] = pdata[4]; // ǰ֧�ܽǶ�
    zxup_buffer[ZXUP_A5A2_POS4_ADDR+1] = pdata[5];
    zxup_buffer[ZXUP_A5BE_POS2_ADDR] = pdata[6]; // ��Ͳת��*****
    zxup_buffer[ZXUP_A5BE_POS2_ADDR+1] = pdata[7];
    tlv_a5a2_valid_flag = 1;
    tlv_a5be_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x1F3:
    if (pdata[0]&BIT(3)) // �ҳ��������ֹ
      zxup_buffer[ZXUP_A5A2_POS5_ADDR] |= BIT(0); // ��1
    else
      zxup_buffer[ZXUP_A5A2_POS5_ADDR] &= ~BIT(0); // ��0

    if (pdata[2]&BIT(0)) // �ƶ���ŷ�
      zxup_buffer[ZXUP_A5BE_POS3_ADDR] |= BIT(1); // ��1
    else
      zxup_buffer[ZXUP_A5BE_POS3_ADDR] &= ~BIT(1); // ��0

    if (pdata[2]&BIT(1)) // ѹ���̵���
      zxup_buffer[ZXUP_A5BE_POS3_ADDR] |= BIT(2); // ��1
    else
      zxup_buffer[ZXUP_A5BE_POS3_ADDR] &= ~BIT(2); // ��0

    zxup_buffer[ZXUP_A5BE_POS1_ADDR] = pdata[6]; // ������
    zxup_buffer[ZXUP_A5BE_POS1_ADDR+1] = pdata[7];
    tlv_a5a2_valid_flag = 1;
    tlv_a5be_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5A5====================================================================================
  case 0x35A:
    zxup_buffer[ZXUP_A5A5_POS1_ADDR] = pdata[3]; // ������ת��
    zxup_buffer[ZXUP_A5A5_POS1_ADDR+1] = pdata[4];
    zxup_buffer[ZXUP_A5A5_POS2_ADDR] = pdata[2]; // ʵ��Ť�ذٷֱ�

    if (pdata[0]&BIT(0)) // ������Ť��ģʽ(����������)1
      zxup_buffer[ZXUP_A5A5_POS16_ADDR] |= BIT(0); // ��1
    else
      zxup_buffer[ZXUP_A5A5_POS16_ADDR] &= ~BIT(0); // ��0
      
    if (pdata[0]&BIT(1)) // ������Ť��ģʽ(����������)2
      zxup_buffer[ZXUP_A5A5_POS16_ADDR] |= BIT(1); // ��1
    else
      zxup_buffer[ZXUP_A5A5_POS16_ADDR] &= ~BIT(1); // ��0

    if (pdata[0]&BIT(2)) // ������Ť��ģʽ(����������)3
      zxup_buffer[ZXUP_A5A5_POS16_ADDR] |= BIT(2); // ��1
    else
      zxup_buffer[ZXUP_A5A5_POS16_ADDR] &= ~BIT(2); // ��0

    if (pdata[0]&BIT(3)) // ������Ť��ģʽ(����������)4
      zxup_buffer[ZXUP_A5A5_POS16_ADDR] |= BIT(3); // ��1
    else
      zxup_buffer[ZXUP_A5A5_POS16_ADDR] &= ~BIT(3); // ��0

    tlv_a5a5_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x35B:
    zxup_buffer[ZXUP_A5A5_POS3_ADDR] = pdata[0]; // Ħ��Ť�ذٷֱ�
    tlv_a5a5_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x35C:
    zxup_buffer[ZXUP_A5A5_POS4_ADDR] = pdata[1]; // �������ѹ��
    zxup_buffer[ZXUP_A5A5_POS5_ADDR] = pdata[2]; // ��������¶�
    tlv_a5a5_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x35D:
    zxup_buffer[ZXUP_A5A5_POS6_ADDR] = pdata[0]; // ��ȴҺ�¶�
    zxup_buffer[ZXUP_A5A5_POS7_ADDR] = pdata[2]; // �����¶�
    zxup_buffer[ZXUP_A5A5_POS7_ADDR+1] = pdata[3];
    tlv_a5a5_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x35E:
    if (pdata[0]&BIT(0)) // ������ˮָʾ��1
      zxup_buffer[ZXUP_A5A5_POS11_ADDR] |= BIT(0); // ��1
    else
      zxup_buffer[ZXUP_A5A5_POS11_ADDR] &= ~BIT(0); // ��0

    if (pdata[0]&BIT(1)) // ������ˮָʾ��2
      zxup_buffer[ZXUP_A5A5_POS11_ADDR] |= BIT(1); // ��1
    else
      zxup_buffer[ZXUP_A5A5_POS11_ADDR] &= ~BIT(1); // ��0

    zxup_buffer[ZXUP_A5A5_POS8_ADDR] = pdata[2]; // ����Һλ
    zxup_buffer[ZXUP_A5A5_POS9_ADDR] = pdata[3]; // ����ѹ��
    tlv_a5a5_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x35F:
    zxup_buffer[ZXUP_A5A5_POS10_ADDR] = pdata[0]; // ������������ʱ��
    zxup_buffer[ZXUP_A5A5_POS10_ADDR+1] = pdata[1];
    zxup_buffer[ZXUP_A5A5_POS10_ADDR+2] = pdata[2];
    zxup_buffer[ZXUP_A5A5_POS10_ADDR+3] = pdata[3];
    tlv_a5a5_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x36A:
    zxup_buffer[ZXUP_A5A5_POS12_ADDR] = pdata[1]; // ����̤��ٷֱ�
    tlv_a5a5_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x36B:
    zxup_buffer[ZXUP_A5A5_POS13_ADDR] = pdata[0]; // ������ȼ��������
    zxup_buffer[ZXUP_A5A5_POS13_ADDR+1] = pdata[1];
    zxup_buffer[ZXUP_A5A5_POS14_ADDR] = pdata[4]; // ������ƽ��ȼ��������
    zxup_buffer[ZXUP_A5A5_POS14_ADDR+1] = pdata[5];
    tlv_a5a5_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x36C:
    zxup_buffer[ZXUP_A5A5_POS15_ADDR] = pdata[1]; // ȼ��Һλ
    tlv_a5a5_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5A6====================================================================================
  case 0x464:
    if (pdata[1]&BIT(0)) // ���ֱ���λ
      zxup_buffer[ZXUP_A5A6_POS1_ADDR] |= BIT(0); // ��1
    else
      zxup_buffer[ZXUP_A5A6_POS1_ADDR] &= ~BIT(0); // ��0

    if (pdata[1]&BIT(1)) // ���ֱ��󿪹�
      zxup_buffer[ZXUP_A5A6_POS1_ADDR] |= BIT(5); // ��1
    else
      zxup_buffer[ZXUP_A5A6_POS1_ADDR] &= ~BIT(5); // ��0

    if (pdata[1]&BIT(2)) // ���ֱ��ҿ���
      zxup_buffer[ZXUP_A5A6_POS1_ADDR] |= BIT(6); // ��1
    else
      zxup_buffer[ZXUP_A5A6_POS1_ADDR] &= ~BIT(6); // ��0

    if (pdata[1]&BIT(3)) // ���ֱ��ȵ�����
      zxup_buffer[ZXUP_A5A6_POS1_ADDR] |= BIT(7); // ��1
    else
      zxup_buffer[ZXUP_A5A6_POS1_ADDR] &= ~BIT(7); // ��0

    if (pdata[1]&BIT(4)) // ���ֱ���λ
      zxup_buffer[ZXUP_A5A6_POS4_ADDR] |= BIT(0); // ��1
    else
      zxup_buffer[ZXUP_A5A6_POS4_ADDR] &= ~BIT(0); // ��0

    if (pdata[1]&BIT(5)) // ���ֱ��󿪹�
      zxup_buffer[ZXUP_A5A6_POS4_ADDR] |= BIT(5); // ��1
    else
      zxup_buffer[ZXUP_A5A6_POS4_ADDR] &= ~BIT(5); // ��0

    if (pdata[1]&BIT(6)) // ���ֱ��ҿ���
      zxup_buffer[ZXUP_A5A6_POS4_ADDR] |= BIT(6); // ��1
    else
      zxup_buffer[ZXUP_A5A6_POS4_ADDR] &= ~BIT(6); // ��0

    if (pdata[1]&BIT(7)) // ���ֱ��ȵ�����
      zxup_buffer[ZXUP_A5A6_POS4_ADDR] |= BIT(7); // ��1
    else
      zxup_buffer[ZXUP_A5A6_POS4_ADDR] &= ~BIT(7); // ��0

    zxup_buffer[ZXUP_A5B3_POS2_ADDR] = pdata[2];
    tlv_a5a6_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x4E4:
    if (pdata[0]&BIT(4)) // ���ֱ�X����
      zxup_buffer[ZXUP_A5A6_POS1_ADDR] |= BIT(1); // ��1
    else
      zxup_buffer[ZXUP_A5A6_POS1_ADDR] &= ~BIT(1); // ��0

    if (pdata[0]&BIT(5)) // ���ֱ�X����
      zxup_buffer[ZXUP_A5A6_POS1_ADDR] |= BIT(2); // ��1
    else
      zxup_buffer[ZXUP_A5A6_POS1_ADDR] &= ~BIT(2); // ��0

    if (pdata[0]&BIT(6)) // ���ֱ�Y����
      zxup_buffer[ZXUP_A5A6_POS1_ADDR] |= BIT(3); // ��1
    else
      zxup_buffer[ZXUP_A5A6_POS1_ADDR] &= ~BIT(3); // ��0

    if (pdata[0]&BIT(7)) // ���ֱ�Y����
      zxup_buffer[ZXUP_A5A6_POS1_ADDR] |= BIT(4); // ��1
    else
      zxup_buffer[ZXUP_A5A6_POS1_ADDR] &= ~BIT(4); // ��0

    if (pdata[0]&BIT(0)) // ���ֱ�X����
      zxup_buffer[ZXUP_A5A6_POS4_ADDR] |= BIT(1); // ��1
    else
      zxup_buffer[ZXUP_A5A6_POS4_ADDR] &= ~BIT(1); // ��0

    if (pdata[0]&BIT(1)) // ���ֱ�X����
      zxup_buffer[ZXUP_A5A6_POS4_ADDR] |= BIT(2); // ��1
    else
      zxup_buffer[ZXUP_A5A6_POS4_ADDR] &= ~BIT(2); // ��0

    if (pdata[0]&BIT(2)) // ���ֱ�Y����
      zxup_buffer[ZXUP_A5A6_POS4_ADDR] |= BIT(3); // ��1
    else
      zxup_buffer[ZXUP_A5A6_POS4_ADDR] &= ~BIT(3); // ��0

    if (pdata[0]&BIT(3)) // ���ֱ�Y����
      zxup_buffer[ZXUP_A5A6_POS4_ADDR] |= BIT(4); // ��1
    else
      zxup_buffer[ZXUP_A5A6_POS4_ADDR] &= ~BIT(4); // ��0

    tlv_a5a6_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x363:
    zxup_buffer[ZXUP_A5A6_POS2_ADDR] = pdata[4]; // ���ֱ�X���
    zxup_buffer[ZXUP_A5A6_POS2_ADDR+1] = pdata[5];
    zxup_buffer[ZXUP_A5A6_POS3_ADDR] = pdata[6]; // ���ֱ�Y���
    zxup_buffer[ZXUP_A5A6_POS3_ADDR+1] = pdata[7];
    zxup_buffer[ZXUP_A5A6_POS5_ADDR] = pdata[0]; // ���ֱ�X���
    zxup_buffer[ZXUP_A5A6_POS5_ADDR+1] = pdata[1];
    zxup_buffer[ZXUP_A5A6_POS6_ADDR] = pdata[2]; // ���ֱ�Y���
    zxup_buffer[ZXUP_A5A6_POS6_ADDR+1] = pdata[3];
    tlv_a5a6_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5A7====================================================================================
  case 0x561:
    zxup_buffer[ZXUP_A5A7_POS1_ADDR] = pdata[0]; // ��Ʒ����(1-4)
    zxup_buffer[ZXUP_A5A7_POS1_ADDR+1] = pdata[1];
    zxup_buffer[ZXUP_A5A7_POS1_ADDR+2] = pdata[2];
    zxup_buffer[ZXUP_A5A7_POS1_ADDR+2] = pdata[3];
    zxversion_buffer[ZXVERSION_A505_POS3_ADDR] = pdata[4];  // ��ʾ���ײ�汾
    zxversion_buffer[ZXVERSION_A505_POS3_ADDR+1] = pdata[5];
    zxversion_buffer[ZXVERSION_A505_POS3_ADDR+2] = pdata[6];
    tlv_a505_valid_flag = 1;
    tlv_a5a7_valid_flag = 1;
    retval = 0x01;
    break;
  
  case 0x181:
    zxup_buffer[ZXUP_A5A7_POS2_ADDR] = pdata[1]; // ��������(ABCDEF)
    zxup_buffer[ZXUP_A5A7_POS2_ADDR+1] = pdata[3];
    zxup_buffer[ZXUP_A5A7_POS2_ADDR+2] = pdata[2];
    zxup_buffer[ZXUP_A5A7_POS11_ADDR] = pdata[4]; // Ŀ�����
    zxup_buffer[ZXUP_A5A7_POS14_ADDR] = pdata[5]; // ����ģʽ
    tlv_a5a7_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x3A1:
    zxup_buffer[ZXUP_A5A7_POS3_ADDR] = pdata[0]; // �������ٶ�
    zxup_buffer[ZXUP_A5A7_POS4_ADDR] = pdata[6]; // �������ٶ�
    zxup_buffer[ZXUP_A5A7_POS5_ADDR] = pdata[4]; // �������ٶ�
    zxup_buffer[ZXUP_A5A7_POS6_ADDR] = pdata[5]; // �������ٶ�
    zxup_buffer[ZXUP_A5A7_POS7_ADDR] = pdata[2]; // ������ٶ�
    zxup_buffer[ZXUP_A5A7_POS8_ADDR] = pdata[3]; // ������ٶ�
    zxup_buffer[ZXUP_A5A7_POS9_ADDR] = pdata[1]; // ���ת�ٶ�
    zxup_buffer[ZXUP_A5A7_POS10_ADDR] = pdata[7]; // �һ�ת�ٶ�
    tlv_a5a7_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x191:
    if (pdata[0]&BIT(0)) // �����涯1
      zxup_buffer[ZXUP_A5A7_POS12_ADDR] |= BIT(1); // ��1
    else
      zxup_buffer[ZXUP_A5A7_POS12_ADDR] &= ~BIT(1); // ��0

    if (pdata[0]&BIT(1)) // �����涯2
      zxup_buffer[ZXUP_A5A7_POS12_ADDR] |= BIT(2); // ��1
    else
      zxup_buffer[ZXUP_A5A7_POS12_ADDR] &= ~BIT(2); // ��0

    if (pdata[0]&BIT(4)) // ��/�����涯
      zxup_buffer[ZXUP_A5A7_POS12_ADDR] |= BIT(3); // ��1
    else
      zxup_buffer[ZXUP_A5A7_POS12_ADDR] &= ~BIT(3); // ��0

    if (pdata[0]&BIT(3)) // �������
      zxup_buffer[ZXUP_A5A7_POS12_ADDR] |= BIT(4); // ��1
    else
      zxup_buffer[ZXUP_A5A7_POS12_ADDR] &= ~BIT(4); // ��0

    tlv_a5a7_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x482:
    if (pdata[2]&BIT(0)) // ȡ��ʹ��
      zxup_buffer[ZXUP_A5A7_POS12_ADDR] |= BIT(0); // ��1
    else
      zxup_buffer[ZXUP_A5A7_POS12_ADDR] &= ~BIT(0); // ��0

    tlv_a5a7_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x291:
    if (pdata[0]&BIT(0)) // ��ת����
      zxup_buffer[ZXUP_A5A7_POS12_ADDR] |= BIT(5); // ��1
    else
      zxup_buffer[ZXUP_A5A7_POS12_ADDR] &= ~BIT(5); // ��0

    tlv_a5a7_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x381:
    if (pdata[3]&BIT(0)) // ���۲�װ����1
      zxup_buffer[ZXUP_A5A7_POS12_ADDR] |= BIT(6); // ��1
    else
      zxup_buffer[ZXUP_A5A7_POS12_ADDR] &= ~BIT(6); // ��0

    if (pdata[3]&BIT(1)) // ���۲�װ����2
      zxup_buffer[ZXUP_A5A7_POS12_ADDR] |= BIT(7); // ��1
    else
      zxup_buffer[ZXUP_A5A7_POS12_ADDR] &= ~BIT(7); // ��0

    if (pdata[3]&BIT(2)) // ���۹����任1
      zxup_buffer[ZXUP_A5A7_POS13_ADDR] |= BIT(0); // ��1
    else
      zxup_buffer[ZXUP_A5A7_POS13_ADDR] &= ~BIT(0); // ��0

    if (pdata[3]&BIT(3)) // ���۹����任2
      zxup_buffer[ZXUP_A5A7_POS13_ADDR] |= BIT(1); // ��1
    else
      zxup_buffer[ZXUP_A5A7_POS13_ADDR] &= ~BIT(1); // ��0

    tlv_a5a7_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x391:
    if (pdata[0]&BIT(0)) // �����Զ�
      zxup_buffer[ZXUP_A5A7_POS13_ADDR] |= BIT(2); // ��1
    else
      zxup_buffer[ZXUP_A5A7_POS13_ADDR] &= ~BIT(2); // ��0

    tlv_a5a7_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5A8====================================================================================
  case 0x382:
    memcpy(&zxup_buffer[ZXUP_A5A8_POS1_ADDR], pdata, 8); // ����ٿ�
    zxup_buffer[ZXUP_A5B1_POS1_ADDR] = pdata[3]; // ����Ŀ��չ���Ƕ�
    tlv_a5a8_valid_flag = 1;
    tlv_a5b1_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x4C1:
    zxup_buffer[ZXUP_A5A8_POS2_ADDR] = pdata[0]; // ����ά��1+2
    zxup_buffer[ZXUP_A5A8_POS2_ADDR+1] = pdata[1];
    tlv_a5a8_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5A9====================================================================================
  case 0x1B1:
    zxup_buffer[ZXUP_A5A9_POS1_ADDR] = pdata[0]; // ���1
    zxup_buffer[ZXUP_A5A9_POS1_ADDR+1] = pdata[1];
    tlv_a5a9_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x1B2:
    zxup_buffer[ZXUP_A5A9_POS2_ADDR] = pdata[0]; // ���2
    zxup_buffer[ZXUP_A5A9_POS2_ADDR+1] = pdata[1];
    tlv_a5a9_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x1B3:
    zxup_buffer[ZXUP_A5A9_POS2_ADDR] = pdata[0]; // ���3
    zxup_buffer[ZXUP_A5A9_POS2_ADDR+1] = pdata[1];
    tlv_a5a9_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5AA====================================================================================
  case 0x188:
    memcpy(&zxup_buffer[ZXUP_A5AA_POS1_ADDR], pdata, 8); // Msg1
    tlv_a5aa_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x288:
    memcpy(&zxup_buffer[ZXUP_A5AA_POS2_ADDR], pdata, 8); // Msg2
    tlv_a5aa_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x388:
    memcpy(&zxup_buffer[ZXUP_A5AA_POS3_ADDR], pdata, 8); // Msg3
    tlv_a5aa_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x488:
    memcpy(&zxup_buffer[ZXUP_A5AA_POS4_ADDR], pdata, 8); // Msg4
    tlv_a5aa_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5AB====================================================================================
  case 0x343:
    memcpy(&zxup_buffer[ZXUP_A5AB_POS1_ADDR], pdata, 8); // �ڵ�״̬
    tlv_a5ab_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5AC====================================================================================
  case 0x493:
    memcpy(&zxup_buffer[ZXUP_A5AC_POS1_ADDR], pdata, 6); // �������ƺͽ��
    zxup_buffer[ZXUP_A5B2_POS3_ADDR] = pdata[6]; // �����������***********
    zxup_buffer[ZXUP_A5B2_POS4_ADDR] = pdata[7]; // ������������***********
    tlv_a5ac_valid_flag = 1;
    tlv_a5b2_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5AD====================================================================================
  case 0x4C3:
    memcpy(&zxup_buffer[ZXUP_A5AD_POS1_ADDR], pdata, 8); // �������ƺͽ��
    zxup_buffer[ZXUP_A5B2_POS5_ADDR] = pdata[6]; // �������Ʊ����***********
    zxup_buffer[ZXUP_A5B2_POS6_ADDR] = pdata[7]; // �������Ʊ����***********
    tlv_a5ad_valid_flag = 1;
    tlv_a5b2_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5AE====================================================================================
  case 0x4B3:
    zxup_buffer[ZXUP_A5AE_POS1_ADDR] = pdata[0]; // �������1
    zxup_buffer[ZXUP_A5AE_POS2_ADDR] = pdata[4]; // �������2
    zxup_buffer[ZXUP_A5AE_POS3_ADDR] = pdata[1]; // ��ؽ��
    zxup_buffer[ZXUP_A5AE_POS4_ADDR] = pdata[2]; // �һ�����1
    zxup_buffer[ZXUP_A5AE_POS5_ADDR] = pdata[5]; // �һ�����2
    zxup_buffer[ZXUP_A5AE_POS6_ADDR] = pdata[3]; // �һؽ��
    tlv_a5ae_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5AF====================================================================================
  case 0x4A3:
    memcpy(&zxup_buffer[ZXUP_A5AF_POS1_ADDR], pdata, 5); // �������������ƺͽ��
    zxup_buffer[ZXUP_A5B2_POS7_ADDR] = pdata[5]; // ��������������***********
    zxup_buffer[ZXUP_A5B2_POS8_ADDR] = pdata[6]; // ��������������***********
    tlv_a5af_valid_flag = 1;
    tlv_a5b2_valid_flag = 1;
    retval = 0x01;
    break;

    //==A5B0====================================================================================
  case 0x4D3:
    memcpy(&zxup_buffer[ZXUP_A5B0_POS1_ADDR], &pdata[3], 4); // �������������ƺͽ��
    tlv_a5b0_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5B1====================================================================================
  // zxup_buffer

  //==A5B2====================================================================================
  case 0x4F3:
    zxup_buffer[ZXUP_A5B2_POS1_ADDR] = pdata[0]; // ����������
    zxup_buffer[ZXUP_A5B2_POS1_ADDR+1] = pdata[1];
    zxup_buffer[ZXUP_A5B2_POS2_ADDR] = pdata[2]; // ����������
    zxup_buffer[ZXUP_A5B2_POS2_ADDR+1] = pdata[3];
    tlv_a5b2_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5B3====================================================================================
  case 0x564:
    zxup_buffer[ZXUP_A5B3_POS1_ADDR] = pdata[0]; // Һѹ���¶�
    zxup_buffer[ZXUP_A5B3_POS1_ADDR+1] = pdata[1];
    zxup_buffer[ZXUP_A5B5_POS10_ADDR] = pdata[4]; // LS1ѹ��*****
    zxup_buffer[ZXUP_A5B5_POS10_ADDR+1] = pdata[5];
    zxup_buffer[ZXUP_A5B5_POS12_ADDR] = pdata[6]; // LS2ѹ��*****
    zxup_buffer[ZXUP_A5B5_POS12_ADDR+1] = pdata[7];
    tlv_a5b3_valid_flag = 1;
    tlv_a5b5_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5B4====================================================================================
  case 0x1A3:
    zxup_buffer[ZXUP_A5B4_POS1_ADDR] = pdata[0]; // 1#���õ�ŷ�
    zxup_buffer[ZXUP_A5B4_POS1_ADDR+1] = pdata[1];
    zxup_buffer[ZXUP_A5B4_POS2_ADDR] = pdata[2]; // 2#���õ�ŷ�
    zxup_buffer[ZXUP_A5B4_POS2_ADDR+1] = pdata[3];

    zxup_buffer[ZXUP_A5B5_POS5_ADDR] = pdata[0]; // �������ŷ�*****
    zxup_buffer[ZXUP_A5B5_POS5_ADDR+1] = pdata[1];
    zxup_buffer[ZXUP_A5B5_POS6_ADDR] = pdata[2]; // �������ŷ�*****
    zxup_buffer[ZXUP_A5B5_POS6_ADDR+1] = pdata[3];

    zxup_buffer[ZXUP_A5BB_POS1_ADDR] = pdata[0]; // ������ŷ�*****
    zxup_buffer[ZXUP_A5BB_POS1_ADDR+1] = pdata[1];
    zxup_buffer[ZXUP_A5BB_POS2_ADDR] = pdata[2]; // �����ŷ�*****
    zxup_buffer[ZXUP_A5BB_POS2_ADDR+1] = pdata[3];

    zxup_buffer[ZXUP_A5BC_POS1_ADDR] = pdata[4]; // ������
    zxup_buffer[ZXUP_A5BC_POS1_ADDR+1] = pdata[5];
    zxup_buffer[ZXUP_A5BC_POS2_ADDR] = pdata[6]; // ��Ͳת��
    zxup_buffer[ZXUP_A5BC_POS2_ADDR+1] = pdata[7];
    tlv_a5b4_valid_flag = 1;
    tlv_a5b5_valid_flag = 1;
    tlv_a5bb_valid_flag = 1;
    tlv_a5bc_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x2A3:
    zxup_buffer[ZXUP_A5B4_POS3_ADDR] = pdata[0]; // ����ѹ��
    zxup_buffer[ZXUP_A5B4_POS3_ADDR+1] = pdata[1];

    zxup_buffer[ZXUP_A5B5_POS9_ADDR] = pdata[0]; // MP1ѹ��*****
    zxup_buffer[ZXUP_A5B5_POS9_ADDR+1] = pdata[1];

    zxup_buffer[ZXUP_A5BB_POS3_ADDR] = pdata[0]; // �ͱ�ѹ��*****
    zxup_buffer[ZXUP_A5BB_POS3_ADDR+1] = pdata[1];

    if (pdata[6]&BIT(0)) // �ƶ���ŷ�*****
      zxup_buffer[ZXUP_A5BC_POS3_ADDR] |= BIT(1); // ��1
    else
      zxup_buffer[ZXUP_A5BC_POS3_ADDR] &= ~BIT(1); // ��0

    if (pdata[6]&BIT(1)) // ѹ���̵���*****
      zxup_buffer[ZXUP_A5BC_POS3_ADDR] |= BIT(2); // ��1
    else
      zxup_buffer[ZXUP_A5BC_POS3_ADDR] &= ~BIT(2); // ��0

    tlv_a5b4_valid_flag = 1;
    tlv_a5b5_valid_flag = 1;
    tlv_a5bb_valid_flag = 1;
    tlv_a5bc_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5B5====================================================================================
  case 0x393:
    zxup_buffer[ZXUP_A5B5_POS1_ADDR] = pdata[0]; // ���ŷ�
    zxup_buffer[ZXUP_A5B5_POS1_ADDR+1] = pdata[1];
    zxup_buffer[ZXUP_A5B5_POS2_ADDR] = pdata[2]; // ����ŷ�
    zxup_buffer[ZXUP_A5B5_POS2_ADDR+1] = pdata[3];
    zxup_buffer[ZXUP_A5B5_POS11_ADDR] = pdata[4]; // MP2ѹ��
    zxup_buffer[ZXUP_A5B5_POS11_ADDR+1] = pdata[5];
    zxup_buffer[ZXUP_A5B7_POS1_ADDR] = pdata[6]; // ������ѹ��*******
    zxup_buffer[ZXUP_A5B7_POS1_ADDR+1] = pdata[7];
    tlv_a5b5_valid_flag = 1;
    tlv_a5b7_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x1C3:
    zxup_buffer[ZXUP_A5B5_POS3_ADDR] = pdata[0]; // ������ŷ�
    zxup_buffer[ZXUP_A5B5_POS3_ADDR+1] = pdata[1];
    zxup_buffer[ZXUP_A5B5_POS4_ADDR] = pdata[2]; // ������ŷ�
    zxup_buffer[ZXUP_A5B5_POS4_ADDR+1] = pdata[3];
    tlv_a5b5_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x3D3:
    zxup_buffer[ZXUP_A5B5_POS7_ADDR] = pdata[0]; // �������ŷ�
    zxup_buffer[ZXUP_A5B5_POS7_ADDR+1] = pdata[1];
    zxup_buffer[ZXUP_A5B5_POS8_ADDR] = pdata[2]; // �������ŷ�
    zxup_buffer[ZXUP_A5B5_POS8_ADDR+1] = pdata[3];
    tlv_a5b5_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x443:
    if (pdata[3]&BIT(3)) // ������ŷ�
      zxup_buffer[ZXUP_A5B5_POS13_ADDR] |= BIT(0); // ��1
    else
      zxup_buffer[ZXUP_A5B5_POS13_ADDR] &= ~BIT(0); // ��0

    if (pdata[0]&BIT(0)) // ���ʧ�ټ��*****
      zxup_buffer[ZXUP_A5BC_POS3_ADDR] |= BIT(0); // ��1
    else
      zxup_buffer[ZXUP_A5BC_POS3_ADDR] &= ~BIT(0); // ��0

    if (pdata[0]&BIT(1)) // ���ʧ�ټ��*****
      zxup_buffer[ZXUP_A5BE_POS3_ADDR] |= BIT(0); // ��1
    else
      zxup_buffer[ZXUP_A5BE_POS3_ADDR] &= ~BIT(0); // ��0

    tlv_a5b5_valid_flag = 1;
    tlv_a5bc_valid_flag = 1;
    tlv_a5be_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5B6====================================================================================
  case 0x264:
    if (pdata[0]&BIT(0)) // 32MPaѹ��
      zxup_buffer[ZXUP_A5B6_POS1_ADDR] |= BIT(0); // ��1
    else
      zxup_buffer[ZXUP_A5B6_POS1_ADDR] &= ~BIT(0); // ��0

    if (pdata[0]&BIT(1)) // 24MPaѹ��1
      zxup_buffer[ZXUP_A5B6_POS1_ADDR] |= BIT(1); // ��1
    else
      zxup_buffer[ZXUP_A5B6_POS1_ADDR] &= ~BIT(1); // ��0
      
    if (pdata[0]&BIT(2)) // 24MPaѹ��2
      zxup_buffer[ZXUP_A5B6_POS1_ADDR] |= BIT(3); // ��1
    else
      zxup_buffer[ZXUP_A5B6_POS1_ADDR] &= ~BIT(3); // ��0
      
    if (pdata[0]&BIT(3)) // 1Mpaѹ��
      zxup_buffer[ZXUP_A5B6_POS1_ADDR] |= BIT(4); // ��1
    else
      zxup_buffer[ZXUP_A5B6_POS1_ADDR] &= ~BIT(4); // ��0

    tlv_a5b6_valid_flag = 1;
    retval = 0x01;

    break;

  case 0x193:
    if (pdata[5]&BIT(5)) // ���ŷ�
      zxup_buffer[ZXUP_A5B6_POS1_ADDR] |= BIT(2); // ��1
    else
      zxup_buffer[ZXUP_A5B6_POS1_ADDR] &= ~BIT(2); // ��0

    if (pdata[5]&BIT(6)) // ����ŷ�
      zxup_buffer[ZXUP_A5B6_POS1_ADDR] |= BIT(5); // ��1
    else
      zxup_buffer[ZXUP_A5B6_POS1_ADDR] &= ~BIT(5); // ��0

    ///////////////////////////////////////////////////////////////////////
    if (pdata[0]&BIT(0)) // ���������
      zxup_buffer[ZXUP_A5B8_POS1_ADDR] |= BIT(0); // ��1
    else
      zxup_buffer[ZXUP_A5B8_POS1_ADDR] &= ~BIT(0); // ��0

    if (pdata[0]&BIT(1)) // ���������
      zxup_buffer[ZXUP_A5B8_POS1_ADDR] |= BIT(1); // ��1
    else
      zxup_buffer[ZXUP_A5B8_POS1_ADDR] &= ~BIT(1); // ��0

    if (pdata[0]&BIT(5)) // ���������
      zxup_buffer[ZXUP_A5B8_POS1_ADDR] |= BIT(2); // ��1
    else
      zxup_buffer[ZXUP_A5B8_POS1_ADDR] &= ~BIT(2); // ��0

    if (pdata[0]&BIT(4)) // ���������
      zxup_buffer[ZXUP_A5B8_POS1_ADDR] |= BIT(3); // ��1
    else
      zxup_buffer[ZXUP_A5B8_POS1_ADDR] &= ~BIT(3); // ��0

    if (pdata[4]&BIT(0)) // ǰ��������
      zxup_buffer[ZXUP_A5B8_POS1_ADDR] |= BIT(4); // ��1
    else
      zxup_buffer[ZXUP_A5B8_POS1_ADDR] &= ~BIT(4); // ��0

    if (pdata[4]&BIT(1)) // ǰ��������
      zxup_buffer[ZXUP_A5B8_POS1_ADDR] |= BIT(5); // ��1
    else
      zxup_buffer[ZXUP_A5B8_POS1_ADDR] &= ~BIT(5); // ��0

    if (pdata[4]&BIT(5)) // ǰ��������
      zxup_buffer[ZXUP_A5B8_POS1_ADDR] |= BIT(6); // ��1
    else
      zxup_buffer[ZXUP_A5B8_POS1_ADDR] &= ~BIT(6); // ��0

    if (pdata[4]&BIT(4)) // ǰ��������
      zxup_buffer[ZXUP_A5B8_POS1_ADDR] |= BIT(7); // ��1
    else
      zxup_buffer[ZXUP_A5B8_POS1_ADDR] &= ~BIT(7); // ��0

    if (pdata[0]&BIT(6)) // ǰ��ͷ���־
      zxup_buffer[ZXUP_A5B8_POS2_ADDR] |= BIT(0); // ��1
    else
      zxup_buffer[ZXUP_A5B8_POS2_ADDR] &= ~BIT(0); // ��0

    if (pdata[0]&BIT(7)) // ���ͷ���־
      zxup_buffer[ZXUP_A5B8_POS2_ADDR] |= BIT(1); // ��1
    else
      zxup_buffer[ZXUP_A5B8_POS2_ADDR] &= ~BIT(1); // ��0

    zxup_buffer[ZXUP_A5B8_POS5_ADDR] = pdata[1];

    if (pdata[5]&BIT(0)) // �ױ������͵�ŷ�
      zxup_buffer[ZXUP_A5B9_POS1_ADDR] |= BIT(0); // ��1
    else
      zxup_buffer[ZXUP_A5B9_POS1_ADDR] &= ~BIT(0); // ��0

    if (pdata[5]&BIT(1)) // �ױ����л���ŷ�
      zxup_buffer[ZXUP_A5B9_POS1_ADDR] |= BIT(1); // ��1
    else
      zxup_buffer[ZXUP_A5B9_POS1_ADDR] &= ~BIT(1); // ��0

    tlv_a5b6_valid_flag = 1;
    tlv_a5b8_valid_flag = 1;
    tlv_a5b9_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x2C3:
    if (pdata[2]&BIT(0)) // ������ŷ�
      zxup_buffer[ZXUP_A5B6_POS1_ADDR] |= BIT(6); // ��1
    else
      zxup_buffer[ZXUP_A5B6_POS1_ADDR] &= ~BIT(6); // ��0

    if (pdata[2]&BIT(3)) // ����������䷧
      zxup_buffer[ZXUP_A5B6_POS1_ADDR] |= BIT(7); // ��1
    else
      zxup_buffer[ZXUP_A5B6_POS1_ADDR] &= ~BIT(7); // ��0

    zxup_buffer[ZXUP_A5BA_POS1_ADDR] = pdata[6]; // ����ƽ�ⷧ����
    zxup_buffer[ZXUP_A5BA_POS1_ADDR+1] = pdata[7];
    tlv_a5b6_valid_flag = 1;
    tlv_a5ba_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5B7====================================================================================
  case 0x293:
    zxup_buffer[ZXUP_A5B7_POS2_ADDR] = pdata[2]; // �����׳���
    zxup_buffer[ZXUP_A5B7_POS2_ADDR+1] = pdata[3];
    tlv_a5b7_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x394:
    zxup_buffer[ZXUP_A5B7_POS3_ADDR] = pdata[6]; // ���ƽ�ⷧ
    zxup_buffer[ZXUP_A5B7_POS3_ADDR+1] = pdata[7];
    tlv_a5b7_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5B8====================================================================================
  case 0x1E9:
    zxup_buffer[ZXUP_A5B8_POS3_ADDR] = pdata[0]; // ǰ��ͷ���⿪��8
    zxup_buffer[ZXUP_A5B8_POS4_ADDR] = pdata[1]; // ���ͷ���⿪��8
    zxup_buffer[ZXUP_A5B9_POS1_ADDR] = pdata[6]; // ������ѹ��
    zxup_buffer[ZXUP_A5B9_POS1_ADDR+1] = pdata[7];
    tlv_a5b8_valid_flag = 1;
    tlv_a5b9_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5B9====================================================================================

  //==A5BA====================================================================================
  case 0x3C3:
    zxup_buffer[ZXUP_A5BA_POS2_ADDR] = pdata[0]; // �ұ��ƽ�ⷧ����
    zxup_buffer[ZXUP_A5BA_POS2_ADDR+1] = pdata[1];
    tlv_a5ba_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5BB====================================================================================
  //==A5BC====================================================================================

  //==A5BD====================================================================================
  case 0x2F3:
    zxup_buffer[ZXUP_A5BD_POS1_ADDR] = pdata[0]; // ������ŷ�
    zxup_buffer[ZXUP_A5BD_POS1_ADDR+1] = pdata[1];
    zxup_buffer[ZXUP_A5BD_POS2_ADDR] = pdata[2]; // �����ŷ�
    zxup_buffer[ZXUP_A5BD_POS2_ADDR+1] = pdata[3];
    zxup_buffer[ZXUP_A5BD_POS3_ADDR] = pdata[4]; // �ͱ�ѹ��
    zxup_buffer[ZXUP_A5BD_POS3_ADDR+1] = pdata[5];
    tlv_a5bd_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5BE====================================================================================
  // zxup_buffer[ZXUP_A5BE_POS1_ADDR]

  //==A5BF====================================================================================
  case 0x2B3:
    zxup_buffer[ZXUP_A5BF_POS1_ADDR] = pdata[0]; // ���ת��ŷ�
    zxup_buffer[ZXUP_A5BF_POS1_ADDR+1] = pdata[1];
    zxup_buffer[ZXUP_A5BF_POS2_ADDR] = pdata[2]; // �һ�ת��ŷ�
    zxup_buffer[ZXUP_A5BF_POS2_ADDR+1] = pdata[3];
    zxup_buffer[ZXUP_A5BF_POS4_ADDR] = pdata[6]; // �ͱ�ѹ������תѹ����⣩
    zxup_buffer[ZXUP_A5BF_POS4_ADDR+1] = pdata[7];
    tlv_a5bf_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5C0====================================================================================
  case 0x3B3:
    if (pdata[0]&BIT(0)) // �ƶ����Ʒ�
      zxup_buffer[ZXUP_A5C0_POS1_ADDR] |= BIT(0); // ��1
    else
      zxup_buffer[ZXUP_A5C0_POS1_ADDR] &= ~BIT(0); // ��0

    if (pdata[0]&BIT(1)) // ѹ�����
      zxup_buffer[ZXUP_A5C0_POS1_ADDR] |= BIT(1); // ��1
    else
      zxup_buffer[ZXUP_A5C0_POS1_ADDR] &= ~BIT(1); // ��0

    zxup_buffer[ZXUP_A5C0_POS2_ADDR] = pdata[6]; // ��ת�ƶ���-����ѹ��
    zxup_buffer[ZXUP_A5C0_POS2_ADDR+1] = pdata[7];
    tlv_a5c0_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5C1====================================================================================
  case 0x364:
    zxup_buffer[ZXUP_A5C1_POS1_ADDR] = pdata[4];
    zxup_buffer[ZXUP_A5C1_POS2_ADDR] = pdata[5];
    tlv_a5c1_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5C2====================================================================================
  case 0x3E4:
    zxup_buffer[ZXUP_A5C2_POS1_ADDR] = pdata[6];  // ѹ��ѡ��
    zxup_buffer[ZXUP_A5C2_POS1_ADDR+1] = pdata[7];

    if (pdata[5]&BIT(0)) // �л�����Y35��
      zxup_buffer[ZXUP_A5C2_POS2_ADDR] |= BIT(0); // ��1
    else
      zxup_buffer[ZXUP_A5C2_POS2_ADDR] &= ~BIT(0); // ��0

    if (pdata[5]&BIT(1)) // �л�����(Y33A)
      zxup_buffer[ZXUP_A5C2_POS2_ADDR] |= BIT(1); // ��1
    else
      zxup_buffer[ZXUP_A5C2_POS2_ADDR] &= ~BIT(1); // ��0

    if (pdata[5]&BIT(2)) // �л�����(Y33B)
      zxup_buffer[ZXUP_A5C2_POS2_ADDR] |= BIT(2); // ��1
    else
      zxup_buffer[ZXUP_A5C2_POS2_ADDR] &= ~BIT(2); // ��0
    
    tlv_a5c2_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5C3====================================================================================
  //==A5C4====================================================================================
  case 0x273:
    if (pdata[0]&BIT(0)) // ��-��������
      zxup_buffer[ZXUP_A5C3_POS4_ADDR] |= BIT(0); // ��1
    else
      zxup_buffer[ZXUP_A5C3_POS4_ADDR] &= ~BIT(0); // ��0

    if (pdata[0]&BIT(1)) // ��-��������
      zxup_buffer[ZXUP_A5C3_POS4_ADDR] |= BIT(1); // ��1
    else
      zxup_buffer[ZXUP_A5C3_POS4_ADDR] &= ~BIT(1); // ��0
    
    if (pdata[0]&BIT(2)) // ��-����չ��
      zxup_buffer[ZXUP_A5C3_POS4_ADDR] |= BIT(2); // ��1
    else
      zxup_buffer[ZXUP_A5C3_POS4_ADDR] &= ~BIT(2); // ��0
    
    if (pdata[0]&BIT(3)) // ��-�����ջ�
      zxup_buffer[ZXUP_A5C3_POS4_ADDR] |= BIT(3); // ��1
    else
      zxup_buffer[ZXUP_A5C3_POS4_ADDR] &= ~BIT(3); // ��0
    
    zxup_buffer[ZXUP_A5C3_POS1_ADDR] = pdata[2]; // ��-������
    zxup_buffer[ZXUP_A5C3_POS1_ADDR+1] = pdata[3];
    zxup_buffer[ZXUP_A5C3_POS2_ADDR] = pdata[6]; // ��-������
    zxup_buffer[ZXUP_A5C3_POS2_ADDR+1] = pdata[7];
    zxup_buffer[ZXUP_A5C3_POS7_ADDR] = pdata[4]; // ��-�Ž�����
    zxup_buffer[ZXUP_A5C3_POS7_ADDR+1] = pdata[5];
    tlv_a5c3_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x373:
    if (pdata[0]&BIT(7)) // ��-����ѹ��ѡ��
      zxup_buffer[ZXUP_A5C3_POS4_ADDR] |= BIT(4); // ��1
    else
      zxup_buffer[ZXUP_A5C3_POS4_ADDR] &= ~BIT(4); // ��0

    if (pdata[1]&BIT(3)) // ��-�����ƶ�
      zxup_buffer[ZXUP_A5C3_POS4_ADDR] |= BIT(5); // ��1
    else
      zxup_buffer[ZXUP_A5C3_POS4_ADDR] &= ~BIT(5); // ��0
    
    if (pdata[1]&BIT(1)) // ��-���ֽ���
      zxup_buffer[ZXUP_A5C3_POS4_ADDR] |= BIT(6); // ��1
    else
      zxup_buffer[ZXUP_A5C3_POS4_ADDR] &= ~BIT(6); // ��0
    
    if (pdata[1]&BIT(0)) // ��-��������
      zxup_buffer[ZXUP_A5C3_POS4_ADDR] |= BIT(7); // ��1
    else
      zxup_buffer[ZXUP_A5C3_POS4_ADDR] &= ~BIT(7); // ��0
    
    if (pdata[1]&BIT(2)) // ��-�Ž��׽���
      zxup_buffer[ZXUP_A5C3_POS5_ADDR] |= BIT(0); // ��1
    else
      zxup_buffer[ZXUP_A5C3_POS5_ADDR] &= ~BIT(0); // ��0
    
    if (pdata[1]&BIT(4)) // ��-������︡��
      zxup_buffer[ZXUP_A5C3_POS5_ADDR] |= BIT(1); // ��1
    else
      zxup_buffer[ZXUP_A5C3_POS5_ADDR] &= ~BIT(1); // ��0
    
    zxup_buffer[ZXUP_A5C4_POS1_ADDR] = pdata[2]; // ��-������
    zxup_buffer[ZXUP_A5C4_POS1_ADDR+1] = pdata[3];
    zxup_buffer[ZXUP_A5C4_POS2_ADDR] = pdata[6]; // ��-������
    zxup_buffer[ZXUP_A5C4_POS2_ADDR+1] = pdata[7];
    zxup_buffer[ZXUP_A5C4_POS7_ADDR] = pdata[4]; // ��-�Ž�����
    zxup_buffer[ZXUP_A5C4_POS7_ADDR+1] = pdata[5];
    
    if (pdata[0]&BIT(0)) // ��-��������
      zxup_buffer[ZXUP_A5C4_POS4_ADDR] |= BIT(0); // ��1
    else
      zxup_buffer[ZXUP_A5C4_POS4_ADDR] &= ~BIT(0); // ��0

    if (pdata[0]&BIT(1)) // ��-��������
      zxup_buffer[ZXUP_A5C4_POS4_ADDR] |= BIT(1); // ��1
    else
      zxup_buffer[ZXUP_A5C4_POS4_ADDR] &= ~BIT(1); // ��0
    
    if (pdata[0]&BIT(2)) // ��-����չ��
      zxup_buffer[ZXUP_A5C4_POS4_ADDR] |= BIT(2); // ��1
    else
      zxup_buffer[ZXUP_A5C4_POS4_ADDR] &= ~BIT(2); // ��0
    
    if (pdata[0]&BIT(3)) // ��-�����ջ�
      zxup_buffer[ZXUP_A5C4_POS4_ADDR] |= BIT(3); // ��1
    else
      zxup_buffer[ZXUP_A5C4_POS4_ADDR] &= ~BIT(3); // ��0

    if (pdata[0]&BIT(7)) // ��-����ѹ��ѡ��
      zxup_buffer[ZXUP_A5C4_POS4_ADDR] |= BIT(4); // ��1
    else
      zxup_buffer[ZXUP_A5C4_POS4_ADDR] &= ~BIT(4); // ��0

    if (pdata[1]&BIT(3)) // ��-�����ƶ�
      zxup_buffer[ZXUP_A5C4_POS4_ADDR] |= BIT(5); // ��1
    else
      zxup_buffer[ZXUP_A5C4_POS4_ADDR] &= ~BIT(5); // ��0
    
    if (pdata[1]&BIT(1)) // ��-���ֽ���
      zxup_buffer[ZXUP_A5C4_POS4_ADDR] |= BIT(6); // ��1
    else
      zxup_buffer[ZXUP_A5C4_POS4_ADDR] &= ~BIT(6); // ��0
    
    if (pdata[1]&BIT(0)) // ��-��������
      zxup_buffer[ZXUP_A5C4_POS4_ADDR] |= BIT(7); // ��1
    else
      zxup_buffer[ZXUP_A5C4_POS4_ADDR] &= ~BIT(7); // ��0
    
    if (pdata[1]&BIT(2)) // ��-�Ž��׽���
      zxup_buffer[ZXUP_A5C4_POS5_ADDR] |= BIT(0); // ��1
    else
      zxup_buffer[ZXUP_A5C4_POS5_ADDR] &= ~BIT(0); // ��0
    
    if (pdata[1]&BIT(4)) // ��-������︡��
      zxup_buffer[ZXUP_A5C4_POS5_ADDR] |= BIT(1); // ��1
    else
      zxup_buffer[ZXUP_A5C4_POS5_ADDR] &= ~BIT(1); // ��0
    
    tlv_a5c3_valid_flag = 1;
    tlv_a5c4_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x504:
    zxup_buffer[ZXUP_A5C3_POS3_ADDR] = pdata[0]; // ��-����������
    zxup_buffer[ZXUP_A5C3_POS3_ADDR+1] = pdata[1];
    zxup_buffer[ZXUP_A5C4_POS3_ADDR] = pdata[2]; // ��-����������
    zxup_buffer[ZXUP_A5C4_POS3_ADDR+1] = pdata[3];
    tlv_a5c3_valid_flag = 1;
    tlv_a5c4_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x473:
    zxup_buffer[ZXUP_A5C3_POS6_ADDR] = pdata[0]; // ��-�Ž�����
    zxup_buffer[ZXUP_A5C3_POS6_ADDR+1] = pdata[1];
    zxup_buffer[ZXUP_A5C4_POS6_ADDR] = pdata[2]; // ��-�Ž�����
    zxup_buffer[ZXUP_A5C4_POS6_ADDR+1] = pdata[3];
    tlv_a5c3_valid_flag = 1;
    tlv_a5c4_valid_flag = 1;
    retval = 0x01;
    break;

  //==�ϳ�ϵͳ�汾====================================================================================
  case 0x562:
    zxversion_buffer[ZXVERSION_A505_POS6_ADDR] = pdata[0];  // ��ʾ��2�ײ�
    zxversion_buffer[ZXVERSION_A505_POS6_ADDR+1] = pdata[1];
    zxversion_buffer[ZXVERSION_A505_POS6_ADDR+2] = pdata[2];
    tlv_a505_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x571:
    zxversion_buffer[ZXVERSION_A505_POS2_ADDR] = pdata[4];  // ��ʾ��1
    zxversion_buffer[ZXVERSION_A505_POS2_ADDR+1] = pdata[5];
    zxversion_buffer[ZXVERSION_A505_POS2_ADDR+2] = pdata[6];
    tlv_a505_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x572:
    zxversion_buffer[ZXVERSION_A505_POS6_ADDR] = pdata[4];  // ��ʾ��2
    zxversion_buffer[ZXVERSION_A505_POS6_ADDR+1] = pdata[5];
    zxversion_buffer[ZXVERSION_A505_POS6_ADDR+2] = pdata[6];
    tlv_a505_valid_flag = 1;
    retval = 0x00;
    break;

  case 0x573:
    zxversion_buffer[ZXVERSION_A505_POS4_ADDR] = pdata[4];  // ������
    zxversion_buffer[ZXVERSION_A505_POS4_ADDR+1] = pdata[5];
    zxversion_buffer[ZXVERSION_A505_POS4_ADDR+2] = pdata[6];
    tlv_a505_valid_flag = 1;
    retval = 0x00;
    break;
    
  case 0x575:
    zxversion_buffer[ZXVERSION_A505_POS1_ADDR] = pdata[4];  // ����������
    zxversion_buffer[ZXVERSION_A505_POS1_ADDR+1] = pdata[5];
    zxversion_buffer[ZXVERSION_A505_POS1_ADDR+2] = pdata[6];
    tlv_a505_valid_flag = 1;
    retval = 0x01;
    break;

  default:
    break;
  }

  return retval;
}

/*************************************************************************
 * ������յ���dCAN����(�³�)
*************************************************************************/
uint8_t CAN_ProcessRecvDownMsg(uint32_t canId, uint8_t *pdata, uint8_t size)
{
  uint8_t retval = 0x00;
  //uint32_t tempVal;

  switch (canId) // �³�ͨ��
  {
  //==A5A3====================================================================================
  case 0x24D:
    memcpy(&zxdown_buffer[ZXDOWN_A5A3_POS1_ADDR], pdata, 8); // ˮƽ֧�ȳ���
    tlv_a5a3_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x22A:
    zxdown_buffer[ZXDOWN_A5A3_POS2_ADDR] = pdata[0];   // ��ǰ֧��ѹ��
    zxdown_buffer[ZXDOWN_A5A3_POS2_ADDR+1] = pdata[1];
    zxdown_buffer[ZXDOWN_A5A3_POS2_ADDR+2] = pdata[2]; // ��ǰ֧��ѹ��
    zxdown_buffer[ZXDOWN_A5A3_POS2_ADDR+3] = pdata[3];
    zxdown_buffer[ZXDOWN_A5A3_POS2_ADDR+4] = pdata[4]; // ���֧��ѹ��
    zxdown_buffer[ZXDOWN_A5A3_POS2_ADDR+5] = pdata[5];
    zxdown_buffer[ZXDOWN_A5A3_POS2_ADDR+6] = pdata[6]; // �Һ�֧��ѹ��
    zxdown_buffer[ZXDOWN_A5A3_POS2_ADDR+7] = pdata[7];
    tlv_a5a3_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x24E:
    memcpy(&zxdown_buffer[ZXDOWN_A5A3_POS3_ADDR], pdata, 8); // �ڸ׳���
    tlv_a5a3_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5A4====================================================================================
  case 0x24B:
    if (pdata[1]&BIT(4)) // Һѹ�����״̬
      zxdown_buffer[ZXDOWN_A5EA_POS2_ADDR] |= BIT(1); // ��1
    else
      zxdown_buffer[ZXDOWN_A5EA_POS2_ADDR] &= ~BIT(1); // ��0
    
    zxdown_buffer[ZXDOWN_A5A4_POS1_ADDR] = pdata[0]; // ����֧��״̬
    zxdown_buffer[ZXDOWN_A5E9_POS1_ADDR] = pdata[6]; // Һѹ���¶�
    zxdown_buffer[ZXDOWN_A5EA_POS8_ADDR] = pdata[2]; // ɢ����Һѹ�����ÿ��Ƶ���
    zxdown_buffer[ZXDOWN_A5EA_POS8_ADDR+1] = pdata[3];
    tlv_a5a4_valid_flag = 1;
    tlv_a5e9_valid_flag = 1;
    tlv_a5ea_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5E0===================================================================================
  case 0x23B:
    zxdown_buffer[ZXDOWN_A5E0_POS1_ADDR] = pdata[0]; // �ڵ�״̬1
    
    if (pdata[1]&BIT(0)) // ����������ź�
      zxdown_buffer[ZXDOWN_A5E0_POS2_ADDR] |= BIT(0); // ��1
    else
      zxdown_buffer[ZXDOWN_A5E0_POS2_ADDR] &= ~BIT(0); // ��0

    if (pdata[1]&BIT(1)) // ABS�����ź�
      zxdown_buffer[ZXDOWN_A5E0_POS2_ADDR] |= BIT(1); // ��1
    else
      zxdown_buffer[ZXDOWN_A5E0_POS2_ADDR] &= ~BIT(1); // ��0
    
    if (pdata[1]&BIT(4)) // �������¶ȸ߼��
      zxdown_buffer[ZXDOWN_A5E6_POS8_ADDR] |= BIT(1); // ��1
    else
      zxdown_buffer[ZXDOWN_A5E6_POS8_ADDR] &= ~BIT(1); // ��0
    
    if (pdata[1]&BIT(3)) // �ƶ���ĥ����
      zxdown_buffer[ZXDOWN_A5E6_POS8_ADDR] |= BIT(2); // ��1
    else
      zxdown_buffer[ZXDOWN_A5E6_POS8_ADDR] &= ~BIT(2); // ��0
    
    if (pdata[1]&BIT(3)) // �ƶ���Ƭĥ���⿪�� ?????????????????????
      zxdown_buffer[ZXDOWN_A5E6_POS8_ADDR] |= BIT(3); // ��1
    else
      zxdown_buffer[ZXDOWN_A5E6_POS8_ADDR] &= ~BIT(3); // ��0
    
    if (pdata[1]&BIT(2)) // ABS����״̬
      zxdown_buffer[ZXDOWN_A5E6_POS8_ADDR] |= BIT(6); // ��1
    else
      zxdown_buffer[ZXDOWN_A5E6_POS8_ADDR] &= ~BIT(6); // ��0
    
    tlv_a5e0_valid_flag = 1;
    tlv_a5e6_valid_flag = 1;
    retval = 0x01;
    break;
    
  //==A5E1====================================================================================
  case 0x21D:
    zxdown_buffer[ZXDOWN_A5E1_POS1_ADDR] = pdata[7];
    if (pdata[0]&BIT(0)) // �ֶ���յ�
      zxdown_buffer[ZXDOWN_A5E8_POS2_ADDR] |= BIT(0); // ��1
    else
      zxdown_buffer[ZXDOWN_A5E8_POS2_ADDR] &= ~BIT(0); // ��0
    
    if (pdata[0]&BIT(1)) // ȡ��standby
      zxdown_buffer[ZXDOWN_A5E8_POS2_ADDR] |= BIT(1); // ��1
    else
      zxdown_buffer[ZXDOWN_A5E8_POS2_ADDR] &= ~BIT(1); // ��0
    
    if (pdata[3]&BIT(0)) // bit0:����ECOģʽ
      zxdown_buffer[ZXDOWN_A5E8_POS2_ADDR] |= BIT(2); // ��1
    else
      zxdown_buffer[ZXDOWN_A5E8_POS2_ADDR] &= ~BIT(2); // ��0
    
    if (pdata[0]&BIT(2)) // ȡ������
      zxdown_buffer[ZXDOWN_A5E8_POS2_ADDR] |= BIT(3); // ��1
    else
      zxdown_buffer[ZXDOWN_A5E8_POS2_ADDR] &= ~BIT(3); // ��0
    
    if (pdata[3]&BIT(4)) // ������յ���� У��
      zxdown_buffer[ZXDOWN_A5E1_POS2_ADDR] |= BIT(0); // ��1
    else
      zxdown_buffer[ZXDOWN_A5E1_POS2_ADDR] &= ~BIT(0); // ��0
    
    if (pdata[3]&BIT(5)) // �����䵹����� У��
      zxdown_buffer[ZXDOWN_A5E1_POS2_ADDR] |= BIT(1); // ��1
    else
      zxdown_buffer[ZXDOWN_A5E1_POS2_ADDR] &= ~BIT(1); // ��0
    
    if (pdata[1]&BIT(0)) // ����������� У��
      zxdown_buffer[ZXDOWN_A5E1_POS2_ADDR] |= BIT(2); // ��1
    else
      zxdown_buffer[ZXDOWN_A5E1_POS2_ADDR] &= ~BIT(2); // ��0
    
    if (pdata[1]&BIT(1)) // �ֶ���ߵ����
      zxdown_buffer[ZXDOWN_A5E1_POS2_ADDR] |= BIT(3); // ��1
    else
      zxdown_buffer[ZXDOWN_A5E1_POS2_ADDR] &= ~BIT(3); // ��0
    
    if (pdata[1]&BIT(2)) // �ֶ���͵����
      zxdown_buffer[ZXDOWN_A5E1_POS2_ADDR] |= BIT(4); // ��1
    else
      zxdown_buffer[ZXDOWN_A5E1_POS2_ADDR] &= ~BIT(4); // ��0

    if (pdata[1]&BIT(3)) // �ּ���ټ��
      zxdown_buffer[ZXDOWN_A5E1_POS2_ADDR] |= BIT(5); // ��1
    else
      zxdown_buffer[ZXDOWN_A5E1_POS2_ADDR] &= ~BIT(5); // ��0
    
    if (pdata[1]&BIT(4)) // �����ټ��
      zxdown_buffer[ZXDOWN_A5E1_POS2_ADDR] |= BIT(6); // ��1
    else
      zxdown_buffer[ZXDOWN_A5E1_POS2_ADDR] &= ~BIT(6); // ��0
    
    if (pdata[6]&BIT(2)) // �ֶ���͵����
      zxdown_buffer[ZXDOWN_A5EA_POS2_ADDR] |= BIT(0); // ��1
    else
      zxdown_buffer[ZXDOWN_A5EA_POS2_ADDR] &= ~BIT(0); // ��0
    
    if (pdata[6]&BIT(6)) // ��ɲ���(���ƶ���⿪��)
      zxdown_buffer[ZXDOWN_A5E6_POS8_ADDR] |= BIT(0); // ��1
    else
      zxdown_buffer[ZXDOWN_A5E6_POS8_ADDR] &= ~BIT(0); // ��0
    
    if (pdata[3]&BIT(6)) // �г��ƶ���⿪��
      zxdown_buffer[ZXDOWN_A5E6_POS8_ADDR] |= BIT(4); // ��1
    else
      zxdown_buffer[ZXDOWN_A5E6_POS8_ADDR] &= ~BIT(4); // ��0
    
    if (pdata[2]&BIT(5)) // ����ѹ����
      zxdown_buffer[ZXDOWN_A5E6_POS8_ADDR] |= BIT(5); // ��1
    else
      zxdown_buffer[ZXDOWN_A5E6_POS8_ADDR] &= ~BIT(5); // ��0
    
    if (pdata[6]&BIT(7)) // PTO���
      zxdown_buffer[ZXDOWN_A5E7_POS1_ADDR] |= BIT(0); // ��1
    else
      zxdown_buffer[ZXDOWN_A5E7_POS1_ADDR] &= ~BIT(0); // ��0

    zxdown_buffer[ZXDOWN_A5E8_POS1_ADDR] = pdata[2]; // ȡ���ҽ�(�ֽ�״̬λ)
    tlv_a5e1_valid_flag = 1;
    tlv_a5e6_valid_flag = 1;
    tlv_a5e7_valid_flag = 1;
    tlv_a5e8_valid_flag = 1;
    tlv_a5ea_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x25E:
    zxdown_buffer[ZXDOWN_A5E1_POS3_ADDR] = pdata[0];  // ����������
    zxdown_buffer[ZXDOWN_A5E1_POS3_ADDR+1] = pdata[1];
    zxdown_buffer[ZXDOWN_A5E1_POS4_ADDR] = pdata[3];  // �����������ѹ
    zxdown_buffer[ZXDOWN_A5E1_POS5_ADDR] = pdata[4];  // �����λ�� 
    
    if (pdata[5]&BIT(0)) // �������ƶ�
      zxdown_buffer[ZXDOWN_A5E6_POS8_ADDR] |= BIT(7); // ��1
    else
      zxdown_buffer[ZXDOWN_A5E6_POS8_ADDR] &= ~BIT(7); // ��0
    
    if (pdata[5]&BIT(1)) // �������ƶ�1��
      zxdown_buffer[ZXDOWN_A5E6_POS9_ADDR] |= BIT(0); // ��1
    else
      zxdown_buffer[ZXDOWN_A5E6_POS9_ADDR] &= ~BIT(0); // ��0
    
    if (pdata[5]&BIT(2)) // �������ƶ�2��
      zxdown_buffer[ZXDOWN_A5E6_POS9_ADDR] |= BIT(1); // ��1
    else
      zxdown_buffer[ZXDOWN_A5E6_POS9_ADDR] &= ~BIT(1); // ��0
    
    if (pdata[5]&BIT(3)) // �������ƶ�3��
      zxdown_buffer[ZXDOWN_A5E6_POS9_ADDR] |= BIT(2); // ��1
    else
      zxdown_buffer[ZXDOWN_A5E6_POS9_ADDR] &= ~BIT(2); // ��0
    
    if (pdata[5]&BIT(4)) // �������ƶ�4��
      zxdown_buffer[ZXDOWN_A5E6_POS9_ADDR] |= BIT(3); // ��1
    else
      zxdown_buffer[ZXDOWN_A5E6_POS9_ADDR] &= ~BIT(3); // ��0
    
    if (pdata[5]&BIT(5)) // ������Ϩ������
      zxdown_buffer[ZXDOWN_A5E7_POS1_ADDR] |= BIT(1); // ��1
    else
      zxdown_buffer[ZXDOWN_A5E7_POS1_ADDR] &= ~BIT(1); // ��0
    
    if (pdata[5]&BIT(6)) // ������Ϩ�����
      zxdown_buffer[ZXDOWN_A5E7_POS1_ADDR] |= BIT(2); // ��1
    else
      zxdown_buffer[ZXDOWN_A5E7_POS1_ADDR] &= ~BIT(2); // ��0
    
    zxdown_buffer[ZXDOWN_A5E5_POS5_ADDR] = pdata[0];  // ����������
    zxdown_buffer[ZXDOWN_A5E5_POS5_ADDR+1] = pdata[1];
    zxdown_buffer[ZXDOWN_A5E5_POS6_ADDR] = pdata[2];  // �������ذٷֱ�
    zxdown_buffer[ZXDOWN_A5EA_POS7_ADDR] = pdata[6]; // �����Ƶ���
    zxdown_buffer[ZXDOWN_A5EA_POS7_ADDR+1] = pdata[7];
    tlv_a5e1_valid_flag = 1;
    tlv_a5e5_valid_flag = 1;
    tlv_a5e6_valid_flag = 1;
    tlv_a5e7_valid_flag = 1;
    tlv_a5ea_valid_flag = 1;
    retval = 0x01;
    break;
    
  //==A5E2====================================================================================
  case 0x24F:
    zxdown_buffer[ZXDOWN_A5E2_POS1_ADDR] = pdata[7]; // ֧�ȴ�ֱ״̬λ
    zxdown_buffer[ZXDOWN_A5E2_POS2_ADDR] = pdata[5]; // ֧��ˮƽ״̬λ
    
    if (pdata[6]&BIT(2)) // ��ǰ�ڱ���
      zxdown_buffer[ZXDOWN_A5E2_POS3_ADDR] |= BIT(0); // ��1
    else
      zxdown_buffer[ZXDOWN_A5E2_POS3_ADDR] &= ~BIT(0); // ��0
    
    if (pdata[6]&BIT(3)) // ��ǰ�ڱ���
      zxdown_buffer[ZXDOWN_A5E2_POS3_ADDR] |= BIT(1); // ��1
    else
      zxdown_buffer[ZXDOWN_A5E2_POS3_ADDR] &= ~BIT(1); // ��0
    
    if (pdata[6]&BIT(0)) // ��ǰ�ڱ���
      zxdown_buffer[ZXDOWN_A5E2_POS3_ADDR] |= BIT(2); // ��1
    else
      zxdown_buffer[ZXDOWN_A5E2_POS3_ADDR] &= ~BIT(2); // ��0
    
    if (pdata[6]&BIT(1)) // ��ǰ�ڱ���
      zxdown_buffer[ZXDOWN_A5E2_POS2_ADDR] |= BIT(3); // ��1
    else
      zxdown_buffer[ZXDOWN_A5E2_POS2_ADDR] &= ~BIT(3); // ��0
    
    if (pdata[6]&BIT(6)) // �Һ�ڱ���
      zxdown_buffer[ZXDOWN_A5E2_POS2_ADDR] |= BIT(4); // ��1
    else
      zxdown_buffer[ZXDOWN_A5E2_POS2_ADDR] &= ~BIT(4); // ��0
    
    if (pdata[6]&BIT(7)) // �Һ�ڱ���
      zxdown_buffer[ZXDOWN_A5E2_POS2_ADDR] |= BIT(5); // ��1
    else
      zxdown_buffer[ZXDOWN_A5E2_POS2_ADDR] &= ~BIT(5); // ��0
    
    if (pdata[6]&BIT(4)) // ���ڱ���
      zxdown_buffer[ZXDOWN_A5E2_POS2_ADDR] |= BIT(6); // ��1
    else
      zxdown_buffer[ZXDOWN_A5E2_POS2_ADDR] &= ~BIT(6); // ��0
    
    if (pdata[6]&BIT(5)) // ���ڱ���
      zxdown_buffer[ZXDOWN_A5E2_POS2_ADDR] |= BIT(7); // ��1
    else
      zxdown_buffer[ZXDOWN_A5E2_POS2_ADDR] &= ~BIT(7); // ��0
    
    tlv_a5e2_valid_flag = 1;
    retval = 0x01;
    break;
    
  //==A5E3====================================================================================
  case 0x22B:
    memcpy(&zxdown_buffer[ZXDOWN_A5E3_POS1_ADDR], pdata, 8); // ����ѹ��
    tlv_a5e3_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x22C:
    memcpy(&zxdown_buffer[ZXDOWN_A5E3_POS2_ADDR], pdata, 8); // �����г�
    tlv_a5e3_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x21E:
    memcpy(&zxdown_buffer[ZXDOWN_A5E3_POS3_ADDR], pdata, 4); // ��������
    tlv_a5e3_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x25F:
    if (pdata[1]&BIT(0)) // ��ǰ�ڱ���
      zxdown_buffer[ZXDOWN_A5E3_POS4_ADDR] |= BIT(0); // ��1
    else
      zxdown_buffer[ZXDOWN_A5E3_POS4_ADDR] &= ~BIT(0); // ��0

    if (pdata[1]&BIT(1)) // ��ǰ�ڱ���
      zxdown_buffer[ZXDOWN_A5E3_POS4_ADDR] |= BIT(1); // ��1
    else
      zxdown_buffer[ZXDOWN_A5E3_POS4_ADDR] &= ~BIT(1); // ��0
    
    if (pdata[0]&BIT(0)) // ��ǰ�ڱ���
      zxdown_buffer[ZXDOWN_A5E3_POS4_ADDR] |= BIT(2); // ��1
    else
      zxdown_buffer[ZXDOWN_A5E3_POS4_ADDR] &= ~BIT(2); // ��0

    if (pdata[0]&BIT(1)) // ��ǰ�ڱ���
      zxdown_buffer[ZXDOWN_A5E3_POS4_ADDR] |= BIT(3); // ��1
    else
      zxdown_buffer[ZXDOWN_A5E3_POS4_ADDR] &= ~BIT(3); // ��0
    
    if (pdata[0]&BIT(2)) // ��ǰ�ڱ���
      zxdown_buffer[ZXDOWN_A5E3_POS4_ADDR] |= BIT(4); // ��1
    else
      zxdown_buffer[ZXDOWN_A5E3_POS4_ADDR] &= ~BIT(4); // ��0
    
    if (pdata[0]&BIT(3)) // ��ǰ�ڱ���
      zxdown_buffer[ZXDOWN_A5E3_POS4_ADDR] |= BIT(5); // ��1
    else
      zxdown_buffer[ZXDOWN_A5E3_POS4_ADDR] &= ~BIT(5); // ��0
    
    if (pdata[0]&BIT(4)) // ��ǰ�ڱ���
      zxdown_buffer[ZXDOWN_A5E3_POS4_ADDR] |= BIT(6); // ��1
    else
      zxdown_buffer[ZXDOWN_A5E3_POS4_ADDR] &= ~BIT(6); // ��0
    
    if (pdata[0]&BIT(5)) // ��ǰ�ڱ���
      zxdown_buffer[ZXDOWN_A5E3_POS4_ADDR] |= BIT(7); // ��1
    else
      zxdown_buffer[ZXDOWN_A5E3_POS4_ADDR] &= ~BIT(7); // ��0
    
    if (pdata[0]&BIT(6)) // ��ǰ�ڱ���
      zxdown_buffer[ZXDOWN_A5E3_POS5_ADDR] |= BIT(0); // ��1
    else
      zxdown_buffer[ZXDOWN_A5E3_POS5_ADDR] &= ~BIT(0); // ��0
    
    if (pdata[0]&BIT(7)) // ��ǰ�ڱ���
      zxdown_buffer[ZXDOWN_A5E3_POS5_ADDR] |= BIT(1); // ��1
    else
      zxdown_buffer[ZXDOWN_A5E3_POS5_ADDR] &= ~BIT(1); // ��0
    
    tlv_a5e3_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5E4====================================================================================
  case 0x22D:
    memcpy(&zxdown_buffer[ZXDOWN_A5E4_POS1_ADDR], pdata, 8); // һ����ת��Ƕ�
    tlv_a5e4_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x22E:
    zxdown_buffer[ZXDOWN_A5E4_POS2_ADDR] = pdata[0];  // �������ת��Ƕ�
    zxdown_buffer[ZXDOWN_A5E4_POS2_ADDR+1] = pdata[1];
    zxdown_buffer[ZXDOWN_A5E4_POS2_ADDR+2] = pdata[2]; // �������ת��Ƕ�
    zxdown_buffer[ZXDOWN_A5E4_POS2_ADDR+3] = pdata[3];
    tlv_a5e4_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x25A:
    zxdown_buffer[ZXDOWN_A5E4_POS2_ADDR+4] = pdata[0]; // �������ת��Ƕ�
    zxdown_buffer[ZXDOWN_A5E4_POS2_ADDR+5] = pdata[1];
    zxdown_buffer[ZXDOWN_A5E4_POS2_ADDR+6] = pdata[2]; // �������ת��Ƕ�
    zxdown_buffer[ZXDOWN_A5E4_POS2_ADDR+7] = pdata[3];
    tlv_a5e4_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x2BB:
    memcpy(&zxdown_buffer[ZXDOWN_A5E4_POS3_ADDR], pdata, 8); // һ���ᴫ��������
    tlv_a5e4_valid_flag = 1;
    retval = 0x01;
    break;    

  case 0x2BC:
    memcpy(&zxdown_buffer[ZXDOWN_A5E4_POS4_ADDR], pdata, 8); // ���������ᴫ��������
    tlv_a5e4_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x25C:
    zxdown_buffer[ZXDOWN_A5E4_POS6_ADDR] = pdata[0];  // Ŀ��ת��ģʽ
    zxdown_buffer[ZXDOWN_A5E4_POS7_ADDR] = pdata[2];  // ת��ϵͳѹ��
    zxdown_buffer[ZXDOWN_A5E4_POS7_ADDR+1] = pdata[3];
    zxdown_buffer[ZXDOWN_A5E4_POS8_ADDR] = pdata[4];  // ������ѹ������
    zxdown_buffer[ZXDOWN_A5E4_POS8_ADDR+1] = pdata[5];
    zxdown_buffer[ZXDOWN_A5E9_POS2_ADDR] = pdata[6]; // Һѹϵͳѹ��
    zxdown_buffer[ZXDOWN_A5E9_POS2_ADDR+1] = pdata[7];
    tlv_a5e4_valid_flag = 1;
    tlv_a5e9_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x2BD:
    zxdown_buffer[ZXDOWN_A5E4_POS11_ADDR] = pdata[0];  // ����ֹ��
    memcpy(&zxdown_buffer[ZXDOWN_A5E4_POS9_ADDR], &pdata[2], 6); // 123������ת��ռ�ձ�
    tlv_a5e4_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x2BE:
    memcpy(&zxdown_buffer[ZXDOWN_A5E4_POS10_ADDR], pdata, 6); // 456������ת��ռ�ձ�
    tlv_a5e4_valid_flag = 1;
    retval = 0x01;
    break;
    
  //==A5E5====================================================================================
  case 0x1CF22D21:
    zxdown_buffer[ZXDOWN_A5E5_POS1_ADDR] = pdata[0];  // ��ǰһ��ת��
    zxdown_buffer[ZXDOWN_A5E5_POS1_ADDR+1] = pdata[1];
    tlv_a5e5_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x1CF28B21:
    if (pdata[1]&BIT(0)) // ת��ϵͳ����ָʾ��E101
      zxdown_buffer[ZXDOWN_A5E5_POS6_ADDR] |= BIT(0); // ��1
    else
      zxdown_buffer[ZXDOWN_A5E5_POS6_ADDR] &= ~BIT(0); // ��0

    if (pdata[1]&BIT(1)) // ��λָʾ��
      zxdown_buffer[ZXDOWN_A5E5_POS6_ADDR] |= BIT(1); // ��1
    else
      zxdown_buffer[ZXDOWN_A5E5_POS6_ADDR] &= ~BIT(1); // ��0
    
    zxdown_buffer[ZXDOWN_A5E5_POS2_ADDR] = pdata[4];  // ת������ѹ��(bar)
    zxdown_buffer[ZXDOWN_A5E5_POS2_ADDR+1] = pdata[5];
    zxdown_buffer[ZXDOWN_A5E5_POS4_ADDR] = pdata[2];  // ������λ
    zxdown_buffer[ZXDOWN_A5E5_POS5_ADDR] = pdata[0];  // ���ֽ�״̬λ1
    tlv_a5e5_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x1CF25C21:
    zxdown_buffer[ZXDOWN_A5E5_POS3_ADDR] = pdata[0];  // Ŀ��ת��ģʽ
    zxdown_buffer[ZXDOWN_A5E5_POS3_ADDR+1] = pdata[1];// ��ǰת��ģʽ
    tlv_a5e5_valid_flag = 1;
    retval = 0x01;
    break;
  
  //==A5E6====================================================================================
  case 0x25D:
    zxdown_buffer[ZXDOWN_A5E6_POS1_ADDR] = pdata[0];  // ��·һ��ѹ
    zxdown_buffer[ZXDOWN_A5E6_POS1_ADDR+1] = pdata[1];
    zxdown_buffer[ZXDOWN_A5E6_POS2_ADDR] = pdata[2];  // ��·һ��ѹ
    zxdown_buffer[ZXDOWN_A5E6_POS2_ADDR+1] = pdata[3];
    zxdown_buffer[ZXDOWN_A5E6_POS3_ADDR] = pdata[4];  // ��·һ��ѹ
    zxdown_buffer[ZXDOWN_A5E6_POS3_ADDR+1] = pdata[5];
    zxdown_buffer[ZXDOWN_A5E6_POS4_ADDR] = pdata[6];  // ��·һ��ѹ
    zxdown_buffer[ZXDOWN_A5E6_POS4_ADDR+1] = pdata[7];
    tlv_a5e6_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x23A:
    zxdown_buffer[ZXDOWN_A5E6_POS7_ADDR] = pdata[4];  // ���������
    zxdown_buffer[ZXDOWN_A5E6_POS7_ADDR+1] = pdata[5];
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
      zxdown_buffer[ZXDOWN_A5EA_POS1_ADDR] |= BIT(0); // ��1
    else
      zxdown_buffer[ZXDOWN_A5EA_POS1_ADDR] &= ~BIT(0); // ��0
    
    zxdown_buffer[ZXDOWN_A5EA_POS3_ADDR] = pdata[1]; // �ϳ�������ˮ��
    zxdown_buffer[ZXDOWN_A5EA_POS4_ADDR] = pdata[2]; // �ϳ�����������ѹ��
    zxdown_buffer[ZXDOWN_A5EA_POS4_ADDR+1] = pdata[3];
    zxdown_buffer[ZXDOWN_A5EA_POS5_ADDR] = pdata[4]; // �ϳ�������ת��
    zxdown_buffer[ZXDOWN_A5EA_POS5_ADDR+1] = pdata[5];
    zxdown_buffer[ZXDOWN_A5EA_POS6_ADDR] = pdata[6]; // �ϳ��ó���ѹ��
    zxdown_buffer[ZXDOWN_A5EA_POS6_ADDR+1] = pdata[7];
    tlv_a5ea_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5EB====================================================================================
  case 0x28C:
    if(pdata[0]>0 && pdata[0]<19) // 1����̥��18����̥
    {
      zxdown_buffer[ZXDOWN_A5EB_POS1_ADDR+pdata[0]-1] = pdata[1];
      tlv_a5eb_valid_flag = 1;
    }
    retval = 0x01;
    break;

  //==A5EC====================================================================================
  case 0x2AB:
    zxdown_buffer[ZXDOWN_A5EC_POS1_ADDR] = pdata[0]; // ��֧����� - �ֽ�״̬λ1
    zxdown_buffer[ZXDOWN_A5EC_POS2_ADDR] = pdata[1]; // ��֧����� - �ֽ�״̬λ2
    zxdown_buffer[ZXDOWN_A5EC_POS3_ADDR] = pdata[2]; // ��֧����� - �ֽ�״̬λ3
    zxdown_buffer[ZXDOWN_A5EC_POS4_ADDR] = pdata[3]; // ��֧����� - �ֽ�״̬λ4
    zxdown_buffer[ZXDOWN_A5EC_POS5_ADDR] = pdata[4]; // ��֧����� - �ֽ�״̬λ5
    zxdown_buffer[ZXDOWN_A5EC_POS6_ADDR] = pdata[5]; // ��֧����� - Ԥ��
    zxdown_buffer[ZXDOWN_A5EC_POS7_ADDR] = pdata[6]; // ��֧����� - DIR
    zxdown_buffer[ZXDOWN_A5EC_POS8_ADDR] = pdata[7]; // ��֧����� - ��ƽ������
    tlv_a5ec_valid_flag = 1;
    retval = 0x01;
    break;
    
  //==A5ED====================================================================================
  case 0x2AC:
    zxdown_buffer[ZXDOWN_A5ED_POS1_ADDR] = pdata[0]; // ��֧����� - �ֽ�״̬λ1
    zxdown_buffer[ZXDOWN_A5ED_POS2_ADDR] = pdata[1]; // ��֧����� - �ֽ�״̬λ2
    zxdown_buffer[ZXDOWN_A5ED_POS3_ADDR] = pdata[2]; // ��֧����� - �ֽ�״̬λ3
    zxdown_buffer[ZXDOWN_A5ED_POS4_ADDR] = pdata[3]; // ��֧����� - �ֽ�״̬λ4
    zxdown_buffer[ZXDOWN_A5ED_POS5_ADDR] = pdata[4]; // ��֧����� - �ֽ�״̬λ5
    zxdown_buffer[ZXDOWN_A5ED_POS6_ADDR] = pdata[5]; // ��֧����� - Ԥ��
    zxdown_buffer[ZXDOWN_A5ED_POS7_ADDR] = pdata[6]; // ��֧����� - DIR
    zxdown_buffer[ZXDOWN_A5ED_POS8_ADDR] = pdata[7]; // ��֧����� - ��ƽ������
    tlv_a5ed_valid_flag = 1;
    retval = 0x01;
    break;
    
  //==A5EE====================================================================================
  case 0x2AD:
    zxdown_buffer[ZXDOWN_A5EE_POS1_ADDR] = pdata[0]; // �п�̨������Ϣ - �ֽ�״̬λ1
    zxdown_buffer[ZXDOWN_A5EE_POS2_ADDR] = pdata[1]; // �п�̨������Ϣ - �ֽ�״̬λ2
    tlv_a5ee_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x2AE:
    zxdown_buffer[ZXDOWN_A5EE_POS3_ADDR] = pdata[0]; // �п�̨������Ϣ - �ֽ�״̬λ3
    zxdown_buffer[ZXDOWN_A5EE_POS4_ADDR] = pdata[1]; // �п�̨������Ϣ - �ֽ�״̬λ4
    tlv_a5ee_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x2AF:
    zxdown_buffer[ZXDOWN_A5EE_POS5_ADDR] = pdata[0]; // �п�̨������Ϣ - �ֽ�״̬λ5
    zxdown_buffer[ZXDOWN_A5EE_POS6_ADDR] = pdata[1]; // �п�̨������Ϣ - �ֽ�״̬λ6
    zxdown_buffer[ZXDOWN_A5EE_POS7_ADDR] = pdata[2]; // �п�̨������Ϣ - �ֽ�״̬λ7
    zxdown_buffer[ZXDOWN_A5EE_POS8_ADDR] = pdata[3]; // ����ת��Ƕ�
    zxdown_buffer[ZXDOWN_A5EE_POS9_ADDR] = pdata[4]; // �ֽ�״̬λ8
    tlv_a5ee_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x40D:
    // ����ѹ��
    // �����¶�
    break;

  //==�³�ϵͳ�汾====================================================================================
  case 0x20F:
    zxversion_buffer[ZXVERSION_A506_POS3_ADDR] = pdata[0];  // P1Ӧ�ò�
    zxversion_buffer[ZXVERSION_A506_POS3_ADDR+1] = pdata[1];
    zxversion_buffer[ZXVERSION_A506_POS3_ADDR+2] = pdata[2];
    zxversion_buffer[ZXVERSION_A506_POS5_ADDR] = pdata[3];  // P2Ӧ�ò�
    zxversion_buffer[ZXVERSION_A506_POS5_ADDR+1] = pdata[4];
    zxversion_buffer[ZXVERSION_A506_POS5_ADDR+2] = pdata[5];
    zxversion_buffer[ZXVERSION_A506_POS7_ADDR] = pdata[6];  // P3Ӧ�ò�
    zxversion_buffer[ZXVERSION_A506_POS7_ADDR+1] = pdata[7];
    tlv_a506_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x21F:
    zxversion_buffer[ZXVERSION_A506_POS7_ADDR+2] = pdata[0];  // P3Ӧ�ò�
    zxversion_buffer[ZXVERSION_A506_POS9_ADDR] = pdata[1];    // P4Ӧ�ò�
    zxversion_buffer[ZXVERSION_A506_POS9_ADDR+1] = pdata[2];
    zxversion_buffer[ZXVERSION_A506_POS9_ADDR+2] = pdata[3];
    zxversion_buffer[ZXVERSION_A506_POS11_ADDR] = pdata[4];   // P5Ӧ�ò�
    zxversion_buffer[ZXVERSION_A506_POS11_ADDR+1] = pdata[5];
    zxversion_buffer[ZXVERSION_A506_POS11_ADDR+2] = pdata[6];
    zxversion_buffer[ZXVERSION_A506_POS17_ADDR] = pdata[7];   // P8Ӧ�ò�
    tlv_a506_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x22F:
    zxversion_buffer[ZXVERSION_A506_POS17_ADDR+1] = pdata[0];   // P8Ӧ�ò�
    zxversion_buffer[ZXVERSION_A506_POS17_ADDR+2] = pdata[1];
    tlv_a506_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x29F:
    if(pdata[0]==1)
    {
      zxversion_buffer[ZXVERSION_A506_POS4_ADDR] = pdata[1];  // P1�ײ�
      zxversion_buffer[ZXVERSION_A506_POS4_ADDR+1] = pdata[2];
      zxversion_buffer[ZXVERSION_A506_POS4_ADDR+2] = pdata[3];
      tlv_a506_valid_flag = 1;
    }
    else if(pdata[0]==2)
    {
      zxversion_buffer[ZXVERSION_A506_POS6_ADDR] = pdata[1];  // P2�ײ�
      zxversion_buffer[ZXVERSION_A506_POS6_ADDR+1] = pdata[2];
      zxversion_buffer[ZXVERSION_A506_POS6_ADDR+2] = pdata[3];
      tlv_a506_valid_flag = 1;
    }
    else if(pdata[0]==3)
    {
      zxversion_buffer[ZXVERSION_A506_POS8_ADDR] = pdata[1];  // P3�ײ�
      zxversion_buffer[ZXVERSION_A506_POS8_ADDR+1] = pdata[2];
      zxversion_buffer[ZXVERSION_A506_POS8_ADDR+2] = pdata[3];
      tlv_a506_valid_flag = 1;
    }
    else if(pdata[0]==4)
    {
      zxversion_buffer[ZXVERSION_A506_POS10_ADDR] = pdata[1];  // P4�ײ�
      zxversion_buffer[ZXVERSION_A506_POS10_ADDR+1] = pdata[2];
      zxversion_buffer[ZXVERSION_A506_POS10_ADDR+2] = pdata[3];
      tlv_a506_valid_flag = 1;
    }
    else if(pdata[0]==5)
    {
      zxversion_buffer[ZXVERSION_A506_POS12_ADDR] = pdata[1];  // P5�ײ�
      zxversion_buffer[ZXVERSION_A506_POS12_ADDR+1] = pdata[2];
      zxversion_buffer[ZXVERSION_A506_POS12_ADDR+2] = pdata[3];
      tlv_a506_valid_flag = 1;
    }
    else if(pdata[0]==6)
    {
      zxversion_buffer[ZXVERSION_A506_POS14_ADDR] = pdata[1];  // P6�ײ�
      zxversion_buffer[ZXVERSION_A506_POS14_ADDR+1] = pdata[2];
      zxversion_buffer[ZXVERSION_A506_POS14_ADDR+2] = pdata[3];
      tlv_a506_valid_flag = 1;
    }
    else if(pdata[0]==7)
    {
      zxversion_buffer[ZXVERSION_A506_POS16_ADDR] = pdata[1];  // P7�ײ�
      zxversion_buffer[ZXVERSION_A506_POS16_ADDR+1] = pdata[2];
      zxversion_buffer[ZXVERSION_A506_POS16_ADDR+2] = pdata[3];
      tlv_a506_valid_flag = 1;
    }
    else if(pdata[0]==8)
    {
      zxversion_buffer[ZXVERSION_A506_POS18_ADDR] = pdata[1];  // P8�ײ�
      zxversion_buffer[ZXVERSION_A506_POS18_ADDR+1] = pdata[2];
      zxversion_buffer[ZXVERSION_A506_POS18_ADDR+2] = pdata[3];
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
 * ������յ���CAN����(���̷�����)
*************************************************************************/
uint8_t CAN_ProcessRecvEngineMsg(uint32_t canId, uint8_t *pdata, uint8_t size)
{
  uint8_t retval = 0x00;
  //uint32_t tempVal;

  switch (canId) // �ϳ�ͨ��
  {
  //==A5EF===================================================================================
  case 0x0CF00300:
    zxengine_buffer[ZXENGINE_A5EF_POS11_ADDR] = pdata[1]; // ����̤��ٷֱ�
    tlv_a5ef_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x0CF00400:
    if (pdata[0]&BIT(0)) // ������Ť��ģʽ1
      zxengine_buffer[ZXENGINE_A5EF_POS22_ADDR] |= BIT(0); // ��1
    else
      zxengine_buffer[ZXENGINE_A5EF_POS22_ADDR] &= ~BIT(0); // ��0
    
    if (pdata[0]&BIT(1)) // ������Ť��ģʽ2
      zxengine_buffer[ZXENGINE_A5EF_POS22_ADDR] |= BIT(1); // ��1
    else
      zxengine_buffer[ZXENGINE_A5EF_POS22_ADDR] &= ~BIT(1); // ��0
    
    if (pdata[0]&BIT(2)) // ������Ť��ģʽ3
      zxengine_buffer[ZXENGINE_A5EF_POS22_ADDR] |= BIT(2); // ��1
    else
      zxengine_buffer[ZXENGINE_A5EF_POS22_ADDR] &= ~BIT(2); // ��0
    
    if (pdata[0]&BIT(3)) // ������Ť��ģʽ4
      zxengine_buffer[ZXENGINE_A5EF_POS22_ADDR] |= BIT(3); // ��1
    else
      zxengine_buffer[ZXENGINE_A5EF_POS22_ADDR] &= ~BIT(3); // ��0

    zxengine_buffer[ZXENGINE_A5EF_POS1_ADDR] = pdata[3]; // ������ת��
    zxengine_buffer[ZXENGINE_A5EF_POS1_ADDR+1] = pdata[4];
    zxengine_buffer[ZXENGINE_A5EF_POS2_ADDR] = pdata[2]; // ʵ��Ť�ذٷֱ�
    tlv_a5ef_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x18FEEE00:
    zxengine_buffer[ZXENGINE_A5EF_POS6_ADDR] = pdata[0]; // ��ȴҺ�¶�
    zxengine_buffer[ZXENGINE_A5EF_POS7_ADDR] = pdata[2]; // �����¶�
    zxengine_buffer[ZXENGINE_A5EF_POS7_ADDR+1] = pdata[3];
    tlv_a5ef_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x18FEEF00:
    zxengine_buffer[ZXENGINE_A5EF_POS8_ADDR] = pdata[2]; // ����Һλ
    zxengine_buffer[ZXENGINE_A5EF_POS9_ADDR] = pdata[3]; // ����ѹ��
    tlv_a5ef_valid_flag = 1;
    retval = 0x01;
    break;
 
  case 0x18FEF600:
    zxengine_buffer[ZXENGINE_A5EF_POS4_ADDR] = pdata[1]; // �������ѹ��
    zxengine_buffer[ZXENGINE_A5EF_POS5_ADDR] = pdata[2]; // ��������¶�
    tlv_a5ef_valid_flag = 1;
    retval = 0x01;
    break;
   
  case 0x18FEE500:
    zxengine_buffer[ZXENGINE_A5EF_POS10_ADDR] = pdata[0]; // ������������ʱ��
    zxengine_buffer[ZXENGINE_A5EF_POS10_ADDR+1] = pdata[1];
    zxengine_buffer[ZXENGINE_A5EF_POS10_ADDR+2] = pdata[2];
    zxengine_buffer[ZXENGINE_A5EF_POS10_ADDR+3] = pdata[3];
    tlv_a5ef_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x18FEDF00:
    zxengine_buffer[ZXENGINE_A5EF_POS3_ADDR] = pdata[0]; // Ħ��Ť�ذٷֱ�
    tlv_a5ef_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x18FEF100:
    zxengine_buffer[ZXENGINE_A5EF_POS13_ADDR] = pdata[3];
    
    if (pdata[4]&BIT(0)) // Ѳ�����ÿ���1
      zxengine_buffer[ZXENGINE_A5EF_POS14_ADDR] |= BIT(0); // ��1
    else
      zxengine_buffer[ZXENGINE_A5EF_POS14_ADDR] &= ~BIT(0); // ��0
    
    if (pdata[4]&BIT(1)) // Ѳ�����ÿ���1
      zxengine_buffer[ZXENGINE_A5EF_POS14_ADDR] |= BIT(1); // ��1
    else
      zxengine_buffer[ZXENGINE_A5EF_POS14_ADDR] &= ~BIT(1); // ��0
    
    if (pdata[4]&BIT(2)) // Ѳ�����ٿ���1
      zxengine_buffer[ZXENGINE_A5EF_POS14_ADDR] |= BIT(2); // ��1
    else
      zxengine_buffer[ZXENGINE_A5EF_POS14_ADDR] &= ~BIT(2); // ��0
    
    if (pdata[4]&BIT(3)) // Ѳ�����ٿ���2
      zxengine_buffer[ZXENGINE_A5EF_POS14_ADDR] |= BIT(3); // ��1
    else
      zxengine_buffer[ZXENGINE_A5EF_POS14_ADDR] &= ~BIT(3); // ��0
    
    if (pdata[4]&BIT(6)) // Ѳ�����Ƽ��ٿ���1
      zxengine_buffer[ZXENGINE_A5EF_POS14_ADDR] |= BIT(4); // ��1
    else
      zxengine_buffer[ZXENGINE_A5EF_POS14_ADDR] &= ~BIT(4); // ��0
    
    if (pdata[4]&BIT(7)) // Ѳ�����Ƽ��ٿ���2
      zxengine_buffer[ZXENGINE_A5EF_POS14_ADDR] |= BIT(5); // ��1
    else
      zxengine_buffer[ZXENGINE_A5EF_POS14_ADDR] &= ~BIT(5); // ��0
    
    if (pdata[6]&BIT(5)) // Ѳ������״̬1
      zxengine_buffer[ZXENGINE_A5EF_POS23_ADDR] |= BIT(4); // ��1
    else
      zxengine_buffer[ZXENGINE_A5EF_POS23_ADDR] &= ~BIT(4); // ��0

    if (pdata[6]&BIT(6)) // Ѳ������״̬2
      zxengine_buffer[ZXENGINE_A5EF_POS23_ADDR] |= BIT(5); // ��1
    else
      zxengine_buffer[ZXENGINE_A5EF_POS23_ADDR] &= ~BIT(5); // ��0

    if (pdata[6]&BIT(7)) // Ѳ������״̬3
      zxengine_buffer[ZXENGINE_A5EF_POS23_ADDR] |= BIT(6); // ��1
    else
      zxengine_buffer[ZXENGINE_A5EF_POS23_ADDR] &= ~BIT(6); // ��0

    zxengine_buffer[ZXENGINE_A5EF_POS12_ADDR] = pdata[1]; // ����
    zxengine_buffer[ZXENGINE_A5EF_POS12_ADDR+1] = pdata[2];
    zxengine_buffer[ZXENGINE_A5EF_POS15_ADDR] = pdata[5]; // Ѳ���趨�ٶ�
    tlv_a5ef_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x18FEF200:
    zxengine_buffer[ZXENGINE_A5EF_POS16_ADDR] = pdata[0]; // ������ȼ��������
    zxengine_buffer[ZXENGINE_A5EF_POS16_ADDR+1] = pdata[1];
    zxengine_buffer[ZXENGINE_A5EF_POS17_ADDR] = pdata[4]; // ������ƽ��ȼ��������
    zxengine_buffer[ZXENGINE_A5EF_POS17_ADDR+1] = pdata[5];
    tlv_a5ef_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x18FEE900:
    zxengine_buffer[ZXENGINE_A5EF_POS18_ADDR] = pdata[4]; // ȼ�����ͺ���
    zxengine_buffer[ZXENGINE_A5EF_POS18_ADDR+1] = pdata[5];
    zxengine_buffer[ZXENGINE_A5EF_POS18_ADDR+2] = pdata[6];
    zxengine_buffer[ZXENGINE_A5EF_POS18_ADDR+3] = pdata[7];
    tlv_a5ef_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x18FEE000:
    zxengine_buffer[ZXENGINE_A5EF_POS19_ADDR] = pdata[4]; // ����ʻ���
    zxengine_buffer[ZXENGINE_A5EF_POS19_ADDR+1] = pdata[5];
    zxengine_buffer[ZXENGINE_A5EF_POS19_ADDR+2] = pdata[6];
    zxengine_buffer[ZXENGINE_A5EF_POS19_ADDR+3] = pdata[7];
    tlv_a5ef_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x18FEFC17:
    zxengine_buffer[ZXENGINE_A5EF_POS20_ADDR] = pdata[1]; // ȼ��Һλ1
    zxengine_buffer[ZXENGINE_A5EF_POS21_ADDR] = pdata[6]; // ȼ��Һλ2
    tlv_a5ef_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x18FD0700:
    if (pdata[0]&BIT(2)) // ��ʻԱ������DWL-1
      zxengine_buffer[ZXENGINE_A5EF_POS23_ADDR] |= BIT(0); // ��1
    else
      zxengine_buffer[ZXENGINE_A5EF_POS23_ADDR] &= ~BIT(0); // ��0
    
    if (pdata[0]&BIT(3)) // ��ʻԱ������DWL-2
      zxengine_buffer[ZXENGINE_A5EF_POS23_ADDR] |= BIT(1); // ��1
    else
      zxengine_buffer[ZXENGINE_A5EF_POS23_ADDR] &= ~BIT(1); // ��0
    
    tlv_a5ef_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x18FEFF00:
    if (pdata[0]&BIT(0)) // ������ˮָʾ��-1
      zxengine_buffer[ZXENGINE_A5EF_POS23_ADDR] |= BIT(2); // ��1
    else
      zxengine_buffer[ZXENGINE_A5EF_POS23_ADDR] &= ~BIT(2); // ��0

    if (pdata[0]&BIT(1)) // ������ˮָʾ��-2
      zxengine_buffer[ZXENGINE_A5EF_POS23_ADDR] |= BIT(3); // ��1
    else
      zxengine_buffer[ZXENGINE_A5EF_POS23_ADDR] &= ~BIT(3); // ��0

    tlv_a5ef_valid_flag = 1;
    retval = 0x01;
    break;
  
  //==A5F0===================================================================================
  //GPS���㣬���ݷ�����PTO�������³�����
  
  //==A5F1===================================================================================
  case 0x18FF203D:
    zxengine_buffer[ZXENGINE_A5F1_POS1_ADDR] = pdata[0]; // ��������״̬
    zxengine_buffer[ZXENGINE_A5F1_POS2_ADDR] = pdata[5]; // T15_DCU
    zxengine_buffer[ZXENGINE_A5F1_POS3_ADDR] = pdata[6]; // ���ر�ѹ��
    zxengine_buffer[ZXENGINE_A5F1_POS3_ADDR+1] = pdata[7];
    tlv_a5f1_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x18FE563D:
    zxengine_buffer[ZXENGINE_A5F1_POS4_ADDR] = pdata[0]; // ������Һλ
    zxengine_buffer[ZXENGINE_A5F1_POS5_ADDR] = pdata[1]; // �������¶�
    tlv_a5f1_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x0CF0233D:
    zxengine_buffer[ZXENGINE_A5F1_POS6_ADDR] = pdata[6]; // ����������
    zxengine_buffer[ZXENGINE_A5F1_POS6_ADDR+1] = pdata[7];
    tlv_a5f1_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x18F00E51:
    zxengine_buffer[ZXENGINE_A5F1_POS7_ADDR] = pdata[0]; // SCR����NOxŨ��
    zxengine_buffer[ZXENGINE_A5F1_POS7_ADDR+1] = pdata[1];
    tlv_a5f1_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x18F00F52:
    zxengine_buffer[ZXENGINE_A5F1_POS8_ADDR] = pdata[0]; // SCR����NOxŨ��
    zxengine_buffer[ZXENGINE_A5F1_POS8_ADDR+1] = pdata[1];
    tlv_a5f1_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x18FD3E3D:
    zxengine_buffer[ZXENGINE_A5F1_POS9_ADDR] = pdata[0]; // SCR���������¶�(T6�¶�)
    zxengine_buffer[ZXENGINE_A5F1_POS9_ADDR+1] = pdata[1];
    zxengine_buffer[ZXENGINE_A5F1_POS10_ADDR] = pdata[3]; // SCR���������¶�(T7�¶�)
    zxengine_buffer[ZXENGINE_A5F1_POS10_ADDR+1] = pdata[4];
    tlv_a5f1_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x18FD9BA3:
    if (pdata[3]&BIT(0)) // Ʒ���¶ȴ�����FMI (SPN 3519)1
      zxengine_buffer[ZXENGINE_A5F1_POS14_ADDR] |= BIT(0); // ��1
    else
      zxengine_buffer[ZXENGINE_A5F1_POS14_ADDR] &= ~BIT(0); // ��0

    if (pdata[3]&BIT(1)) // Ʒ���¶ȴ�����FMI (SPN 3519)2
      zxengine_buffer[ZXENGINE_A5F1_POS14_ADDR] |= BIT(1); // ��1
    else
      zxengine_buffer[ZXENGINE_A5F1_POS14_ADDR] &= ~BIT(1); // ��0

    if (pdata[3]&BIT(2)) // Ʒ���¶ȴ�����FMI (SPN 3519)3
      zxengine_buffer[ZXENGINE_A5F1_POS14_ADDR] |= BIT(2); // ��1
    else
      zxengine_buffer[ZXENGINE_A5F1_POS14_ADDR] &= ~BIT(2); // ��0

    if (pdata[3]&BIT(3)) // Ʒ���¶ȴ�����FMI (SPN 3519)4
      zxengine_buffer[ZXENGINE_A5F1_POS14_ADDR] |= BIT(3); // ��1
    else
      zxengine_buffer[ZXENGINE_A5F1_POS14_ADDR] &= ~BIT(3); // ��0

    if (pdata[3]&BIT(4)) // Ʒ���¶ȴ�����FMI (SPN 3519)5
      zxengine_buffer[ZXENGINE_A5F1_POS14_ADDR] |= BIT(4); // ��1
    else
      zxengine_buffer[ZXENGINE_A5F1_POS14_ADDR] &= ~BIT(4); // ��0

    if (pdata[4]&BIT(0)) // Ʒ�ʴ�����FMI (SPN3520)1
      zxengine_buffer[ZXENGINE_A5F1_POS15_ADDR] |= BIT(0); // ��1
    else
      zxengine_buffer[ZXENGINE_A5F1_POS15_ADDR] &= ~BIT(0); // ��0

    if (pdata[4]&BIT(1)) // Ʒ�ʴ�����FMI (SPN3520)2
      zxengine_buffer[ZXENGINE_A5F1_POS15_ADDR] |= BIT(1); // ��1
    else
      zxengine_buffer[ZXENGINE_A5F1_POS15_ADDR] &= ~BIT(1); // ��0

    if (pdata[4]&BIT(2)) // Ʒ�ʴ�����FMI (SPN3520)3
      zxengine_buffer[ZXENGINE_A5F1_POS15_ADDR] |= BIT(2); // ��1
    else
      zxengine_buffer[ZXENGINE_A5F1_POS15_ADDR] &= ~BIT(2); // ��0

    if (pdata[4]&BIT(3)) // Ʒ�ʴ�����FMI (SPN3520)4
      zxengine_buffer[ZXENGINE_A5F1_POS15_ADDR] |= BIT(3); // ��1
    else
      zxengine_buffer[ZXENGINE_A5F1_POS15_ADDR] &= ~BIT(3); // ��0

    if (pdata[4]&BIT(4)) // Ʒ�ʴ�����FMI (SPN3520)5
      zxengine_buffer[ZXENGINE_A5F1_POS15_ADDR] |= BIT(4); // ��1
    else
      zxengine_buffer[ZXENGINE_A5F1_POS15_ADDR] &= ~BIT(4); // ��0

    if (pdata[5]&BIT(0)) // �߻����Լ�����(SPN3521)1
      zxengine_buffer[ZXENGINE_A5F1_POS16_ADDR] |= BIT(0); // ��1
    else
      zxengine_buffer[ZXENGINE_A5F1_POS16_ADDR] &= ~BIT(0); // ��0

    if (pdata[5]&BIT(1)) // �߻����Լ�����(SPN3521)2
      zxengine_buffer[ZXENGINE_A5F1_POS16_ADDR] |= BIT(1); // ��1
    else
      zxengine_buffer[ZXENGINE_A5F1_POS16_ADDR] &= ~BIT(1); // ��0

    if (pdata[5]&BIT(2)) // �߻����Լ�����(SPN3521)3
      zxengine_buffer[ZXENGINE_A5F1_POS16_ADDR] |= BIT(2); // ��1
    else
      zxengine_buffer[ZXENGINE_A5F1_POS16_ADDR] &= ~BIT(2); // ��0

    if (pdata[5]&BIT(3)) // �߻����Լ�����(SPN3521)4
      zxengine_buffer[ZXENGINE_A5F1_POS16_ADDR] |= BIT(3); // ��1
    else
      zxengine_buffer[ZXENGINE_A5F1_POS16_ADDR] &= ~BIT(3); // ��0

    zxengine_buffer[ZXENGINE_A5F1_POS13_ADDR] = pdata[1]; // ����Ʒ�ʴ������¶�(SPN 3515)
    zxengine_buffer[ZXENGINE_A5F1_POS11_ADDR] = pdata[1]; // ����Ũ��(SPN 3516)
    tlv_a5f1_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x18FCBD3D:
    zxengine_buffer[ZXENGINE_A5F1_POS6_ADDR] = pdata[4]; // �ۼ�����������
    zxengine_buffer[ZXENGINE_A5F1_POS6_ADDR+1] = pdata[5];
    zxengine_buffer[ZXENGINE_A5F1_POS6_ADDR+2] = pdata[6];
    zxengine_buffer[ZXENGINE_A5F1_POS6_ADDR+3] = pdata[7];
    tlv_a5f1_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x18FE56A3:
    if (pdata[4]&BIT(0)) // ������Һλ������ʧЧģʽFMI-1
      zxengine_buffer[ZXENGINE_A5F1_POS17_ADDR] |= BIT(0); // ��1
    else
      zxengine_buffer[ZXENGINE_A5F1_POS17_ADDR] &= ~BIT(0); // ��0

    if (pdata[4]&BIT(1)) // ������Һλ������ʧЧģʽFMI-2
      zxengine_buffer[ZXENGINE_A5F1_POS17_ADDR] |= BIT(1); // ��1
    else
      zxengine_buffer[ZXENGINE_A5F1_POS17_ADDR] &= ~BIT(1); // ��0

    if (pdata[4]&BIT(2)) // ������Һλ������ʧЧģʽFMI-3
      zxengine_buffer[ZXENGINE_A5F1_POS17_ADDR] |= BIT(2); // ��1
    else
      zxengine_buffer[ZXENGINE_A5F1_POS17_ADDR] &= ~BIT(2); // ��0

    if (pdata[4]&BIT(3)) // ������Һλ������ʧЧģʽFMI-4
      zxengine_buffer[ZXENGINE_A5F1_POS17_ADDR] |= BIT(3); // ��1
    else
      zxengine_buffer[ZXENGINE_A5F1_POS17_ADDR] &= ~BIT(3); // ��0

    if (pdata[4]&BIT(4)) // ������Һλ������ʧЧģʽFMI-5
      zxengine_buffer[ZXENGINE_A5F1_POS17_ADDR] |= BIT(4); // ��1
    else
      zxengine_buffer[ZXENGINE_A5F1_POS17_ADDR] &= ~BIT(4); // ��0

    if (pdata[5]&BIT(0)) // �������¶ȴ�����ʧЧģʽFMI-1
      zxengine_buffer[ZXENGINE_A5F1_POS18_ADDR] |= BIT(0); // ��1
    else
      zxengine_buffer[ZXENGINE_A5F1_POS18_ADDR] &= ~BIT(0); // ��0

    if (pdata[5]&BIT(1)) // �������¶ȴ�����ʧЧģʽFMI-2
      zxengine_buffer[ZXENGINE_A5F1_POS18_ADDR] |= BIT(1); // ��1
    else
      zxengine_buffer[ZXENGINE_A5F1_POS18_ADDR] &= ~BIT(1); // ��0

    if (pdata[5]&BIT(2)) // �������¶ȴ�����ʧЧģʽFMI-3
      zxengine_buffer[ZXENGINE_A5F1_POS18_ADDR] |= BIT(2); // ��1
    else
      zxengine_buffer[ZXENGINE_A5F1_POS18_ADDR] &= ~BIT(2); // ��0

    if (pdata[5]&BIT(3)) // �������¶ȴ�����ʧЧģʽFMI-4
      zxengine_buffer[ZXENGINE_A5F1_POS18_ADDR] |= BIT(3); // ��1
    else
      zxengine_buffer[ZXENGINE_A5F1_POS18_ADDR] &= ~BIT(3); // ��0

    if (pdata[5]&BIT(4)) // �������¶ȴ�����ʧЧģʽFMI-5
      zxengine_buffer[ZXENGINE_A5F1_POS18_ADDR] |= BIT(4); // ��1
    else
      zxengine_buffer[ZXENGINE_A5F1_POS18_ADDR] &= ~BIT(4); // ��0
    
    tlv_a5f1_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x18FEDF3D:
    if (pdata[7]&BIT(0)) // Nox������¶��״̬1
      zxengine_buffer[ZXENGINE_A5F1_POS19_ADDR] |= BIT(0); // ��1
    else
      zxengine_buffer[ZXENGINE_A5F1_POS19_ADDR] &= ~BIT(0); // ��0

    if (pdata[7]&BIT(1)) // Nox������¶��״̬2
      zxengine_buffer[ZXENGINE_A5F1_POS19_ADDR] |= BIT(1); // ��1
    else
      zxengine_buffer[ZXENGINE_A5F1_POS19_ADDR] &= ~BIT(1); // ��0

    tlv_a5f1_valid_flag = 1;
    retval = 0x01;
    break;
  
  //==A5F2===================================================================================
  case 0x18FD2000:
    zxengine_buffer[ZXENGINE_A5F2_POS1_ADDR] = pdata[0]; // DOC���������¶�
    zxengine_buffer[ZXENGINE_A5F2_POS1_ADDR+1] = pdata[1];
    zxengine_buffer[ZXENGINE_A5F2_POS2_ADDR] = pdata[2]; // DPF���������¶�
    zxengine_buffer[ZXENGINE_A5F2_POS2_ADDR+1] = pdata[3];
    tlv_a5f2_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x18FF1400:
    if (pdata[1]&BIT(0)) // DPF��������״̬1
      zxengine_buffer[ZXENGINE_A5F2_POS5_ADDR] |= BIT(0); // ��1
    else
      zxengine_buffer[ZXENGINE_A5F2_POS5_ADDR] &= ~BIT(0); // ��0

    if (pdata[1]&BIT(1)) // DPF��������״̬2
      zxengine_buffer[ZXENGINE_A5F2_POS5_ADDR] |= BIT(1); // ��1
    else
      zxengine_buffer[ZXENGINE_A5F2_POS5_ADDR] &= ~BIT(1); // ��0

    zxengine_buffer[ZXENGINE_A5F2_POS3_ADDR] = pdata[0]; // DPF̼����������
    tlv_a5f2_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x18FDB200:
    zxengine_buffer[ZXENGINE_A5F2_POS4_ADDR] = pdata[0]; // DPFѹ��
    zxengine_buffer[ZXENGINE_A5F2_POS4_ADDR+1] = pdata[1];
    tlv_a5f2_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x18FD7C00:
    if (pdata[0]&BIT(0)) // DPF����ָʾ��״̬1
      zxengine_buffer[ZXENGINE_A5F2_POS6_ADDR] |= BIT(0); // ��1
    else
      zxengine_buffer[ZXENGINE_A5F2_POS6_ADDR] &= ~BIT(0); // ��0

    if (pdata[0]&BIT(1)) // DPF����ָʾ��״̬2
      zxengine_buffer[ZXENGINE_A5F2_POS6_ADDR] |= BIT(1); // ��1
    else
      zxengine_buffer[ZXENGINE_A5F2_POS6_ADDR] &= ~BIT(1); // ��0

    if (pdata[0]&BIT(2)) // DPF����ָʾ��״̬3
      zxengine_buffer[ZXENGINE_A5F2_POS6_ADDR] |= BIT(2); // ��1
    else
      zxengine_buffer[ZXENGINE_A5F2_POS6_ADDR] &= ~BIT(2); // ��0

    if (pdata[2]&BIT(0)) // DPF������ֹ״̬1
      zxengine_buffer[ZXENGINE_A5F2_POS7_ADDR] |= BIT(0); // ��1
    else
      zxengine_buffer[ZXENGINE_A5F2_POS7_ADDR] &= ~BIT(0); // ��0

    if (pdata[2]&BIT(1)) // DPF������ֹ״̬2
      zxengine_buffer[ZXENGINE_A5F2_POS7_ADDR] |= BIT(1); // ��1
    else
      zxengine_buffer[ZXENGINE_A5F2_POS7_ADDR] &= ~BIT(1); // ��0
      
    tlv_a5f2_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x18E00021:
    if (pdata[5]&BIT(0)) // DPF��������״̬1
      zxengine_buffer[ZXENGINE_A5F2_POS8_ADDR] |= BIT(0); // ��1
    else
      zxengine_buffer[ZXENGINE_A5F2_POS8_ADDR] &= ~BIT(0); // ��0

    if (pdata[5]&BIT(1)) // DPF��������״̬2
      zxengine_buffer[ZXENGINE_A5F2_POS8_ADDR] |= BIT(1); // ��1
    else
      zxengine_buffer[ZXENGINE_A5F2_POS8_ADDR] &= ~BIT(1); // ��0

    if (pdata[5]&BIT(2)) // DPF������ֹ����״̬1
      zxengine_buffer[ZXENGINE_A5F2_POS8_ADDR] |= BIT(2); // ��1
    else
      zxengine_buffer[ZXENGINE_A5F2_POS8_ADDR] &= ~BIT(2); // ��0

    if (pdata[5]&BIT(3)) // DPF������ֹ����״̬2
      zxengine_buffer[ZXENGINE_A5F2_POS8_ADDR] |= BIT(3); // ��1
    else
      zxengine_buffer[ZXENGINE_A5F2_POS8_ADDR] &= ~BIT(3); // ��0

    tlv_a5f2_valid_flag = 1;
    retval = 0x01;
    break;
  
  default:
    retval = 0x00;
    break;
  }
  
  return retval;
}


