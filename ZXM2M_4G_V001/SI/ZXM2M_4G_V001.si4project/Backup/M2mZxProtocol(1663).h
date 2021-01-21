/*****************************************************************************
* @FileName: M2mZxProtocol.h
* @Engineer: TenYan
* @Company:  �칤��Ϣ����Ӳ����
* @version   V1.0
* @Date:     2020-12-18
* @brief     �칤�������ػ�canͨ��Э�鶨��
******************************************************************************/
#ifndef _ZX_M2M_PROTOCOL_H_
#define _ZX_M2M_PROTOCOL_H_

#include  "types.h"
#include  "M2mProtocol.h"

/******************************************************************************
* Macros(����������Ϣ): ��ź͵�ַ����
******************************************************************************/
//==TAG-A501ͨ��״̬��2(����ר��)===============================================
#define ZXINFO_A501_ADDR         0  // ��ʼ��ַ
#define zxinfo_buffer_a501       (zxinfo_buffer+ZXINFO_A501_ADDR)  // ��ʼ��ַ����
#define ZXINFO_A501_POS1_ADDR    0                          // ����״̬λ
#define ZXINFO_A501_POS2_ADDR    (ZXINFO_A501_POS1_ADDR+1)  // ����״̬λ
#define ZXINFO_A501_POS3_ADDR    (ZXINFO_A501_POS2_ADDR+1)  // �󶨺�����״̬λ
#define ZXINFO_A501_POS4_ADDR    (ZXINFO_A501_POS3_ADDR+1)  // ����ԭ��ͻ�ȡ��Ϣ״̬λ
#define SIZE_OF_ZXINFO_A501      (ZXINFO_A501_POS4_ADDR+1)  // ���ֽ���(4B)

//==TAG-A504�ɼ�Э����Ϣ=========================================================
#define ZXINFO_A504_ADDR         (ZXINFO_A501_ADDR+SIZE_OF_ZXINFO_A501)  // ��ʼ��ַ
#define zxinfo_buffer_a504       (zxinfo_buffer+ZXINFO_A504_ADDR)  // ��ʼ��ַ����
#define ZXINFO_A504_POS1_ADDR    0                          // ��������
#define ZXINFO_A504_POS2_ADDR    (ZXINFO_A504_POS1_ADDR+1)  // �ϳ�CANЭ��
#define ZXINFO_A504_POS3_ADDR    (ZXINFO_A504_POS2_ADDR+1)  // �³�CANЭ��
#define SIZE_OF_ZXINFO_A504      (ZXINFO_A504_POS3_ADDR+1)  // ���ֽ���(3B)

//==TAG-A5FF�ն�״̬��Ϣ==========================================================
#define ZXINFO_A5FF_ADDR         (ZXINFO_A504_ADDR+SIZE_OF_ZXINFO_A504)  // ��ʼ��ַ
#define zxinfo_buffer_a5ff       (zxinfo_buffer+ZXINFO_A5FF_ADDR)  // ��ʼ��ַ����
#define ZXINFO_A5FF_POS1_ADDR    0                          // �ⲿ��Դ��ѹ
#define ZXINFO_A5FF_POS2_ADDR    (ZXINFO_A5FF_POS1_ADDR+2)  // ���õ�ص�ѹ
#define ZXINFO_A5FF_POS3_ADDR    (ZXINFO_A5FF_POS2_ADDR+2)  // �������ɼ�1
#define ZXINFO_A5FF_POS4_ADDR    (ZXINFO_A5FF_POS3_ADDR+1)  // �������ɼ�2
#define ZXINFO_A5FF_POS5_ADDR    (ZXINFO_A5FF_POS4_ADDR+1)  // �������ɼ�3
#define ZXINFO_A5FF_POS6_ADDR    (ZXINFO_A5FF_POS5_ADDR+1)  // �����࿪����
#define ZXINFO_A5FF_POS7_ADDR    (ZXINFO_A5FF_POS6_ADDR+1)  // CANͨ��״̬
#define ZXINFO_A5FF_POS8_ADDR    (ZXINFO_A5FF_POS7_ADDR+1)  // RTCʱ��
#define ZXINFO_A5FF_POS9_ADDR    (ZXINFO_A5FF_POS8_ADDR+6)  // ����ģʽ
#define ZXINFO_A5FF_POS10_ADDR   (ZXINFO_A5FF_POS9_ADDR+1)  // ST�̼��汾��Ϣ
#define SIZE_OF_ZXINFO_A5FF      (ZXINFO_A5FF_POS10_ADDR+2)  // ���ֽ���(11B)


// ����������Ϣ���ݻ����С
#define SIZE_OF_ZXINFO_BUFFER  (ZXINFO_A5FF_ADDR+SIZE_OF_ZXINFO_A5FF)

/******************************************************************************
* Macros(�ϳ�ͨ��): ���ػ��ϳ�������ź͵�ַ����
******************************************************************************/
//==TAG-A5A0������������Ϣ(����)==================================================
#define ZXUP_A5A0_ADDR    0  // ��ʼ��ַ
#define zxup_buffer_a5a0  (zxup_buffer+ZXUP_A5A0_ADDR)  // ��ʼ��ַ����
#define ZXUP_A5A0_POS1    0  // ����������1-2
#define ZXUP_A5A0_POS2    (ZXUP_A5A0_POS1+2)  // ��������(ABCDEFGH)
#define ZXUP_A5A0_POS3    (ZXUP_A5A0_POS2+4)  // ����
#define ZXUP_A5A0_POS4    (ZXUP_A5A0_POS3+1)  // ����ͷ���Ƕ�
#define ZXUP_A5A0_POS5    (ZXUP_A5A0_POS4+2)  // ���۸����Ƕ�
#define ZXUP_A5A0_POS6    (ZXUP_A5A0_POS5+2)  // ���۳���
#define ZXUP_A5A0_POS7    (ZXUP_A5A0_POS6+2)  // ���ڱ۳���
#define ZXUP_A5A0_POS8    (ZXUP_A5A0_POS7+1)  // �ڱ۰ٷֱ�
#define ZXUP_A5A0_POS9    (ZXUP_A5A0_POS8+8)  // �ױ�������
#define ZXUP_A5A0_POS10   (ZXUP_A5A0_POS9+1)  // ���۳���
#define ZXUP_A5A0_POS11   (ZXUP_A5A0_POS10+2)  // ���۽Ƕ�
#define ZXUP_A5A0_POS12   (ZXUP_A5A0_POS11+2)  // ����/���۸����Ƕ�
#define ZXUP_A5A0_POS13   (ZXUP_A5A0_POS12+2)  // ����/����ͷ���Ƕ�
#define ZXUP_A5A0_POS14   (ZXUP_A5A0_POS13+2)  // ǻѹ��
#define ZXUP_A5A0_POS15   (ZXUP_A5A0_POS14+8)  // ��ͷ�߶�
#define ZXUP_A5A0_POS16   (ZXUP_A5A0_POS15+2)  // �����
#define ZXUP_A5A0_POS17   (ZXUP_A5A0_POS16+2)  // ʵ������
#define ZXUP_A5A0_POS18   (ZXUP_A5A0_POS17+2)  // ���ذٷֱ�
#define ZXUP_A5A0_POS19   (ZXUP_A5A0_POS18+2)  // ��������
#define ZXUP_A5A0_POS20   (ZXUP_A5A0_POS19+2)  // LMI���ϴ���1-4
#define ZXUP_A5A0_POS21   (ZXUP_A5A0_POS20+8)  // LMI���ϴ���5-8
#define ZXUP_A5A0_POS22   (ZXUP_A5A0_POS21+8)  // �ǶԳƹ��ϴ���
#define ZXUP_A5A0_POS23   (ZXUP_A5A0_POS22+4)  // LMI����ʱ��
#define ZXUP_A5A0_POS24   (ZXUP_A5A0_POS23+4)  // ǿ��λ
#define ZXUP_A5A0_POS25   (ZXUP_A5A0_POS24+1)  // ˮƽ��X
#define ZXUP_A5A0_POS26   (ZXUP_A5A0_POS25+2)  // ˮƽ��Y
#define ZXUP_A5A0_POS27   (ZXUP_A5A0_POS26+2)  // ����
#define ZXUP_A5A0_POS28   (ZXUP_A5A0_POS27+2)  // ��ת�Ƕ�
#define SIZE_OF_ZXUP_A5A0 (ZXUP_A5A0_POS28+2) // ���ֽ���(82B)

//==TAG-A5A1���𹤿���Ϣ==========================================================
#define ZXUP_A5A1_ADDR    (ZXUP_A5A0_ADDR+SIZE_OF_ZXUP_A5A0)  // ��ʼ��ַ
#define zxup_buffer_a5a1  (zxup_buffer+ZXUP_A5A1_ADDR) // ��ʼ��ַ����
#define ZXUP_A5A1_POS1    0                   // �������Ƕ�
#define ZXUP_A5A1_POS2    (ZXUP_A5A1_POS1+2)  // ��������
#define ZXUP_A5A1_POS3    (ZXUP_A5A1_POS2+2)  // ����չ���Ƕ�
#define ZXUP_A5A1_POS4    (ZXUP_A5A1_POS3+2)  // �������Ƕ�
#define ZXUP_A5A1_POS5    (ZXUP_A5A1_POS4+2)  // �ҳ������Ƕ�
#define ZXUP_A5A1_POS6    (ZXUP_A5A1_POS5+2)  // �ҳ�������
#define ZXUP_A5A1_POS7    (ZXUP_A5A1_POS6+2)  // �ҳ���չ���Ƕ�
#define ZXUP_A5A1_POS8    (ZXUP_A5A1_POS7+2)  // �ҳ������Ƕ�
#define ZXUP_A5A1_POS9    (ZXUP_A5A1_POS8+2)  // ����-��ֹ�ͽ���
#define ZXUP_A5A1_POS10   (ZXUP_A5A1_POS9+1)  // �ҳ���-��ֹ�ͽ���
#define ZXUP_A5A1_POS11   (ZXUP_A5A1_POS10+1)  // �������
#define ZXUP_A5A1_POS12   (ZXUP_A5A1_POS11+1)  // �ҳ������
#define ZXUP_A5A1_POS13   (ZXUP_A5A1_POS12+1)  // �����Ž��׳���
#define ZXUP_A5A1_POS14   (ZXUP_A5A1_POS13+2)  // �ҳ����Ž��׳���
#define SIZE_OF_ZXUP_A5A1 (ZXUP_A5A1_POS14+2)  // ���ֽ���(24B)

//==TAG-A5A2���۹�����Ϣ==========================================================
#define ZXUP_A5A2_ADDR    (ZXUP_A5A1_ADDR+SIZE_OF_ZXUP_A5A1)  // ��ʼ��ַ
#define zxup_buffer_a5a2  (zxup_buffer+ZXUP_A5A2_ADDR)// ��ʼ��ַ����
#define ZXUP_A5A2_POS1    0                   // ����������
#define ZXUP_A5A2_POS2    (ZXUP_A5A2_POS1+2)  // ����������
#define ZXUP_A5A2_POS3    (ZXUP_A5A2_POS2+2)  // ������ѹ��
#define ZXUP_A5A2_POS4    (ZXUP_A5A2_POS3+2)  // ǰ֧�ܽǶ�
#define ZXUP_A5A2_POS5    (ZXUP_A5A2_POS4+2)  // ����֧�ܼ��
#define SIZE_OF_ZXUP_A5A2 (ZXUP_A5A2_POS5+1)  // ���ֽ���(9B)

//==TAG-A5A5�ϳ���������Ϣ========================================================
#define ZXUP_A5A5_ADDR    (ZXUP_A5A2_ADDR+SIZE_OF_ZXUP_A5A2)  // ��ʼ��ַ
#define zxup_buffer_a5a5  (zxup_buffer+ZXUP_A5A5_ADDR)// ��ʼ��ַ����
#define ZXUP_A5A5_POS1    0                   // ������ת��
#define ZXUP_A5A5_POS2    (ZXUP_A5A5_POS1+2)  // ʵ��Ť�ذٷֱ�
#define ZXUP_A5A5_POS3    (ZXUP_A5A5_POS2+1)  // Ħ��Ť�ذٷֱ�
#define ZXUP_A5A5_POS4    (ZXUP_A5A5_POS3+1)  // �������ѹ��
#define ZXUP_A5A5_POS5    (ZXUP_A5A5_POS4+1)  // ��������¶�
#define ZXUP_A5A5_POS6    (ZXUP_A5A5_POS5+1)  // ��ȴҺ�¶�
#define ZXUP_A5A5_POS7    (ZXUP_A5A5_POS6+1)  // �����¶�
#define ZXUP_A5A5_POS8    (ZXUP_A5A5_POS7+2)  // ����Һλ
#define ZXUP_A5A5_POS9    (ZXUP_A5A5_POS8+1)  // ����ѹ��
#define ZXUP_A5A5_POS10   (ZXUP_A5A5_POS9+1)  // ������������ʱ��
#define ZXUP_A5A5_POS11   (ZXUP_A5A5_POS10+4)  // ������ˮָʾ��
#define ZXUP_A5A5_POS12   (ZXUP_A5A5_POS11+1)  // ����̤��ٷֱ�
#define ZXUP_A5A5_POS13   (ZXUP_A5A5_POS12+1)  // ������ȼ��������
#define ZXUP_A5A5_POS14   (ZXUP_A5A5_POS13+2)  // ������ƽ��ȼ��������
#define ZXUP_A5A5_POS15   (ZXUP_A5A5_POS14+2)  // ȼ��Һλ
#define ZXUP_A5A5_POS16   (ZXUP_A5A5_POS15+1)  // ������Ť��ģʽ(����������)
#define SIZE_OF_ZXUP_A5A5 (ZXUP_A5A5_POS16+1)  // ���ֽ���(23B)

//==TAG-A5A6�ֱ���Ϣ==============================================================
#define ZXUP_A5A6_ADDR    (ZXUP_A5A5_ADDR+SIZE_OF_ZXUP_A5A5)  // ��ʼ��ַ
#define zxup_buffer_a5a6  (zxup_buffer+ZXUP_A5A6_ADDR)// ��ʼ��ַ����
#define ZXUP_A5A6_POS1    0                   // ���ֱ�״̬λ
#define ZXUP_A5A6_POS2    (ZXUP_A5A6_POS1+1)  // ���ֱ�X���
#define ZXUP_A5A6_POS3    (ZXUP_A5A6_POS2+2)  // ���ֱ�Y���
#define ZXUP_A5A6_POS4    (ZXUP_A5A6_POS3+2)  // ���ֱ�״̬λ
#define ZXUP_A5A6_POS5    (ZXUP_A5A6_POS4+1)  // ���ֱ�X���
#define ZXUP_A5A6_POS6    (ZXUP_A5A6_POS5+2)  // ���ֱ�Y���
#define SIZE_OF_ZXUP_A5A6 (ZXUP_A5A6_POS6+2)  // ���ֽ���(10B)

//==TAG-A5A7��ʾ��1��Ϣ===========================================================
#define ZXUP_A5A7_ADDR    (ZXUP_A5A6_ADDR+SIZE_OF_ZXUP_A5A6)  // ��ʼ��ַ
#define zxup_buffer_a5a7  (zxup_buffer+ZXUP_A5A7_ADDR)// ��ʼ��ַ����
#define ZXUP_A5A7_POS1    0                   // ��Ʒ����
#define ZXUP_A5A7_POS2    (ZXUP_A5A7_POS1+4)  // ��������
#define ZXUP_A5A7_POS3    (ZXUP_A5A7_POS2+4)  // �������ٶ�
#define ZXUP_A5A7_POS4    (ZXUP_A5A7_POS3+1)  // �������ٶ�
#define ZXUP_A5A7_POS5    (ZXUP_A5A7_POS4+1)  // �������ٶ�
#define ZXUP_A5A7_POS6    (ZXUP_A5A7_POS5+1)  // �������ٶ�
#define ZXUP_A5A7_POS7    (ZXUP_A5A7_POS6+1)  // ������ٶ�
#define ZXUP_A5A7_POS8    (ZXUP_A5A7_POS7+1)  // ������ٶ�
#define ZXUP_A5A7_POS9    (ZXUP_A5A7_POS8+1)  // ���ת�ٶ�
#define ZXUP_A5A7_POS10   (ZXUP_A5A7_POS9+1)  // �һ�ת�ٶ�
#define ZXUP_A5A7_POS11   (ZXUP_A5A7_POS10+1)  // Ŀ�����
#define ZXUP_A5A7_POS12   (ZXUP_A5A7_POS11+1)  // ״̬λ1
#define ZXUP_A5A7_POS13   (ZXUP_A5A7_POS12+1)  // ״̬λ2
#define ZXUP_A5A7_POS14   (ZXUP_A5A7_POS13+1)  // ����ģʽ
#define SIZE_OF_ZXUP_A5A7 (ZXUP_A5A7_POS14+1)  // ���ֽ���(20B)

//==TAG-A5A8��ʾ��2��Ϣ===========================================================
#define ZXUP_A5A8_ADDR    (ZXUP_A5A7_ADDR+SIZE_OF_ZXUP_A5A7)  // ��ʼ��ַ
#define zxup_buffer_a5a8  (zxup_buffer+ZXUP_A5A8_ADDR)  // ��ʼ��ַ����
#define ZXUP_A5A8_POS1    0                   // ����ٿ�
#define ZXUP_A5A8_POS2    (ZXUP_A5A8_POS1+8)  // ����ά��
#define SIZE_OF_ZXUP_A5A8 (ZXUP_A5A8_POS2+2)  // ���ֽ���(10B)

//==TAG-A5A9���������Ϣ==========================================================
#define ZXUP_A5A9_ADDR    (ZXUP_A5A8_ADDR+SIZE_OF_ZXUP_A5A8)  // ��ʼ��ַ
#define zxup_buffer_a5a9  (zxup_buffer+ZXUP_A5A9_ADDR)// ��ʼ��ַ����
#define ZXUP_A5A9_POS1    0                   // ���1
#define ZXUP_A5A9_POS2    (ZXUP_A5A9_POS1+2)  // ���2
#define ZXUP_A5A9_POS3    (ZXUP_A5A9_POS2+2)  // ���3
#define SIZE_OF_ZXUP_A5A9 (ZXUP_A5A9_POS3+2)  // ���ֽ���(6B)

//==TAG-A5AA���߲ٿ���Ϣ==========================================================
#define ZXUP_A5AA_ADDR    (ZXUP_A5A9_ADDR+SIZE_OF_ZXUP_A5A9)  // ��ʼ��ַ
#define zxup_buffer_a5aa  (zxup_buffer+ZXUP_A5AA_ADDR)// ��ʼ��ַ����
#define ZXUP_A5AA_POS1    0                   // Msg1
#define ZXUP_A5AA_POS2    (ZXUP_A5AA_POS1+8)  // Msg2
#define ZXUP_A5AA_POS3    (ZXUP_A5AA_POS2+8)  // Msg3
#define ZXUP_A5AA_POS4    (ZXUP_A5AA_POS3+8)  // Msg4
#define SIZE_OF_ZXUP_A5AA (ZXUP_A5AA_POS4+8)  // ���ֽ���(32B)

//==TAG-A5AB����������Ϣ==========================================================
#define ZXUP_A5AB_ADDR    (ZXUP_A5AA_ADDR+SIZE_OF_ZXUP_A5AA)  // ��ʼ��ַ
#define zxup_buffer_a5ab  (zxup_buffer+ZXUP_A5AB_ADDR)// ��ʼ��ַ����
#define ZXUP_A5AB_POS1    0                   // �ڵ�״̬
#define SIZE_OF_ZXUP_A5AB (ZXUP_A5AB_POS1+8)  // ���ֽ���(8B)

//==TAG-A5AC�����߼���Ϣ==========================================================
#define ZXUP_A5AC_ADDR    (ZXUP_A5AB_ADDR+SIZE_OF_ZXUP_A5AB)  // ��ʼ��ַ
#define zxup_buffer_a5ac  (zxup_buffer+ZXUP_A5AC_ADDR)// ��ʼ��ַ����
#define ZXUP_A5AC_POS1    0                   // �������ƺͽ��
#define SIZE_OF_ZXUP_A5AC (ZXUP_A5AC_POS1+6)  // ���ֽ���(6B)

//==TAG-A5AD����߼���Ϣ==========================================================
#define ZXUP_A5AD_ADDR    (ZXUP_A5AC_ADDR+SIZE_OF_ZXUP_A5AC)  // ��ʼ��ַ
#define zxup_buffer_a5ad  (zxup_buffer+ZXUP_A5AD_ADDR)// ��ʼ��ַ����
#define ZXUP_A5AD_POS1    0                   // �������ƺͽ��
#define SIZE_OF_ZXUP_A5AD (ZXUP_A5AD_POS1+8)  // ���ֽ���(8B)

//==TAG-A5AE��ת�߼���Ϣ==========================================================
#define ZXUP_A5AE_ADDR    (ZXUP_A5AD_ADDR+SIZE_OF_ZXUP_A5AD)  // ��ʼ��ַ
#define zxup_buffer_a5ae  (zxup_buffer+ZXUP_A5AE_ADDR)// ��ʼ��ַ����
#define ZXUP_A5AE_POS1    0                   // �������1
#define ZXUP_A5AE_POS2    (ZXUP_A5AE_POS1+1)  // �������2
#define ZXUP_A5AE_POS3    (ZXUP_A5AE_POS2+1)  // ��ؽ��
#define ZXUP_A5AE_POS4    (ZXUP_A5AE_POS3+1)  // �һ�����1
#define ZXUP_A5AE_POS5    (ZXUP_A5AE_POS4+1)  // �һ�����2
#define ZXUP_A5AE_POS6    (ZXUP_A5AE_POS5+1)  // �һؽ��
#define SIZE_OF_ZXUP_A5AE (ZXUP_A5AE_POS6+1)  // ���ֽ���(6B)

//==TAG-A5AF�������߼���Ϣ========================================================
#define ZXUP_A5AF_ADDR    (ZXUP_A5AE_ADDR+SIZE_OF_ZXUP_A5AE)  // ��ʼ��ַ
#define zxup_buffer_a5af  (zxup_buffer+ZXUP_A5AF_ADDR)// ��ʼ��ַ����
#define ZXUP_A5AF_POS1    0                   // �������������ƺͽ��
#define SIZE_OF_ZXUP_A5AF (ZXUP_A5AF_POS1+5)  // ���ֽ���(5B)

//==TAG-A5B0�������߼���Ϣ========================================================
#define ZXUP_A5B0_ADDR    (ZXUP_A5AF_ADDR+SIZE_OF_ZXUP_A5AF)  // ��ʼ��ַ
#define zxup_buffer_a5b0  (zxup_buffer+ZXUP_A5B0_ADDR)// ��ʼ��ַ����
#define ZXUP_A5B0_POS1    0                   // �������������ƺͽ��
#define SIZE_OF_ZXUP_A5B0 (ZXUP_A5B0_POS1+4)  // ���ֽ���(4B)

//==TAG-A5B1�����߼���Ϣ==========================================================
#define ZXUP_A5B1_ADDR    (ZXUP_A5B0_ADDR+SIZE_OF_ZXUP_A5B0)  // ��ʼ��ַ
#define zxup_buffer_a5b1  (zxup_buffer+ZXUP_A5B1_ADDR)// ��ʼ��ַ����
#define ZXUP_A5B1_POS1    0                   // ����Ŀ��չ���Ƕ�
#define ZXUP_A5B1_POS2    (ZXUP_A5B1_POS1+1)  // ����Ŀ���Ž��Ƕ�
#define ZXUP_A5B1_POS3    (ZXUP_A5B1_POS2+2)  // ����һ���Ž������߼�
#define ZXUP_A5B1_POS4    (ZXUP_A5B1_POS3+2)  // �Ž�����״̬λ
#define SIZE_OF_ZXUP_A5B1 (ZXUP_A5B1_POS4+1)  // ���ֽ���(6B)

//==TAG-A5B2�����߼���Ϣ==========================================================
#define ZXUP_A5B2_ADDR    (ZXUP_A5B1_ADDR+SIZE_OF_ZXUP_A5B1)  // ��ʼ��ַ
#define zxup_buffer_a5b2  (zxup_buffer+ZXUP_A5B2_ADDR)// ��ʼ��ַ����
#define ZXUP_A5B2_POS1    0                        // ����������
#define ZXUP_A5B2_POS2    (ZXUP_A5B2_POS1+2)  // ����������
#define ZXUP_A5B2_POS3    (ZXUP_A5B2_POS2+2)  // �����������
#define ZXUP_A5B2_POS4    (ZXUP_A5B2_POS3+1)  // ������������
#define ZXUP_A5B2_POS5    (ZXUP_A5B2_POS4+1)  // �������Ʊ����
#define ZXUP_A5B2_POS6    (ZXUP_A5B2_POS5+1)  // �������Ʊ����
#define ZXUP_A5B2_POS7    (ZXUP_A5B2_POS6+1)  // ��������������
#define ZXUP_A5B2_POS8    (ZXUP_A5B2_POS7+1)  // ��������������
#define SIZE_OF_ZXUP_A5B2 (ZXUP_A5B2_POS8+1)  // ���ֽ���(10B)

//==TAG-A5B3Һѹ���¶�============================================================
#define ZXUP_A5B3_ADDR    (ZXUP_A5B2_ADDR+SIZE_OF_ZXUP_A5B2)  // ��ʼ��ַ
#define zxup_buffer_a5b3  (zxup_buffer+ZXUP_A5B3_ADDR)// ��ʼ��ַ����
#define ZXUP_A5B3_POS1    0                        // Һѹ���¶�
#define ZXUP_A5B3_POS2    (ZXUP_A5B3_POS1+2)  // Һѹ��λ
#define ZXUP_A5B3_POS3    (ZXUP_A5B3_POS2+1)  // ������������
#define SIZE_OF_ZXUP_A5B3 (ZXUP_A5B3_POS3+1)  // ���ֽ���(4B)

//==TAG-A5B4������Ϣ==============================================================
#define ZXUP_A5B4_ADDR    (ZXUP_A5B3_ADDR+SIZE_OF_ZXUP_A5B3)  // ��ʼ��ַ
#define zxup_buffer_a5b4  (zxup_buffer+ZXUP_A5B4_ADDR)// ��ʼ��ַ����
#define ZXUP_A5B4_POS1    0                   // 1#���õ�ŷ�
#define ZXUP_A5B4_POS2    (ZXUP_A5B4_POS1+2)  // 2#���õ�ŷ�
#define ZXUP_A5B4_POS3    (ZXUP_A5B4_POS2+2)  // ����ѹ��
#define SIZE_OF_ZXUP_A5B4 (ZXUP_A5B4_POS3+2)  // ���ֽ���(6B)

//==TAG-A5B5����1 XHVME4400P1=====================================================
#define ZXUP_A5B5_ADDR    (ZXUP_A5B4_ADDR+SIZE_OF_ZXUP_A5B4)  // ��ʼ��ַ
#define zxup_buffer_a5b5  (zxup_buffer+ZXUP_A5B5_ADDR)// ��ʼ��ַ����
#define ZXUP_A5B5_POS1    0                   // ���ŷ�
#define ZXUP_A5B5_POS2    (ZXUP_A5B5_POS1+2)  // ����ŷ�
#define ZXUP_A5B5_POS3    (ZXUP_A5B5_POS2+2)  // ������ŷ�
#define ZXUP_A5B5_POS4    (ZXUP_A5B5_POS3+2)  // ������ŷ�
#define ZXUP_A5B5_POS5    (ZXUP_A5B5_POS4+2)  // �������ŷ�
#define ZXUP_A5B5_POS6    (ZXUP_A5B5_POS5+2)  // �������ŷ�
#define ZXUP_A5B5_POS7    (ZXUP_A5B5_POS6+2)  // �������ŷ�
#define ZXUP_A5B5_POS8    (ZXUP_A5B5_POS7+2)  // �������ŷ�
#define ZXUP_A5B5_POS9    (ZXUP_A5B5_POS8+2)  // MP1ѹ��
#define ZXUP_A5B5_POS10   (ZXUP_A5B5_POS9+2)  // LS1ѹ��
#define ZXUP_A5B5_POS11   (ZXUP_A5B5_POS10+2)  // MP2ѹ��
#define ZXUP_A5B5_POS12   (ZXUP_A5B5_POS11+2)  // LS2ѹ��
#define ZXUP_A5B5_POS13   (ZXUP_A5B5_POS12+2)  // ������ŷ�
#define SIZE_OF_ZXUP_A5B5 (ZXUP_A5B5_POS13+1)  // ���ֽ���(23B)

//==TAG-A5B6����3 ���λ==========================================================
#define ZXUP_A5B6_ADDR    (ZXUP_A5B5_ADDR+SIZE_OF_ZXUP_A5B5)  // ��ʼ��ַ
#define zxup_buffer_a5b6  (zxup_buffer+ZXUP_A5B6_ADDR)// ��ʼ��ַ����
#define ZXUP_A5B6_POS1    0                   // ״̬λ
#define SIZE_OF_ZXUP_A5B6 (ZXUP_A5B6_POS1+1)  // ���ֽ���(1B)

//==TAG-A5B7������Ϣ==============================================================
#define ZXUP_A5B7_ADDR    (ZXUP_A5B6_ADDR+SIZE_OF_ZXUP_A5B6)  // ��ʼ��ַ
#define zxup_buffer_a5b7  (zxup_buffer+ZXUP_A5B7_ADDR)// ��ʼ��ַ����
#define ZXUP_A5B7_POS1    0                   // ������ѹ��
#define ZXUP_A5B7_POS2    (ZXUP_A5B7_POS1+2)  // �����׳���
#define ZXUP_A5B7_POS3    (ZXUP_A5B7_POS2+2)  // ���ƽ�ⷧ
#define SIZE_OF_ZXUP_A5B7 (ZXUP_A5B7_POS3+2)  // ���ֽ���(6B)

//==TAG-A5B8�����߼���Ϣ==========================================================
#define ZXUP_A5B8_ADDR    (ZXUP_A5B7_ADDR+SIZE_OF_ZXUP_A5B7)  // ��ʼ��ַ
#define zxup_buffer_a5b8  (zxup_buffer+ZXUP_A5B8_ADDR)// ��ʼ��ַ����
#define ZXUP_A5B8_POS1    0                   // �������ͽ���״̬λ
#define ZXUP_A5B8_POS2    (ZXUP_A5B8_POS1+1)  // ͷ���־
#define ZXUP_A5B8_POS3    (ZXUP_A5B8_POS2+1)  // ǰ��ͷ���⿪��8
#define ZXUP_A5B8_POS4    (ZXUP_A5B8_POS3+1)  // ���ͷ���⿪��8
#define ZXUP_A5B8_POS5    (ZXUP_A5B8_POS4+1)  // ��λ
#define SIZE_OF_ZXUP_A5B8 (ZXUP_A5B8_POS5+1)  // ���ֽ���(5B)

//==TAG-A5B9�ױ������Ʒ�==========================================================
#define ZXUP_A5B9_ADDR    (ZXUP_A5B8_ADDR+SIZE_OF_ZXUP_A5B8)  // ��ʼ��ַ
#define zxup_buffer_a5b9  (zxup_buffer+ZXUP_A5B9_ADDR)// ��ʼ��ַ����
#define ZXUP_A5B9_POS1    0                   // ״̬�ֽ�
#define ZXUP_A5B9_POS2    (ZXUP_A5B9_POS1+1)  // ������ѹ��
#define SIZE_OF_ZXUP_A5B9 (ZXUP_A5B9_POS2+2)  // ���ֽ���(3B)

//==TAG-A5BA���ƽ�ⷧ============================================================
#define ZXUP_A5BA_ADDR    (ZXUP_A5B9_ADDR+SIZE_OF_ZXUP_A5B9)  // ��ʼ��ַ
#define zxup_buffer_a5ba  (zxup_buffer+ZXUP_A5BA_ADDR)// ��ʼ��ַ����
#define ZXUP_A5BA_POS1    0                   // ����ƽ�ⷧ����
#define ZXUP_A5BA_POS2    (ZXUP_A5BA_POS1+2)  // �ұ��ƽ�ⷧ����
#define SIZE_OF_ZXUP_A5BA (ZXUP_A5BA_POS2+2)  // ���ֽ���(4B)

//==TAG-A5BB�����================================================================
#define ZXUP_A5BB_ADDR    (ZXUP_A5BA_ADDR+SIZE_OF_ZXUP_A5BA)  // ��ʼ��ַ
#define zxup_buffer_a5bb  (zxup_buffer+ZXUP_A5BB_ADDR)// ��ʼ��ַ����
#define ZXUP_A5BB_POS1    0                   // ������ŷ�
#define ZXUP_A5BB_POS2    (ZXUP_A5BB_POS1+2)  // �����ŷ�
#define ZXUP_A5BB_POS3    (ZXUP_A5BB_POS2+2)  // �ͱ�ѹ��
#define SIZE_OF_ZXUP_A5BB (ZXUP_A5BB_POS3+2)  // ���ֽ���(6B)

//==TAG-A5BC�������==============================================================
#define ZXUP_A5BC_ADDR    (ZXUP_A5BB_ADDR+SIZE_OF_ZXUP_A5BB)  // ��ʼ��ַ
#define zxup_buffer_a5bc  (zxup_buffer+ZXUP_A5BC_ADDR)// ��ʼ��ַ����
#define ZXUP_A5BC_POS1    0                   // ������
#define ZXUP_A5BC_POS2    (ZXUP_A5BC_POS1+2)  // ��Ͳת��
#define ZXUP_A5BC_POS3    (ZXUP_A5BC_POS2+2)  // ״̬λ
#define SIZE_OF_ZXUP_A5BC (ZXUP_A5BC_POS3+1)  // ���ֽ���(5B)

//==TAG-A5BD�����================================================================
#define ZXUP_A5BD_ADDR    (ZXUP_A5BC_ADDR+SIZE_OF_ZXUP_A5BC)  // ��ʼ��ַ
#define zxup_buffer_a5bd  (zxup_buffer+ZXUP_A5BD_ADDR)// ��ʼ��ַ����
#define ZXUP_A5BD_POS1    0                   // ������ŷ�
#define ZXUP_A5BD_POS2    (ZXUP_A5BD_POS1+2)  // �����ŷ�
#define ZXUP_A5BD_POS3    (ZXUP_A5BD_POS2+2)  // �ͱ�ѹ��
#define SIZE_OF_ZXUP_A5BD (ZXUP_A5BD_POS3+2)  // ���ֽ���(6B)

//==TAG-A5BE�������==============================================================
#define ZXUP_A5BE_ADDR    (ZXUP_A5BD_ADDR+SIZE_OF_ZXUP_A5BD)  // ��ʼ��ַ
#define zxup_buffer_a5be  (zxup_buffer+ZXUP_A5BE_ADDR)// ��ʼ��ַ����
#define ZXUP_A5BE_POS1    0                   // ������
#define ZXUP_A5BE_POS2    (ZXUP_A5BE_POS1+2)  // ��Ͳת��
#define ZXUP_A5BE_POS3    (ZXUP_A5BE_POS2+2)  // ״̬λ
#define SIZE_OF_ZXUP_A5BE (ZXUP_A5BE_POS3+1)  // ���ֽ���(5B)

//==TAG-A5BF��ת��================================================================
#define ZXUP_A5BF_ADDR    (ZXUP_A5BE_ADDR+SIZE_OF_ZXUP_A5BE)  // ��ʼ��ַ
#define zxup_buffer_a5bf  (zxup_buffer+ZXUP_A5BF_ADDR)// ��ʼ��ַ����
#define ZXUP_A5BF_POS1    0                   // ���ת��ŷ�
#define ZXUP_A5BF_POS2    (ZXUP_A5BF_POS1+2)  // �һ�ת��ŷ�
#define ZXUP_A5BF_POS3    (ZXUP_A5BF_POS2+2)  // ��ת���巧
#define ZXUP_A5BF_POS4    (ZXUP_A5BF_POS3+2)  // �ͱ�ѹ������תѹ����⣩
#define SIZE_OF_ZXUP_A5BF (ZXUP_A5BF_POS4+2)  // ���ֽ���(8B)

//==TAG-A5C0��ת�ƶ���============================================================
#define ZXUP_A5C0_ADDR    (ZXUP_A5BF_ADDR+SIZE_OF_ZXUP_A5BF)  // ��ʼ��ַ
#define zxup_buffer_a5c0  (zxup_buffer+ZXUP_A5C0_ADDR)// ��ʼ��ַ����
#define ZXUP_A5C0_POS1    0                   // �ֽ�״̬λ
#define ZXUP_A5C0_POS2    (ZXUP_A5C0_POS1+1)  // ��Ͳת��
#define SIZE_OF_ZXUP_A5C0 (ZXUP_A5C0_POS2+2)  // ���ֽ���(3B)

//==TAG-A5C1������1===============================================================
#define ZXUP_A5C1_ADDR    (ZXUP_A5C0_ADDR+SIZE_OF_ZXUP_A5C0)  // ��ʼ��ַ
#define zxup_buffer_a5c1  (zxup_buffer+ZXUP_A5C1_ADDR)// ��ʼ��ַ����
#define ZXUP_A5C1_POS1    0                   // �ֽ�״̬λ1
#define ZXUP_A5C1_POS2    (ZXUP_A5C1_POS1+1)  // �ֽ�״̬λ2
#define SIZE_OF_ZXUP_A5C1 (ZXUP_A5C1_POS2+1)  // ���ֽ���(2B)

//==TAG-A5C2������2(�����λ)=====================================================
#define ZXUP_A5C2_ADDR    (ZXUP_A5C1_ADDR+SIZE_OF_ZXUP_A5C1)  // ��ʼ��ַ
#define zxup_buffer_a5c2  (zxup_buffer+ZXUP_A5C2_ADDR)// ��ʼ��ַ����
#define ZXUP_A5C2_POS1    0                   // ѹ��ѡ��
#define ZXUP_A5C2_POS2    (ZXUP_A5C2_POS1+2)  // �ֽ�״̬λ
#define SIZE_OF_ZXUP_A5C2 (ZXUP_A5C2_POS2+1)  // ���ֽ���(3B)

//==TAG-A5C3������============================================================
#define ZXUP_A5C3_ADDR    (ZXUP_A5C2_ADDR+SIZE_OF_ZXUP_A5C2)  // ��ʼ��ַ
#define zxup_buffer_a5c3  (zxup_buffer+ZXUP_A5C3_ADDR)// ��ʼ��ַ����
#define ZXUP_A5C3_POS1    0                   // ������
#define ZXUP_A5C3_POS2    (ZXUP_A5C3_POS1+2)  // ������
#define ZXUP_A5C3_POS3    (ZXUP_A5C3_POS2+2)  // ����������
#define ZXUP_A5C3_POS4    (ZXUP_A5C3_POS3+2)  // �ֽ�״̬λ1
#define ZXUP_A5C3_POS5    (ZXUP_A5C3_POS4+1)  // �ֽ�״̬λ2
#define ZXUP_A5C3_POS6    (ZXUP_A5C3_POS5+1)  // �Ž�����
#define ZXUP_A5C3_POS7    (ZXUP_A5C3_POS6+2)  // �Ž�����
#define SIZE_OF_ZXUP_A5C3 (ZXUP_A5C3_POS7+2)  // ���ֽ���(12B)

//==TAG-A5C4�ҳ�����============================================================
#define ZXUP_A5C4_ADDR    (ZXUP_A5C3_ADDR+SIZE_OF_ZXUP_A5C3)  // ��ʼ��ַ
#define zxup_buffer_a5c4  (zxup_buffer+ZXUP_A5C4_ADDR)// ��ʼ��ַ����
#define ZXUP_A5C4_POS1    0                   // ������
#define ZXUP_A5C4_POS2    (ZXUP_A5C4_POS1+2)  // ������
#define ZXUP_A5C4_POS3    (ZXUP_A5C4_POS2+2)  // ����������
#define ZXUP_A5C4_POS4    (ZXUP_A5C4_POS3+2)  // �ֽ�״̬λ1
#define ZXUP_A5C4_POS5    (ZXUP_A5C4_POS4+1)  // �ֽ�״̬λ2
#define ZXUP_A5C4_POS6    (ZXUP_A5C4_POS5+1)  // �Ž�����
#define ZXUP_A5C4_POS7    (ZXUP_A5C4_POS6+2)  // �Ž�����
#define SIZE_OF_ZXUP_A5C4 (ZXUP_A5C4_POS7+2)  // ���ֽ���(12B)

//==TAG-A5C8��ҵ�ͺ���Ϣ==========================================================
#define ZXUP_A5C8_ADDR    (ZXUP_A5C4_ADDR+SIZE_OF_ZXUP_A5C4)  // ��ʼ��ַ
#define zxup_buffer_a5c8  (zxup_buffer+ZXUP_A5C8_ADDR)// ��ʼ��ַ����
#define ZXUP_A5C8_POS1    0                   // ��ҵ������ʱ��
#define ZXUP_A5C8_POS2    (ZXUP_A5C8_POS1+4)  // ��ҵȼ����������
#define ZXUP_A5C8_POS3    (ZXUP_A5C8_POS2+4)  // ��ҵƽ���ͺ�
#define SIZE_OF_ZXUP_A5C8 (ZXUP_A5C8_POS3+2)  // ���ֽ���(10B)

//==TAG-A5C9 ECU��Ӧ����CAN֡=====================================================
#define ZXUP_A5C9_ADDR    (ZXUP_A5C8_ADDR+SIZE_OF_ZXUP_A5C8)  // ��ʼ��ַ
#define zxup_buffer_a5c9  (zxup_buffer+ZXUP_A5C9_ADDR)// ��ʼ��ַ����
#define ZXUP_A5C9_POS1    0                   // ��ҵ������ʱ��
#define ZXUP_A5C9_POS2    (ZXUP_A5C9_POS1+1)  // ��ҵȼ����������
#define ZXUP_A5C9_POS3    (ZXUP_A5C9_POS2+4)  // ��ҵƽ���ͺ�
#define SIZE_OF_ZXUP_A5C9 (ZXUP_A5C9_POS3+8)  // ���ֽ���(13B)

// �ϳ����ݻ����С
#define SIZE_OF_ZXUP_BUFFER  (ZXUP_A5C9_ADDR+SIZE_OF_ZXUP_A5C9)

/******************************************************************************
* Macros(�³�ͨ��): ���ػ��³�������ź͵�ַ����
******************************************************************************/
//==TAG-A5E0�ڵ�״̬==============================================================
#define ZXDOWN_A5E0_ADDR    0  // ��ʼ��ַ
#define zxdown_buffer_a5e0  (zxdown_buffer+ZXDOWN_A5E0_ADDR)  // ��ʼ��ַ����
#define ZXDOWN_A5E0_POS1    0                     // �ֽ�״̬λ1
#define ZXDOWN_A5E0_POS2    (ZXDOWN_A5E0_POS1+1)  // �ֽ�״̬λ2
#define SIZE_OF_ZXDOWN_A5E0 (ZXDOWN_A5E0_POS2+1)  // ���ֽ���(2B)

//==TAG-A5E1����ϵͳ==============================================================
#define ZXDOWN_A5E1_ADDR    (ZXDOWN_A5E0_ADDR+SIZE_OF_ZXDOWN_A5E0)  // ��ʼ��ַ
#define zxdown_buffer_a5e1  (zxdown_buffer+ZXDOWN_A5E1_ADDR) // ��ʼ��ַ����
#define ZXDOWN_A5E1_POS1    0                     // �����䵵λ
#define ZXDOWN_A5E1_POS2    (ZXDOWN_A5E1_POS1+1)  // �ֽ�״̬λ
#define ZXDOWN_A5E1_POS3    (ZXDOWN_A5E1_POS2+1)  // ����������
#define ZXDOWN_A5E1_POS4    (ZXDOWN_A5E1_POS3+2)  // �����������ѹ
#define ZXDOWN_A5E1_POS5    (ZXDOWN_A5E1_POS4+1)  // �����λ��
#define SIZE_OF_ZXDOWN_A5E1 (ZXDOWN_A5E1_POS5+1)  // ���ֽ���(6B)

//==TAG-A5E2֧�������Ϣ==========================================================
#define ZXDOWN_A5E2_ADDR    (ZXDOWN_A5E1_ADDR+SIZE_OF_ZXDOWN_A5E1)  // ��ʼ��ַ
#define zxdown_buffer_a5e2  (zxdown_buffer+ZXDOWN_A5E2_ADDR)// ��ʼ��ַ����
#define ZXDOWN_A5E2_POS1    0                     // �ֽ�״̬λ1
#define ZXDOWN_A5E2_POS2    (ZXDOWN_A5E2_POS1+1)  // �ֽ�״̬λ2
#define ZXDOWN_A5E2_POS3    (ZXDOWN_A5E2_POS2+1)  // �ֽ�״̬λ3
#define SIZE_OF_ZXDOWN_A5E2 (ZXDOWN_A5E2_POS3+1)  // ���ֽ���(3B)

//==TAG-A5E3����ϵͳ==============================================================
#define ZXDOWN_A5E3_ADDR    (ZXDOWN_A5E2_ADDR+SIZE_OF_ZXDOWN_A5E2)  // ��ʼ��ַ
#define zxdown_buffer_a5e3  (zxdown_buffer+ZXDOWN_A5E3_ADDR)// ��ʼ��ַ����
#define ZXDOWN_A5E3_POS1    0                     // ����ѹ��
#define ZXDOWN_A5E3_POS2    (ZXDOWN_A5E3_POS1+8)  // �����г�
#define ZXDOWN_A5E3_POS3    (ZXDOWN_A5E3_POS2+8)  // ��������
#define ZXDOWN_A5E3_POS4    (ZXDOWN_A5E3_POS3+4)  // �ֽ�״̬λ1
#define ZXDOWN_A5E3_POS5    (ZXDOWN_A5E3_POS4+1)  // �ֽ�״̬λ2
#define SIZE_OF_ZXDOWN_A5E3 (ZXDOWN_A5E3_POS5+1)  // ���ֽ���(22B)

//==TAG-A5E4ת��ϵͳ��ȫ���棩====================================================
#define ZXDOWN_A5E4_ADDR    (ZXDOWN_A5E3_ADDR+SIZE_OF_ZXDOWN_A5E3)  // ��ʼ��ַ
#define zxdown_buffer_a5e4  (zxdown_buffer+ZXDOWN_A5E4_ADDR)// ��ʼ��ַ����
#define ZXDOWN_A5E4_POS1    0                     // һ����ת��Ƕ�
#define ZXDOWN_A5E4_POS2    (ZXDOWN_A5E4_POS1+8)  // ����������ת��Ƕ�
#define ZXDOWN_A5E4_POS3    (ZXDOWN_A5E4_POS2+8)  // һ���ᴫ��������
#define ZXDOWN_A5E4_POS4    (ZXDOWN_A5E4_POS3+8)  // ���������ᴫ��������
#define ZXDOWN_A5E4_POS5    (ZXDOWN_A5E4_POS4+8)  // ��ǰת��ģʽ
#define ZXDOWN_A5E4_POS6    (ZXDOWN_A5E4_POS5+1)  // Ŀ��ת��ģʽ
#define ZXDOWN_A5E4_POS7    (ZXDOWN_A5E4_POS6+1)  // ת��ϵͳѹ��
#define ZXDOWN_A5E4_POS8    (ZXDOWN_A5E4_POS7+2)  // ������ѹ������
#define ZXDOWN_A5E4_POS9    (ZXDOWN_A5E4_POS8+2)  // 123������ת��ռ�ձ�
#define ZXDOWN_A5E4_POS10   (ZXDOWN_A5E4_POS9+6)  // 456������ת��ռ�ձ�
#define ZXDOWN_A5E4_POS11   (ZXDOWN_A5E4_POS10+6)  // ����ֹ��
#define SIZE_OF_ZXDOWN_A5E4 (ZXDOWN_A5E4_POS11+1)  // ���ֽ���(51B)

//==TAG-A5E5ת��ϵͳ��������======================================================
#define ZXDOWN_A5E5_ADDR    (ZXDOWN_A5E4_ADDR+SIZE_OF_ZXDOWN_A5E4)  // ��ʼ��ַ
#define zxdown_buffer_a5e5  (zxdown_buffer+ZXDOWN_A5E5_ADDR)// ��ʼ��ַ����
#define ZXDOWN_A5E5_POS1    0                     // ��ǰһ��ת��
#define ZXDOWN_A5E5_POS2    (ZXDOWN_A5E5_POS1+2)  // ת������ѹ��(bar)
#define ZXDOWN_A5E5_POS3    (ZXDOWN_A5E5_POS2+2)  // Ŀ��ת��ģʽ+��ǰת��ģʽ
#define ZXDOWN_A5E5_POS4    (ZXDOWN_A5E5_POS3+2)  // ������λ
#define ZXDOWN_A5E5_POS5    (ZXDOWN_A5E5_POS4+1)  // �ֽ�״̬λ1
#define ZXDOWN_A5E5_POS6    (ZXDOWN_A5E5_POS5+1)  // �ֽ�״̬λ2
#define SIZE_OF_ZXDOWN_A5E5 (ZXDOWN_A5E5_POS6+1)  // ���ֽ���(9B)

//==TAG-A5E6�ƶ�ϵͳ==============================================================
#define ZXDOWN_A5E6_ADDR    (ZXDOWN_A5E5_ADDR+SIZE_OF_ZXDOWN_A5E5)  // ��ʼ��ַ
#define zxdown_buffer_a5e6  (zxdown_buffer+ZXDOWN_A5E6_ADDR)// ��ʼ��ַ����
#define ZXDOWN_A5E6_POS1    0                     // ��·һ��ѹ
#define ZXDOWN_A5E6_POS2    (ZXDOWN_A5E6_POS1+2)  // ��·����ѹ
#define ZXDOWN_A5E6_POS3    (ZXDOWN_A5E6_POS2+2)  // �ƶ�ѹ��
#define ZXDOWN_A5E6_POS4    (ZXDOWN_A5E6_POS3+2)  // ��ѹ���
#define ZXDOWN_A5E6_POS5    (ZXDOWN_A5E6_POS4+2)  // ����������
#define ZXDOWN_A5E6_POS6    (ZXDOWN_A5E6_POS5+2)  // �������ذٷֱ�
#define ZXDOWN_A5E6_POS7    (ZXDOWN_A5E6_POS6+1)  // ���������
#define ZXDOWN_A5E6_POS8    (ZXDOWN_A5E6_POS7+2)  // �ֽ�״̬λ1
#define ZXDOWN_A5E6_POS9    (ZXDOWN_A5E6_POS8+1)  // �ֽ�״̬λ2
#define SIZE_OF_ZXDOWN_A5E6 (ZXDOWN_A5E6_POS9+1)  // ���ֽ���(15B)

//==TAG-A5E7����ϵͳ==============================================================
#define ZXDOWN_A5E7_ADDR    (ZXDOWN_A5E6_ADDR+SIZE_OF_ZXDOWN_A5E6)  // ��ʼ��ַ
#define zxdown_buffer_a5e7  (zxdown_buffer+ZXDOWN_A5E7_ADDR)// ��ʼ��ַ����
#define ZXDOWN_A5E7_POS1    0                     // �ֽ�״̬λ
#define SIZE_OF_ZXDOWN_A5E7 (ZXDOWN_A5E7_POS1+1)  // ���ֽ���(1B)

//==TAG-A5E8����ȡ��ϵͳ==========================================================
#define ZXDOWN_A5E8_ADDR    (ZXDOWN_A5E7_ADDR+SIZE_OF_ZXDOWN_A5E7)  // ��ʼ��ַ
#define zxdown_buffer_a5e8  (zxdown_buffer+ZXDOWN_A5E8_ADDR)// ��ʼ��ַ����
#define ZXDOWN_A5E8_POS1    0                     // �ֽ�״̬λ1
#define ZXDOWN_A5E8_POS2    (ZXDOWN_A5E8_POS1+1)  // �ֽ�״̬λ2
#define SIZE_OF_ZXDOWN_A5E8 (ZXDOWN_A5E8_POS2+1)  // ���ֽ���(2B)

//==TAG-A5E9Һѹϵͳ==============================================================
#define ZXDOWN_A5E9_ADDR    (ZXDOWN_A5E8_ADDR+SIZE_OF_ZXDOWN_A5E8)  // ��ʼ��ַ
#define zxdown_buffer_a5e9  (zxdown_buffer+ZXDOWN_A5E9_ADDR)// ��ʼ��ַ����
#define ZXDOWN_A5E9_POS1    0                     // Һѹ���¶�
#define ZXDOWN_A5E9_POS2    (ZXDOWN_A5E9_POS1+1)  // Һѹϵͳѹ��
#define SIZE_OF_ZXDOWN_A5E9 (ZXDOWN_A5E9_POS2+2)  // ���ֽ���(3B)

//==TAG-A5EA˫��������ϵͳ========================================================
#define ZXDOWN_A5EA_ADDR    (ZXDOWN_A5E9_ADDR+SIZE_OF_ZXDOWN_A5E9)  // ��ʼ��ַ
#define zxdown_buffer_a5ea  (zxdown_buffer+ZXDOWN_A5EA_ADDR)// ��ʼ��ַ����
#define ZXDOWN_A5EA_POS1    0                     // �ֽ�״̬λ1
#define ZXDOWN_A5EA_POS2    (ZXDOWN_A5EA_POS1+1)  // �ֽ�״̬λ2
#define ZXDOWN_A5EA_POS3    (ZXDOWN_A5EA_POS2+1)  // �ϳ�������ˮ��
#define ZXDOWN_A5EA_POS4    (ZXDOWN_A5EA_POS3+1)  // �ϳ�����������ѹ��
#define ZXDOWN_A5EA_POS5    (ZXDOWN_A5EA_POS4+2)  // �ϳ�������ת��
#define ZXDOWN_A5EA_POS6    (ZXDOWN_A5EA_POS5+2)  // �ϳ��ó���ѹ��
#define ZXDOWN_A5EA_POS7    (ZXDOWN_A5EA_POS6+2)  // �����Ƶ���
#define ZXDOWN_A5EA_POS8    (ZXDOWN_A5EA_POS7+2)  // ɢ����Һѹ�����ÿ��Ƶ���
#define SIZE_OF_ZXDOWN_A5EA (ZXDOWN_A5EA_POS8+2)  // ���ֽ���(13B)

//==TAG-A5EB��̥̥ѹ==============================================================
#define ZXDOWN_A5EB_ADDR    (ZXDOWN_A5EA_ADDR+SIZE_OF_ZXDOWN_A5EA)  // ��ʼ��ַ
#define zxdown_buffer_a5eb  (zxdown_buffer+ZXDOWN_A5EB_ADDR)// ��ʼ��ַ����
#define ZXDOWN_A5EB_POS1    0         // ̥ѹ#1-18
#define SIZE_OF_ZXDOWN_A5EB (ZXDOWN_A5EB_POS1+18)  // ���ֽ���(18B)

//==TAG-A5EC��֧�����============================================================
#define ZXDOWN_A5EC_ADDR    (ZXDOWN_A5EB_ADDR+SIZE_OF_ZXDOWN_A5EB)  // ��ʼ��ַ
#define zxdown_buffer_a5ec  (zxdown_buffer+ZXDOWN_A5EC_ADDR)// ��ʼ��ַ����
#define ZXDOWN_A5EC_POS1    0                     // �ֽ�״̬λ1
#define ZXDOWN_A5EC_POS2    (ZXDOWN_A5EC_POS1+1)  // �ֽ�״̬λ2
#define ZXDOWN_A5EC_POS3    (ZXDOWN_A5EC_POS2+1)  // �ֽ�״̬λ3
#define ZXDOWN_A5EC_POS4    (ZXDOWN_A5EC_POS3+1)  // �ֽ�״̬λ4
#define ZXDOWN_A5EC_POS5    (ZXDOWN_A5EC_POS4+1)  // �ֽ�״̬λ5
#define ZXDOWN_A5EC_POS6    (ZXDOWN_A5EC_POS5+1)  // Ԥ��
#define ZXDOWN_A5EC_POS7    (ZXDOWN_A5EC_POS6+1)  // DIR
#define ZXDOWN_A5EC_POS8    (ZXDOWN_A5EC_POS7+1)  // ��ƽ������
#define SIZE_OF_ZXDOWN_A5EC (ZXDOWN_A5EC_POS8+1)  // ���ֽ���(8B)

//==TAG-A5ED��֧�����============================================================
#define ZXDOWN_A5ED_ADDR    (ZXDOWN_A5EC_ADDR+SIZE_OF_ZXDOWN_A5EC)  // ��ʼ��ַ
#define zxdown_buffer_a5ed  (zxdown_buffer+ZXDOWN_A5ED_ADDR)// ��ʼ��ַ����
#define ZXDOWN_A5ED_POS1    0                     // �ֽ�״̬λ1
#define ZXDOWN_A5ED_POS2    (ZXDOWN_A5ED_POS1+1)  // �ֽ�״̬λ2
#define ZXDOWN_A5ED_POS3    (ZXDOWN_A5ED_POS2+1)  // �ֽ�״̬λ3
#define ZXDOWN_A5ED_POS4    (ZXDOWN_A5ED_POS3+1)  // �ֽ�״̬λ4
#define ZXDOWN_A5ED_POS5    (ZXDOWN_A5ED_POS4+1)  // �ֽ�״̬λ5
#define ZXDOWN_A5ED_POS6    (ZXDOWN_A5ED_POS5+1)  // Ԥ��
#define ZXDOWN_A5ED_POS7    (ZXDOWN_A5ED_POS6+1)  // DIR
#define ZXDOWN_A5ED_POS8    (ZXDOWN_A5ED_POS7+1)  // ��ƽ������
#define SIZE_OF_ZXDOWN_A5ED (ZXDOWN_A5ED_POS8+1)  // ���ֽ���(8B)

//==TAG-A5EE�п�̨������Ϣ========================================================
#define ZXDOWN_A5EE_ADDR    (ZXDOWN_A5ED_ADDR+SIZE_OF_ZXDOWN_A5ED)  // ��ʼ��ַ
#define zxdown_buffer_a5ee  (zxdown_buffer+ZXDOWN_A5EE_ADDR)// ��ʼ��ַ����
#define ZXDOWN_A5EE_POS1    0                     // �ֽ�״̬λ1
#define ZXDOWN_A5EE_POS2    (ZXDOWN_A5EE_POS1+1)  // �ֽ�״̬λ2
#define ZXDOWN_A5EE_POS3    (ZXDOWN_A5EE_POS2+1)  // �ֽ�״̬λ3
#define ZXDOWN_A5EE_POS4    (ZXDOWN_A5EE_POS3+1)  // �ֽ�״̬λ4
#define ZXDOWN_A5EE_POS5    (ZXDOWN_A5EE_POS4+1)  // �ֽ�״̬λ5
#define ZXDOWN_A5EE_POS6    (ZXDOWN_A5EE_POS5+1)  // �ֽ�״̬λ6
#define ZXDOWN_A5EE_POS7    (ZXDOWN_A5EE_POS6+1)  // �ֽ�״̬λ7
#define ZXDOWN_A5EE_POS8    (ZXDOWN_A5EE_POS7+1)  // ����ת��Ƕ�
#define ZXDOWN_A5EE_POS9    (ZXDOWN_A5EE_POS8+1)  // �ֽ�״̬λ8
#define SIZE_OF_ZXDOWN_A5EE (ZXDOWN_A5EE_POS9+1)  // ���ֽ���(9B)

//==TAG-A5A3֧����ҵ��Ϣ==========================================================
#define ZXDOWN_A5A3_ADDR    (ZXDOWN_A5EE_ADDR+SIZE_OF_ZXDOWN_A5EE)  // ��ʼ��ַ
#define zxdown_buffer_a5a3  (zxdown_buffer+ZXDOWN_A5A3_ADDR)// ��ʼ��ַ����
#define ZXDOWN_A5A3_POS1    0                     // ˮƽ֧�ȳ���
#define ZXDOWN_A5A3_POS2    (ZXDOWN_A5A3_POS1+8)  // ֧��ѹ��
#define ZXDOWN_A5A3_POS3    (ZXDOWN_A5A3_POS2+8)  // �ڸ׳���
#define SIZE_OF_ZXDOWN_A5A3 (ZXDOWN_A5A3_POS3+8)  // ���ֽ���(24B)

//==TAG-A5A4����֧����ҵ��Ϣ=======================================================
#define ZXDOWN_A5A4_ADDR    (ZXDOWN_A5A3_ADDR+SIZE_OF_ZXDOWN_A5A3)  // ��ʼ��ַ
#define zxdown_buffer_a5a4  (zxdown_buffer+ZXDOWN_A5A4_ADDR)// ��ʼ��ַ����
#define ZXDOWN_A5A4_POS1    0                     // ����֧��״̬
#define ZXDOWN_A5A4_POS2    (ZXDOWN_A5A4_POS1+1)  // ��ǰ����֧��ѹ��
#define ZXDOWN_A5A4_POS3    (ZXDOWN_A5A4_POS2+2)  // ��ǰ����֧��ѹ��
#define SIZE_OF_ZXDOWN_A5A4 (ZXDOWN_A5A4_POS3+2)  // ���ֽ���(5B)

// �³��������ݻ����С
#define SIZE_OF_ZXDOWN_BUFFER   (ZXDOWN_A5A4_ADDR+SIZE_OF_ZXDOWN_A5A4)

/******************************************************************************
* Macros(���̷�����): ���ػ����̷�����������ź͵�ַ����
******************************************************************************/
//==TAG-A502������Ϣ(������������)================================================
#define ZXENGINE_A502_ADDR      (0)  // ��ʼ��ַ
#define zxengine_buffer_a502    (zxengine_buffer+ZXENGINE_A502_ADDR)  // ��ʼ��ַ����
#define ZXENGINE_A502_POS1_ADDR 0                            // ����ѹ��
#define ZXENGINE_A502_POS2_ADDR (ZXENGINE_A502_POS1_ADDR+1)  // �����¶�
#define SIZE_OF_ZXENGINE_A502   (ZXENGINE_A502_POS2_ADDR+2)  // ���ֽ���(3B)

//==TAG-A5EF���������в���(���ġ����塢����)======================================
#define ZXENGINE_A5EF_ADDR    (ZXENGINE_A502_ADDR+SIZE_OF_ZXENGINE_A502)  // ��ʼ��ַ
#define zxengine_buffer_a5ef  (zxengine_buffer+ZXENGINE_A5EF_ADDR)  // ��ʼ��ַ����
#define ZXENGINE_A5EF_POS1    0                       // ������ת��
#define ZXENGINE_A5EF_POS2    (ZXENGINE_A5EF_POS1+2)  // ʵ��Ť�ذٷֱ�
#define ZXENGINE_A5EF_POS3    (ZXENGINE_A5EF_POS2+1)  // Ħ��Ť�ذٷֱ�
#define ZXENGINE_A5EF_POS4    (ZXENGINE_A5EF_POS3+1)  // �������ѹ��
#define ZXENGINE_A5EF_POS5    (ZXENGINE_A5EF_POS4+1)  // ��������¶�
#define ZXENGINE_A5EF_POS6    (ZXENGINE_A5EF_POS5+1)  // ��ȴҺ�¶�
#define ZXENGINE_A5EF_POS7    (ZXENGINE_A5EF_POS6+1)  // �����¶�
#define ZXENGINE_A5EF_POS8    (ZXENGINE_A5EF_POS7+2)  // ����Һλ
#define ZXENGINE_A5EF_POS9    (ZXENGINE_A5EF_POS8+1)  // ����ѹ��
#define ZXENGINE_A5EF_POS10   (ZXENGINE_A5EF_POS9+1)  // ������������ʱ��
#define ZXENGINE_A5EF_POS11   (ZXENGINE_A5EF_POS10+4)  // ����̤��ٷֱ�
#define ZXENGINE_A5EF_POS12   (ZXENGINE_A5EF_POS11+1)  // ����
#define ZXENGINE_A5EF_POS13   (ZXENGINE_A5EF_POS12+2)  // �ֽ�״̬λ1
#define ZXENGINE_A5EF_POS14   (ZXENGINE_A5EF_POS13+1)  // �ֽ�״̬λ2
#define ZXENGINE_A5EF_POS15   (ZXENGINE_A5EF_POS14+1)  // Ѳ���趨�ٶ�
#define ZXENGINE_A5EF_POS16   (ZXENGINE_A5EF_POS15+1)  // ������ȼ��������
#define ZXENGINE_A5EF_POS17   (ZXENGINE_A5EF_POS16+2)  // ������ƽ��ȼ��������
#define ZXENGINE_A5EF_POS18   (ZXENGINE_A5EF_POS17+2)  // ȼ�����ͺ���
#define ZXENGINE_A5EF_POS19   (ZXENGINE_A5EF_POS18+4)  // ����ʻ���
#define ZXENGINE_A5EF_POS20   (ZXENGINE_A5EF_POS19+4)  // ȼ��Һλ1
#define ZXENGINE_A5EF_POS21   (ZXENGINE_A5EF_POS20+1)  // ȼ��Һλ2
#define ZXENGINE_A5EF_POS22   (ZXENGINE_A5EF_POS21+1)  // �ֽ�״̬λ3
#define ZXENGINE_A5EF_POS23   (ZXENGINE_A5EF_POS22+1)  // �ֽ�״̬λ4
#define SIZE_OF_ZXENGINE_A5EF (ZXENGINE_A5EF_POS23+1)  // ���ֽ���(37B)

//==TAG-A5F0��ʻ�ͺ�==============================================================
#define ZXENGINE_A5F0_ADDR    (ZXENGINE_A5EF_ADDR+SIZE_OF_ZXENGINE_A5EF) // ��ʼ��ַ
#define zxengine_buffer_a5f0  (zxengine_buffer+ZXENGINE_A5F0_ADDR)// ��ʼ��ַ����
#define ZXENGINE_A5F0_POS1    0                       // ��ʻ������ʱ��
#define ZXENGINE_A5F0_POS2    (ZXENGINE_A5F0_POS1+4)  // ��ʻȼ�����ͺ���
#define ZXENGINE_A5F0_POS3    (ZXENGINE_A5F0_POS2+4)  // �ٹ����ͺ�
#define SIZE_OF_ZXENGINE_A5F0 (ZXENGINE_A5F0_POS3+4)  // ���ֽ���(12B)

//==TAG-A5F1 SCR���������壩======================================================
#define ZXENGINE_A5F1_ADDR    (ZXENGINE_A5F0_ADDR+SIZE_OF_ZXENGINE_A5F0)                             // ��ʼ��ַ
#define zxengine_buffer_a5f1  (zxengine_buffer+ZXENGINE_A5F1_ADDR)// ��ʼ��ַ����
#define ZXENGINE_A5F1_POS1    0                       // ��������״̬
#define ZXENGINE_A5F1_POS2    (ZXENGINE_A5F1_POS1+1)  // T15_DCU
#define ZXENGINE_A5F1_POS3    (ZXENGINE_A5F1_POS2+1)  // ���ر�ѹ��
#define ZXENGINE_A5F1_POS4    (ZXENGINE_A5F1_POS3+2)  // ������Һλ
#define ZXENGINE_A5F1_POS5    (ZXENGINE_A5F1_POS4+1)  // �������¶�
#define ZXENGINE_A5F1_POS6    (ZXENGINE_A5F1_POS5+1)  // ����������
#define ZXENGINE_A5F1_POS7    (ZXENGINE_A5F1_POS6+2)  // SCR����NOxŨ��
#define ZXENGINE_A5F1_POS8    (ZXENGINE_A5F1_POS7+2)  // SCR����NOxŨ��
#define ZXENGINE_A5F1_POS9    (ZXENGINE_A5F1_POS8+2)  // SCR���������¶�(T6�¶�)
#define ZXENGINE_A5F1_POS10   (ZXENGINE_A5F1_POS9+2)  // SCR���������¶�(T7�¶�)
#define ZXENGINE_A5F1_POS11   (ZXENGINE_A5F1_POS10+2)  // ����Ũ��(SPN 3516)
#define ZXENGINE_A5F1_POS12   (ZXENGINE_A5F1_POS11+1)  // �ۼ�����������
#define ZXENGINE_A5F1_POS13   (ZXENGINE_A5F1_POS12+4)  // ����Ʒ�ʴ������¶�(SPN 3515)
#define ZXENGINE_A5F1_POS14   (ZXENGINE_A5F1_POS13+1)  // Ʒ���¶ȴ�����FMI (SPN 3519)
#define ZXENGINE_A5F1_POS15   (ZXENGINE_A5F1_POS14+1)  // Ʒ�ʴ�����FMI (SPN3520)
#define ZXENGINE_A5F1_POS16   (ZXENGINE_A5F1_POS15+1)  // �߻����Լ�����(SPN3521)
#define ZXENGINE_A5F1_POS17   (ZXENGINE_A5F1_POS16+1)  // ������Һλ������ʧЧģʽFMI
#define ZXENGINE_A5F1_POS18   (ZXENGINE_A5F1_POS17+1)  // �������¶ȴ�����ʧЧģʽFMI
#define ZXENGINE_A5F1_POS19   (ZXENGINE_A5F1_POS18+1)  // Nox������¶��״̬
#define SIZE_OF_ZXENGINE_A5F1 (ZXENGINE_A5F1_POS19+1)  // ���ֽ���(28B)

//==TAG-A5F2 DPF����(������=======================================================
#define ZXENGINE_A5F2_ADDR    (ZXENGINE_A5F1_ADDR+SIZE_OF_ZXENGINE_A5F1)                             // ��ʼ��ַ
#define zxengine_buffer_a5f2  (zxengine_buffer+ZXENGINE_A5F2_ADDR)// ��ʼ��ַ����
#define ZXENGINE_A5F2_POS1    0                       // DOC���������¶�
#define ZXENGINE_A5F2_POS2    (ZXENGINE_A5F2_POS1+2)  // DPF���������¶�
#define ZXENGINE_A5F2_POS3    (ZXENGINE_A5F2_POS2+2)  // DPF̼����������
#define ZXENGINE_A5F2_POS4    (ZXENGINE_A5F2_POS3+1)  // DPFѹ��
#define ZXENGINE_A5F2_POS5    (ZXENGINE_A5F2_POS4+2)  // �ֽ�״̬λ1
#define ZXENGINE_A5F2_POS6    (ZXENGINE_A5F2_POS5+1)  // �ֽ�״̬λ2
#define ZXENGINE_A5F2_POS7    (ZXENGINE_A5F2_POS6+1)  // �ֽ�״̬λ3
#define ZXENGINE_A5F2_POS8    (ZXENGINE_A5F2_POS7+1)  // �ֽ�״̬λ4
#define SIZE_OF_ZXENGINE_A5F2 (ZXENGINE_A5F2_POS8+1)  // ���ֽ���(11B)

// �³����������ݻ����С
#define SIZE_OF_ZXENGINE_BUFFER  (ZXENGINE_A5F2_ADDR+SIZE_OF_ZXENGINE_A5F2)

/******************************************************************************
* Macros(ͳ������Ϣ): ������ź͵�ַ����
******************************************************************************/
//==TAG-A5C5 ����Ƶ��ͳ��1========================================================
#define ZXSTATISTICS_A5C5_ADDR    0  // ��ʼ��ַ
#define zxstatistics_buffer_a5c5  (zxstatistics_buffer+ZXSTATISTICS_A5C5_ADDR)
#define ZXSTATISTICS_A5C5_POS1    0                           // ������
#define ZXSTATISTICS_A5C5_POS2    (ZXSTATISTICS_A5C5_POS1+8)  // ������
#define ZXSTATISTICS_A5C5_POS3    (ZXSTATISTICS_A5C5_POS2+8)  // �����
#define ZXSTATISTICS_A5C5_POS4    (ZXSTATISTICS_A5C5_POS3+8)  // �����
#define ZXSTATISTICS_A5C5_POS5    (ZXSTATISTICS_A5C5_POS4+8)  // �����
#define ZXSTATISTICS_A5C5_POS6    (ZXSTATISTICS_A5C5_POS5+8)  // �������
#define ZXSTATISTICS_A5C5_POS7    (ZXSTATISTICS_A5C5_POS6+8)  // ��������
#define ZXSTATISTICS_A5C5_POS8    (ZXSTATISTICS_A5C5_POS7+8)  // ��������
#define ZXSTATISTICS_A5C5_POS9    (ZXSTATISTICS_A5C5_POS8+8)  // ��������
#define ZXSTATISTICS_A5C5_POS10   (ZXSTATISTICS_A5C5_POS9+8)  // ��������
#define ZXSTATISTICS_A5C5_POS11   (ZXSTATISTICS_A5C5_POS10+8)  // ���ת
#define ZXSTATISTICS_A5C5_POS12   (ZXSTATISTICS_A5C5_POS11+8)  // �һ�ת
#define SIZE_OF_ZXSTATISTICS_A5C5 (ZXSTATISTICS_A5C5_POS12+8)  // ���ֽ���(96B)

//==TAG-A5C6 ����Ƶ��ͳ��2========================================================
#define ZXSTATISTICS_A5C6_ADDR    (ZXSTATISTICS_A5C5_ADDR+SIZE_OF_ZXSTATISTICS_A5C5)  // ��ʼ��ַ
#define zxstatistics_buffer_a5c6  (zxstatistics_buffer+ZXSTATISTICS_A5C6_ADDR)
#define ZXSTATISTICS_A5C6_POS1    0                           // �ո���
#define ZXSTATISTICS_A5C6_POS2    (ZXSTATISTICS_A5C6_POS1+8)  // �ո���
#define ZXSTATISTICS_A5C6_POS3    (ZXSTATISTICS_A5C6_POS2+8)  // ������
#define ZXSTATISTICS_A5C6_POS4    (ZXSTATISTICS_A5C6_POS3+8)  // ������
#define ZXSTATISTICS_A5C6_POS5    (ZXSTATISTICS_A5C6_POS4+8)  // �����
#define ZXSTATISTICS_A5C6_POS6    (ZXSTATISTICS_A5C6_POS5+8)  // �������
#define ZXSTATISTICS_A5C6_POS7    (ZXSTATISTICS_A5C6_POS6+8)  // ��������
#define ZXSTATISTICS_A5C6_POS8    (ZXSTATISTICS_A5C6_POS7+8)  // ��������
#define ZXSTATISTICS_A5C6_POS9    (ZXSTATISTICS_A5C6_POS8+8)  // ��������
#define ZXSTATISTICS_A5C6_POS10   (ZXSTATISTICS_A5C6_POS9+8)  // ��������
#define ZXSTATISTICS_A5C6_POS11   (ZXSTATISTICS_A5C6_POS10+8)  // ���ת
#define ZXSTATISTICS_A5C6_POS12   (ZXSTATISTICS_A5C6_POS11+8)  // �һ�ת
#define SIZE_OF_ZXSTATISTICS_A5C6 (ZXSTATISTICS_A5C6_POS12+8)  // ���ֽ���(96B)

//==TAG-A5C7 ��ȫͳ��=============================================================
#define ZXSTATISTICS_A5C7_ADDR    (ZXSTATISTICS_A5C6_ADDR+SIZE_OF_ZXSTATISTICS_A5C6)  // ��ʼ��ַ
#define zxstatistics_buffer_a5c7  (zxstatistics_buffer+ZXSTATISTICS_A5C7_ADDR)
#define ZXSTATISTICS_A5C7_POS1    0                           // ����
#define ZXSTATISTICS_A5C7_POS2    (ZXSTATISTICS_A5C7_POS1+8)  // ��Ȧ
#define ZXSTATISTICS_A5C7_POS3    (ZXSTATISTICS_A5C7_POS2+8)  // ����
#define ZXSTATISTICS_A5C7_POS4    (ZXSTATISTICS_A5C7_POS3+8)  // ��ǿ��
#define ZXSTATISTICS_A5C7_POS5    (ZXSTATISTICS_A5C7_POS4+8)  // ��װ����
#define ZXSTATISTICS_A5C7_POS6    (ZXSTATISTICS_A5C7_POS5+8)  // �����ǿ��
#define ZXSTATISTICS_A5C7_POS7    (ZXSTATISTICS_A5C7_POS6+8)  // ����ǿ��
#define ZXSTATISTICS_A5C7_POS8    (ZXSTATISTICS_A5C7_POS7+8)  // ��Ȧǿ��
#define ZXSTATISTICS_A5C7_POS9    (ZXSTATISTICS_A5C7_POS8+8)  // ���ٳ���
#define SIZE_OF_ZXSTATISTICS_A5C7 (ZXSTATISTICS_A5C7_POS9+8)  // ���ֽ���(72B)

//==TAG-301E ����ʱ��ͳ������======================================================
#define ZXSTATISTICS_301E_ADDR       (ZXSTATISTICS_A5C6_ADDR+SIZE_OF_ZXSTATISTICS_A5C6)  // ��ʼ��ַ
#define zxstatistics_buffer_301e     (zxstatistics_buffer+ZXSTATISTICS_301E_ADDR)
#define ZXSTATISTICS_301E_POS1_ADDR  0                                // ������ʱ��
#define ZXSTATISTICS_301E_POS2_ADDR  (ZXSTATISTICS_301E_POS1_ADDR+4)  // ��������ʱ��
#define SIZE_OF_ZXSTATISTICS_301E    (ZXSTATISTICS_301E_POS2_ADDR+4)  // ���ֽ���(8B)

// ͳ�����ݻ����С
#define SIZE_OF_ZXSTATISTICS_BUFFER  (ZXSTATISTICS_301E_ADDR+SIZE_OF_ZXSTATISTICS_301E)

/******************************************************************************
* Macros(�ϳ����³��汾��Ϣ): ������ź͵�ַ����
******************************************************************************/
//==TAG-A505 �ϳ�ϵͳ�汾=========================================================
#define ZXVERSION_A505_ADDR    0  // ��ʼ��ַ
#define zxversion_buffer_a505  (zxversion_buffer+ZXVERSION_A505_ADDR)
#define ZXVERSION_A505_POS1    0                        // ����������
#define ZXVERSION_A505_POS2    (ZXVERSION_A505_POS1+3)  // ��ʾ��1
#define ZXVERSION_A505_POS3    (ZXVERSION_A505_POS2+3)  // ��ʾ���ײ�汾
#define ZXVERSION_A505_POS4    (ZXVERSION_A505_POS3+3)  // GPS�ն�
#define ZXVERSION_A505_POS5    (ZXVERSION_A505_POS4+3)  // ������
#define ZXVERSION_A505_POS6    (ZXVERSION_A505_POS5+3)  // ��ʾ��2
#define ZXVERSION_A505_POS7    (ZXVERSION_A505_POS6+3)  // ��ʾ��2�ײ�
#define SIZE_OF_ZXVERSION_A505 (ZXVERSION_A505_POS7+3)  // ���ֽ���(18B)

//==TAG-A506 �³�ϵͳ�汾=========================================================
#define ZXVERSION_A506_ADDR   (ZXVERSION_A505_ADDR+SIZE_OF_ZXVERSION_A505)  // ��ʼ��ַ
#define zxversion_buffer_a506 (zxversion_buffer+ZXVERSION_A506_ADDR)
#define ZXVERSION_A506_POS1   0                          // ��ʾ��Ӧ�ò�
#define ZXVERSION_A506_POS2   (ZXVERSION_A506_POS1+3)    // ��ʾ���ײ�
#define ZXVERSION_A506_POS3   (ZXVERSION_A506_POS2+3)    // P1Ӧ�ò�
#define ZXVERSION_A506_POS4   (ZXVERSION_A506_POS3+3)    // P1�ײ�
#define ZXVERSION_A506_POS5   (ZXVERSION_A506_POS4+3)    // P2Ӧ�ò�
#define ZXVERSION_A506_POS6   (ZXVERSION_A506_POS5+3)    // P2�ײ�
#define ZXVERSION_A506_POS7   (ZXVERSION_A506_POS6+3)    // P3Ӧ�ò�
#define ZXVERSION_A506_POS8   (ZXVERSION_A506_POS7+3)    // P3�ײ�
#define ZXVERSION_A506_POS9   (ZXVERSION_A506_POS8+3)    // P4Ӧ�ò�
#define ZXVERSION_A506_POS10   (ZXVERSION_A506_POS9+3)   // P4�ײ�
#define ZXVERSION_A506_POS11   (ZXVERSION_A506_POS10+3)  // P5Ӧ�ò�
#define ZXVERSION_A506_POS12   (ZXVERSION_A506_POS11+3)  // P5�ײ�
#define ZXVERSION_A506_POS13   (ZXVERSION_A506_POS12+3)  // P6Ӧ�ò�
#define ZXVERSION_A506_POS14   (ZXVERSION_A506_POS13+3)  // P6�ײ�
#define ZXVERSION_A506_POS15   (ZXVERSION_A506_POS14+3)  // P7Ӧ�ò�
#define ZXVERSION_A506_POS16   (ZXVERSION_A506_POS15+3)  // P7�ײ�
#define ZXVERSION_A506_POS17   (ZXVERSION_A506_POS16+3)  // P8Ӧ�ò�
#define ZXVERSION_A506_POS18   (ZXVERSION_A506_POS17+3)  // P8�ײ�
#define SIZE_OF_ZXVERSION_A506 (ZXVERSION_A506_POS18+3)  // ���ֽ���(54B)

// �汾��Ϣ���ݻ����С
#define SIZE_OF_ZXVERSION_BUFFER   (ZXVERSION_A506_ADDR+SIZE_OF_ZXVERSION_A506)

/******************************************************************************
 * Data Types
 ******************************************************************************/
typedef union
{
	unsigned short word;
	struct{
		unsigned bit0 : 1;
		unsigned bit1 : 1;
		unsigned bit2 : 1;
		unsigned bit3 : 1;
		unsigned bit4 : 1;
		unsigned bit5 : 1;
		unsigned bit6 : 1;
		unsigned bit7 : 1;
		unsigned bit8 : 1;
		unsigned bit9 : 1;
		unsigned bit10 : 1;
		unsigned bit11 : 1;
		unsigned bit12 : 1;
		unsigned bit13 : 1;
		unsigned bit14 : 1;
		unsigned bit15 : 1;
	} w;
}bittype2;

// TLV��Ч��־λ
extern bittype2 zxinfo_tlv_flag;
#define tlv_a501_valid_flag    zxinfo_tlv_flag.w.bit0  //==TAG-A501ͨ��״̬��2(����ר��)
#define tlv_a504_valid_flag    zxinfo_tlv_flag.w.bit1  //==TAG-A504�ɼ�Э����Ϣ
#define tlv_a5ff_valid_flag    zxinfo_tlv_flag.w.bit2  //==TAG-A5FF�ն˲ɼ���Ϣ
//#define X    zxinfo_tlv_flag.w.bit3
//#define X    zxinfo_tlv_flag.w.bit4
//#define X    zxinfo_tlv_flag.w.bit5
//#define X    zxinfo_tlv_flag.w.bit6
//#define X    zxinfo_tlv_flag.w.bit7
//#define X    zxinfo_tlv_flag.w.bit8
//#define X    zxinfo_tlv_flag.w.bit9
//#define X    zxinfo_tlv_flag.w.bit10
//#define X    zxinfo_tlv_flag.w.bit11
//#define X    zxinfo_tlv_flag.w.bit12
//#define X    zxinfo_tlv_flag.w.bit13
//#define X    zxinfo_tlv_flag.w.bit14
//#define X    zxinfo_tlv_flag.w.bit15

// TLV��Ч��־λ
extern bittype2 zxup_tlv_flag1;
#define tlv_a5a0_valid_flag    zxup_tlv_flag1.w.bit0  //==TAG-A5A0������������Ϣ(����)
#define tlv_a5a1_valid_flag    zxup_tlv_flag1.w.bit1  //==TAG-A5A1���𹤿���Ϣ
#define tlv_a5a2_valid_flag    zxup_tlv_flag1.w.bit2  //==TAG-A5A2���۹�����Ϣ
//#define X    zxup_tlv_flag1.w.bit3
//#define X    zxup_tlv_flag1.w.bit4
#define tlv_a5a5_valid_flag    zxup_tlv_flag1.w.bit5  //==TAG-A5A5�ϳ���������Ϣ
#define tlv_a5a6_valid_flag    zxup_tlv_flag1.w.bit6  //==TAG-A5A6�ֱ���Ϣ
#define tlv_a5a7_valid_flag    zxup_tlv_flag1.w.bit7  //==TAG-A5A7��ʾ��1��Ϣ
#define tlv_a5a8_valid_flag    zxup_tlv_flag1.w.bit8  //==TAG-A5A8��ʾ��2��Ϣ
#define tlv_a5a9_valid_flag    zxup_tlv_flag1.w.bit9  //==TAG-A5A9���������Ϣ
#define tlv_a5aa_valid_flag    zxup_tlv_flag1.w.bit10 //==TAG-A5AA���߲ٿ���Ϣ
#define tlv_a5ab_valid_flag    zxup_tlv_flag1.w.bit11 //==TAG-A5AB����������Ϣ
#define tlv_a5ac_valid_flag    zxup_tlv_flag1.w.bit12 //==TAG-A5AC�����߼���Ϣ
#define tlv_a5ad_valid_flag    zxup_tlv_flag1.w.bit13 //==TAG-A5AD����߼���Ϣ
#define tlv_a5ae_valid_flag    zxup_tlv_flag1.w.bit14 //==TAG-A5AE��ת�߼���Ϣ
#define tlv_a5af_valid_flag    zxup_tlv_flag1.w.bit15 //==TAG-A5AF�������߼���Ϣ

// TLV��Ч��־λ
extern bittype2 zxup_tlv_flag2;
#define tlv_a5b0_valid_flag    zxup_tlv_flag2.w.bit0  //==TAG-A5B0�������߼���Ϣ
#define tlv_a5b1_valid_flag    zxup_tlv_flag2.w.bit1  //==TAG-A5B1�����߼���Ϣ
#define tlv_a5b2_valid_flag    zxup_tlv_flag2.w.bit2  //==TAG-A5B2�����߼���Ϣ
#define tlv_a5b3_valid_flag    zxup_tlv_flag2.w.bit3  //==TAG-A5B3Һѹ���¶�
#define tlv_a5b4_valid_flag    zxup_tlv_flag2.w.bit4  //==TAG-A5B4������Ϣ
#define tlv_a5b5_valid_flag    zxup_tlv_flag2.w.bit5  //==TAG-A5B5����1 XHVME4400P1
#define tlv_a5b6_valid_flag    zxup_tlv_flag2.w.bit6  //==TAG-A5B6����3 ���λ
#define tlv_a5b7_valid_flag    zxup_tlv_flag2.w.bit7  //==TAG-A5B7������Ϣ
#define tlv_a5b8_valid_flag    zxup_tlv_flag2.w.bit8  //==TAG-A5B8�����߼���Ϣ
#define tlv_a5b9_valid_flag    zxup_tlv_flag2.w.bit9  //==TAG-A5B9�ױ������Ʒ�
#define tlv_a5ba_valid_flag    zxup_tlv_flag2.w.bit10 //==TAG-A5BA���ƽ�ⷧ
#define tlv_a5bb_valid_flag    zxup_tlv_flag2.w.bit11 //==TAG-A5BB�����
#define tlv_a5bc_valid_flag    zxup_tlv_flag2.w.bit12 //==TAG-A5BC�������
#define tlv_a5bd_valid_flag    zxup_tlv_flag2.w.bit13 //==TAG-A5BD�����
#define tlv_a5be_valid_flag    zxup_tlv_flag2.w.bit14 //==TAG-A5BE�������
#define tlv_a5bf_valid_flag    zxup_tlv_flag2.w.bit15 //==TAG-A5BF��ת��

// TLV��Ч��־λ
extern bittype2 zxup_tlv_flag3;
#define tlv_a5c0_valid_flag    zxup_tlv_flag3.w.bit0  //==TAG-A5C0��ת�ƶ���
#define tlv_a5c1_valid_flag    zxup_tlv_flag3.w.bit1  //==TAG-A5C1������1
#define tlv_a5c2_valid_flag    zxup_tlv_flag3.w.bit2  //==TAG-A5C2������2(�����λ)
#define tlv_a5c3_valid_flag    zxup_tlv_flag3.w.bit3  //==TAG-A5C3������
#define tlv_a5c4_valid_flag    zxup_tlv_flag3.w.bit4  //==TAG-A5C4�ҳ�����
//#define X    zxup_tlv_flag3.w.bit5
//#define X    zxup_tlv_flag3.w.bit6
//#define X    zxup_tlv_flag3.w.bit7
#define tlv_a5c8_valid_flag    zxup_tlv_flag3.w.bit8  //==TAG-A5C8��ҵ�ͺ���Ϣ
#define tlv_a5c9_valid_flag    zxup_tlv_flag3.w.bit9  //==TAG-A5C9 ECU��Ӧ����CAN֡
//#define X    zxup_tlv_flag3.w.bit10
//#define X    zxup_tlv_flag3.w.bit11
//#define X    zxup_tlv_flag3.w.bit12
//#define X    zxup_tlv_flag3.w.bit13
//#define X    zxup_tlv_flag3.w.bit14
//#define X    zxup_tlv_flag3.w.bit15

// TLV��Ч��־λ
extern bittype2 zxdown_tlv_flag1;
#define tlv_a5e0_valid_flag    zxdown_tlv_flag1.w.bit0  //==TAG-A5E0�ڵ�״̬
#define tlv_a5e1_valid_flag    zxdown_tlv_flag1.w.bit1  //==TAG-A5E1����ϵͳ
#define tlv_a5e2_valid_flag    zxdown_tlv_flag1.w.bit2  //==TAG-A5E2֧�������Ϣ
#define tlv_a5e3_valid_flag    zxdown_tlv_flag1.w.bit3  //==TAG-A5E3����ϵͳ
#define tlv_a5e4_valid_flag    zxdown_tlv_flag1.w.bit4  //==TAG-A5E4ת��ϵͳ��ȫ���棩
#define tlv_a5e5_valid_flag    zxdown_tlv_flag1.w.bit5  //==TAG-A5E5ת��ϵͳ��������
#define tlv_a5e6_valid_flag    zxdown_tlv_flag1.w.bit6  //==TAG-A5E6�ƶ�ϵͳ
#define tlv_a5e7_valid_flag    zxdown_tlv_flag1.w.bit7  //==TAG-A5E7����ϵͳ
#define tlv_a5e8_valid_flag    zxdown_tlv_flag1.w.bit8  //==TAG-A5E8����ȡ��ϵͳ
#define tlv_a5e9_valid_flag    zxdown_tlv_flag1.w.bit9  //==TAG-A5E9Һѹϵͳ
#define tlv_a5ea_valid_flag    zxdown_tlv_flag1.w.bit10 //==TAG-A5EA˫��������ϵͳ
#define tlv_a5eb_valid_flag    zxdown_tlv_flag1.w.bit11 //==TAG-A5EB��̥̥ѹ
#define tlv_a5ec_valid_flag    zxdown_tlv_flag1.w.bit12 //==TAG-A5EC��֧�����
#define tlv_a5ed_valid_flag    zxdown_tlv_flag1.w.bit13 //==TAG-A5ED��֧�����
#define tlv_a5ee_valid_flag    zxdown_tlv_flag1.w.bit14 //==TAG-A5EE�п�̨������Ϣ
//#define x    zxdown_tlv_flag1.w.bit15

// TLV��Ч��־λ
extern bittype2 zxdown_tlv_flag2;
#define tlv_a5a3_valid_flag    zxdown_tlv_flag2.w.bit0  //==TAG-A5A3֧����ҵ��Ϣ
#define tlv_a5a4_valid_flag    zxdown_tlv_flag2.w.bit1  //==TAG-A5A4����֧����ҵ��Ϣ
//#define X    zxdown_tlv_flag2.w.bit2
//#define X    zxdown_tlv_flag2.w.bit3
//#define X    zxdown_tlv_flag2.w.bit4
//#define X    zxdown_tlv_flag2.w.bit5
//#define X    zxdown_tlv_flag2.w.bit6
//#define X    zxdown_tlv_flag2.w.bit7
//#define X    zxdown_tlv_flag2.w.bit8
//#define X    zxdown_tlv_flag2.w.bit9
//#define X    zxdown_tlv_flag2.w.bit10
//#define X    zxdown_tlv_flag2.w.bit11
//#define X    zxdown_tlv_flag2.w.bit12
//#define X    zxdown_tlv_flag2.w.bit13
//#define X    zxdown_tlv_flag2.w.bit14
//#define X    zxdown_tlv_flag2.w.bit15

// TLV��Ч��־λ
extern bittype2 zxengine_tlv_flag;
#define tlv_a502_valid_flag    zxengine_tlv_flag.w.bit0  //==TAG-A502������Ϣ(������������)
#define tlv_a5ef_valid_flag    zxengine_tlv_flag.w.bit1  //==TAG-A5EF���������в���(���ġ����塢����)
#define tlv_a5f0_valid_flag    zxengine_tlv_flag.w.bit2  //==TAG-A5F0��ʻ�ͺ�
#define tlv_a5f1_valid_flag    zxengine_tlv_flag.w.bit3  //==TAG-A5F1 SCR���������壩
#define tlv_a5f2_valid_flag    zxengine_tlv_flag.w.bit4  //==TAG-A5F2 DPF����(������
//#define X    zxengine_tlv_flag.w.bit5
//#define X    zxengine_tlv_flag.w.bit6
//#define X    zxengine_tlv_flag.w.bit7
//#define X    zxengine_tlv_flag.w.bit8
//#define X    zxengine_tlv_flag.w.bit9
//#define X    zxengine_tlv_flag.w.bit10
//#define X    zxengine_tlv_flag.w.bit11
//#define X    zxengine_tlv_flag.w.bit12
//#define X    zxengine_tlv_flag.w.bit13
//#define X    zxengine_tlv_flag.w.bit14
//#define X    zxengine_tlv_flag.w.bit15

// TLV��Ч��־λ
extern bittype2 zxversion_tlv_flag;
#define tlv_a505_valid_flag    zxversion_tlv_flag.w.bit0  //==TAG-A505 �ϳ�ϵͳ�汾
#define tlv_a506_valid_flag    zxversion_tlv_flag.w.bit1  //==TAG-A506 �³�ϵͳ�汾
//#define X    zxversion_tlv_flag.w.bit2
//#define X    zxversion_tlv_flag.w.bit3
//#define X    zxversion_tlv_flag.w.bit4
//#define X    zxversion_tlv_flag.w.bit5
//#define X    zxversion_tlv_flag.w.bit6
//#define X    zxversion_tlv_flag.w.bit7
//#define X    zxversion_tlv_flag.w.bit8
//#define X    zxversion_tlv_flag.w.bit9
//#define X    zxversion_tlv_flag.w.bit10
//#define X    zxversion_tlv_flag.w.bit11
//#define X    zxversion_tlv_flag.w.bit12
//#define X    zxversion_tlv_flag.w.bit13
//#define X    zxversion_tlv_flag.w.bit14
//#define X    zxversion_tlv_flag.w.bit15

// TLV��Ч��־λ
extern bittype2 zxstatistics_tlv_flag;
#define tlv_a5c5_valid_flag    zxstatistics_tlv_flag.w.bit0  //==TAG-A5C5 ����Ƶ��ͳ��1
#define tlv_a5c6_valid_flag    zxstatistics_tlv_flag.w.bit1  //==TAG-A5C6 ����Ƶ��ͳ��2
#define tlv_a5c7_valid_flag    zxstatistics_tlv_flag.w.bit2  //==TAG-A5C7 ��ȫͳ��
#define tlv_301e_valid_flag    zxstatistics_tlv_flag.w.bit3  //==TAG-301E ����ʱ��ͳ������
//#define X    zxstatistics_tlv_flag.w.bit4
//#define X    zxstatistics_tlv_flag.w.bit5
//#define X    zxstatistics_tlv_flag.w.bit6
//#define X    zxstatistics_tlv_flag.w.bit7
//#define X    zxstatistics_tlv_flag.w.bit8
//#define X    zxstatistics_tlv_flag.w.bit9
//#define X    zxstatistics_tlv_flag.w.bit10
//#define X    zxstatistics_tlv_flag.w.bit11
//#define X    zxstatistics_tlv_flag.w.bit12
//#define X    zxstatistics_tlv_flag.w.bit13
//#define X    zxstatistics_tlv_flag.w.bit14
//#define X    zxstatistics_tlv_flag.w.bit15

extern uint8_t zxinfo_buffer[SIZE_OF_ZXINFO_BUFFER]; /// ������Ϣ
extern uint8_t zxup_buffer[SIZE_OF_ZXUP_BUFFER]; /// �ϳ����ݻ���
extern uint8_t zxdown_buffer[SIZE_OF_ZXDOWN_BUFFER]; /// �³��������ݻ���
extern uint8_t zxengine_buffer[SIZE_OF_ZXENGINE_BUFFER]; /// �³����������ݻ���
extern uint8_t zxstatistics_buffer[SIZE_OF_ZXSTATISTICS_BUFFER]; ///ͳ�����ݻ���
extern uint8_t zxversion_buffer[SIZE_OF_ZXVERSION_BUFFER]; /// �汾��Ϣ����

/******************************************************************************
 * Typedef
 ******************************************************************************/
// ������������
typedef enum 
{
  ZXTC_MSG_TYPE_TCW = 0, // ������Ϣ
  ZXTC_MSG_TYPE_TCS, // �ն���Ϣ
  ZXTC_MSG_TYPE_TCB, // �汾��Ϣ
  ZXTC_MSG_TYPE_TCT, // ͳ����Ϣ
  ZXTC_MSG_TYPE_TCD, // ������Ϣ
  NUMBER_OF_ZXTC_MSG_TYPES
}zxtc_msg_type_t;

// ���͹���TCW�ṹ��
#define MAX_NUM_OF_ZXUP_TLV     50
#define MAX_NUM_OF_ZXDOWN_TLV   30
#define MAX_NUM_OF_ZXENGINE_TLV 10
typedef struct
{
  uint8_t pid_vehicle;  // ��������
  uint8_t pid_up;  // �ϳ�CANЭ��
  uint8_t pid_down;  // �³�CANЭ��
  uint8_t valid_tlv_num;  // ��Ч��TLV������

  uint8_t zxup_tlv_num;  // �ϳ�TLV����
  uint16_t zxup_tlv_table[MAX_NUM_OF_ZXUP_TLV];  // �ϳ�TLV��

  uint8_t zxdown_tlv_num;  // �³�TLV����
  uint16_t zxdown_tlv_table[MAX_NUM_OF_ZXDOWN_TLV];  // �³�TLV��
}zxtcw_context_t;
extern zxtcw_context_t zxtcw_context;

/******************************************************************************
 *   Function prototypes
 ******************************************************************************/
uint16_t iZxM2m_AnalyzeTlvMsg_A510(uint8_t* pValue, uint16_t len); //==����ר��:������
uint16_t iZxM2m_AnalyzeTlvMsg_A511(uint8_t* pValue, uint16_t len); //==����ר��:���������
uint16_t iZxM2m_AnalyzeTlvMsg_A512(uint8_t* pValue, uint16_t len); //==����ר��:����Э������
uint16_t iZxM2m_AnalyzeTlvMsg_A513(uint8_t* pValue, uint16_t len); //==����ר��:VIN������

#endif  /* _ZX_M2M_PROTOCOL_H_ */

