/*****************************************************************************
* @FileName: tcw.h
* @Engineer: TenYan
* @Company:  �칤��Ϣ����Ӳ����
* @version   V1.0
* @Date:     2020-11-20
* @brief     �칤�������ػ�canͨ��Э�鶨��
******************************************************************************/
#ifndef TCW_H_
#define TCW_H_

/******************************************************************************
* Includes
******************************************************************************/

/******************************************************************************
* Macros(����������Ϣ): ��ź͵�ַ����
******************************************************************************/
//==TAG-A501ͨ��״̬��2(����ר��)=================================================

//==TAG-A502������Ϣ(������������)================================================

//==TAG-A504�ɼ�Э����Ϣ==========================================================

/******************************************************************************
* Macros(�ϳ�ͨ��): ���ػ��ϳ�������ź͵�ַ����
******************************************************************************/
//==TAG-A5A0������������Ϣ(����)==================================================
#define zxup_buffer_a5a0              (zxup_buffer)    // ��ʼ��ַ����
#define ZXUP_A5A0_POS1_ADDR    0                                       // ����������1-2
#define ZXUP_A5A0_POS2_ADDR    (ZXUP_A5A0_POS1_ADDR+2)  // ��������(ABCDEFGH)
#define ZXUP_A5A0_POS3_ADDR    (ZXUP_A5A0_POS2_ADDR+4)  // ����
#define ZXUP_A5A0_POS4_ADDR    (ZXUP_A5A0_POS3_ADDR+1)  // ����ͷ���Ƕ�
#define ZXUP_A5A0_POS5_ADDR    (ZXUP_A5A0_POS4_ADDR+2)  // ���۸����Ƕ�
#define ZXUP_A5A0_POS6_ADDR    (ZXUP_A5A0_POS5_ADDR+2)  // ���۳���
#define ZXUP_A5A0_POS7_ADDR    (ZXUP_A5A0_POS6_ADDR+2)  // ���ڱ۳���
#define ZXUP_A5A0_POS8_ADDR    (ZXUP_A5A0_POS7_ADDR+1)  // �ڱ۰ٷֱ�
#define ZXUP_A5A0_POS9_ADDR    (ZXUP_A5A0_POS8_ADDR+8)  // �ױ�������
#define ZXUP_A5A0_POS10_ADDR   (ZXUP_A5A0_POS9_ADDR+1)  // ���۳���
#define ZXUP_A5A0_POS11_ADDR   (ZXUP_A5A0_POS10_ADDR+2)  // ���۽Ƕ�
#define ZXUP_A5A0_POS12_ADDR   (ZXUP_A5A0_POS11_ADDR+2)  // ����/���۸����Ƕ�
#define ZXUP_A5A0_POS13_ADDR   (ZXUP_A5A0_POS12_ADDR+2)  // ����/����ͷ���Ƕ�
#define ZXUP_A5A0_POS14_ADDR   (ZXUP_A5A0_POS13_ADDR+2)  // ǻѹ��
#define ZXUP_A5A0_POS15_ADDR   (ZXUP_A5A0_POS14_ADDR+8)  // ��ͷ�߶�
#define ZXUP_A5A0_POS16_ADDR   (ZXUP_A5A0_POS15_ADDR+2)  // �����
#define ZXUP_A5A0_POS17_ADDR   (ZXUP_A5A0_POS16_ADDR+2)  // ʵ������
#define ZXUP_A5A0_POS18_ADDR   (ZXUP_A5A0_POS17_ADDR+2)  // ���ذٷֱ�
#define ZXUP_A5A0_POS19_ADDR   (ZXUP_A5A0_POS18_ADDR+2)  // ��������
#define ZXUP_A5A0_POS20_ADDR   (ZXUP_A5A0_POS19_ADDR+2)  // LMI���ϴ���1-4
#define ZXUP_A5A0_POS21_ADDR   (ZXUP_A5A0_POS20_ADDR+8)  // LMI���ϴ���5-8
#define ZXUP_A5A0_POS22_ADDR   (ZXUP_A5A0_POS21_ADDR+8)  // �ǶԳƹ��ϴ���
#define ZXUP_A5A0_POS23_ADDR   (ZXUP_A5A0_POS22_ADDR+4)  // LMI����ʱ��
#define ZXUP_A5A0_POS24_ADDR   (ZXUP_A5A0_POS23_ADDR+4)  // ǿ��λ
#define ZXUP_A5A0_POS25_ADDR   (ZXUP_A5A0_POS24_ADDR+1)  // ˮƽ��X
#define ZXUP_A5A0_POS26_ADDR   (ZXUP_A5A0_POS25_ADDR+2)  // ˮƽ��Y
#define ZXUP_A5A0_POS27_ADDR   (ZXUP_A5A0_POS26_ADDR+2)  // ����
#define ZXUP_A5A0_POS28_ADDR   (ZXUP_A5A0_POS27_ADDR+2)  // ��ת�Ƕ�
#define SIZE_OF_ZXUP_A5A0         (ZXUP_A5A0_POS28_ADDR+2) // ���ֽ���(82B)

//==TAG-A5A1���𹤿���Ϣ==========================================================
#define zxup_buffer_a5a1       (zxup_buffer_a5a0+SIZE_OF_ZXUP_A5A0) // ��ʼ��ַ����
#define ZXUP_A5A1_POS1_ADDR    0                        // �������Ƕ�
#define ZXUP_A5A1_POS2_ADDR    (ZXUP_A5A1_POS1_ADDR+2)  // ��������
#define ZXUP_A5A1_POS3_ADDR    (ZXUP_A5A1_POS2_ADDR+2)  // ����չ���Ƕ�
#define ZXUP_A5A1_POS4_ADDR    (ZXUP_A5A1_POS3_ADDR+2)  // �������Ƕ�
#define ZXUP_A5A1_POS5_ADDR    (ZXUP_A5A1_POS4_ADDR+2)  // �ҳ������Ƕ�
#define ZXUP_A5A1_POS6_ADDR    (ZXUP_A5A1_POS5_ADDR+2)  // �ҳ�������
#define ZXUP_A5A1_POS7_ADDR    (ZXUP_A5A1_POS6_ADDR+2)  // �ҳ���չ���Ƕ�
#define ZXUP_A5A1_POS8_ADDR    (ZXUP_A5A1_POS7_ADDR+2)  // �ҳ������Ƕ�
#define ZXUP_A5A1_POS9_ADDR    (ZXUP_A5A1_POS8_ADDR+2)  // ����-��ֹ�ͽ���
#define ZXUP_A5A1_POS10_ADDR   (ZXUP_A5A1_POS9_ADDR+1)  // �ҳ���-��ֹ�ͽ���
#define ZXUP_A5A1_POS11_ADDR   (ZXUP_A5A1_POS10_ADDR+1)  // �������
#define ZXUP_A5A1_POS12_ADDR   (ZXUP_A5A1_POS11_ADDR+1)  // �ҳ������
#define ZXUP_A5A1_POS13_ADDR   (ZXUP_A5A1_POS12_ADDR+1)  // �����Ž��׳���
#define ZXUP_A5A1_POS14_ADDR   (ZXUP_A5A1_POS13_ADDR+2)  // �ҳ����Ž��׳���
#define SIZE_OF_ZXUP_A5A1         (ZXUP_A5A1_POS14_ADDR+2)  // ���ֽ���(24B)

//==TAG-A5A2���۹�����Ϣ==========================================================
#define zxup_buffer_a5a2       (zxup_buffer_a5a1+SIZE_OF_ZXUP_A5A1)// ��ʼ��ַ����
#define ZXUP_A5A2_POS1_ADDR    0                        // ����������
#define ZXUP_A5A2_POS2_ADDR    (ZXUP_A5A2_POS1_ADDR+2)  // ����������
#define ZXUP_A5A2_POS3_ADDR    (ZXUP_A5A2_POS2_ADDR+2)  // ������ѹ��
#define ZXUP_A5A2_POS4_ADDR    (ZXUP_A5A2_POS3_ADDR+2)  // ǰ֧�ܽǶ�
#define ZXUP_A5A2_POS5_ADDR    (ZXUP_A5A2_POS4_ADDR+2)  // ����֧�ܼ��
#define SIZE_OF_ZXUP_A5A2      (ZXUP_A5A2_POS5_ADDR+1)  // ���ֽ���(9B)

//==TAG-A5A5�ϳ���������Ϣ========================================================
#define zxup_buffer_a5a5       (zxup_buffer_a5a2+SIZE_OF_ZXUP_A5A2)// ��ʼ��ַ����
#define ZXUP_A5A5_POS1_ADDR    0                        // ������ת��
#define ZXUP_A5A5_POS2_ADDR    (ZXUP_A5A5_POS1_ADDR+2)  // ʵ��Ť�ذٷֱ�
#define ZXUP_A5A5_POS3_ADDR    (ZXUP_A5A5_POS2_ADDR+1)  // Ħ��Ť�ذٷֱ�
#define ZXUP_A5A5_POS4_ADDR    (ZXUP_A5A5_POS3_ADDR+1)  // �������ѹ��
#define ZXUP_A5A5_POS5_ADDR    (ZXUP_A5A5_POS4_ADDR+1)  // ��������¶�
#define ZXUP_A5A5_POS6_ADDR    (ZXUP_A5A5_POS5_ADDR+1)  // ��ȴҺ�¶�
#define ZXUP_A5A5_POS7_ADDR    (ZXUP_A5A5_POS6_ADDR+1)  // �����¶�
#define ZXUP_A5A5_POS8_ADDR    (ZXUP_A5A5_POS7_ADDR+2)  // ����Һλ
#define ZXUP_A5A5_POS9_ADDR    (ZXUP_A5A5_POS8_ADDR+1)  // ����ѹ��
#define ZXUP_A5A5_POS10_ADDR   (ZXUP_A5A5_POS9_ADDR+1)  // ������������ʱ��
#define ZXUP_A5A5_POS11_ADDR   (ZXUP_A5A5_POS10_ADDR+4)  // ������ˮָʾ��
#define ZXUP_A5A5_POS12_ADDR   (ZXUP_A5A5_POS11_ADDR+1)  // ����̤��ٷֱ�
#define ZXUP_A5A5_POS13_ADDR   (ZXUP_A5A5_POS12_ADDR+1)  // ������ȼ��������
#define ZXUP_A5A5_POS14_ADDR   (ZXUP_A5A5_POS13_ADDR+2)  // ������ƽ��ȼ��������
#define ZXUP_A5A5_POS15_ADDR   (ZXUP_A5A5_POS14_ADDR+2)  // ȼ��Һλ
#define ZXUP_A5A5_POS16_ADDR   (ZXUP_A5A5_POS15_ADDR+1)  // ������Ť��ģʽ(����������)
#define SIZE_OF_ZXUP_A5A5      (ZXUP_A5A5_POS16_ADDR+1)  // ���ֽ���(23B)

//==TAG-A5A6�ֱ���Ϣ==============================================================
#define zxup_buffer_a5a6       (zxup_buffer_a5a5+SIZE_OF_ZXUP_A5A5)// ��ʼ��ַ����
#define ZXUP_A5A6_POS1_ADDR    0                        // ���ֱ�״̬λ
#define ZXUP_A5A6_POS2_ADDR    (ZXUP_A5A6_POS1_ADDR+1)  // ���ֱ�X���
#define ZXUP_A5A6_POS3_ADDR    (ZXUP_A5A6_POS2_ADDR+2)  // ���ֱ�Y���
#define ZXUP_A5A6_POS4_ADDR    (ZXUP_A5A6_POS3_ADDR+2)  // ���ֱ�״̬λ
#define ZXUP_A5A6_POS5_ADDR    (ZXUP_A5A6_POS4_ADDR+1)  // ���ֱ�X���
#define ZXUP_A5A6_POS6_ADDR    (ZXUP_A5A6_POS5_ADDR+2)  // ���ֱ�Y���
#define SIZE_OF_ZXUP_A5A6      (ZXUP_A5A6_POS6_ADDR+2)  // ���ֽ���(10B)

//==TAG-A5A7��ʾ��1��Ϣ===========================================================
#define zxup_buffer_a5a7       (zxup_buffer_a5a6+SIZE_OF_ZXUP_A5A6)// ��ʼ��ַ����
#define ZXUP_A5A7_POS1_ADDR    0                        // ��Ʒ����
#define ZXUP_A5A7_POS2_ADDR    (ZXUP_A5A7_POS1_ADDR+4)  // ��������
#define ZXUP_A5A7_POS3_ADDR    (ZXUP_A5A7_POS2_ADDR+4)  // �������ٶ�
#define ZXUP_A5A7_POS4_ADDR    (ZXUP_A5A7_POS3_ADDR+1)  // �������ٶ�
#define ZXUP_A5A7_POS5_ADDR    (ZXUP_A5A7_POS4_ADDR+1)  // �������ٶ�
#define ZXUP_A5A7_POS6_ADDR    (ZXUP_A5A7_POS5_ADDR+1)  // �������ٶ�
#define ZXUP_A5A7_POS7_ADDR    (ZXUP_A5A7_POS6_ADDR+1)  // ������ٶ�
#define ZXUP_A5A7_POS8_ADDR    (ZXUP_A5A7_POS7_ADDR+1)  // ������ٶ�
#define ZXUP_A5A7_POS9_ADDR    (ZXUP_A5A7_POS8_ADDR+1)  // ���ת�ٶ�
#define ZXUP_A5A7_POS10_ADDR   (ZXUP_A5A7_POS9_ADDR+1)  // �һ�ת�ٶ�
#define ZXUP_A5A7_POS11_ADDR   (ZXUP_A5A7_POS10_ADDR+1)  // Ŀ�����
#define ZXUP_A5A7_POS12_ADDR   (ZXUP_A5A7_POS11_ADDR+1)  // ״̬λ1
#define ZXUP_A5A7_POS13_ADDR   (ZXUP_A5A7_POS12_ADDR+1)  // ״̬λ2
#define ZXUP_A5A7_POS14_ADDR   (ZXUP_A5A7_POS13_ADDR+1)  // ����ģʽ
#define SIZE_OF_ZXUP_A5A7      (ZXUP_A5A7_POS14_ADDR+1)  // ���ֽ���(20B)

//==TAG-A5A8��ʾ��2��Ϣ===========================================================
#define zxup_buffer_a5a8       (zxup_buffer_a5a7+SIZE_OF_ZXUP_A5A7)// ��ʼ��ַ����
#define ZXUP_A5A8_POS1_ADDR    0                        // ����ٿ�
#define ZXUP_A5A8_POS2_ADDR    (ZXUP_A5A8_POS1_ADDR+8)  // ����ά��
#define SIZE_OF_ZXUP_A5A8      (ZXUP_A5A8_POS2_ADDR+2)  // ���ֽ���(10B)

//==TAG-A5A9���������Ϣ==========================================================
#define zxup_buffer_a5a9       (zxup_buffer_a5a8+SIZE_OF_ZXUP_A5A8)// ��ʼ��ַ����
#define ZXUP_A5A9_POS1_ADDR    0                        // ���1
#define ZXUP_A5A9_POS2_ADDR    (ZXUP_A5A9_POS1_ADDR+2)  // ���2
#define ZXUP_A5A9_POS3_ADDR    (ZXUP_A5A9_POS2_ADDR+2)  // ���3
#define SIZE_OF_ZXUP_A5A9      (ZXUP_A5A9_POS3_ADDR+2)  // ���ֽ���(6B)

//==TAG-A5AA���߲ٿ���Ϣ==========================================================
#define zxup_buffer_a5aa       (zxup_buffer_a5a9+SIZE_OF_ZXUP_A5A9)// ��ʼ��ַ����
#define ZXUP_A5AA_POS1_ADDR    0                        // Msg1
#define ZXUP_A5AA_POS2_ADDR    (ZXUP_A5AA_POS1_ADDR+8)  // Msg2
#define ZXUP_A5AA_POS3_ADDR    (ZXUP_A5AA_POS2_ADDR+8)  // Msg3
#define ZXUP_A5AA_POS4_ADDR    (ZXUP_A5AA_POS2_ADDR+8)  // Msg4
#define SIZE_OF_ZXUP_A5AA      (ZXUP_A5AA_POS4_ADDR+8)  // ���ֽ���(32B)

//==TAG-A5AB����������Ϣ==========================================================
#define zxup_buffer_a5ab       (zxup_buffer_a5aa+SIZE_OF_ZXUP_A5AA)// ��ʼ��ַ����
#define ZXUP_A5AB_POS1_ADDR    0                        // �ڵ�״̬
#define SIZE_OF_ZXUP_A5AB      (ZXUP_A5AB_POS1_ADDR+8)  // ���ֽ���(8B)

//==TAG-A5AC�����߼���Ϣ==========================================================
#define zxup_buffer_a5ac       (zxup_buffer_a5ab+SIZE_OF_ZXUP_A5AB)// ��ʼ��ַ����
#define ZXUP_A5AC_POS1_ADDR    0                        // �������ƺͽ��
#define SIZE_OF_ZXUP_A5AC      (ZXUP_A5AC_POS1_ADDR+6)  // ���ֽ���(6B)

//==TAG-A5AD����߼���Ϣ==========================================================
#define zxup_buffer_a5ad       (zxup_buffer_a5ac+SIZE_OF_ZXUP_A5AC)// ��ʼ��ַ����
#define ZXUP_A5AD_POS1_ADDR    0                        // �������ƺͽ��
#define SIZE_OF_ZXUP_A5AD      (ZXUP_A5AD_POS1_ADDR+8)  // ���ֽ���(8B)

//==TAG-A5AE��ת�߼���Ϣ==========================================================
#define zxup_buffer_a5ae       (zxup_buffer_a5ad+SIZE_OF_ZXUP_A5AD)// ��ʼ��ַ����
#define ZXUP_A5AE_POS1_ADDR    0                        // �������1
#define ZXUP_A5AE_POS2_ADDR    (ZXUP_A5AE_POS1_ADDR+1)  // �������2
#define ZXUP_A5AE_POS3_ADDR    (ZXUP_A5AE_POS2_ADDR+1)  // ��ؽ��
#define ZXUP_A5AE_POS4_ADDR    (ZXUP_A5AE_POS3_ADDR+1)  // �һ�����1
#define ZXUP_A5AE_POS5_ADDR    (ZXUP_A5AE_POS4_ADDR+1)  // �һ�����2
#define ZXUP_A5AE_POS6_ADDR    (ZXUP_A5AE_POS5_ADDR+1)  // �һؽ��
#define SIZE_OF_ZXUP_A5AE      (ZXUP_A5AE_POS6_ADDR+1)  // ���ֽ���(6B)

//==TAG-A5AF�������߼���Ϣ========================================================
#define zxup_buffer_a5af       (zxup_buffer_a5ae+SIZE_OF_ZXUP_A5AE)// ��ʼ��ַ����
#define ZXUP_A5AF_POS1_ADDR    0                        // �������������ƺͽ��
#define SIZE_OF_ZXUP_A5AF      (ZXUP_A5AF_POS1_ADDR+5)  // ���ֽ���(5B)

//==TAG-A5B0�������߼���Ϣ========================================================
#define zxup_buffer_a5b0       (zxup_buffer_a5af+SIZE_OF_ZXUP_A5AF)// ��ʼ��ַ����
#define ZXUP_A5B0_POS1_ADDR    0                        // �������������ƺͽ��
#define SIZE_OF_ZXUP_A5B0      (ZXUP_A5B0_POS1_ADDR+4)  // ���ֽ���(4B)

//==TAG-A5B1�����߼���Ϣ==========================================================
#define zxup_buffer_a5b1       (zxup_buffer_a5b0+SIZE_OF_ZXUP_A5B0)// ��ʼ��ַ����
#define ZXUP_A5B1_POS1_ADDR    0                        // ����Ŀ��չ���Ƕ�
#define ZXUP_A5B1_POS2_ADDR    (ZXUP_A5B1_POS1_ADDR+1)  // ����Ŀ���Ž��Ƕ�
#define ZXUP_A5B1_POS3_ADDR    (ZXUP_A5B1_POS2_ADDR+2)  // ����һ���Ž������߼�
#define ZXUP_A5B1_POS4_ADDR    (ZXUP_A5B1_POS2_ADDR+2)  // �Ž�����״̬λ
#define SIZE_OF_ZXUP_A5B1      (ZXUP_A5B1_POS4_ADDR+1)  // ���ֽ���(6B)

//==TAG-A5B2�����߼���Ϣ==========================================================
#define zxup_buffer_a5b2       (zxup_buffer_a5b1+SIZE_OF_ZXUP_A5B1)// ��ʼ��ַ����
#define ZXUP_A5B2_POS1_ADDR    0                        // ����������
#define ZXUP_A5B2_POS2_ADDR    (ZXUP_A5B2_POS1_ADDR+2)  // ����������
#define ZXUP_A5B2_POS3_ADDR    (ZXUP_A5B2_POS2_ADDR+2)  // �����������
#define ZXUP_A5B2_POS4_ADDR    (ZXUP_A5B2_POS3_ADDR+1)  // ������������
#define ZXUP_A5B2_POS5_ADDR    (ZXUP_A5B2_POS4_ADDR+1)  // �������Ʊ����
#define ZXUP_A5B2_POS6_ADDR    (ZXUP_A5B2_POS5_ADDR+1)  // �������Ʊ����
#define ZXUP_A5B2_POS7_ADDR    (ZXUP_A5B2_POS6_ADDR+1)  // ��������������
#define ZXUP_A5B2_POS8_ADDR    (ZXUP_A5B2_POS7_ADDR+1)  // ��������������
#define SIZE_OF_ZXUP_A5B2      (ZXUP_A5B2_POS8_ADDR+1)  // ���ֽ���(10B)

//==TAG-A5B3Һѹ���¶�============================================================
#define zxup_buffer_a5b3       (zxup_buffer_a5b2+SIZE_OF_ZXUP_A5B2)// ��ʼ��ַ����
#define ZXUP_A5B3_POS1_ADDR    0                        // Һѹ���¶�
#define ZXUP_A5B3_POS2_ADDR    (ZXUP_A5B3_POS1_ADDR+2)  // Һѹ��λ
#define ZXUP_A5B3_POS3_ADDR    (ZXUP_A5B3_POS2_ADDR+1)  // ������������
#define SIZE_OF_ZXUP_A5B3      (ZXUP_A5B3_POS3_ADDR+1)  // ���ֽ���(4B)

//==TAG-A5B4������Ϣ==============================================================
#define zxup_buffer_a5b4       (zxup_buffer_a5b3+SIZE_OF_ZXUP_A5B3)// ��ʼ��ַ����
#define ZXUP_A5B4_POS1_ADDR    0                        // 1#���õ�ŷ�
#define ZXUP_A5B4_POS2_ADDR    (ZXUP_A5B4_POS1_ADDR+2)  // 2#���õ�ŷ�
#define ZXUP_A5B4_POS3_ADDR    (ZXUP_A5B4_POS2_ADDR+2)  // ����ѹ��
#define SIZE_OF_ZXUP_A5B4      (ZXUP_A5B4_POS3_ADDR+2)  // ���ֽ���(6B)

//==TAG-A5B5����1 XHVME4400P1=====================================================
#define zxup_buffer_a5b5       (zxup_buffer_a5b4+SIZE_OF_ZXUP_A5B4)// ��ʼ��ַ����
#define ZXUP_A5B5_POS1_ADDR    0                        // ���ŷ�
#define ZXUP_A5B5_POS2_ADDR    (ZXUP_A5B5_POS1_ADDR+2)  // ����ŷ�
#define ZXUP_A5B5_POS3_ADDR    (ZXUP_A5B5_POS2_ADDR+2)  // ������ŷ�
#define ZXUP_A5B5_POS4_ADDR    (ZXUP_A5B5_POS3_ADDR+2)  // ������ŷ�
#define ZXUP_A5B5_POS5_ADDR    (ZXUP_A5B5_POS4_ADDR+2)  // �������ŷ�
#define ZXUP_A5B5_POS6_ADDR    (ZXUP_A5B5_POS5_ADDR+2)  // �������ŷ�
#define ZXUP_A5B5_POS7_ADDR    (ZXUP_A5B5_POS6_ADDR+2)  // �������ŷ�
#define ZXUP_A5B5_POS8_ADDR    (ZXUP_A5B5_POS7_ADDR+2)  //�������ŷ�
#define ZXUP_A5B5_POS9_ADDR    (ZXUP_A5B5_POS8_ADDR+2)  // MP1ѹ��
#define ZXUP_A5B5_POS10_ADDR   (ZXUP_A5B5_POS9_ADDR+2)  // LS1ѹ��
#define ZXUP_A5B5_POS11_ADDR   (ZXUP_A5B5_POS10_ADDR+2)  // MP2ѹ��
#define ZXUP_A5B5_POS12_ADDR   (ZXUP_A5B5_POS11_ADDR+2)  // LS2ѹ��
#define ZXUP_A5B5_POS13_ADDR   (ZXUP_A5B5_POS12_ADDR+2)  // ������ŷ�
#define SIZE_OF_ZXUP_A5B5      (ZXUP_A5B5_POS13_ADDR+1)  // ���ֽ���(23B)

//==TAG-A5B6����3 ���λ==========================================================
#define zxup_buffer_a5b6       (zxup_buffer_a5b5+SIZE_OF_ZXUP_A5B5)// ��ʼ��ַ����
#define ZXUP_A5B6_POS1_ADDR    0                        // ״̬λ
#define SIZE_OF_ZXUP_A5B6      (ZXUP_A5B6_POS1_ADDR+1)  // ���ֽ���(1B)

//==TAG-A5B7������Ϣ==============================================================
#define zxup_buffer_a5b7       (zxup_buffer_a5b6+SIZE_OF_ZXUP_A5B6)// ��ʼ��ַ����
#define ZXUP_A5B7_POS1_ADDR    0                        // ������ѹ��
#define ZXUP_A5B7_POS2_ADDR    (ZXUP_A5B7_POS1_ADDR+2)  // �����׳���
#define ZXUP_A5B7_POS3_ADDR    (ZXUP_A5B7_POS2_ADDR+2)  // ���ƽ�ⷧ
#define SIZE_OF_ZXUP_A5B7      (ZXUP_A5B7_POS3_ADDR+2)  // ���ֽ���(6B)

//==TAG-A5B8�����߼���Ϣ==========================================================
#define zxup_buffer_a5b8       (zxup_buffer_a5b7+SIZE_OF_ZXUP_A5B7)// ��ʼ��ַ����
#define ZXUP_A5B8_POS1_ADDR    0                        // �������ͽ���״̬λ
#define ZXUP_A5B8_POS2_ADDR    (ZXUP_A5B8_POS1_ADDR+1)  // ͷ���־
#define ZXUP_A5B8_POS3_ADDR    (ZXUP_A5B8_POS2_ADDR+1)  // ǰ��ͷ���⿪��8
#define ZXUP_A5B8_POS4_ADDR    (ZXUP_A5B8_POS3_ADDR+1)  // ���ͷ���⿪��8
#define ZXUP_A5B8_POS5_ADDR    (ZXUP_A5B8_POS4_ADDR+1)  // ��λ
#define SIZE_OF_ZXUP_A5B8      (ZXUP_A5B8_POS5_ADDR+1)  // ���ֽ���(5B)

//==TAG-A5B9�ױ������Ʒ�==========================================================
#define zxup_buffer_a5b9       (zxup_buffer_a5b8+SIZE_OF_ZXUP_A5B8)// ��ʼ��ַ����
#define ZXUP_A5B9_POS1_ADDR    0                        // ״̬�ֽ�
#define ZXUP_A5B9_POS2_ADDR    (ZXUP_A5B9_POS1_ADDR+1)  // ������ѹ��
#define SIZE_OF_ZXUP_A5B9      (ZXUP_A5B9_POS2_ADDR+2)  // ���ֽ���(3B)

//==TAG-A5BA���ƽ�ⷧ============================================================
#define zxup_buffer_a5ba       (zxup_buffer_a5b9+SIZE_OF_ZXUP_A5B9)// ��ʼ��ַ����
#define ZXUP_A5BA_POS1_ADDR    0                        // ����ƽ�ⷧ����
#define ZXUP_A5BA_POS2_ADDR    (ZXUP_A5BA_POS1_ADDR+1)  // �ұ��ƽ�ⷧ����
#define SIZE_OF_ZXUP_A5BA      (ZXUP_A5BA_POS2_ADDR+2)  // ���ֽ���(4B)

//==TAG-A5BB�����================================================================
#define zxup_buffer_a5bb       (zxup_buffer_a5ba+SIZE_OF_ZXUP_A5BA)// ��ʼ��ַ����
#define ZXUP_A5BB_POS1_ADDR    0                        // ������ŷ�
#define ZXUP_A5BB_POS2_ADDR    (ZXUP_A5BB_POS1_ADDR+2)  // �����ŷ�
#define ZXUP_A5BB_POS3_ADDR    (ZXUP_A5BB_POS2_ADDR+2)  // �ͱ�ѹ��
#define SIZE_OF_ZXUP_A5BB      (ZXUP_A5BB_POS3_ADDR+2)  // ���ֽ���(6B)

//==TAG-A5BC�������==============================================================
#define zxup_buffer_a5bc       (zxup_buffer_a5bb+SIZE_OF_ZXUP_A5BB)// ��ʼ��ַ����
#define ZXUP_A5BC_POS1_ADDR    0                        // ������
#define ZXUP_A5BC_POS2_ADDR    (ZXUP_A5BC_POS1_ADDR+2)  // ��Ͳת��
#define ZXUP_A5BC_POS3_ADDR    (ZXUP_A5BC_POS2_ADDR+2)  // ״̬λ
#define SIZE_OF_ZXUP_A5BC      (ZXUP_A5BC_POS3_ADDR+1)  // ���ֽ���(5B)

//==TAG-A5BD�����================================================================
#define zxup_buffer_a5bd       (zxup_buffer_a5bc+SIZE_OF_ZXUP_A5BC)// ��ʼ��ַ����
#define ZXUP_A5BD_POS1_ADDR    0                        // ������ŷ�
#define ZXUP_A5BD_POS2_ADDR    (ZXUP_A5BD_POS1_ADDR+2)  // �����ŷ�
#define ZXUP_A5BD_POS3_ADDR    (ZXUP_A5BD_POS2_ADDR+2)  // �ͱ�ѹ��
#define SIZE_OF_ZXUP_A5BD      (ZXUP_A5BD_POS3_ADDR+2)  // ���ֽ���(6B)

//==TAG-A5BE�������==============================================================
#define zxup_buffer_a5be       (zxup_buffer_a5bd+SIZE_OF_ZXUP_A5BD)// ��ʼ��ַ����
#define ZXUP_A5BE_POS1_ADDR    0                        // ������
#define ZXUP_A5BE_POS2_ADDR    (ZXUP_A5BE_POS1_ADDR+2)  // ��Ͳת��
#define ZXUP_A5BE_POS3_ADDR    (ZXUP_A5BE_POS2_ADDR+2)  // ״̬λ
#define SIZE_OF_ZXUP_A5BE      (ZXUP_A5BE_POS3_ADDR+1)  // ���ֽ���(5B)

//==TAG-A5BF��ת��================================================================
#define zxup_buffer_a5bf       (zxup_buffer_a5be+SIZE_OF_ZXUP_A5BE)// ��ʼ��ַ����
#define ZXUP_A5BF_POS1_ADDR    0                        // ���ת��ŷ�
#define ZXUP_A5BF_POS2_ADDR    (ZXUP_A5BF_POS1_ADDR+2)  // �һ�ת��ŷ�
#define ZXUP_A5BF_POS3_ADDR    (ZXUP_A5BF_POS2_ADDR+2)  // ��ת���巧
#define ZXUP_A5BF_POS4_ADDR    (ZXUP_A5BF_POS3_ADDR+2)  // �ͱ�ѹ������תѹ����⣩
#define SIZE_OF_ZXUP_A5BF      (ZXUP_A5BF_POS4_ADDR+2)  // ���ֽ���(8B)

//==TAG-A5C0��ת�ƶ���============================================================
#define zxup_buffer_a5c0       (zxup_buffer_a5bf+SIZE_OF_ZXUP_A5BF)// ��ʼ��ַ����
#define ZXUP_A5C0_POS1_ADDR    0                        // �ֽ�״̬λ
#define ZXUP_A5C0_POS2_ADDR    (ZXUP_A5C0_POS1_ADDR+1)  // ��Ͳת��
#define SIZE_OF_ZXUP_A5C0      (ZXUP_A5C0_POS2_ADDR+2)  // ���ֽ���(3B)

//==TAG-A5C1������1===============================================================
#define zxup_buffer_a5c1       (zxup_buffer_a5c0+SIZE_OF_ZXUP_A5C0)// ��ʼ��ַ����
#define ZXUP_A5C1_POS1_ADDR    0                        // �ֽ�״̬λ1
#define ZXUP_A5C1_POS2_ADDR    (ZXUP_A5C1_POS1_ADDR+1)  // �ֽ�״̬λ2
#define SIZE_OF_ZXUP_A5C1      (ZXUP_A5C1_POS2_ADDR+1)  // ���ֽ���(2B)

//==TAG-A5C2������2(�����λ)=====================================================
#define zxup_buffer_a5c2       (zxup_buffer_a5c1+SIZE_OF_ZXUP_A5C1)// ��ʼ��ַ����
#define ZXUP_A5C2_POS1_ADDR    0                        // ѹ��ѡ��
#define ZXUP_A5C2_POS2_ADDR    (ZXUP_A5C2_POS1_ADDR+2)  // �ֽ�״̬λ
#define SIZE_OF_ZXUP_A5C2      (ZXUP_A5C2_POS2_ADDR+1)  // ���ֽ���(3B)

//==TAG-A5C3������============================================================
#define zxup_buffer_a5c3       (zxup_buffer_a5c2+SIZE_OF_ZXUP_A5C2)// ��ʼ��ַ����
#define ZXUP_A5C3_POS1_ADDR    0                        // ������
#define ZXUP_A5C3_POS2_ADDR    (ZXUP_A5C3_POS1_ADDR+2)  // ������
#define ZXUP_A5C3_POS3_ADDR    (ZXUP_A5C3_POS2_ADDR+2)  // ����������
#define ZXUP_A5C3_POS4_ADDR    (ZXUP_A5C3_POS3_ADDR+2)  // �ֽ�״̬λ1
#define ZXUP_A5C3_POS5_ADDR    (ZXUP_A5C3_POS4_ADDR+1)  // �ֽ�״̬λ2
#define ZXUP_A5C3_POS6_ADDR    (ZXUP_A5C3_POS5_ADDR+1)  // �Ž�����
#define ZXUP_A5C3_POS7_ADDR    (ZXUP_A5C3_POS6_ADDR+2)  // �Ž�����
#define SIZE_OF_ZXUP_A5C3      (ZXUP_A5C3_POS7_ADDR+2)  // ���ֽ���(12B)

//==TAG-A5C4�ҳ�����============================================================
#define zxup_buffer_a5c4       (zxup_buffer_a5c3+SIZE_OF_ZXUP_A5C3)// ��ʼ��ַ����
#define ZXUP_A5C4_POS1_ADDR    0                        // ������
#define ZXUP_A5C4_POS2_ADDR    (ZXUP_A5C4_POS1_ADDR+2)  // ������
#define ZXUP_A5C4_POS3_ADDR    (ZXUP_A5C4_POS2_ADDR+2)  // ����������
#define ZXUP_A5C4_POS4_ADDR    (ZXUP_A5C4_POS3_ADDR+2)  // �ֽ�״̬λ1
#define ZXUP_A5C4_POS5_ADDR    (ZXUP_A5C4_POS4_ADDR+1)  // �ֽ�״̬λ2
#define ZXUP_A5C4_POS6_ADDR    (ZXUP_A5C4_POS5_ADDR+1)  // �Ž�����
#define ZXUP_A5C4_POS7_ADDR    (ZXUP_A5C4_POS6_ADDR+2)  // �Ž�����
#define SIZE_OF_ZXUP_A5C4      (ZXUP_A5C4_POS7_ADDR+2)  // ���ֽ���(12B)

//==TAG-A5C8��ҵ�ͺ���Ϣ==========================================================
#define zxup_buffer_a5c8              (zxup_buffer_a5c4+SIZE_OF_ZXUP_A5C4)// ��ʼ��ַ����
#define ZXUP_A5C8_POS1_ADDR    0                        // ��ҵ������ʱ��
#define ZXUP_A5C8_POS2_ADDR    (ZXUP_A5C8_POS1_ADDR+4)  // ��ҵȼ����������
#define ZXUP_A5C8_POS3_ADDR    (ZXUP_A5C8_POS2_ADDR+4)  // ��ҵƽ���ͺ�
#define SIZE_OF_ZXUP_A5C8      (ZXUP_A5C8_POS3_ADDR+2)  // ���ֽ���(10B)

//==TAG-A5C9 ECU��Ӧ����CAN֡=====================================================
#define zxup_buffer_a5c9       (zxup_buffer_a5c8+SIZE_OF_ZXUP_A5C8)// ��ʼ��ַ����
#define ZXUP_A5C9_POS1_ADDR    0                        // ��ҵ������ʱ��
#define ZXUP_A5C9_POS2_ADDR    (ZXUP_A5C9_POS1_ADDR+1)  // ��ҵȼ����������
#define ZXUP_A5C9_POS3_ADDR    (ZXUP_A5C9_POS2_ADDR+4)  // ��ҵƽ���ͺ�
#define SIZE_OF_ZXUP_A5C9      (ZXUP_A5C9_POS3_ADDR+8)  // ���ֽ���(13B)

// �ϳ����ݻ����С
#define SIZE_OF_ZXUP_BUFFER   (zxup_buffer_a5c9+SIZE_OF_ZXUP_A5C9-zxup_buffer_a5a0)

/******************************************************************************
* Macros(�³�ͨ��): ���ػ��³�������ź͵�ַ����
******************************************************************************/
//==TAG-A5E0�ڵ�״̬==============================================================
#define zxdown_buffer_a5e0          (zxdown_buffer)   // ��ʼ��ַ����
#define ZXDOWN_A5E0_POS1_ADDR    0                          // �ֽ�״̬λ1
#define ZXDOWN_A5E0_POS2_ADDR    (ZXDOWN_A5E0_POS1_ADDR+1)  // �ֽ�״̬λ2
#define SIZE_OF_ZXDOWN_A5E0      (ZXDOWN_A5E0_POS2_ADDR+1)  // ���ֽ���(2B)

//==TAG-A5E1����ϵͳ==============================================================
#define zxdown_buffer_a5e1       (zxdown_buffer_a5e0+SIZE_OF_ZXDOWN_A5E0) // ��ʼ��ַ����
#define ZXDOWN_A5E1_POS1_ADDR    0                          // �����䵵λ
#define ZXDOWN_A5E1_POS2_ADDR    (ZXDOWN_A5E1_POS1_ADDR+1)  // �ֽ�״̬λ
#define ZXDOWN_A5E1_POS3_ADDR    (ZXDOWN_A5E1_POS2_ADDR+1)  // ����������
#define ZXDOWN_A5E1_POS4_ADDR    (ZXDOWN_A5E1_POS3_ADDR+2)  // �����������ѹ
#define ZXDOWN_A5E1_POS5_ADDR    (ZXDOWN_A5E1_POS4_ADDR+1)  // �����λ��
#define SIZE_OF_ZXDOWN_A5E1      (ZXDOWN_A5E1_POS5_ADDR+1)  // ���ֽ���(6B)

//==TAG-A5E2֧�������Ϣ==========================================================
#define zxdown_buffer_a5e2       (zxdown_buffer_a5e1+SIZE_OF_ZXDOWN_A5E1)// ��ʼ��ַ����
#define ZXDOWN_A5E2_POS1_ADDR    0                          // �ֽ�״̬λ1
#define ZXDOWN_A5E2_POS2_ADDR    (ZXDOWN_A5E2_POS1_ADDR+1)  // �ֽ�״̬λ2
#define ZXDOWN_A5E2_POS3_ADDR    (ZXDOWN_A5E2_POS2_ADDR+1)  // �ֽ�״̬λ3
#define SIZE_OF_ZXDOWN_A5E2      (ZXDOWN_A5E2_POS3_ADDR+1)  // ���ֽ���(3B)

//==TAG-A5E3����ϵͳ==============================================================
#define zxdown_buffer_a5e3       (zxdown_buffer_a5e2+SIZE_OF_ZXDOWN_A5E2)// ��ʼ��ַ����
#define ZXDOWN_A5E3_POS1_ADDR    0                          // ����ѹ��
#define ZXDOWN_A5E3_POS2_ADDR    (ZXDOWN_A5E3_POS1_ADDR+8)  // �����г�
#define ZXDOWN_A5E3_POS3_ADDR    (ZXDOWN_A5E3_POS2_ADDR+8)  // ��������
#define ZXDOWN_A5E3_POS4_ADDR    (ZXDOWN_A5E3_POS3_ADDR+4)  // �ֽ�״̬λ1
#define ZXDOWN_A5E3_POS5_ADDR    (ZXDOWN_A5E3_POS4_ADDR+1)  // �ֽ�״̬λ2
#define SIZE_OF_ZXDOWN_A5E3      (ZXDOWN_A5E3_POS5_ADDR+1)  // ���ֽ���(22B)

//==TAG-A5E4ת��ϵͳ��ȫ���棩====================================================
#define zxdown_buffer_a5e4       (zxdown_buffer_a5e3+SIZE_OF_ZXDOWN_A5E3)// ��ʼ��ַ����
#define ZXDOWN_A5E4_POS1_ADDR    0                          // һ����ת��Ƕ�
#define ZXDOWN_A5E4_POS2_ADDR    (ZXDOWN_A5E4_POS1_ADDR+8)  // ����������ת��Ƕ�
#define ZXDOWN_A5E4_POS3_ADDR    (ZXDOWN_A5E4_POS2_ADDR+8)  // һ���ᴫ��������
#define ZXDOWN_A5E4_POS4_ADDR    (ZXDOWN_A5E4_POS3_ADDR+8)  // ���������ᴫ��������
#define ZXDOWN_A5E4_POS5_ADDR    (ZXDOWN_A5E4_POS4_ADDR+8)  // ��ǰת��ģʽ
#define ZXDOWN_A5E4_POS6_ADDR    (ZXDOWN_A5E4_POS5_ADDR+1)  // Ŀ��ת��ģʽ
#define ZXDOWN_A5E4_POS7_ADDR    (ZXDOWN_A5E4_POS6_ADDR+1)  // ת��ϵͳѹ��
#define ZXDOWN_A5E4_POS8_ADDR    (ZXDOWN_A5E4_POS7_ADDR+2)  // ������ѹ������
#define ZXDOWN_A5E4_POS9_ADDR    (ZXDOWN_A5E4_POS8_ADDR+2)  // 123������ת��ռ�ձ�
#define ZXDOWN_A5E4_POS10_ADDR   (ZXDOWN_A5E4_POS9_ADDR+6)  // 456������ת��ռ�ձ�
#define ZXDOWN_A5E4_POS11_ADDR   (ZXDOWN_A5E4_POS10_ADDR+6)  // ����ֹ��
#define SIZE_OF_ZXDOWN_A5E4      (ZXDOWN_A5E4_POS11_ADDR+1)  // ���ֽ���(51B)

//==TAG-A5E5ת��ϵͳ��������======================================================
#define zxdown_buffer_a5e5       (zxdown_buffer_a5e4+SIZE_OF_ZXDOWN_A5E4)// ��ʼ��ַ����
#define ZXDOWN_A5E5_POS1_ADDR    0                          // ��ǰһ��ת��
#define ZXDOWN_A5E5_POS2_ADDR    (ZXDOWN_A5E5_POS1_ADDR+2)  // ת������ѹ��(bar)
#define ZXDOWN_A5E5_POS3_ADDR    (ZXDOWN_A5E5_POS2_ADDR+2)  // Ŀ��ת��ģʽ+��ǰת��ģʽ
#define ZXDOWN_A5E5_POS4_ADDR    (ZXDOWN_A5E5_POS3_ADDR+2)  // ������λ
#define ZXDOWN_A5E5_POS5_ADDR    (ZXDOWN_A5E5_POS4_ADDR+1)  // �ֽ�״̬λ1
#define ZXDOWN_A5E5_POS6_ADDR    (ZXDOWN_A5E5_POS5_ADDR+1)  // �ֽ�״̬λ2
#define SIZE_OF_ZXDOWN_A5E5      (ZXDOWN_A5E5_POS6_ADDR+1)  // ���ֽ���(9B)

//==TAG-A5E6�ƶ�ϵͳ==============================================================
#define zxdown_buffer_a5e6       (zxdown_buffer_a5e5+SIZE_OF_ZXDOWN_A5E5)// ��ʼ��ַ����
#define ZXDOWN_A5E6_POS1_ADDR    0                          // ��·һ��ѹ
#define ZXDOWN_A5E6_POS2_ADDR    (ZXDOWN_A5E6_POS1_ADDR+2)  // ��·����ѹ
#define ZXDOWN_A5E6_POS3_ADDR    (ZXDOWN_A5E6_POS2_ADDR+2)  // �ƶ�ѹ��
#define ZXDOWN_A5E6_POS4_ADDR    (ZXDOWN_A5E6_POS3_ADDR+2)  // ��ѹ���
#define ZXDOWN_A5E6_POS5_ADDR    (ZXDOWN_A5E6_POS4_ADDR+2)  // ����������
#define ZXDOWN_A5E6_POS6_ADDR    (ZXDOWN_A5E6_POS5_ADDR+2)  // �������ذٷֱ�
#define ZXDOWN_A5E6_POS7_ADDR    (ZXDOWN_A5E6_POS6_ADDR+1)  // ���������
#define ZXDOWN_A5E6_POS8_ADDR    (ZXDOWN_A5E6_POS7_ADDR+2)  // �ֽ�״̬λ1
#define ZXDOWN_A5E6_POS9_ADDR    (ZXDOWN_A5E6_POS8_ADDR+1)  // �ֽ�״̬λ2
#define SIZE_OF_ZXDOWN_A5E6      (ZXDOWN_A5E6_POS9_ADDR+1)  // ���ֽ���(15B)

//==TAG-A5E7����ϵͳ==============================================================
#define zxdown_buffer_a5e7       (zxdown_buffer_a5e6+SIZE_OF_ZXDOWN_A5E6)// ��ʼ��ַ����
#define ZXDOWN_A5E7_POS1_ADDR    0                          // �ֽ�״̬λ
#define SIZE_OF_ZXDOWN_A5E7      (ZXDOWN_A5E7_POS1_ADDR+1)  // ���ֽ���(1B)

//==TAG-A5E8����ȡ��ϵͳ==========================================================
#define zxdown_buffer_a5e8       (zxdown_buffer_a5e7+SIZE_OF_ZXDOWN_A5E7)// ��ʼ��ַ����
#define ZXDOWN_A5E8_POS1_ADDR    0                          // �ֽ�״̬λ1
#define ZXDOWN_A5E8_POS2_ADDR    (ZXDOWN_A5E8_POS1_ADDR+1)  // �ֽ�״̬λ2
#define SIZE_OF_ZXDOWN_A5E8      (ZXDOWN_A5E8_POS2_ADDR+1)  // ���ֽ���(2B)

//==TAG-A5E9Һѹϵͳ==============================================================
#define zxdown_buffer_a5e9       (zxdown_buffer_a5e8+SIZE_OF_ZXDOWN_A5E8)// ��ʼ��ַ����
#define ZXDOWN_A5E9_POS1_ADDR    0                                       // Һѹ���¶�
#define ZXDOWN_A5E9_POS2_ADDR    (ZXDOWN_A5E9_POS1_ADDR+1)  // Һѹϵͳѹ��
#define SIZE_OF_ZXDOWN_A5E9      (ZXDOWN_A5E9_POS2_ADDR+2)  // ���ֽ���(3B)

//==TAG-A5EA˫��������ϵͳ========================================================
#define zxdown_buffer_a5ea       (zxdown_buffer_a5e9+SIZE_OF_ZXDOWN_A5E9)// ��ʼ��ַ����
#define ZXDOWN_A5EA_POS1_ADDR    0                          // �ֽ�״̬λ1
#define ZXDOWN_A5EA_POS2_ADDR    (ZXDOWN_A5EA_POS1_ADDR+1)  // �ֽ�״̬λ2
#define ZXDOWN_A5EA_POS3_ADDR    (ZXDOWN_A5EA_POS2_ADDR+1)  // �ϳ�������ˮ��
#define ZXDOWN_A5EA_POS4_ADDR    (ZXDOWN_A5EA_POS3_ADDR+1)  // �ϳ�����������ѹ��
#define ZXDOWN_A5EA_POS5_ADDR    (ZXDOWN_A5EA_POS4_ADDR+2)  // �ϳ�������ת��
#define ZXDOWN_A5EA_POS6_ADDR    (ZXDOWN_A5EA_POS5_ADDR+2)  // �ϳ��ó���ѹ��
#define ZXDOWN_A5EA_POS7_ADDR    (ZXDOWN_A5EA_POS6_ADDR+2)  // �����Ƶ���
#define ZXDOWN_A5EA_POS8_ADDR    (ZXDOWN_A5EA_POS7_ADDR+2)  // ɢ����Һѹ�����ÿ��Ƶ���
#define SIZE_OF_ZXDOWN_A5EA      (ZXDOWN_A5EA_POS8_ADDR+2)  // ���ֽ���(13B)

//==TAG-A5EB��̥̥ѹ==============================================================
#define zxdown_buffer_a5eb       (zxdown_buffer_a5ea+SIZE_OF_ZXDOWN_A5EA)// ��ʼ��ַ����
#define ZXDOWN_A5EB_POS1_ADDR    0                          // ̥ѹ#1-6
#define ZXDOWN_A5EB_POS2_ADDR    (ZXDOWN_A5EB_POS1_ADDR+6)  // ̥ѹ#7-12
#define ZXDOWN_A5EB_POS3_ADDR    (ZXDOWN_A5EB_POS2_ADDR+6)  // ̥ѹ#13-18
#define SIZE_OF_ZXDOWN_A5EB      (ZXDOWN_A5EB_POS3_ADDR+6)  // ���ֽ���(18B)

//==TAG-A5EC��֧�����============================================================
#define zxdown_buffer_a5ec       (zxdown_buffer_a5eb+SIZE_OF_ZXDOWN_A5EB)// ��ʼ��ַ����
#define ZXDOWN_A5EC_POS1_ADDR    0                          // �ֽ�״̬λ1
#define ZXDOWN_A5EC_POS2_ADDR    (ZXDOWN_A5EC_POS1_ADDR+1)  // �ֽ�״̬λ2
#define ZXDOWN_A5EC_POS3_ADDR    (ZXDOWN_A5EC_POS2_ADDR+1)  // �ֽ�״̬λ3
#define ZXDOWN_A5EC_POS4_ADDR    (ZXDOWN_A5EC_POS3_ADDR+1)  // �ֽ�״̬λ4
#define ZXDOWN_A5EC_POS5_ADDR    (ZXDOWN_A5EC_POS4_ADDR+1)  // �ֽ�״̬λ5
#define ZXDOWN_A5EC_POS6_ADDR    (ZXDOWN_A5EC_POS5_ADDR+1)  // Ԥ��
#define ZXDOWN_A5EC_POS7_ADDR    (ZXDOWN_A5EC_POS6_ADDR+1)  // DIR
#define ZXDOWN_A5EC_POS8_ADDR    (ZXDOWN_A5EC_POS7_ADDR+1)  // ��ƽ������
#define SIZE_OF_ZXDOWN_A5EC      (ZXDOWN_A5EC_POS8_ADDR+1)  // ���ֽ���(8B)

//==TAG-A5ED��֧�����============================================================
#define zxdown_buffer_a5ed       (zxdown_buffer_a5ec+SIZE_OF_ZXDOWN_A5EC)// ��ʼ��ַ����
#define ZXDOWN_A5ED_POS1_ADDR    0                          // �ֽ�״̬λ1
#define ZXDOWN_A5ED_POS2_ADDR    (ZXDOWN_A5ED_POS1_ADDR+1)  // �ֽ�״̬λ2
#define ZXDOWN_A5ED_POS3_ADDR    (ZXDOWN_A5ED_POS2_ADDR+1)  // �ֽ�״̬λ3
#define ZXDOWN_A5ED_POS4_ADDR    (ZXDOWN_A5ED_POS3_ADDR+1)  // �ֽ�״̬λ4
#define ZXDOWN_A5ED_POS5_ADDR    (ZXDOWN_A5ED_POS4_ADDR+1)  // �ֽ�״̬λ5
#define ZXDOWN_A5ED_POS6_ADDR    (ZXDOWN_A5ED_POS5_ADDR+1)  // Ԥ��
#define ZXDOWN_A5ED_POS7_ADDR    (ZXDOWN_A5ED_POS6_ADDR+1)  // DIR
#define ZXDOWN_A5ED_POS8_ADDR    (ZXDOWN_A5ED_POS7_ADDR+1)  // ��ƽ������
#define SIZE_OF_ZXDOWN_A5ED      (ZXDOWN_A5ED_POS8_ADDR+1)  // ���ֽ���(8B)

//==TAG-A5EE�п�̨������Ϣ========================================================
#define zxdown_buffer_a5ee       (zxdown_buffer_a5ed+SIZE_OF_ZXDOWN_A5ED)// ��ʼ��ַ����
#define ZXDOWN_A5EE_POS1_ADDR    0                          // �ֽ�״̬λ1
#define ZXDOWN_A5EE_POS2_ADDR    (ZXDOWN_A5EE_POS1_ADDR+1)  // �ֽ�״̬λ2
#define ZXDOWN_A5EE_POS3_ADDR    (ZXDOWN_A5EE_POS2_ADDR+1)  // �ֽ�״̬λ3
#define ZXDOWN_A5EE_POS4_ADDR    (ZXDOWN_A5EE_POS3_ADDR+1)  // �ֽ�״̬λ4
#define ZXDOWN_A5EE_POS5_ADDR    (ZXDOWN_A5EE_POS4_ADDR+1)  // �ֽ�״̬λ5
#define ZXDOWN_A5EE_POS6_ADDR    (ZXDOWN_A5EE_POS5_ADDR+1)  // �ֽ�״̬λ6
#define ZXDOWN_A5EE_POS7_ADDR    (ZXDOWN_A5EE_POS6_ADDR+1)  // �ֽ�״̬λ7
#define ZXDOWN_A5EE_POS8_ADDR    (ZXDOWN_A5EE_POS7_ADDR+1)  // ����ת��Ƕ�
#define ZXDOWN_A5EE_POS9_ADDR    (ZXDOWN_A5EE_POS8_ADDR+1)  // �ֽ�״̬λ8
#define SIZE_OF_ZXDOWN_A5EE      (ZXDOWN_A5EE_POS9_ADDR+1)  // ���ֽ���(9B)

//==TAG-A5A3֧����ҵ��Ϣ==========================================================
#define zxdown_buffer_a5a3       (zxdown_buffer_a5ee+SIZE_OF_ZXDOWN_A5EE)// ��ʼ��ַ����
#define ZXDOWN_A5A3_POS1_ADDR    0                        // ˮƽ֧�ȳ���
#define ZXDOWN_A5A3_POS2_ADDR    (ZXDOWN_A5A3_POS1_ADDR+8)  // ֧��ѹ��
#define ZXDOWN_A5A3_POS3_ADDR    (ZXDOWN_A5A3_POS2_ADDR+8)  // �ڸ׳���
#define SIZE_OF_ZXDOWN_A5A3        (ZXDOWN_A5A3_POS3_ADDR+8)  // ���ֽ���(24B)

//==TAG-A5A4����֧����ҵ��Ϣ=======================================================
#define zxdown_buffer_a5a4       (zxdown_buffer_a5a3+SIZE_OF_ZXDOWN_A5A3)// ��ʼ��ַ����
#define ZXDOWN_A5A4_POS1_ADDR    0                        // ����֧��״̬
#define ZXDOWN_A5A4_POS2_ADDR    (ZXDOWN_A5A4_POS1_ADDR+1)  // ��ǰ����֧��ѹ��
#define ZXDOWN_A5A4_POS3_ADDR    (ZXDOWN_A5A4_POS2_ADDR+2)  // ��ǰ����֧��ѹ��
#define SIZE_OF_ZXDOWN_A5A4      (ZXDOWN_A5A4_POS3_ADDR+2)  // ���ֽ���(5B)

// �³��������ݻ����С
#define SIZE_OF_ZXDOWN_BUFFER   (zxdown_buffer_a5a4+SIZE_OF_ZXDOWN_A5A4-zxdown_buffer_a5e0) 

/******************************************************************************
* Macros(���̷�����): ���ػ����̷�����������ź͵�ַ����
******************************************************************************/
//==TAG-A5EF���������в���(���ġ����塢����)======================================
#define zxengine_buffer_a5ef              zxengine_buffer  // ��ʼ��ַ����
#define ZXENGINE_A5EF_POS1_ADDR    0                            // ������ת��
#define ZXENGINE_A5EF_POS2_ADDR    (ZXENGINE_A5EF_POS1_ADDR+2)  // ʵ��Ť�ذٷֱ�
#define ZXENGINE_A5EF_POS3_ADDR    (ZXENGINE_A5EF_POS2_ADDR+1)  // Ħ��Ť�ذٷֱ�
#define ZXENGINE_A5EF_POS4_ADDR    (ZXENGINE_A5EF_POS3_ADDR+1)  // �������ѹ��
#define ZXENGINE_A5EF_POS5_ADDR    (ZXENGINE_A5EF_POS4_ADDR+1)  // ��������¶�
#define ZXENGINE_A5EF_POS6_ADDR    (ZXENGINE_A5EF_POS5_ADDR+1)  // ��ȴҺ�¶�
#define ZXENGINE_A5EF_POS7_ADDR    (ZXENGINE_A5EF_POS6_ADDR+1)  // �����¶�
#define ZXENGINE_A5EF_POS8_ADDR    (ZXENGINE_A5EF_POS7_ADDR+2)  // ����Һλ
#define ZXENGINE_A5EF_POS9_ADDR    (ZXENGINE_A5EF_POS8_ADDR+1)  // ����ѹ��
#define ZXENGINE_A5EF_POS10_ADDR   (ZXENGINE_A5EF_POS9_ADDR+1)  // ������������ʱ��
#define ZXENGINE_A5EF_POS11_ADDR   (ZXENGINE_A5EF_POS10_ADDR+4)  // ����̤��ٷֱ�
#define ZXENGINE_A5EF_POS12_ADDR   (ZXENGINE_A5EF_POS11_ADDR+1)  // ����
#define ZXENGINE_A5EF_POS13_ADDR   (ZXENGINE_A5EF_POS12_ADDR+2)  // �ֽ�״̬λ1
#define ZXENGINE_A5EF_POS14_ADDR   (ZXENGINE_A5EF_POS13_ADDR+1)  // �ֽ�״̬λ2
#define ZXENGINE_A5EF_POS15_ADDR   (ZXENGINE_A5EF_POS14_ADDR+1)  // Ѳ���趨�ٶ�
#define ZXENGINE_A5EF_POS16_ADDR   (ZXENGINE_A5EF_POS15_ADDR+1)  // ������ȼ��������
#define ZXENGINE_A5EF_POS17_ADDR   (ZXENGINE_A5EF_POS16_ADDR+2)  // ������ƽ��ȼ��������
#define ZXENGINE_A5EF_POS18_ADDR   (ZXENGINE_A5EF_POS17_ADDR+2)  // ȼ�����ͺ���
#define ZXENGINE_A5EF_POS19_ADDR   (ZXENGINE_A5EF_POS18_ADDR+4)  // ����ʻ���
#define ZXENGINE_A5EF_POS20_ADDR   (ZXENGINE_A5EF_POS19_ADDR+4)  // ȼ��Һλ1
#define ZXENGINE_A5EF_POS21_ADDR   (ZXENGINE_A5EF_POS20_ADDR+1)  // ȼ��Һλ2
#define ZXENGINE_A5EF_POS22_ADDR   (ZXENGINE_A5EF_POS21_ADDR+1)  // �ֽ�״̬λ3
#define ZXENGINE_A5EF_POS23_ADDR   (ZXENGINE_A5EF_POS22_ADDR+1)  // �ֽ�״̬λ4
#define SIZE_OF_ZXENGINE_A5EF      (ZXENGINE_A5EF_POS23_ADDR+1)  // ���ֽ���(37B)

//==TAG-A5F0��ʻ�ͺ�==============================================================
#define zxengine_buffer_a5f0              (zxengine_buffer_a5ef+SIZE_OF_ZXENGINE_A5EF)// ��ʼ��ַ����
#define ZXENGINE_A5F0_POS1_ADDR    0                            // ��ʻ������ʱ��
#define ZXENGINE_A5F0_POS2_ADDR    (ZXENGINE_A5F0_POS1_ADDR+4)  // ��ʻȼ�����ͺ���
#define ZXENGINE_A5F0_POS3_ADDR    (ZXENGINE_A5F0_POS2_ADDR+4)  // �ٹ����ͺ�
#define SIZE_OF_ZXENGINE_A5F0      (ZXENGINE_A5F0_POS3_ADDR+4)  // ���ֽ���(12B)

//==TAG-A5F1 SCR���������壩======================================================
#define zxengine_buffer_a5f1               (zxengine_buffer_a5f0+SIZE_OF_ZXENGINE_A5F0)// ��ʼ��ַ����
#define ZXENGINE_A5F1_POS1_ADDR    0                            // ��������״̬
#define ZXENGINE_A5F1_POS2_ADDR    (ZXENGINE_A5F1_POS1_ADDR+1)  // T15_DCU
#define ZXENGINE_A5F1_POS3_ADDR    (ZXENGINE_A5F1_POS2_ADDR+1)  // ���ر�ѹ��
#define ZXENGINE_A5F1_POS4_ADDR    (ZXENGINE_A5F1_POS3_ADDR+2)  // ������Һλ
#define ZXENGINE_A5F1_POS5_ADDR    (ZXENGINE_A5F1_POS4_ADDR+1)  // �������¶�
#define ZXENGINE_A5F1_POS6_ADDR    (ZXENGINE_A5F1_POS5_ADDR+1)  // ����������
#define ZXENGINE_A5F1_POS7_ADDR    (ZXENGINE_A5F1_POS6_ADDR+2)  // SCR����NOxŨ��
#define ZXENGINE_A5F1_POS8_ADDR    (ZXENGINE_A5F1_POS7_ADDR+2)  // SCR����NOxŨ��
#define ZXENGINE_A5F1_POS9_ADDR    (ZXENGINE_A5F1_POS8_ADDR+2)  // SCR���������¶�(T6�¶�)
#define ZXENGINE_A5F1_POS10_ADDR   (ZXENGINE_A5F1_POS9_ADDR+2)  // SCR���������¶�(T7�¶�)
#define ZXENGINE_A5F1_POS11_ADDR   (ZXENGINE_A5F1_POS10_ADDR+2)  // ����Ũ��(SPN 3516)
#define ZXENGINE_A5F1_POS12_ADDR   (ZXENGINE_A5F1_POS11_ADDR+1)  // �ۼ�����������
#define ZXENGINE_A5F1_POS13_ADDR   (ZXENGINE_A5F1_POS12_ADDR+4)  // ����Ʒ�ʴ������¶�(SPN 3515)
#define ZXENGINE_A5F1_POS14_ADDR   (ZXENGINE_A5F1_POS13_ADDR+1)  // Ʒ���¶ȴ�����FMI (SPN 3519)
#define ZXENGINE_A5F1_POS15_ADDR   (ZXENGINE_A5F1_POS14_ADDR+1)  // Ʒ�ʴ�����FMI (SPN3520)
#define ZXENGINE_A5F1_POS16_ADDR   (ZXENGINE_A5F1_POS15_ADDR+1)  // �߻����Լ�����(SPN3521)
#define ZXENGINE_A5F1_POS17_ADDR   (ZXENGINE_A5F1_POS16_ADDR+1)  // ������Һλ������ʧЧģʽFMI
#define ZXENGINE_A5F1_POS18_ADDR   (ZXENGINE_A5F1_POS17_ADDR+1)  // �������¶ȴ�����ʧЧģʽFMI
#define ZXENGINE_A5F1_POS19_ADDR   (ZXENGINE_A5F1_POS18_ADDR+1)  // Nox������¶��״̬
#define SIZE_OF_ZXENGINE_A5F1      (ZXENGINE_A5F1_POS19_ADDR+1)  // ���ֽ���(28B)

//==TAG-A5F2 DPF����(������=======================================================
#define zxengine_buffer_a5f2              (zxengine_buffer_a5f1+SIZE_OF_ZXENGINE_A5F1)// ��ʼ��ַ����
#define ZXENGINE_A5F2_POS1_ADDR    0                            // DOC���������¶�
#define ZXENGINE_A5F2_POS2_ADDR    (ZXENGINE_A5F2_POS1_ADDR+2)  // DPF���������¶�
#define ZXENGINE_A5F2_POS3_ADDR    (ZXENGINE_A5F2_POS2_ADDR+2)  // DPF̼����������
#define ZXENGINE_A5F2_POS4_ADDR    (ZXENGINE_A5F2_POS3_ADDR+1)  // DPFѹ��
#define ZXENGINE_A5F2_POS5_ADDR    (ZXENGINE_A5F2_POS4_ADDR+2)  // �ֽ�״̬λ1
#define SIZE_OF_ZXENGINE_A5F2      (ZXENGINE_A5F2_POS5_ADDR+1)  // ���ֽ���(8B)

// �³����������ݻ����С
#define SIZE_OF_ZXENGINE_BUFFER   (zxengine_buffer_a5f2+SIZE_OF_ZXENGINE_A5F2-zxengine_buffer_a5ef) 

/******************************************************************************
* Macros(ͳ������Ϣ): ������ź͵�ַ����
******************************************************************************/
//==TAG-A5C5 ����Ƶ��ͳ��1========================================================
#define zxstatistics_buffer_a5c5        zxstatistics_buffer
#deinfe ZXSTATISTICS_A5C5_POS1_ADDR   0  // ������
#deinfe ZXSTATISTICS_A5C5_POS2_ADDR   (ZXSTATISTICS_A5C5_POS1_ADDR+8)  // ������
#deinfe ZXSTATISTICS_A5C5_POS3_ADDR   (ZXSTATISTICS_A5C5_POS2_ADDR+8)  // �����
#deinfe ZXSTATISTICS_A5C5_POS4_ADDR   (ZXSTATISTICS_A5C5_POS3_ADDR+8)  // �����
#deinfe ZXSTATISTICS_A5C5_POS5_ADDR   (ZXSTATISTICS_A5C5_POS4_ADDR+8)  // �����
#deinfe ZXSTATISTICS_A5C5_POS6_ADDR   (ZXSTATISTICS_A5C5_POS5_ADDR+8)  // �������
#deinfe ZXSTATISTICS_A5C5_POS7_ADDR   (ZXSTATISTICS_A5C5_POS6_ADDR+8)  // ��������
#deinfe ZXSTATISTICS_A5C5_POS8_ADDR   (ZXSTATISTICS_A5C5_POS7_ADDR+8)  // ��������
#deinfe ZXSTATISTICS_A5C5_POS9_ADDR   (ZXSTATISTICS_A5C5_POS8_ADDR+8)  // ��������
#deinfe ZXSTATISTICS_A5C5_POS10_ADDR  (ZXSTATISTICS_A5C5_POS9_ADDR+8)  // ��������
#deinfe ZXSTATISTICS_A5C5_POS11_ADDR   (ZXSTATISTICS_A5C5_POS10_ADDR+8)  // ���ת
#deinfe ZXSTATISTICS_A5C5_POS12_ADDR   (ZXSTATISTICS_A5C5_POS11_ADDR+8)  // �һ�ת
#define SIZE_OF_ZXSTATISTICS_A5C5        (ZXSTATISTICS_A5C5_POS12_ADDR+8)  // ���ֽ���(96B)

//==TAG-A5C6 ����Ƶ��ͳ��2========================================================
#define zxstatistics_buffer_a5c6        (zxstatistics_buffer_a5c5+SIZE_OF_ZXSTATISTICS_A5C5)
#deinfe ZXSTATISTICS_A5C6_POS1_ADDR   0  // �ո���
#deinfe ZXSTATISTICS_A5C6_POS2_ADDR   (ZXSTATISTICS_A5C6_POS1_ADDR+8)  // �ո���
#deinfe ZXSTATISTICS_A5C6_POS3_ADDR   (ZXSTATISTICS_A5C6_POS2_ADDR+8)  // ������
#deinfe ZXSTATISTICS_A5C6_POS4_ADDR   (ZXSTATISTICS_A5C6_POS3_ADDR+8)  // ������
#deinfe ZXSTATISTICS_A5C6_POS5_ADDR   (ZXSTATISTICS_A5C6_POS4_ADDR+8)  // �����
#deinfe ZXSTATISTICS_A5C6_POS6_ADDR   (ZXSTATISTICS_A5C6_POS5_ADDR+8)  // �������
#deinfe ZXSTATISTICS_A5C6_POS7_ADDR   (ZXSTATISTICS_A5C6_POS6_ADDR+8)  // ��������
#deinfe ZXSTATISTICS_A5C6_POS8_ADDR   (ZXSTATISTICS_A5C6_POS7_ADDR+8)  // ��������
#deinfe ZXSTATISTICS_A5C6_POS9_ADDR   (ZXSTATISTICS_A5C6_POS8_ADDR+8)  // ��������
#deinfe ZXSTATISTICS_A5C6_POS10_ADDR   (ZXSTATISTICS_A5C6_POS9_ADDR+8)  // ��������
#deinfe ZXSTATISTICS_A5C6_POS11_ADDR   (ZXSTATISTICS_A5C6_POS10_ADDR+8)  // ���ת
#deinfe ZXSTATISTICS_A5C6_POS12_ADDR   (ZXSTATISTICS_A5C6_POS11_ADDR+8)  // �һ�ת
#define SIZE_OF_ZXSTATISTICS_A5C6        (ZXSTATISTICS_A5C6_POS12_ADDR+8)  // ���ֽ���(96B)

//==TAG-A5C7 ��ȫͳ��=============================================================
#define zxstatistics_buffer_a5c7        (zxstatistics_buffer_a5c6+SIZE_OF_ZXSTATISTICS_A5C6)
#deinfe ZXSTATISTICS_A5C7_POS1_ADDR   0  // ����
#deinfe ZXSTATISTICS_A5C7_POS2_ADDR   (ZXSTATISTICS_A5C7_POS1_ADDR+8)  // ��Ȧ
#deinfe ZXSTATISTICS_A5C7_POS3_ADDR   (ZXSTATISTICS_A5C7_POS2_ADDR+8)  // ����
#deinfe ZXSTATISTICS_A5C7_POS4_ADDR   (ZXSTATISTICS_A5C7_POS3_ADDR+8)  // ��ǿ��
#deinfe ZXSTATISTICS_A5C7_POS5_ADDR   (ZXSTATISTICS_A5C7_POS4_ADDR+8)  // ��װ����
#deinfe ZXSTATISTICS_A5C7_POS6_ADDR   (ZXSTATISTICS_A5C7_POS5_ADDR+8)  // �����ǿ��
#deinfe ZXSTATISTICS_A5C7_POS7_ADDR   (ZXSTATISTICS_A5C7_POS6_ADDR+8)  // ����ǿ��
#deinfe ZXSTATISTICS_A5C7_POS8_ADDR   (ZXSTATISTICS_A5C7_POS7_ADDR+8)  // ��Ȧǿ��
#deinfe ZXSTATISTICS_A5C7_POS9_ADDR   (ZXSTATISTICS_A5C7_POS8_ADDR+8)  // ���ٳ���
#define SIZE_OF_ZXSTATISTICS_A5C7       (ZXSTATISTICS_A5C7_POS9_ADDR+8)  // ���ֽ���(72B)

/******************************************************************************
* Macros(�ϳ����³��汾��Ϣ): ������ź͵�ַ����
******************************************************************************/
//==TAG-A505 �ϳ�ϵͳ�汾=========================================================
#define zxversion_buffer_a505        zxversion_buffer
#deinfe ZXVERSION_A505_POS1_ADDR   0  // ����������
#deinfe ZXVERSION_A505_POS2_ADDR   (ZXVERSION_A505_POS1_ADDR+3)  // ��ʾ��1
#deinfe ZXVERSION_A505_POS3_ADDR   (ZXVERSION_A505_POS2_ADDR+3)  // ��ʾ���ײ�汾
#deinfe ZXVERSION_A505_POS4_ADDR   (ZXVERSION_A505_POS3_ADDR+3)  // GPS�ն�
#deinfe ZXVERSION_A505_POS5_ADDR   (ZXVERSION_A505_POS4_ADDR+3)  // ������
#deinfe ZXVERSION_A505_POS6_ADDR   (ZXVERSION_A505_POS5_ADDR+3)  // ��ʾ��2
#define SIZE_OF_ZXVERSION_A505       (ZXVERSION_A505_POS6_ADDR+3)  // ���ֽ���(18B)

//==TAG-A506 �³�ϵͳ�汾=========================================================
#define zxversion_buffer_a506        (zxversion_buffer_a505+SIZE_OF_ZXVERSION_A505)
#deinfe ZXVERSION_A506_POS1_ADDR   0   // ��ʾ��Ӧ�ò�
#deinfe ZXVERSION_A506_POS2_ADDR   (ZXVERSION_A506_POS1_ADDR+3)    // ��ʾ���ײ�
#deinfe ZXVERSION_A506_POS3_ADDR   (ZXVERSION_A506_POS2_ADDR+3)    // P1Ӧ�ò�
#deinfe ZXVERSION_A506_POS4_ADDR   (ZXVERSION_A506_POS3_ADDR+3)    // P1�ײ�
#deinfe ZXVERSION_A506_POS5_ADDR   (ZXVERSION_A506_POS4_ADDR+3)    // P2Ӧ�ò�
#deinfe ZXVERSION_A506_POS6_ADDR   (ZXVERSION_A506_POS5_ADDR+3)    // P2�ײ�
#deinfe ZXVERSION_A506_POS7_ADDR   (ZXVERSION_A506_POS6_ADDR+3)    // P3Ӧ�ò�
#deinfe ZXVERSION_A506_POS8_ADDR   (ZXVERSION_A506_POS7_ADDR+3)    // P3�ײ�
#deinfe ZXVERSION_A506_POS9_ADDR   (ZXVERSION_A506_POS8_ADDR+3)    // P4Ӧ�ò�
#deinfe ZXVERSION_A506_POS10_ADDR   (ZXVERSION_A506_POS9_ADDR+3)   // P4�ײ�
#deinfe ZXVERSION_A506_POS11_ADDR   (ZXVERSION_A506_POS10_ADDR+3)  // P5Ӧ�ò�
#deinfe ZXVERSION_A506_POS12_ADDR   (ZXVERSION_A506_POS11_ADDR+3)  // P5�ײ�
#deinfe ZXVERSION_A506_POS13_ADDR   (ZXVERSION_A506_POS12_ADDR+3)  // P6Ӧ�ò�
#deinfe ZXVERSION_A506_POS14_ADDR   (ZXVERSION_A506_POS13_ADDR+3)  // P6�ײ�
#deinfe ZXVERSION_A506_POS15_ADDR   (ZXVERSION_A506_POS14_ADDR+3)  // P7Ӧ�ò�
#deinfe ZXVERSION_A506_POS16_ADDR   (ZXVERSION_A506_POS15_ADDR+3)  // P7�ײ�
#deinfe ZXVERSION_A506_POS17_ADDR   (ZXVERSION_A506_POS16_ADDR+3)  // P8Ӧ�ò�
#deinfe ZXVERSION_A506_POS18_ADDR   (ZXVERSION_A506_POS17_ADDR+3)  // P8�ײ�
#define SIZE_OF_ZXVERSION_A506       (ZXVERSION_A506_POS18_ADDR+3)  // ���ֽ���(54B)

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
extern bittype2 zxup_tlv_flag1;
#define tlv_a5a0_valid_flag    zxup_tlv_flag1.w.bit0
#define tlv_a5a1_valid_flag    zxup_tlv_flag1.w.bit1
#define tlv_a5a2_valid_flag    zxup_tlv_flag1.w.bit2
//#define X    zxup_tlv_flag1.w.bit3
//#define X    zxup_tlv_flag1.w.bit4
#define tlv_a5a5_valid_flag    zxup_tlv_flag1.w.bit5
#define tlv_a5a6_valid_flag    zxup_tlv_flag1.w.bit6
#define tlv_a5a7_valid_flag    zxup_tlv_flag1.w.bit7
#define tlv_a5a8_valid_flag    zxup_tlv_flag1.w.bit8
#define tlv_a5a9_valid_flag    zxup_tlv_flag1.w.bit9
#define tlv_a5aa_valid_flag    zxup_tlv_flag1.w.bit10
#define tlv_a5ab_valid_flag    zxup_tlv_flag1.w.bit11
#define tlv_a5ac_valid_flag    zxup_tlv_flag1.w.bit12
#define tlv_a5ad_valid_flag    zxup_tlv_flag1.w.bit13
#define tlv_a5ae_valid_flag    zxup_tlv_flag1.w.bit14
#define tlv_a5af_valid_flag    zxup_tlv_flag1.w.bit15

// TLV��Ч��־λ
extern bittype2 zxup_tlv_flag2;
#define tlv_a5b0_valid_flag    zxup_tlv_flag2.w.bit0
#define tlv_a5b1_valid_flag    zxup_tlv_flag2.w.bit1
#define tlv_a5b2_valid_flag    zxup_tlv_flag2.w.bit2
#define tlv_a5b3_valid_flag    zxup_tlv_flag2.w.bit3
#define tlv_a5b4_valid_flag    zxup_tlv_flag2.w.bit4
#define tlv_a5b5_valid_flag    zxup_tlv_flag2.w.bit5
#define tlv_a5b6_valid_flag    zxup_tlv_flag2.w.bit6
#define tlv_a5b7_valid_flag    zxup_tlv_flag2.w.bit7
#define tlv_a5b8_valid_flag    zxup_tlv_flag2.w.bit8
#define tlv_a5b9_valid_flag    zxup_tlv_flag2.w.bit9
#define tlv_a5ba_valid_flag    zxup_tlv_flag2.w.bit10
#define tlv_a5bb_valid_flag    zxup_tlv_flag2.w.bit11
#define tlv_a5bc_valid_flag    zxup_tlv_flag2.w.bit12
#define tlv_a5bd_valid_flag    zxup_tlv_flag2.w.bit13
#define tlv_a5be_valid_flag    zxup_tlv_flag2.w.bit14
#define tlv_a5bf_valid_flag    zxup_tlv_flag2.w.bit15

// TLV��Ч��־λ
extern bittype2 zxup_tlv_flag3;
#define tlv_a5c0_valid_flag    zxup_tlv_flag3.w.bit0
#define tlv_a5c1_valid_flag    zxup_tlv_flag3.w.bit1
#define tlv_a5c2_valid_flag    zxup_tlv_flag3.w.bit2
#define tlv_a5c3_valid_flag    zxup_tlv_flag3.w.bit3
#define tlv_a5c4_valid_flag    zxup_tlv_flag3.w.bit4
#define tlv_a5c5_valid_flag    zxup_tlv_flag3.w.bit5
#define tlv_a5c6_valid_flag    zxup_tlv_flag3.w.bit6
#define tlv_a5c7_valid_flag    zxup_tlv_flag3.w.bit7
#define tlv_a5c8_valid_flag    zxup_tlv_flag3.w.bit8
#define tlv_a5c9_valid_flag    zxup_tlv_flag3.w.bit9
#define tlv_a5ca_valid_flag    zxup_tlv_flag3.w.bit10
#define tlv_a5cb_valid_flag    zxup_tlv_flag3.w.bit11
#define tlv_a5cc_valid_flag    zxup_tlv_flag3.w.bit12
#define tlv_a5cd_valid_flag    zxup_tlv_flag3.w.bit13
#define tlv_a5ce_valid_flag    zxup_tlv_flag3.w.bit14
#define tlv_a5cf_valid_flag    zxup_tlv_flag3.w.bit15

// TLV��Ч��־λ
extern bittype2 zxdown_tlv_flag1;
#define tlv_a5e0_valid_flag    zxdown_tlv_flag1.w.bit0
#define tlv_a5e1_valid_flag    zxdown_tlv_flag1.w.bit1
#define tlv_a5e2_valid_flag    zxdown_tlv_flag1.w.bit2
#define tlv_a5e3_valid_flag    zxdown_tlv_flag1.w.bit3
#define tlv_a5e4_valid_flag    zxdown_tlv_flag1.w.bit4
#define tlv_a5e5_valid_flag    zxdown_tlv_flag1.w.bit5
#define tlv_a5e6_valid_flag    zxdown_tlv_flag1.w.bit6
#define tlv_a5e7_valid_flag    zxdown_tlv_flag1.w.bit7
#define tlv_a5e8_valid_flag    zxdown_tlv_flag1.w.bit8
#define tlv_a5e9_valid_flag    zxdown_tlv_flag1.w.bit9
#define tlv_a5ea_valid_flag    zxdown_tlv_flag1.w.bit10
#define tlv_a5eb_valid_flag    zxdown_tlv_flag1.w.bit11
#define tlv_a5ec_valid_flag    zxdown_tlv_flag1.w.bit12
#define tlv_a5ed_valid_flag    zxdown_tlv_flag1.w.bit13
#define tlv_a5ee_valid_flag    zxdown_tlv_flag1.w.bit14
//#define x    zxdown_tlv_flag1.w.bit15

// TLV��Ч��־λ
extern bittype2 zxdown_tlv_flag2;
#define tlv_a5a3_valid_flag    zxdown_tlv_flag2.w.bit0
#define tlv_a5a4_valid_flag    zxdown_tlv_flag2.w.bit1
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
#define tlv_a5f0_valid_flag    zxengine_tlv_flag.w.bit0
#define tlv_a5f1_valid_flag    zxengine_tlv_flag.w.bit1
#define tlv_a5f2_valid_flag    zxengine_tlv_flag.w.bit2
#define tlv_a5f3_valid_flag    zxengine_tlv_flag.w.bit3
#define tlv_a5f4_valid_flag    zxengine_tlv_flag.w.bit4
#define tlv_a5f5_valid_flag    zxengine_tlv_flag.w.bit5
#define tlv_a5f6_valid_flag    zxengine_tlv_flag.w.bit6
#define tlv_a5f7_valid_flag    zxengine_tlv_flag.w.bit7
#define tlv_a5f8_valid_flag    zxengine_tlv_flag.w.bit8
#define tlv_a5f9_valid_flag    zxengine_tlv_flag.w.bit9
#define tlv_a5fa_valid_flag    zxengine_tlv_flag.w.bit10
#define tlv_a5fb_valid_flag    zxengine_tlv_flag.w.bit11
#define tlv_a5fc_valid_flag    zxengine_tlv_flag.w.bit12
#define tlv_a5fd_valid_flag    zxengine_tlv_flag.w.bit13
#define tlv_a5fe_valid_flag    zxengine_tlv_flag.w.bit14
#define tlv_a5ef_valid_flag    zxengine_tlv_flag.w.bit15  // ע�����λ

/******************************************************************************
 * Function prototypes
 ******************************************************************************/



#endif /* TCW_H_ */
