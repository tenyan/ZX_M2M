/*****************************************************************************
* @FileName: 
* @Engineer: TenYan
* @Company:  �칤��Ϣ����Ӳ����
* @version   V1.0
* @Date:     2020-11-20
* @brief     �칤�������ػ�canͨ��Э�鶨��
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
#define ZXUP_A5A0_ADDR    0                             // ��ʼ��ַ
#define zxup_buffer_a5a0              (zxup_buffer)    // ��ʼ��ַ����
#define ZXUP_A5A0_POS1    0                               // ����������1-2
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
#define SIZE_OF_ZXUP_A5A0         (ZXUP_A5A0_POS28+2) // ���ֽ���(82B)

//==TAG-A5A1���𹤿���Ϣ==========================================================
#define zxup_buffer_a5a1       (zxup_buffer+SIZE_OF_ZXUP_A5A0) // ��ʼ��ַ����
#define ZXUP_A5A1_ADDR    (ZXUP_A5A0_ADDR+SIZE_OF_ZXUP_A5A0)    // ��ʼ��ַ
#define ZXUP_A5A1_POS1    0                               // �������Ƕ�
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
#define SIZE_OF_ZXUP_A5A1         (ZXUP_A5A1_POS14+2)  // ���ֽ���(24B)

//==TAG-A5A2���۹�����Ϣ==========================================================
#define zxup_buffer_a5a2       (zxup_buffer+SIZE_OF_ZXUP_A5A1)// ��ʼ��ַ����
#define ZXUP_A5A2_ADDR    (ZXUP_A5A1_ADDR+SIZE_OF_ZXUP_A5A1)    // ��ʼ��ַ
#define ZXUP_A5A2_POS1    0            // ����������
#define ZXUP_A5A2_POS2    (ZXUP_A5A2_POS1+2)  // ����������
#define ZXUP_A5A2_POS3    (ZXUP_A5A2_POS2+2)  // ������ѹ��
#define ZXUP_A5A2_POS4    (ZXUP_A5A2_POS3+2)  // ǰ֧�ܽǶ�
#define ZXUP_A5A2_POS5    (ZXUP_A5A2_POS4+2)  // ����֧�ܼ��
#define SIZE_OF_ZXUP_A5A2      (ZXUP_A5A2_POS5+1)  // ���ֽ���(9B)

//==TAG-A5A5�ϳ���������Ϣ========================================================
#define zxup_buffer_a5a5       (zxup_buffer+SIZE_OF_ZXUP_A5A2)// ��ʼ��ַ����
#define ZXUP_A5A5_ADDR    (ZXUP_A5A2_ADDR+SIZE_OF_ZXUP_A5A2)    // ��ʼ��ַ
#define ZXUP_A5A5_POS1    0            // ������ת��
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
#define SIZE_OF_ZXUP_A5A5      (ZXUP_A5A5_POS16+1)  // ���ֽ���(23B)

//==TAG-A5A6�ֱ���Ϣ==============================================================
#define zxup_buffer_a5a6       (zxup_buffer+SIZE_OF_ZXUP_A5A5)// ��ʼ��ַ����
#define ZXUP_A5A6_ADDR    (ZXUP_A5A5_ADDR+SIZE_OF_ZXUP_A5A5)    // ��ʼ��ַ
#define ZXUP_A5A6_POS1    0            // ���ֱ�״̬λ
#define ZXUP_A5A6_POS2    (ZXUP_A5A6_POS1+1)  // ���ֱ�X���
#define ZXUP_A5A6_POS3    (ZXUP_A5A6_POS2+2)  // ���ֱ�Y���
#define ZXUP_A5A6_POS4    (ZXUP_A5A6_POS3+2)  // ���ֱ�״̬λ
#define ZXUP_A5A6_POS5    (ZXUP_A5A6_POS4+1)  // ���ֱ�X���
#define ZXUP_A5A6_POS6    (ZXUP_A5A6_POS5+2)  // ���ֱ�Y���
#define SIZE_OF_ZXUP_A5A6      (ZXUP_A5A6_POS6+2)  // ���ֽ���(10B)

//==TAG-A5A7��ʾ��1��Ϣ===========================================================
#define zxup_buffer_a5a7       (zxup_buffer+SIZE_OF_ZXUP_A5A6)// ��ʼ��ַ����
#define ZXUP_A5A7_ADDR    (ZXUP_A5A6_ADDR+SIZE_OF_ZXUP_A5A6)    // ��ʼ��ַ
#define ZXUP_A5A7_POS1    0            // ��Ʒ����
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
#define SIZE_OF_ZXUP_A5A7      (ZXUP_A5A7_POS14+1)  // ���ֽ���(20B)

//==TAG-A5A8��ʾ��2��Ϣ===========================================================
#define zxup_buffer_a5a8       (zxup_buffer+SIZE_OF_ZXUP_A5A7)// ��ʼ��ַ����
#define ZXUP_A5A8_ADDR    (ZXUP_A5A7_ADDR+SIZE_OF_ZXUP_A5A7)    // ��ʼ��ַ
#define ZXUP_A5A8_POS1    0            // ����ٿ�
#define ZXUP_A5A8_POS2    (ZXUP_A5A8_POS1+8)  // ����ά��
#define SIZE_OF_ZXUP_A5A8      (ZXUP_A5A8_POS2+2)  // ���ֽ���(10B)

//==TAG-A5A9���������Ϣ==========================================================
#define zxup_buffer_a5a9       (zxup_buffer+SIZE_OF_ZXUP_A5A8)// ��ʼ��ַ����
#define ZXUP_A5A9_ADDR    (ZXUP_A5A8_ADDR+SIZE_OF_ZXUP_A5A8)    // ��ʼ��ַ
#define ZXUP_A5A9_POS1    0            // ���1
#define ZXUP_A5A9_POS2    (ZXUP_A5A9_POS1+2)  // ���2
#define ZXUP_A5A9_POS3    (ZXUP_A5A9_POS2+2)  // ���3
#define SIZE_OF_ZXUP_A5A9      (ZXUP_A5A9_POS3+2)  // ���ֽ���(6B)

//==TAG-A5AA���߲ٿ���Ϣ==========================================================
#define zxup_buffer_a5aa       (zxup_buffer+SIZE_OF_ZXUP_A5A9)// ��ʼ��ַ����
#define ZXUP_A5AA_ADDR    (ZXUP_A5A9_ADDR+SIZE_OF_ZXUP_A5A9)    // ��ʼ��ַ
#define ZXUP_A5AA_POS1    0            // Msg1
#define ZXUP_A5AA_POS2    (ZXUP_A5AA_POS1+8)  // Msg2
#define ZXUP_A5AA_POS3    (ZXUP_A5AA_POS2+8)  // Msg3
#define ZXUP_A5AA_POS4    (ZXUP_A5AA_POS2+8)  // Msg4
#define SIZE_OF_ZXUP_A5AA      (ZXUP_A5AA_POS4+8)  // ���ֽ���(32B)

//==TAG-A5AB����������Ϣ==========================================================
#define zxup_buffer_a5ab       (zxup_buffer+SIZE_OF_ZXUP_A5AA)// ��ʼ��ַ����
#define ZXUP_A5AB_ADDR    (ZXUP_A5AA_ADDR+SIZE_OF_ZXUP_A5AA)    // ��ʼ��ַ
#define ZXUP_A5AB_POS1    0          // �ڵ�״̬
#define SIZE_OF_ZXUP_A5AB      (ZXUP_A5AB_POS1+8)  // ���ֽ���(8B)

//==TAG-A5AC�����߼���Ϣ==========================================================
#define zxup_buffer_a5ac       (zxup_buffer+SIZE_OF_ZXUP_A5AB)// ��ʼ��ַ����
#define ZXUP_A5AC_ADDR    (ZXUP_A5AB_ADDR+SIZE_OF_ZXUP_A5AB)    // ��ʼ��ַ
#define ZXUP_A5AC_POS1    0          // �������ƺͽ��
#define SIZE_OF_ZXUP_A5AC      (ZXUP_A5AC_POS1+6)  // ���ֽ���(6B)

//==TAG-A5AD����߼���Ϣ==========================================================
#define zxup_buffer_a5ad       (zxup_buffer+SIZE_OF_ZXUP_A5AC)// ��ʼ��ַ����
#define ZXUP_A5AD_ADDR    (ZXUP_A5AC_ADDR+SIZE_OF_ZXUP_A5AC)    // ��ʼ��ַ
#define ZXUP_A5AD_POS1    0          // �������ƺͽ��
#define SIZE_OF_ZXUP_A5AD      (ZXUP_A5AD_POS1+8)  // ���ֽ���(8B)

//==TAG-A5AE��ת�߼���Ϣ==========================================================
#define zxup_buffer_a5ae       (zxup_buffer+SIZE_OF_ZXUP_A5AD)// ��ʼ��ַ����
#define ZXUP_A5AE_ADDR    (ZXUP_A5AD_ADDR+SIZE_OF_ZXUP_A5AD)    // ��ʼ��ַ
#define ZXUP_A5AE_POS1    0            // �������1
#define ZXUP_A5AE_POS2    (ZXUP_A5AE_POS1+1)  // �������2
#define ZXUP_A5AE_POS3    (ZXUP_A5AE_POS2+1)  // ��ؽ��
#define ZXUP_A5AE_POS4    (ZXUP_A5AE_POS3+1)  // �һ�����1
#define ZXUP_A5AE_POS5    (ZXUP_A5AE_POS4+1)  // �һ�����2
#define ZXUP_A5AE_POS6    (ZXUP_A5AE_POS5+1)  // �һؽ��
#define SIZE_OF_ZXUP_A5AE      (ZXUP_A5AE_POS6+1)  // ���ֽ���(6B)

//==TAG-A5AF�������߼���Ϣ========================================================
#define zxup_buffer_a5af       (zxup_buffer+SIZE_OF_ZXUP_A5AE)// ��ʼ��ַ����
#define ZXUP_A5AF_ADDR    (ZXUP_A5AE_ADDR+SIZE_OF_ZXUP_A5AE)    // ��ʼ��ַ
#define ZXUP_A5AF_POS1    0          // �������������ƺͽ��
#define SIZE_OF_ZXUP_A5AF      (ZXUP_A5AF_POS1+5)  // ���ֽ���(5B)

//==TAG-A5B0�������߼���Ϣ========================================================
#define zxup_buffer_a5b0       (zxup_buffer+SIZE_OF_ZXUP_A5AF)// ��ʼ��ַ����
#define ZXUP_A5B0_ADDR    (ZXUP_A5AF_ADDR+SIZE_OF_ZXUP_A5AF)    // ��ʼ��ַ
#define ZXUP_A5B0_POS1  0            // �������������ƺͽ��
#define SIZE_OF_ZXUP_A5B0      (ZXUP_A5B0_POS1+4)  // ���ֽ���(4B)

//==TAG-A5B1�����߼���Ϣ==========================================================
#define zxup_buffer_a5b1       (zxup_buffer+SIZE_OF_ZXUP_A5B0)// ��ʼ��ַ����
#define ZXUP_A5B1_ADDR    (ZXUP_A5B0_ADDR+SIZE_OF_ZXUP_A5B0)    // ��ʼ��ַ
#define ZXUP_A5B1_POS1    0             // ����Ŀ��չ���Ƕ�
#define ZXUP_A5B1_POS2    (ZXUP_A5B1_POS1+1)  // ����Ŀ���Ž��Ƕ�
#define ZXUP_A5B1_POS3    (ZXUP_A5B1_POS2+2)  // ����һ���Ž������߼�
#define ZXUP_A5B1_POS4    (ZXUP_A5B1_POS2+2)  // �Ž�����״̬λ
#define SIZE_OF_ZXUP_A5B1      (ZXUP_A5B1_POS4+1)  // ���ֽ���(6B)

//==TAG-A5B2�����߼���Ϣ==========================================================
#define zxup_buffer_a5b2       (zxup_buffer+SIZE_OF_ZXUP_A5B1)// ��ʼ��ַ����
#define ZXUP_A5B2_ADDR    (ZXUP_A5B1_ADDR+SIZE_OF_ZXUP_A5B1)    // ��ʼ��ַ
#define ZXUP_A5B2_POS1    0                        // ����������
#define ZXUP_A5B2_POS2    (ZXUP_A5B2_POS1+2)  // ����������
#define ZXUP_A5B2_POS3    (ZXUP_A5B2_POS2+2)  // �����������
#define ZXUP_A5B2_POS4    (ZXUP_A5B2_POS3+1)  // ������������
#define ZXUP_A5B2_POS5    (ZXUP_A5B2_POS4+1)  // �������Ʊ����
#define ZXUP_A5B2_POS6    (ZXUP_A5B2_POS5+1)  // �������Ʊ����
#define ZXUP_A5B2_POS7    (ZXUP_A5B2_POS6+1)  // ��������������
#define ZXUP_A5B2_POS8    (ZXUP_A5B2_POS7+1)  // ��������������
#define SIZE_OF_ZXUP_A5B2      (ZXUP_A5B2_POS8+1)  // ���ֽ���(10B)

//==TAG-A5B3Һѹ���¶�============================================================
#define zxup_buffer_a5b3       (zxup_buffer+SIZE_OF_ZXUP_A5B2)// ��ʼ��ַ����
#define ZXUP_A5B3_ADDR    (ZXUP_A5B2_ADDR+SIZE_OF_ZXUP_A5B2)    // ��ʼ��ַ
#define ZXUP_A5B3_POS1    0                        // Һѹ���¶�
#define ZXUP_A5B3_POS2    (ZXUP_A5B3_POS1+2)  // Һѹ��λ
#define ZXUP_A5B3_POS3    (ZXUP_A5B3_POS2+1)  // ������������
#define SIZE_OF_ZXUP_A5B3      (ZXUP_A5B3_POS3+1)  // ���ֽ���(4B)

//==TAG-A5B4������Ϣ==============================================================
#define zxup_buffer_a5b4       (zxup_buffer+SIZE_OF_ZXUP_A5B3)// ��ʼ��ַ����
#define ZXUP_A5B4_ADDR    (ZXUP_A5B3_ADDR+SIZE_OF_ZXUP_A5B3)    // ��ʼ��ַ
#define ZXUP_A5B4_POS1    0                        // 1#���õ�ŷ�
#define ZXUP_A5B4_POS2    (ZXUP_A5B4_POS1+2)  // 2#���õ�ŷ�
#define ZXUP_A5B4_POS3    (ZXUP_A5B4_POS2+2)  // ����ѹ��
#define SIZE_OF_ZXUP_A5B4      (ZXUP_A5B4_POS3+2)  // ���ֽ���(6B)

//==TAG-A5B5����1 XHVME4400P1=====================================================
#define zxup_buffer_a5b5       (zxup_buffer+SIZE_OF_ZXUP_A5B4)// ��ʼ��ַ����
#define ZXUP_A5B5_ADDR    (ZXUP_A5B4_ADDR+SIZE_OF_ZXUP_A5B4)    // ��ʼ��ַ
#define ZXUP_A5B5_POS1    0                        // ���ŷ�
#define ZXUP_A5B5_POS2    (ZXUP_A5B5_POS1+2)  // ����ŷ�
#define ZXUP_A5B5_POS3    (ZXUP_A5B5_POS2+2)  // ������ŷ�
#define ZXUP_A5B5_POS4    (ZXUP_A5B5_POS3+2)  // ������ŷ�
#define ZXUP_A5B5_POS5    (ZXUP_A5B5_POS4+2)  // �������ŷ�
#define ZXUP_A5B5_POS6    (ZXUP_A5B5_POS5+2)  // �������ŷ�
#define ZXUP_A5B5_POS7    (ZXUP_A5B5_POS6+2)  // �������ŷ�
#define ZXUP_A5B5_POS8    (ZXUP_A5B5_POS7+2)  //�������ŷ�
#define ZXUP_A5B5_POS9    (ZXUP_A5B5_POS8+2)  // MP1ѹ��
#define ZXUP_A5B5_POS10   (ZXUP_A5B5_POS9+2)  // LS1ѹ��
#define ZXUP_A5B5_POS11   (ZXUP_A5B5_POS10+2)  // MP2ѹ��
#define ZXUP_A5B5_POS12   (ZXUP_A5B5_POS11+2)  // LS2ѹ��
#define ZXUP_A5B5_POS13   (ZXUP_A5B5_POS12+2)  // ������ŷ�
#define SIZE_OF_ZXUP_A5B5      (ZXUP_A5B5_POS13+1)  // ���ֽ���(23B)

//==TAG-A5B6����3 ���λ==========================================================
#define zxup_buffer_a5b6       (zxup_buffer+SIZE_OF_ZXUP_A5B5)// ��ʼ��ַ����
#define ZXUP_A5B6_ADDR    (ZXUP_A5B5_ADDR+SIZE_OF_ZXUP_A5B5)    // ��ʼ��ַ
#define ZXUP_A5B6_POS1    0                        // ״̬λ
#define SIZE_OF_ZXUP_A5B6      (ZXUP_A5B6_POS1+1)  // ���ֽ���(1B)

//==TAG-A5B7������Ϣ==============================================================
#define zxup_buffer_a5b7       (zxup_buffer+SIZE_OF_ZXUP_A5B6)// ��ʼ��ַ����
#define ZXUP_A5B7_ADDR    (ZXUP_A5B6_ADDR+SIZE_OF_ZXUP_A5B6)    // ��ʼ��ַ
#define ZXUP_A5B7_POS1    0                        // ������ѹ��
#define ZXUP_A5B7_POS2    (ZXUP_A5B7_POS1+2)  // �����׳���
#define ZXUP_A5B7_POS3    (ZXUP_A5B7_POS2+2)  // ���ƽ�ⷧ
#define SIZE_OF_ZXUP_A5B7      (ZXUP_A5B7_POS3+2)  // ���ֽ���(6B)

//==TAG-A5B8�����߼���Ϣ==========================================================
#define zxup_buffer_a5b8       (zxup_buffer+SIZE_OF_ZXUP_A5B7)// ��ʼ��ַ����
#define ZXUP_A5B8_ADDR    (ZXUP_A5B7_ADDR+SIZE_OF_ZXUP_A5B7)    // ��ʼ��ַ
#define ZXUP_A5B8_POS1    0                        // �������ͽ���״̬λ
#define ZXUP_A5B8_POS2    (ZXUP_A5B8_POS1+1)  // ͷ���־
#define ZXUP_A5B8_POS3    (ZXUP_A5B8_POS2+1)  // ǰ��ͷ���⿪��8
#define ZXUP_A5B8_POS4    (ZXUP_A5B8_POS3+1)  // ���ͷ���⿪��8
#define ZXUP_A5B8_POS5    (ZXUP_A5B8_POS4+1)  // ��λ
#define SIZE_OF_ZXUP_A5B8      (ZXUP_A5B8_POS5+1)  // ���ֽ���(5B)

//==TAG-A5B9�ױ������Ʒ�==========================================================
#define zxup_buffer_a5b9       (zxup_buffer+SIZE_OF_ZXUP_A5B8)// ��ʼ��ַ����
#define ZXUP_A5B9_ADDR    (ZXUP_A5B8_ADDR+SIZE_OF_ZXUP_A5B8)    // ��ʼ��ַ
#define ZXUP_A5B9_POS1    0                        // ״̬�ֽ�
#define ZXUP_A5B9_POS2    (ZXUP_A5B9_POS1+1)  // ������ѹ��
#define SIZE_OF_ZXUP_A5B9      (ZXUP_A5B9_POS2+2)  // ���ֽ���(3B)

//==TAG-A5BA���ƽ�ⷧ============================================================
#define zxup_buffer_a5ba       (zxup_buffer+SIZE_OF_ZXUP_A5B9)// ��ʼ��ַ����
#define ZXUP_A5BA_ADDR    (ZXUP_A5B9_ADDR+SIZE_OF_ZXUP_A5B9)    // ��ʼ��ַ
#define ZXUP_A5BA_POS1    0                        // ����ƽ�ⷧ����
#define ZXUP_A5BA_POS2    (ZXUP_A5BA_POS1+2)  // �ұ��ƽ�ⷧ����
#define SIZE_OF_ZXUP_A5BA      (ZXUP_A5BA_POS2+2)  // ���ֽ���(4B)

//==TAG-A5BB�����================================================================
#define zxup_buffer_a5bb       (zxup_buffer+SIZE_OF_ZXUP_A5BA)// ��ʼ��ַ����
#define ZXUP_A5BB_ADDR    (ZXUP_A5BA_ADDR+SIZE_OF_ZXUP_A5BA)   // ��ʼ��ַ
#define ZXUP_A5BB_POS1    0                        // ������ŷ�
#define ZXUP_A5BB_POS2    (ZXUP_A5BB_POS1+2)  // �����ŷ�
#define ZXUP_A5BB_POS3    (ZXUP_A5BB_POS2+2)  // �ͱ�ѹ��
#define SIZE_OF_ZXUP_A5BB      (ZXUP_A5BB_POS3+2)  // ���ֽ���(6B)

//==TAG-A5BC�������==============================================================
#define zxup_buffer_a5bc       (zxup_buffer+SIZE_OF_ZXUP_A5BB)// ��ʼ��ַ����
#define ZXUP_A5BC_ADDR    (ZXUP_A5BB_ADDR+SIZE_OF_ZXUP_A5BB)    // ��ʼ��ַ
#define ZXUP_A5BC_POS1    0                        // ������
#define ZXUP_A5BC_POS2    (ZXUP_A5BC_POS1+2)  // ��Ͳת��
#define ZXUP_A5BC_POS3    (ZXUP_A5BC_POS2+2)  // ״̬λ
#define SIZE_OF_ZXUP_A5BC      (ZXUP_A5BC_POS3+1)  // ���ֽ���(5B)

//==TAG-A5BD�����================================================================
#define zxup_buffer_a5bd       (zxup_buffer+SIZE_OF_ZXUP_A5BC)// ��ʼ��ַ����
#define ZXUP_A5BD_ADDR    (ZXUP_A5BC_ADDR+SIZE_OF_ZXUP_A5BC)    // ��ʼ��ַ
#define ZXUP_A5BD_POS1    0                        // ������ŷ�
#define ZXUP_A5BD_POS2    (ZXUP_A5BD_POS1+2)  // �����ŷ�
#define ZXUP_A5BD_POS3    (ZXUP_A5BD_POS2+2)  // �ͱ�ѹ��
#define SIZE_OF_ZXUP_A5BD      (ZXUP_A5BD_POS3+2)  // ���ֽ���(6B)

//==TAG-A5BE�������==============================================================
#define zxup_buffer_a5be       (zxup_buffer+SIZE_OF_ZXUP_A5BD)// ��ʼ��ַ����
#define ZXUP_A5BE_ADDR    (ZXUP_A5BD_ADDR+SIZE_OF_ZXUP_A5BD)    // ��ʼ��ַ
#define ZXUP_A5BE_POS1    0                        // ������
#define ZXUP_A5BE_POS2    (ZXUP_A5BE_POS1+2)  // ��Ͳת��
#define ZXUP_A5BE_POS3    (ZXUP_A5BE_POS2+2)  // ״̬λ
#define SIZE_OF_ZXUP_A5BE      (ZXUP_A5BE_POS3+1)  // ���ֽ���(5B)

//==TAG-A5BF��ת��================================================================
#define zxup_buffer_a5bf       (zxup_buffer+SIZE_OF_ZXUP_A5BE)// ��ʼ��ַ����
#define ZXUP_A5BF_ADDR    (ZXUP_A5BE_ADDR+SIZE_OF_ZXUP_A5BE)    // ��ʼ��ַ
#define ZXUP_A5BF_POS1    0                        // ���ת��ŷ�
#define ZXUP_A5BF_POS2    (ZXUP_A5BF_POS1+2)  // �һ�ת��ŷ�
#define ZXUP_A5BF_POS3    (ZXUP_A5BF_POS2+2)  // ��ת���巧
#define ZXUP_A5BF_POS4    (ZXUP_A5BF_POS3+2)  // �ͱ�ѹ������תѹ����⣩
#define SIZE_OF_ZXUP_A5BF      (ZXUP_A5BF_POS4+2)  // ���ֽ���(8B)

//==TAG-A5C0��ת�ƶ���============================================================
#define zxup_buffer_a5c0       (zxup_buffer+SIZE_OF_ZXUP_A5BF)// ��ʼ��ַ����
#define ZXUP_A5C0_ADDR    (ZXUP_A5BF_ADDR+SIZE_OF_ZXUP_A5BF)    // ��ʼ��ַ
#define ZXUP_A5C0_POS1    0                        // �ֽ�״̬λ
#define ZXUP_A5C0_POS2    (ZXUP_A5C0_POS1+1)  // ��Ͳת��
#define SIZE_OF_ZXUP_A5C0      (ZXUP_A5C0_POS2+2)  // ���ֽ���(3B)

//==TAG-A5C1������1===============================================================
#define zxup_buffer_a5c1       (zxup_buffer+SIZE_OF_ZXUP_A5C0)// ��ʼ��ַ����
#define ZXUP_A5C1_ADDR    (ZXUP_A5C0_ADDR+SIZE_OF_ZXUP_A5C0)    // ��ʼ��ַ
#define ZXUP_A5C1_POS1    0                        // �ֽ�״̬λ1
#define ZXUP_A5C1_POS2    (ZXUP_A5C1_POS1+1)  // �ֽ�״̬λ2
#define SIZE_OF_ZXUP_A5C1      (ZXUP_A5C1_POS2+1)  // ���ֽ���(2B)

//==TAG-A5C2������2(�����λ)=====================================================
#define zxup_buffer_a5c2       (zxup_buffer+SIZE_OF_ZXUP_A5C1)// ��ʼ��ַ����
#define ZXUP_A5C2_ADDR    (ZXUP_A5C1_ADDR+SIZE_OF_ZXUP_A5C1)    // ��ʼ��ַ
#define ZXUP_A5C2_POS1    0                        // ѹ��ѡ��
#define ZXUP_A5C2_POS2    (ZXUP_A5C2_POS1+2)  // �ֽ�״̬λ
#define SIZE_OF_ZXUP_A5C2      (ZXUP_A5C2_POS2+1)  // ���ֽ���(3B)

//==TAG-A5C3������============================================================
#define zxup_buffer_a5c3       (zxup_buffer+SIZE_OF_ZXUP_A5C2)// ��ʼ��ַ����
#define ZXUP_A5C3_ADDR    (ZXUP_A5C2_ADDR+SIZE_OF_ZXUP_A5C2)    // ��ʼ��ַ
#define ZXUP_A5C3_POS1    0                        // ������
#define ZXUP_A5C3_POS2    (ZXUP_A5C3_POS1+2)  // ������
#define ZXUP_A5C3_POS3    (ZXUP_A5C3_POS2+2)  // ����������
#define ZXUP_A5C3_POS4    (ZXUP_A5C3_POS3+2)  // �ֽ�״̬λ1
#define ZXUP_A5C3_POS5    (ZXUP_A5C3_POS4+1)  // �ֽ�״̬λ2
#define ZXUP_A5C3_POS6    (ZXUP_A5C3_POS5+1)  // �Ž�����
#define ZXUP_A5C3_POS7    (ZXUP_A5C3_POS6+2)  // �Ž�����
#define SIZE_OF_ZXUP_A5C3      (ZXUP_A5C3_POS7+2)  // ���ֽ���(12B)

//==TAG-A5C4�ҳ�����============================================================
#define zxup_buffer_a5c4       (zxup_buffer+SIZE_OF_ZXUP_A5C3)// ��ʼ��ַ����
#define ZXUP_A5C4_ADDR    (ZXUP_A5C3_ADDR+SIZE_OF_ZXUP_A5C3)    // ��ʼ��ַ
#define ZXUP_A5C4_POS1    0                        // ������
#define ZXUP_A5C4_POS2    (ZXUP_A5C4_POS1+2)  // ������
#define ZXUP_A5C4_POS3    (ZXUP_A5C4_POS2+2)  // ����������
#define ZXUP_A5C4_POS4    (ZXUP_A5C4_POS3+2)  // �ֽ�״̬λ1
#define ZXUP_A5C4_POS5    (ZXUP_A5C4_POS4+1)  // �ֽ�״̬λ2
#define ZXUP_A5C4_POS6    (ZXUP_A5C4_POS5+1)  // �Ž�����
#define ZXUP_A5C4_POS7    (ZXUP_A5C4_POS6+2)  // �Ž�����
#define SIZE_OF_ZXUP_A5C4      (ZXUP_A5C4_POS7+2)  // ���ֽ���(12B)

//==TAG-A5C8��ҵ�ͺ���Ϣ==========================================================
#define zxup_buffer_a5c8              (zxup_buffer+SIZE_OF_ZXUP_A5C4)// ��ʼ��ַ����
#define ZXUP_A5C8_ADDR    (ZXUP_A5C4_ADDR+SIZE_OF_ZXUP_A5C4)    // ��ʼ��ַ
#define ZXUP_A5C8_POS1    0                        // ��ҵ������ʱ��
#define ZXUP_A5C8_POS2    (ZXUP_A5C8_POS1+4)  // ��ҵȼ����������
#define ZXUP_A5C8_POS3    (ZXUP_A5C8_POS2+4)  // ��ҵƽ���ͺ�
#define SIZE_OF_ZXUP_A5C8      (ZXUP_A5C8_POS3+2)  // ���ֽ���(10B)

//==TAG-A5C9 ECU��Ӧ����CAN֡=====================================================
#define zxup_buffer_a5c9       (zxup_buffer+SIZE_OF_ZXUP_A5C8)// ��ʼ��ַ����
#define ZXUP_A5C9_ADDR    (ZXUP_A5C8_ADDR+SIZE_OF_ZXUP_A5C8)    // ��ʼ��ַ
#define ZXUP_A5C9_POS1    0                        // ��ҵ������ʱ��
#define ZXUP_A5C9_POS2    (ZXUP_A5C9_POS1+1)  // ��ҵȼ����������
#define ZXUP_A5C9_POS3    (ZXUP_A5C9_POS2+4)  // ��ҵƽ���ͺ�
#define SIZE_OF_ZXUP_A5C9      (ZXUP_A5C9_POS3+8)  // ���ֽ���(13B)

// �ϳ����ݻ����С
#define SIZE_OF_ZXUP_BUFFER    (ZXUP_A5C9_ADDR+SIZE_OF_ZXUP_A5C9) //(zxup_buffer_a5c9+SIZE_OF_ZXUP_A5C9-zxup_buffer_a5a0)

/******************************************************************************
* Macros(�³�ͨ��): ���ػ��³�������ź͵�ַ����
******************************************************************************/
//==TAG-A5E0�ڵ�״̬==============================================================
#define zxdown_buffer_a5e0          (zxdown_buffer)   // ��ʼ��ַ����
#define ZXDOWN_A5E0_ADDR    0                          // ��ʼ��ַ
#define ZXDOWN_A5E0_POS1    0                          // �ֽ�״̬λ1
#define ZXDOWN_A5E0_POS2    (ZXDOWN_A5E0_POS1+1)  // �ֽ�״̬λ2
#define SIZE_OF_ZXDOWN_A5E0      (ZXDOWN_A5E0_POS2+1)  // ���ֽ���(2B)

//==TAG-A5E1����ϵͳ==============================================================
#define zxdown_buffer_a5e1       (zxdown_buffer+SIZE_OF_ZXDOWN_A5E0) // ��ʼ��ַ����
#define ZXDOWN_A5E1_ADDR    (ZXDOWN_A5E0_ADDR+SIZE_OF_ZXDOWN_A5E0)                          // ��ʼ��ַ
#define ZXDOWN_A5E1_POS1    0                          // �����䵵λ
#define ZXDOWN_A5E1_POS2    (ZXDOWN_A5E1_POS1+1)  // �ֽ�״̬λ
#define ZXDOWN_A5E1_POS3    (ZXDOWN_A5E1_POS2+1)  // ����������
#define ZXDOWN_A5E1_POS4    (ZXDOWN_A5E1_POS3+2)  // �����������ѹ
#define ZXDOWN_A5E1_POS5    (ZXDOWN_A5E1_POS4+1)  // �����λ��
#define SIZE_OF_ZXDOWN_A5E1      (ZXDOWN_A5E1_POS5+1)  // ���ֽ���(6B)

//==TAG-A5E2֧�������Ϣ==========================================================
#define zxdown_buffer_a5e2       (zxdown_buffer+SIZE_OF_ZXDOWN_A5E1)// ��ʼ��ַ����
#define ZXDOWN_A5E2_ADDR    (ZXDOWN_A5E1_ADDR+SIZE_OF_ZXDOWN_A5E1)                          // ��ʼ��ַ
#define ZXDOWN_A5E2_POS1    0                          // �ֽ�״̬λ1
#define ZXDOWN_A5E2_POS2    (ZXDOWN_A5E2_POS1+1)  // �ֽ�״̬λ2
#define ZXDOWN_A5E2_POS3    (ZXDOWN_A5E2_POS2+1)  // �ֽ�״̬λ3
#define SIZE_OF_ZXDOWN_A5E2      (ZXDOWN_A5E2_POS3+1)  // ���ֽ���(3B)

//==TAG-A5E3����ϵͳ==============================================================
#define zxdown_buffer_a5e3       (zxdown_buffer+SIZE_OF_ZXDOWN_A5E2)// ��ʼ��ַ����
#define ZXDOWN_A5E3_ADDR    (ZXDOWN_A5E2_ADDR+SIZE_OF_ZXDOWN_A5E2)                          // ��ʼ��ַ
#define ZXDOWN_A5E3_POS1    0                          // ����ѹ��
#define ZXDOWN_A5E3_POS2    (ZXDOWN_A5E3_POS1+8)  // �����г�
#define ZXDOWN_A5E3_POS3    (ZXDOWN_A5E3_POS2+8)  // ��������
#define ZXDOWN_A5E3_POS4    (ZXDOWN_A5E3_POS3+4)  // �ֽ�״̬λ1
#define ZXDOWN_A5E3_POS5    (ZXDOWN_A5E3_POS4+1)  // �ֽ�״̬λ2
#define SIZE_OF_ZXDOWN_A5E3      (ZXDOWN_A5E3_POS5+1)  // ���ֽ���(22B)

//==TAG-A5E4ת��ϵͳ��ȫ���棩====================================================
#define zxdown_buffer_a5e4       (zxdown_buffer+SIZE_OF_ZXDOWN_A5E3)// ��ʼ��ַ����
#define ZXDOWN_A5E4_ADDR    (ZXDOWN_A5E3_ADDR+SIZE_OF_ZXDOWN_A5E3)                          // ��ʼ��ַ
#define ZXDOWN_A5E4_POS1    0                          // һ����ת��Ƕ�
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
#define SIZE_OF_ZXDOWN_A5E4      (ZXDOWN_A5E4_POS11+1)  // ���ֽ���(51B)

//==TAG-A5E5ת��ϵͳ��������======================================================
#define zxdown_buffer_a5e5       (zxdown_buffer+SIZE_OF_ZXDOWN_A5E4)// ��ʼ��ַ����
#define ZXDOWN_A5E5_ADDR    (ZXDOWN_A5E4_ADDR+SIZE_OF_ZXDOWN_A5E4)                          // ��ʼ��ַ
#define ZXDOWN_A5E5_POS1    0                          // ��ǰһ��ת��
#define ZXDOWN_A5E5_POS2    (ZXDOWN_A5E5_POS1+2)  // ת������ѹ��(bar)
#define ZXDOWN_A5E5_POS3    (ZXDOWN_A5E5_POS2+2)  // Ŀ��ת��ģʽ+��ǰת��ģʽ
#define ZXDOWN_A5E5_POS4    (ZXDOWN_A5E5_POS3+2)  // ������λ
#define ZXDOWN_A5E5_POS5    (ZXDOWN_A5E5_POS4+1)  // �ֽ�״̬λ1
#define ZXDOWN_A5E5_POS6    (ZXDOWN_A5E5_POS5+1)  // �ֽ�״̬λ2
#define SIZE_OF_ZXDOWN_A5E5      (ZXDOWN_A5E5_POS6+1)  // ���ֽ���(9B)

//==TAG-A5E6�ƶ�ϵͳ==============================================================
#define zxdown_buffer_a5e6       (zxdown_buffer+SIZE_OF_ZXDOWN_A5E5)// ��ʼ��ַ����
#define ZXDOWN_A5E6_ADDR    (ZXDOWN_A5E5_ADDR+SIZE_OF_ZXDOWN_A5E5)                          // ��ʼ��ַ
#define ZXDOWN_A5E6_POS1    0                          // ��·һ��ѹ
#define ZXDOWN_A5E6_POS2    (ZXDOWN_A5E6_POS1+2)  // ��·����ѹ
#define ZXDOWN_A5E6_POS3    (ZXDOWN_A5E6_POS2+2)  // �ƶ�ѹ��
#define ZXDOWN_A5E6_POS4    (ZXDOWN_A5E6_POS3+2)  // ��ѹ���
#define ZXDOWN_A5E6_POS5    (ZXDOWN_A5E6_POS4+2)  // ����������
#define ZXDOWN_A5E6_POS6    (ZXDOWN_A5E6_POS5+2)  // �������ذٷֱ�
#define ZXDOWN_A5E6_POS7    (ZXDOWN_A5E6_POS6+1)  // ���������
#define ZXDOWN_A5E6_POS8    (ZXDOWN_A5E6_POS7+2)  // �ֽ�״̬λ1
#define ZXDOWN_A5E6_POS9    (ZXDOWN_A5E6_POS8+1)  // �ֽ�״̬λ2
#define SIZE_OF_ZXDOWN_A5E6      (ZXDOWN_A5E6_POS9+1)  // ���ֽ���(15B)

//==TAG-A5E7����ϵͳ==============================================================
#define zxdown_buffer_a5e7       (zxdown_buffer+SIZE_OF_ZXDOWN_A5E6)// ��ʼ��ַ����
#define ZXDOWN_A5E7_ADDR    (ZXDOWN_A5E6_ADDR+SIZE_OF_ZXDOWN_A5E6)                          // ��ʼ��ַ
#define ZXDOWN_A5E7_POS1    0                          // �ֽ�״̬λ
#define SIZE_OF_ZXDOWN_A5E7      (ZXDOWN_A5E7_POS1+1)  // ���ֽ���(1B)

//==TAG-A5E8����ȡ��ϵͳ==========================================================
#define zxdown_buffer_a5e8       (zxdown_buffer+SIZE_OF_ZXDOWN_A5E7)// ��ʼ��ַ����
#define ZXDOWN_A5E8_ADDR    (ZXDOWN_A5E7_ADDR+SIZE_OF_ZXDOWN_A5E7)                          // ��ʼ��ַ
#define ZXDOWN_A5E8_POS1    0                          // �ֽ�״̬λ1
#define ZXDOWN_A5E8_POS2    (ZXDOWN_A5E8_POS1+1)  // �ֽ�״̬λ2
#define SIZE_OF_ZXDOWN_A5E8      (ZXDOWN_A5E8_POS2+1)  // ���ֽ���(2B)

//==TAG-A5E9Һѹϵͳ==============================================================
#define zxdown_buffer_a5e9       (zxdown_buffer+SIZE_OF_ZXDOWN_A5E8)// ��ʼ��ַ����
#define ZXDOWN_A5E9_ADDR    (ZXDOWN_A5E8_ADDR+SIZE_OF_ZXDOWN_A5E8)                          // ��ʼ��ַ
#define ZXDOWN_A5E9_POS1    0                                       // Һѹ���¶�
#define ZXDOWN_A5E9_POS2    (ZXDOWN_A5E9_POS1+1)  // Һѹϵͳѹ��
#define SIZE_OF_ZXDOWN_A5E9      (ZXDOWN_A5E9_POS2+2)  // ���ֽ���(3B)

//==TAG-A5EA˫��������ϵͳ========================================================
#define zxdown_buffer_a5ea       (zxdown_buffer+SIZE_OF_ZXDOWN_A5E9)// ��ʼ��ַ����
#define ZXDOWN_A5EA_ADDR    (ZXDOWN_A5E9_ADDR+SIZE_OF_ZXDOWN_A5E9)                          // ��ʼ��ַ
#define ZXDOWN_A5EA_POS1    0                          // �ֽ�״̬λ1
#define ZXDOWN_A5EA_POS2    (ZXDOWN_A5EA_POS1+1)  // �ֽ�״̬λ2
#define ZXDOWN_A5EA_POS3    (ZXDOWN_A5EA_POS2+1)  // �ϳ�������ˮ��
#define ZXDOWN_A5EA_POS4    (ZXDOWN_A5EA_POS3+1)  // �ϳ�����������ѹ��
#define ZXDOWN_A5EA_POS5    (ZXDOWN_A5EA_POS4+2)  // �ϳ�������ת��
#define ZXDOWN_A5EA_POS6    (ZXDOWN_A5EA_POS5+2)  // �ϳ��ó���ѹ��
#define ZXDOWN_A5EA_POS7    (ZXDOWN_A5EA_POS6+2)  // �����Ƶ���
#define ZXDOWN_A5EA_POS8    (ZXDOWN_A5EA_POS7+2)  // ɢ����Һѹ�����ÿ��Ƶ���
#define SIZE_OF_ZXDOWN_A5EA      (ZXDOWN_A5EA_POS8+2)  // ���ֽ���(13B)

//==TAG-A5EB��̥̥ѹ==============================================================
#define zxdown_buffer_a5eb       (zxdown_buffer+SIZE_OF_ZXDOWN_A5EA)// ��ʼ��ַ����
#define ZXDOWN_A5EB_ADDR    (ZXDOWN_A5EA_ADDR+SIZE_OF_ZXDOWN_A5EA)                          // ��ʼ��ַ
#define ZXDOWN_A5EB_POS1    0                          // ̥ѹ#1-6
#define ZXDOWN_A5EB_POS2    (ZXDOWN_A5EB_POS1+6)  // ̥ѹ#7-12
#define ZXDOWN_A5EB_POS3    (ZXDOWN_A5EB_POS2+6)  // ̥ѹ#13-18
#define SIZE_OF_ZXDOWN_A5EB      (ZXDOWN_A5EB_POS3+6)  // ���ֽ���(18B)

//==TAG-A5EC��֧�����============================================================
#define zxdown_buffer_a5ec       (zxdown_buffer+SIZE_OF_ZXDOWN_A5EB)// ��ʼ��ַ����
#define ZXDOWN_A5EC_ADDR    (ZXDOWN_A5EB_ADDR+SIZE_OF_ZXDOWN_A5EB)                          // ��ʼ��ַ
#define ZXDOWN_A5EC_POS1    0                          // �ֽ�״̬λ1
#define ZXDOWN_A5EC_POS2    (ZXDOWN_A5EC_POS1+1)  // �ֽ�״̬λ2
#define ZXDOWN_A5EC_POS3    (ZXDOWN_A5EC_POS2+1)  // �ֽ�״̬λ3
#define ZXDOWN_A5EC_POS4    (ZXDOWN_A5EC_POS3+1)  // �ֽ�״̬λ4
#define ZXDOWN_A5EC_POS5    (ZXDOWN_A5EC_POS4+1)  // �ֽ�״̬λ5
#define ZXDOWN_A5EC_POS6    (ZXDOWN_A5EC_POS5+1)  // Ԥ��
#define ZXDOWN_A5EC_POS7    (ZXDOWN_A5EC_POS6+1)  // DIR
#define ZXDOWN_A5EC_POS8    (ZXDOWN_A5EC_POS7+1)  // ��ƽ������
#define SIZE_OF_ZXDOWN_A5EC      (ZXDOWN_A5EC_POS8+1)  // ���ֽ���(8B)

//==TAG-A5ED��֧�����============================================================
#define zxdown_buffer_a5ed       (zxdown_buffer+SIZE_OF_ZXDOWN_A5EC)// ��ʼ��ַ����
#define ZXDOWN_A5ED_ADDR    (ZXDOWN_A5EC_ADDR+SIZE_OF_ZXDOWN_A5EC)                          // ��ʼ��ַ
#define ZXDOWN_A5ED_POS1    0                          // �ֽ�״̬λ1
#define ZXDOWN_A5ED_POS2    (ZXDOWN_A5ED_POS1+1)  // �ֽ�״̬λ2
#define ZXDOWN_A5ED_POS3    (ZXDOWN_A5ED_POS2+1)  // �ֽ�״̬λ3
#define ZXDOWN_A5ED_POS4    (ZXDOWN_A5ED_POS3+1)  // �ֽ�״̬λ4
#define ZXDOWN_A5ED_POS5    (ZXDOWN_A5ED_POS4+1)  // �ֽ�״̬λ5
#define ZXDOWN_A5ED_POS6    (ZXDOWN_A5ED_POS5+1)  // Ԥ��
#define ZXDOWN_A5ED_POS7    (ZXDOWN_A5ED_POS6+1)  // DIR
#define ZXDOWN_A5ED_POS8    (ZXDOWN_A5ED_POS7+1)  // ��ƽ������
#define SIZE_OF_ZXDOWN_A5ED      (ZXDOWN_A5ED_POS8+1)  // ���ֽ���(8B)

//==TAG-A5EE�п�̨������Ϣ========================================================
#define zxdown_buffer_a5ee       (zxdown_buffer+SIZE_OF_ZXDOWN_A5ED)// ��ʼ��ַ����
#define ZXDOWN_A5EE_ADDR    (ZXDOWN_A5ED_ADDR+SIZE_OF_ZXDOWN_A5ED)                          // ��ʼ��ַ
#define ZXDOWN_A5EE_POS1    0                          // �ֽ�״̬λ1
#define ZXDOWN_A5EE_POS2    (ZXDOWN_A5EE_POS1+1)  // �ֽ�״̬λ2
#define ZXDOWN_A5EE_POS3    (ZXDOWN_A5EE_POS2+1)  // �ֽ�״̬λ3
#define ZXDOWN_A5EE_POS4    (ZXDOWN_A5EE_POS3+1)  // �ֽ�״̬λ4
#define ZXDOWN_A5EE_POS5    (ZXDOWN_A5EE_POS4+1)  // �ֽ�״̬λ5
#define ZXDOWN_A5EE_POS6    (ZXDOWN_A5EE_POS5+1)  // �ֽ�״̬λ6
#define ZXDOWN_A5EE_POS7    (ZXDOWN_A5EE_POS6+1)  // �ֽ�״̬λ7
#define ZXDOWN_A5EE_POS8    (ZXDOWN_A5EE_POS7+1)  // ����ת��Ƕ�
#define ZXDOWN_A5EE_POS9    (ZXDOWN_A5EE_POS8+1)  // �ֽ�״̬λ8
#define SIZE_OF_ZXDOWN_A5EE      (ZXDOWN_A5EE_POS9+1)  // ���ֽ���(9B)

//==TAG-A5A3֧����ҵ��Ϣ==========================================================
#define zxdown_buffer_a5a3       (zxdown_buffer+SIZE_OF_ZXDOWN_A5EE)// ��ʼ��ַ����
#define ZXDOWN_A5A3_ADDR    (ZXDOWN_A5EE_ADDR+SIZE_OF_ZXDOWN_A5EE)                          // ��ʼ��ַ
#define ZXDOWN_A5A3_POS1    0                        // ˮƽ֧�ȳ���
#define ZXDOWN_A5A3_POS2    (ZXDOWN_A5A3_POS1+8)  // ֧��ѹ��
#define ZXDOWN_A5A3_POS3    (ZXDOWN_A5A3_POS2+8)  // �ڸ׳���
#define SIZE_OF_ZXDOWN_A5A3        (ZXDOWN_A5A3_POS3+8)  // ���ֽ���(24B)

//==TAG-A5A4����֧����ҵ��Ϣ=======================================================
#define zxdown_buffer_a5a4       (zxdown_buffer+SIZE_OF_ZXDOWN_A5A3)// ��ʼ��ַ����
#define ZXDOWN_A5A4_ADDR    (ZXDOWN_A5A3_ADDR+SIZE_OF_ZXDOWN_A5A3)                          // ��ʼ��ַ
#define ZXDOWN_A5A4_POS1    0                        // ����֧��״̬
#define ZXDOWN_A5A4_POS2    (ZXDOWN_A5A4_POS1+1)  // ��ǰ����֧��ѹ��
#define ZXDOWN_A5A4_POS3    (ZXDOWN_A5A4_POS2+2)  // ��ǰ����֧��ѹ��
#define SIZE_OF_ZXDOWN_A5A4      (ZXDOWN_A5A4_POS3+2)  // ���ֽ���(5B)

// �³��������ݻ����С
#define SIZE_OF_ZXDOWN_BUFFER   (ZXDOWN_A5A4_ADDR+SIZE_OF_ZXDOWN_A5A4)

/******************************************************************************
* Macros(���̷�����): ���ػ����̷�����������ź͵�ַ����
******************************************************************************/
//==TAG-A5EF���������в���(���ġ����塢����)======================================
#define zxengine_buffer_a5ef              zxengine_buffer  // ��ʼ��ַ����
#define ZXENGINE_A5EF_ADDR    0                            // ��ʼ��ַ
#define ZXENGINE_A5EF_POS1    0                            // ������ת��
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
#define SIZE_OF_ZXENGINE_A5EF      (ZXENGINE_A5EF_POS23+1)  // ���ֽ���(37B)

//==TAG-A5F0��ʻ�ͺ�==============================================================
#define zxengine_buffer_a5f0              (zxengine_buffer+SIZE_OF_ZXENGINE_A5EF)// ��ʼ��ַ����
#define ZXENGINE_A5F0_ADDR    (ZXENGINE_A5EF_ADDR+SIZE_OF_ZXENGINE_A5EF) // ��ʼ��ַ
#define ZXENGINE_A5F0_POS1    0                            // ��ʻ������ʱ��
#define ZXENGINE_A5F0_POS2    (ZXENGINE_A5F0_POS1+4)  // ��ʻȼ�����ͺ���
#define ZXENGINE_A5F0_POS3    (ZXENGINE_A5F0_POS2+4)  // �ٹ����ͺ�
#define SIZE_OF_ZXENGINE_A5F0      (ZXENGINE_A5F0_POS3+4)  // ���ֽ���(12B)

//==TAG-A5F1 SCR���������壩======================================================
#define zxengine_buffer_a5f1               (zxengine_buffer+SIZE_OF_ZXENGINE_A5F0)// ��ʼ��ַ����
#define ZXENGINE_A5F1_ADDR    (ZXENGINE_A5F0_ADDR+SIZE_OF_ZXENGINE_A5F0)                             // ��ʼ��ַ
#define ZXENGINE_A5F1_POS1    0                            // ��������״̬
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
#define SIZE_OF_ZXENGINE_A5F1      (ZXENGINE_A5F1_POS19+1)  // ���ֽ���(28B)

//==TAG-A5F2 DPF����(������=======================================================
#define zxengine_buffer_a5f2              (zxengine_buffer+SIZE_OF_ZXENGINE_A5F1)// ��ʼ��ַ����
#define ZXENGINE_A5F2_ADDR    (ZXENGINE_A5F1_ADDR+SIZE_OF_ZXENGINE_A5F1)                             // ��ʼ��ַ
#define ZXENGINE_A5F2_POS1    0                            // DOC���������¶�
#define ZXENGINE_A5F2_POS2    (ZXENGINE_A5F2_POS1+2)  // DPF���������¶�
#define ZXENGINE_A5F2_POS3    (ZXENGINE_A5F2_POS2+2)  // DPF̼����������
#define ZXENGINE_A5F2_POS4    (ZXENGINE_A5F2_POS3+1)  // DPFѹ��
#define ZXENGINE_A5F2_POS5    (ZXENGINE_A5F2_POS4+2)  // �ֽ�״̬λ1
#define ZXENGINE_A5F2_POS6    (ZXENGINE_A5F2_POS5+1)  // �ֽ�״̬λ2
#define ZXENGINE_A5F2_POS7    (ZXENGINE_A5F2_POS6+1)  // �ֽ�״̬λ3
#define ZXENGINE_A5F2_POS8    (ZXENGINE_A5F2_POS7+1)  // �ֽ�״̬λ4
#define SIZE_OF_ZXENGINE_A5F2      (ZXENGINE_A5F2_POS8+1)  // ���ֽ���(11B)

// �³����������ݻ����С
#define SIZE_OF_ZXENGINE_BUFFER   (ZXENGINE_A5F2_ADDR+SIZE_OF_ZXENGINE_A5F2)

/******************************************************************************
* Macros(ͳ������Ϣ): ������ź͵�ַ����
******************************************************************************/
//==TAG-A5C5 ����Ƶ��ͳ��1========================================================
#define zxstatistics_buffer_a5c5        zxstatistics_buffer
#define ZXSTATISTICS_A5C5_ADDR   0  // ��ʼ��ַ
#define ZXSTATISTICS_A5C5_POS1   0  // ������
#define ZXSTATISTICS_A5C5_POS2   (ZXSTATISTICS_A5C5_POS1+8)  // ������
#define ZXSTATISTICS_A5C5_POS3   (ZXSTATISTICS_A5C5_POS2+8)  // �����
#define ZXSTATISTICS_A5C5_POS4   (ZXSTATISTICS_A5C5_POS3+8)  // �����
#define ZXSTATISTICS_A5C5_POS5   (ZXSTATISTICS_A5C5_POS4+8)  // �����
#define ZXSTATISTICS_A5C5_POS6   (ZXSTATISTICS_A5C5_POS5+8)  // �������
#define ZXSTATISTICS_A5C5_POS7   (ZXSTATISTICS_A5C5_POS6+8)  // ��������
#define ZXSTATISTICS_A5C5_POS8   (ZXSTATISTICS_A5C5_POS7+8)  // ��������
#define ZXSTATISTICS_A5C5_POS9   (ZXSTATISTICS_A5C5_POS8+8)  // ��������
#define ZXSTATISTICS_A5C5_POS10  (ZXSTATISTICS_A5C5_POS9+8)  // ��������
#define ZXSTATISTICS_A5C5_POS11   (ZXSTATISTICS_A5C5_POS10+8)  // ���ת
#define ZXSTATISTICS_A5C5_POS12   (ZXSTATISTICS_A5C5_POS11+8)  // �һ�ת
#define SIZE_OF_ZXSTATISTICS_A5C5        (ZXSTATISTICS_A5C5_POS12+8)  // ���ֽ���(96B)

//==TAG-A5C6 ����Ƶ��ͳ��2========================================================
#define zxstatistics_buffer_a5c6        (zxstatistics_buffer+SIZE_OF_ZXSTATISTICS_A5C5)
#define ZXSTATISTICS_A5C6_ADDR   (ZXSTATISTICS_A5C5_ADDR+SIZE_OF_ZXSTATISTICS_A5C5)  // ��ʼ��ַ
#define ZXSTATISTICS_A5C6_POS1   0  // �ո���
#define ZXSTATISTICS_A5C6_POS2   (ZXSTATISTICS_A5C6_POS1+8)  // �ո���
#define ZXSTATISTICS_A5C6_POS3   (ZXSTATISTICS_A5C6_POS2+8)  // ������
#define ZXSTATISTICS_A5C6_POS4   (ZXSTATISTICS_A5C6_POS3+8)  // ������
#define ZXSTATISTICS_A5C6_POS5   (ZXSTATISTICS_A5C6_POS4+8)  // �����
#define ZXSTATISTICS_A5C6_POS6   (ZXSTATISTICS_A5C6_POS5+8)  // �������
#define ZXSTATISTICS_A5C6_POS7   (ZXSTATISTICS_A5C6_POS6+8)  // ��������
#define ZXSTATISTICS_A5C6_POS8   (ZXSTATISTICS_A5C6_POS7+8)  // ��������
#define ZXSTATISTICS_A5C6_POS9   (ZXSTATISTICS_A5C6_POS8+8)  // ��������
#define ZXSTATISTICS_A5C6_POS10   (ZXSTATISTICS_A5C6_POS9+8)  // ��������
#define ZXSTATISTICS_A5C6_POS11   (ZXSTATISTICS_A5C6_POS10+8)  // ���ת
#define ZXSTATISTICS_A5C6_POS12   (ZXSTATISTICS_A5C6_POS11+8)  // �һ�ת
#define SIZE_OF_ZXSTATISTICS_A5C6        (ZXSTATISTICS_A5C6_POS12+8)  // ���ֽ���(96B)

//==TAG-A5C7 ��ȫͳ��=============================================================
#define zxstatistics_buffer_a5c7        (zxstatistics_buffer+SIZE_OF_ZXSTATISTICS_A5C6)
#define ZXSTATISTICS_A5C7_ADDR   (ZXSTATISTICS_A5C6_ADDR+SIZE_OF_ZXSTATISTICS_A5C6)    // ��ʼ��ַ
#define ZXSTATISTICS_A5C7_POS1   0  // ����
#define ZXSTATISTICS_A5C7_POS2   (ZXSTATISTICS_A5C7_POS1+8)  // ��Ȧ
#define ZXSTATISTICS_A5C7_POS3   (ZXSTATISTICS_A5C7_POS2+8)  // ����
#define ZXSTATISTICS_A5C7_POS4   (ZXSTATISTICS_A5C7_POS3+8)  // ��ǿ��
#define ZXSTATISTICS_A5C7_POS5   (ZXSTATISTICS_A5C7_POS4+8)  // ��װ����
#define ZXSTATISTICS_A5C7_POS6   (ZXSTATISTICS_A5C7_POS5+8)  // �����ǿ��
#define ZXSTATISTICS_A5C7_POS7   (ZXSTATISTICS_A5C7_POS6+8)  // ����ǿ��
#define ZXSTATISTICS_A5C7_POS8   (ZXSTATISTICS_A5C7_POS7+8)  // ��Ȧǿ��
#define ZXSTATISTICS_A5C7_POS9   (ZXSTATISTICS_A5C7_POS8+8)  // ���ٳ���
#define SIZE_OF_ZXSTATISTICS_A5C7       (ZXSTATISTICS_A5C7_POS9+8)  // ���ֽ���(72B)

//==TAG-301E ����ʱ��ͳ������======================================================
#define zxstatistics_buffer_301e      (zxstatistics_buffer+SIZE_OF_ZXSTATISTICS_A5C7)
#define ZXSTATISTICS_301E_ADDR   (ZXSTATISTICS_A5C6_ADDR+SIZE_OF_ZXSTATISTICS_A5C6)   // ��ʼ��ַ
#define ZXSTATISTICS_301E_POS1_ADDR   0  // ������ʱ��
#define ZXSTATISTICS_301E_POS2_ADDR   (ZXSTATISTICS_301E_POS1_ADDR+4)  // ��������ʱ��
#define SIZE_OF_ZXSTATISTICS_301E       (ZXSTATISTICS_301E_POS2_ADDR+4)  // ���ֽ���(8B)

// ͳ�����ݻ����С
#define SIZE_OF_ZXSTATISTICS_BUFFER   (ZXSTATISTICS_301E_ADDR+SIZE_OF_ZXSTATISTICS_301E)

/******************************************************************************
* Macros(�ϳ����³��汾��Ϣ): ������ź͵�ַ����
******************************************************************************/
//==TAG-A505 �ϳ�ϵͳ�汾=========================================================
#define zxversion_buffer_a505        zxversion_buffer
#define ZXVERSION_A505_ADDR   0  // ��ʼ��ַ
#define ZXVERSION_A505_POS1   0  // ����������
#define ZXVERSION_A505_POS2   (ZXVERSION_A505_POS1+3)  // ��ʾ��1
#define ZXVERSION_A505_POS3   (ZXVERSION_A505_POS2+3)  // ��ʾ���ײ�汾
#define ZXVERSION_A505_POS4   (ZXVERSION_A505_POS3+3)  // GPS�ն�
#define ZXVERSION_A505_POS5   (ZXVERSION_A505_POS4+3)  // ������
#define ZXVERSION_A505_POS6   (ZXVERSION_A505_POS5+3)  // ��ʾ��2
#define ZXVERSION_A505_POS7   (ZXVERSION_A505_POS6+3)  // ��ʾ��2�ײ�
#define SIZE_OF_ZXVERSION_A505       (ZXVERSION_A505_POS7+3)  // ���ֽ���(18B)

//==TAG-A506 �³�ϵͳ�汾=========================================================
#define zxversion_buffer_a506        (zxversion_buffer+SIZE_OF_ZXVERSION_A505)
#define ZXVERSION_A506_ADDR   (ZXVERSION_A505_ADDR+SIZE_OF_ZXVERSION_A505)  // ��ʼ��ַ
#define ZXVERSION_A506_POS1   0   // ��ʾ��Ӧ�ò�
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
#define SIZE_OF_ZXVERSION_A506       (ZXVERSION_A506_POS18+3)  // ���ֽ���(54B)

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


// TLV״̬��Ч��־
bittype2 zxup_tlv_flag1;
bittype2 zxup_tlv_flag2;
bittype2 zxup_tlv_flag3;
bittype2 zxdown_tlv_flag1;
bittype2 zxdown_tlv_flag2;
bittype2 zxengine_tlv_flag;

uint8_t zxup_buffer[SIZE_OF_ZXUP_BUFFER]; /// �ϳ����ݻ���
uint8_t zxdown_buffer[SIZE_OF_ZXDOWN_BUFFER]; /// �³��������ݻ���
uint8_t zxengine_buffer[SIZE_OF_ZXENGINE_BUFFER]; /// �³����������ݻ���
uint8_t zxstatistics_buffer[SIZE_OF_ZXSTATISTICS_BUFFER]; ///ͳ�����ݻ���
uint8_t zxversion_buffer[SIZE_OF_ZXVERSION_BUFFER]; /// �汾��Ϣ����


/******************************************************************************
* ����TLV, ������Ϣ����
******************************************************************************/
uint16_t iZxM2m_BuildTlvMsg(uint8_t *pbuf, uint8_t tlv_valid_flag, uint16_t tag, uint16_t length, uint8_t* pValue)
{
  uint16_t len = 0;
  if (tlv_valid_flag)
  {
    pbuf[len++] = (tag>>8) & 0xFF; // TAG
    pbuf[len++] = tag & 0xFF;
    pbuf[len++] = (length>>8) & 0xFF; // LENGTH
    pbuf[len++] = length & 0xFF;
    memcpy(&pbuf[len], pValue, length); // VALUE
    len += length;
  }

  return len;
}


/******************************************************************************
 * Typedef
 ******************************************************************************/
typedef uint16_t (*Zxm2m_BuildTlvMsgFun)(uint8_t *pbuf);
//typedef uint16_t (*Zxm2m_AnalyzeTlvMsgFun)(uint8_t* pValue, uint16_t len);
typedef struct
{
  uint16_t type;
  Zxm2m_BuildTlvMsgFun pfun_build;
  //Zxm2m_AnalyzeTlvMsgFun pfun_analyze;
}Zxm2m_CmdTlv_t;

//==type���յ���˳����д=================================================
Zxm2m_CmdTlv_t Zxm2m_CmdDealTbl[]=
{
  /*TLV type ,build hdl ,analyze hdl*/
  
  /*****************�����ϳ�ͨ��TLV**********************************/
  {0xA5A0, iZxM2m_BuildTlvMsg_A5A0},// TLV-A5A0������������Ϣ(����)
  {0xA5A1, iZxM2m_BuildTlvMsg_A5A1},// TLV-A5A1���𹤿���Ϣ
  {0xA5A2, iZxM2m_BuildTlvMsg_A5A2},// TLV-A5A2���۹�����Ϣ
  {0xA5A3, iZxM2m_BuildTlvMsg_A5A3},// TLV-A5A3֧����ҵ��Ϣ
  {0xA5A4, iZxM2m_BuildTlvMsg_A5A4},// TLV-A5A4����֧����ҵ��Ϣ
  {0xA5A5, iZxM2m_BuildTlvMsg_A5A5},// TLV-A5A5�ϳ���������Ϣ
  {0xA5A6, iZxM2m_BuildTlvMsg_A5A6},// TLV-A5A6�ֱ���Ϣ
  {0xA5A7, iZxM2m_BuildTlvMsg_A5A7},// TLV-A5A7��ʾ��1��Ϣ
  {0xA5A8, iZxM2m_BuildTlvMsg_A5A8},// TLV-A5A8��ʾ��2��Ϣ
  {0xA5A9, iZxM2m_BuildTlvMsg_A5A9},// TTLV-A5A9���������Ϣ
  {0xA5AA, iZxM2m_BuildTlvMsg_A5AA},// TLV-A5AA���߲ٿ���Ϣ
  {0xA5AB, iZxM2m_BuildTlvMsg_A5AB},// TLV-A5AB����������Ϣ
  {0xA5AC, iZxM2m_BuildTlvMsg_A5AC},// TLV-A5AC�����߼���Ϣ
  {0xA5AD, iZxM2m_BuildTlvMsg_A5AD},// TLV-A5AD����߼���Ϣ  
  {0xA5AE, iZxM2m_BuildTlvMsg_A5AE},// TLV-A5AE��ת�߼���Ϣ
  {0xA5AF, iZxM2m_BuildTlvMsg_A5AF},// TLV-A5AF�������߼���Ϣ
  {0xA5B0, iZxM2m_BuildTlvMsg_A5B0},// TLV-A5B0�������߼���Ϣ
  {0xA5B1, iZxM2m_BuildTlvMsg_A5B1},// TLV-A5B1�����߼���Ϣ
  {0xA5B2, iZxM2m_BuildTlvMsg_A5B2},// TLV-A5B2�����߼���Ϣ
  {0xA5B3, iZxM2m_BuildTlvMsg_A5B3},// TLV-A5B3Һѹ���¶�  
  {0xA5B4, iZxM2m_BuildTlvMsg_A5B4},// TLV-A5B4������Ϣ
  {0xA5B5, iZxM2m_BuildTlvMsg_A5B5},// TLV-A5B5����1 XHVME4400P1
  {0xA5B6, iZxM2m_BuildTlvMsg_A5B6},// TLV-A5B6����3 ���λ
  {0xA5B7, iZxM2m_BuildTlvMsg_A5B7},// TLV-A5B7������Ϣ
  {0xA5B8, iZxM2m_BuildTlvMsg_A5B8},// TLV-A5B8�����߼���Ϣ
  {0xA5B9, iZxM2m_BuildTlvMsg_A5B9},// TLV-A5B9�ױ������Ʒ�  
  {0xA5BA, iZxM2m_BuildTlvMsg_A5BA},// TLV-A5BA���ƽ�ⷧ
  {0xA5BB, iZxM2m_BuildTlvMsg_A5BB},// TLV-A5BB�����
  {0xA5BC, iZxM2m_BuildTlvMsg_A5BC},// TLV-A5BC�������
  {0xA5BD, iZxM2m_BuildTlvMsg_A5BD},// TLV-A5BD�����
  {0xA5BE, iZxM2m_BuildTlvMsg_A5BE},// TLV-A5BE������� 
  {0xA5BF, iZxM2m_BuildTlvMsg_A5BF},// TLV-A5BF��ת��
  {0xA5C0, iZxM2m_BuildTlvMsg_A5C0},// TLV-A5C0��ת�ƶ���
  {0xA5C1, iZxM2m_BuildTlvMsg_A5C1},// TLV-A5C1������1
  {0xA5C2, iZxM2m_BuildTlvMsg_A5C2},// TLV-A5C2������2(�����λ)
  {0xA5C3, iZxM2m_BuildTlvMsg_A5C3},// TLV-A5C3������
  {0xA5C4, iZxM2m_BuildTlvMsg_A5C4},// TLV-A5C4�ҳ�����  
  {0xA5C8, iZxM2m_BuildTlvMsg_A5C8},// TLV-A5C8��ҵ�ͺ���Ϣ
  {0xA5C9, iZxM2m_BuildTlvMsg_A5C9},// TLV-A5C9 ECU��Ӧ����CAN֡
 
  /*****************�����³�ͨ��TLV**********************************/
  {0xA5E0, iZxM2m_BuildTlvMsg_A5E0},// TLV-A5E0�ڵ�״̬
  {0xA5E1, iZxM2m_BuildTlvMsg_A5E1},// TLV-A5E1����ϵͳ
  {0xA5E2, iZxM2m_BuildTlvMsg_A5E2},// TLV-A5E2֧�������Ϣ
  {0xA5E3, iZxM2m_BuildTlvMsg_A5E3},// TLV-A5E3����ϵͳ  
  {0xA5E4, iZxM2m_BuildTlvMsg_A5E4},// TLV-A5E4ת��ϵͳ(ȫ����)
  {0xA5E5, iZxM2m_BuildTlvMsg_A5E5},// TLV-A5E5ת��ϵͳ(����)
  {0xA5E6, iZxM2m_BuildTlvMsg_A5E6},// TLV-A5E6�ƶ�ϵͳ
  {0xA5E7, iZxM2m_BuildTlvMsg_A5E7},// TLV-A5E7����ϵͳ
  {0xA5E8, iZxM2m_BuildTlvMsg_A5E8},// TLV-A5E8����ȡ��ϵͳ
  {0xA5E9, iZxM2m_BuildTlvMsg_A5E9},// TLV-A5E9Һѹϵͳ
  {0xA5EA, iZxM2m_BuildTlvMsg_A5EA},// TLV-A5EA˫��������ϵͳ
  {0xA5EB, iZxM2m_BuildTlvMsg_A5EB},// TLV-A5EB��̥̥ѹ
  {0xA5EC, iZxM2m_BuildTlvMsg_A5EC},// TLV-A5EC��֧�����
  {0xA5ED, iZxM2m_BuildTlvMsg_A5ED},// TLV-A5ED��֧�����
  {0xA5EE, iZxM2m_BuildTlvMsg_A5EE},// TLV-A5EE�п�̨������Ϣ
  
  /*****************�������̷�����TLV*********************************/
  {0xA5EF, iZxM2m_BuildTlvMsg_A5EF},// TLV-A5EF���������в���(���ġ����塢����)
  {0xA5F0, iZxM2m_BuildTlvMsg_A5F0},// TLV-A5F0��ʻ�ͺ�
  {0xA5F1, iZxM2m_BuildTlvMsg_A5F1},// TLV-A5F1 SCR����(����)
  {0xA5F2, iZxM2m_BuildTlvMsg_A5F2},// TLV-A5F2 DPF����(����)

  /*****************�����汾��ϢTLV*********************************/
  {0xA505, iZxM2m_BuildTlvMsg_A505},// TLV-A505 �ϳ�ϵͳ�汾
  {0xA506, iZxM2m_BuildTlvMsg_A506},// TLV-A506 �³�ϵͳ�汾

  /*****************����Ƶ��ͳ��TLV*********************************/
  {0xA5C5, iZxM2m_BuildTlvMsg_A5C5},// TLV-A5C5 ����Ƶ��ͳ��1
  {0xA5C6, iZxM2m_BuildTlvMsg_A5C6},// TLV-A5C6 ����Ƶ��ͳ��2
  {0xA5C7, iZxM2m_BuildTlvMsg_A5C7},// TLV-A5C7 ��ȫͳ��
};
#define NUM_OF_ZXM2M_CMD_DEAL   (sizeof(Zxm2m_CmdDealTbl)/sizeof(Zxm2m_CmdDealTbl))





























���̵׵�����
A5A0-byte69��
A5A7-byte17&18��

A5B3-byte3��
A5B9-byte1&2��

A5C2-byte1&2��
A5C3-byte1-5��
A5C3-byte8-11��
A5C4-byte0-5��
A5C4-byte8-11��

A5E1-byte0��
A5E3-byte0-7��
A5E4-byte0-31��
A5E5-byte0-3��
A5E6-byte10-12��
A5EA-byte2��

A5EB-byte0-17��

A5EF-byte19.bit6��
A5EF-byte19.bit6��
A5EF-byte36.bit4��
A505-byte18-20


