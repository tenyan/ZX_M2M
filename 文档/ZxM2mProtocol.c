
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
* �����ϳ�ͨ��TLV
******************************************************************************/
//==TLV-A5A0������������Ϣ(����)==================================================
uint16_t iZxM2m_BuildTlvMsg_A5A0(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5a0_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xA0;
    pbuf[len++] = (SIZE_OF_ZXUP_A5A0>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5A0 & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5a0, SIZE_OF_ZXUP_A5A0); // VALUE
    len += SIZE_OF_ZXUP_A5A0;
  }

  return len;
}

//==TLV-A5A1���𹤿���Ϣ==========================================================
uint16_t iZxM2m_BuildTlvMsg_A5A1(uint8_t *pbuf)
{
  uint16_t len = 0;
  if (tlv_a5a1_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xA1;
    pbuf[len++] = (SIZE_OF_ZXUP_A5A1>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5A1 & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5a1, SIZE_OF_ZXUP_A5A1); // VALUE
    len += SIZE_OF_ZXUP_A5A1;
  }

  return len;
}

//==TLV-A5A2���۹�����Ϣ==========================================================
uint16_t iZxM2m_BuildTlvMsg_A5A2(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5a2_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xA2;
    pbuf[len++] = (SIZE_OF_ZXUP_A5A2>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5A2 & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5a2, SIZE_OF_ZXUP_A5A2); // VALUE
    len += SIZE_OF_ZXUP_A5A2;
  }

  return len;
}

//==TLV-A5A3֧����ҵ��Ϣ=========================================================
uint16_t iZxM2m_BuildTlvMsg_A5A3(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5a3_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xA3;
    pbuf[len++] = (SIZE_OF_ZXUP_A5A3>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5A3 & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5a3, SIZE_OF_ZXUP_A5A3); // VALUE
    len += SIZE_OF_ZXUP_A5A3;
  }

  return len;
}

//==TLV-A5A4����֧����ҵ��Ϣ======================================================
uint16_t iZxM2m_BuildTlvMsg_A5A4(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5a4_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xA4;
    pbuf[len++] = (SIZE_OF_ZXUP_A5A4>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5A4 & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5a4, SIZE_OF_ZXUP_A5A4); // VALUE
    len += SIZE_OF_ZXUP_A5A4;
  }

  return len;
}

//==TLV-A5A5�ϳ���������Ϣ========================================================
uint16_t iZxM2m_BuildTlvMsg_A5A5(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5a5_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xA5;
    pbuf[len++] = (SIZE_OF_ZXUP_A5A5>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5A5 & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5a5, SIZE_OF_ZXUP_A5A5); // VALUE
    len += SIZE_OF_ZXUP_A5A5;
  }

  return len;
}

//==TLV-A5A6�ֱ���Ϣ==============================================================
uint16_t iZxM2m_BuildTlvMsg_A5A6(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5a6_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xA6;
    pbuf[len++] = (SIZE_OF_ZXUP_A5A6>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5A6 & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5a6, SIZE_OF_ZXUP_A5A6); // VALUE
    len += SIZE_OF_ZXUP_A5A6;
  }

  return len;
}

//==TLV-A5A7��ʾ��1��Ϣ===========================================================
uint16_t iZxM2m_BuildTlvMsg_A5A7(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5a7_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xA7;
    pbuf[len++] = (SIZE_OF_ZXUP_A5A7>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5A7 & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5a7, SIZE_OF_ZXUP_A5A7); // VALUE
    len += SIZE_OF_ZXUP_A5A7;
  }

  return len;
}

//==TLV-A5A8��ʾ��2��Ϣ===========================================================
uint16_t iZxM2m_BuildTlvMsg_A5A8(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5a8_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xA8;
    pbuf[len++] = (SIZE_OF_ZXUP_A5A8>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5A8 & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5a8, SIZE_OF_ZXUP_A5A8); // VALUE
    len += SIZE_OF_ZXUP_A5A8;
  }

  return len;
}

//==TLV-A5A9���������Ϣ==========================================================
uint16_t iZxM2m_BuildTlvMsg_A5A9(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5a9_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xA9;
    pbuf[len++] = (SIZE_OF_ZXUP_A5A9>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5A9 & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5a9, SIZE_OF_ZXUP_A5A9); // VALUE
    len += SIZE_OF_ZXUP_A5A9;
  }

  return len;
}

//==TLV-A5AA���߲ٿ���Ϣ==========================================================
uint16_t iZxM2m_BuildTlvMsg_A5AA(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5aa_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xAA;
    pbuf[len++] = (SIZE_OF_ZXUP_A5AA>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5AA & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5aa, SIZE_OF_ZXUP_A5AA); // VALUE
    len += SIZE_OF_ZXUP_A5AA;
  }

  return len;
}

//==TLV-A5AB����������Ϣ==========================================================
uint16_t iZxM2m_BuildTlvMsg_A5AB(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5ab_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xAB;
    pbuf[len++] = (SIZE_OF_ZXUP_A5AB>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5AB & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5ab, SIZE_OF_ZXUP_A5AB); // VALUE
    len += SIZE_OF_ZXUP_A5AB;
  }

  return len;
}

//==TLV-A5AC�����߼���Ϣ==========================================================
uint16_t iZxM2m_BuildTlvMsg_A5AC(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5ac_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xAC;
    pbuf[len++] = (SIZE_OF_ZXUP_A5AC>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5AC & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5ac, SIZE_OF_ZXUP_A5AC); // VALUE
    len += SIZE_OF_ZXUP_A5AC;
  }

  return len;
}

//==TLV-A5AD����߼���Ϣ==========================================================
uint16_t iZxM2m_BuildTlvMsg_A5AD(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5ad_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xAD;
    pbuf[len++] = (SIZE_OF_ZXUP_A5AD>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5AD & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5ad, SIZE_OF_ZXUP_A5AD); // VALUE
    len += SIZE_OF_ZXUP_A5AD;
  }

  return len;
}

//==TLV-A5AE��ת�߼���Ϣ==========================================================
uint16_t iZxM2m_BuildTlvMsg_A5AE(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5ae_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xAE;
    pbuf[len++] = (SIZE_OF_ZXUP_A5AE>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5AE & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5ae, SIZE_OF_ZXUP_A5AE); // VALUE
    len += SIZE_OF_ZXUP_A5AE;
  }

  return len;
}

//==TLV-A5AF�������߼���Ϣ========================================================
uint16_t iZxM2m_BuildTlvMsg_A5AF(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5af_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xAF;
    pbuf[len++] = (SIZE_OF_ZXUP_A5AF>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5AF & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5af, SIZE_OF_ZXUP_A5AF); // VALUE
    len += SIZE_OF_ZXUP_A5AF;
  }

  return len;
}

//==TLV-A5B0�������߼���Ϣ========================================================
uint16_t iZxM2m_BuildTlvMsg_A5B0(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5b0_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xB0;
    pbuf[len++] = (SIZE_OF_ZXUP_A5B0>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5B0 & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5b0, SIZE_OF_ZXUP_A5B0); // VALUE
    len += SIZE_OF_ZXUP_A5B0;
  }

  return len;
}

//==TLV-A5B1�����߼���Ϣ==========================================================
uint16_t iZxM2m_BuildTlvMsg_A5B1(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5b1_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xB1;
    pbuf[len++] = (SIZE_OF_ZXUP_A5B1>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5B1 & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5b1, SIZE_OF_ZXUP_A5B1); // VALUE
    len += SIZE_OF_ZXUP_A5B1;
  }

  return len;
}

//==TLV-A5B2�����߼���Ϣ==========================================================
uint16_t iZxM2m_BuildTlvMsg_A5B2(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5b2_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xB2;
    pbuf[len++] = (SIZE_OF_ZXUP_A5B2>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5B2 & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5b2, SIZE_OF_ZXUP_A5B2); // VALUE
    len += SIZE_OF_ZXUP_A5B2;
  }

  return len;
}

//==TLV-A5B3Һѹ���¶�============================================================
uint16_t iZxM2m_BuildTlvMsg_A5B3(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5b3_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xB3;
    pbuf[len++] = (SIZE_OF_ZXUP_A5B3>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5B3 & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5b3, SIZE_OF_ZXUP_A5B3); // VALUE
    len += SIZE_OF_ZXUP_A5B3;
  }

  return len;
}

//==TLV-A5B4������Ϣ==============================================================
uint16_t iZxM2m_BuildTlvMsg_A5B4(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5b4_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xB4;
    pbuf[len++] = (SIZE_OF_ZXUP_A5B4>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5B4 & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5b4, SIZE_OF_ZXUP_A5B4); // VALUE
    len += SIZE_OF_ZXUP_A5B4;
  }

  return len;
}

//==TLV-A5B5����1 XHVME4400P1=====================================================
uint16_t iZxM2m_BuildTlvMsg_A5B5(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5b5_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xB5;
    pbuf[len++] = (SIZE_OF_ZXUP_A5B5>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5B5 & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5b5, SIZE_OF_ZXUP_A5B5); // VALUE
    len += SIZE_OF_ZXUP_A5B5;
  }

  return len;
}

//==TLV-A5B6����3 ���λ==========================================================
uint16_t iZxM2m_BuildTlvMsg_A5B6(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5b6_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xB6;
    pbuf[len++] = (SIZE_OF_ZXUP_A5B6>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5B6 & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5b6, SIZE_OF_ZXUP_A5B6); // VALUE
    len += SIZE_OF_ZXUP_A5B6;
  }

  return len;
}

//==TLV-A5B7������Ϣ==============================================================
uint16_t iZxM2m_BuildTlvMsg_A5B7(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5b7_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xB7;
    pbuf[len++] = (SIZE_OF_ZXUP_A5B7>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5B7 & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5b7, SIZE_OF_ZXUP_A5B7); // VALUE
    len += SIZE_OF_ZXUP_A5B7;
  }

  return len;
}

//==TLV-A5B8�����߼���Ϣ==========================================================
uint16_t iZxM2m_BuildTlvMsg_A5B8(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5b8_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xB8;
    pbuf[len++] = (SIZE_OF_ZXUP_A5B8>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5B8 & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5b8, SIZE_OF_ZXUP_A5B8); // VALUE
    len += SIZE_OF_ZXUP_A5B8;
  }

  return len;
}

//==TLV-A5B9�ױ������Ʒ�==========================================================
uint16_t iZxM2m_BuildTlvMsg_A5B9(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5b9_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xB9;
    pbuf[len++] = (SIZE_OF_ZXUP_A5B9>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5B9 & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5b9, SIZE_OF_ZXUP_A5B9); // VALUE
    len += SIZE_OF_ZXUP_A5B9;
  }

  return len;
}

//==TLV-A5BA���ƽ�ⷧ============================================================
uint16_t iZxM2m_BuildTlvMsg_A5BA(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5ba_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xBA;
    pbuf[len++] = (SIZE_OF_ZXUP_A5BA>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5BA & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5ba, SIZE_OF_ZXUP_A5BA); // VALUE
    len += SIZE_OF_ZXUP_A5BA;
  }

  return len;
}

//==TLV-A5BB�����================================================================
uint16_t iZxM2m_BuildTlvMsg_A5BB(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5bb_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xBB;
    pbuf[len++] = (SIZE_OF_ZXUP_A5BB>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5BB & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5bb, SIZE_OF_ZXUP_A5BB); // VALUE
    len += SIZE_OF_ZXUP_A5BB;
  }

  return len;
}

//==TLV-A5BC�������==============================================================
uint16_t iZxM2m_BuildTlvMsg_A5BC(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5bc_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xBC;
    pbuf[len++] = (SIZE_OF_ZXUP_A5BC>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5BC & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5bc, SIZE_OF_ZXUP_A5BC); // VALUE
    len += SIZE_OF_ZXUP_A5BC;
  }

  return len;
}

//==TLV-A5BD�����================================================================
uint16_t iZxM2m_BuildTlvMsg_A5BD(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5bd_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xBD;
    pbuf[len++] = (SIZE_OF_ZXUP_A5BD>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5BD & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5bd, SIZE_OF_ZXUP_A5BD); // VALUE
    len += SIZE_OF_ZXUP_A5BD;
  }

  return len;
}

//==TLV-A5BE�������==============================================================
uint16_t iZxM2m_BuildTlvMsg_A5BE(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5be_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xBE;
    pbuf[len++] = (SIZE_OF_ZXUP_A5BE>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5BE & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5be, SIZE_OF_ZXUP_A5BE); // VALUE
    len += SIZE_OF_ZXUP_A5BE;
  }

  return len;
}

//==TLV-A5BF��ת��================================================================
uint16_t iZxM2m_BuildTlvMsg_A5BF(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5bf_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xBF;
    pbuf[len++] = (SIZE_OF_ZXUP_A5BF>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5BF & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5bf, SIZE_OF_ZXUP_A5BF); // VALUE
    len += SIZE_OF_ZXUP_A5BF;
  }

  return len;
}

//==TLV-A5C0��ת�ƶ���============================================================
uint16_t iZxM2m_BuildTlvMsg_A5C0(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5c0_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xC0;
    pbuf[len++] = (SIZE_OF_ZXUP_A5C0>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5C0 & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5c0, SIZE_OF_ZXUP_A5C0); // VALUE
    len += SIZE_OF_ZXUP_A5C0;
  }

  return len;
}

//==TLV-A5C1������1===============================================================
uint16_t iZxM2m_BuildTlvMsg_A5C1(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5c1_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xC1;
    pbuf[len++] = (SIZE_OF_ZXUP_A5C1>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5C1 & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5c1, SIZE_OF_ZXUP_A5C1); // VALUE
    len += SIZE_OF_ZXUP_A5C1;
  }

  return len;
}

//==TLV-A5C2������2(�����λ)=====================================================
uint16_t iZxM2m_BuildTlvMsg_A5C2(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5c2_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xC2;
    pbuf[len++] = (SIZE_OF_ZXUP_A5C2>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5C2 & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5c2, SIZE_OF_ZXUP_A5C2); // VALUE
    len += SIZE_OF_ZXUP_A5C2;
  }

  return len;
}

//==TLV-A5C3������============================================================
uint16_t iZxM2m_BuildTlvMsg_A5C3(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5c3_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xC3;
    pbuf[len++] = (SIZE_OF_ZXUP_A5C3>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5C3 & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5c3, SIZE_OF_ZXUP_A5C3); // VALUE
    len += SIZE_OF_ZXUP_A5C3;
  }

  return len;
}

//==TLV-A5C4�ҳ�����============================================================
uint16_t iZxM2m_BuildTlvMsg_A5C4(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5c4_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xC4;
    pbuf[len++] = (SIZE_OF_ZXUP_A5C4>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5C4 & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5c4, SIZE_OF_ZXUP_A5C4); // VALUE
    len += SIZE_OF_ZXUP_A5C4;
  }

  return len;
}

//==TLV-A5C8��ҵ�ͺ���Ϣ==========================================================
uint16_t iZxM2m_BuildTlvMsg_A5C8(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5c8_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xC8;
    pbuf[len++] = (SIZE_OF_ZXUP_A5C8>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5C8 & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5c8, SIZE_OF_ZXUP_A5C8); // VALUE
    len += SIZE_OF_ZXUP_A5C8;
  }

  return len;
}

//==TLV-A5C9 ECU��Ӧ����CAN֡=====================================================
uint16_t iZxM2m_BuildTlvMsg_A5C9(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5c9_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xC9;
    pbuf[len++] = (SIZE_OF_ZXUP_A5C9>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5C9 & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5c9, SIZE_OF_ZXUP_A5C9); // VALUE
    len += SIZE_OF_ZXUP_A5C9;
  }

  return len;
}

/******************************************************************************
* �����³�ͨ��TLV
******************************************************************************/
//==TLV-A5E0�ڵ�״̬==============================================================
uint16_t iZxM2m_BuildTlvMsg_A5E0(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5e0_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xE0;
    pbuf[len++] = (SIZE_OF_ZXUP_A5E0>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5E0 & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5e0, SIZE_OF_ZXUP_A5E0); // VALUE
    len += SIZE_OF_ZXUP_A5E0;
  }

  return len;
}

//==TLV-A5E1����ϵͳ==============================================================
uint16_t iZxM2m_BuildTlvMsg_A5E1(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5e1_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xE1;
    pbuf[len++] = (SIZE_OF_ZXUP_A5E1>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5E1 & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5e1, SIZE_OF_ZXUP_A5E1); // VALUE
    len += SIZE_OF_ZXUP_A5E1;
  }

  return len;
}

//==TLV-A5E2֧�������Ϣ==========================================================
uint16_t iZxM2m_BuildTlvMsg_A5E2(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5e2_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xE2;
    pbuf[len++] = (SIZE_OF_ZXUP_A5E2>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5E2 & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5e2, SIZE_OF_ZXUP_A5E2); // VALUE
    len += SIZE_OF_ZXUP_A5E2;
  }

  return len;
}

//==TLV-A5E3����ϵͳ==============================================================
uint16_t iZxM2m_BuildTlvMsg_A5E3(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5e3_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xE3;
    pbuf[len++] = (SIZE_OF_ZXUP_A5E3>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5E3 & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5e3, SIZE_OF_ZXUP_A5E3); // VALUE
    len += SIZE_OF_ZXUP_A5E3;
  }

  return len;
}

//==TLV-A5E4ת��ϵͳ��ȫ���棩====================================================
uint16_t iZxM2m_BuildTlvMsg_A5E4(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5e4_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xE4;
    pbuf[len++] = (SIZE_OF_ZXUP_A5E4>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5E4 & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5e4, SIZE_OF_ZXUP_A5E4); // VALUE
    len += SIZE_OF_ZXUP_A5E4;
  }

  return len;
}

//==TLV-A5E5ת��ϵͳ��������======================================================
uint16_t iZxM2m_BuildTlvMsg_A5E5(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5e5_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xE5;
    pbuf[len++] = (SIZE_OF_ZXUP_A5E5>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5E5 & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5e5, SIZE_OF_ZXUP_A5E5); // VALUE
    len += SIZE_OF_ZXUP_A5E5;
  }

  return len;
}

//==TLV-A5E6�ƶ�ϵͳ==============================================================
uint16_t iZxM2m_BuildTlvMsg_A5E6(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5e6_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xE6;
    pbuf[len++] = (SIZE_OF_ZXUP_A5E6>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5E6 & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5e6, SIZE_OF_ZXUP_A5E6); // VALUE
    len += SIZE_OF_ZXUP_A5E6;
  }

  return len;
}

//==TLV-A5E7����ϵͳ==============================================================
uint16_t iZxM2m_BuildTlvMsg_A5E7(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5e7_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xE7;
    pbuf[len++] = (SIZE_OF_ZXUP_A5E7>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5E7 & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5e7, SIZE_OF_ZXUP_A5E7); // VALUE
    len += SIZE_OF_ZXUP_A5E7;
  }

  return len;
}

//==TLV-A5E8����ȡ��ϵͳ==========================================================
uint16_t iZxM2m_BuildTlvMsg_A5E8(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5e8_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xE8;
    pbuf[len++] = (SIZE_OF_ZXUP_A5E8>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5E8 & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5e8, SIZE_OF_ZXUP_A5E8); // VALUE
    len += SIZE_OF_ZXUP_A5E8;
  }

  return len;
}

//==TLV-A5E9Һѹϵͳ==============================================================
uint16_t iZxM2m_BuildTlvMsg_A5E9(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5e9_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xE9;
    pbuf[len++] = (SIZE_OF_ZXUP_A5E9>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5E9 & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5e9, SIZE_OF_ZXUP_A5E9); // VALUE
    len += SIZE_OF_ZXUP_A5E9;
  }

  return len;
}

//==TLV-A5EA˫��������ϵͳ========================================================
uint16_t iZxM2m_BuildTlvMsg_A5EA(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5ea_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xEA;
    pbuf[len++] = (SIZE_OF_ZXUP_A5EA>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5EA & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5ea, SIZE_OF_ZXUP_A5EA); // VALUE
    len += SIZE_OF_ZXUP_A5EA;
  }

  return len;
}

//==TLV-A5EB��̥̥ѹ==============================================================
uint16_t iZxM2m_BuildTlvMsg_A5EB(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5eb_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xEB;
    pbuf[len++] = (SIZE_OF_ZXUP_A5EB>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5EB & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5eb, SIZE_OF_ZXUP_A5EB); // VALUE
    len += SIZE_OF_ZXUP_A5EB;
  }

  return len;
}

//==TLV-A5EC��֧�����============================================================
uint16_t iZxM2m_BuildTlvMsg_A5EC(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5ec_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xEC;
    pbuf[len++] = (SIZE_OF_ZXUP_A5EC>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5EC & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5ec, SIZE_OF_ZXUP_A5EC); // VALUE
    len += SIZE_OF_ZXUP_A5EC;
  }

  return len;
}

//==TLV-A5ED��֧�����============================================================
uint16_t iZxM2m_BuildTlvMsg_A5ED(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5ed_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xED;
    pbuf[len++] = (SIZE_OF_ZXUP_A5ED>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5ED & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5ed, SIZE_OF_ZXUP_A5ED); // VALUE
    len += SIZE_OF_ZXUP_A5ED;
  }

  return len;
}

//==TLV-A5EE�п�̨������Ϣ========================================================
uint16_t iZxM2m_BuildTlvMsg_A5EE(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5ee_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xEE;
    pbuf[len++] = (SIZE_OF_ZXUP_A5EE>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5EE & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5ee, SIZE_OF_ZXUP_A5EE); // VALUE
    len += SIZE_OF_ZXUP_A5EE;
  }

  return len;
}

/******************************************************************************
* �������̷�����TLV
******************************************************************************/
//==TLV-A5EF���������в���(���ġ����塢����)======================================
uint16_t iZxM2m_BuildTlvMsg_A5EF(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5ef_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xEF;
    pbuf[len++] = (SIZE_OF_ZXUP_A5EF>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5EF & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5ef, SIZE_OF_ZXUP_A5EF); // VALUE
    len += SIZE_OF_ZXUP_A5EF;
  }

  return len;
}

//==TLV-A5F0��ʻ�ͺ�==============================================================
uint16_t iZxM2m_BuildTlvMsg_A5F0(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5f0_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xF0;
    pbuf[len++] = (SIZE_OF_ZXUP_A5F0>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5F0 & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5f0, SIZE_OF_ZXUP_A5F0); // VALUE
    len += SIZE_OF_ZXUP_A5F0;
  }

  return len;
}

//==TLV-A5F1 SCR���������壩======================================================
uint16_t iZxM2m_BuildTlvMsg_A5F1(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5f1_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xF1;
    pbuf[len++] = (SIZE_OF_ZXUP_A5F1>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5F1 & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5f1, SIZE_OF_ZXUP_A5F1); // VALUE
    len += SIZE_OF_ZXUP_A5F1;
  }

  return len;
}

//==TLV-A5F2 DPF����(������=======================================================
uint16_t iZxM2m_BuildTlvMsg_A5F2(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5f2_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xF2;
    pbuf[len++] = (SIZE_OF_ZXUP_A5F2>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5F2 & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5f2, SIZE_OF_ZXUP_A5F2); // VALUE
    len += SIZE_OF_ZXUP_A5F2;
  }

  return len;
}



