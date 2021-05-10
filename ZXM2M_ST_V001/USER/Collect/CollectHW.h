/*****************************************************************************
* Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
* @FileName: CollectHW.h
* @Engineer: TenYan
* @Company:  徐工信息智能硬件部
* @version:  V1.0
* @Date:     2020-10-10
* @brief:
******************************************************************************/
#ifndef _COLLECT_HW_H_
#define _COLLECT_HW_H_

#include "types.h"

/******************************************************************************
 * Pre-Processor Defines
 ******************************************************************************/
#define ADC1_DR_ADDRESS    ((uint32_t)&ADC1->DR)
#define ADC2_DR_ADDRESS    ((uint32_t)&ADC2->DR)
#define ADC3_DR_ADDRESS    ((uint32_t)&ADC3->DR)

/******************************************************************************
 * Macros
 ******************************************************************************/
#define MAX_AV_NB_SAMPLE         0x20
#define DEFAULT_AV_NB_SAMPLE     0x20

#define ADC_CHANNEL_VBAT_NB_SAMPLES      DEFAULT_AV_NB_SAMPLE
#define ADC_CHANNEL_VRAW_NB_SAMPLES      DEFAULT_AV_NB_SAMPLE
#define ADC_CHANNEL_INTTEMP_NB_SAMPLES   DEFAULT_AV_NB_SAMPLE
#define ADC_CHANNEL_VREF_NB_SAMPLES      DEFAULT_AV_NB_SAMPLE

/******************************************************************************
 *   Data Types
 ******************************************************************************/
__packed typedef struct
{
  uint16_t vbat;       // UNIT = 10mV
  uint8_t battery_cutoff_debounce_counter;
  uint8_t BATTERY_CUTOFF_VOLTAGE_FLAG;
  uint8_t battery_low_debounce_counter;
  uint8_t BATTERY_LOW_VOLTAGE_FLAG;
  uint8_t battery_recovery_debounce_counter;
  uint8_t BATTERY_RECOVERY_VOLTAGE_FLAG;

  uint16_t vraw;      // UNIT = 10mV
  uint16_t inttemp;
  uint16_t vref;
}adc_result_type_t;
extern adc_result_type_t adc_result;

/******************************************************************************
 *   Application Specific Globals
 ******************************************************************************/
enum adc_channel_list
{
  ADC_CHANNEL_VBAT      = 0,
  ADC_CHANNEL_VRAW      = 1,
  ADC_CHANNEL_INT_TEMP  = 2,
  ADC_CHANNEL_VREF      = 3,
  NUMBER_OF_ADC_CHANNEL
};

/**
 Struct to collect samples from ADC and building an average
 over MAX_AV_NB_SAMPLE values.
**/
__packed typedef struct
{
  uint32_t sum;                      /// Sum of samples in buffer (avg = sum / av_nb_sample)
  uint16_t values[MAX_AV_NB_SAMPLE]; /// Buffer for sample values from ADC
  uint8_t  head;                     /// Position index of write head in buffer
  uint8_t  av_nb_sample;             /// Number of samples to use in buffer (used for avg)
}adc_buf_type_t;

extern adc_buf_type_t adc_buffers[NUMBER_OF_ADC_CHANNEL];
#define buf_vbat      adc_buffers[ADC_CHANNEL_VBAT]
#define buf_vraw      adc_buffers[ADC_CHANNEL_VRAW]
#define buf_inttemp   adc_buffers[ADC_CHANNEL_INT_TEMP]
#define buf_vref      adc_buffers[ADC_CHANNEL_VREF]

/******************************************************************************
 *   Function prototypes
 ******************************************************************************/
void adc_push_dma_sample(void);
void get_battery_voltage(void);
void get_main_power_voltage(void);
void CollectHw_Init(void);

#endif

