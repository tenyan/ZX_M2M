/*****************************************************************************
* @FileName: tbox_machine.h
* @Engineer: TenYan
* @Company:  徐工信息智能硬件部
* @version   V1.0
* @Date:     2020-10-16
* @brief
******************************************************************************/
#ifndef _TBOX_MACHINE_H_
#define _TBOX_MACHINE_H_

/******************************************************************************
 *   Pre-Processor Defines
 ******************************************************************************/

/******************************************************************************
 *   Macros
 ******************************************************************************/

/******************************************************************************
 *   Data Types
 ******************************************************************************/
typedef enum
{
  TBOX_FALSE = 0,
  TBOX_TRUE  = 1
} tbox_bool_t;

// states for tbox mode
typedef enum
{
  TBOX_MODE_WORKING = 0x00,
  TBOX_MODE_SLEEP,
  TBOX_MODE_POWER_OFF,
  TBOX_MODE_TOWING,
  TBOX_MODE_IAP,
}tbox_mode_t;
extern tbox_mode_t tbox_mode;

// states for iap
typedef enum
{
  TBOX_IAP_STATE_INIT = 0x00,
  TBOX_IAP_STATE_VERIFY,
  TBOX_IAP_STATE_IAP,
  TBOX_IAP_STATE_EXIT,
  TBOX_IAP_STATE_REBOOT
}tbox_iap_state_t;
extern tbox_iap_state_t tbox_iap_state;

// states for sleep
typedef enum
{
  TBOX_SLEEP_STATE_INIT = 0x00,
  TBOX_SLEEP_STATE_LIGHT,
  TBOX_SLEEP_STATE_DEEP,
  TBOX_SLEEP_STATE_EXIT
}tbox_sleep_state_t;
extern tbox_sleep_state_t tbox_sleep_state;

// states for T-BOX
typedef enum
{
  TBOX_STATE_POWERON = 0x00,
  TBOX_STATE_WORKING,
  TBOX_STATE_SLEEP,
  TBOX_STATE_IAP
} tbox_state_t;
extern tbox_state_t tbox_state;
extern uint8_t tbox_started_delay;
extern uint8_t tbox_state_transition;

//extern rtc_date_t rtc_time;
extern uint8_t net_public_data_buffer[1460];

/******************************************************************************
 *   Function prototypes
 ******************************************************************************/
void tbox_initialize(void);
void tbox_state_machine(void);

void Tbox_SetMachineState(tbox_state_t state);
void Tbox_ResetSleepTime(void);

#endif /* _TBOX_MACHINE_H_ */

