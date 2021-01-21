/**********************************************************************
* FileName:        led.h
* Engineer:        TenYan
* Date:            2020/10/10
* Dependencies:    Header (.h) files if applicable, see below
************************************************************************/
#ifndef LED_H_
#define LED_H_

#include "types.h"

/******************************************************************************
 *   Macros
 ******************************************************************************/
#define SLOW_OFF_TIME		90		// 70 * 10ms = 0.7s
#define SLOW_ON_TIME		10		// 30 * 10ms = 0.3s
#define FAST_OFF_TIME		20		// 20 * 10ms = 200ms
#define FAST_ON_TIME		5		  // 5 * 10ms = 50ms

/******************************************************************************
 *   Data Types
 ******************************************************************************/
typedef enum
{
  LED_OFF = 0,
  LED_ON  = 1,
} led_bool_t;

enum
{
  LED_POWER = 0,
  LED_CAN1,
  LED_CAN2,
  LED_GPS,
  LED_WIFI,
  LED_ETH,
  NUMBER_OF_LEDS
};  //led_control items

typedef struct
{
  uint8_t blink;
  uint8_t num_of_blinks;
  uint8_t pin_state;			//1 = on, 0 = off
  uint8_t blink_count;
  uint8_t	timer_count;
  uint8_t change_flag;
}led_object_type_t;
extern led_object_type_t led_control[NUMBER_OF_LEDS];

enum
{
  STEADY_OFF,
  SLOW_NUMBERED,				//slow = 1s on, 2s off
  FAST_NUMBERED,				//fast = 100ms on, 100ms off
  SLOW_CONTINUOUS,
  FAST_CONTINUOUS,
  STEADY_ON
}; //enum for the led blink

/******************************************************************************
 *   Function prototypes
 ******************************************************************************/
void led_state_machine(void);
void led_on_off(led_object_type_t* pThis,unsigned char led);

void turn_on_all_leds(void);
void turn_off_all_leds(void);

#endif /* LED_H_ */
