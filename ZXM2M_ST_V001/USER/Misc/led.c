/**********************************************************************
* Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
* @FileName:        led.c
* @Engineer:        TenYan
* @Company:  徐工信息智能硬件部
* @Date:            5/29/2020
* @Dependencies:    Header (.h) files if applicable, see below
************************************************************************/
#include "main.h"
#include "led.h"

led_object_type_t led_control[NUMBER_OF_LEDS];

/******************************************************************************
 * Led State Machine must be called from time based tasks/periodic basis
 * The time is based on the Slow and Fast on/off timers in led.h
 * For this Program it is every 10ms
******************************************************************************/
void led_state_machine(void)
{
  unsigned char i;
  unsigned char temp = 0;

  for (i = 0; i < NUMBER_OF_LEDS; i++)
  {
    switch (led_control[i].blink)
    {
    case SLOW_NUMBERED:
    case FAST_NUMBERED:
    case SLOW_CONTINUOUS:
    case FAST_CONTINUOUS:

      /**
       * This is an intentional fall through case for the
       * SLOW and FAST NUMBERED and the SLOW and FAST CONTINUOUS
       */
      if (led_control[i].pin_state == LED_OFF)
      {
        if (led_control[i].change_flag == TRUE)
        {
          //it was just changed to go off.
          led_control[i].change_flag = FALSE;
          /**
           * If it is a numbered blink then increment the count.
           */
          if ((led_control[i].blink == SLOW_NUMBERED) || (led_control[i].blink == FAST_NUMBERED))
          {
            led_control[i].blink_count++;
          }

          if ((led_control[i].blink == SLOW_NUMBERED) || (led_control[i].blink == SLOW_CONTINUOUS))
          {
            led_control[i].timer_count = SLOW_OFF_TIME;
          }
          else
          {
            led_control[i].timer_count = FAST_OFF_TIME;
          }

        }
        else
        {
          //its already been off...wait for off time to finish.
          if (led_control[i].timer_count)
          {
            led_control[i].timer_count--;
          }
          else  //time expired.
          {
            //the below if, will cut off immediately so we don't get a little
            //blip of light after the number of blinks are done
            if ((led_control[i].blink == SLOW_NUMBERED) || (led_control[i].blink == FAST_NUMBERED))
            {
              temp = led_control[i].blink_count;
              if (temp >= led_control[i].num_of_blinks)
              {
                //don't delay off.  turn steady off immediately
                led_control[i].blink = STEADY_OFF;
              }
              else
              {
                led_control[i].pin_state = LED_ON;
              }
            }
            else
            {
              led_control[i].pin_state = LED_ON;
            }

            //just changed, set flag
            led_control[i].change_flag = TRUE;
          }
        }
      }
      else
      {
        //led is currently on
        if (led_control[i].change_flag == TRUE)
        {
          //it was just changed to go on
          led_control[i].change_flag = FALSE;

          /**
           * If it is numbered, check to see if you have blinked the total
           * number of blinks desired.
           */
          temp = led_control[i].blink_count;
          if ((led_control[i].blink == SLOW_NUMBERED ||
               led_control[i].blink == FAST_NUMBERED) && (temp > led_control[i].num_of_blinks))
          {
            //if you use blink_count >= .num of blinks you will get a partial
            //blink on the last one.
            led_control[i].blink = STEADY_OFF;
          }
          else
          {
            if (led_control[i].blink == SLOW_NUMBERED || led_control[i].blink == SLOW_CONTINUOUS)
            {
              led_control[i].timer_count = SLOW_ON_TIME;
            }
            else
            {
              led_control[i].timer_count = FAST_ON_TIME;
            }
          }
        }
        else
        {
          //its already been on and waiting for slow on timer to run out
          if (led_control[i].timer_count != 0x00)
          {
            led_control[i].timer_count--;
          }
          else //time expired.
          {
            led_control[i].pin_state = LED_OFF;
            led_control[i].change_flag = TRUE;
          }
        }
      }
      break;

    case STEADY_ON:
      led_control[i].blink_count = 0x00;
      led_control[i].num_of_blinks = 0x00;
      led_control[i].pin_state = LED_ON;
      led_control[i].change_flag = FALSE;
      break;

    case STEADY_OFF:
    default:
      led_control[i].num_of_blinks = 0x00;
      led_control[i].blink_count = 0x00;
      led_control[i].pin_state = LED_OFF;
      led_control[i].change_flag = FALSE;
      break;
    }//end switch state

    led_on_off(&led_control[i], i);

  }//end for (going through all 8 leds)

}//end led state machine

/******************************************************************************
*
******************************************************************************/
void led_on_off(led_object_type_t* pThis, unsigned char led)
{
  switch (led)
  {
  case LED_POWER:
    if (pThis->pin_state)
    {
      POWER_LED_ON();
    }
    else
    {
      POWER_LED_OFF();
    }
    break;
    
  case LED_CAN1:
    if (pThis->pin_state)
    {
      CAN1_LED_ON();
    }
    else
    {
      CAN1_LED_OFF();
    }
    break;
    
  case LED_CAN2:
    if (pThis->pin_state)
    {
      CAN2_LED_ON();
    }
    else
    {
      CAN2_LED_OFF();
    }
    break;

  case LED_GPS:
    if (pThis->pin_state)
    {
      GPS_LED_ON();
    }
    else
    {
      GPS_LED_OFF();
    }
    break;
    
  case LED_WIFI:
    if (pThis->pin_state)
    {
      WIFI_LED_ON();
    }
    else
    {
      WIFI_LED_OFF();
    }
    break;
    
  case LED_ETH:
    if (pThis->pin_state)
    {
      ETH_LED_ON();
    }
    else
    {
      ETH_LED_OFF();
    }
    break;

  default:
    break;
  }
}

/******************************************************************************
*
******************************************************************************/
void turn_on_all_leds(void)
{
  unsigned char i;
  for (i = 0; i < NUMBER_OF_LEDS; i++)
  {
    led_control[i].blink = SLOW_NUMBERED;
    led_control[i].num_of_blinks = 0x01;
  }
}

/******************************************************************************
*
******************************************************************************/
void turn_off_all_leds(void)
{
  unsigned char i;
  for (i = 0; i < NUMBER_OF_LEDS; i++)
  {
    led_control[i].blink = STEADY_OFF;
  }
}

/*
#if 0
led_control[LED_SYSTEM].blink = SLOW_NUMBERED;
led_control[LED_SYSTEM].num_of_blinks = 0x0A;

led_control[LED_SYSTEM].blink = FAST_NUMBERED;
led_control[LED_SYSTEM].num_of_blinks = 0x0A;


led_control[LED_SYSTEM].blink = FAST_CONTINUOUS;
led_control[LED_SYSTEM].blink = SLOW_CONTINUOUS;


led_control[LED_SYSTEM].blink = STEADY_OFF;
led_control[LED_SYSTEM].blink = STEADY_ON;
#endif
*/

//-----文件led.c结束---------------------------------------------
