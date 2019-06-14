/*
 * led.c
 *
 *  Created on: 2019. 3. 7.
 *      Author: HanCheol Cho
 */



#include "led.h"


typedef struct
{
  GPIO_TypeDef *port;
  uint16_t      pin;
  GPIO_PinState on_state;
  GPIO_PinState off_state;
} led_tbl_t;


const led_tbl_t led_tbl[LED_MAX_CH] =
{
  {GPIOG, GPIO_PIN_6, GPIO_PIN_RESET, GPIO_PIN_SET},
  {GPIOD, GPIO_PIN_4, GPIO_PIN_RESET, GPIO_PIN_SET},
  {GPIOD, GPIO_PIN_5, GPIO_PIN_RESET, GPIO_PIN_SET},
  {GPIOK, GPIO_PIN_3, GPIO_PIN_RESET, GPIO_PIN_SET},
};



bool ledInit(void)
{
  uint32_t i;

  for (i=0; i<LED_MAX_CH; i++)
  {
    ledOff(i);
  }

  return true;
}

void ledOn(uint8_t ch)
{
  if (ch < LED_MAX_CH)
  {
    HAL_GPIO_WritePin(led_tbl[ch].port, led_tbl[ch].pin, led_tbl[ch].on_state);
  }
}

void ledOff(uint8_t ch)
{
  if (ch < LED_MAX_CH)
  {
    HAL_GPIO_WritePin(led_tbl[ch].port, led_tbl[ch].pin, led_tbl[ch].off_state);
  }
}

void ledToggle(uint8_t ch)
{
  if (ch < LED_MAX_CH)
  {
    HAL_GPIO_TogglePin(led_tbl[ch].port, led_tbl[ch].pin);
  }
}
