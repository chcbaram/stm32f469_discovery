/*
 * ap.cpp
 *
 *  Created on: 2019. 6. 14.
 *      Author: HanCheol Cho
 */




#include "ap.h"


void updateLed(void);


void apInit(void)
{
  cmdifOpen(_DEF_UART1, 57600);
  uartOpen(_DEF_UART2, 115200);
}

void apMain(void)
{
  uint32_t pre_time;
  usb_hid_joy_msg_t joy_msg;


  usbHidStart();

  while(1)
  {
    cmdifMain();

    if (millis()-pre_time >= 100)
    {
      pre_time = millis();
      ledToggle(_DEF_LED1);

      usbHidJoyCurrentRead(&joy_msg);
    }

    updateLed();
  }
}

void updateLed(void)
{
  static uint8_t state = 0;
  static uint32_t pre_time;


  if (usbHidJoyIsConnected() == true)
  {
    ledOn(_DEF_LED4);
  }
  else
  {
    ledOff(_DEF_LED4);
  }

  switch(state)
  {
    case 0:
      if (usbHidJoyIsReceived() == true)
      {
        ledOn(_DEF_LED3);
        state = 1;
        pre_time = millis();
      }
      break;

    case 1:
      if (millis()-pre_time >= 50)
      {
        ledOff(_DEF_LED3);
        state = 2;
        pre_time = millis();
      }
      break;

    case 2:
      if (millis()-pre_time >= 50)
      {
        state = 0;
        pre_time = millis();
      }
      break;
  }
}
