/*
 * ap.cpp
 *
 *  Created on: 2019. 6. 14.
 *      Author: HanCheol Cho
 */




#include "ap.h"

extern "C"
{
#include "GUI_App.h"
#include "GUI.h"
#include "DIALOG.h"

extern  WM_HWIN CreateWindow(void);
extern  WM_HWIN CreateFramewin(void);

}




void apInit(void)
{
  uartOpen(_DEF_UART1, 57600);
  cmdifOpen(_DEF_UART1, 57600);

  GRAPHICS_HW_Init();
  GRAPHICS_Init();


  CreateWindow();

  if (tsInit(800, 480) == true)
  {
    printf("Touch \t: OK\r\n");
  }
  else
  {
    printf("Touch \t: Fail\r\n");
  }
}

void apMain(void)
{
  uint32_t pre_time;
  GUI_PID_STATE TS_State;

  while(1)
  {
    GUI_Exec();
    //cmdifMain();
    tsUpdateTouchData();

    if(tsIsDetected() > 0)
    {
      TS_State.x = tsGetXAxis(0);
      TS_State.y = tsGetYAxis(0);
      TS_State.Pressed = 1 ;
    }
    else
    {
      TS_State.Pressed = 0 ;
    }
    GUI_Exec();
    GUI_Exec1();
    GUI_PID_StoreState(&TS_State);


    if (millis()-pre_time >= 500)
    {
      pre_time = millis();
      ledToggle(_DEF_LED1);

      GUI_Clear();
      GUI_SetColor(GUI_WHITE);
      GUI_SetFont(&GUI_Font32_1);
      GUI_DispStringAt("Hello world!", (LCD_GetXSize()-150)/2, (LCD_GetYSize()-20)/2);
      GUI_Delay(500);
    }
  }
}
