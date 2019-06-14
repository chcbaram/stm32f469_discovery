/*
 * bsp.c
 *
 *  Created on: 2019. 6. 14.
 *      Author: HanCheol Cho
 */




#include "bsp.h"
#include "uart.h"


extern void cubeInit(void);


void bspInit(void)
{
  cubeInit();
}


int __io_putchar(int ch)
{
  //ITM_SendChar(ch);
  uartWrite(_DEF_UART1, (uint8_t *)&ch, 1);

  return 1;
}
