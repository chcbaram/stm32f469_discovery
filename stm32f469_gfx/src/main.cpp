/*
 * main.cpp
 *
 *  Created on: 2017. 2. 13.
 *      Author: baram
 */



#include "main.h"




#define configAP_TASK_PRIORITY                 ( tskIDLE_PRIORITY + 3 )
#define configAP_TASK_STK_SIZE                 ( 4096 / 4 )


extern int touchgfxMain(void);

static void threadMain(void const *params);


int main(void)
{
  hwInit();
  apInit();


  touchgfxMain();

  osThreadDef(threadMain, threadMain, osPriorityNormal, 0, 4096 / 4);
  osThreadCreate(osThread(threadMain), NULL);

  osKernelStart();

  return 0;
}

void threadMain(void const *params)
{
  apMain();
}
