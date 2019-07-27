/*
 * micros.c
 *
 *  Created on: 2019. 3. 7.
 *      Author: HanCheol Cho
 */




#include "micros.h"



static TIM_HandleTypeDef  TimHandle2;



bool microsInit(void)
{
  __HAL_RCC_TIM2_CLK_ENABLE();


  /* Set TIMx instance */
  TimHandle2.Instance = TIM2;

  /* Initialize TIM3 peripheral as follow:
         + Period = 10000 - 1
         + Prescaler = ((SystemCoreClock/2)/10000) - 1
         + ClockDivision = 0
         + Counter direction = Up
   */
  TimHandle2.Init.Period         = 0xFFFFFFFF;;
  TimHandle2.Init.Prescaler      = (SystemCoreClock/2/1000000)-1;
  TimHandle2.Init.ClockDivision  = 0;
  TimHandle2.Init.CounterMode    = TIM_COUNTERMODE_UP;

  HAL_TIM_Base_Init(&TimHandle2);
  HAL_TIM_Base_Start_IT(&TimHandle2);


  return true;
}

uint32_t micros(void)
{
  return TimHandle2.Instance->CNT;
}
