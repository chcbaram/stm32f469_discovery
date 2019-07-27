/*
 * bsp.h
 *
 *  Created on: 2019. 6. 14.
 *      Author: HanCheol Cho
 */

#ifndef SRC_BSP_BSP_H_
#define SRC_BSP_BSP_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "def.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"


#include "cmsis_os.h"

void bspInit(void);
void bspDeinit(void);

extern void delay(uint32_t ms);
extern uint32_t millis(void);
extern uint32_t micros(void);


#ifdef __cplusplus
 }
#endif
#endif /* SRC_BSP_BSP_H_ */
