/*
 * drv_ts.h
 *
 *  Created on: Feb 13, 2018
 *      Author: opus
 */

#ifndef DRV_TS_H_
#define DRV_TS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "hw_def.h"



#define TOUCH_EVENT_NB_MAX      4
#define GEST_ID_NB_MAX          7


bool tsInit(uint16_t ts_SizeX, uint16_t ts_SizeY);
bool tsUpdateTouchData(void);
bool tsResetTouchData(void);

uint8_t tsIsDetected(void);
uint16_t tsGetXAxis(uint8_t detect_num);
uint16_t tsGetYAxis(uint8_t detect_num);
uint8_t tsGetWeight(uint8_t detect_num);
uint8_t tsGetArea(uint8_t detect_num);
touch_gesture_t tsGetGestureId(void);
err_code_t tsExtiConfig(void);

#ifdef __cplusplus
}
#endif
#endif /* DRV_TS_H_ */
