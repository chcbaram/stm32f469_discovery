
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_dma2d.h"
#include "stm32f4xx_hal_ltdc.h"
#include "hw.h"

#include <stdbool.h>

#include <cmsis_os.h>

extern DMA2D_HandleTypeDef hdma2d;
extern DSI_HandleTypeDef hdsi;

static volatile int overrunCnt;

extern struct HwJPEG_Context_s HwJPEG_Context;




void DMA2D_IRQHandler(void)
{
  HAL_DMA2D_IRQHandler(&hdma2d);
}

void DSI_IRQHandler(void)
{
  HAL_DSI_IRQHandler(&hdsi);
}

void LTDC_ER_IRQHandler(void)
{
  if (LTDC->ISR & 2)
  {
    LTDC->ICR = 2;
    overrunCnt++;
  }
}
