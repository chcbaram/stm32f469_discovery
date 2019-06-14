/**
  ******************************************************************************
  * @file    stm32469i_discovery_ts.h
  * @author  MCD Application Team
  * @brief   This file contains the common defines and functions prototypes for
  *          the stm32469i_discovery_ts.c driver.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/*
 * ts.c
 *
 *  Created on: Feb 13, 2018
 *      Author: opus
 */


#include "ts.h"
#include "ft6x06.h"
#include "i2c.h"

#define TS_I2C_ADDRESS                   ((uint16_t)(0x54>>1))

/** @brief With FT6206 : maximum 2 touches detected simultaneously
  */
#define TS_MAX_NB_TOUCH                 ((uint32_t) FT6206_MAX_DETECTABLE_TOUCH)
#define TS_NO_IRQ_PENDING               ((uint8_t) 0)
#define TS_IRQ_PENDING                  ((uint8_t) 1)

#define TS_SWAP_NONE                    ((uint8_t) 0x01)
#define TS_SWAP_X                       ((uint8_t) 0x02)
#define TS_SWAP_Y                       ((uint8_t) 0x04)
#define TS_SWAP_XY                      ((uint8_t) 0x08)

/**
*  @brief ts_data_t
*  Define TS State structure
*/
typedef struct
{
  uint8_t  detect_cnt;                /*!< Total number of active touches detected at last scan */
  uint16_t x_axis[TS_MAX_NB_TOUCH];      /*!< Touch X[0], X[1] coordinates on 12 bits */
  uint16_t y_axis[TS_MAX_NB_TOUCH];      /*!< Touch Y[0], Y[1] coordinates on 12 bits */

#if (TS_MULTI_TOUCH_SUPPORTED == 1)
  uint8_t  weight[TS_MAX_NB_TOUCH]; /*!< Touch_Weight[0], Touch_Weight[1] : weight property of touches */
  uint8_t  event_id[TS_MAX_NB_TOUCH];     /*!< Touch_EventId[0], Touch_EventId[1] : take value of type @ref TS_TouchEventTypeDef */
  uint8_t  area[TS_MAX_NB_TOUCH];   /*!< Touch_Area[0], Touch_Area[1] : touch area of each touch */
  uint32_t gesture_id; /*!< type of gesture detected : take value of type @ref TS_GestureIdTypeDef */
#endif  /* TS_MULTI_TOUCH_SUPPORTED == 1 */
} ts_data_t;

static ts_data_t ts_data;
static TS_DrvTypeDef *ts_driver;
static err_code_t  ts_orientation;
static err_code_t  i2c_addr = 0;

#if (TS_MULTI_TOUCH_SUPPORTED == 1)
/* Table for touchscreen event information display on LCD : table indexed on enum @ref TS_TouchEventTypeDef information */
char * ts_event_string_tab[TOUCH_EVENT_NB_MAX] = { "None",
                                                   "Press down",
                                                   "Lift up",
                                                   "Contact"
                                                  };

/* Table for touchscreen gesture Id information display on LCD : table indexed on enum @ref TS_GestureIdTypeDef information */
char * ts_gesture_id_string_tab[GEST_ID_NB_MAX] = { "None",
                                                    "Move Up",
                                                    "Move Right",
                                                    "Move Down",
                                                    "Move Left",
                                                    "Zoom In",
                                                    "Zoom Out"
                                                  };

static err_code_t drvTsUpdateGestureId(void);
#endif /* (TS_MULTI_TOUCH_SUPPORTED == 1) */

static void drvTsMspInit(void);

/**
  * @brief  Initializes and configures the touch screen functionalities and
  *         configures all necessary hardware resources (GPIOs, I2C, clocks..).
  * @param  x_size : Maximum X size of the TS area on LCD
  * @param  y_size : Maximum Y size of the TS area on LCD
  * @retval OK if all initializations are OK. Other value if error.
  */
bool tsInit(uint16_t x_size, uint16_t y_size)
{
  bool ret = false;

  err_code_t ts_status = OK;
  uint8_t ts_id1, ts_id2 = 0;
  /* Note : i2c_addr is un-initialized here, but is not used at all in init function */
  /* but the prototype of Init() is like that in template and should be respected       */

  /* Initialize the communication channel to sensor (I2C) if necessary */
  /* that is initialization is done only once after a power up         */
  ft6x06_ts_drv.Init(i2c_addr);

  ts_id1 = ft6x06_ts_drv.ReadID(TS_I2C_ADDRESS);

  i2c_addr    = TS_I2C_ADDRESS;

  /* Scan FT6xx6 TouchScreen IC controller ID register by I2C Read       */
  /* Verify this is a FT6206 or FT6336G, otherwise this is an error case */
  if((ts_id1 == FT6206_ID_VALUE) || (ts_id2 == FT6206_ID_VALUE))
  {
    /* Found FT6206 : Initialize the TS driver structure */
    ts_driver = &ft6x06_ts_drv;

    /* Get LCD chosen orientation */
    if(x_size < y_size)
    {
      ts_orientation = TS_SWAP_NONE;
    }
    else
    {
      ts_orientation = TS_SWAP_XY | TS_SWAP_Y;
      //ts_orientation = TS_SWAP_XY | TS_SWAP_X;;
    }

    if(ts_status == OK)
    {
      /* Software reset the TouchScreen */
      ts_driver->Reset(i2c_addr);

      /* Calibrate, Configure and Start the TouchScreen driver */
      ts_driver->Start(i2c_addr);

    } /* of if(ts_status == OK) */
  }
  else
  {
    ts_status = ERR_TS_DEV_NOT_FOUND;
  }

  if (ts_status == OK)
  {
    ret = true;
  }
  return ret;
}

/**
  * @brief  Returns status and positions of the touch screen.
  * @param  ts_data: Pointer to touch screen current state structure
  * @retval OK if all initializations are OK. Other value if error.
  */
bool tsUpdateTouchData(void)
{
  static uint32_t _x[TS_MAX_NB_TOUCH] = {0, 0};
  static uint32_t _y[TS_MAX_NB_TOUCH] = {0, 0};
  err_code_t ts_status = OK;
  bool ret = false;
  uint16_t tmp;
  uint16_t Raw_x[TS_MAX_NB_TOUCH];
  uint16_t Raw_y[TS_MAX_NB_TOUCH];
  uint16_t xDiff;
  uint16_t yDiff;
  uint32_t index;
#if (TS_MULTI_TOUCH_SUPPORTED == 1)
  uint32_t weight = 0;
  uint32_t area = 0;
  uint32_t event = 0;
#endif /* TS_MULTI_TOUCH_SUPPORTED == 1 */

  /* Check and update the number of touches active detected */
  ts_data.detect_cnt = ts_driver->DetectTouch(i2c_addr);
  if(ts_data.detect_cnt)
  {
    for(index=0; index < ts_data.detect_cnt; index++)
    {
      /* Get each touch coordinates */
      ts_driver->GetXY(i2c_addr, &(Raw_x[index]), &(Raw_y[index]));

      if(ts_orientation & TS_SWAP_XY)
      {
        tmp = Raw_x[index];
        Raw_x[index] = Raw_y[index];
        Raw_y[index] = tmp;
      }

      if(ts_orientation & TS_SWAP_X)
      {
        Raw_x[index] = FT_6206_MAX_WIDTH - 1 - Raw_x[index];
      }

      if(ts_orientation & TS_SWAP_Y)
      {
        Raw_y[index] = FT_6206_MAX_HEIGHT - 1 - Raw_y[index];
      }

      xDiff = Raw_x[index] > _x[index]? (Raw_x[index] - _x[index]): (_x[index] - Raw_x[index]);
      yDiff = Raw_y[index] > _y[index]? (Raw_y[index] - _y[index]): (_y[index] - Raw_y[index]);

      if ((xDiff + yDiff) > 5)
      {
        _x[index] = Raw_x[index];
        _y[index] = Raw_y[index];
      }

      ts_data.x_axis[index] = _x[index];
      ts_data.y_axis[index] = _y[index];

#if (TS_MULTI_TOUCH_SUPPORTED == 1)
      /* Get touch info related to the current touch */
      ft6x06_TS_GetTouchInfo(i2c_addr, index, &weight, &area, &event);

      /* Update ts_data structure */
      ts_data.weight[index] = weight;
      ts_data.area[index]   = area;

      /* Remap touch event */
      switch(event)
      {
        case FT6206_TOUCH_EVT_FLAG_PRESS_DOWN  :
          ts_data.event_id[index] = TOUCH_EVENT_PRESS_DOWN;
          break;
        case FT6206_TOUCH_EVT_FLAG_LIFT_UP :
          ts_data.event_id[index] = TOUCH_EVENT_LIFT_UP;
          break;
        case FT6206_TOUCH_EVT_FLAG_CONTACT :
          ts_data.event_id[index] = TOUCH_EVENT_CONTACT;
          break;
        case FT6206_TOUCH_EVT_FLAG_NO_EVENT :
          ts_data.event_id[index] = TOUCH_EVENT_NO_EVT;
          break;
        default :
          ts_status = ERR_TS;
          break;
      } /* of switch(event) */

#endif /* TS_MULTI_TOUCH_SUPPORTED == 1 */
    } /* of for(index=0; index < ts_data->detect_cnt; index++) */

#if (TS_MULTI_TOUCH_SUPPORTED == 1)
    /* Get gesture Id */
    ts_status = drvTsUpdateGestureId();
#endif /* TS_MULTI_TOUCH_SUPPORTED == 1 */

  } /* end of if(ts_data->detect_cnt != 0) */

  if (ts_status == OK)
  {
    ret = true;
  }
  return ret;
}

uint8_t tsIsDetected(void)
{
  return ts_data.detect_cnt;
}

uint16_t tsGetXAxis(uint8_t detect_num)
{
  return ts_data.x_axis[detect_num];
}

uint16_t tsGetYAxis(uint8_t detect_num)
{
  return ts_data.y_axis[detect_num];
}

#if (TS_MULTI_TOUCH_SUPPORTED == 1)
uint8_t tsGetWeight(uint8_t detect_num)
{
  return ts_data.weight[detect_num];
}

touch_event_t tsGetEventId(uint8_t detect_num)
{
  return (touch_event_t) ts_data.event_id[detect_num];
}

uint8_t tsGetArea(uint8_t detect_num)
{
  return ts_data.area[detect_num];
}

touch_gesture_t tsGetGestureId(void)
{
  return (touch_gesture_t) ts_data.gesture_id;
}
#endif /* (TS_MULTI_TOUCH_SUPPORTED == 1) */

/**
  * @brief  Function used to reset all touch data before a new acquisition
  *         of touch information.
  * @param  ts_data: Pointer to touch screen current state structure
  * @retval OK if OK, TE_ERROR if problem found.
  */
bool tsResetTouchData(void)
{
  //err_code_t ts_status = ERR_TS;
  uint32_t index;

#if (TS_MULTI_TOUCH_SUPPORTED == 1)
  ts_data.gesture_id = GEST_ID_NO_GESTURE;
#endif
  ts_data.detect_cnt = 0;

  for(index = 0; index < TS_MAX_NB_TOUCH; index++)
  {
    ts_data.x_axis[index]   = 0;
    ts_data.y_axis[index]   = 0;
#if (TS_MULTI_TOUCH_SUPPORTED == 1)
    ts_data.area[index]     = 0;
    ts_data.event_id[index] = TOUCH_EVENT_NO_EVT;
    ts_data.weight[index]   = 0;
#endif
  }

  //ts_status = OK;

  return true;
}

/**
  * @brief  Configures and enables the touch screen interrupts both at GPIO level and
  * in TouchScreen IC driver configuration.
  * @retval OK if all initializations are OK.
  */
err_code_t drvTsExtiConfig(void)
{
  err_code_t ts_status = OK;
  GPIO_InitTypeDef GPIO_InitStruct;

  /* Msp Init of GPIO used for TS_INT pin coming from TouchScreen driver IC FT6x06 */
  /* When touchscreen is operated in interrupt mode */
  drvTsMspInit();

  /* Configure Interrupt mode for TS_INT pin falling edge : when a new touch is available */
  /* TS_INT pin is active on low level on new touch available */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  HAL_GPIO_Init(GPIOJ, &GPIO_InitStruct);

  /* Enable and set the TS_INT EXTI Interrupt to an intermediate priority */
  HAL_NVIC_SetPriority((IRQn_Type)(EXTI9_5_IRQn), 0x05, 0x00);
  HAL_NVIC_EnableIRQ((IRQn_Type)(EXTI9_5_IRQn));

  /* Enable the TS in interrupt mode */
  /* In that case the INT output of FT6206 when new touch is available */
  /* is active on low level and directed on EXTI */
  ts_driver->EnableIT(i2c_addr);

  return (ts_status);
}

#if (TS_MULTI_TOUCH_SUPPORTED == 1)
/**
  * @brief  Update gesture Id following a touch detected.
  * @param  ts_data: Pointer to touch screen current state structure
  * @retval OK if all initializations are OK. Other value if error.
  */
static err_code_t drvTsUpdateGestureId(void)
{
  uint32_t gesture_id = 0;
  err_code_t  ts_status = OK;

  /* Get gesture Id */
  ft6x06_TS_GetGestureID(i2c_addr, &gesture_id);

  /* Remap gesture Id to a TS_GestureIdTypeDef value */
  switch(gesture_id)
  {
    case FT6206_GEST_ID_NO_GESTURE :
      ts_data.gesture_id = GEST_ID_NO_GESTURE;
      break;
    case FT6206_GEST_ID_MOVE_UP :
      ts_data.gesture_id = GEST_ID_MOVE_UP;
      break;
    case FT6206_GEST_ID_MOVE_RIGHT :
      ts_data.gesture_id = GEST_ID_MOVE_RIGHT;
      break;
    case FT6206_GEST_ID_MOVE_DOWN :
      ts_data.gesture_id = GEST_ID_MOVE_DOWN;
      break;
    case FT6206_GEST_ID_MOVE_LEFT :
      ts_data.gesture_id = GEST_ID_MOVE_LEFT;
      break;
    case FT6206_GEST_ID_ZOOM_IN :
      ts_data.gesture_id = GEST_ID_ZOOM_IN;
      break;
    case FT6206_GEST_ID_ZOOM_OUT :
      ts_data.gesture_id = GEST_ID_ZOOM_OUT;
      break;
    default :
      ts_status = ERR_TS;
      break;
  } /* of switch(gesture_id) */

  return(ts_status);
}
#endif /* TS_MULTI_TOUCH_SUPPORTED == 1 */


/**
  * @brief  Initializes the TS_INT pin MSP.
  */
static void drvTsMspInit(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  __HAL_RCC_GPIOJ_CLK_ENABLE();

  /* GPIO configuration in input for TouchScreen interrupt signal on TS_INT pin */
  GPIO_InitStruct.Pin       = ((uint32_t)GPIO_PIN_5);

  GPIO_InitStruct.Mode      = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOJ, &GPIO_InitStruct);
}



/******************************** LINK TS (TouchScreen) ***********************/


/**
  * @brief  Initialize I2C communication
  *         channel from MCU to TouchScreen (TS).
  */
void TS_IO_Init(void)
{
}

/**
  * @brief  Writes single data with I2C communication
  *         channel from MCU to TouchScreen.
  * @param  Addr: I2C address
  * @param  Reg: Register address
  * @param  Value: Data to be written
  */
void TS_IO_Write(uint8_t Addr, uint8_t Reg, uint8_t Value)
{
  bool ret;


  ret = i2cWriteByte(_DEF_I2C1, Addr, Reg, Value, 10);
  if (ret != true)
  {
    i2cRecovery(_DEF_I2C1);
  }
}

/**
  * @brief  Reads single data with I2C communication
  *         channel from TouchScreen.
  * @param  Addr: I2C address
  * @param  Reg: Register address
  * @retval Read data
  */
uint8_t TS_IO_Read(uint8_t Addr, uint8_t Reg)
{
  bool ret;
  uint8_t Value = 0;


  ret = i2cReadBytes(_DEF_I2C1, Addr, Reg, &Value, 1, 10);

  if (ret != true)
  {
    i2cRecovery(_DEF_I2C1);
  }

  return Value;
}

/**
  * @brief  Reads multiple data with I2C communication
  *         channel from TouchScreen.
  * @param  Addr: I2C address
  * @param  Reg: Register address
  * @param  Buffer: Pointer to data buffer
  * @param  Length: Length of the data
  * @retval Number of read data
  */
uint16_t TS_IO_ReadMultiple(uint8_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length)
{
  bool ret;
  uint16_t status = 0;

  ret = i2cReadBytes(_DEF_I2C1, Addr, Reg, Buffer, Length, 50);

  if (ret != true)
  {
    i2cRecovery(_DEF_I2C1);
    status = 1;
  }



  return status;
}

/**
  * @brief  Writes multiple data with I2C communication
  *         channel from MCU to TouchScreen.
  * @param  Addr: I2C address
  * @param  Reg: Register address
  * @param  Buffer: Pointer to data buffer
  * @param  Length: Length of the data
  */
void TS_IO_WriteMultiple(uint8_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length)
{
  bool ret;

  ret = i2cWriteBytes(_DEF_I2C1, Addr, Reg, Buffer, Length, 100);

  if (ret != true)
  {
    i2cRecovery(_DEF_I2C1);
  }
}

/**
  * @brief  Delay function used in TouchScreen low level driver.
  * @param  Delay: Delay in ms
  */
void TS_IO_Delay(uint32_t Delay)
{
  HAL_Delay(Delay);
}



