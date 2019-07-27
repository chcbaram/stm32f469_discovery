/*
 * ts.c
 *
 *  Created on: Feb 13, 2018
 *      Author: opus
 */


#include "ts.h"
#include "ft6x06.h"
#include "i2c.h"


#define TS_I2C_ADDRESS                   (((uint16_t)0x54)>>1)
#define TS_I2C_ADDRESS_A02               (((uint16_t)0x70)>>1)


#define TS_INT_PIN                        ((uint32_t)GPIO_PIN_5)
#define TS_INT_GPIO_PORT                  ((GPIO_TypeDef*)GPIOJ)
#define TS_INT_GPIO_CLK_ENABLE()          __HAL_RCC_GPIOJ_CLK_ENABLE()
#define TS_INT_GPIO_CLK_DISABLE()         __HAL_RCC_GPIOJ_CLK_DISABLE()
#define TS_INT_EXTI_IRQn                  EXTI9_5_IRQn


__weak void BSP_TS_INT_MspInit(void);



static TS_DrvTypeDef *ts_driver;
static uint8_t  ts_orientation;
static uint8_t  I2C_Address = 0;

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

/**
  * @}
  */

/** @defgroup STM32H745I_DISCOVERY_TS_Private_Function_Prototypes  Private Function Prototypes
  * @{
  */
/**
  * @}
  */

/** @defgroup STM32H745I_DISCOVERY_TS_Exported_Functions Exported Functions
  * @{
  */

/**
  * @brief  Initializes and configures the touch screen functionalities and
  *         configures all necessary hardware resources (GPIOs, I2C, clocks..).
  * @param  ts_SizeX: Maximum X size of the TS area on LCD
  * @param  ts_SizeY: Maximum Y size of the TS area on LCD
  * @retval TS_OK if all initializations are OK. Other value if error.
  */
uint8_t tsInit(uint16_t ts_SizeX, uint16_t ts_SizeY)
{
  uint8_t ts_status = TS_OK;
  uint8_t ts_id1, ts_id2 = 0;
  /* Note : I2C_Address is un-initialized here, but is not used at all in init function */
  /* but the prototype of Init() is like that in template and should be respected       */

  /* Initialize the communication channel to sensor (I2C) if necessary */
  /* that is initialization is done only once after a power up         */
  ft6x06_ts_drv.Init(I2C_Address);

  ts_id1 = ft6x06_ts_drv.ReadID(TS_I2C_ADDRESS);
  if(ts_id1 != FT6206_ID_VALUE)
  {
    ts_id2 = ft6x06_ts_drv.ReadID(TS_I2C_ADDRESS_A02);
    I2C_Address    = TS_I2C_ADDRESS_A02;
  }
  else
  {
    I2C_Address    = TS_I2C_ADDRESS;
  }

  /* Scan FT6xx6 TouchScreen IC controller ID register by I2C Read       */
  /* Verify this is a FT6206 or FT6336G, otherwise this is an error case */
  if((ts_id1 == FT6206_ID_VALUE) || (ts_id2 == FT6206_ID_VALUE))
  {
    /* Found FT6206 : Initialize the TS driver structure */
    ts_driver = &ft6x06_ts_drv;

    /* Get LCD chosen orientation */
    if(ts_SizeX < ts_SizeY)
    {
      ts_orientation = TS_SWAP_NONE;
    }
    else
    {
      ts_orientation = TS_SWAP_XY | TS_SWAP_Y;
    }

    if(ts_status == TS_OK)
    {
      /* Software reset the TouchScreen */
      ts_driver->Reset(I2C_Address);

      /* Calibrate, Configure and Start the TouchScreen driver */
      ts_driver->Start(I2C_Address);

    } /* of if(ts_status == TS_OK) */
  }
  else
  {
    ts_status = TS_DEVICE_NOT_FOUND;
  }

  return (ts_status);
}

/**
  * @brief  DeInitializes the TouchScreen.
  * @retval TS state
  */
uint8_t tsDeInit(void)
{
  /* Actually ts_driver does not provide a DeInit function */
  return TS_OK;
}

/**
  * @brief  Configures and enables the touch screen interrupts.
  * @retval TS_OK if all initializations are OK. Other value if error.
  */
uint8_t tsITConfig(void)
{
  uint8_t ts_status = TS_OK;
  GPIO_InitTypeDef gpio_init_structure;

  /* Msp Init of GPIO used for TS_INT pin coming from TouchScreen driver IC FT6x06 */
  /* When touchscreen is operated in interrupt mode */
  BSP_TS_INT_MspInit();

  /* Configure Interrupt mode for TS_INT pin falling edge : when a new touch is available */
  /* TS_INT pin is active on low level on new touch available */
  gpio_init_structure.Pin = TS_INT_PIN;
  gpio_init_structure.Pull = GPIO_PULLUP;
  gpio_init_structure.Speed = GPIO_SPEED_FAST;
  gpio_init_structure.Mode = GPIO_MODE_IT_FALLING;
  HAL_GPIO_Init(TS_INT_GPIO_PORT, &gpio_init_structure);

  /* Enable and set the TS_INT EXTI Interrupt to an intermediate priority */
  HAL_NVIC_SetPriority((IRQn_Type)(TS_INT_EXTI_IRQn), 0x05, 0x00);
  HAL_NVIC_EnableIRQ((IRQn_Type)(TS_INT_EXTI_IRQn));

  /* Enable the TS in interrupt mode */
  /* In that case the INT output of FT6206 when new touch is available */
  /* is active on low level and directed on EXTI */
  ts_driver->EnableIT(I2C_Address);

  return (ts_status);
}

/**
  * @brief  Gets the touch screen interrupt status.
  * @retval TS_OK if all initializations are OK. Other value if error.
  */
uint8_t tsITGetStatus(void)
{
  /* Return the TS IT status */
  return (ts_driver->GetITStatus(I2C_Address));
}

/**
  * @brief  Returns status and positions of the touch screen.
  * @param  TS_State: Pointer to touch screen current state structure
  * @retval TS_OK if all initializations are OK. Other value if error.
  */
uint8_t tsGetState(TS_StateTypeDef *TS_State)
{
  static uint32_t _x[TS_MAX_NB_TOUCH] = {0, 0};
  static uint32_t _y[TS_MAX_NB_TOUCH] = {0, 0};
  uint8_t ts_status = TS_OK;
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
  TS_State->touchDetected = ts_driver->DetectTouch(I2C_Address);
  if(TS_State->touchDetected)
  {
    for(index=0; index < TS_State->touchDetected; index++)
    {
      /* Get each touch coordinates */
      ts_driver->GetXY(I2C_Address, &(Raw_x[index]), &(Raw_y[index]));

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


      TS_State->touchX[index] = _x[index];
      TS_State->touchY[index] = _y[index];

#if (TS_MULTI_TOUCH_SUPPORTED == 1)

      /* Get touch info related to the current touch */
      ft6x06_TS_GetTouchInfo(I2C_Address, index, &weight, &area, &event);

      /* Update TS_State structure */
      TS_State->touchWeight[index] = weight;
      TS_State->touchArea[index]   = area;

      /* Remap touch event */
      switch(event)
      {
        case FT6206_TOUCH_EVT_FLAG_PRESS_DOWN  :
          TS_State->touchEventId[index] = TOUCH_EVENT_PRESS_DOWN;
          break;
        case FT6206_TOUCH_EVT_FLAG_LIFT_UP :
          TS_State->touchEventId[index] = TOUCH_EVENT_LIFT_UP;
          break;
        case FT6206_TOUCH_EVT_FLAG_CONTACT :
          TS_State->touchEventId[index] = TOUCH_EVENT_CONTACT;
          break;
        case FT6206_TOUCH_EVT_FLAG_NO_EVENT :
          TS_State->touchEventId[index] = TOUCH_EVENT_NO_EVT;
          break;
        default :
          ts_status = TS_ERROR;
          break;
      } /* of switch(event) */

#endif /* TS_MULTI_TOUCH_SUPPORTED == 1 */

    } /* of for(index=0; index < TS_State->touchDetected; index++) */

#if (TS_MULTI_TOUCH_SUPPORTED == 1)
    /* Get gesture Id */
    ts_status = tsGet_GestureId(TS_State);
#endif /* TS_MULTI_TOUCH_SUPPORTED == 1 */

  } /* end of if(TS_State->touchDetected != 0) */

  return (ts_status);
}

#if (TS_MULTI_TOUCH_SUPPORTED == 1)
/**
  * @brief  Update gesture Id following a touch detected.
  * @param  TS_State: Pointer to touch screen current state structure
  * @retval TS_OK if all initializations are OK. Other value if error.
  */
uint8_t tsGet_GestureId(TS_StateTypeDef *TS_State)
{
  uint32_t gestureId = 0;
  uint8_t  ts_status = TS_OK;

  /* Get gesture Id */
  ft6x06_TS_GetGestureID(I2C_Address, &gestureId);

  /* Remap gesture Id to a TS_GestureIdTypeDef value */
  switch(gestureId)
  {
    case FT6206_GEST_ID_NO_GESTURE :
      TS_State->gestureId = GEST_ID_NO_GESTURE;
      break;
    case FT6206_GEST_ID_MOVE_UP :
      TS_State->gestureId = GEST_ID_MOVE_UP;
      break;
    case FT6206_GEST_ID_MOVE_RIGHT :
      TS_State->gestureId = GEST_ID_MOVE_RIGHT;
      break;
    case FT6206_GEST_ID_MOVE_DOWN :
      TS_State->gestureId = GEST_ID_MOVE_DOWN;
      break;
    case FT6206_GEST_ID_MOVE_LEFT :
      TS_State->gestureId = GEST_ID_MOVE_LEFT;
      break;
    case FT6206_GEST_ID_ZOOM_IN :
      TS_State->gestureId = GEST_ID_ZOOM_IN;
      break;
    case FT6206_GEST_ID_ZOOM_OUT :
      TS_State->gestureId = GEST_ID_ZOOM_OUT;
      break;
    default :
      ts_status = TS_ERROR;
      break;
  } /* of switch(gestureId) */

  return(ts_status);
}


/** @defgroup STM32469I-Discovery_TS_Private_Functions STM32469I Discovery TS Private Functions
  * @{
  */


/**
  * @brief  Function used to reset all touch data before a new acquisition
  *         of touch information.
  * @param  TS_State: Pointer to touch screen current state structure
  * @retval TS_OK if OK, TE_ERROR if problem found.
  */
uint8_t tsResetTouchData(TS_StateTypeDef *TS_State)
{
  uint8_t ts_status = TS_ERROR;
  uint32_t index;

  if (TS_State != (TS_StateTypeDef *)NULL)
  {
    TS_State->gestureId = GEST_ID_NO_GESTURE;
    TS_State->touchDetected = 0;

    for(index = 0; index < TS_MAX_NB_TOUCH; index++)
    {
      TS_State->touchX[index]       = 0;
      TS_State->touchY[index]       = 0;
      TS_State->touchArea[index]    = 0;
      TS_State->touchEventId[index] = TOUCH_EVENT_NO_EVT;
      TS_State->touchWeight[index]  = 0;
    }

    ts_status = TS_OK;

  } /* of if (TS_State != (TS_StateTypeDef *)NULL) */

  return (ts_status);
}
#endif /* TS_MULTI_TOUCH_SUPPORTED == 1 */


/**
  * @brief  Clears all touch screen interrupts.
  */
void tsITClear(void)
{
  /* Clear TS IT pending bits */
  ts_driver->ClearIT(I2C_Address);
}










__weak void BSP_TS_INT_MspInit(void)
{
  GPIO_InitTypeDef  gpio_init_structure;

  TS_INT_GPIO_CLK_ENABLE();

  /* GPIO configuration in input for TouchScreen interrupt signal on TS_INT pin */
  gpio_init_structure.Pin       = TS_INT_PIN;

  gpio_init_structure.Mode      = GPIO_MODE_INPUT;
  gpio_init_structure.Pull      = GPIO_PULLUP;
  gpio_init_structure.Speed     = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(TS_INT_GPIO_PORT, &gpio_init_structure);
}






/******************************** LINK TS (TouchScreen) ***********************/


/**
  * @brief  Initialize I2C communication
  *         channel from MCU to TouchScreen (TS).
  */
void TS_IO_Init(void)
{
  i2cBegin(_DEF_I2C1, 400);
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


  ret = i2cWriteByte(_DEF_I2C1, Addr, Reg, Value, 100);
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


  ret = i2cReadBytes(_DEF_I2C1, Addr, Reg, &Value, 1, 100);

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

  ret = i2cReadBytes(_DEF_I2C1, Addr, Reg, Buffer, Length, 500);

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



