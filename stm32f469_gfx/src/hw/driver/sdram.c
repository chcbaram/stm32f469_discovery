/*
 * sdram.c
 *
 *  Created on: 2019. 6. 23.
 *      Author: Baram
 */




#include "sdram.h"



#define SDRAM_OK            ((uint8_t)0x00)
#define SDRAM_ERROR         ((uint8_t)0x01)
#define SDRAM_DEVICE_ADDR   ((uint32_t)0xC0000000)
#define SDRAM_DEVICE_SIZE   ((uint32_t)0x1000000)     // 16MBytes




uint8_t BSP_SDRAM_Init(void);
uint8_t BSP_SDRAM_DeInit(void);
void    BSP_SDRAM_Initialization_sequence(uint32_t RefreshCount);
uint8_t BSP_SDRAM_ReadData(uint32_t uwStartAddress, uint32_t *pData, uint32_t uwDataSize);
uint8_t BSP_SDRAM_ReadData_DMA(uint32_t uwStartAddress, uint32_t *pData, uint32_t uwDataSize);
uint8_t BSP_SDRAM_WriteData(uint32_t uwStartAddress, uint32_t *pData, uint32_t uwDataSize);
uint8_t BSP_SDRAM_WriteData_DMA(uint32_t uwStartAddress, uint32_t *pData, uint32_t uwDataSize);
uint8_t BSP_SDRAM_Sendcmd(FMC_SDRAM_CommandTypeDef *SdramCmd);
void    BSP_SDRAM_DMA_IRQHandler(void);

/* These function can be modified in case the current settings (e.g. DMA stream)
   need to be changed for specific application needs */
void    BSP_SDRAM_MspInit(SDRAM_HandleTypeDef  *hsdram, void *Params);
void    BSP_SDRAM_MspDeInit(SDRAM_HandleTypeDef  *hsdram, void *Params);



bool sdramInit(void)
{
  bool ret = true;;

  if (BSP_SDRAM_Init() != 0x00)
  {
    ret = false;
  }

  return ret;
}

uint32_t sdramGetAddr(void)
{
  return SDRAM_DEVICE_ADDR;
}

uint32_t sdramGetLength(void)
{
  return SDRAM_DEVICE_SIZE;
}





#define SDRAM_MEMORY_WIDTH FMC_SDRAM_MEM_BUS_WIDTH_32
#define SDCLOCK_PERIOD     FMC_SDRAM_CLOCK_PERIOD_2

/* SDRAM refresh counter (90 MHz SD clock) */
#define REFRESH_COUNT       ((uint32_t)0x0569)
#define  SDRAM_TIMEOUT      ((uint32_t)0xFFFF)

/* DMA definitions for SDRAM DMA transfer */
#define __DMAx_CLK_ENABLE     __HAL_RCC_DMA2_CLK_ENABLE
#define __DMAx_CLK_DISABLE    __HAL_RCC_DMA2_CLK_DISABLE
#define SDRAM_DMAx_CHANNEL    DMA_CHANNEL_0
#define SDRAM_DMAx_STREAM     DMA2_Stream0
#define SDRAM_DMAx_IRQn       DMA2_Stream0_IRQn
#define SDRAM_DMAx_IRQHandler DMA2_Stream0_IRQHandler


/**
  * @brief  FMC SDRAM Mode definition register defines
  */
#define SDRAM_MODEREG_BURST_LENGTH_1             ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_LENGTH_2             ((uint16_t)0x0001)
#define SDRAM_MODEREG_BURST_LENGTH_4             ((uint16_t)0x0002)
#define SDRAM_MODEREG_BURST_LENGTH_8             ((uint16_t)0x0004)
#define SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL      ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_TYPE_INTERLEAVED     ((uint16_t)0x0008)
#define SDRAM_MODEREG_CAS_LATENCY_2              ((uint16_t)0x0020)
#define SDRAM_MODEREG_CAS_LATENCY_3              ((uint16_t)0x0030)
#define SDRAM_MODEREG_OPERATING_MODE_STANDARD    ((uint16_t)0x0000)
#define SDRAM_MODEREG_WRITEBURST_MODE_PROGRAMMED ((uint16_t)0x0000)
#define SDRAM_MODEREG_WRITEBURST_MODE_SINGLE     ((uint16_t)0x0200)



static SDRAM_HandleTypeDef sdramHandle;
static FMC_SDRAM_TimingTypeDef Timing;
static FMC_SDRAM_CommandTypeDef Command;



/**
  * @brief  Initializes the SDRAM device.
  * @retval SDRAM status
  */
uint8_t BSP_SDRAM_Init(void)
{
  static uint8_t sdramstatus = SDRAM_ERROR;

  /* SDRAM device configuration */
  sdramHandle.Instance = FMC_SDRAM_DEVICE;

  /* Timing configuration for 90 MHz as SD clock frequency (System clock is up to 180 MHz) */
  Timing.LoadToActiveDelay    = 2;
  Timing.ExitSelfRefreshDelay = 7;
  Timing.SelfRefreshTime      = 4;
  Timing.RowCycleDelay        = 7;
  Timing.WriteRecoveryTime    = 2;
  Timing.RPDelay              = 2;
  Timing.RCDDelay             = 2;

  sdramHandle.Init.SDBank             = FMC_SDRAM_BANK1;
  sdramHandle.Init.ColumnBitsNumber   = FMC_SDRAM_COLUMN_BITS_NUM_8;
  sdramHandle.Init.RowBitsNumber      = FMC_SDRAM_ROW_BITS_NUM_12;
  sdramHandle.Init.MemoryDataWidth    = SDRAM_MEMORY_WIDTH;
  sdramHandle.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  sdramHandle.Init.CASLatency         = FMC_SDRAM_CAS_LATENCY_3;
  sdramHandle.Init.WriteProtection    = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  sdramHandle.Init.SDClockPeriod      = SDCLOCK_PERIOD;
  sdramHandle.Init.ReadBurst          = FMC_SDRAM_RBURST_ENABLE;
  sdramHandle.Init.ReadPipeDelay      = FMC_SDRAM_RPIPE_DELAY_0;

  /* SDRAM controller initialization */
  /* __weak function can be surcharged by the application code */
  BSP_SDRAM_MspInit(&sdramHandle, (void *)NULL);
  if(HAL_SDRAM_Init(&sdramHandle, &Timing) != HAL_OK)
  {
    sdramstatus = SDRAM_ERROR;
  }
  else
  {
    sdramstatus = SDRAM_OK;
  }

  /* SDRAM initialization sequence */
  BSP_SDRAM_Initialization_sequence(REFRESH_COUNT);

  return sdramstatus;
}

/**
  * @brief  DeInitializes the SDRAM device.
  * @retval SDRAM status : SDRAM_OK or SDRAM_ERROR.
  */
uint8_t BSP_SDRAM_DeInit(void)
{
  static uint8_t sdramstatus = SDRAM_ERROR;

  /* SDRAM device configuration */
  sdramHandle.Instance = FMC_SDRAM_DEVICE;

  if(HAL_SDRAM_DeInit(&sdramHandle) == HAL_OK)
  {
    sdramstatus = SDRAM_OK;

  /* SDRAM controller De-initialization */
   BSP_SDRAM_MspDeInit(&sdramHandle, (void *)NULL);
  }

  return sdramstatus;
}


/**
  * @brief  Programs the SDRAM device.
  * @param  RefreshCount: SDRAM refresh counter value
  */
void BSP_SDRAM_Initialization_sequence(uint32_t RefreshCount)
{
  __IO uint32_t tmpmrd = 0;

  /* Step 1: Configure a clock configuration enable command */
  Command.CommandMode            = FMC_SDRAM_CMD_CLK_ENABLE;
  Command.CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK1;
  Command.AutoRefreshNumber      = 1;
  Command.ModeRegisterDefinition = 0;

  /* Send the command */
  HAL_SDRAM_SendCommand(&sdramHandle, &Command, SDRAM_TIMEOUT);

  /* Step 2: Insert 100 us minimum delay */
  /* Inserted delay is equal to 1 ms due to systick time base unit (ms) */
  HAL_Delay(1);

  /* Step 3: Configure a PALL (precharge all) command */
  Command.CommandMode            = FMC_SDRAM_CMD_PALL;
  Command.CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK1;
  Command.AutoRefreshNumber      = 1;
  Command.ModeRegisterDefinition = 0;

  /* Send the command */
  HAL_SDRAM_SendCommand(&sdramHandle, &Command, SDRAM_TIMEOUT);

  /* Step 4: Configure an Auto Refresh command */
  Command.CommandMode            = FMC_SDRAM_CMD_AUTOREFRESH_MODE;
  Command.CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK1;
  Command.AutoRefreshNumber      = 8;
  Command.ModeRegisterDefinition = 0;

  /* Send the command */
  HAL_SDRAM_SendCommand(&sdramHandle, &Command, SDRAM_TIMEOUT);

  /* Step 5: Program the external memory mode register */
  tmpmrd = (uint32_t)SDRAM_MODEREG_BURST_LENGTH_1          |\
                     SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL   |\
                     SDRAM_MODEREG_CAS_LATENCY_3           |\
                     SDRAM_MODEREG_OPERATING_MODE_STANDARD |\
                     SDRAM_MODEREG_WRITEBURST_MODE_SINGLE;

  Command.CommandMode            = FMC_SDRAM_CMD_LOAD_MODE;
  Command.CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK1;
  Command.AutoRefreshNumber      = 1;
  Command.ModeRegisterDefinition = tmpmrd;

  /* Send the command */
  HAL_SDRAM_SendCommand(&sdramHandle, &Command, SDRAM_TIMEOUT);

  /* Step 6: Set the refresh rate counter */
  /* Set the device refresh rate */
  HAL_SDRAM_ProgramRefreshRate(&sdramHandle, RefreshCount);
}

/**
  * @brief  Reads an mount of data from the SDRAM memory in polling mode.
  * @param  uwStartAddress: Read start address
  * @param  pData: Pointer to data to be read
  * @param  uwDataSize: Size of read data from the memory
  * @retval SDRAM status : SDRAM_OK or SDRAM_ERROR.
  */
uint8_t BSP_SDRAM_ReadData(uint32_t uwStartAddress, uint32_t *pData, uint32_t uwDataSize)
{
  if(HAL_SDRAM_Read_32b(&sdramHandle, (uint32_t *)uwStartAddress, pData, uwDataSize) != HAL_OK)
  {
    return SDRAM_ERROR;
  }
  else
  {
    return SDRAM_OK;
  }
}

/**
  * @brief  Reads an mount of data from the SDRAM memory in DMA mode.
  * @param  uwStartAddress: Read start address
  * @param  pData: Pointer to data to be read
  * @param  uwDataSize: Size of read data from the memory
  * @retval SDRAM status : SDRAM_OK or SDRAM_ERROR.
  */
uint8_t BSP_SDRAM_ReadData_DMA(uint32_t uwStartAddress, uint32_t *pData, uint32_t uwDataSize)
{
  if(HAL_SDRAM_Read_DMA(&sdramHandle, (uint32_t *)uwStartAddress, pData, uwDataSize) != HAL_OK)
  {
    return SDRAM_ERROR;
  }
  else
  {
    return SDRAM_OK;
  }
}

/**
  * @brief  Writes an mount of data to the SDRAM memory in polling mode.
  * @param  uwStartAddress: Write start address
  * @param  pData: Pointer to data to be written
  * @param  uwDataSize: Size of written data from the memory
  * @retval SDRAM status : SDRAM_OK or SDRAM_ERROR.
  */
uint8_t BSP_SDRAM_WriteData(uint32_t uwStartAddress, uint32_t *pData, uint32_t uwDataSize)
{
  if(HAL_SDRAM_Write_32b(&sdramHandle, (uint32_t *)uwStartAddress, pData, uwDataSize) != HAL_OK)
  {
    return SDRAM_ERROR;
  }
  else
  {
    return SDRAM_OK;
  }
}

/**
  * @brief  Writes an mount of data to the SDRAM memory in DMA mode.
  * @param  uwStartAddress: Write start address
  * @param  pData: Pointer to data to be written
  * @param  uwDataSize: Size of written data from the memory
  * @retval SDRAM status : SDRAM_OK or SDRAM_ERROR.
  */
uint8_t BSP_SDRAM_WriteData_DMA(uint32_t uwStartAddress, uint32_t *pData, uint32_t uwDataSize)
{
  if(HAL_SDRAM_Write_DMA(&sdramHandle, (uint32_t *)uwStartAddress, pData, uwDataSize) != HAL_OK)
  {
    return SDRAM_ERROR;
  }
  else
  {
    return SDRAM_OK;
  }
}

/**
  * @brief  Sends command to the SDRAM bank.
  * @param  SdramCmd: Pointer to SDRAM command structure
  * @retval HAL status : SDRAM_OK or SDRAM_ERROR.
  */
uint8_t BSP_SDRAM_Sendcmd(FMC_SDRAM_CommandTypeDef *SdramCmd)
{
  if(HAL_SDRAM_SendCommand(&sdramHandle, SdramCmd, SDRAM_TIMEOUT) != HAL_OK)
  {
    return SDRAM_ERROR;
  }
  else
  {
    return SDRAM_OK;
  }
}

/**
  * @brief  Handles SDRAM DMA transfer interrupt request.
  */
void BSP_SDRAM_DMA_IRQHandler(void)
{
  HAL_DMA_IRQHandler(sdramHandle.hdma);
}

/**
  * @brief  Initializes SDRAM MSP.
  * @note   This function can be surcharged by application code.
  * @param  hsdram: pointer on SDRAM handle
  * @param  Params: pointer on additional configuration parameters, can be NULL.
  */
__weak void BSP_SDRAM_MspInit(SDRAM_HandleTypeDef  *hsdram, void *Params)
{
  static DMA_HandleTypeDef dma_handle;
  GPIO_InitTypeDef gpio_init_structure;

  if(hsdram != (SDRAM_HandleTypeDef  *)NULL)
  {
    /* Enable FMC clock */
    __HAL_RCC_FMC_CLK_ENABLE();

    /* Enable chosen DMAx clock */
    __DMAx_CLK_ENABLE();

    /* Enable GPIOs clock */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOI_CLK_ENABLE();

    /* Common GPIO configuration */
    gpio_init_structure.Mode      = GPIO_MODE_AF_PP;
    gpio_init_structure.Pull      = GPIO_PULLUP;
    gpio_init_structure.Speed     = GPIO_SPEED_FAST;
    gpio_init_structure.Alternate = GPIO_AF12_FMC;

    /* GPIOC configuration : PC0 is SDNWE */
    gpio_init_structure.Pin   = GPIO_PIN_0;
    HAL_GPIO_Init(GPIOC, &gpio_init_structure);

    /* GPIOD configuration */
    gpio_init_structure.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_8| GPIO_PIN_9 | GPIO_PIN_10 |\
                                GPIO_PIN_14 | GPIO_PIN_15;


    HAL_GPIO_Init(GPIOD, &gpio_init_structure);

    /* GPIOE configuration */
    gpio_init_structure.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_7| GPIO_PIN_8 | GPIO_PIN_9 |\
                                GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 |\
                                GPIO_PIN_15;

    HAL_GPIO_Init(GPIOE, &gpio_init_structure);

    /* GPIOF configuration */
    gpio_init_structure.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2| GPIO_PIN_3 | GPIO_PIN_4 |\
                                GPIO_PIN_5 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 |\
                                GPIO_PIN_15;

    HAL_GPIO_Init(GPIOF, &gpio_init_structure);

    /* GPIOG configuration */
    gpio_init_structure.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4| GPIO_PIN_5 | GPIO_PIN_8 |\
                                GPIO_PIN_15;
    HAL_GPIO_Init(GPIOG, &gpio_init_structure);

    /* GPIOH configuration */
    gpio_init_structure.Pin   = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_8 | GPIO_PIN_9 |\
                                GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 |\
                                GPIO_PIN_15;
    HAL_GPIO_Init(GPIOH, &gpio_init_structure);

    /* GPIOI configuration */
    gpio_init_structure.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 |\
                                GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_9 | GPIO_PIN_10;
    HAL_GPIO_Init(GPIOI, &gpio_init_structure);

    /* Configure common DMA parameters */
    dma_handle.Init.Channel             = SDRAM_DMAx_CHANNEL;
    dma_handle.Init.Direction           = DMA_MEMORY_TO_MEMORY;
    dma_handle.Init.PeriphInc           = DMA_PINC_ENABLE;
    dma_handle.Init.MemInc              = DMA_MINC_ENABLE;
    dma_handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    dma_handle.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
    dma_handle.Init.Mode                = DMA_NORMAL;
    dma_handle.Init.Priority            = DMA_PRIORITY_HIGH;
    dma_handle.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    dma_handle.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    dma_handle.Init.MemBurst            = DMA_MBURST_SINGLE;
    dma_handle.Init.PeriphBurst         = DMA_PBURST_SINGLE;

    dma_handle.Instance = SDRAM_DMAx_STREAM;

    /* Associate the DMA handle */
    __HAL_LINKDMA(hsdram, hdma, dma_handle);

    /* Deinitialize the stream for new transfer */
    HAL_DMA_DeInit(&dma_handle);

    /* Configure the DMA stream */
    HAL_DMA_Init(&dma_handle);

    /* NVIC configuration for DMA transfer complete interrupt */
    HAL_NVIC_SetPriority(SDRAM_DMAx_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(SDRAM_DMAx_IRQn);

  } /* of if(hsdram != (SDRAM_HandleTypeDef  *)NULL) */
}

/**
  * @brief  DeInitializes SDRAM MSP.
  * @note   This function can be surcharged by application code.
  * @param  hsdram: pointer on SDRAM handle
  * @param  Params: pointer on additional configuration parameters, can be NULL.
  */
__weak void BSP_SDRAM_MspDeInit(SDRAM_HandleTypeDef  *hsdram, void *Params)
{
    static DMA_HandleTypeDef dma_handle;

    if(hsdram != (SDRAM_HandleTypeDef  *)NULL)
    {
      /* Disable NVIC configuration for DMA interrupt */
      HAL_NVIC_DisableIRQ(SDRAM_DMAx_IRQn);

      /* Deinitialize the stream for new transfer */
      dma_handle.Instance = SDRAM_DMAx_STREAM;
      HAL_DMA_DeInit(&dma_handle);

      /* DeInit GPIO pins can be done in the application
       (by surcharging this __weak function) */

      /* GPIO pins clock, FMC clock and DMA clock can be shut down in the application
       by surcharging this __weak function */

    } /* of if(hsdram != (SDRAM_HandleTypeDef  *)NULL) */
}
