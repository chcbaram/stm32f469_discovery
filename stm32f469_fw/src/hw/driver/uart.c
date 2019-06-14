/*
 * uart.c
 *
 *  Created on: 2019. 3. 7.
 *      Author: HanCheol Cho
 */




#include "uart.h"
#include "qbuffer.h"



/*
  _DEF_UART1
      USART1

*/



#define UART_MODE_POLLING       0
#define UART_MODE_INTERRUPT     1
#define UART_MODE_DMA           2
#define UART_MODE_VCP           3

#define UART_HW_NONE            0
#define UART_HW_STM32_VCP       1
#define UART_HW_STM32_UART      2
#define UART_HW_MAX14830        3


#define UART_RX_BUF_LENGTH      256




typedef struct
{
  bool     is_open;
  uint8_t  ch;
  uint32_t baud;
  uint8_t  tx_mode;
  uint8_t  rx_mode;
  uint8_t  hw_driver;

  uint8_t  rx_buf[UART_RX_BUF_LENGTH];

  qbuffer_t qbuffer_rx;

  UART_HandleTypeDef *handle;

  bool  tx_done;
  void  (*txDoneISR)(void);

  uint32_t err_cnt;
} uart_t;



static uart_t uart_tbl[UART_MAX_CH];


extern UART_HandleTypeDef huart3;


static UART_HandleTypeDef *p_uart_handle[UART_MAX_CH] =
    {
        &huart3
    };



void uartStartRx(uint8_t channel);
void uartRxHandler(uint8_t channel);



bool uartInit(void)
{
  uint8_t i;


  for (i=0; i<UART_MAX_CH; i++)
  {
    uart_tbl[i].is_open  = false;
    uart_tbl[i].rx_mode  = UART_MODE_POLLING;
    uart_tbl[i].tx_mode  = UART_MODE_POLLING;
    uart_tbl[i].tx_done  = false;
    uart_tbl[i].txDoneISR = NULL;
    uart_tbl[i].err_cnt  = 0;
    uart_tbl[i].hw_driver = UART_HW_NONE;
  }

  return true;
}

bool uartOpen(uint8_t channel, uint32_t baud)
{
  bool ret = false;
  uart_t *p_uart;


  if (channel >= UART_MAX_CH)
  {
    return false;
  }

  switch(channel)
  {
    case _DEF_UART1:
    case _DEF_UART2:
    case _DEF_UART3:
    case _DEF_UART4:

      p_uart = &uart_tbl[channel];

      p_uart->baud     = baud;

      p_uart->rx_mode  = UART_MODE_INTERRUPT;
      p_uart->tx_mode  = UART_MODE_POLLING;
      p_uart->hw_driver = UART_HW_STM32_UART;

      p_uart->handle =  p_uart_handle[channel];



      p_uart->handle->Init.BaudRate     = baud;
      p_uart->handle->Init.WordLength   = UART_WORDLENGTH_8B;
      p_uart->handle->Init.StopBits     = UART_STOPBITS_1;
      p_uart->handle->Init.Parity       = UART_PARITY_NONE;
      p_uart->handle->Init.Mode         = UART_MODE_TX_RX;
      p_uart->handle->Init.HwFlowCtl    = UART_HWCONTROL_NONE;
      p_uart->handle->Init.OverSampling = UART_OVERSAMPLING_16;

      qbufferCreate(&p_uart->qbuffer_rx, p_uart->rx_buf, UART_RX_BUF_LENGTH);

      HAL_UART_DeInit(p_uart->handle);
      HAL_UART_Init(p_uart->handle);

      p_uart->is_open  = true;

      uartStartRx(channel);
      ret = true;
      break;
  }

  return ret;
}

void uartSetTxDoneISR(uint8_t channel, void (*func)(void))
{
  uart_tbl[channel].txDoneISR = func;
}

void uartStartRx(uint8_t channel)
{
  uart_t *p_uart = &uart_tbl[channel];

  if (p_uart->rx_mode == UART_MODE_INTERRUPT)
  {
    HAL_UART_Receive_IT(p_uart->handle, p_uart->rx_buf, 1);
  }
  if (p_uart->rx_mode == UART_MODE_DMA)
  {
    HAL_UART_Receive_DMA(p_uart->handle, (uint8_t *)p_uart->qbuffer_rx.p_buf, p_uart->qbuffer_rx.length);
  }
}

bool uartClose(uint8_t channel)
{
  bool ret = false;


  if (channel >= UART_MAX_CH)
  {
    return false;
  }


  if (uart_tbl[channel].is_open == true && uart_tbl[channel].hw_driver == UART_HW_STM32_UART)
  {
    if(HAL_UART_DeInit(uart_tbl[channel].handle) == HAL_OK)
    {
      ret = true;
    }
  }


  return ret;
}

uint32_t uartAvailable(uint8_t channel)
{
  uint32_t ret;
  uart_t *p_uart = &uart_tbl[channel];


  if (channel >= UART_MAX_CH)
  {
    return 0;
  }

  if (p_uart->rx_mode == UART_MODE_INTERRUPT)
  {
    ret = qbufferAvailable(&uart_tbl[channel].qbuffer_rx);
  }
  if (p_uart->rx_mode == UART_MODE_DMA)
  {
    //p_uart->qbuffer_rx.ptr_in = p_uart->qbuffer_rx.length - ((DMA_Stream_TypeDef *)p_uart->hdma_rx.Instance)->NDTR;
    ret = qbufferAvailable(&p_uart->qbuffer_rx);
  }

  return ret;
}

void uartFlush(uint8_t channel)
{
  if (uart_tbl[channel].rx_mode == UART_MODE_INTERRUPT)
  {
    qbufferFlush(&uart_tbl[channel].qbuffer_rx);
  }
  if (uart_tbl[channel].rx_mode == UART_MODE_DMA)
  {
    //uart_tbl[channel].qbuffer_rx.ptr_in  = uart_tbl[channel].qbuffer_rx.length - ((DMA_Stream_TypeDef *)uart_tbl[channel].hdma_rx.Instance)->NDTR;
    uart_tbl[channel].qbuffer_rx.ptr_out = uart_tbl[channel].qbuffer_rx.ptr_in;
  }
}

void uartPutch(uint8_t channel, uint8_t ch)
{
  uartWrite(channel, &ch, 1 );
}

uint8_t uartGetch(uint8_t channel)
{
  uint8_t ret = 0;


  while(1)
  {
    if (uartAvailable(channel) > 0)
    {
      ret = uartRead(channel);
      break;
    }
  }

  return ret;
}

int32_t uartWrite(uint8_t channel, uint8_t *p_data, uint32_t length)
{
  int32_t ret = 0;
  uart_t *p_uart = &uart_tbl[channel];


  if (p_uart->tx_mode == UART_MODE_POLLING)
  {
    if (HAL_UART_Transmit(p_uart->handle, (uint8_t*)p_data, length, 1000) == HAL_OK)
    {
      ret = length;
    }
  }
  if (p_uart->tx_mode == UART_MODE_DMA)
  {
    if (HAL_UART_Transmit_DMA(p_uart->handle, p_data, length) == HAL_OK)
    {
      ret = length;
    }
  }
  return ret;
}

uint8_t uartRead(uint8_t channel)
{
  uint8_t ret = 0;
  uart_t *p_uart = &uart_tbl[channel];


  if (p_uart->rx_mode == UART_MODE_INTERRUPT)
  {
    qbufferRead(&p_uart->qbuffer_rx, &ret, 1);
  }
  if (p_uart->rx_mode == UART_MODE_DMA)
  {
    qbufferRead(&p_uart->qbuffer_rx, &ret, 1);
  }
  return ret;
}

bool uartSendBreak(uint8_t channel)
{
  return true;
}


int32_t uartPrintf(uint8_t channel, const char *fmt, ...)
{
  int32_t ret = 0;
  va_list arg;
  va_start (arg, fmt);
  int32_t len;
  char print_buffer[256];


  len = vsnprintf(print_buffer, 255, fmt, arg);
  va_end (arg);

  ret = uartWrite(channel, (uint8_t *)print_buffer, len);

  return ret;
}

void uartRxHandler(uint8_t channel)
{
  uart_t *p_uart = &uart_tbl[channel];


  if(p_uart->rx_mode == UART_MODE_INTERRUPT)
  {
    qbufferWrite(&p_uart->qbuffer_rx, &p_uart->rx_buf[0], 1);

    __HAL_UNLOCK(p_uart->handle);
    uartStartRx(channel);
  }
}

void uartErrHandler(uint8_t channel)
{
  uartStartRx(channel);
  uartFlush(channel);

  uart_tbl[channel].err_cnt++;
}

uint32_t uartGetErrCnt(uint8_t channel)
{
  return uart_tbl[channel].err_cnt;
}

void uartResetErrCnt(uint8_t channel)
{
  uartStartRx(channel);
  uartFlush(channel);

  uart_tbl[channel].err_cnt = 0;
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
}




void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
  if (UartHandle->Instance == uart_tbl[_DEF_UART1].handle->Instance)
  {
    uartErrHandler(_DEF_UART1);
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == uart_tbl[_DEF_UART1].handle->Instance)
  {
    uartRxHandler(_DEF_UART1);
  }
}
