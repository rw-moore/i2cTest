
#include "stm32h7xx_hal.h"
#include "comms.h"
#include "uart_io.h"
#include <string.h>

/*
 *  UART I/O.  Not thread safe.  Call from higher-priority
 *  interrupts at your own peril.
 *
 *  TODO:
 *  1. Break ring buffer code from UART initialization code
 *  2. Use data structures instead of static data where possible to
 *     allow multiple UART instances.  This is currently a tangled mess.
 *  3. NUCLEOH7 should be supplied to the compiler when compiling
 *     for debug.
 */


static UART_HandleTypeDef gHuart;

#ifdef NUCLEOH7

#define DEBUG_UART_INSTANCE USART3
#define DEBUG_UART_IRQ USART3_IRQn

void USART3_IRQHandler() {
  HAL_UART_IRQHandler(&gHuart);
}

#else

#define DEBUG_UART_INSTANCE UART4
#define DEBUG_UART_IRQ UART4_IRQn

void USART4_IRQHandler() {
  HAL_UART_IRQHandler(&gHuart);
}

#endif // NUCLEOH7


/* If BUFSZ is changed, both macros must be changed */
#define BUFSZ 512
#define INC_POINTER(value, cnt) ((value + cnt) & 0x1FF)

typedef struct {
  uint8_t data[BUFSZ];
  uint16_t tail;
  uint16_t head;
  uint16_t limit;
} io_buf;

static io_buf ibuf;
static io_buf obuf;


size_t next_write_chunk(io_buf* buf, size_t maxlen) {
  /* Get length of the next contiguous writable chunk of buffer, which
   * may end at BUFSZ or at end */
  size_t cnt = BUFSZ - (buf->head);
  if (buf->limit > buf->head) {
    cnt = buf->limit - buf->head;
  }
  if (cnt > maxlen) {
    cnt = maxlen;
  }
  return cnt;
}


size_t next_read_chunk(io_buf* buf, size_t maxlen) {
  /* Get length of the next contiguous readable chunk of buffer, which
   * may end at BUFSZ or at end.  The empty buffer condition must
   * return zero, unlike the write case above.
   */
  size_t cnt = BUFSZ - (buf->tail);
  if (buf->head >= buf->tail) {
    cnt = buf->head - buf->tail;
  }
  if (cnt > maxlen) {
    cnt = maxlen;
  }
  return cnt;
}


void uart_tx() {
  size_t cnt = next_read_chunk(&obuf, BUFSZ);
  if (cnt > 0) {
    if (HAL_UART_Transmit_IT(&gHuart, obuf.data + obuf.tail, cnt) == HAL_OK) {
      obuf.tail = INC_POINTER(obuf.tail, cnt);
    }
  }
}


/* Check if UART transmission has ceased and restart it */
void uart_tx_restart() {

  if (gHuart.gState == HAL_UART_STATE_READY) {
    /*
     * No need to lock out USART IRQ.  UART BUSY-->READY is
     * set in UART_EndTransmit_IT immediately before calling
     * HAL_UART_TxCpltCallback from the ISR.  This breaks
     * though if we write from a higher-priority ISR.
     */
    uart_tx();
  }
}


void uart_rx() {
  if (INC_POINTER(ibuf.head, 1) != ibuf.tail) {
    /* No need to lock out IRQ as long as we're not sending from
     * a higher priority ISR
     */
    HAL_UART_Receive_IT(&gHuart, ibuf.data + ibuf.head, 1);
  }
}


size_t send_nb(uint8_t* data, size_t len) {

  /* Write the first chunk of data into the buffer */
  size_t cnt = next_write_chunk(&obuf, len);

  if (cnt > 0) {
    memcpy(obuf.data + obuf.head, data, cnt);
    obuf.head = INC_POINTER(obuf.head, cnt);
    uart_tx_restart();
  }
  return cnt;
}


size_t send_debug(uint8_t* data, size_t len) {
  size_t cnt = send_nb(data, len);
  cnt += send_nb(data + cnt, len - cnt);
  return cnt;
}


void send_sync_debug(uint8_t* data, size_t len) {

  for (uint8_t* end = data + len; data != end; ) {
    data += send_nb(data, end - data);
  }
}


size_t recv_nb(uint8_t* data, size_t len) {

  size_t cnt = next_read_chunk(&ibuf, len);

  if (cnt > 0) {
    memcpy(data, ibuf.data + ibuf.tail, cnt);
    ibuf.tail = INC_POINTER(ibuf.tail, cnt);
    /* Ensure Rx process is started */
    uart_rx();
  }
  return cnt;
}


size_t recv_debug(uint8_t* data, size_t len) {
  size_t cnt = recv_nb(data, len);
  cnt += recv_nb(data + cnt, len - cnt);
  return cnt;
}


void recv_sync_debug(uint8_t* data, size_t len) {
  for (uint8_t* end = data + len; data != end; ) {
    /* This spins on the read function */
    data += recv_nb(data, end - data);
  }
}


void USART_UART_Init() {

  gHuart.Instance = DEBUG_UART_INSTANCE;
  gHuart.Init.BaudRate = 115200;
  gHuart.Init.WordLength = UART_WORDLENGTH_8B;
  gHuart.Init.StopBits = UART_STOPBITS_1;
  gHuart.Init.Parity = UART_PARITY_NONE;
  gHuart.Init.Mode = UART_MODE_TX_RX;
  gHuart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  gHuart.Init.OverSampling = UART_OVERSAMPLING_16;
  gHuart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  gHuart.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  gHuart.FifoMode = UART_FIFOMODE_DISABLE;
//  gHuart.Init.TXFIFOThreshold = UART_TXFIFO_THRESHOLD_1_8;
//  gHuart.Init.RXFIFOThreshold = UART_RXFIFO_THRESHOLD_1_8;
  gHuart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&gHuart);
  HAL_NVIC_SetPriority(DEBUG_UART_IRQ, 0, 0);
  HAL_NVIC_EnableIRQ(DEBUG_UART_IRQ);
}

int x = 1;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {

  if (huart->Instance == DEBUG_UART_INSTANCE) {
    /* We've transmitted all bytes from limit to tail.  Increment
     * the limit so that it is tail - 1.
     */
    obuf.limit = obuf.tail;
    obuf.limit = INC_POINTER(obuf.limit, BUFSZ - 1);
    uart_tx();
  }
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
  if (huart->Instance == DEBUG_UART_INSTANCE) {
    /* Increment the receive pointer and restart the receive process */
    ibuf.head = INC_POINTER(ibuf.head, 1);
    uart_rx();
  }
}


/* HAL bug: Restart the receive process on error */
void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) {
  if (huart->ErrorCode == HAL_UART_ERROR_ORE) {
    uart_rx();
  }
}

void uart_io_init() {

  ibuf.tail = 0;
  ibuf.head = 0;
  ibuf.limit = BUFSZ - 1;
  obuf.tail = 0;
  obuf.head = 0;
  obuf.limit = BUFSZ - 1;
  USART_UART_Init();
  uart_rx();
}

