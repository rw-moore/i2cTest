
#include "tcp_io.h"
#include "comms.h"
#include "lwip/debug.h"
#include "lwip/stats.h"
#include "lwip/tcp.h"
#include "lwip.h"
#include <stdbool.h>
#include <string.h>

/*
 *  TODO:
 *  1. Consider possibilities of not calling netif->input in the ISR.
 *     This should work well enough for debugging/testing, but it is not
 *     particularly nice.
 *  2. Is refusal to accept an incoming connection the behavior we want?
 */

#ifdef ETHCOMMS

#define WCHUNKSIZE 128
#define SERVPORT 5012

/* Global data */
static struct tcp_pcb *tcp_server_pcb = NULL;
static struct tcp_pcb *conn_pcb_ = NULL;
static struct pbuf* rbuf_ = NULL;
static size_t rOffset_ = 0;


void HAL_ETH_RxCpltCallback(ETH_HandleTypeDef *heth) {

  /* Process the frame(s) in IRQ context.  This is a kludge,
   * but it should generally be OK.  The interrupt is at the
   * lowest priority, so Ethernet packets will essentially
   * be processed in the background without blocking out
   * critical tasks.  This has been tested to be true.  Since
   * mainloop context doesn't work in our use case, the only
   * other solution is an OS of some sort.
   */
  MX_LWIP_Process();
}


/*
 *  Send data.  Block until send is complete if no connection
 *  or TCP buffers are full, and never return an error.  Not
 *  thread-safe; do not call from ISR.
 */
size_t send(uint8_t* data, size_t len) {

  uint8_t* origin = data;
  bool ok = true;
  for (uint8_t* end = data + len; ok && (data != end); ) {
    size_t chunkSize = WCHUNKSIZE;
    if ( (end - data) < chunkSize) {
      chunkSize = (end - data);
    }
    HAL_NVIC_DisableIRQ(ETH_IRQn);
    /* LwIP critical section */
    if (conn_pcb_ != NULL) {
      err_t werr = tcp_write(conn_pcb_, data, chunkSize, 0x01);
      if (werr == ERR_OK) {
        data += chunkSize;
        if (data == end && len > 1) {
          /* Message complete.  Send it immediately.
           * Avoid sending one-byte frames if possible.
           */
          tcp_output(conn_pcb_);
        }
      } else if (werr == ERR_MEM) {
        ok = false;
      }
    } else {
      ok = false;
    }
    /* End LwIP critical section */
    HAL_NVIC_EnableIRQ(ETH_IRQn);
  }
  return data - origin;
}


void send_sync(uint8_t* data, size_t len) {

  uint8_t* end = data + len;
  for (uint8_t* wptr = data; wptr != end; ) {
    wptr += send(wptr, (end - wptr) );
  }
}


/*
 *  Receive data in non-blocking mode.  Return the number of
 *  bytes received.  Read no more than len.  Not thread-safe;
 *  do not call from ISR.
 */
size_t recv(uint8_t* data, size_t len) {

  uint8_t* wptr = data;
  uint8_t* end = data + len;
  while (wptr != end) {

    if (rbuf_ == NULL) {
      /* No more data */
      break;
    }

    if (rOffset_ < rbuf_->len) {
      /* We have a buffer that we can read */
      size_t readlen = end - wptr;
      if (readlen > (rbuf_->len - rOffset_)) {
        readlen = rbuf_->len - rOffset_;
      }
      memcpy(wptr, rbuf_->payload + rOffset_, readlen);
      wptr += readlen;
      rOffset_ += readlen;
    }

    /* Handle case where we're at the end of current buffer */
    if (rbuf_->len == rOffset_) {
      HAL_NVIC_DisableIRQ(ETH_IRQn);
      /* LwIP critical section */
      struct pbuf* currentBuf = rbuf_;
      rbuf_ = rbuf_->next;
      if (rbuf_ != NULL) {
        pbuf_ref(rbuf_);
      }
      currentBuf->next = NULL;
      for (unsigned freed = 0; !freed; ) {
        /* Kill the old buffer */
    	freed = pbuf_free(currentBuf);
      }
      /* Record reception of this data.  Note: we could do this
       * when we actually copy bytes into the local buffer to be
       * more accurate, but I doubt it really matters. */
      tcp_recved(conn_pcb_, rOffset_);
      rOffset_ = 0;
      /* End LwIP critical section */
      HAL_NVIC_EnableIRQ(ETH_IRQn);
    }
  }
  return wptr - data;
}


void tcp_io_close() {

  /* remove all callbacks */
  tcp_recv(conn_pcb_, NULL);
  tcp_err(conn_pcb_, NULL);
  tcp_poll(conn_pcb_, NULL, 0);

  /* close tcp connection */
  tcp_close(conn_pcb_);
  conn_pcb_ = NULL;
}


/**
  * tcp_recv LwIP callback.  Handle incoming data.
  */
err_t tcp_io_recv(void *arg,
                  struct tcp_pcb *tpcb,
                  struct pbuf *p,
                  err_t err) {

  LWIP_UNUSED_ARG(arg);
  LWIP_UNUSED_ARG(tpcb);

  /* if we receive an empty tcp frame from client => close connection */
  if (p == NULL) {
    /* remote host closed connection */
    tcp_io_close();
    return ERR_OK;
  }

  /* A non empty frame was received from client, but we have an error */
  if (err != ERR_OK) {

    /* free received pbuf */
    if (p != NULL) {
      pbuf_free(p);
    }
    return err;
  }

  if (p->len == 0) {
	 return ERR_OK;
  }

  /* We need to copy the data out of the
   * DMA buffer, otherwise it'll disappear.
   */
  struct pbuf *rp = pbuf_alloc(PBUF_RAW, p->tot_len, PBUF_POOL);
  pbuf_copy(rp, p);
  pbuf_free(p);

  if (rbuf_ == NULL) {
	rbuf_ = rp;
  } else {
	pbuf_chain(rbuf_, rp);
  }

  return ERR_OK;
}


void tcp_io_error(void *arg,
                  err_t err) {

  LWIP_UNUSED_ARG(arg);
  LWIP_UNUSED_ARG(err);
}


/**
  * Accept a new connection
  */
err_t tcp_io_accept(void *arg,
                    struct tcp_pcb *newpcb,
                    err_t err) {

  LWIP_UNUSED_ARG(arg);
  LWIP_UNUSED_ARG(err);

  if (conn_pcb_ != NULL) {
    /* Already connected */
    tcp_close(newpcb);
    return ERR_OK;
  }

  conn_pcb_ = newpcb;

  /* set priority for the newly accepted tcp connection newpcb */
  tcp_setprio(conn_pcb_, TCP_PRIO_MIN);
  tcp_recv(conn_pcb_, tcp_io_recv);
  tcp_err(conn_pcb_, tcp_io_error);
  return ERR_OK;
}


/**
  * Start the server listening.
  */
void tcp_io_init() {

  /* Initialize LwIP */
  MX_LWIP_Init();

  /* Enable ETH interrupt requests */
  HAL_NVIC_EnableIRQ(ETH_IRQn);

  /* create new tcp pcb */
  tcp_server_pcb = tcp_new();

  if (tcp_server_pcb != NULL) {
    err_t err;

    /* bind pcb to telnet port */
    err = tcp_bind(tcp_server_pcb, IP_ADDR_ANY, SERVPORT);

    if (err == ERR_OK) {
      /* start tcp listening for global_pcb */
      tcp_server_pcb = tcp_listen(tcp_server_pcb);

      /* initialize LwIP tcp_accept callback function */
      tcp_accept(tcp_server_pcb, tcp_io_accept);
    }
    else {
      /* deallocate the pcb */
      memp_free(MEMP_TCP_PCB, tcp_server_pcb);
    }
  }
}


#endif // ETHCOMMS
