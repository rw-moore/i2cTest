
#include "comms.h"
#include "tcp_io.h"
#include "uart_io.h"
#include <stdarg.h>
#include <string.h>

/*
 *  1. We probably don't want to define _write and _read here if
 *     we have other files
 */


/*
 *  Build blocking receive from non-blocking.  Just spin.
 */
void recv_sync(uint8_t* data, size_t len) {
  uint8_t* end = data + len;
  for (uint8_t* rptr = data; rptr != end; ) {
    /* This spins on the read function */
    rptr += recv(rptr, end - rptr);
  }
}


/*
 *  Implement _read and _write for std io streams
 */
int _write(int file, char *data, int len) {

  if (file == 2) {
    send_sync_debug((uint8_t*)data, len);
  } else if (file == 1) {
    send_sync((uint8_t*)data, len);
  }
  return len;
}


int _read(int file, char *data, int len) {
  if (file == 0) {
    recv_sync((uint8_t*)data, len);
  }
  return len;
}


void comms_init() {

  uart_io_init();
#ifdef ETHCOMMS
  tcp_io_init();
#endif // ETHCOMMS
}
