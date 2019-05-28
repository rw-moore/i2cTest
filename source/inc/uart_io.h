
#ifndef __DEBUG_IO_H__
#define __DEBUG_IO_H__

#include <stdint.h>
#include <stdio.h>

void uart_io_init();
size_t send_debug(uint8_t* data, size_t len);
void send_sync_debug(uint8_t* data, size_t len);
size_t recv_debug(uint8_t* data, size_t len);
void recv_sync_debug(uint8_t* data, size_t len);
#endif /* __DEBUG_IO */
