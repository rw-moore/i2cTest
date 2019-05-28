
#ifndef __COMMS_H__
#define __COMMS_H__

#include <stdint.h>
#include <stdio.h>

/*
 *  TODO:
 *  1. ETHCOMMS and DEBUG should be supplied on the
 *     command line when compiling for debug.
 */

#define NUCLEOH7
#ifdef NUCLEOH7
// Nucleo board implies ETHCOMMS
#define ETHCOMMS
#endif // NUCLEOH7


void comms_init();

/* Communications on the main interface */
size_t send(uint8_t* data, size_t len);
void send_sync(uint8_t* data, size_t len);
size_t recv(uint8_t* data, size_t len);
void recv_sync(uint8_t* data, size_t len);

/* Communications on the debug interface */
size_t send_debug(uint8_t* data, size_t len);
void send_sync_debug(uint8_t* data, size_t len);
size_t recv_debug(uint8_t* data, size_t len);
void recv_sync_debug(uint8_t* data, size_t len);
#endif /* __COMMS */
