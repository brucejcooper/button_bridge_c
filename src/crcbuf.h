#ifndef _CRCBUF_H
#define _CRCBUF_H

#include <stdint.h>
#include <stdbool.h>

void crc_update(uint8_t val, uint16_t *crc);
void crc_append(uint8_t *bufptr, uint8_t val,  uint16_t *crc);

#endif
