#ifndef _MODBUS_H
#define _MODBUS_H

#include "async.h"

void modbus_init(int tx_pin, int rx_pin, int cs_pin);

void modbus_enumerate_task(async_ctx_t *ctx);
void modbus_set_coil(uint8_t devaddr, int coil_num, int val);

#endif
