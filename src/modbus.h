#ifndef _MODBUS_H
#define _MODBUS_H

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

void modbus_init(int tx_pin, int rx_pin, int cs_pin);

void modbus_enumerate();
void modbus_set_coil(uint8_t devaddr, int coil_num, int val);
bool modbus_get_coil(uint8_t devaddr, int coil_num);
void modbus_poll();
size_t modbus_write_discovery_message(int device_id, int coil_id, char *msg, size_t msg_len, char *topic, size_t topic_len, char *switchDeviceID);

#endif
