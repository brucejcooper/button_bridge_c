#ifndef _MODBUS_H
#define _MODBUS_H

extern const char *modbus_entity_prefix;

void modbus_init(int tx_pin, int rx_pin, int cs_pin);

void modbus_enumerate();
void modbus_set_coil(uint8_t devaddr, int coil_num, int val);
void modbus_poll();

#endif
