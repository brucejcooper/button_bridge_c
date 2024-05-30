#ifndef _DALI_H
#define _DALI_H

#include <stdint.h>
#include <stdbool.h>

#define DALI_NAK -1
#define DALI_TIMEOUT -2
#define DALI_BUS_ERROR -3

extern const char *dali_entity_prefix;

void dali_init(uint32_t tx_pin, uint32_t rx_pin);
void dali_poll();
void dali_toggle(int addr);
void dali_set_on(int addr, bool is_on);
void dali_set_level(int addr, int level);
void dali_fade(int addr, int velocity);
void dali_enumerate();

#endif
