#ifndef _BUTTONS_H
#define _BUTTONS_H

#include <pico/stdlib.h>

// Bus type stored in the top two bits of the address byte
typedef enum
{
    BUS_TYPE_MODBUS = 0,
    BUS_TYPE_DALI = 1,
    BUS_TYPE_NONE = 3, // We use this because flash is initialised to 0xFF when empty
} bus_type_t;

#define NUM_FIXTURES 24
#define NUM_BUTTONS_PER_FIXTURE 7

extern const char *fixture_entity_prefix;
extern const char *fixture_binding_postfix;

void buttons_init();
void buttons_enumerate();
void buttons_poll();
void set_and_persist_binding(uint fixture, uint button, uint8_t encoded_binding);

#endif