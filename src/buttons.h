#ifndef _BUTTONS_H
#define _BUTTONS_H

#include <pico/stdlib.h>

// Bus type stored in the top two bits of the address byte
typedef enum {
    BINDING_TYPE_MODBUS = 0,
    BINDING_TYPE_DALI = 1,
    BINDING_TYPE_NONE = 3,
} binding_type_t;

typedef struct {
    binding_type_t type;
    unsigned int address;
} binding_t;

typedef struct {
    int addr;
    bool released;
    uint num_repeats;
    int velocity;
    uint countdown;
} button_ctx_t;

#define NUM_FIXTURES 24
#define NUM_BUTTONS_PER_FIXTURE 7

extern button_ctx_t button_ctx[NUM_FIXTURES * NUM_BUTTONS_PER_FIXTURE];

// Mask for the bottom 6 bits, which should be the address on the bus.
#define BINDING_ADDRESS_MASK 0x3F

void buttons_init();
void buttons_enumerate();
void buttons_poll();
void set_and_persist_binding(unsigned int addr, uint16_t encoded_binding);
void init_binding_reg_from_flash(uint index, binding_t *binding);
bool is_button_pressed(int fixture, int button);

#endif