#include "regs.h"

#include <pico/mutex.h>
#include <string.h>
#include "modbus.h"

extern uint8_t coils[MAX_COILS / 8];
extern uint8_t discrete_input[MAX_COILS / 8];
extern uint8_t holding_registers[MAX_HOLDING_REGISTERS * 2];
auto_init_mutex(register_access_lock);

#define LIGHT_REG_BANK_SZ (MAX_DALI_LIGHTS * 2)
#define SWITCH_BINDINGS_SZ (MAX_DISCRETE_INPUTS * 2)

#define switch_bindings_ptr (holding_registers)

#define light_status_ptr (switch_bindings_ptr + SWITCH_BINDINGS_SZ)
#define light_level_ptr (light_status_ptr + 1)

#define light_max_ptr (light_status_ptr + LIGHT_REG_BANK_SZ)
#define light_min_ptr (light_max_ptr + 1)

#define light_ext_fade_ptr (light_max_ptr + LIGHT_REG_BANK_SZ)
#define light_fade_ptr (light_ext_fade_ptr + 1)

#define light_system_failure_lvl_ptr (light_ext_fade_ptr + LIGHT_REG_BANK_SZ)
#define light_power_on_lvl_ptr (light_system_failure_lvl_ptr + 1)

#define light_groups_ptr (light_system_failure_lvl_ptr + LIGHT_REG_BANK_SZ)

static inline void lock_regs() {
    mutex_enter_blocking(&register_access_lock);
}
static inline void unlock_regs() { mutex_exit(&register_access_lock); }

uint8_t coils[MAX_COILS / 8];
uint8_t discrete_input[MAX_COILS / 8];
uint8_t holding_registers[MAX_HOLDING_REGISTERS * 2];

// -- Discrete Inputs

void set_discrete_input(int addr) {
    if (addr >= MAX_DISCRETE_INPUTS) {
        return;
    }
    uint8_t *reg_ptr = discrete_input + addr / 8;
    lock_regs();
    *reg_ptr |= 1 << (addr % 8);
    unlock_regs();
}

void clear_discrete_input(int addr) {
    if (addr >= MAX_DISCRETE_INPUTS) {
        return;
    }
    uint8_t *reg_ptr = discrete_input + addr / 8;
    lock_regs();
    *reg_ptr &= ~(1 << (addr % 8));
    unlock_regs();
}

void copy_discrete_inputs(uint8_t *out, unsigned addr, size_t num) {
    if (addr + num >= MAX_DISCRETE_INPUTS) {
        return;
    }
    lock_regs();
    memcpy(out, discrete_input + (addr / 8), num / 8);
    unlock_regs();
}

// -- Coils

void set_coil_reg(int addr) {
    if (addr >= MAX_COILS) {
        return;
    }

    uint8_t *reg_ptr = coils + (addr / 8);
    uint8_t val = 1 << (addr % 8);
    lock_regs();
    *reg_ptr |= val;
    unlock_regs();
}

void clear_coil_reg(int addr) {
    if (addr >= MAX_COILS) {
        return;
    }

    uint8_t *reg_ptr = coils + (addr / 8);
    uint8_t val = ~(1 << (addr % 8));
    lock_regs();
    *reg_ptr &= val;
    unlock_regs();
}

void toggle_coil_reg(int addr) {
    if (addr >= MAX_COILS) {
        return;
    }

    uint8_t *reg_ptr = coils + (addr / 8);
    uint8_t val = 1 << (addr % 8);
    lock_regs();
    *reg_ptr ^= val;
    unlock_regs();
}

bool is_coil_set(unsigned coil) {
    if (coil >= MAX_COILS) {
        return false;
    }

    lock_regs();
    bool val = coils[coil / 8] & (1 << (coil % 8));
    unlock_regs();
    return val;
}

void copy_coil_values(uint8_t *out, unsigned addr, size_t num) {
    if (addr + num >= MAX_COILS) {
        return;
    }
    lock_regs();
    memcpy(out, coils + addr / 8, num / 8);
    unlock_regs();
}

// -- Holding registers

void copy_holding_regs(uint8_t *out, unsigned addr, size_t num) {
    if (addr + num >= MAX_HOLDING_REGISTERS) {
        return;
    }
    lock_regs();
    memcpy(out, holding_registers + addr * 2, num * 2);
    unlock_regs();
}

void set_holding_reg(unsigned addr, unsigned value) {
    if (addr >= MAX_HOLDING_REGISTERS) {
        return;
    }
    uint8_t *reg_ptr = holding_registers + addr * 2;
    lock_regs();
    *reg_ptr++ = value >> 8;
    *reg_ptr++ = value;
    unlock_regs();
}

void set_holding_reg_byte(unsigned addr, unsigned byte, unsigned value) {
    if (addr >= MAX_HOLDING_REGISTERS) {
        return;
    }
    uint8_t *ptr = holding_registers + addr * 2 + 1 - byte;
    lock_regs();
    *ptr = value;
    unlock_regs();
}

void set_holding_reg_nibble(unsigned addr, unsigned nibble_no, unsigned value) {
    if (addr >= MAX_HOLDING_REGISTERS) {
        return;
    }
    assert(nibble_no < 4);
    uint8_t *ptr = holding_registers + addr * 2 + 1 - (nibble_no / 2);
    unsigned shift = (nibble_no % 2) * 4;
    unsigned mask = ~(7 << shift);
    unsigned shiftedVal = (value & 7) << shift;

    lock_regs();
    *ptr = (*ptr & ~mask) | shiftedVal;
    unlock_regs();
}

int get_holding_reg(unsigned addr) {
    if (addr >= MAX_HOLDING_REGISTERS) {
        return -1;
    }
    uint8_t *reg_ptr = holding_registers + addr * 2;
    lock_regs();
    int val = (*reg_ptr << 8) | *(reg_ptr + 1);
    unlock_regs();
    return val;
}


void set_holding_reg_bit(int addr, int bit) {
    if (addr >= MAX_HOLDING_REGISTERS) {
        return;
    }
    uint8_t *reg_ptr = coils + addr * 2;
    if (bit < 8) {
        reg_ptr++;
    }
    lock_regs();
    *reg_ptr |= 1 << (addr % 8);
    unlock_regs();
}

void clear_holding_reg_bit(int addr, int bit) {
    if (addr >= MAX_HOLDING_REGISTERS) {
        return;
    }
    uint8_t *reg_ptr = coils + addr * 2;
    if (bit < 8) {
        reg_ptr++;
    }
    lock_regs();
    *reg_ptr &= ~(1 << (bit % 8));
    unlock_regs();
}

void toggle_holding_reg_bit(int addr, int bit) {
    if (addr >= MAX_HOLDING_REGISTERS) {
        return;
    }
    uint8_t *reg_ptr = coils + addr * 2;
    if (bit < 8) {
        reg_ptr++;
    }
    lock_regs();
    *reg_ptr ^= 1 << (bit % 8);
    unlock_regs();
}
