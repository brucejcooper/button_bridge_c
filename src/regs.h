#ifndef REGS_H
#define REGS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define MAX_COILS 256
#define MAX_DISCRETE_INPUTS 256
#define MAX_DALI_LIGHTS 64
#define NUM_VALUES_PER_LIGHT 16

#define MAX_HOLDING_REGISTERS (MAX_DALI_LIGHTS * NUM_VALUES_PER_LIGHT)

#define BINDINGS_HR_BASE 0
#define DALI_HR_BASE (BINDINGS_HR_BASE + MAX_DISCRETE_INPUTS)

#define DALI_HR_BANK(bank_no) (DALI_HR_BASE + (bank_no)*MAX_DALI_LIGHTS)
typedef enum { 
    DALI_HR_BANKID_STATUS = 0,
    DALI_HR_BANKID_MINMAX,
    DALI_HR_BANKID_FADE,
    DALI_HR_BANKID_POWERON,
    DALI_HR_BANKID_GROUPS,
} dali_hr_bankid_t;

#define DALI_HR_BANK_ID_FROM_REGID(addr) (((addr) - DALI_HR_BASE) / MAX_DALI_LIGHTS)
#define DALI_ADDR_FROM_REGID(addr) (((addr) - DALI_HR_BASE) % MAX_DALI_LIGHTS)

#define DALI_STATUS_HR_BASE DALI_HR_BANK(DALI_HR_BANKID_STATUS)
#define DALI_MINMAX_HR_BASE DALI_HR_BANK(DALI_HR_BANKID_MINMAX)
#define DALI_FADE_HR_BASE DALI_HR_BANK(DALI_HR_BANKID_FADE)
#define DALI_POWERON_HR_BASE DALI_HR_BANK(DALI_HR_BANKID_POWERON)
#define DALI_GROUPS_HR_BASE DALI_HR_BANK(DALI_HR_BANKID_GROUPS)

// Discrete Inputs
void set_discrete_input(int addr);
void clear_discrete_input(int addr);
void copy_discrete_inputs(uint8_t *out, unsigned addr, size_t num);

// Coils
void set_coil_reg(int addr);
void clear_coil_reg(int addr);
void toggle_coil_reg(int addr);
bool is_coil_set(unsigned coil);
void copy_coil_values(uint8_t *out, unsigned addr, size_t num);

// Holding Regs
void copy_holding_regs(uint8_t *out, unsigned addr, size_t num);
void set_holding_reg(unsigned addr, unsigned value);
int get_holding_reg(unsigned addr);

void set_holding_reg_byte(unsigned addr, unsigned byte, unsigned value);
void set_holding_reg_nibble(unsigned addr, unsigned nibble_no, unsigned value);
void set_holding_reg_bit(int addr, int bit) ;
void clear_holding_reg_bit(int addr, int bit);
void toggle_holding_reg_bit(int addr, int bit);



#endif