#ifndef _DALI_H
#define _DALI_H

#include <stdbool.h>
#include <stdint.h>

#define DALI_NAK -1
#define DALI_TIMEOUT -2
#define DALI_BUS_ERROR -3

typedef struct __attribute__((__packed__)) {
    // We don't load the first two bytes of the memory bank, as the device doesn't respond for the second reserved one.
    // uint8_t last_offset;
    // uint8_t _res;
    uint8_t last_mem_bank;
    uint8_t gtin[6];
    uint16_t fimware_version;
    uint8_t id[8];
    uint16_t hw_version;
    uint8_t dali_version;
    uint8_t num_gear;
    uint8_t num_devices;
    uint8_t num_logical_gear;
    uint8_t num_logical_devices;
    uint8_t this_device_index;
} dali_device_bank_0_t;

typedef enum {
    DALI_GEAR_TYPE_FLOURESCENT_LAMP = 0,
    DALI_GEAR_TYPE_EMERGENCY_LIGHTING = 1,
    DALI_GEAR_TYPE_HID_LAMP = 2,
    DALI_GEAR_TYPE_LOW_VOLTAGE_HALOGEN_LAMP = 3,
    DALI_GEAR_TYPE_INCANDESCENT_LAMP_DIMMER = 4,
    DALI_GEAR_TYPE_DC_CONTROLLED_DIMMER = 5,
    DALI_GEAR_TYPE_LED_LAMP = 6,
    DALI_GEAR_TYPE_RELAY = 7,
    DALI_GEAR_TYPE_COLOUR = 8,

    DALI_GEAR_TYPE_GEAR_GROUP = 128,  // This isn't a DALI GearType.  Its a magic value we put in for Groups.
    DALI_GEAR_TYPE_NONE = 255,        // No device present.
} dali_gear_type_t;

typedef void (*dali_result_cb_t)(int result);


// extern dali_dev_data_t dali_devices[64];
extern bool dali_scan_in_progress;

bool dali_is_fadeable(int addr);

void dali_exec_cmd(uint16_t cmd, dali_result_cb_t resultHandler, bool sendTwice);

void dali_init(uint32_t tx_pin, uint32_t rx_pin);
void dali_poll();
void dali_toggle(int addr, dali_result_cb_t cb);
void dali_set_on(int addr, bool is_on, dali_result_cb_t cb);
void dali_set_level(int addr, int level, dali_result_cb_t cb);
void dali_set_min_max_level(int addr, unsigned min, unsigned max, dali_result_cb_t cb);
void dali_set_fade_time_rate(int addr, unsigned time, unsigned rate, dali_result_cb_t cb);
void dali_set_power_on_level(int addr, int powerOnLevel, int systemFailLevel, dali_result_cb_t cb);
void dali_remove_from_group(int addr, int group, dali_result_cb_t cb);
void dali_add_to_group(int addr, int group, dali_result_cb_t cb);

void dali_fade(int addr, int velocity, dali_result_cb_t cb);
bool dali_enumerate();

#endif
