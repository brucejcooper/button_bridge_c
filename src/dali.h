#ifndef _DALI_H
#define _DALI_H

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#define DALI_NAK -1
#define DALI_TIMEOUT -2
#define DALI_BUS_ERROR -3


typedef struct __attribute__((__packed__))  {
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


typedef enum  {
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
    DALI_GEAR_TYPE_NONE = 255,  // No device present.
} dali_gear_type_t;



typedef struct {
    int addr;
    dali_gear_type_t type;
    uint8_t min;
    uint8_t max;
    uint8_t level;
    uint16_t groups;
    uint8_t power_on_level;
    dali_device_bank_0_t bank0;
    bool values_changed;
} dali_dev_data_t;





extern dali_dev_data_t dali_devices[64];
extern bool dali_scan_in_progress;

inline bool dali_is_fadeable(dali_dev_data_t *dev) {
    return dev->min != dev->max;
}

void dali_init(uint32_t tx_pin, uint32_t rx_pin);
void dali_poll();
void dali_toggle(int addr);
void dali_set_on(int addr, bool is_on);
void dali_set_level(int addr, int level);
void dali_fade(int addr, int velocity);
bool dali_start_scan();
size_t dali_write_discovery_msg(dali_dev_data_t *dev, char *msg, size_t msg_len, char *topic, size_t topic_len, char *switchDeviceID);
size_t dali_write_values_msg(dali_dev_data_t *dev, char *msg, size_t msg_len, char *topic, size_t topic_len, char *switchDeviceID);

#endif
