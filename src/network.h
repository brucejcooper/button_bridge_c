#ifndef __NETWORK_H__
#define __NETWORK_H__

#include <stdint.h>

typedef enum {
  EVT_BTN_PRESSED,                // Button was pressed (Time sensitive)
  EVT_BTN_HELD,                 // Button was held down (Time sensitive)
  EVT_BTN_RELEASED,              // Button was released (Time sensitive)
  EVT_MODBUS_DEVICE_DISCOVERED,
  EVT_MODBUS_COIL_STATE_CHANGED,

  EVT_DALI_DEVICE_SCAN_COMPLETED,
  EVT_DALI_LEVEL_CHANGED,

  EVT_BUTTON_BINDING_CHANGED,
} device_event_type_t;

void enqueue_device_update(device_event_type_t dev, void *data);
void network_init();
void network_thread();

#endif