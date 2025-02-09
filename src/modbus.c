#include "modbus.h"
#include "log.h"
#include "modbus.pio.h"
#include "network.h"
#include "stdbool.h"
#include "stdint.h"
#include <hardware/clocks.h>
#include <hardware/irq.h>
#include <hardware/structs/timer.h>
#include <hardware/timer.h>
#include <pico/platform.h>
#include <pico/stdlib.h>
#include <pico/util/queue.h>
#include <cJSON.h>
#include <string.h>
#include <stdio.h>

static uint32_t coil_values;
static uint8_t buf[9];
static uint8_t *bufptr;

static uint8_t xor = 0;
static uint16_t crc = 0xFFFF;

typedef enum {
  OP_CLEAR,
  OP_SET,
  OP_TOGGLE,
  OP_FETCHALL,
  OP_NONE = 0xFF,
} op_t;

typedef struct {
  int address;
  int coil;
  op_t op;
} modbus_cmd_t;

#define MODBUS_BAUD_RATE 9600
#define QUEUE_DEPTH 10

static queue_t cmd_queue;
static modbus_cmd_t in_flight = {
    .address = 0,
    .coil = 0,
    .op = OP_NONE,
};
absolute_time_t timeout;

#define BYTE_TRANSMISSION_DELAY_US(n_bits) (n_bits * 10 * 1000000 / 9600)

static const PIO pio = pio1;
static const unsigned int tx_sm = 0;
static const unsigned int rx_sm = 1;

static const char *TAG = "MODBUS";

static inline void reset_buf() {
  bufptr = buf;
  xor = 0;
  crc = 0xFFFF;
}

static void append(uint8_t val) {
  static const uint16_t table[256] = {0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241, 0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440, 0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40, 0x0A00, 0xCAC1,
                                      0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841, 0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40, 0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41, 0x1400, 0xD4C1, 0xD581, 0x1540,
                                      0xD701, 0x17C0, 0x1680, 0xD641, 0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040, 0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240, 0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0,
                                      0x3480, 0xF441, 0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41, 0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840, 0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
                                      0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40, 0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640, 0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041, 0xA001, 0x60C0,
                                      0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240, 0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441, 0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41, 0xAA01, 0x6AC0, 0x6B80, 0xAB41,
                                      0x6900, 0xA9C1, 0xA881, 0x6840, 0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41, 0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40, 0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1,
                                      0xB681, 0x7640, 0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041, 0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241, 0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
                                      0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40, 0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841, 0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40, 0x4E00, 0x8EC1,
                                      0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41, 0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641, 0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040};

  *bufptr++ = val;
  xor = val ^ crc;
  crc >>= 8;
  crc ^= table[xor];
}

/**
 * When transmitting, we append the CRC that we caluclated each time we called append()
 */
static inline void append_crc() {
  *bufptr++ = crc & 0xFF;
  *bufptr++ = crc >> 8;
}

/**
 * When receiving, the CRC calculation will end up 0 once a valid set of CRC values are appended.
 */
static inline bool crc_is_valid() { return crc == 0; }

/**
 * Checks to see if we have read a certain number of bytes from the UART
 */
static bool bytes_available(size_t expected_sz) {
  while (bufptr - buf < expected_sz) {
    int c;
    if ((c = modbus_rx_program_getc(pio, rx_sm)) < 0) {
      return false;
    }
    append(c);
  }
  return true;
}

bool modbus_get_coil(uint8_t devaddr, int coil_num) { return (coil_values >> coil_num) & 0x01; }

static void send_set_coil(uint8_t devaddr, int coil_num, op_t val) {
  reset_buf();
  append(devaddr);
  append(0x05);
  append(00);
  append(coil_num);
  append(val == OP_TOGGLE ? 0x55 : val == OP_SET ? 0xFF : 0x00);
  append(0x00);
  append_crc();

  modbus_tx_program_putbuf(pio, tx_sm, buf, 8);

  switch (val) {
  case OP_CLEAR:
    coil_values &= ~(1 << coil_num);
    break;
  case OP_SET:
    coil_values |= 1 << coil_num;
    break;
  case OP_TOGGLE:
    // TODO go stateless - its more reliable, plus it'll save us 4 bytes :P
    coil_values ^= 1 << coil_num;
    break;
  default:
    defer_log(TAG, "invalid opcode");
    return;
  }

  enqueue_device_update(EVT_MODBUS_COIL_STATE_CHANGED, (void *)coil_num);
}

static void request_coil_status(int devaddr) {
  reset_buf();
  append(devaddr);
  append(0x01);
  append(0x00);
  append(0x00);
  append(0x00);
  append(0x20);
  append_crc();
  // Technically this can block, but it probably won't (the FIFO is 8 bytes long)
  modbus_tx_program_putbuf(pio, tx_sm, buf, 8);
}

void modbus_enumerate() {
  modbus_cmd_t cmd = {.address = 1, .coil = 0, .op = OP_FETCHALL};
  queue_try_add(&cmd_queue, &cmd);
}

void modbus_set_coil(uint8_t devaddr, int coil_num, int val) {
  modbus_cmd_t cmd = {
      .address = devaddr,
      .coil = coil_num,
      .op = val,
  };
  queue_try_add(&cmd_queue, &cmd);
}

void modbus_poll() {

  if (in_flight.op != OP_NONE) {
    // log_int("Waiting for bytes. Have", bufptr - buf);
    if (bytes_available(in_flight.op == OP_FETCHALL ? 9 : 8)) {
      if (crc_is_valid()) {
        switch (in_flight.op) {
        case OP_FETCHALL:

          coil_values = (buf[3] << 24) | (buf[4] << 16) | (buf[5] << 8) | buf[6];
          // log_i("coils 0x%08x", coil_values);
          for (int coil = 0; coil < 32; coil++) {
            enqueue_device_update(EVT_MODBUS_DEVICE_DISCOVERED, (void *)coil);
          }
          for (int coil = 0; coil < 32; coil++) {
            enqueue_device_update(EVT_MODBUS_COIL_STATE_CHANGED, (void *)coil);
          }

          break;
        default:
          defer_log(TAG, "Invalid opcode %d while polling response", in_flight.op);
          break;
        }
      } else {
        defer_log(TAG, "CRC Mismatch");
      }
      in_flight.op = OP_NONE;
    } else if (time_reached(timeout)) {
      defer_log(TAG, "Timeout waiting for Modbus response");
      in_flight.op = OP_NONE;
    }
  }

  if (queue_try_remove(&cmd_queue, &in_flight)) {
    pio_sm_restart(pio, tx_sm);
    timeout = make_timeout_time_ms(1000); // Give the TX 1000 ms.
    switch (in_flight.op) {
    case OP_SET:
    case OP_CLEAR:
    case OP_TOGGLE:
      send_set_coil(in_flight.address, in_flight.coil, in_flight.op);
      break;
    case OP_FETCHALL:
      request_coil_status(1);
      break;
    default:
      defer_log(TAG, "Invalid opcode %d while dequeueing");
      break;
    }
    // We want the receive fifo to be completely empty - it should be, but its always good to be sure
    pio_sm_restart(pio, rx_sm);

    // We expect to see a response in a handful of ms, but give it a bit longer, just in case.
    timeout = make_timeout_time_ms(100);
    // Reset read buffer for read.
    reset_buf();
  }
}

void modbus_init(int tx_pin, int rx_pin, int de_pin) {

  queue_init(&cmd_queue, sizeof(modbus_cmd_t), QUEUE_DEPTH);

  uint offset = pio_add_program(pio, &modbus_tx_program);
  modbus_tx_program_init(pio, tx_sm, offset, tx_pin, de_pin, MODBUS_BAUD_RATE);
  offset = pio_add_program(pio, &modbus_rx_program);
  modbus_rx_program_init(pio, rx_sm, offset, rx_pin, MODBUS_BAUD_RATE);

  coil_values = 0;

  // The RS485 chip seems to need a little bit of start up time before it can receive commands after reboot.
  // We put a sleep in here as a cheap way of ensuring that.
  sleep_ms(1);
}



size_t modbus_write_discovery_message(int device_id, int coil_id, char *msg, size_t msg_len, char *topic, size_t topic_len, char *switchDeviceID) {
  char *p = msg;

  char deviceName[50];
  char name[50];
  char identifier[50];
  char stateTopic[100];
  char commandTopic[100];
  char availabilityTopic[100];
  char uniqueid[50];
  size_t sz = 0;


  sprintf(deviceName, "Modbus Device %02d", device_id);
  sprintf(name, "Coil %02d", coil_id);
  sprintf(identifier, "%s/modbus%d", switchDeviceID, device_id);
  sprintf(stateTopic, "switchy/%s/modbus/%d/%d", switchDeviceID, device_id, coil_id);
  sprintf(commandTopic, "switchy/%s/modbus/%d/%d/set", switchDeviceID, device_id, coil_id);
  sprintf(availabilityTopic, "switchy/%s/available", switchDeviceID);
  sprintf(uniqueid, "%s_modbus_coil%02d%02d", switchDeviceID, device_id, coil_id);

  cJSON *jmsg = cJSON_CreateObject();
  cJSON *device = cJSON_CreateObject();

  cJSON_AddStringToObject(device, "name", deviceName);
  cJSON *idArray = cJSON_AddArrayToObject(device, "identifiers");
  cJSON_AddItemToArray(idArray, cJSON_CreateString(identifier));
  cJSON_AddStringToObject(device, "via_device", switchDeviceID);

  cJSON_AddStringToObject(jmsg, "unique_id", uniqueid);
  cJSON_AddStringToObject(jmsg, "name", name);
  cJSON_AddStringToObject(jmsg, "state_topic", stateTopic);
  cJSON_AddStringToObject(jmsg, "comand_topic", commandTopic);
  cJSON_AddStringToObject(jmsg, "availabilty_topic", name);
  cJSON_AddBoolToObject(jmsg, "optimistic", false);
  cJSON_AddNumberToObject(jmsg, "qos", 1);

  cJSON_PrintPreallocated(jmsg,msg, msg_len, false);
  sz = strlen(msg);

  sprintf(topic, "homeassistant/switch/%s/modbus%02d%02d/config", switchDeviceID, device_id, coil_id);
  return sz;
}
