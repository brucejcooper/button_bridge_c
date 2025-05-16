#include "modbus_receiver.h"
#include "buttons.h"
#include "crcbuf.h"
#include "dali.h"
#include "modbus.h"
#include <hardware/flash.h>
#include <hardware/irq.h>
#include <hardware/sync.h>
#include <hardware/timer.h>
#include <hardware/watchdog.h>
#include <pico/multicore.h>
#include <pico/sem.h>
#include <pico/stdio.h>
#include <pico/stdlib.h>
#include <pico/sync.h>
#include <pico/time.h>
#include <pico/types.h>
#include <pico/util/queue.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// queue used as a simple sempaphore.
static semaphore_t downstream_response_ready;
static uint16_t read_crc = 0xFFFF;

#define MODBUS_SERVER_READ_MAX_PACKET_SZ 256
static absolute_time_t last_read = 0;
static uint8_t cmd_bytes[MODBUS_SERVER_READ_MAX_PACKET_SZ];
static uint8_t res_bytes[MODBUS_SERVER_READ_MAX_PACKET_SZ];
static uint8_t *response;

#define DEVICE_ADDR_FIXTURES 128
#define DEVICE_ADDR_DALI 129

void set_response_to_error(uint8_t device, modbus_cmd_t cmd, modbus_err_t err) {
  // reset the buffer, if it had anything written to it.  This will overwrite
  // what was there.
  response[1] |= 0x80; // Set the error MSB on the cmd.
  response[2] = err;
  response = res_bytes + 3;
}

void modbus_write_registers(uint8_t device, uint8_t cmd, uint16_t addr,
                            uint16_t data[], uint16_t num_regs) {

  // TODO set the registers
  *response++ = addr >> 8;
  *response++ = addr & 0xFF;
  *response++ = num_regs >> 8;
  *response++ = num_regs & 0xFF;
}

void modbus_set_dali_coil(uint8_t device, uint8_t cmd, uint16_t addr,
                          uint16_t value) {
  dali_dev_data_t *dalidev = dali_devices + addr;
  if (value == 0xFF00) {
    dali_toggle(addr);
    value = dalidev->level > 0 ? 0 : 0xFF00;
  } else {
    dali_set_on(addr, value == 0xFF00);
  }

  // Set the value.
  *response++ = addr >> 8;
  *response++ = addr & 0xFF;
  *response++ = value >> 8;
  *response++ = 0x00;
}

void modbus_write_register(uint8_t device, uint8_t cmd, uint16_t addr,
                           uint16_t value) {

  if (device == DEVICE_ADDR_FIXTURES) {
    binding_t binding;

    binding.type = value >> 14;
    switch (binding.type) {
    case BINDING_TYPE_DALI:
      binding.device = (value >> 6) & 0xFF;
      binding.address = value & 0x3F;
      break;

    case BINDING_TYPE_MODBUS:
      binding.device = (value >> 7) & 0x7F;
      binding.address = value & 0x7F;
      break;
    case BINDING_TYPE_NONE:
      break;
    }
    set_and_persist_binding(addr / 7, addr % 7, &binding);

  
  }
  *response++ = addr >> 8;
  *response++ = addr & 0xFF;
  *response++ = value >> 8;
  *response++ = value & 0xFF;

  // Set the value.
}

/**
 * Note - This function will be called on the first core, not the core that the
 * IO loop is running on.
 */
static void
modbus_noncompliant_coil_fetch_completed(modbus_task_state_t state,
                                         uint8_t *downstream_response,
                                         size_t sz, void *arg) {

  if (state == MODBUS_TASK_STATE_DONE) {
    // The CRC has already been validated by the downstream receiver.
    // Copy the response, except for the CRC (which will be calculated again)
    memcpy(res_bytes, downstream_response, sz - 2);
    response = res_bytes + sz - 2;

    // If downstream was successful and we have at least a few bytes, reverse
    // them
    if (sz >= 5 && (downstream_response[1] & 0x80) == 0) {
      int num_bytes = downstream_response[2];
      for (int i = 0; i < num_bytes; i++) {
        res_bytes[i + 3] = downstream_response[2 + num_bytes - i];
      }
    };
  } else {
    set_response_to_error(cmd_bytes[0], cmd_bytes[1],
                          MODBUS_ERR_GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND);
  }

  // Let the responder thread know it can continue.
  sem_release(&downstream_response_ready);
  if (arg) {
    free(arg);
  }
}

void await_downstream_response() {
  int status;
  if (!sem_acquire_timeout_ms(&downstream_response_ready, 1000)) {
    // Timeout while waiting for a response.
    set_response_to_error(cmd_bytes[0], cmd_bytes[1],
                          MODBUS_ERR_GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND);
  }
}

void send_modbus_bits(uint8_t device, modbus_cmd_t cmd, uint16_t addr,
                      uint16_t count) {
  unsigned numBytes = count / 8;
  if (count % 8) {
    numBytes++;
  }

  // Prepare the start of the response
  *response++ = numBytes;

  // We are dealing with Buttons.
  if (device == DEVICE_ADDR_FIXTURES &&
      cmd == MODBUS_CMD_READ_DISCRETE_INPUTS) {
    if (addr + count > NUM_FIXTURES * NUM_BUTTONS_PER_FIXTURE) {
      set_response_to_error(device, cmd, MODBUS_ERR_ILLEGAL_DATA_ADDR);
      return;
    }
    uint8_t b = 0;
    int bitsShifted = 0;

    while (count--) {
      bool bit_set = is_button_pressed(addr / 7, addr % 7);
      addr++;
      b >>= 1;
      if (bit_set) {
        b |= 0x80;
      }
      if (++bitsShifted == 8) {
        // We have a whole byte, so send it.
        *response++ = b;
        b = 0;
        bitsShifted = 0;
      }
    }
    // If there are any remainder bits, we need to shunt them to the end of the
    // byte and transmit it.
    if (bitsShifted) {
      // Send remainder bits.
      *response++ = b >> (8 - bitsShifted);
    }
  } else if (device == 1 && cmd == MODBUS_CMD_READ_COILS) {
    // This device is not properly MODBUS compliant.  It will only return all
    // coils at once, and it reverses the byte order.

    if (addr + count > 32) {
      set_response_to_error(device, cmd, MODBUS_ERR_ILLEGAL_DATA_ADDR);
      return;
    }
    modbus_downstream_get_coils(device, 0, 32,
                                modbus_noncompliant_coil_fetch_completed);

    // Wait for the completion
    await_downstream_response();
  }
}

void send_modbus_registers(uint8_t device, uint8_t cmd, uint16_t addr,
                           uint16_t count) {
  binding_t binding;
  dali_dev_data_t *dali_device;
  uint16_t val;

  if (cmd == MODBUS_CMD_READ_HOLDING_REGISTERS) {
    int maxReg = device == DEVICE_ADDR_FIXTURES
                     ? NUM_FIXTURES * NUM_BUTTONS_PER_FIXTURE
                 : device == DEVICE_ADDR_DALI ? 64 * 6
                                              : 0;
    if (addr + count > maxReg) {
      set_response_to_error(device, cmd, MODBUS_ERR_ILLEGAL_DATA_ADDR);
      return;
    }
    *response++ = count * 2;

    while (count--) {
      switch (device) {
      case DEVICE_ADDR_FIXTURES:
        // Its the switch fixtures
        get_binding_at_index(addr, &binding);
        // 7 bits for address, 7 bits for device, and 2 bits for

        val = (binding.type << 14);
        switch (binding.type) {
        case BINDING_TYPE_DALI:
          val |= ((binding.device & 0xFF) << 6) | (binding.address & 0x3F);
          break;
        case BINDING_TYPE_MODBUS:
          val |= ((binding.device & 0x7F) << 7) | (binding.address & 0x7F);
          break;
        case BINDING_TYPE_NONE:
          // Nothing to add
          break;
        };

        break;
      case DEVICE_ADDR_DALI:
        // Its a dali device.
        dali_device = dali_devices + (addr % 64);

        switch (addr / 64) {
        case 0: // Level and Status - these are the values most likely to change
          val = dali_device->level | (dali_device->status << 8);
          break;
        case 1: // Type and ??
          val = dali_device->type;
          break;
        case 2: // Min + Max
          val = (dali_device->max << 8) | dali_device->min;
          break;
        case 3: // Ext Fade + Fade
          val = (dali_device->ext_fade << 8) | dali_device->fade;
          break;
        case 4: // groups
          val = dali_device->groups;
          break;
        case 5: // Pwr on level + system failure level
          val = dali_device->power_on_level | (dali_device->failure_level << 8);
          break;
        }
        break;
      default:
        set_response_to_error(device, cmd, MODBUS_ERR_ILLEGAL_DATA_ADDR);
        return;
      }

      *response++ = val >> 8;
      *response++ = val;
      addr++;
    }
  }
}

static int modbus_get_deviceid() {
  int v = getchar();
  if (v >= 0) {
    // Update the CRC - As this is the first byte in the stream, we reset the
    // CRC first.
    read_crc = 0xFFFF;
    crc_update(v, &read_crc);
  }
  return v;
}

static int modbus_get_uint8() {
  int v = getchar_timeout_us(1750);
  if (v >= 0) {
    // Update the CRC
    crc_update(v, &read_crc);
  }
  return v;
}

static bool modbus_read_crc() {
  int val = modbus_get_uint8();
  if (val < 0) {
    return -1;
  }
  val = modbus_get_uint8();
  if (val < 0) {
    return -1;
  }
  return read_crc == 0;
}

static int modbus_get_uint16() {
  int high = modbus_get_uint8();
  if (high < 0) {
    return -1;
  }
  int low = modbus_get_uint8();
  if (low < 0) {
    return -1;
  };
  return high << 8 | low;
}

static int modbus_get_bytestr(uint8_t *out) {
  int num_bytes = modbus_get_uint8();
  if (num_bytes < 0) {
    return -1;
  }
  int val;
  for (int i = 0; i < num_bytes; i++) {
    val = modbus_get_uint8();
    if (val < 0) {
      return val;
    }
    *out++ = val;
  }
  return num_bytes;
}

/**
 * Note - This function will be called on the first core, not the core that
 the
 * IO loop is running on.
 */
static void modbus_downstream_task_completed(modbus_task_state_t state,
                                             uint8_t *downstream_response,
                                             size_t sz, void *arg) {
  //   printf("L CMD response is %d\n", state);
  if (state == MODBUS_TASK_STATE_DONE) {
    memcpy(res_bytes, downstream_response, sz - 2);
    response = res_bytes + sz - 2;
  } else {
    set_response_to_error(cmd_bytes[0], cmd_bytes[1],
                          MODBUS_ERR_GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND);
  }
  // Let the main thread know it can continue.
  sem_release(&downstream_response_ready);
  if (arg) {
    free(arg);
  }
}

static void modbus_run_cmd() {
  int device, addr, count, value, expected_bytes, byte_count;
  uint8_t bytes[256];

  // Wait until the bus is idle for at least 1.75ms (bus reset)
  // This discards all characters till this point.
  //   while (getchar_timeout_us(1750) >= 0) {
  //   }

  device = modbus_get_deviceid();
  int cmd = modbus_get_uint8();
  if (cmd < 0) {
    return;
  }

  // Reset the response. Response packets always echo back out the first two
  // bytes (although errors change the MSB of the second)
  response = res_bytes;
  *response++ = device;
  *response++ = cmd;

  switch (cmd) {
  case MODBUS_CMD_READ_DISCRETE_INPUTS:
  case MODBUS_CMD_READ_COILS:
    addr = modbus_get_uint16();
    if (addr < 0) {
      break;
    }
    count = modbus_get_uint16();
    if (count < 0) {
      break;
    }
    if (!modbus_read_crc()) {
      break;
    }
    send_modbus_bits(device, cmd, addr, count);
    break;

  case MODBUS_CMD_READ_INPUT_REGISTERS:
  case MODBUS_CMD_READ_HOLDING_REGISTERS:
    addr = modbus_get_uint16();
    if (addr < 0) {
      break;
    }
    count = modbus_get_uint16();
    if (count < 0) {
      break;
    }
    // You can only request up to 125 values.
    if (count > 125) {
      set_response_to_error(device, cmd, MODBUS_ERR_ILLEGAL_DATA_ADDR);
      return;
    }
    if (!modbus_read_crc()) {
      break;
    }

    send_modbus_registers(device, cmd, addr, count);
    break;

  case MODBUS_CMD_WRITE_SINGLE_COIL:

    addr = modbus_get_uint16();
    if (addr < 0) {
      break;
    }
    value = modbus_get_uint16();
    if (value < 0) {
      break;
    }
    if (!modbus_read_crc()) {
      break;
    }

    switch (device) {
    case DEVICE_ADDR_DALI:
      modbus_set_dali_coil(device, cmd, addr, value);
      break;
    default:
      modbus_downstream_set_coil(device, addr, value,
                                 modbus_downstream_task_completed);
      await_downstream_response();
      break;
    }
    break;

  case MODBUS_CMD_WRITE_SINGLE_REGISTER:
    addr = modbus_get_uint16();
    if (addr < 0) {
      break;
    }
    value = modbus_get_uint16();
    if (value < 0) {
      break;
    }
    if (!modbus_read_crc()) {
      break;
    }

    modbus_write_register(device, cmd, addr, value);
    break;

  case MODBUS_CMD_WRITE_MULTIPLE_COILS:

    addr = modbus_get_uint16();
    if (addr < 0) {
      break;
    }
    count = modbus_get_uint16();
    if (count < 0) {
      break;
    }
    byte_count = modbus_get_bytestr(bytes);
    if (byte_count < 0) {
      break;
    }
    if (!modbus_read_crc()) {
      break;
    }

    expected_bytes = count / 8;
    if (count % 8) {
      expected_bytes++;
    }

    if (byte_count != expected_bytes) {
      set_response_to_error(device, cmd, MODBUS_ERR_ILLEGAL_DATA_VALUE);
    } else {
      modbus_downstream_set_coils(device, addr, count, bytes,
                                  modbus_downstream_task_completed);
    }
    break;

  case MODBUS_CMD_WRITE_MULTIPLE_REGISTERS:
    addr = modbus_get_uint16();
    if (addr < 0) {
      break;
    }
    count = modbus_get_uint16();
    if (count < 0) {
      break;
    }
    expected_bytes = count * 2;
    byte_count = modbus_get_bytestr(bytes);
    if (byte_count < 0) {
      break;
    }
    if (!modbus_read_crc()) {
      break;
    }
    if (byte_count != expected_bytes) {
      set_response_to_error(device, cmd, MODBUS_ERR_ILLEGAL_DATA_VALUE);
    } else {
      // The values will be non word aligned in the source, so we need
      // to copy them.
      uint16_t values[count];
      memcpy(values, bytes, count * 2);
      modbus_write_registers(device, cmd, addr, values, count);
    }
    break;
  }
  size_t sz = response - res_bytes;
  if (sz > 2) {
    // Send the response packet to the user.
    response = res_bytes;
    uint16_t crc = 0xFFFF;
    while (sz--) {
      stdio_putchar_raw(*response);
      crc_update(*response++, &crc);
    }
    stdio_putchar_raw(crc);
    stdio_putchar_raw(crc >> 8);
  }
}

void modbus_server_thread() {
  sem_init(&downstream_response_ready, 0, 1);
  while (1) {
    modbus_run_cmd();
  }
}
