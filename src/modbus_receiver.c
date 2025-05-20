#include "modbus_receiver.h"

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

#include "buttons.h"
#include "crcbuf.h"
#include "dali.h"
#include "modbus.h"
#include "regs.h"

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
    response[1] |= 0x80;  // Set the error MSB on the cmd that was set earlier
    response[2] = err;
    response = res_bytes + 3;
}

void await_downstream_response() {
    if (!sem_acquire_timeout_ms(&downstream_response_ready, 1000)) {
        // Timeout while waiting for a response.
        set_response_to_error(cmd_bytes[0], cmd_bytes[1], MODBUS_ERR_GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND);
    }
}

void modbus_write_registers(uint8_t device, uint8_t cmd, uint16_t addr, uint16_t data[], uint16_t num_regs) {
    // TODO set the registers
    *response++ = addr >> 8;
    *response++ = addr & 0xFF;
    *response++ = num_regs >> 8;
    *response++ = num_regs & 0xFF;
}

static void modbus_set_coil_completed(modbus_task_state_t state, uint8_t *cmd, uint8_t *downstream_response, size_t sz) {
    switch (state) {
        case MODBUS_TASK_STATE_DONE:
            // The response is reflection a copy of the call.
            memcpy(res_bytes, downstream_response, sz - 2);
            response = res_bytes + sz - 2;
            // Register Has already been updated by downstream system
            break;
        default:
            set_response_to_error(cmd_bytes[0], cmd_bytes[1], MODBUS_ERR_GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND);
            break;
    }
    // Let the main thread know it can continue.
    sem_release(&downstream_response_ready);
}

void set_coil(uint8_t device, uint8_t cmd, uint16_t addr, uint16_t value) {
    if (addr >= MAX_COILS) {
        set_response_to_error(device, cmd, MODBUS_ERR_ILLEGAL_DATA_ADDR);
        return;
    }
    modbus_downstream_set_coil(1, addr, value, modbus_set_coil_completed);
    await_downstream_response();
}

bool set_holding_register_action(int addr, uint16_t value) {
    // First set of values are
    if (addr < MAX_DISCRETE_INPUTS) {
        // Its a binding
    }

    // After the bindings, we have banks of 64 registers, one register for each fitting on the dali bus.
    addr -= MAX_DISCRETE_INPUTS;
    if (addr < MAX_DALI_LIGHTS) {
        // Its light level - Ignore status
        dali_set_level(addr, value & 0xFF);

        // The dali refresh code will update the register for us once we're done.
        return false;
    }

    addr -= MAX_DALI_LIGHTS;
    if (addr < MAX_DALI_LIGHTS) {
        // Its Max and Min
        dali_set_max_level(addr, value >> 8);
        dali_set_min_level(addr, value & 0xFF);
        return true;
    }

    addr -= MAX_DALI_LIGHTS;
    if (addr < MAX_DALI_LIGHTS) {
        // Its Ext Fade Fade Time and Fade Time/Rate
        dali_set_extended_fade_time(addr, value >> 8);
        dali_set_fade_time(addr, value & 0xFF);
        dali_set_fade_rate(addr, value & 0xFF);
        return true;
    }

    addr -= MAX_DALI_LIGHTS;
    if (addr < MAX_DALI_LIGHTS) {
        // Its System Level and failure level
        dali_set_system_failure_level(addr, value >> 8);
        dali_set_power_on_level(addr, value & 0xFF);
        return true;
    }

    addr -= MAX_DALI_LIGHTS;
    if (addr < MAX_DALI_LIGHTS) {
        // Its groups.
        // Each group must be set or removed as a separate operation. We do this by looking at current groups
        uint16_t currentGroups = get_holding_reg(DALI_GROUPS_HR_BASE + addr);
        uint16_t diff = currentGroups ^ value;
        for (int group = 0; group < 16; group++) {
            if (diff & 0x01) {
                if (currentGroups & 0x01) {
                    // It was present, so we remove it.
                    dali_remove_from_group(addr, group);
                } else {
                    dali_add_to_group(addr, group);
                }
            }
            currentGroups >>= 1;
            diff >>= 1;
        }
        return true;
    }

    // If we get here, it means we've run out of registers, so it will be a no-op
    return true;
}

void modbus_write_holding_register(uint8_t device, uint8_t cmd, uint16_t addr, uint16_t value) {
    if (addr >= MAX_HOLDING_REGISTERS) {
        set_response_to_error(device, cmd, MODBUS_ERR_ILLEGAL_DATA_ADDR);
        return;
    }

    // Set the register via a mutex to avoid cross-core problems, but only if its not doing a fade after the change.
    set_holding_register_action(addr, value);
    await_downstream_response();

    *response++ = addr >> 8;
    *response++ = addr & 0xFF;
    *response++ = value >> 8;
    *response++ = value & 0xFF;

    // Set the value.
}

void send_modbus_bits(uint8_t device, modbus_cmd_t cmd, uint16_t addr, uint16_t count) {
    if (addr + count > MAX_COILS || addr % 8 != 0 || count % 8 != 0) {
        set_response_to_error(device, cmd, MODBUS_ERR_ILLEGAL_DATA_ADDR);
        return;
    }
    unsigned numBytes = count / 8;
    // Copy Coil values from regs into the output.
    *response++ = numBytes;
    if (cmd == MODBUS_CMD_READ_COILS) {
        copy_coil_values(response, addr, count);
    } else {
        copy_discrete_inputs(response, addr, count);
    }
    response += numBytes;
}

void send_modbus_holding_registers(uint8_t device, uint8_t cmd, uint16_t addr, uint16_t count) {
    binding_t binding;
    uint16_t val;

    if (addr + count > MAX_HOLDING_REGISTERS) {
        set_response_to_error(device, cmd, MODBUS_ERR_ILLEGAL_DATA_ADDR);
        return;
    }
    *response++ = count * 2;
    copy_holding_regs(response, addr, count);
    response += count * 2;
}

static void dali_command_complete(int res) {
    if (res == DALI_NAK) {
        set_response_to_error(cmd_bytes[0], cmd_bytes[1], MODBUS_ERR_NACK);
    } else if (res == DALI_BUS_ERROR) {
        set_response_to_error(cmd_bytes[0], cmd_bytes[1], MODBUS_ERR_SLAVE_DEVICE_FAIL);
    } else if (res == DALI_TIMEOUT) {
        set_response_to_error(cmd_bytes[0], cmd_bytes[1], MODBUS_ERR_GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND);
    } else {
        *response++ = res;
    }
    sem_release(&downstream_response_ready);
}

static void exec_custom_dali(int device, uint16_t command) {
    dali_exec_cmd(command, dali_command_complete);
    await_downstream_response();
}

static int modbus_read_device() {
    int v = getchar();
    if (v >= 0) {
        // Update the CRC - As this is the first byte in the stream, we reset the
        // CRC first.
        read_crc = 0xFFFF;
        crc_update(v, &read_crc);
    }
    return v;
}

static int modbus_read_uint8() {
    int v = getchar_timeout_us(1750);
    if (v >= 0) {
        // Update the CRC
        crc_update(v, &read_crc);
    }
    return v;
}

static bool modbus_read_crc() {
    int val = modbus_read_uint8();
    if (val < 0) {
        return -1;
    }
    val = modbus_read_uint8();
    if (val < 0) {
        return -1;
    }
    return read_crc == 0;
}

static int modbus_read_uint16() {
    int high = modbus_read_uint8();
    if (high < 0) {
        return -1;
    }
    int low = modbus_read_uint8();
    if (low < 0) {
        return -1;
    };
    return high << 8 | low;
}

static int modbus_read_bytestr(uint8_t *out) {
    int num_bytes = modbus_read_uint8();
    if (num_bytes < 0) {
        return -1;
    }
    int val;
    for (int i = 0; i < num_bytes; i++) {
        val = modbus_read_uint8();
        if (val < 0) {
            return val;
        }
        *out++ = val;
    }
    return num_bytes;
}

static void modbus_run_cmd() {
    int device, addr, count, value, expected_bytes, byte_count;
    uint8_t bytes[256];

    // Wait until the bus is idle for at least 1.75ms (bus reset)
    // This discards all characters till this point.
    //   while (getchar_timeout_us(1750) >= 0) {
    //   }

    device = modbus_read_device();
    int cmd = modbus_read_uint8();
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
            addr = modbus_read_uint16();
            if (addr < 0) {
                break;
            }
            count = modbus_read_uint16();
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
            addr = modbus_read_uint16();
            if (addr < 0) {
                break;
            }
            count = modbus_read_uint16();
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

            send_modbus_holding_registers(device, cmd, addr, count);
            break;

        case MODBUS_CMD_WRITE_SINGLE_COIL:

            addr = modbus_read_uint16();
            if (addr < 0) {
                break;
            }
            value = modbus_read_uint16();
            if (value < 0) {
                break;
            }
            if (!modbus_read_crc()) {
                break;
            }

            set_coil(device, cmd, addr, value);
            break;

        case MODBUS_CMD_WRITE_SINGLE_REGISTER:
            addr = modbus_read_uint16();
            if (addr < 0) {
                break;
            }
            value = modbus_read_uint16();
            if (value < 0) {
                break;
            }
            if (!modbus_read_crc()) {
                break;
            }

            modbus_write_holding_register(device, cmd, addr, value);
            break;

        case MODBUS_CMD_WRITE_MULTIPLE_COILS:

            addr = modbus_read_uint16();
            if (addr < 0) {
                break;
            }
            count = modbus_read_uint16();
            if (count < 0) {
                break;
            }
            byte_count = modbus_read_bytestr(bytes);
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
                // we don't support setting multiple coils (yet);
                set_response_to_error(device, cmd, MODBUS_ERR_ILLEGAL_FUNCTION);
            }
            break;

        case MODBUS_CMD_WRITE_MULTIPLE_REGISTERS:
            addr = modbus_read_uint16();
            if (addr < 0) {
                break;
            }
            count = modbus_read_uint16();
            if (count < 0) {
                break;
            }
            expected_bytes = count * 2;
            byte_count = modbus_read_bytestr(bytes);
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

        case MODBUS_CMD_CUSTOM_EXEC_DALI:
            value = modbus_read_uint16();
            if (addr < 0) {
                break;
            }
            if (!modbus_read_crc()) {
                break;
            }
            exec_custom_dali(device, value);

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
