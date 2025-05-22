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

static semaphore_t downstream_response_ready;
static uint16_t read_crc = 0xFFFF;

#define MODBUS_SERVER_READ_MAX_PACKET_SZ 256
static absolute_time_t last_read = 0;
static uint8_t cmd_bytes[MODBUS_SERVER_READ_MAX_PACKET_SZ];
static uint8_t res_bytes[MODBUS_SERVER_READ_MAX_PACKET_SZ];
static uint8_t *response;

typedef struct {
    unsigned addr;
    unsigned changed;
    unsigned newGroups;
    unsigned nextGroupId;
} dali_group_change_t;
dali_group_change_t groupChange;

static void set_response_to_error(modbus_err_t err) {
    // reset the buffer, if it had anything written to it.  This will overwrite what was there.
    res_bytes[0] = cmd_bytes[0];
    res_bytes[1] = cmd_bytes[1] | 0x80;  // Set the error MSB on the cmd that was set earlier
    res_bytes[2] = err;
    response = res_bytes + 3;
}

static bool await_downstream_response() {
    if (!sem_acquire_timeout_ms(&downstream_response_ready, 1000)) {
        // Timeout while waiting for a response.
        // toggleLED();
        set_response_to_error(MODBUS_ERR_GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND);
        return false;
    }
    return true;
}

static void modbus_write_registers(uint8_t device, uint8_t cmd, uint16_t addr, uint16_t data[], uint16_t num_regs) {
    // TODO set the registers
    *response++ = addr >> 8;
    *response++ = addr & 0xFF;
    *response++ = num_regs >> 8;
    *response++ = num_regs & 0xFF;
}

static void modbus_set_coil_completed(modbus_task_state_t state, uint8_t *cmd, uint8_t *downstream_response, size_t sz) {
    if (state != MODBUS_TASK_STATE_DONE) {
        set_response_to_error(MODBUS_ERR_GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND);
    }
    // Let the main thread know it can continue.
    sem_release(&downstream_response_ready);
}

static bool daliCommandSucceeded(int res, bool nakIsOkay) {
    if (res < 0) {
        switch (res) {
            case DALI_NAK:
                set_response_to_error(MODBUS_ERR_NACK);

                break;
            case DALI_BUS_ERROR:
                set_response_to_error(MODBUS_ERR_SLAVE_DEVICE_FAIL);
                break;
            case DALI_TIMEOUT:
            default:
                set_response_to_error(MODBUS_ERR_GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND);
                break;
        }
        return false;
    }
    return true;
}

static void dali_command_complete(int res) {
    daliCommandSucceeded(res, true);
    sem_release(&downstream_response_ready);
}

static inline bool no_response_set() { return response - res_bytes == 2; }

void set_coil(uint8_t device, uint8_t cmd, uint16_t addr, uint16_t value) {
    if (addr < MAX_COILS) {
        modbus_downstream_set_coil(1 + addr / 32, addr % 32, value, modbus_set_coil_completed);
        await_downstream_response();
    } else if (addr < MAX_COILS + MAX_DALI_LIGHTS) {
        dali_toggle(addr - MAX_COILS, dali_command_complete);
        await_downstream_response();
        // The level is not guaranteed to have been changed immediately after this, as fading is an asynchronous process.
    } else {
        set_response_to_error(MODBUS_ERR_ILLEGAL_DATA_ADDR);
    }
    // Copy the request to the response, if no error was set
    if (no_response_set()) {
        memcpy(response, cmd_bytes + 2, 4);
        response += 4;
    }
}

static void dali_custom_command_complete(int res) {
    if (daliCommandSucceeded(res, true)) {
        *response++ = res;
    }
    sem_release(&downstream_response_ready);
}

static void dali_group_change_step(int responseFromLastChange) {
    if (daliCommandSucceeded(responseFromLastChange, true)) {
        while (groupChange.nextGroupId < 16) {
            if (groupChange.changed & 0x01) {
                // Send the change, then wait for a callback.
                if (groupChange.newGroups & 1) {
                    dali_add_to_group(groupChange.addr, groupChange.nextGroupId, dali_group_change_step);
                } else {
                    dali_remove_from_group(groupChange.addr, groupChange.nextGroupId, dali_group_change_step);
                }
                return;  // Without releasing the semaphore, as we haven't finished the loop yet.  The command we just enqueued
                         // will call this callback again to complete the set command.
            }
            groupChange.changed >>= 1;
            groupChange.newGroups >>= 1;
            groupChange.nextGroupId++;
        }
    }
    sem_release(&downstream_response_ready);
}

void set_holding_register_action(int addr, uint16_t value) {
    // First set of values are Button Bindings
    if (addr < MAX_DISCRETE_INPUTS) {
        set_and_persist_binding(addr, value);
        return;
    }

    // After the bindings, we have banks of 64 registers, one register for each ballast on the dali bus.
    addr -= MAX_DISCRETE_INPUTS;
    unsigned dali_bank = DALI_HR_BANK_ID_FROM_REGID(addr);
    addr = DALI_ADDR_FROM_REGID(addr);
    uint16_t currentGroups, diff;

    switch (dali_bank) {
        case DALI_HR_BANKID_STATUS:
            // Its light level - Ignore status
            dali_set_level(addr, value & 0xFF, dali_command_complete);
            await_downstream_response();
            break;
        case DALI_HR_BANKID_MINMAX:
            dali_set_min_max_level(addr, value & 0xFF, value >> 8, dali_command_complete);
            await_downstream_response();
            break;
        case DALI_HR_BANKID_FADE:
            // Its Ext Fade Fade Time and Fade Time/Rate
            dali_set_fade_time_rate(addr, value & 0xFF, value >> 8, dali_command_complete);
            await_downstream_response();
            break;
        case DALI_HR_BANKID_POWERON:
            // Its System Level and failure level
            dali_set_power_on_level(addr, value & 0xFF, value >> 8, dali_command_complete);
            await_downstream_response();
            break;
        case DALI_HR_BANKID_GROUPS:
            // Each group must be set or removed as a separate operation. We do this as a series of operations, only completing
            // once we've done all 16.
            groupChange.changed = get_holding_reg(DALI_GROUPS_HR_BASE + addr) ^ value;
            if (groupChange.changed) {
                groupChange.newGroups = value;
                groupChange.nextGroupId = 0;
                groupChange.addr = addr;

                dali_group_change_step(0);
                await_downstream_response();
            }
            break;
        default:
            set_response_to_error(MODBUS_ERR_ILLEGAL_DATA_ADDR);
            break;
    }
}

void modbus_write_holding_register(uint8_t device, uint8_t cmd, uint16_t addr, uint16_t value) {
    if (addr >= MAX_HOLDING_REGISTERS) {
        set_response_to_error(MODBUS_ERR_ILLEGAL_DATA_ADDR);
        return;
    }

    // Response is just a reflection of the input
    *response++ = addr >> 8;
    *response++ = addr & 0xFF;
    *response++ = value >> 8;
    *response++ = value & 0xFF;

    // We do the action after setting the response, so that it can set an error code if it wants.
    set_holding_register_action(addr, value);
}

void read_modbus_bits(uint8_t device, modbus_cmd_t cmd, uint16_t addr, uint16_t count) {
    if (addr + count > MAX_COILS || addr % 8 != 0 || count % 8 != 0) {
        set_response_to_error(MODBUS_ERR_ILLEGAL_DATA_ADDR);
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
        set_response_to_error(MODBUS_ERR_ILLEGAL_DATA_ADDR);
        return;
    }
    *response++ = count * 2;
    copy_holding_regs(response, addr, count);
    response += count * 2;
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

static bool start_process(int process_type) {
    switch (process_type) {
        case 0:
            dali_enumerate();
            return true;
        default:
            return false;
    }
}

static void modbus_run_cmd() {
    int device, addr, count, value, expected_bytes, byte_count;
    int cmd_repeat;
    uint8_t bytes[256];

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
            read_modbus_bits(device, cmd, addr, count);
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
                set_response_to_error(MODBUS_ERR_ILLEGAL_DATA_ADDR);
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
                set_response_to_error(MODBUS_ERR_ILLEGAL_DATA_VALUE);
            } else {
                // we don't support setting multiple coils (yet);
                set_response_to_error(MODBUS_ERR_ILLEGAL_FUNCTION);
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
                set_response_to_error(MODBUS_ERR_ILLEGAL_DATA_VALUE);
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
            if (value < 0) {
                break;
            }
            cmd_repeat = modbus_read_uint8();
            if (cmd_repeat < 0) {
                break;
            }
            if (!modbus_read_crc()) {
                break;
            }
            dali_exec_cmd(value, dali_custom_command_complete, cmd_repeat ? true : false);
            await_downstream_response();
            break;

        case MODBUS_CMD_CUSTOM_START_PROCESS:
            value = modbus_read_uint8();
            if (addr < 0) {
                break;
            }
            if (!modbus_read_crc()) {
                break;
            }
            if (start_process(value)) {
                *response++ = value;
            } else {
                set_response_to_error(MODBUS_ERR_ILLEGAL_DATA_ADDR);
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
