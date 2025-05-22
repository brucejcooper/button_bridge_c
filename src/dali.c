#include "dali.h"

#include <hardware/clocks.h>
#include <hardware/structs/clocks.h>
#include <pico/stdlib.h>
#include <pico/util/queue.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dali.pio.h"
#include "modbus.h"
#include "regs.h"
#include "stdbool.h"
#include "stdint.h"

struct dali_cmd_t;
typedef void (*cmd_chain_cb_t)(int res, struct dali_cmd_t *cb);
typedef struct dali_cmd_t {
    unsigned int addr;
    uint16_t op;
    uint8_t param;
    bool sendTwice;
    cmd_chain_cb_t then;
    dali_result_cb_t finally;
} dali_cmd_t;

#define DALI_ADDR_FROM_CMD(cmd) ((cmd >> 9) & 0x3F)
#define DALI_CMD_STRIP_ADDR(cmd) ((cmd) & 0x1FF)

#define DALI_CMD_QUERY_STATUS(addr) (((addr) << 9) | 0x190)
#define DALI_CMD_QUERY_DEVICE_TYPE(addr) (((addr) << 9) | 0x199)
#define DALI_CMD_QUERY_ACTUAL_LEVEL(addr) (((addr) << 9) | 0x1a0)
#define DALI_CMD_QUERY_MAX(addr) (((addr) << 9) | 0x1a1)
#define DALI_CMD_QUERY_MIN(addr) (((addr) << 9) | 0x1a2)
#define DALI_CMD_QUERY_POWER_ON_LEVEL(addr) (((addr) << 9) | 0x1a3)
#define DALI_CMD_QUERY_SYSTEM_FAILURE_LEVEL(addr) (((addr) << 9) | 0x1a4)
#define DALI_CMD_QUERY_FADE_RATE_FADE_TIME(addr) (((addr) << 9) | 0x1a5)
#define DALI_CMD_QUERY_EXTENDED_FADE_RATE(addr) (((addr) << 9) | 0x1a8)
#define DALI_CMD_QUERY_GROUPS_ZERO_TO_SEVEN(addr) (((addr) << 9) | 0x1c0)
#define DALI_CMD_QUERY_GROUPS_EIGHT_TO_FIFTEEN(addr) (((addr) << 9) | 0x1c1)

#define DALI_CMD_RECALL_LAST_ACTIVE_LEVEL(addr) (((addr) << 9) | 0x10a)
#define DALI_CMD_OFF(addr) (((addr) << 9) | 0x100)
#define DALI_CMD_UP(addr) (((addr) << 9) | 0x101)
#define DALI_CMD_DOWN(addr) (((addr) << 9) | 0x102)
#define DALI_CMD_STEP_UP(addr) (((addr) << 9) | 0x103)
#define DALI_CMD_STEP_DOWN(addr) (((addr) << 9) | 0x104)
#define DALI_CMD_RECALL_MAX(addr) (((addr) << 9) | 0x105)
#define DALI_CMD_RECALL_MIN(addr) (((addr) << 9) | 0x106)
#define DALI_CMD_STEP_DOWN_AND_OFF(addr) (((addr) << 9) | 0x107)
#define DALI_CMD_ON_AND_STEP_UP(addr) (((addr) << 9) | 0x108)
#define DALI_CMD_READ_MEMORY_LOCATION(addr) (((addr) << 9) | 0x1c5)

#define DALI_CMD_SET_MAX_LEVEL(addr) (((addr) << 9) | 0x12a)
#define DALI_CMD_SET_MIN_LEVEL(addr) (((addr) << 9) | 0x12b)
#define DALI_CMD_SET_SYSTEM_FAIL_LEVEL(addr) (((addr) << 9) | 0x12c)
#define DALI_CMD_SET_POWER_ON_LEVEL(addr) (((addr) << 9) | 0x12d)
#define DALI_CMD_SET_FADE_TIME(addr) (((addr) << 9) | 0x12e)
#define DALI_CMD_SET_FADE_RATE(addr) (((addr) << 9) | 0x12f)
#define DALI_CMD_SET_EXTENDED_FADE_TIME(addr) (((addr) << 9) | 0x130)

#define DALI_CMD_ADD_TO_GROUP(addr, group) (((addr) << 9) | 0x160 | group)
#define DALI_CMD_REMOVE_FROM_GROUP(addr, group) (((addr) << 9) | 0x170 | group)

#define DALI_CMD_SET_DTR0(val) (0xa300 | val)
#define DALI_CMD_SET_DTR1(val) (0xc300 | val)
#define DALI_CMD_SET_DTR2(val) (0xc500 | val)

#define DALI_STATUS_BALLAST 0x01
#define DALI_STATUS_LAMP_FAILURE 0x02
#define DALI_STATUS_ARC_POWER_ON 0x04
#define DALI_STATUS_LIMIT_ERROR 0x08
#define DALI_STATUS_FADE_IN_PROGRESS 0x10
#define DALI_STATUS_RESET_STATE 0x20
#define DALI_STATUS_MISSING_SHORT_ADDR 0x40
#define DALI_STATUS_MISSING_POWER_FAILURE 0x80

#define DALI_MAX_ADDR 63

static const PIO pio = pio0;
static const unsigned int dali_sm = 0;

#define QUEUE_DEPTH 70
static queue_t dali_queue;
static dali_cmd_t in_flight = {.op = 0, .sendTwice = false, .addr = 0xFF, .then = NULL, .finally = NULL, .param = 0};

bool dali_scan_in_progress = false;

// ------------------------- in-flight action callbacks ----------------

static void request_level_update(int addr);
static void scan_got_result(int min, dali_cmd_t *cmd);

static char tmp[30];

static const char *TAG = "DALI";

static void scan_dali_device(int addr) {
    // defer_log(TAG, "Scanning DALI Address %d", addr);
    // This is a new enqueue, rather than a continuation, because we want it
    // possible for other commands to execute interleaved.

    dali_cmd_t cmd = {.op = DALI_CMD_QUERY_DEVICE_TYPE(addr),
                      .addr = addr,
                      .then = scan_got_result,
                      .finally = NULL,
                      .sendTwice = false,
                      .param = 0};
    if (!queue_try_add(&dali_queue, &cmd)) {
        onError();
    }
}

static void scan_next(int previousAddr) {
    if (previousAddr < DALI_MAX_ADDR) {
        // Start a new task to enumerate the next address.
        scan_dali_device(previousAddr + 1);
    } else {
        dali_scan_in_progress = false;
        // defer_log(TAG, "Dali Scan Done");
    }
}

static inline char *dali_err_to_str(int val) {
    switch (val) {
        case DALI_NAK:
            return "Not Acknowledged";
        case DALI_TIMEOUT:
            return "Timeout";
        case DALI_BUS_ERROR:
            return "Bus Error";
        default:
            return val >= 0 ? "No Error" : "Unknown error";
    }
}

bool dali_is_fadeable(int addr) {
    return false;
    if (addr >= 64) {
        return false;
    }
    int minmax = get_holding_reg(DALI_MINMAX_HR_BASE + addr);
    int max = minmax >> 8;
    int min = minmax & 0xFF;
    return (min != max);
}

static void scan_failed(dali_cmd_t *cmd, int res) {
    // defer_log(TAG, "Scan of device %d cmd 0x%04x %s", addr, cmd->op,
    // dali_err_to_str(res)); enqueue_device_update(EVT_DALI_DEVICE_DISCOVERED,
    // dev);
    set_holding_reg(DALI_STATUS_HR_BASE + cmd->addr, 0xFFFF);
    set_holding_reg(DALI_MINMAX_HR_BASE + cmd->addr, 0xFFFF);
    set_holding_reg(DALI_POWERON_HR_BASE + cmd->addr, 0xFFFF);
    set_holding_reg(DALI_FADE_HR_BASE + cmd->addr, 0xFFFF);
    set_holding_reg(DALI_GROUPS_HR_BASE + cmd->addr, 0);

    // Start a new task to enumerate the next address.
    scan_next(cmd->addr);
}

static uint8_t *memptr;

// static void memory_scan_byte_read(int result, dali_cmd_t *cmd) {
//     dali_dev_data_t *dev = cmd->dev;
//     int addr = dev - dali_devices;
//     int memaddr;

//     // defer_log(TAG, "Byte %d of device %d returned %d", (memptr - ((uint8_t *)
//     // &(dev->bank0))), addr, result);

//     if (result < 0) {
//         scan_failed(cmd, result);
//     } else {
//         *memptr++ = result;
//         int num_read = memptr - ((uint8_t *)&(dev->bank0));
//         if (num_read == sizeof(dali_device_bank_0_t)) {
//             // defer_log(TAG, "Discovered device %d on DALI bus", dev->addr);
//             // enqueue_device_update(EVT_DALI_DEVICE_DISCOVERED, dev);
//             request_level_update(dev);
//             // If necessary, start a scan for the next device address.
//             scan_next(addr);
//         } else {
//             cmd->op = DALI_CMD_READ_MEMORY_LOCATION(addr);
//             cmd->then = memory_scan_byte_read;
//         }
//     }
// }

// static void set_memory_scan_address_done(int result, dali_cmd_t *cmd) {
//     dali_dev_data_t *dev = cmd->dev;
//     int addr = dev - dali_devices;

//     if (result != -1) {
//         scan_failed(cmd, result);
//     } else {
//         // defer_log(TAG, "Set Address 2, Requesting Read of memory from device %d",
//         // addr);
//         memptr = (uint8_t *)&(dev->bank0);
//         cmd->op = DALI_CMD_READ_MEMORY_LOCATION(addr);
//         cmd->then = memory_scan_byte_read;
//     }
// }

// static void set_memory_scan_bank_done(int result, dali_cmd_t *cmd) {
//     if (result != -1) {
//         // defer_log(TAG, "Error setting DTR0");
//         scan_failed(cmd, result);
//     } else {
//         // defer_log(TAG, "Set Memory bank 0, setting address 2");
//         cmd->op = DALI_CMD_SET_DTR0(2);
//         cmd->then = set_memory_scan_address_done;
//     }
// }

static void scan_got_result(int result, dali_cmd_t *cmd) {
    // defer_log(TAG, "Scan of %d cmd 0x%04x result %d", addr, cmd->op, result);

    if (result < 0) {
        scan_failed(cmd, result);
    } else {
        cmd->then = scan_got_result;

        switch (DALI_CMD_STRIP_ADDR(cmd->op)) {
            case DALI_CMD_QUERY_DEVICE_TYPE(0):
                // TODO we don't store the Type anywhere yet.
                // dev->type = result;
                cmd->op = DALI_CMD_QUERY_MIN(cmd->addr);
                break;
            case DALI_CMD_QUERY_MIN(0):
                set_holding_reg_byte(DALI_MINMAX_HR_BASE + cmd->addr, 0, result);
                cmd->op = DALI_CMD_QUERY_MAX(cmd->addr);
                break;
            case DALI_CMD_QUERY_MAX(0):
                set_holding_reg_byte(DALI_MINMAX_HR_BASE + cmd->addr, 1, result);
                cmd->op = DALI_CMD_QUERY_POWER_ON_LEVEL(cmd->addr);
                break;
            case DALI_CMD_QUERY_POWER_ON_LEVEL(0):
                set_holding_reg_byte(DALI_POWERON_HR_BASE + cmd->addr, 0, result);
                cmd->op = DALI_CMD_QUERY_SYSTEM_FAILURE_LEVEL(cmd->addr);
                break;
            case DALI_CMD_QUERY_SYSTEM_FAILURE_LEVEL(0):
                set_holding_reg_byte(DALI_POWERON_HR_BASE + cmd->addr, 1, result);
                cmd->op = DALI_CMD_QUERY_FADE_RATE_FADE_TIME(cmd->addr);
                break;
            case DALI_CMD_QUERY_FADE_RATE_FADE_TIME(0):
                set_holding_reg_byte(DALI_FADE_HR_BASE + cmd->addr, 0, result);
                cmd->op = DALI_CMD_QUERY_EXTENDED_FADE_RATE(cmd->addr);
                break;
            case DALI_CMD_QUERY_EXTENDED_FADE_RATE(0):
                set_holding_reg_byte(DALI_FADE_HR_BASE + cmd->addr, 1, result);
                cmd->op = DALI_CMD_QUERY_GROUPS_ZERO_TO_SEVEN(cmd->addr);
                break;
            case DALI_CMD_QUERY_GROUPS_ZERO_TO_SEVEN(0):
                set_holding_reg_byte(DALI_GROUPS_HR_BASE + cmd->addr, 0, result);
                cmd->op = DALI_CMD_QUERY_GROUPS_EIGHT_TO_FIFTEEN(cmd->addr);
                break;
            case DALI_CMD_QUERY_GROUPS_EIGHT_TO_FIFTEEN(0):
                set_holding_reg_byte(DALI_GROUPS_HR_BASE + cmd->addr, 1, result);
                request_level_update(cmd->addr);
                cmd->then = NULL;
                scan_next(cmd->addr);
                break;
        }
    }
}

static void fade_received_status(int status, dali_cmd_t *cmd) {
    int addr = DALI_ADDR_FROM_CMD(cmd->op);
    int device = 1;
    if (status >= 0) {
        set_holding_reg_byte(DALI_STATUS_HR_BASE + cmd->addr, 1, status);
    }

    if (status & DALI_STATUS_FADE_IN_PROGRESS) {
        // Fade still active.  Check again
        request_level_update(cmd->addr);
    }
}

static void fade_received_level(int lvl, dali_cmd_t *cmd) {
    int addr = DALI_ADDR_FROM_CMD(cmd->op);
    int device = 1;
    if (lvl >= 0) {
        set_holding_reg_byte(DALI_STATUS_HR_BASE + cmd->addr, 0, lvl);
    }
    cmd->op = DALI_CMD_QUERY_STATUS(addr);
    cmd->then = fade_received_status;
}

static void async_report_level_with_fade(int ret, dali_cmd_t *cmd) {
    // We don't want other commands to be stalled waiting for a fade to conclude,
    // so we push a new command rather than update the existing one.
    dali_cmd_t newcmd = {.op = DALI_CMD_QUERY_ACTUAL_LEVEL(cmd->addr),
                         .addr = cmd->addr,
                         .then = fade_received_level,
                         .finally = NULL,
                         .sendTwice = false,
                         .param = 0};
    queue_try_add(&dali_queue, &newcmd);
}

static void noop_result_handler(int ret, dali_cmd_t *cmd) {}

void dali_exec_cmd(uint16_t cmd, dali_result_cb_t resultHandler, bool sendTwice) {
    dali_cmd_t newcmd = {
        .op = cmd, .addr = 0, .then = noop_result_handler, .finally = resultHandler, .sendTwice = sendTwice, .param = 0};
    queue_try_add(&dali_queue, &newcmd);
}

static void request_level_update(int addr) {
    dali_cmd_t newcmd = {.op = DALI_CMD_QUERY_ACTUAL_LEVEL(addr),
                         .addr = addr,
                         .then = fade_received_level,
                         .finally = NULL,
                         .sendTwice = false,
                         .param = 0};
    queue_try_add(&dali_queue, &newcmd);
}

static void toggle_level_received(int lvl, dali_cmd_t *cmd) {
    // Follow through from level check to send out the on or off command.
    cmd->op = lvl > 0 ? DALI_CMD_OFF(cmd->addr) : DALI_CMD_RECALL_LAST_ACTIVE_LEVEL(cmd->addr);
    cmd->then = async_report_level_with_fade;
}

// ----------------------------- API -------------------------

static inline void send_dali_cmd(uint cmd) {
    // This says blocking, but it is very unlikely that it will ever block, due to
    // the serial nature of how commands are executed.
    pio_sm_put_blocking(pio, dali_sm, ((cmd << 15) | 0x80000000) + 88);
}

void dali_toggle(int addr, dali_result_cb_t cb) {
    dali_cmd_t cmd = {.op = DALI_CMD_QUERY_ACTUAL_LEVEL(addr),
                      .addr = addr,
                      .then = toggle_level_received,
                      .finally = cb,
                      .sendTwice = false,
                      .param = 0};
    queue_try_add(&dali_queue, &cmd);
}

void dali_set_on(int addr, bool is_on, dali_result_cb_t cb) {
    dali_cmd_t cmd = {.op = is_on ? DALI_CMD_RECALL_LAST_ACTIVE_LEVEL(addr) : DALI_CMD_OFF(addr),
                      .addr = addr,
                      .then = async_report_level_with_fade,
                      .finally = cb,
                      .sendTwice = false,
                      .param = is_on};
    queue_try_add(&dali_queue, &cmd);
}

void dali_set_level(int addr, int level, dali_result_cb_t cb) {
    dali_cmd_t cmd = {.op = addr << 9 | level,
                      .addr = addr,
                      .then = async_report_level_with_fade,
                      .finally = cb,
                      .sendTwice = false,
                      .param = level};
    queue_try_add(&dali_queue, &cmd);
}

// --------- MIN / MAX Register

static void set_max_complete(int res, dali_cmd_t *cmd) {
    if (res >= 0) {
        set_holding_reg_byte(DALI_MINMAX_HR_BASE + cmd->addr, 1, cmd->param);
    }
}

static void set_max_to_dtr0(int res, dali_cmd_t *cmd) {
    if (res >= 0) {
        cmd->op = DALI_CMD_SET_MAX_LEVEL(cmd->addr);
        cmd->sendTwice = true;
        cmd->then = set_max_complete;
    }
}

static void set_min_complete(int res, dali_cmd_t *cmd) {
    if (res >= 0) {
        set_holding_reg_byte(DALI_MINMAX_HR_BASE + cmd->addr, 0, cmd->param & 0xFF);

        cmd->op = DALI_CMD_SET_DTR0(cmd->param >> 8);
        cmd->sendTwice = false;
        cmd->then = set_max_to_dtr0;
    }
}

static void set_min_to_dtr0(int res, dali_cmd_t *cmd) {
    if (res >= 0) {
        cmd->op = DALI_CMD_SET_MIN_LEVEL(cmd->addr);
        cmd->sendTwice = true;
        cmd->then = set_min_complete;
    }
}

void dali_set_min_max_level(int addr, unsigned min, unsigned max, dali_result_cb_t cb) {
    dali_cmd_t cmd = {.op = DALI_CMD_SET_DTR0(min),
                      .addr = addr,
                      .then = set_min_to_dtr0,
                      .finally = cb,
                      .sendTwice = false,
                      .param = min | max << 8};
    queue_try_add(&dali_queue, &cmd);
}

// ----- FADE TIME/RATE Register

static void set_fade_rate_complete(int res, dali_cmd_t *cmd) {
    // Fade time becomes the lower nibble of the LSB of the Holding register.
    if (res >= 0) {
        set_holding_reg_nibble(DALI_FADE_HR_BASE + cmd->addr, 0, cmd->param);
    }
}

static void set_fade_rate_to_dtr0(int res, dali_cmd_t *cmd) {
    if (res >= 0) {
        cmd->op = DALI_CMD_SET_FADE_RATE(cmd->addr);
        cmd->sendTwice = true;
        cmd->then = set_fade_rate_complete;
    }
}

static void set_fade_time_complete(int ret, dali_cmd_t *cmd) {
    // Fade time becomes the upper nibble of the LSB of the Holding register.
    if (ret >= 0) {
        set_holding_reg_nibble(DALI_FADE_HR_BASE + cmd->addr, 1, cmd->param);

        cmd->op = DALI_CMD_SET_DTR0(cmd->param >> 8);
        cmd->sendTwice = false;
        cmd->then = set_fade_rate_to_dtr0;
    }
}

static void set_fade_time_to_dtr0(int res, dali_cmd_t *cmd) {
    if (res >= 0) {
        cmd->op = DALI_CMD_SET_FADE_TIME(cmd->addr);
        cmd->sendTwice = true;
        cmd->then = set_fade_time_complete;
    }
}

void dali_set_fade_time_rate(int addr, unsigned time, unsigned rate, dali_result_cb_t cb) {
    dali_cmd_t cmd = {.op = DALI_CMD_SET_DTR0(time),
                      .addr = addr,
                      .then = set_fade_time_to_dtr0,
                      .finally = cb,
                      .sendTwice = false,
                      .param = time | rate << 8};
    queue_try_add(&dali_queue, &cmd);
}

// -------------- Power on level Register

static void set_system_failure_level_complete(int res, dali_cmd_t *cmd) {
    if (res >= 0) {
        set_holding_reg_byte(DALI_POWERON_HR_BASE + cmd->addr, 1, cmd->param);
    }
}

static void dali_set_system_failure_level_to_dtr0(int res, dali_cmd_t *cmd) {
    if (res >= 0) {
        cmd->op = DALI_CMD_SET_SYSTEM_FAIL_LEVEL(cmd->addr);
        cmd->sendTwice = true;
        cmd->then = set_system_failure_level_complete;
    }
}

static void set__power_on_level_complete(int res, dali_cmd_t *cmd) {
    if (res >= 0) {
        set_holding_reg_byte(DALI_POWERON_HR_BASE + cmd->addr, 0, cmd->param);

        cmd->op = DALI_CMD_SET_DTR0(cmd->param >> 8);
        cmd->sendTwice = false;
        cmd->then = dali_set_system_failure_level_to_dtr0;
    }
}

static void dali_set_power_on_level_to_dtr0(int res, dali_cmd_t *cmd) {
    if (res >= 0) {
        cmd->op = DALI_CMD_SET_POWER_ON_LEVEL(cmd->addr);
        cmd->sendTwice = true;
        cmd->then = set__power_on_level_complete;
    }
}

void dali_set_power_on_level(int addr, int powerOnLevel, int systemFailLevel, dali_result_cb_t cb) {
    dali_cmd_t cmd = {.op = DALI_CMD_SET_DTR0(powerOnLevel),
                      .addr = addr,
                      .then = dali_set_power_on_level_to_dtr0,
                      .finally = cb,
                      .sendTwice = false,
                      .param = powerOnLevel | systemFailLevel << 8};
    queue_try_add(&dali_queue, &cmd);
}

// -------------- Groups Register

static void dali_remove_from_group_completed(int res, dali_cmd_t *cmd) {
    if (res >= 0) {
        clear_holding_reg_bit(DALI_GROUPS_HR_BASE + cmd->addr, cmd->param);
    }
}

void dali_remove_from_group(int addr, int group, dali_result_cb_t cb) {
    dali_cmd_t cmd = {.op = DALI_CMD_REMOVE_FROM_GROUP(addr, group),
                      .addr = addr,
                      .then = dali_remove_from_group_completed,
                      .finally = cb,
                      .sendTwice = true,
                      .param = group};
    queue_try_add(&dali_queue, &cmd);
}

static void dali_add_to_group_completed(int res, dali_cmd_t *cmd) {
    if (res >= 0) {
        set_holding_reg_bit(DALI_GROUPS_HR_BASE + cmd->addr, cmd->param);
    }
}

void dali_add_to_group(int addr, int group, dali_result_cb_t cb) {
    dali_cmd_t cmd = {.op = DALI_CMD_ADD_TO_GROUP(addr, group),
                      .addr = addr,
                      .then = dali_add_to_group_completed,
                      .finally = cb,
                      .sendTwice = true,
                      .param = group};
    queue_try_add(&dali_queue, &cmd);
}

bool dali_enumerate() {
    if (dali_scan_in_progress) {
        return false;
    }
    // First, see if the first address returns a level.  Callbacks will iterate the rest.
    dali_scan_in_progress = true;
    scan_dali_device(0);
    return true;
}

void dali_fade(int addr, int velocity, dali_result_cb_t cb) {
    dali_cmd_t cmd = {.op = velocity > 0 ? DALI_CMD_UP(addr) : DALI_CMD_DOWN(addr),
                      .addr = addr,
                      .then = async_report_level_with_fade,
                      .finally = cb,
                      .sendTwice = false,
                      .param = 0};
    queue_try_add(&dali_queue, &cmd);
}

void dali_poll() {
    if (in_flight.then) {
        if (!pio_sm_is_rx_fifo_empty(pio, dali_sm)) {
            uint32_t v = pio_sm_get(pio, dali_sm);
            int res = v == 0xFFFFFFFF ? -1 : v & 0xFF;

            // We treat a NAK as a valid response for the purposes of repeating ourselves.  This is especially important as
            // almost every command that requires retransmission always returns NAK
            if (res >= DALI_NAK && in_flight.sendTwice) {
                in_flight.sendTwice = false;
                // We need to repeat the command again.
                send_dali_cmd(in_flight.op);
            } else {
                cmd_chain_cb_t next_action = in_flight.then;
                in_flight.then = NULL;
                next_action(res, &in_flight);
                // If the callback explicitly sets a new *then* callback we willtransmit its operation immediately, otherwise we
                // will assume that that transaction is done.
                if (in_flight.then) {
                    // The callback has set a followup command, so send it out.
                    if (in_flight.sendTwice == 0) {
                        in_flight.sendTwice = false;
                    }
                    send_dali_cmd(in_flight.op);
                } else {
                    // Command is Completely done.  Call Finally handler.
                    if (in_flight.finally) {
                        in_flight.finally(res);
                    }
                }
            }
        }
    } else if (queue_try_remove(&dali_queue, &in_flight)) {
        pio_sm_restart(pio, dali_sm);
        send_dali_cmd(in_flight.op);
    }
}

void dali_init(uint32_t tx_pin, uint32_t rx_pin) {
    for (int i = 0; i < 64; i++) {
        set_holding_reg(DALI_STATUS_HR_BASE + i, 0xFFFF);
        set_holding_reg(DALI_MINMAX_HR_BASE + i, 0xFFFF);
    }
    // An enumeration will use 64 entries in the queue, so we give it some space.
    queue_init(&dali_queue, sizeof(dali_cmd_t), QUEUE_DEPTH);

    uint offset = pio_add_program(pio, &dali_tx_program);

    // Tell PIO to initially drive output-low on the selected pin, then map PIO
    // onto that pin with the IO muxes.
    pio_gpio_init(pio, tx_pin);
    pio_gpio_init(pio, rx_pin);
    pio_sm_set_consecutive_pindirs(pio, dali_sm, rx_pin, 1, false);
    pio_sm_set_consecutive_pindirs(pio, dali_sm, tx_pin, 1, true);

    pio_sm_config c = dali_tx_program_get_default_config(offset);

    sm_config_set_out_shift(&c, false, true, 32);
    sm_config_set_in_shift(&c, false, false, 32);
    sm_config_set_out_pins(&c, tx_pin, 1);
    sm_config_set_set_pins(&c, tx_pin, 1);
    sm_config_set_in_pins(&c, rx_pin);
    sm_config_set_jmp_pin(&c, rx_pin);

    // SM transmits 1 half bit per 8 execution cycles.
    float div = (float)clock_get_hz(clk_sys) / (2 * 8 * 1200);
    sm_config_set_clkdiv(&c, div);

    pio_sm_init(pio, dali_sm, offset, &c);
    pio_sm_set_enabled(pio, dali_sm, true);

    dali_enumerate();
}
