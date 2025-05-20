#include "modbus.h"

#include <hardware/clocks.h>
#include <hardware/irq.h>
#include <hardware/structs/timer.h>
#include <hardware/timer.h>
#include <pico/platform.h>
#include <pico/stdio.h>
#include <pico/stdlib.h>
#include <pico/util/queue.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "crcbuf.h"
#include "modbus.pio.h"
#include "regs.h"
#include "stdbool.h"
#include "stdint.h"

#define MODBUS_BAUD_RATE 9600
#define QUEUE_DEPTH 10

static queue_t task_queue;
static modbus_task_state_t current_task_state = MODBUS_TASK_STATE_IDLE;
static uint8_t response[256];
static uint16_t response_crc;
static uint8_t *response_ptr;
static size_t response_sz;

typedef struct modbus_task_t {
    uint8_t *cmd;
    size_t sz;
    modbus_task_cb callback;
} modbus_task_t;

static modbus_task_t current_task;
static absolute_time_t timeout;

static const PIO pio = pio1;
static const unsigned int tx_sm = 0;
static const unsigned int rx_sm = 1;

static const char *TAG = "MODBUS";

bool modbus_downstream_task_enqueue(uint8_t *cmd, size_t sz, modbus_task_cb callback) {
    modbus_task_t t = {
        .callback = callback,
        .cmd = cmd,
        .sz = sz,
    };
    return queue_try_add(&task_queue, &t);
}

#define container_of(ptr, type, member)                    \
    ({                                                     \
        const typeof(((type *)0)->member) *__mptr = (ptr); \
        (type *)((char *)__mptr - offsetof(type, member)); \
    })

#define LED_PIN 25  // Onboard LED pin for the Pico

void toggleLED() {
    gpio_put(LED_PIN, !gpio_get(LED_PIN));
}

void onError() {
    // Make the LED blink on error
    while (true) {
        toggleLED();
        sleep_ms(1000);
    }
}

/**
 * This is the only direct action the Device takes on modbus itself (when a
 * button is pressed, if a binding is set)
 */
void modbus_downstream_set_coil(uint8_t devaddr, uint16_t coil_num, uint16_t value, modbus_task_cb cb) {
    // Allocate enough space for the command and its buffer.
    uint8_t *cmd = (uint8_t *)malloc(8);
    if (cmd == NULL) {
        onError();
    }
    uint16_t crc = 0xFFFF;
    uint8_t *ptr = cmd;

    crc_append(ptr++, devaddr, &crc);
    crc_append(ptr++, MODBUS_CMD_WRITE_SINGLE_COIL, &crc);
    crc_append(ptr++, coil_num >> 8, &crc);
    crc_append(ptr++, coil_num, &crc);
    crc_append(ptr++, value >> 8, &crc);  // 0x55 = toggle.  0xFF = on, 0x00 = off.
    crc_append(ptr++, value, &crc);
    *ptr++ = crc & 0xFF;
    *ptr++ = crc >> 8;
    modbus_downstream_task_enqueue(cmd, 8, cb);
}

void modbus_downstream_set_coils(uint8_t devaddr, uint16_t coil_num, uint16_t count, uint8_t *value, modbus_task_cb cb) {
    // Allocate enough space for the command and its buffer.
    int byte_count = 8 + count / 8;
    if (count % 8) {
        byte_count++;
    }
    uint8_t *cmd = (uint8_t *)malloc(byte_count);
    if (cmd == NULL) {
        onError();
    }
    uint16_t crc = 0xFFFF;
    uint8_t *ptr = cmd;

    crc_append(ptr++, devaddr, &crc);
    crc_append(ptr++, MODBUS_CMD_WRITE_MULTIPLE_COILS, &crc);
    crc_append(ptr++, 00, &crc);
    crc_append(ptr++, coil_num, &crc);
    crc_append(ptr++, 0x55, &crc);  // 0x55 = toggle.  0xFF = on, 0x00 = off.
    crc_append(ptr++, 0x00, &crc);
    *ptr++ = crc & 0xFF;
    *ptr++ = crc >> 8;
    modbus_downstream_task_enqueue(cmd, byte_count, cb);
}


/**
 * This is the only direct action the Device takes on modbus itself (when a
 * button is pressed, if a binding is set)
 */
void modbus_downstream_get_coils() {
    // Allocate enough space for the command and its buffer.
    uint8_t *cmd = (uint8_t *)malloc(8);
    if (cmd == NULL) {
        onError();
    }
    uint16_t crc = 0xFFFF;
    uint8_t *ptr = cmd;

    crc_append(ptr++, 1, &crc);
    crc_append(ptr++, MODBUS_CMD_READ_COILS, &crc);
    crc_append(ptr++, 0, &crc);  // Address
    crc_append(ptr++, 0, &crc);
    crc_append(ptr++, 0, &crc);  // Number of coils
    crc_append(ptr++, 32, &crc);
    *ptr++ = crc & 0xFF;
    *ptr++ = crc >> 8;
    modbus_downstream_task_enqueue(cmd, 8, NULL);
}

/**
 * Returns true if we have a complete packet read
 */
int modbus_expected_response_length(uint8_t *buf, size_t sz) {
    int expected_len = 0;

    if (sz > 2) {
        modbus_cmd_t cmd = buf[1];

        if (cmd >= 0x80) {
            // Its an exception, and therefore must be 5 bytes long.
            expected_len = 5;
        } else {
            switch (cmd) {
                case MODBUS_CMD_READ_COILS:
                case MODBUS_CMD_READ_DISCRETE_INPUTS:
                case MODBUS_CMD_READ_HOLDING_REGISTERS:
                case MODBUS_CMD_READ_INPUT_REGISTERS:
                    if (sz >= 3) {
                        expected_len = 5 + buf[2];
                    }
                    break;
                case MODBUS_CMD_CUSTOM_EXEC_DALI:
                    expected_len = 5;
                    break;
                case MODBUS_CMD_WRITE_SINGLE_COIL:
                case MODBUS_CMD_WRITE_SINGLE_REGISTER:
                case MODBUS_CMD_WRITE_MULTIPLE_COILS:
                case MODBUS_CMD_WRITE_MULTIPLE_REGISTERS:
                    expected_len = 8;
                    break;
            }
        }
    }
    if (expected_len && response_sz >= expected_len) {
        return expected_len;
    }
    return -expected_len;
}

void reflect_command_success_to_regs(uint8_t *cmd) {
    uint8_t function = cmd[1];
    uint16_t addr, value;
    uint32_t coils;

    switch (function) {
        case MODBUS_CMD_WRITE_SINGLE_COIL:
            addr = (cmd[2] << 8) | cmd[3];
            value = cmd[4];  // We only care about the high byte
            if (addr < MAX_COILS) {
                switch (value) {
                    case 0:
                        clear_coil_reg(addr);
                        break;
                    case 0xFF:
                        set_coil_reg(addr);
                        break;
                    case 0x55:
                        toggle_coil_reg(addr);
                        break;
                    default:
                        break;
                }
            }
            break;
        case MODBUS_CMD_READ_COILS:
            coils = response[6] | (response[5] << 8) | (response[4] << 16) | (response[3] << 24);
            unsigned bit = 0x01;
            for (int i = 0; i < 32; i++, bit <<= 1) {
                if (coils & bit) {
                    set_coil_reg(i);
                } else {
                    clear_coil_reg(i);
                }
            }
            break;
    }
}

void modbus_poll() {
    int expected;
    int data_read;

    switch (current_task_state) {
        case MODBUS_TASK_STATE_IDLE:
            // No active task.  Check the queue
            if (!queue_try_remove(&task_queue, &current_task)) {
                break;
            }
            current_task_state = MODBUS_TASK_STATE_PENDING;
            // The task _should_ be in the pending state.  Fall through.

        case MODBUS_TASK_STATE_PENDING:
            // We are starting a new task (presumably that has just been de-queued)
            pio_sm_restart(pio, tx_sm);
            // Reset read buffer for read, just in case it wasn't already.

            modbus_tx_program_putbuf(pio, tx_sm, current_task.cmd, current_task.sz);
            current_task_state = MODBUS_TASK_STATE_AWAITING_RESPONSE;
            response_crc = 0xFFFF;
            response_sz = 0;
            response_ptr = response;

            // We want the receive fifo to be completely empty - it should be, but its
            // always good to be sure
            pio_sm_restart(pio, rx_sm);

            // We expect to see a response in a handful of ms, but give it a bit
            // longer, just in case.
            timeout = make_timeout_time_ms(100);
            // We could fall through here, but there can't be any bytes in the receive buffer yet, so there isn't much point.
            break;

        case MODBUS_TASK_STATE_AWAITING_RESPONSE:
            // Check to see if we have enough response bytes.
            // If we do, this call will return a positive number.
            expected = modbus_expected_response_length(response, response_sz);

            if (expected > 0) {
                // We have a valid response packet.  Check its CRC.
                if (response_crc == 0) {
                    reflect_command_success_to_regs(current_task.cmd);
                    current_task_state = MODBUS_TASK_STATE_DONE;
                } else {
                    // Invalid CRC.
                    current_task_state = MODBUS_TASK_STATE_INVALID_CRC;
                }
            } else if (time_reached(timeout)) {
                // defer_log(TAG, "Timeout waiting for Modbus response");
                current_task_state = MODBUS_TASK_STATE_TIMEOUT;
            } else {
                // Not enough data yet. Try to read a character from the RS485 port
                if ((data_read = modbus_rx_program_getc(pio, rx_sm)) >= 0) {
                    *response_ptr++ = data_read;
                    crc_update(data_read, &response_crc);
                    response_sz++;
                    // printf("L Read %02X - Expect %d of %d - CRC %d\n", data_read, expected, response_sz, response_crc);
                }
            }
            // If we changed to a terminal state, call the callback
            if (current_task_state != MODBUS_TASK_STATE_AWAITING_RESPONSE) {
                // Task was completed, but we need to wait for the inter-command delay.
                timeout = make_timeout_time_us(1750);
                // In the mean time, call the task callback if defined
                if (current_task.callback) {
                    current_task.callback(current_task_state, current_task.cmd, response, response_sz);
                }
                // We always use dynamically allocated commands, so free it up again. 
                free(current_task.cmd);
            }
            break;

        default:
            // The task has completed, but we need to wait for the mandatory 1.75ms
            // break between commands before transmitting another one.
            if (time_reached(timeout)) {
                // Done!  Let the loop fetch a new task on its next poll.
                current_task_state = MODBUS_TASK_STATE_IDLE;
            }
            break;
    }
}

void modbus_init(int tx_pin, int rx_pin, int de_pin) {
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);


    queue_init(&task_queue, sizeof(modbus_task_t), QUEUE_DEPTH);

    uint offset = pio_add_program(pio, &modbus_tx_program);
    modbus_tx_program_init(pio, tx_sm, offset, tx_pin, de_pin, MODBUS_BAUD_RATE);
    offset = pio_add_program(pio, &modbus_rx_program);
    modbus_rx_program_init(pio, rx_sm, offset, rx_pin, MODBUS_BAUD_RATE);

    // Start with empty coils values.
    for (int i = 0; i < MAX_COILS; i++) {
        clear_coil_reg(i);
    }

    // The RS485 chip seems to need a little bit of start up time before it can
    // receive commands after reboot. We put a sleep in here as a cheap way of
    // ensuring that.
    sleep_ms(100);
    modbus_downstream_get_coils();
}