#include <hardware/clocks.h>
#include <pico/stdlib.h>
#include "dali.h"
#include "stdbool.h"
#include "stdint.h"
#include "dali.pio.h"
#include <string.h>
#include <stdio.h>
#include "queue.h"

#define DALI_ADDR_FROM_CMD(cmd) ((cmd >> 9) & 0x3F)

#define DALI_CMD_QUERY_STATUS(addr) (((addr) << 9) | 0x190)
#define DALI_CMD_QUERY_DEVICE_TYPE(addr) (((addr) << 9) | 0x199)
#define DALI_CMD_QUERY_ACTUAL_LEVEL(addr) (((addr) << 9) | 0x1a0)
#define DALI_CMD_QUERY_MAX(addr) (((addr) << 9) | 0x1a1)
#define DALI_CMD_QUERY_MIN(addr) (((addr) << 9) | 0x1a2)
#define DALI_CMD_QUERY_POWER_ON_LEVEL(addr) (((addr) << 9) | 0x1a3)
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

struct dali_cmd_t;
typedef void (*cmd_complete_cb_t)(int res, struct dali_cmd_t *cb);

typedef struct dali_cmd_t
{
    uint16_t op;
    cmd_complete_cb_t then;
} dali_cmd_t;

#define QUEUE_DEPTH 10
static dali_cmd_t queue_data[QUEUE_DEPTH];
static queue_t cmd_queue;
static dali_cmd_t in_flight = {
    .op = 0,
    .then = NULL,
};
// During an enumerate, we need to collate several values at once - We do that here.
// Due to the fact that all values are done during one "in_flight" tx, we don't need to worry about re-entrant behavior.
static int enumerate_min;

const char *dali_entity_prefix = "dali";

// ------------------------- in-flight action callbacks ----------------

static void report_level_with_fade(int _unused, dali_cmd_t *cmd);
static void enumerate_got_min(int min, dali_cmd_t *cmd);

static void enumerate_addr(int addr)
{
    // This is a new enqueue, rather than a continuation, because we want it possible for other commands to execute interleaved.
    dali_cmd_t cmd = {
        .op = DALI_CMD_QUERY_MIN(addr),
        .then = enumerate_got_min,
    };
    queue_add(&cmd_queue, &cmd);
}

static void enumerate_got_max(int max, dali_cmd_t *cmd)
{
    int addr = DALI_ADDR_FROM_CMD(cmd->op);
    printf("\r\tdevice %s%d name=\"DALI light fixture %d\"\n", dali_entity_prefix, addr, addr);
    printf("\r\tlight %s%d device=%s%d", dali_entity_prefix, addr, dali_entity_prefix, addr);
    // If device is dimmable, add additional parameters to discovery line
    if (enumerate_min != max)
    {
        printf(" brightness=true supported_color_modes=brightness brightness_scale=254 min=%d max=%d", enumerate_min, max);
    }
    printf("\n");
    // Seperately enqueue a command to report the level, testing to see if it has faded.
    // We do this because, although unlikely, the device might have been mid-fade during the enumerate.
    report_level_with_fade(0, cmd);
    if (addr < DALI_MAX_ADDR)
    {
        // Start a new task to enumerate the next address.
        enumerate_addr(addr + 1);
    }
}

static void enumerate_got_min(int min, dali_cmd_t *cmd)
{

    int addr = DALI_ADDR_FROM_CMD(cmd->op);
    if (min >= 0)
    {
        enumerate_min = min;
        cmd->op = DALI_CMD_QUERY_MIN(DALI_ADDR_FROM_CMD(cmd->op));
        cmd->then = enumerate_got_max;
    }
    else if (addr < DALI_MAX_ADDR)
    {
        // There was no reply from the device.  It probably doesn't exist.
        // TODO consider retries...
        // Start a new task to enumerate the next address.
        printf("No device responded for address %d\n", addr + 1);
        enumerate_addr(addr + 1);
    }
}

static void fade_received_status(int status, dali_cmd_t *cmd)
{
    int addr = DALI_ADDR_FROM_CMD(cmd->op);
    if (status & DALI_STATUS_FADE_IN_PROGRESS)
    {
        // Fade still active.  Enqueue another one
        report_level_with_fade(status, cmd);
    }
}

static void fade_received_level(int lvl, dali_cmd_t *cmd)
{
    int addr = DALI_ADDR_FROM_CMD(cmd->op);
    printf("\r\t%s%d state=%s brightness=%d\n", dali_entity_prefix, addr, lvl > 0 ? "on" : "off", lvl);

    cmd->op = DALI_CMD_QUERY_STATUS(addr);
    cmd->then = fade_received_status;
}

static void report_level_with_fade(int _unused, dali_cmd_t *cmd)
{
    int addr = DALI_ADDR_FROM_CMD(cmd->op);
    // We don't want other commands to be stalled waiting for a fade to conclude, so we push a new command rather than update the existing one.
    dali_cmd_t newcmd = {
        .op = DALI_CMD_QUERY_ACTUAL_LEVEL(addr),
        .then = fade_received_level,
    };
    queue_add(&cmd_queue, &newcmd);
}

static void toggle_action(int lvl, dali_cmd_t *cmd)
{
    // Follow through from level check to send out the on or off command.
    int addr = DALI_ADDR_FROM_CMD(cmd->op);
    cmd->op = lvl > 0 ? DALI_CMD_OFF(addr) : DALI_CMD_RECALL_LAST_ACTIVE_LEVEL(addr);
    cmd->then = report_level_with_fade;
}

// ----------------------------- API -------------------------

void dali_init(uint32_t tx_pin, uint32_t rx_pin)
{
    printf("DALI TX pin %d, RX pin %d\n", tx_pin, rx_pin);
    // An enumeration will use 64 entries in the queue, so we give it some space.
    queue_init(&cmd_queue, queue_data, sizeof(dali_cmd_t), QUEUE_DEPTH);

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
}

static inline void send_dali_cmd(uint cmd)
{
    // This says blocking, but it is very unlikely that it will ever block, due to the serial nature of how commands are executed.
    pio_sm_put_blocking(pio, dali_sm, ((cmd << 15) | 0x80000000) + 88);
}

void dali_poll()
{
    if (in_flight.then)
    {
        if (!pio_sm_is_rx_fifo_empty(pio, dali_sm))
        {
            uint32_t v = pio_sm_get(pio, dali_sm);
            cmd_complete_cb_t cb = in_flight.then;
            // If the callback explicitly sets a new *op* and *then*, then we will transmit it, otherwise we will assume that that transaction is done.
            in_flight.then = NULL;
            int res = v == 0xFFFFFFFF ? -1 : v & 0xFF;
            cb(res, &in_flight);
            if (in_flight.then)
            {
                // The callback has set a followup command, so send it out.
                send_dali_cmd(in_flight.op);
            }
        }
    }
    else if (queue_get(&cmd_queue, &in_flight))
    {
        send_dali_cmd(in_flight.op);
    }
}

void dali_toggle(int addr)
{
    dali_cmd_t cmd = {
        .op = DALI_CMD_QUERY_ACTUAL_LEVEL(addr),
        .then = toggle_action,
    };
    queue_add(&cmd_queue, &cmd);
}

void dali_set_on(int addr, bool is_on)
{
    dali_cmd_t cmd = {
        .op = is_on ? DALI_CMD_RECALL_LAST_ACTIVE_LEVEL(addr) : DALI_CMD_OFF(addr),
        .then = report_level_with_fade,
    };
    queue_add(&cmd_queue, &cmd);
}

void dali_set_level(int addr, int level)
{
    dali_cmd_t cmd = {
        .op = addr << 9 | level,
        .then = report_level_with_fade,
    };
    queue_add(&cmd_queue, &cmd);
}

void dali_enumerate()
{
    // First, see if the first address returns a level.  Callbacks will iterate the rest.
    printf("Enumerating all DALI devices\n");
    enumerate_addr(0);
}

void dali_fade(int addr, int velocity)
{
    dali_cmd_t cmd = {
        .op = velocity > 0 ? DALI_CMD_UP(addr) : DALI_CMD_DOWN(addr),
        .then = report_level_with_fade,
    };
    queue_add(&cmd_queue, &cmd);
}
