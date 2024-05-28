#include <hardware/clocks.h>
#include "dali.h"
#include "stdbool.h"
#include "stdint.h"
#include "dali.pio.h"
#include <string.h>
#include <stdio.h>
#include "async.h"

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

static const PIO pio = pio0;
static const unsigned int dali_sm = 0;

static volatile int bus_lock = 0;

static inline int dali_send_cmd(uint16_t cmd)
{
    if (pio_sm_is_tx_fifo_full(pio, dali_sm))
    {
        return 0;
    }
    uint32_t v = ((cmd << 15) | 0x80000000) + 88;

    pio_sm_put(pio, dali_sm, v);
    return 1;
}

static inline int dali_get_response(int *out)
{
    if (pio_sm_is_rx_fifo_empty(pio, dali_sm))
    {
        return 0;
    }
    int v = pio_sm_get(pio, dali_sm);
    if (out)
    {
        *out = v == 0xFFFFFFFF ? -1 : v & 0xFF;
    }
    return 1;
}

static inline void report_status(int addr, int level)
{
    printf("\r\tdali%d state=%s brightness=%d\n", addr, level > 0 ? "on" : "off", level);
}

static inline bool acquire_lock()
{
    if (bus_lock == 0)
    {
        bus_lock = 1;
        return true;
    }
    return false;
}

static inline bool release_lock()
{
    bus_lock = 0;
}

static void process_dali_cmd_and_report_task(async_ctx_t *ctx)
{
    int level;

    async_begin();
    await(acquire_lock());
    await(dali_send_cmd(ctx->idata));
    await(dali_get_response(NULL));

    // Level was insta-applied (we hope), so we can simply report once.
    await(dali_send_cmd(DALI_CMD_QUERY_ACTUAL_LEVEL(DALI_ADDR_FROM_CMD(ctx->idata))));
    await(dali_get_response(&level));
    report_status(DALI_ADDR_FROM_CMD(ctx->idata), level);

    release_lock();
    async_end();
}

static void process_dali_cmd_and_report_fade_task(async_ctx_t *ctx)
{
    // Used to store results. Remember that it will be reset each time the function yeilds
    async_begin();
    await(acquire_lock());

    await(dali_send_cmd(ctx->idata));
    await(dali_get_response(NULL));
    release_lock();
    async_yield();

    // We extract the original address from the command, and use that in the subsequent status checks.
    ctx->idata = DALI_ADDR_FROM_CMD(ctx->idata);
    do
    {
        await(acquire_lock());
        await(dali_send_cmd(DALI_CMD_QUERY_ACTUAL_LEVEL(ctx->idata)));
        await(dali_get_response(&ctx->scratch));
        report_status(ctx->idata, ctx->scratch);

        await(dali_send_cmd(DALI_CMD_QUERY_STATUS(ctx->idata)));
        await(dali_get_response(&ctx->scratch));
        release_lock();
        async_yield();
    } while (ctx->scratch > 0 && ctx->scratch & DALI_STATUS_FADE_IN_PROGRESS);
    async_end();
}

void dali_enumerate_task(async_ctx_t *ctx)
{
    // Cheat's way of getting long lived results - Don't start two enumerations in parallel
    static int level, min, max;

    async_begin();

    for (ctx->idata = 0; ctx->idata < 64; ctx->idata++)
    {
        await(acquire_lock());
        await(dali_send_cmd(DALI_CMD_QUERY_ACTUAL_LEVEL(ctx->idata)));
        await(dali_get_response(&level));
        if (level >= 0)
        {
            await(dali_send_cmd(DALI_CMD_QUERY_MIN(ctx->idata)));
            await(dali_get_response(&min));
            await(dali_send_cmd(DALI_CMD_QUERY_MAX(ctx->idata)));
            await(dali_get_response(&max));
            printf("\r\tdevice dali%d name=\"DALI light fixture %d\"\n", ctx->idata, ctx->idata);

            printf("\r\tlight dali%d device=dali%d", ctx->idata, ctx->idata);
            // If device is dimmable, add additional parameters to discovery line
            if (min != max)
            {
                printf(" brightness=true supported_color_modes=brightness brightness_scale=254 min=%d max=%d", min, max);
            }
            printf("\n");
            report_status(ctx->idata, level);
        }
        release_lock();
        async_yield(); // Give other async tasks that want access to the bus a go. Without this, we would simply re-lock immediately.
    }
    async_end();
}

void dali_set_level(int addr, int level)
{
    async_start_task(process_dali_cmd_and_report_fade_task, addr << 9 | level);
}

static void toggle_task(async_ctx_t *ctx)
{
    int val;

    async_begin();
    await(acquire_lock());

    await(dali_send_cmd(DALI_CMD_QUERY_ACTUAL_LEVEL(ctx->idata)));
    await(dali_get_response(&val));

    if (val)
    {
        await(dali_send_cmd(DALI_CMD_OFF(ctx->idata)));
        await(dali_get_response(&val));
        await(dali_send_cmd(DALI_CMD_QUERY_ACTUAL_LEVEL(ctx->idata)));
        await(dali_get_response(&val));
        report_status(ctx->idata, val);
    }
    else
    {
        await(dali_send_cmd(DALI_CMD_RECALL_LAST_ACTIVE_LEVEL(ctx->idata)));
        await(dali_get_response(&val));

        val = DALI_STATUS_FADE_IN_PROGRESS;
        while (val > 0 && val & DALI_STATUS_FADE_IN_PROGRESS)
        {
            await(dali_send_cmd(DALI_CMD_QUERY_ACTUAL_LEVEL(ctx->idata)));
            await(dali_get_response(&val));
            report_status(DALI_ADDR_FROM_CMD(ctx->idata), val);
            await(dali_send_cmd(DALI_CMD_QUERY_STATUS(ctx->idata)));
            await(dali_get_response(&val));
        }
    }

    release_lock();
    async_end();
}

void dali_set_on(int addr, bool is_on)
{
    if (is_on)
    {
        async_start_task(process_dali_cmd_and_report_fade_task, DALI_CMD_RECALL_LAST_ACTIVE_LEVEL(addr));
    }
    else
    {
        async_start_task(process_dali_cmd_and_report_task, DALI_CMD_OFF(addr));
    }
}

void dali_fade(int addr, int velocity)
{
    async_start_task(process_dali_cmd_and_report_fade_task, velocity > 0 ? DALI_CMD_UP(addr) : DALI_CMD_DOWN(addr));
}

void dali_toggle(int addr)
{
    async_start_task(toggle_task, addr);
}

void dali_init(uint32_t tx_pin, uint32_t rx_pin)
{
    printf("DALI TX pin %d, RX pin %d\n", tx_pin, rx_pin);
    // Set up DALI program
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