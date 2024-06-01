

#include <pico/stdlib.h>
#include <hardware/flash.h>
#include <hardware/regs/addressmap.h>
#include <pico/sync.h>
#include <string.h>
#include <stdio.h>
#include "buttons.h"
#include "dali.h"
#include "modbus.h"
#include "cli.h"

#define BUTTON_SER_PIN 6
#define BUTTON_CLK_PIN 7
#define ROW_BASE_PIN 8

#define MS_TO_COUNTDOWN(ms) (ms / 5)

// Mask for the bottom 6 bits, which should be the address on the bus.

// We are assuming 2MB of flash, and we use the last sector (4Kb).  The first two words will be magic values, then the rest will be bindings
#define FLASH_CONFIG_OFFSET ((2 * 1024 * 1024) - FLASH_SECTOR_SIZE)
static const uint8_t *bindings = (const uint8_t *)(XIP_BASE + FLASH_CONFIG_OFFSET);

// Trying to use as little memory as possible (when we transition to CH559), we cram stuff together - 2 bytes per switch x 168 is pretty good.
typedef struct
{
    bool released;
    int velocity;
    uint countdown;
} button_ctx_t;

static button_ctx_t button_ctx[NUM_FIXTURES * NUM_BUTTONS_PER_FIXTURE];
static int fixture = 0;
static button_ctx_t *ctx = button_ctx;
static absolute_time_t next_button_scan;

static void persist_bindings(uint8_t *sector)
{
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(FLASH_CONFIG_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(FLASH_CONFIG_OFFSET, sector, FLASH_PAGE_SIZE);
    restore_interrupts(ints);
}

/**
 * Tests that the binding config stored in flash is valid - if not, it erases it.
 */
static void test_flash_config()
{
    static const char binding_magicvals[8] = "MechCFG1";

    if (memcmp(bindings + FLASH_PAGE_SIZE - 8, binding_magicvals, sizeof(binding_magicvals)) != 0)
    {
        // log_i("Flash is not initialised - Blanking it out");
        uint8_t sector[FLASH_PAGE_SIZE];
        memcpy(sector + FLASH_PAGE_SIZE - sizeof(binding_magicvals), binding_magicvals, sizeof(binding_magicvals));
        memset(sector, 0xFF, FLASH_PAGE_SIZE - sizeof(binding_magicvals));
        persist_bindings(sector);
    }
}

static void print_binding_status(int fixture, int button)
{
    log_msg_t msg = {
        .type = STATUS_PRINT_VALUES,
        .bus = MSG_SRC_BUTTON_FIXTURE,
        .device = fixture,
        .address = button,
        .vals = bindings[fixture * NUM_BUTTONS_PER_FIXTURE + button],
    };

    print_msg(&msg);
}

void set_and_persist_binding(uint fixture, uint button, uint8_t encoded_binding)
{
    assert(fixture < NUM_FIXTURES && button < NUM_BUTTONS_PER_FIXTURE);
    uint8_t sector[FLASH_PAGE_SIZE];
    memcpy(sector, bindings, FLASH_PAGE_SIZE);
    sector[fixture * NUM_BUTTONS_PER_FIXTURE + button] = encoded_binding;
    persist_bindings(sector);
    print_binding_status(fixture, button);
}

static void log_button_evt(button_ctx_t *ctx, char *evt)
{
    int idx = ctx - button_ctx;
}

/**
 * Processes the repeat handling of a button
 *
 * This task will be deleted when the button is released, so it should take care not to allocate any memory,
 * as it won't be afforded an opportunity to clean up.
 */
static void button_pressed(button_ctx_t *ctx)
{
    log_button_evt(ctx, "pressed");
    ctx->velocity = -ctx->velocity; // Switch direction
    ctx->countdown = ctx->velocity ? MS_TO_COUNTDOWN(250) : MS_TO_COUNTDOWN(750);
}

static void button_timeout_check(button_ctx_t *ctx)
{
    char tmpbuf[16];
    if (ctx->countdown && --(ctx->countdown) == 0)
    {
        if (ctx->released)
        {
            ctx->velocity = 0;
        }
        else
        {
            ctx->countdown = MS_TO_COUNTDOWN(250);
            if (ctx->velocity == 0)
            {
                // Set it to dimming downwards - Clear the DIMDIR bit and set the DIMMING bit
                ctx->velocity = -1;
            }

            log_button_evt(ctx, ctx->velocity > 0 ? "brighten" : "dim");
            int i = ctx - button_ctx;
            switch (bindings[i] >> 6)
            {
            case BINDING_TYPE_DALI:
                dali_fade(bindings[i] & BUS_ADDRESS_MASK, ctx->velocity);
                break;
            default:
                // log_i("binding %s is not dimmable", binding_tostr(bindings[i], tmpbuf));
                break;
            }
        }
    }
}

static void button_released(button_ctx_t *ctx)
{
    int i = ctx - button_ctx;

    if (ctx->velocity == 0)
    {
        log_button_evt(ctx, "clicked");

        switch (bindings[i] >> 6)
        {
        case BINDING_TYPE_DALI:
            // log_i("Toggling Dali");
            dali_toggle(bindings[i] & BUS_ADDRESS_MASK);
            break;
        case BINDING_TYPE_MODBUS:
            // log_i("Toggling Modbus");
            modbus_set_coil(1, bindings[i] & BUS_ADDRESS_MASK, 2);
            break;
        default:
            log_button_evt(ctx, "not bound");
            break;
        }
    }
    else
    {
        log_button_evt(ctx, "released");
        ctx->countdown = ctx->velocity ? MS_TO_COUNTDOWN(500) : 0;
    }
}

void buttons_enumerate()
{
    log_msg_t msg = {
        .type = STATUS_PRINT_DEVICE,
        .bus = MSG_SRC_BUTTON_FIXTURE,
    };

    int i = 0;

    for (int fixture = 0; fixture < NUM_FIXTURES; fixture++)
    {
        msg.device = fixture;
        for (int button = 0; button < NUM_BUTTONS_PER_FIXTURE; button++)
        {
            msg.address = button;

            print_msg(&msg);
            print_binding_status(fixture, button);
        }
    }
}

void buttons_init()
{
    test_flash_config();

    for (int i = 0; i < NUM_BUTTONS_PER_FIXTURE; i++)
    {
        gpio_init(ROW_BASE_PIN + i);
        gpio_set_dir(ROW_BASE_PIN + i, false);
        gpio_set_pulls(ROW_BASE_PIN + i, true, false);
    }

    gpio_init(BUTTON_SER_PIN);
    gpio_set_dir(BUTTON_SER_PIN, true);
    gpio_init(BUTTON_CLK_PIN);
    gpio_set_dir(BUTTON_CLK_PIN, true);

    gpio_put(BUTTON_CLK_PIN, 0);

    // Set up all of the fixture and button data structures
    button_ctx_t *ctx = button_ctx;
    for (int fixture_idx = 0; fixture_idx < NUM_FIXTURES; fixture_idx++)
    {
        for (int btn_idx = 0; btn_idx < NUM_BUTTONS_PER_FIXTURE; btn_idx++)
        {
            ctx->released = true; // Default to released, not dimming
            ctx->velocity = 0;
            ctx->countdown = 0;
            ctx++;
        }

        // While we go, also clear out the first set of readings from the button matrix.
        // The flip flops in the shift registers start in an unknown state, so we can't trust
        // any readings uptil we've done a full loop.
        gpio_put(BUTTON_SER_PIN, fixture_idx < (NUM_FIXTURES - 1));
        sleep_us(2);
        gpio_put(BUTTON_CLK_PIN, 1);
        sleep_us(2);
        gpio_put(BUTTON_CLK_PIN, 0);
    }
    next_button_scan = get_absolute_time();
}

void buttons_poll()
{
    uint32_t val;

    // Wait for our next iteration time - We want to read every 5 milliseconds.
    if (time_reached(next_button_scan))
    {
        next_button_scan = delayed_by_us(next_button_scan, (5000 / NUM_FIXTURES));

        // next_iter = delayed_by_us(next_iter, BUTTON_SAMPLE_PERIOD);
        // First, clock out the data for the _next_ iteration - 0 for the last one and 1 for all others
        gpio_put(BUTTON_SER_PIN, fixture == (NUM_FIXTURES - 1) ? 0 : 1);
        sleep_us(1);                 // We need to delay between setting data and clocking so that it will be guaranteed to be correct at the chip
        gpio_put(BUTTON_CLK_PIN, 1); // Clock out the data we just set.
        sleep_us(20);                // Give the inputs a moment to settle into their correct values.

        // Now read in the data for one fixture.
        val = (gpio_get_all() >> ROW_BASE_PIN);
        // Turn the changes from a bit field into indexes for more easy consumption.
        for (int button = 0; button < NUM_BUTTONS_PER_FIXTURE; button++)
        {
            int rel = val & 0x01;
            if (ctx->released != rel)
            {
                ctx->released = rel;
                // Cancel any running task.
                if (ctx->released)
                {
                    button_released(ctx);
                }
                else
                {
                    button_pressed(ctx);
                }
            }
            else
            {
                // See if a button timer has expired.
                button_timeout_check(ctx);
            }
            val >>= 1;
            ctx++;
        }

        // Drive the clock back down low - This forms a handy indicator of how much time in each iteration we've consumed.
        // The clock will be high for the settle time plus procesisng time.
        gpio_put(BUTTON_CLK_PIN, 0);
        if (fixture == NUM_FIXTURES - 1)
        {
            fixture = 0;
            ctx = button_ctx;
        }
        else
        {
            fixture++;
        }
    }
}
