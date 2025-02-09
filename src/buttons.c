

#include <inttypes.h>
#include <pico/stdlib.h>
#include <hardware/flash.h>
#include <hardware/regs/addressmap.h>
#include <pico/sync.h>
#include <pico/multicore.h>
#include <string.h>
#include <stdio.h>
#include "buttons.h"
#include "dali.h"
#include "modbus.h"
#include "network.h"
#include "stringutil.h"

#define BUTTON_SER_PIN 6
#define BUTTON_CLK_PIN 7
#define ROW_BASE_PIN 8

#define MS_TO_COUNTDOWN(ms) (ms / 5)

// We are assuming 2MB of flash, and we use the last sector (4Kb).  The very last value will be set to a magic value.
#define FLASH_CONFIG_OFFSET ((2 * 1024 * 1024) - FLASH_SECTOR_SIZE)

// This must be at least NUM_FIXTURES * NUM_BUTTONS_PER_FIXTURE + 1
#define NUM_BINDINGS 256
// Must be a multiple of 256 (the flash PAGE size) - Ensure that NUM_BINDINGS is set so that this is true.
#define CONFIG_SZ (NUM_BINDINGS * sizeof(uint32_t))
static const uint32_t *bindings = (const uint32_t *)(XIP_BASE + FLASH_CONFIG_OFFSET);

// Trying to use as little memory as possible (when we transition to CH559), we cram stuff together - 2 bytes per switch x 168 is pretty good.

button_ctx_t button_ctx[NUM_FIXTURES * NUM_BUTTONS_PER_FIXTURE];

static int fixture = 0;
static button_ctx_t *ctx = button_ctx;
static absolute_time_t next_button_scan;

critical_section_t binding_access_lock;

uint32_t encode_binding(binding_t *binding)
{
    uint32_t v = binding->type << 24 | (binding->device & 0xFF) << 16 | (binding->address & 0xFFFF);

    return v;
}

static inline void lock()
{
    critical_section_enter_blocking(&binding_access_lock);
}

static inline void unlock()
{
    critical_section_exit(&binding_access_lock);
}

void get_binding_at_index(uint index, binding_t *binding)
{
    lock();
    decode_binding(bindings[index], binding);
    unlock();
}

void decode_binding(uint32_t encoded, binding_t *binding)
{
    binding->type = encoded >> 24;
    binding->device = (encoded >> 16) & 0xFF;
    binding->address = encoded & 0xFFFF;
}

#define MAGIC_VALUE ('M' << 24 | 'E' << 16 | 'C' << 8 | 'H')

static void persist_bindings(uint32_t sector[NUM_BINDINGS])
{
    sector[NUM_BINDINGS - 1] = MAGIC_VALUE;
    fflush(stdout);
    multicore_lockout_start_blocking();
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(FLASH_CONFIG_OFFSET, FLASH_SECTOR_SIZE); // Must always erase entire sectors
    flash_range_program(FLASH_CONFIG_OFFSET, (const uint8_t *)sector, CONFIG_SZ);
    restore_interrupts(ints);
    multicore_lockout_end_blocking();
}

/**
 * Tests that the binding config stored in flash is valid - if not, it erases it.
 */
static void test_flash_config()
{
    lock();
    binding_t binding = {
        .type = BINDING_TYPE_NONE,
        .device = 0,
        .address = 0,
    };
    if (bindings[NUM_BINDINGS - 1] != MAGIC_VALUE)
    {
        uint32_t sector[NUM_BINDINGS];
        for (int i = 0; i < NUM_BINDINGS - 1; i++)
        {
            sector[i] = encode_binding(&binding);
        }
        persist_bindings(sector);
    }
    unlock();
}

bool parse_binding(char *sval, binding_t *binding) {
    int address_min = 0;
    int address_max;
    bool has_address = true;
    char *after;

    if (starts_with(sval, "dali", &after))
    {
        binding->type = BINDING_TYPE_DALI;
        address_max = 63;
    }
    else if (starts_with(sval, "modbus", &after))
    {
        binding->type = BINDING_TYPE_MODBUS;
        address_max = 31;
    }
    else if (strcmp(sval, "none") == 0)
    {
        binding->type = BINDING_TYPE_NONE;
        has_address = false;
    }
    else
    {
        return false;
    }

    if (has_address)
    {
        int id = strtoimax(after, &after, 10);
        binding->device = id / 100;
        binding->address = id % 100;
        if (*after != 0 || binding->address < address_min || binding->address > address_max)
        {
            return false;
        }
    }
    return true;
}


uint32_t get_binding(uint fixture, uint button) {
    assert(fixture < NUM_FIXTURES && button < NUM_BUTTONS_PER_FIXTURE);
    lock();
    uint32_t val = bindings[fixture * NUM_BUTTONS_PER_FIXTURE + button];
    unlock();
    return val;
}

void set_and_persist_binding(uint fixture, uint button, binding_t *binding)
{

    assert(fixture < NUM_FIXTURES && button < NUM_BUTTONS_PER_FIXTURE);
    uint32_t sector[NUM_BINDINGS];
    memcpy(sector, bindings, NUM_BINDINGS * sizeof(uint32_t));
    uint32_t new_binding = encode_binding(binding);
    sector[fixture * NUM_BUTTONS_PER_FIXTURE + button] = new_binding;

    lock();
    persist_bindings(sector);
    unlock();

    enqueue_device_update(EVT_BUTTON_BINDING_CHANGED, button_ctx + fixture * NUM_BUTTONS_PER_FIXTURE + button);
}

/**
 * Processes the repeat handling of a button
 *
 * This task will be deleted when the button is released, so it should take care not to allocate any memory,
 * as it won't be afforded an opportunity to clean up.
 */
static void button_pressed(button_ctx_t *ctx)
{
    enqueue_device_update(EVT_BTN_PRESSED, ctx);

    ctx->velocity = -ctx->velocity; // Switch direction
    ctx->countdown = ctx->velocity ? MS_TO_COUNTDOWN(250) : MS_TO_COUNTDOWN(750);
}


bool is_button_pressed(int fixture, int button) {
    int index = fixture * NUM_BUTTONS_PER_FIXTURE + button;

    return !button_ctx[index].released;
}

static void button_timeout_check(button_ctx_t *ctx)
{
    binding_t binding;

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
            get_binding_at_index(ctx - button_ctx, &binding);
            enqueue_device_update(EVT_BTN_HELD, ctx);

            // Only DALI devices are dimmable
            if (binding.type == BINDING_TYPE_DALI)
            {
                dali_fade(binding.address, ctx->velocity);
            }
        }
    }
}

static void button_released(button_ctx_t *ctx)
{
    binding_t binding;
    get_binding_at_index(ctx - button_ctx, &binding);
    enqueue_device_update(EVT_BTN_RELEASED, ctx);


    if (ctx->velocity == 0)
    {
        switch (binding.type)
        {
        case BINDING_TYPE_DALI:
            // log_i("Toggling Dali");
            dali_toggle(binding.address);
            break;
        case BINDING_TYPE_MODBUS:
            // log_i("Toggling Modbus");
            modbus_set_coil(1, binding.address, 2);
            break;
        default:
            break;
        }
    }
    else
    {
        ctx->countdown = ctx->velocity ? MS_TO_COUNTDOWN(500) : 0;
    }
}

void buttons_enumerate()
{
    int i = 0;
    button_ctx_t *btn = button_ctx;
    for (i = 0; i < NUM_BUTTONS_PER_FIXTURE * NUM_FIXTURES; i++) {
        enqueue_device_update(EVT_BUTTON_BINDING_CHANGED, btn++);
    }
}

void buttons_init()
{
    critical_section_init(&binding_access_lock);

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
