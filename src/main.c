
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <pico/stdlib.h>
#include <hardware/clocks.h>
#include <hardware/flash.h>
#include <hardware/sync.h>
#include <hardware/irq.h>
#include <hardware/watchdog.h>
#include <string.h>
#include "dali.h"
#include "modbus.h"
#include "async.h"

#define NUM_FIXTURES 24
#define NUM_BUTTONS_PER_FIXTURE 7
#define RS485_CS_PIN 1
#define DALI_TX_PIN 2
#define DALI_RX_PIN 3
#define RS485_TX_PIN 4
#define RS485_RX_PIN 5
#define BUTTON_SER_PIN 6
#define BUTTON_CLK_PIN 7
#define ROW_BASE_PIN 8

// The number of times we sample each key per second.
#define BUTTON_SAMPLE_PERIOD 1000000 / (NUM_FIXTURES * 200)

#define DIM_DIRECTION_DOWN 0
#define DIM_DIRECTION_UP 0

// Bus type stored in the top two bits of the address byte
typedef enum
{
    BUS_TYPE_MODBUS = 0,
    BUS_TYPE_DALI = 1,
    BUS_TYPE_NONE = 3, // We use this because flash is initialised to 0xFF when empty
} bus_type_t;

// Mask for the bottom 6 bits, which should be the address on the bus.
#define BUS_ADDRESS_MASK 0x3F

// We are assuming 2MB of flash, and we use the last sector (4Kb).  The first two words will be magic values, then the rest will be bindings
#define FLASH_CONFIG_OFFSET ((2 * 1024 * 1024) - FLASH_SECTOR_SIZE)
static const uint8_t *bindings = (const uint8_t *)(XIP_BASE + FLASH_CONFIG_OFFSET);

// Trying to use as little memory as possible (when we transition to CH559), we cram stuff together - 2 bytes per switch x 168 is pretty good.
typedef struct __attribute__((packed))
{
    uint countdown : 8;
    uint released : 1;
    uint dimming : 1;
    uint direction : 1;
} button_ctx_t;

static button_ctx_t button_ctx[NUM_FIXTURES * NUM_BUTTONS_PER_FIXTURE];

static char input_buf[40];
static char *input_ptr = input_buf;

void switches_enumerate_task(async_ctx_t *ctx);

static inline char *binding_tostr(uint8_t binding, char *out)
{
    switch (binding >> 6)
    {
    case BUS_TYPE_NONE:
        strcpy(out, "none");
        break;
    case BUS_TYPE_DALI:
        sprintf(out, "dali%d", binding & BUS_ADDRESS_MASK);
        break;
    case BUS_TYPE_MODBUS:
        sprintf(out, "modbus%d", binding & BUS_ADDRESS_MASK);
        break;
    default:
        sprintf(out, "inv%08x", binding);
        break;
    }
    return out;
}

static void print_sml_i(int i)
{
    putchar('0' + (i / 10));
    putchar('0' + (i % 10));
}

static void log_button_evt(button_ctx_t *ctx, char *evt)
{
    int idx = ctx - button_ctx;
    printf("%lu btn %d/%d %s\n", (time_us_32() / 1000), idx / NUM_BUTTONS_PER_FIXTURE, idx % NUM_BUTTONS_PER_FIXTURE, evt);
}

/**
 * Processes the repeat handling of a button
 *
 * This task will be deleted when the button is released, so it should take care not to allocate any memory,
 * as it won't be afforded an opportunity to clean up.
 */
static inline void button_pressed(button_ctx_t *ctx)
{
    log_button_evt(ctx, "pressed");
    // If we're dimming, reverse direction
    if (ctx->dimming)
    {
        ctx->direction = !ctx->direction; // Switch direction
        ctx->countdown = 50;
    }
    else
    {
        ctx->countdown = 150; // Set long press timer to 750ms
    }
}

static inline void button_released(button_ctx_t *ctx)
{
    int i = ctx - button_ctx;

    if (!ctx->dimming)
    {
        // Not dimming
        ctx->countdown = 0;
        log_button_evt(ctx, "clicked");

        switch (bindings[i] >> 6)
        {
        case BUS_TYPE_DALI:
            dali_toggle(bindings[i] & BUS_ADDRESS_MASK);
            break;
        case BUS_TYPE_MODBUS:
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

        // When we release after dimming, we wait for up to 330 ms for the user to press again, reversing the direction
        ctx->countdown = 66;
    }
}

static void button_process(button_ctx_t *ctx, int is_released)
{
    char tmpbuf[16];

    if (ctx->released != is_released)
    {
        ctx->released = is_released;
        if (is_released)
        {
            button_released(ctx);
        }
        else
        {
            button_pressed(ctx);
        }
    }
    // If we're counting down and we get to 0, take action.
    if (ctx->countdown && (--(ctx->countdown)) == 0)
    {
        int i = ctx - button_ctx;
        if (ctx->released)
        {
            // The button wasn't re-pressed within the timout period.
            // Reset to normal velocity, so the next press can start a new transaction.
            ctx->dimming = 0;
            ctx->direction = DIM_DIRECTION_DOWN;
        }
        else
        {
            ctx->countdown = 50; // Repeat again in 250ms

            if (!ctx->dimming)
            {
                // Set it to dimming downwards - Clear the DIMDIR bit and set the DIMMING bit
                ctx->dimming = 1;
                ctx->direction = DIM_DIRECTION_DOWN;
            }

            log_button_evt(ctx, "long press");

            switch (bindings[i] >> 6)
            {
            case BUS_TYPE_DALI:
                int addr = bindings[i] & BUS_ADDRESS_MASK;

                printf("dali%d %s\n", addr, ctx->direction ? "brighten" : "dim");
                dali_fade(addr, ctx->direction);
                break;
            default:
                printf("binding %s is not dimmable\n", binding_tostr(bindings[i], tmpbuf));
                break;
            }
        }
    }
}

static void tolower_str(char *c)
{
    while (*c)
    {
        *c++ = tolower(*c);
    }
}

static void enumerate_all()
{
    // Start tasks to enumerate the different busses.
    async_start_task(dali_enumerate_task, 0);
    async_start_task(modbus_enumerate_task, 1);
    async_start_task(switches_enumerate_task, 0);
}

static bool process_dali_bus_cmd(char *cmd)
{
    if (strcmp(cmd, "provision") == 0)
    {
        printf("Provisioning devices on the dali bus\n");
        printf("Not implemented yet\n");
        return true;
    }
    return false;
}

static bool process_modbus_bus_cmd(char *cmd)
{
    return false;
}

static bool process_addressed_dali_bus_cmd(int addr, char *cmd)
{
    char *end;
    int level = strtoimax(cmd, &end, 0);
    if (*end == 0 && level >= 0 && level <= 254)
    {
        // It was a valid level
        dali_set_level(addr, level);
    }
    else if (strcmp(cmd, "on") == 0)
    {
        dali_set_on(addr, 1);
    }
    else if (strcmp(cmd, "off") == 0)
    {
        dali_set_on(addr, 0);
    }
    else if (strcmp(cmd, "toggle") == 0)
    {
        dali_toggle(addr);
    }
    else
    {
        return false;
    }
    return true;
}

static bool process_addressed_modbus_cmd(int addr, char *cmd)
{
    if (strcmp(cmd, "on") == 0)
    {
        modbus_set_coil(1, addr, 1);
    }
    else if (strcmp(cmd, "off") == 0)
    {
        modbus_set_coil(1, addr, 0);
    }
    else if (strcmp(cmd, "toggle") == 0)
    {
        modbus_set_coil(1, addr, 2);
    }
    else
    {
        return false;
    }
    return true;
}

static bool process_dali_cmd(char *entityid, char *cmd)
{
    if (*entityid == 0)
    {
        return process_dali_bus_cmd(cmd);
    }
    else
    {
        char *end;
        int address = strtoimax(entityid, &end, 0);
        if (*end != 0 || address < 0 || address >= 64)
        {
            return false;
        }

        return process_addressed_dali_bus_cmd(address, cmd);
    }
    return false;
}

static bool process_modbus_cmd(char *entityid, char *cmd)
{
    if (*entityid == 0)
    {
        return process_modbus_bus_cmd(cmd);
    }
    else
    {
        char *end;
        int address = strtoimax(entityid, &end, 0);
        if (*end != 0 || address < 0 || address >= 32)
        {
            return false;
        }
        return process_addressed_modbus_cmd(address, cmd);
    }
    return false;
}

static bool process_switch_cmd(char *entityid, char *cmd)
{
    printf("Processing switch cmd %s on entity %s\n", cmd, entityid);
    return false;
}

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
        printf("Flash is not initialised - Blanking it out\n");
        uint8_t sector[FLASH_PAGE_SIZE];
        memcpy(sector + FLASH_PAGE_SIZE - sizeof(binding_magicvals), binding_magicvals, sizeof(binding_magicvals));
        memset(sector, 0xFF, FLASH_PAGE_SIZE - sizeof(binding_magicvals));
        persist_bindings(sector);
    }
}

static void set_and_persist_binding(uint fixture, uint button, uint8_t encoded_binding)
{
    assert(fixture < NUM_FIXTURES && button < NUM_BUTTONS_PER_FIXTURE);
    uint8_t sector[FLASH_PAGE_SIZE];
    memcpy(sector, bindings, FLASH_PAGE_SIZE);
    sector[fixture * NUM_BUTTONS_PER_FIXTURE + button] = encoded_binding;
    persist_bindings(sector);
}

static bool process_set_binding_cmd(int fixture, int button, char *val)
{
    if (strncmp(val, "dali", 4) == 0)
    {
        char *after;
        int address = strtoimax(val + 4, &after, 10);
        if (*after != 0 || address < 0 || address >= 64)
        {
            return false;
        }
        printf("Setting fx%d_binding%d = dali%d\n", fixture, button, address);
        set_and_persist_binding(fixture, button, BUS_TYPE_DALI << 6 | address);
    }
    else if (strncmp(val, "modbus", 6) == 0)
    {
        char *after;
        int address = strtoimax(val + 6, &after, 10);
        if (*after != 0 || address < 0 || address >= 32)
        {
            return false;
        }
        printf("Setting fx%d_binding%d = modbus%d\n", fixture, button, address);
        set_and_persist_binding(fixture, button, BUS_TYPE_MODBUS << 6 | address);
    }
    else if (strcmp(val, "none") == 0)
    {
        printf("Clearing binding for fx%d_binding%d\n", fixture, button);
        set_and_persist_binding(fixture, button, 0xFF);
    }
    else
    {
        return false;
    }

    return true;
}

static void process_cmd(char *cmd)
{
    char *tok;
    tolower_str(cmd);
    tok = strsep(&cmd, " \t");
    bool success = false;
    if (tok != NULL)
    {
        if (strcmp(tok, "enumerate") == 0)
        {
            printf("Enumerating entities\n");
            enumerate_all();
            success = true;
        }
        else if (strncmp(tok, "dali", 4) == 0)
        {
            success = process_dali_cmd(tok + 4, cmd);
        }
        else if (strncmp(tok, "modbus", 6) == 0)
        {
            success = process_modbus_cmd(tok + 6, cmd);
        }
        else if (strncmp(tok, "button_fixture", 14) == 0)
        {
            char *after_fix;
            int fixture = strtoimax(tok + 14, &after_fix, 10);
            if (after_fix != (tok + 2) && strncmp(after_fix, "_binding", 8) == 0 && fixture >= 0 && fixture < NUM_FIXTURES)
            {
                int button = strtoimax(after_fix + 8, &tok, 10);
                if (*tok == 0 && button >= 0 && button < NUM_BUTTONS_PER_FIXTURE)
                {
                    success = process_set_binding_cmd(fixture, button, cmd);
                }
            }
        }
        if (!success)
        {
            printf("Invalid command\n");
        }
    }

    // printf("Finished processing\n");
}

void switches_enumerate_task(async_ctx_t *ctx)
{
    async_begin();
    char tmpstr[16];
    for (ctx->idata = 0; ctx->idata < NUM_FIXTURES * NUM_BUTTONS_PER_FIXTURE; ctx->idata++)
    {
        int fixture = ctx->idata / NUM_BUTTONS_PER_FIXTURE;
        int button = ctx->idata % NUM_BUTTONS_PER_FIXTURE;

        if (button == 0)
        {
            printf("\r\tdevice button_fixture%d name=\"Button Fixture %d\"\n", fixture, fixture);
        }
        printf("\r\ttext button_fixture%d_binding%d device=button_fixture%d pattern=(none|mqtt\\d\\d?|dali\\d\\d?)\n", fixture, button, fixture);
        printf("\r\tbutton_fixture%d_binding%d %s\n", fixture, button, binding_tostr(bindings[ctx->idata], tmpstr));
        async_yield();
    }
    async_end();
}

void loop()
{
    uint8_t val;

    // At the start, enumerate all DALI devices.
    enumerate_all();

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
            ctx->dimming = 0;
            ctx->direction = DIM_DIRECTION_DOWN;
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

    absolute_time_t next_iter = make_timeout_time_us(BUTTON_SAMPLE_PERIOD);

    ctx = button_ctx;
    int fixture = 0;
    for (;;)
    {
        // Wait for our next iteration time - This will happen
        sleep_until(next_iter);
        next_iter = delayed_by_us(next_iter, BUTTON_SAMPLE_PERIOD);
        // First, clock out the data for the _next_ iteration - 0 for the last one and 1 for all others
        gpio_put(BUTTON_SER_PIN, fixture == (NUM_FIXTURES - 1) ? 0 : 1);
        sleep_us(1);                 // We need to delay between setting data and clocking so that it will be guaranteed to be correct at the chip
        gpio_put(BUTTON_CLK_PIN, 1); // Clock out the data we just set.
        sleep_us(20);                // Give the inputs a moment to settle into their correct values.

        // Now read in the data for one fixture.
        val = (gpio_get_all() >> ROW_BASE_PIN);
        // Turn the changes from a bit field into indexes for more easy consumption.
        for (int btn_idx = 0; btn_idx < NUM_BUTTONS_PER_FIXTURE; btn_idx++)
        {
            button_process(ctx++, val & 0x01);
            val >>= 1;
        }

        // Increment which fixture we're working on, wrapping around as needed.
        if (++fixture == NUM_FIXTURES)
        {
            ctx = button_ctx;
            fixture = 0;
        }
        async_exec_tasks();

        int c = getchar_timeout_us(0);
        if (c >= 0) // Errors are -ve
        {
            if (c == 0x04)
            {
                printf("Resetting\n");
                *((volatile uint32_t *)(PPB_BASE + 0x0ED0C)) = 0x5FA0004;
            }
            else
            {
                putchar(c);
                if (c == '\n')
                {
                    *input_ptr = 0;
                    input_ptr = input_buf;
                    process_cmd(input_buf);
                }
                else if (c == 0x08)
                {
                    if (input_ptr > input_buf)
                    {
                        putchar(' ');
                        putchar(0x08);
                        input_ptr--;
                    }
                }
                else
                {
                    if (input_ptr - input_buf < sizeof(input_buf) - 1 && c != '\r')
                    {
                        *input_ptr++ = c;
                    }
                }
            }
        }
        // Drive the clock back down low - This forms a handy indicator of how much time in each iteration we've consumed.
        // The clock will be high for the settle time plus procesisng time.
        gpio_put(BUTTON_CLK_PIN, 0);

        watchdog_update();
        // if (fixture_idx == 23)
        //     printf("\r");
    }
}

int main()
{
    stdio_init_all();

    sleep_ms(1000);

    if (watchdog_caused_reboot())
    {
        printf("Watchdog caused reset\n");
    }
    else
    {
        printf("normal restart");
    }

    // Check that we have a valid config sector - This is a magit string at the _END_ of the first page of the config
    test_flash_config();

    dali_init(DALI_TX_PIN, DALI_RX_PIN);
    modbus_init(RS485_TX_PIN, RS485_RX_PIN, RS485_CS_PIN);

    async_init();

    printf("Enabling Watchdog\n");
    watchdog_enable(1000, 1);

    loop();
    return 0;
}
