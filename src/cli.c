#include "cli.h"
#include <pico/stdlib.h>
#include <stdarg.h>
#include <hardware/regs/addressmap.h>
#include "hardware/pll.h"
#include "hardware/clocks.h"
#include <hardware/regs/vreg_and_chip_reset.h>
#include <hardware/structs/vreg_and_chip_reset.h>
#include "hardware/structs/pll.h"
#include "hardware/structs/clocks.h"
#include <hardware/watchdog.h>
#include <pico/stdio.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <inttypes.h>
#include "dali.h"
#include "modbus.h"
#include "buttons.h"
#include <pico/util/queue.h>
#include <stdio.h>

#define MAX_COMMAND_LENGTH 80
static char input_buf[MAX_COMMAND_LENGTH];
static char *input_ptr = input_buf;

static queue_t output_queue;

// Used as the prefix for all button fixture entities
static const char *fixture_entity_prefix = "button_fixture";
static const char *fixture_binding_postfix = ".binding";
static const char *dali_entity_prefix = "dali";
static const char *modbus_entity_prefix = "modbus";

static bool starts_with(const char *src, const char *prefix, char **after)
{
    int len = strlen(prefix);
    if (strncmp(src, prefix, len) == 0)
    {
        *after = (char *)src + len;
        return true;
    }
    return false;
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
    if (*entityid != 0)
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

static bool process_set_binding_cmd(int fixture, int button, char *val)
{
    char *after;
    if (starts_with(val, dali_entity_prefix, &after))
    {
        int address = strtoimax(after, &after, 10);
        if (*after != 0 || address < 0 || address >= 64)
        {
            return false;
        }
        set_and_persist_binding(fixture, button, BINDING_TYPE_DALI << 6 | address);
    }
    else if (starts_with(val, modbus_entity_prefix, &after))
    {
        int address = strtoimax(after, &after, 10);
        if (*after != 0 || address < 0 || address >= 32)
        {
            return false;
        }
        set_and_persist_binding(fixture, button, BINDING_TYPE_MODBUS << 6 | address);
    }
    else if (strcmp(val, "none") == 0)
    {
        set_and_persist_binding(fixture, button, 0xFF);
    }
    else
    {
        return false;
    }

    return true;
}

void enumerate_all();

static void process_cmd(char *cmd)
{
    char *tok, *after_prefix;
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
        else if (strcmp(tok, "debug") == 0)
        {
            printf("Reboot reason: %x\n", vreg_and_chip_reset_hw->chip_reset);
            success = true;
            uint f_pll_sys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_PLL_SYS_CLKSRC_PRIMARY);
            uint f_pll_usb = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_PLL_USB_CLKSRC_PRIMARY);
            uint f_rosc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_ROSC_CLKSRC);
            uint f_clk_sys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS);
            uint f_clk_ref = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_REF);
            uint f_clk_peri = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_PERI);
            uint f_clk_usb = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_USB);
            uint f_clk_adc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_ADC);
            uint f_clk_rtc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_RTC);

            printf("pll_sys  = %dkHz\n", f_pll_sys);
            printf("pll_usb  = %dkHz\n", f_pll_usb);
            printf("rosc     = %dkHz\n", f_rosc);
            printf("clk_sys  = %dkHz\n", f_clk_sys);
            printf("clk_peri = %dkHz\n", f_clk_peri);
            printf("clk_ref  = %dkHz\n", f_clk_ref);
            printf("clk_usb  = %dkHz\n", f_clk_usb);
            printf("clk_adc  = %dkHz\n", f_clk_adc);
            printf("clk_rtc  = %dkHz\n", f_clk_rtc);
        }
        else if (starts_with(tok, dali_entity_prefix, &after_prefix))
        {
            success = process_dali_cmd(after_prefix, cmd);
        }
        else if (starts_with(tok, modbus_entity_prefix, &after_prefix))
        {
            success = process_modbus_cmd(after_prefix, cmd);
        }
        else if (starts_with(tok, fixture_entity_prefix, &after_prefix))
        {
            char *after_num;
            int fixture = strtoimax(after_prefix, &after_num, 10);
            if (after_num != after_prefix && starts_with(after_num, fixture_binding_postfix, &after_prefix) && fixture >= 0 && fixture < NUM_FIXTURES)
            {
                int button = strtoimax(after_prefix, &tok, 10);
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
}

void cli_init()
{
    queue_init(&output_queue, sizeof(log_msg_t), 200);
}

static inline char *binding_tostr(uint8_t binding, char *out, size_t sz)
{
    switch (binding >> 6)
    {
    case BINDING_TYPE_NONE:
        strcpy(out, "none");
        break;
    case BINDING_TYPE_DALI:
        snprintf(out, sz, "%s%d", dali_entity_prefix, binding & BINDING_ADDRESS_MASK);
        break;
    case BINDING_TYPE_MODBUS:
        snprintf(out, sz, "%s%d", modbus_entity_prefix, binding & BINDING_ADDRESS_MASK);
        break;
    default:
        snprintf(out, sz, "inv%08x", binding);
        break;
    }
    return out;
}

void cli_poll()
{
    int c;
    log_msg_t msg;
    char tmpstr[16];

    if ((c = getchar_timeout_us(0)) >= 0)
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
                    *input_ptr++ = tolower(c);
                }
            }
        }
    }

    // See if there's any log messages to de-queue and print
    // This will de-queue at most one at a time, so that the watchdog and other processes keep fed.
    if (queue_try_remove(&output_queue, &msg))
    {
        switch (msg.type)
        {
        case STATUS_PRINT_DEVICE:
            switch (msg.bus)
            {
            case MSG_SRC_DALI:
                if (msg.vals[0] != msg.vals[1])
                {
                    printf("\r\tlight %s%d brightness=true supported_color_modes=brightness brightness_scale=254 min=%d max=%d\n", dali_entity_prefix, msg.address, msg.vals[0], msg.vals[1]);
                }
                else
                {
                    printf("\r\tlight %s%d\n", dali_entity_prefix, msg.address);
                }
                break;
            case MSG_SRC_MODBUS:
                printf("\r\tswitch %s%d\n", modbus_entity_prefix, msg.address);
                break;
            case MSG_SRC_BUTTON_FIXTURE:
                printf("\r\ttext %s%d%s%d pattern=(none|%s(\\d|[1-2]\\d|3[01])|%s(\\d|[1-5]\\d|6[0-3]))\n", fixture_entity_prefix, msg.device, fixture_binding_postfix, msg.address, modbus_entity_prefix, dali_entity_prefix);
                break;
            default:
                break;
            }
            break;
        case STATUS_PRINT_VALUES:
            switch (msg.bus)
            {
            case MSG_SRC_DALI:
                printf("\r\t%s%d state=%s brightness=%d\n", dali_entity_prefix, msg.address, msg.vals[0] > 0 ? "on" : "off", msg.vals[0]);

                break;
            case MSG_SRC_MODBUS:
                printf("\r\t%s%d %s\n", modbus_entity_prefix, msg.address, msg.vals[0] ? "on" : "off");
                break;
            case MSG_SRC_BUTTON_FIXTURE:
                printf("\r\t%s%d%s%d %s\n", fixture_entity_prefix, msg.device, fixture_binding_postfix, msg.address, binding_tostr(msg.vals[0], tmpstr, sizeof(tmpstr)));

                break;
            default:
                break;
            }
            break;

        case STATUS_LOG:
            printf("%s\n", msg.msg);
            break;
        case STATUS_LOGINT:
            printf("%s %d\n", msg.msg, msg.vals[0]);
            break;
        }
    }
}

void print_msg(log_msg_t *msg)
{
    queue_try_add(&output_queue, msg);
}

void log_i(char *c)
{
    log_msg_t msg = {
        .type = STATUS_LOG,
        .msg = c,
    };
    print_msg(&msg);
}

void log_int(char *c, int val)
{
    log_msg_t msg = {
        .type = STATUS_LOGINT,
        .msg = c,
        .vals = {val, 0, 0}};
    print_msg(&msg);
}
