#include "cli.h"
#include <pico/stdlib.h>
#include <hardware/regs/addressmap.h>
#include "hardware/clocks.h"
#include <hardware/regs/vreg_and_chip_reset.h>
#include <hardware/structs/vreg_and_chip_reset.h>
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

// Used as the prefix for all button fixture entities
static const char *fixture_entity_prefix = "button_fixture";
static const char *fixture_binding_postfix = "_binding";
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

static bool process_addressed_dali_bus_cmd(int bus, int addr, char *cmd)
{
    char *tok;
    int onoff = -1;
    int newlevel = -1;

    while (*(tok = strsep(&cmd, " \t")))
    {
        char *key = strsep(&tok, "=");
        if (*tok)
        {

            if (strcmp(key, "state") == 0)
            {
                if (strcmp(tok, "on") == 0)
                {
                    onoff = 1;
                }
                else if (strcmp(tok, "off") == 0)
                {
                    onoff = 0;
                }
                else if (strcmp(tok, "toggle") == 0)
                {
                    onoff = 2;
                }
                else
                {
                    return false;
                }
            }
            else if (strcmp(key, "brightness") == 0)
            {
                char *end;
                newlevel = strtoimax(tok, &end, 0);
                if (*end)
                {
                    return false;
                }
            }
            else
            {
                return false;
            }
        }
        else
        {
            if (strcmp(key, "on") == 0)
            {
                onoff = 1;
            }
            else if (strcmp(key, "off") == 0)
            {
                onoff = 0;
            }
            else if (strcmp(key, "toggle") == 0)
            {
                onoff = 2;
            }
            else
            {
                newlevel = strtoimax(key, &tok, 0);
                if (*tok)
                {
                    return false;
                }
            }
        }
    }

    if (newlevel >= 0 && newlevel <= 254)
    {
        printf("Setting dali[%d] level to %d\n", addr, newlevel);
        dali_set_level(addr, newlevel);
        return true;
    }
    else if (onoff != -1)
    {
        if (onoff == 2)
        {
            printf("Toggling DALI[%d]\n", addr);

            dali_toggle(addr);
            return true;
        }
        else
        {
            printf("setting DALI[%d] onoff to %d\n", addr, onoff);
            dali_set_on(addr, onoff);
            return true;
        }
    }
    return false;
}

static bool process_addressed_modbus_cmd(int device, int addr, char *cmd)
{
    if (strcmp(cmd, "on") == 0)
    {
        modbus_set_coil(device, addr, 1);
    }
    else if (strcmp(cmd, "off") == 0)
    {
        modbus_set_coil(device, addr, 0);
    }
    else if (strcmp(cmd, "toggle") == 0)
    {
        modbus_set_coil(device, addr, 2);
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
        int idnum = strtoimax(entityid, &end, 10);
        if (*end == 0)
        {
            int device = idnum / 100;
            int address = idnum % 100;

            if (device < 1 && device > 1 && address < 0 | address >= 64)
            {
                return false;
            }
            return process_addressed_dali_bus_cmd(device, address, cmd);
        }
    }
    return false;
}

static bool process_modbus_cmd(char *entityid, char *cmd)
{
    if (*entityid != 0)
    {
        char *end;
        int id = strtoimax(entityid, &end, 10);
        if (*end == 0)
        {
            int device = id / 100;
            int address = id % 100;
            if (device < 1 || device > 1 || address < 0 || address > 32)
            {
                return false;
            }
            return process_addressed_modbus_cmd(device, address, cmd);
        }
    }
    return false;
}


static bool process_set_binding_cmd(int fixture, int button, char *val)
{
    int address_min = 0;
    int address_max;
    bool has_address = true;
    binding_t binding = {
        .type = BINDING_TYPE_NONE,
        .device = 0,
        .address = 0,
    };

    char *after;
    if (starts_with(val, dali_entity_prefix, &after))
    {
        binding.type = BINDING_TYPE_DALI;
        address_max = 63;
    }
    else if (starts_with(val, modbus_entity_prefix, &after))
    {
        binding.type = BINDING_TYPE_MODBUS;
        address_max = 31;
    }
    else if (strcmp(val, "none") == 0)
    {
        binding.type = BINDING_TYPE_NONE;
        has_address = false;
    }
    else
    {
        return false;
    }

    if (has_address)
    {
        int id = strtoimax(after, &after, 10);
        binding.device = id / 100;
        binding.address = id % 100;
        if (*after != 0 || binding.address < address_min || binding.address > address_max)
        {
            return false;
        }
    }
    set_and_persist_binding(fixture, button, &binding);
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
        else 
        if (strcmp(tok, "debug") == 0)
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
    }
    if (!success)
    {
        printf("Invalid command\n");
    }
}

void cli_init()
{
}

static inline char *binding_tostr(uint32_t encoded, char *out, size_t sz)
{
    binding_t binding;
    decode_binding(encoded, &binding);
    const char *typestr;

    switch (binding.type)
    {
    case BINDING_TYPE_NONE:
        strcpy(out, "none");
        return out;
    case BINDING_TYPE_DALI:
        typestr = dali_entity_prefix;
        break;
    case BINDING_TYPE_MODBUS:
        typestr = modbus_entity_prefix;
        break;
    default:
        typestr = "inv";
    }
    snprintf(out, sz, "%s%02d%02d", typestr, binding.device, binding.address);
    return out;
}

void cli_poll()
{
    int c;
    char tmpstr[16];

    if ((c = getchar_timeout_us(100)) >= 0)
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

}
