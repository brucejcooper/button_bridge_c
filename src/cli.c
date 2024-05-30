#include "cli.h"
#include <pico/stdlib.h>
#include <hardware/regs/addressmap.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <inttypes.h>
#include "dali.h"
#include "modbus.h"
#include "buttons.h"

#define MAX_COMMAND_LENGTH 40
static char input_buf[MAX_COMMAND_LENGTH];
static char *input_ptr = input_buf;

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
        set_and_persist_binding(fixture, button, BUS_TYPE_DALI << 6 | address);
    }
    else if (starts_with(val, modbus_entity_prefix, &after))
    {
        int address = strtoimax(after, &after, 10);
        if (*after != 0 || address < 0 || address >= 32)
        {
            return false;
        }
        set_and_persist_binding(fixture, button, BUS_TYPE_MODBUS << 6 | address);
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

    // printf("Finished processing\n");
}

void cli_init()
{
}

void cli_poll()
{
    int c;
    while ((c = getchar_timeout_us(0)) >= 0)
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
