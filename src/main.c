
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <pico/stdlib.h>
#include <hardware/flash.h>
#include <hardware/sync.h>
#include <hardware/irq.h>
#include <hardware/watchdog.h>
#include "hardware/pll.h"
#include "hardware/clocks.h"
#include "hardware/structs/pll.h"
#include "hardware/structs/clocks.h"
#include <pico/util/queue.h>
#include <string.h>
#include "dali.h"
#include "modbus.h"
#include "buttons.h"
#include "cli.h"

#define RS485_CS_PIN 1
#define DALI_TX_PIN 2
#define DALI_RX_PIN 3
#define RS485_TX_PIN 4
#define RS485_RX_PIN 5

void enumerate_all()
{
    // Start tasks to enumerate the different busses.
    dali_enumerate();
    modbus_enumerate();
    buttons_enumerate();
}

int main()
{
    // Set ourselves up to run at 48Mhz, off the USB PLL - This should save some current and we don't have much in the
    // way of speed requirements, and making both clocks use the same PLL minimises usage.
    clock_configure(clk_sys,
                    CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
                    CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
                    48 * MHZ,
                    48 * MHZ);

    // Turn off PLL sys for good measure
    pll_deinit(pll_sys);

    // CLK peri is clocked from clk_sys so need to change clk_peri's freq
    clock_configure(clk_peri,
                    0,
                    CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS,
                    48 * MHZ,
                    48 * MHZ);

    stdio_init_all();
    cli_init();

    // Upon restart, wait a bit to give any processes that wish to attach to the CDC USB device a chance before we start printing
    sleep_ms(1000);
    printf("%s reset\n", watchdog_caused_reboot() ? "watchdog" : "normal");

    dali_init(DALI_TX_PIN, DALI_RX_PIN);
    modbus_init(RS485_TX_PIN, RS485_RX_PIN, RS485_CS_PIN);
    buttons_init();

    // At the start, enumerate all devices.
    enumerate_all();

    watchdog_enable(500, 1);
    for (;;)
    {
        buttons_poll();
        dali_poll();
        modbus_poll();
        cli_poll();

        watchdog_update();
    }
    return 0;
}

// modbus0 toggle
