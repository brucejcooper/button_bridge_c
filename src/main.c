
#include <pico/stdlib.h>
#include <hardware/flash.h>
#include <hardware/sync.h>
#include <hardware/irq.h>
#include <hardware/watchdog.h>
#include <pico/util/queue.h>
#include <pico/multicore.h>
#include <pico/sync.h>
#include "dali.h"
#include "modbus.h"
#include "buttons.h"
#include "network.h"

#include "pico/multicore.h"


#define RS485_CS_PIN 1
#define DALI_TX_PIN 2
#define DALI_RX_PIN 3
#define RS485_TX_PIN 4
#define RS485_RX_PIN 5



void scan_loop()
{
    buttons_poll();
    dali_poll();
    modbus_poll();
    watchdog_update();

}

int main()
{
    // // Set ourselves up to run at 48Mhz, off the USB PLL - This should save some current and we don't have much in the
    // // way of speed requirements, and making both clocks use the same PLL minimises usage.
    // clock_configure(clk_sys,
    //                 CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
    //                 CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
    //                 48 * MHZ,
    //                 48 * MHZ);

    // // Turn off PLL sys for good measure
    // pll_deinit(pll_sys);

    // // CLK peri is clocked from clk_sys so need to change clk_peri's freq
    // clock_configure(clk_peri,
    //                 0,
    //                 CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS,
    //                 48 * MHZ,
    //                 48 * MHZ);

    stdio_init_all();


    network_init();
    dali_init(DALI_TX_PIN, DALI_RX_PIN);
    modbus_init(RS485_TX_PIN, RS485_RX_PIN, RS485_CS_PIN);
    buttons_init();


    multicore_lockout_victim_init();
    multicore_launch_core1(network_thread);

    // At the start, enumerate all devices.
    // enumerate_all();
    watchdog_enable(1000, 1);

    // Do an initial scan of the DALI bus. 
    dali_start_scan();

    for (;;)
    {
        scan_loop();
    }
    return 0;
}
