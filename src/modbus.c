#include <hardware/clocks.h>
#include <pico/stdlib.h>
#include <hardware/uart.h>
#include <hardware/irq.h>
#include "modbus.h"
#include "stdbool.h"
#include "stdint.h"
#include <string.h>
#include <stdio.h>
#include "queue.h"

static uint32_t coil_values;
static uint8_t buf[9];
static uint8_t *bufptr;

static uint8_t xor = 0;
static uint16_t crc = 0xFFFF;

static int cs_pin;

#define UART uart1

typedef enum
{
    OP_CLEAR,
    OP_SET,
    OP_TOGGLE,
    OP_FETCH,
    OP_NONE = 0xFF,
} op_t;

typedef struct
{
    int address;
    int coil;
    op_t op;
} cmd_t;

#define QUEUE_DEPTH 10
static cmd_t queue_data[QUEUE_DEPTH];
static queue_t cmd_queue;
static cmd_t in_flight = {
    .address = 0,
    .coil = 0,
    .op = OP_NONE,
};
absolute_time_t timeout;

const char *modbus_entity_prefix = "modbus";

static inline void reset_buf()
{
    bufptr = buf;
    xor = 0;
    crc = 0xFFFF;
}

void modbus_init(int tx_pin, int rx_pin, int cs)
{

    queue_init(&cmd_queue, queue_data, sizeof(cmd_t), QUEUE_DEPTH);

    uart_init(UART, 9600);
    coil_values = 0;

    cs_pin = cs;
    gpio_init(cs_pin);
    gpio_set_dir(cs_pin, true);

    // Without this, while driving the level floats, which can confuse the uart
    gpio_pull_up(rx_pin);

    // Set DE to 0 (Put it in receive mode)
    gpio_put(cs_pin, 0);

    gpio_set_function(tx_pin, GPIO_FUNC_UART);
    gpio_set_function(rx_pin, GPIO_FUNC_UART);
    reset_buf();
    // The RS485 chip seems to need a little bit of start up time before it can receive commands after reboot.
    // We put a sleep in here as a cheap way of ensuring that.
    sleep_ms(1);
}

static void append(uint8_t val)
{
    static const uint16_t table[256] = {
        0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
        0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
        0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
        0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
        0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
        0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
        0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
        0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
        0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
        0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
        0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
        0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
        0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
        0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
        0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
        0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
        0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
        0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
        0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
        0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
        0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
        0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
        0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
        0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
        0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
        0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
        0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
        0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
        0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
        0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
        0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
        0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040};

    *bufptr++ = val;
    xor = val ^ crc;
    crc >>= 8;
    crc ^= table[xor];
}

static inline void append_crc()
{
    *bufptr++ = crc & 0xFF;
    *bufptr++ = crc >> 8;
}

static inline bool crc_is_valid()
{
    return crc == 0;
}

static void print_status(int index, bool is_on)
{
    printf("\r\t%s%d %s\n", modbus_entity_prefix, index, is_on ? "on" : "off");
}

/**
 * Checks to see if we have read a certain number of bytes from the UART
 */
static bool bytes_available(size_t expected_sz)
{
    do
    {
        if (bufptr - buf >= expected_sz)
        {
            return true;
        }
        if (!uart_is_readable(UART))
        {
            return false;
        }
        append(uart_getc(UART));
    } while (true);
    return false; // Dead code
}

#define BYTE_TRANSMISSION_DELAY_US(n_bits) (n_bits * 10 * 1000000 / 9600)

static inline void wait_for_transmission(int nbytes)
{
    // Wait for enough time for the transmission to occur
    // TODO use hardware timer
    sleep_until(make_timeout_time_us(BYTE_TRANSMISSION_DELAY_US(8)));
    // Set DE back low, so that we can receive the response
    gpio_put(cs_pin, 0);
}

static void send_set_coil(uint8_t devaddr, int coil_num, op_t val)
{
    reset_buf();
    append(devaddr);
    append(0x05);
    append(00);
    append(coil_num);
    append(val == OP_TOGGLE ? 0x55 : val == OP_SET ? 0xFF
                                                   : 0x00);
    append(0x00);
    append_crc();

    // Set DE to 1 (Write Mode)
    gpio_put(cs_pin, 1);

    // This could techically block, but we only write one thing at a time, and we've got a lock
    // around it, so its _very_ unlikely as long as we write less than the Queue depth
    uart_write_blocking(UART, buf, 8);

    wait_for_transmission(8);
    switch (val)
    {
    case OP_CLEAR:
        coil_values &= ~(1 << coil_num);
        break;
    case OP_SET:
        coil_values |= 1 << coil_num;
        break;
    case OP_TOGGLE:
        // TODO go stateless - its more reliable, plus it'll save us 4 bytes :P
        coil_values ^= 1 << coil_num;
        break;
    }
    print_status(coil_num, coil_values & (1 << coil_num));
}

static void request_coil_status(int devaddr)
{
    printf("Enumerating Modbus devices at address %d\n", devaddr);

    reset_buf();
    append(devaddr);
    append(0x01);
    append(0x00);
    append(0x00);
    append(0x00);
    append(0x20);
    append_crc();
    gpio_put(cs_pin, 1);
    // Technically this can block, but it probably won't (the FIFO is 32 bytes long)
    uart_write_blocking(UART, buf, 8);
    wait_for_transmission(8);
}

void modbus_enumerate()
{
    cmd_t cmd = {
        .address = 1,
        .coil = 0,
        .op = OP_FETCH};
    queue_add(&cmd_queue, &cmd);
}

void modbus_set_coil(uint8_t devaddr, int coil_num, int val)
{
    cmd_t cmd = {
        .address = devaddr,
        .coil = coil_num,
        .op = val,
    };
    queue_add(&cmd_queue, &cmd);
}

void modbus_poll()
{

    if (in_flight.op != OP_NONE)
    {
        if (bytes_available(in_flight.op == OP_FETCH ? 9 : 8))
        {
            if (crc_is_valid())
            {
                switch (in_flight.op)
                {
                case OP_FETCH:

                    coil_values = (buf[3] << 24) | (buf[4] << 16) | (buf[5] << 8) | buf[6];
                    printf("coils 0x%08x\n", coil_values);

                    for (int coil = 0; coil < 32; coil++)
                    {
                        printf("\r\tdevice %s%d name=\"Modbus relay %d\"\n", modbus_entity_prefix, coil, coil);
                        printf("\r\tswitch %s%d device=%s%d\n", modbus_entity_prefix, coil, modbus_entity_prefix, coil);
                        print_status(coil, coil_values & (1 << coil));
                    }
                    break;
                default:
                    printf("Ignoring response from modbus set_coil op\n");
                }
            }
            else
            {
                printf("CRC mismatch on response\n");
            }
        }
        else if (time_reached(timeout))
        {
            printf("Timeout receiving response from Modbus. Assuming no response will be forthcoming\n");
        }
        in_flight.op = OP_NONE;
    }

    if (uart_is_writable(UART) && queue_get(&cmd_queue, &in_flight))
    {
        timeout = make_timeout_time_ms(1000); // Give the TX 1000 ms.
        switch (in_flight.op)
        {
        case OP_SET:
        case OP_CLEAR:
        case OP_TOGGLE:
            send_set_coil(in_flight.address, in_flight.coil, in_flight.op);
            break;
        case OP_FETCH:
            break;
        }
        // We expect to see a response in a handful of ms, but give it a bit longer, just in case.
        timeout = make_timeout_time_ms(100);
        // Reset read buffer for read.
        reset_buf();
    }
}