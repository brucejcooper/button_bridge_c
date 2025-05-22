#ifndef _MODBUS_H
#define _MODBUS_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

typedef enum {
    MODBUS_TASK_STATE_IDLE,
    MODBUS_TASK_STATE_PENDING,
    MODBUS_TASK_STATE_AWAITING_RESPONSE,
    // These are all terminal states. 
    MODBUS_TASK_STATE_DONE,
    MODBUS_TASK_STATE_TIMEOUT,
    MODBUS_TASK_STATE_INVALID_CRC,
} modbus_task_state_t;


typedef enum {
    MODBUS_CMD_READ_COILS = 0x01,
    MODBUS_CMD_READ_DISCRETE_INPUTS = 0x02,
    MODBUS_CMD_READ_HOLDING_REGISTERS = 0x03,
    MODBUS_CMD_READ_INPUT_REGISTERS = 0x04,
    MODBUS_CMD_WRITE_SINGLE_COIL = 0x05,
    MODBUS_CMD_WRITE_SINGLE_REGISTER = 0x06,
    MODBUS_CMD_WRITE_MULTIPLE_COILS = 0x15,
    MODBUS_CMD_WRITE_MULTIPLE_REGISTERS = 0x16,
    MODBUS_CMD_CUSTOM_EXEC_DALI = 0x44,
    MODBUS_CMD_CUSTOM_START_PROCESS = 0x45,
} modbus_cmd_t;
  
typedef enum {
    MODBUS_ERR_ILLEGAL_FUNCTION = 0x01,
    MODBUS_ERR_ILLEGAL_DATA_ADDR = 0x02,
    MODBUS_ERR_ILLEGAL_DATA_VALUE = 0x03,
    MODBUS_ERR_SLAVE_DEVICE_FAIL = 0x04,
    MODBUS_ERR_ACK = 0x05,
    MODBUS_ERR_SLAVE_DEVICE_BUSY = 0x06,
    MODBUS_ERR_NACK = 0x07,
    MODBUS_ERR_MEMORY_PARITY_ERROR = 0x08,
    MODBUS_ERR_GATEWAY_PATH_UNAVAILABLE = 0x0a,
    MODBUS_ERR_GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND = 0x0b,
} modbus_err_t;

typedef void (*modbus_task_cb)(modbus_task_state_t state, uint8_t *cmd, uint8_t *response, size_t sz);


void modbus_init(int tx_pin, int rx_pin, int cs_pin);
void modbus_poll();
void modbus_downstream_set_coil(uint8_t devaddr, uint16_t coil_num, uint16_t value, modbus_task_cb cb);

int modbus_expected_length(uint8_t *buf, size_t sz);
void onError();
void toggleLED();
void setLED(bool on);



#endif
