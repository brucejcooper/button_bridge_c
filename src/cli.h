#ifndef _CLI_H
#define _CLI_H

typedef enum
{
    STATUS_PRINT_VALUES,
    STATUS_PRINT_DEVICE,
    STATUS_LOG,
    STATUS_LOGINT,
} log_type_t;

typedef enum
{
    MSG_SRC_DALI,
    MSG_SRC_MODBUS,
    MSG_SRC_BUTTON_FIXTURE,
} msg_src_t;

typedef struct
{
    log_type_t type;

    msg_src_t bus;
    int device;
    int address; // Sub-address when the device can have multiple entries.
    char *msg;
    int vals[3];
} log_msg_t;

void cli_init();
void cli_poll();

void print_msg(log_msg_t *cmd);
void log_i(char *msg);
void log_int(char *c, int val);

#endif