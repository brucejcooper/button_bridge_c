#include "log.h"
#include <stdarg.h>
#include <stdio.h>
#include <pico/mutex.h>


auto_init_mutex(my_mutex);

static char buf[10240];
static char *ptr = buf;


void defer_log(const char *tag, const char *fmt, ...) {
    mutex_enter_blocking(&my_mutex);
    int len = ptr - buf;

    if (len < (sizeof(buf)-50)) {
        ptr += sprintf(ptr, "%s: ", tag);
        va_list args;
        va_start(args, fmt);
        int num = vsprintf(ptr, fmt, args);
        va_end(args);
        ptr += num;
        *ptr++ = '\n';
        *ptr = 0;
    }
    mutex_exit(&my_mutex);

}

void flush_log() {
    mutex_enter_blocking(&my_mutex);
    if (ptr > buf) {
        printf("%s", buf);
        ptr = buf;
    }
    mutex_exit(&my_mutex);

}
