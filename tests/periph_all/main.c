/*
 * Copyright (C) 2014 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup tests
 * @{
 *
 * @file
 * @brief       Test application for the low-level I2C peripheral driver
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Kevin Weiss <kevin.weiss@haw-hamburg.de>
 *
 * @}
 */

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <stdbool.h>

#include "periph_conf.h"
#include "periph/timer.h"
#include "periph/i2c.h"
#include "periph/gpio.h"
#include "periph/uart.h"
#include "stdio_uart.h"
#include "xtimer.h"
#include "mutex.h"
#include "thread.h"
#include "msg.h"
#include "ringbuffer.h"
#include "shell.h"

#include "sc_args.h"

static int set(int argc, char **argv)
{
    if (argc < 3) {
        printf("usage: %s <port> <pin>\n", argv[0]);
        return 1;
    }
    gpio_init(GPIO_PIN(atoi(argv[1]), atoi(argv[2])), GPIO_OUT);
    gpio_set(GPIO_PIN(atoi(argv[1]), atoi(argv[2])));
    printf("Success: Pin set\n");
    return 0;
}

static int clear(int argc, char **argv)
{
    if (argc < 3) {
        printf("usage: %s <port> <pin>\n", argv[0]);
        return 1;
    }
    gpio_init(GPIO_PIN(atoi(argv[1]), atoi(argv[2])), GPIO_OUT);
    gpio_clear(GPIO_PIN(atoi(argv[1]), atoi(argv[2])));
    printf("Success: Pin cleared\n");
    return 0;
}

int cmd_lock(int argc, char **argv)
{
    (void)argv;
    (void)argc;

    while(1);
    return 0;
}





#define ARG_ERROR       (-1)
#define CONVERT_ERROR   (-32768)
#define RESULT_OK       (0)
#define RESULT_ERROR    (-1)
#define INVALID_ARGS    puts("Error: Invalid number of arguments")
#define PARSE_ERROR     puts("Error: unable to parse arguments")

#define CB_TOGGLE_STR   "cb_toggle"
#define CB_HIGH_STR     "cb_high"
#define CB_LOW_STR      "cb_low"


static mutex_t cb_mutex;
static gpio_t debug_pins[TIMER_NUMOF];

static inline void _debug_toogle(gpio_t pin)
{
    if (pin != GPIO_UNDEF) {
        gpio_toggle(pin);
    }
}

static inline void _debug_set(gpio_t pin)
{
    if (pin != GPIO_UNDEF) {
        gpio_set(pin);
    }
}

static inline void _debug_clear(gpio_t pin)
{
    if (pin != GPIO_UNDEF) {
        gpio_clear(pin);
    }
}

static int _print_cmd_result(const char *cmd, bool success, int ret,
                             bool print_ret)
{
    printf("%s: %s()", success ? "Success" : "Error", cmd);

    if (print_ret) {
        printf(": [%d]", ret);
    }

    printf("\n");

    return success ? RESULT_OK : RESULT_ERROR;
}

void cb_toggle(void *arg, int channel)
{
    (void)channel;
    gpio_t pin = (gpio_t)(intptr_t)arg;
    _debug_toogle(pin);
    mutex_unlock(&cb_mutex);
}

void cb_high(void *arg, int channel)
{
    (void)channel;
    gpio_t pin = (gpio_t)(intptr_t)arg;
    _debug_set(pin);
    mutex_unlock(&cb_mutex);
}

void cb_low(void *arg, int channel)
{
    (void)channel;
    gpio_t pin = (gpio_t)(intptr_t)arg;
    _debug_clear(pin);
    mutex_unlock(&cb_mutex);
}

/* API calls */

int cmd_timer_init(int argc, char **argv)
{
    if (sc_args_check(argc, argv, 3, 3, "DEV FREQ CALLBACK") != ARGS_OK) {
        return ARGS_ERROR;
    }

    int dev = sc_arg2dev(argv[1], TIMER_NUMOF);
    if (dev < 0) {
        return -ENODEV;
    }

    long freq = 0;
    if (sc_arg2long(argv[2], &freq) != ARGS_OK) {
        return ARGS_ERROR;
    }

    timer_cb_t cb = NULL;
    if (strncmp(CB_TOGGLE_STR, argv[3], strlen(argv[3])) == 0) {
        cb = cb_toggle;
    }
    else if (strncmp(CB_HIGH_STR, argv[3], strlen(argv[3])) == 0) {
        cb = cb_high;
    }
    else if (strncmp(CB_LOW_STR, argv[3], strlen(argv[3])) == 0) {
        cb = cb_low;
    }
    else {
        printf("no valid callback name given. Valid values are %s, %s or %s\n",
               CB_TOGGLE_STR, CB_HIGH_STR, CB_LOW_STR);
        return ARGS_ERROR;
    }

    int res = timer_init(dev, freq, cb, (void*)(intptr_t)debug_pins[dev]);

    return _print_cmd_result("timer_init", res == 0, res, true);
}

int _timer_set(int argc, char **argv, bool absolute)
{
    if (sc_args_check(argc, argv, 3, 3, "DEV CHANNEL TICKS") != ARGS_OK) {
        return ARGS_ERROR;
    }

    int dev = sc_arg2dev(argv[1], TIMER_NUMOF);
    if (dev < 0) {
        return -ENODEV;
    }

    int chan = 0;
    if (sc_arg2int(argv[2], &chan) != ARGS_OK) {
        return ARGS_ERROR;
    }

    unsigned int timeout = 0;
    if (sc_arg2uint(argv[3], &timeout) != ARGS_OK) {
        return ARGS_ERROR;
    }

    int res = 0;
    mutex_lock(&cb_mutex);

    _debug_toogle(debug_pins[dev]);
    if (absolute) {
        res = timer_set_absolute(dev, chan, timeout);
    }
    else {
        res = timer_set(dev, chan, timeout);
    }

    /* wait for unlock by cb */
    mutex_lock(&cb_mutex);

    /* reset mutex state */
    mutex_unlock(&cb_mutex);
    return res;
}

int cmd_timer_set(int argc, char **argv)
{
    int res = _timer_set(argc, argv, false);
    return _print_cmd_result("timer_set", (res == 0), res, true);
}

int cmd_timer_set_absolute(int argc, char **argv)
{
    int res = _timer_set(argc, argv, true);
    return _print_cmd_result("timer_set_absolute", (res == 0), res, true);
}

int cmd_timer_clear(int argc, char **argv)
{
    if (sc_args_check(argc, argv, 2, 2, "DEV CHANNEL") != ARGS_OK) {
        return ARGS_ERROR;
    }

    int dev = sc_arg2dev(argv[1], TIMER_NUMOF);
    if (dev < 0) {
        return -ENODEV;
    }

    int chan = 0;
    if (sc_arg2int(argv[2], &chan) != ARGS_OK) {
        return ARGS_ERROR;
    }

    int res = timer_clear(dev, chan);

    return _print_cmd_result("timer_clear", (res == 0), res, true);
}

int cmd_timer_read(int argc, char **argv)
{
    if (sc_args_check(argc, argv, 1, 1, "DEV") != ARGS_OK) {
        return ARGS_ERROR;
    }

    int dev = sc_arg2dev(argv[1], TIMER_NUMOF);
    if (dev < 0) {
        return -ENODEV;
    }

    printf("Success: timer_read(): [%u]\n", timer_read(dev));
    return RESULT_OK;
}

int cmd_timer_start(int argc, char **argv)
{
    if (sc_args_check(argc, argv, 1, 1, "DEV") != ARGS_OK) {
        return ARGS_ERROR;
    }

    int dev = sc_arg2dev(argv[1], TIMER_NUMOF);
    if (dev < 0) {
        return -ENODEV;
    }

    timer_start(dev);
    return _print_cmd_result("timer_start", true, 0, false);
}

int cmd_timer_stop(int argc, char **argv)
{
    if (sc_args_check(argc, argv, 1, 1, "DEV") != ARGS_OK) {
        return ARGS_ERROR;
    }

    int dev = sc_arg2dev(argv[1], TIMER_NUMOF);
    if (dev < 0) {
        return -ENODEV;
    }

    timer_stop(dev);
    return _print_cmd_result("timer_stop", true, 0, false);
}

/* helper calls (non-API) */

int cmd_timer_debug_pin(int argc, char **argv)
{
    if (sc_args_check(argc, argv, 3, 3, "DEV PORT PIN") != ARGS_OK) {
        return ARGS_ERROR;
    }

    int dev = sc_arg2dev(argv[1], TIMER_NUMOF);
    if (dev < 0) {
        return -ENODEV;
    }

    /* parse and init debug pin */
    uint32_t port, pin = 0;
    if ((sc_arg2u32(argv[2], &port) != ARGS_OK) ||
        (sc_arg2u32(argv[3], &pin) != ARGS_OK)) {
        return _print_cmd_result("timer_debug_pin", false, 1, false);
    }

    debug_pins[dev] = GPIO_PIN(port, pin);
    gpio_init(debug_pins[dev], GPIO_OUT);

    return _print_cmd_result("timer_debug_pin", true, 0, false);
}

int cmd_timer_bench_read(int argc, char **argv)
{
    if (sc_args_check(argc, argv, 2, 2, "DEV REPEAT") != ARGS_OK) {
        return ARGS_ERROR;
    }

    int dev = sc_arg2dev(argv[1], TIMER_NUMOF);
    if (dev < 0) {
        return -ENODEV;
    }

    unsigned int repeat = 0;
    if (sc_arg2uint(argv[2], &repeat) != ARGS_OK) {
        return ARGS_ERROR;
    }

    _debug_toogle(debug_pins[dev]);

    for (unsigned int i = 0; i < repeat; i++) {
        timer_read(dev);
    }

    _debug_toogle(debug_pins[dev]);

    return _print_cmd_result("cmd_timer_read_bench", true, 0, false);
}










#define SHELL_BUFSIZE       (128U)
#define UART_BUFSIZE        (128U)

#define PRINTER_PRIO        (THREAD_PRIORITY_MAIN - 1)
#define PRINTER_TYPE        (0xabcd)

#define POWEROFF_DELAY      (250U * US_PER_MS)      /* quarter of a second */

#ifndef STDIO_UART_DEV
#define STDIO_UART_DEV      (UART_UNDEF)
#endif

typedef struct {
    char rx_mem[UART_BUFSIZE];
    ringbuffer_t rx_buf;
} uart_ctx_t;

static uart_ctx_t ctx[UART_NUMOF];

static kernel_pid_t printer_pid;
static char printer_stack[THREAD_STACKSIZE_MAIN];

#ifdef MODULE_PERIPH_UART_MODECFG
static uart_data_bits_t data_bits_lut[] = { UART_DATA_BITS_5, UART_DATA_BITS_6,
                                            UART_DATA_BITS_7, UART_DATA_BITS_8 };
static int data_bits_lut_len = sizeof(data_bits_lut)/sizeof(data_bits_lut[0]);

static uart_stop_bits_t stop_bits_lut[] = { UART_STOP_BITS_1, UART_STOP_BITS_2 };
static int stop_bits_lut_len = sizeof(stop_bits_lut)/sizeof(stop_bits_lut[0]);
#endif

static void rx_cb(void *arg, uint8_t data)
{
    uart_t dev = (uart_t)arg;

    ringbuffer_add_one(&(ctx[dev].rx_buf), data);
    if (data == '\n') {
        msg_t msg;
        msg.content.value = (uint32_t)dev;
        msg_send(&msg, printer_pid);
    }
}

static void *printer(void *arg)
{
    (void)arg;
    msg_t msg;
    msg_t msg_queue[8];
    msg_init_queue(msg_queue, 8);

    while (1) {
        msg_receive(&msg);
        uart_t dev = (uart_t)msg.content.value;
        char c;

        printf("Success: UART_DEV(%i) RX: [", dev);
        do {
            c = (int)ringbuffer_get_one(&(ctx[dev].rx_buf));
            if (c == '\n') {
                puts("]\\n");
            }
            else if (c >= ' ' && c <= '~') {
                printf("%c", c);
            }
            else {
                printf("0x%02x", (unsigned char)c);
            }
        } while (c != '\n');
    }

    /* this should never be reached */
    return NULL;
}

static void sleep_test(int num, uart_t uart)
{
    printf("UARD_DEV(%i): test uart_poweron() and uart_poweroff()  ->  ", num);
    uart_poweroff(uart);
    xtimer_usleep(POWEROFF_DELAY);
    uart_poweron(uart);
    puts("[OK]");
}

static int cmd_uart_init(int argc, char **argv)
{
    int res = sc_args_check(argc, argv, 2, 2, "DEV BAUD");
    if (res != ARGS_OK) {
        return 1;
    }

    int dev = sc_arg2dev(argv[1], UART_NUMOF);
    if ((dev < 0) || (UART_DEV(dev) == STDIO_UART_DEV)){
        return -ENODEV;
    }

    uint32_t baud = 0;
    if (sc_arg2u32(argv[2], &baud) != ARGS_OK) {
        return 1;
    }

    /* initialize UART */
    res = uart_init(UART_DEV(dev), baud, rx_cb, (void *)dev);
    if (res == UART_NOBAUD) {
        printf("Error: Given baudrate (%u) not possible\n", (unsigned int)baud);
        return 1;
    }
    else if (res != UART_OK) {
        puts("Error: Unable to initialize UART device");
        return 1;
    }
    printf("Success: Initialized UART_DEV(%i) at BAUD %"PRIu32"\n", dev, baud);

    /* also test if poweron() and poweroff() work (or at least don't break
     * anything) */
    sleep_test(dev, UART_DEV(dev));

    return 0;
}

#ifdef MODULE_PERIPH_UART_MODECFG
static int cmd_uart_mode(int argc, char **argv)
{
    uart_data_bits_t data_bits;
    uart_parity_t  parity;
    uart_stop_bits_t  stop_bits;

    int res = sc_args_check(argc, argv, 4, 4, "DEV DATA_BITS PARITY STOP_BITS");
    if (res != ARGS_OK) {
        return 1;
    }

    int dev = sc_arg2dev(argv[1], UART_NUMOF);
    if ((dev < 0) || (UART_DEV(dev) == STDIO_UART_DEV)){
        return -ENODEV;
    }

    int data_bits_arg = 0;
    if (sc_arg2int(argv[2], &data_bits_arg) != ARGS_OK) {
        return 1;
    }
    data_bits_arg -= 5;
    if (data_bits_arg >= 0 && data_bits_arg < data_bits_lut_len) {
        data_bits = data_bits_lut[data_bits_arg];
    }
    else {
        printf("Error: Invalid number of data_bits (%i).\n", data_bits_arg + 5);
        return 1;
    }

    argv[3][0] &= ~0x20;
    switch (argv[3][0]) {
        case 'N':
            parity = UART_PARITY_NONE;
            break;
        case 'E':
            parity = UART_PARITY_EVEN;
            break;
        case 'O':
            parity = UART_PARITY_ODD;
            break;
        case 'M':
            parity = UART_PARITY_MARK;
            break;
        case 'S':
            parity = UART_PARITY_SPACE;
            break;
        default:
            printf("Error: Invalid parity (%c).\n", argv[3][0]);
            return 1;
    }

    int stop_bits_arg = 0;
    if (sc_arg2int(argv[4], &stop_bits_arg) != ARGS_OK) {
        return 1;
    }
    stop_bits_arg -= 1;

    if (stop_bits_arg >= 0 && stop_bits_arg < stop_bits_lut_len) {
        stop_bits = stop_bits_lut[stop_bits_arg];
    }
    else {
        printf("Error: Invalid number of stop bits (%i).\n", stop_bits_arg + 1);
        return 1;
    }

    if (uart_mode(UART_DEV(dev), data_bits, parity, stop_bits) != UART_OK) {
        puts("Error: Unable to apply UART settings");
        return 1;
    }
    printf("Success: Successfully applied UART_DEV(%i) settings\n", dev);

    return 0;
}
#endif /* MODULE_PERIPH_UART_MODECFG */

static int cmd_uart_write(int argc, char **argv)
{
    int res = sc_args_check(argc, argv, 2, 2, "DEV DATA");
    if (res != ARGS_OK) {
        return 1;
    }

    int dev = sc_arg2dev(argv[1], UART_NUMOF);
    if ((dev < 0) || (UART_DEV(dev) == STDIO_UART_DEV)){
        return -ENODEV;
    }

    printf("UART_DEV(%i) TX: %s\n", dev, argv[2]);
    uint8_t endline = (uint8_t)'\n';
    uart_write(UART_DEV(dev), (uint8_t *)argv[2], strlen(argv[2]));
    uart_write(UART_DEV(dev), &endline, 1);
    return 0;
}









#ifndef I2C_ACK
#define I2C_ACK         (0)
#endif

#define BUFSIZE         (128U)

/* i2c_buf is global to reduce stack memory consumtion */
static uint8_t i2c_buf[BUFSIZE];

static inline void _print_i2c_read(i2c_t dev, uint16_t *reg, uint8_t *buf, int len)
{
    printf("Success: i2c_%i read %i byte(s) ", dev, len);
    if (reg != NULL) {
        printf("from reg 0x%02x ", *reg);
    }
    printf(": [");
    for (int i = 0; i < len; i++) {
        if (i != 0) {
            printf(", ");
        }
        printf("0x%02x", buf[i]);
    }
    printf("]\n");
}

static int _print_i2c_error(int res)
{
    if (res == -EOPNOTSUPP) {
        printf("Error: EOPNOTSUPP [%d]\n", -res);
        return 1;
    }
    else if (res == -EINVAL) {
        printf("Error: EINVAL [%d]\n", -res);
        return 1;
    }
    else if (res == -EAGAIN) {
        printf("Error: EAGAIN [%d]\n", -res);
        return 1;
    }
    else if (res == -ENXIO) {
        printf("Error: ENXIO [%d]\n", -res);
        return 1;
    }
    else if (res == -EIO) {
        printf("Error: EIO [%d]\n", -res);
        return 1;
    }
    else if (res == -ETIMEDOUT) {
        printf("Error: ETIMEDOUT [%d]\n", -res);
        return 1;
    }
    else if (res == I2C_ACK) {
        printf("Success: I2C_ACK [%d]\n", res);
        return 0;
    }
    printf("Error: Unknown error [%d]\n", res);
    return 1;
}

int cmd_i2c_acquire(int argc, char **argv)
{

    int res = sc_args_check(argc, argv, 1, 1, "DEV");
    if (res != ARGS_OK) {
        return 1;
    }

    int dev = sc_arg2dev(argv[1], I2C_NUMOF);
    if (dev < 0) {
        return -ENODEV;
    }
    printf("Command: i2c_acquire(%i)\n", dev);
    res = i2c_acquire(dev);
    if (res == I2C_ACK) {
        printf("Success: i2c_%i acquired\n", dev);
        return 0;
    }
    return _print_i2c_error(res);
}

int cmd_i2c_release(int argc, char **argv)
{
    int res = sc_args_check(argc, argv, 1, 1, "DEV");
    if (res != ARGS_OK) {
        return 1;
    }

    int dev = sc_arg2dev(argv[1], I2C_NUMOF);
    if (dev < 0) {
        return -ENODEV;
    }

    printf("Command: i2c_release(%i)\n", dev);
    i2c_release(dev);

    printf("Success: i2c_%i released\n", dev);
    return 0;
}

int cmd_i2c_read_reg(int argc, char **argv)
{
    int res = sc_args_check(argc, argv, 4, 4, "DEV ADDR REG FLAG");
    if (res != ARGS_OK) {
        return 1;
    }

    int dev = sc_arg2dev(argv[1], I2C_NUMOF);
    if (dev < 0) {
        return -ENODEV;
    }

    uint16_t addr = 0;
    uint16_t reg = 0;
    uint8_t flags = 0;

    if (sc_arg2u16(argv[2], &addr) != ARGS_OK
        || sc_arg2u16(argv[3], &reg) != ARGS_OK
        || sc_arg2u8(argv[4], &flags) != ARGS_OK) {
        return 1;
    }

    printf("Command: i2c_read_reg(%i, 0x%02x, 0x%02x, 0x%02x)\n",
           dev, addr, reg, flags);
    uint8_t data;
    res = i2c_read_reg(dev, addr, reg, &data, flags);

    if (res == I2C_ACK) {
        _print_i2c_read(dev, &reg, &data, 1);
        return 0;
    }
    return _print_i2c_error(res);
}

int cmd_i2c_read_regs(int argc, char **argv)
{
    int res = sc_args_check(argc, argv, 5, 5, "DEV ADDR REG LEN FLAG");
    if (res != ARGS_OK) {
        return 1;
    }

    int dev = sc_arg2dev(argv[1], I2C_NUMOF);
    if (dev < 0) {
        return -ENODEV;
    }

    uint16_t addr = 0;
    uint16_t reg = 0;
    int len = 0;
    uint8_t flags = 0;

    if (sc_arg2u16(argv[2], &addr) != ARGS_OK
        || sc_arg2u16(argv[3], &reg) != ARGS_OK
        || sc_arg2int(argv[4], &len) != ARGS_OK
        || sc_arg2u8(argv[5], &flags) != ARGS_OK) {
        return 1;
    }

    if (len < 1 || len > (int)BUFSIZE) {
        puts("Error: invalid LENGTH parameter given");
        return 1;
    }
    else {
        printf("Command: i2c_read_regs(%i, 0x%02x, 0x%02x, %i, 0x%02x)\n",
               dev, addr, reg, len, flags);
        res = i2c_read_regs(dev, addr, reg, i2c_buf, len, flags);
    }

    if (res == I2C_ACK) {
        _print_i2c_read(dev, &reg, i2c_buf, len);
        return 0;
    }
    return _print_i2c_error(res);
}

int cmd_i2c_read_byte(int argc, char **argv)
{
    int res = sc_args_check(argc, argv, 3, 3, "DEV ADDR FLAG");
    if (res != ARGS_OK) {
        return 1;
    }

    int dev = sc_arg2dev(argv[1], I2C_NUMOF);
    if (dev < 0) {
        return -ENODEV;
    }

    uint16_t addr = 0;
    uint8_t flags = 0;

    if (sc_arg2u16(argv[2], &addr) != ARGS_OK
        || sc_arg2u8(argv[3], &flags) != ARGS_OK) {
        return 1;
    }

    printf("Command: i2c_read_byte(%i, 0x%02x, 0x%02x)\n", dev, addr, flags);
    uint8_t data;
    res = i2c_read_byte(dev, addr, &data, flags);

    if (res == I2C_ACK) {
        _print_i2c_read(dev, NULL, &data, 1);
        return 0;
    }
    return _print_i2c_error(res);
}

int cmd_i2c_read_bytes(int argc, char **argv)
{
    int res = sc_args_check(argc, argv, 4, 4, "DEV ADDR LENGTH FLAG");
    if (res != ARGS_OK) {
        return 1;
    }

    int dev = sc_arg2dev(argv[1], I2C_NUMOF);
    if (dev < 0) {
        return -ENODEV;
    }

    uint16_t addr = 0;
    int len = 0;
    uint8_t flags = 0;

    if (sc_arg2u16(argv[2], &addr) != ARGS_OK
        || sc_arg2int(argv[3], &len) != ARGS_OK
        || sc_arg2u8(argv[4], &flags) != ARGS_OK) {
        return 1;
    }

    if (len < 1 || len > (int)BUFSIZE) {
        puts("Error: invalid LENGTH parameter given");
        return 1;
    }
    else {
        printf("Command: i2c_read_bytes(%i, 0x%02x, %i, 0x%02x)\n", dev,
         addr, len, flags);
        res = i2c_read_bytes(dev, addr, i2c_buf, len, flags);
    }

    if (res == I2C_ACK) {
        _print_i2c_read(dev, NULL, i2c_buf, len);
        return 0;
    }
    return _print_i2c_error(res);
}

int cmd_i2c_write_byte(int argc, char **argv)
{
    int res = sc_args_check(argc, argv, 4, 4, "DEV ADDR BYTE FLAG");
    if (res != ARGS_OK) {
        return 1;
    }

    int dev = sc_arg2dev(argv[1], I2C_NUMOF);
    if (dev < 0) {
        return -ENODEV;
    }

    uint16_t addr = 0;
    uint8_t data = 0;
    uint8_t flags = 0;

    if (sc_arg2u16(argv[2], &addr) != ARGS_OK
        || sc_arg2u8(argv[3], &data) != ARGS_OK
        || sc_arg2u8(argv[4], &flags) != ARGS_OK) {
        return 1;
    }

    printf("Command: i2c_write_byte(%i, 0x%02x, 0x%02x, [0x%02x",
           dev, addr, flags, data);
    puts("])");
    res = i2c_write_byte(dev, addr, data, flags);

    if (res == I2C_ACK) {
        printf("Success: i2c_%i wrote 1 byte to the bus\n", dev);
        return 0;
    }
    return _print_i2c_error(res);
}

int cmd_i2c_write_bytes(int argc, char **argv)
{
    int res = sc_args_check(argc, argv, 4, 3 + BUFSIZE,
                            "DEV ADDR FLAG BYTE0 [BYTE1 [BYTE_n [...]]]");
    if (res != ARGS_OK) {
        return 1;
    }

    int dev = sc_arg2dev(argv[1], I2C_NUMOF);
    if (dev < 0) {
        return -ENODEV;
    }

    uint16_t addr = 0;;
    uint8_t flags = 0;

    if (sc_arg2u16(argv[2], &addr) != ARGS_OK
        || sc_arg2u8(argv[3], &flags) != ARGS_OK) {
        return 1;
    }
    int len = argc - 4;
    for (int i = 0; i < len; i++) {
        if (sc_arg2u8(argv[i + 4], &i2c_buf[i]) != ARGS_OK) {
            return 1;
        }
    }

    printf("Command: i2c_write_bytes(%i, 0x%02x, 0x%02x, [", dev, addr, flags);
    for (int i = 0; i < (argc - 4); i++) {
        if (i != 0) {
            printf(", ");
        }
        printf("0x%02x", i2c_buf[i]);
    }
    puts("])");
    res = i2c_write_bytes(dev, addr, i2c_buf, len, flags);

    if (res == I2C_ACK) {
        printf("Success: i2c_%i wrote %i bytes\n", dev, len);
        return 0;
    }
    return _print_i2c_error(res);
}

int cmd_i2c_write_reg(int argc, char **argv)
{
    int res = sc_args_check(argc, argv, 5, 5, "DEV ADDR REG BYTE FLAG");
    if (res != ARGS_OK) {
        return 1;
    }

    int dev = sc_arg2dev(argv[1], I2C_NUMOF);
    if (dev < 0) {
        return -ENODEV;
    }

    uint16_t addr = 0;
    uint16_t reg = 0;
    uint8_t data = 0;
    uint8_t flags = 0;

    if (sc_arg2u16(argv[2], &addr) != ARGS_OK
        || sc_arg2u16(argv[3], &reg) != ARGS_OK
        || sc_arg2u8(argv[4], &data) != ARGS_OK
        || sc_arg2u8(argv[5], &flags) != ARGS_OK) {
        return 1;
    }

    printf("Command: i2c_write_reg(%i, 0x%02x, 0x%02x, 0x%02x, [0x%02x",
           dev, addr, reg, flags, data);
    puts("])");
    res = i2c_write_reg(dev, addr, reg, data, flags);

    if (res == I2C_ACK) {
        printf("Success: i2c_%i wrote 1 byte\n", dev);
        return 0;
    }
    return _print_i2c_error(res);
}

int cmd_i2c_write_regs(int argc, char **argv)
{
    int res = sc_args_check(argc, argv, 5, 4 + BUFSIZE, "DEV ADDR REG FLAG BYTE0 [BYTE1 ...]");
    if (res != ARGS_OK) {
        return 1;
    }

    int dev = sc_arg2dev(argv[1], I2C_NUMOF);
    if (dev < 0) {
        return -ENODEV;
    }

    uint16_t addr = 0;
    uint16_t reg = 0;
    uint8_t flags = 0;

    if (sc_arg2u16(argv[2], &addr) != ARGS_OK
        || sc_arg2u16(argv[3], &reg) != ARGS_OK
        || sc_arg2u8(argv[4], &flags) != ARGS_OK) {
        return 1;
    }
    int len = argc - 5;
    for (int i = 0; i < len; i++) {
        if (sc_arg2u8(argv[i + 5], &i2c_buf[i]) != ARGS_OK) {
            return 1;
        }
    }

    printf("Command: i2c_write_regs(%i, 0x%02x, 0x%02x, 0x%02x, [",
           dev, addr, reg, flags);
    for (int i = 0; i < len; i++) {
        if (i != 0) {
            printf(", ");
        }
        printf("0x%02x", i2c_buf[i]);
    }
    puts("])");
    res = i2c_write_regs(dev, addr, reg, i2c_buf, len, flags);

    if (res == I2C_ACK) {
        printf("Success: i2c_%i wrote %i bytes to reg 0x%02x\n",
            dev, len, reg);
        return 0;
    }
    return _print_i2c_error(res);
}

int cmd_i2c_get_devs(int argc, char **argv)
{
    (void)argv;
    (void)argc;

    printf("Command: return I2C_NUMOF\n");
    printf("Success: Amount of i2c devices: [%d]\n", I2C_NUMOF);
    return 0;
}




int cmd_xtimer_now(int argc, char **argv)
{
    (void)argv;
    (void)argc;

    uint32_t now = xtimer_now().ticks32;
    printf("Success: xtimer_now(): [%"PRIu32"]\n", now);
    return 0;
}




int cmd_get_metadata(int argc, char **argv)
{
    (void)argv;
    (void)argc;

    printf("Success: [%s, %s]\n", RIOT_BOARD, RIOT_APPLICATION);

    return 0;
}

static const shell_command_t shell_commands[] = {
    { "i2c_acquire", "Get access to the I2C bus", cmd_i2c_acquire },
    { "i2c_release", "Release to the I2C bus", cmd_i2c_release },
    { "i2c_read_reg", "Read byte from register", cmd_i2c_read_reg },
    { "i2c_read_regs", "Read bytes from registers", cmd_i2c_read_regs },
    { "i2c_read_byte", "Read byte from the I2C device", cmd_i2c_read_byte },
    { "i2c_read_bytes", "Read bytes from the I2C device", cmd_i2c_read_bytes },
    { "i2c_write_byte", "Write byte to the I2C device", cmd_i2c_write_byte },
    { "i2c_write_bytes", "Write bytes to the I2C device", cmd_i2c_write_bytes },
    { "i2c_write_reg", "Write byte to register", cmd_i2c_write_reg },
    { "i2c_write_regs", "Write bytes to registers", cmd_i2c_write_regs },
    { "i2c_get_devs", "Gets amount of supported i2c devices", cmd_i2c_get_devs },
    { "gpio_set", "set pin to HIGH", set },
    { "gpio_clear", "set pin to LOW", clear },
    { "lock", "Lock the device", cmd_lock},
    { "timer_init", "Initialize timer device", cmd_timer_init },
    { "timer_set", "set timer to relative value", cmd_timer_set },
    { "timer_set_absolute", "set timer to absolute value",
      cmd_timer_set_absolute },
    { "timer_clear", "clear timer", cmd_timer_clear },
    { "timer_read", "read timer", cmd_timer_read },
    { "timer_start", "start timer", cmd_timer_start },
    { "timer_stop", "stop timer", cmd_timer_stop },
    { "timer_debug_pin", "config debug pin", cmd_timer_debug_pin },
    { "timer_read_bench", "execute multiple reads to determine overhead",
      cmd_timer_bench_read },
    { "uart_init", "Initialize a UART device with a given baudrate", cmd_uart_init },
#ifdef MODULE_PERIPH_UART_MODECFG
    { "uart_mode", "Setup data bits, stop bits and parity for a given UART device", cmd_uart_mode },
#endif
    { "uart_write", "Send a buffer through given UART device", cmd_uart_write },
    { "xtimer_now", "Get number of ticks (32Bit) from xtimer", cmd_xtimer_now },
    { "get_metadata", "Get the metadata of the test firmware", cmd_get_metadata },

    { NULL, NULL, NULL }
};

int main(void)
{
    puts("Test all periphs");

    /* initialize ringbuffers */
    for (unsigned i = 0; i < UART_NUMOF; i++) {
        ringbuffer_init(&(ctx[i].rx_buf), ctx[i].rx_mem, UART_BUFSIZE);
    }

    /* start the printer thread */
    printer_pid = thread_create(printer_stack, sizeof(printer_stack),
                                PRINTER_PRIO, 0, printer, NULL, "printer");

    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);

    return 0;
}
