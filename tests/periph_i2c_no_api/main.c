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

#include "periph_conf.h"
#include "periph/i2c.h"
#include "shell.h"

#include "sc_args.h"

#define DEFAULT_DEV_STR "0"
#define DEFAULT_ADDR_STR "85"
#define DEFAULT_REG_STR "0"
#define DEFAULT_FLAG_STR "0"

#define DEFAULT_VAL_1_1 "21"

#define DEFAULT_VAL_2_1 "31"
#define DEFAULT_VAL_2_2 "32"

#define DEFAULT_VAL_3_1 "41"
#define DEFAULT_VAL_3_2 "42"
#define DEFAULT_VAL_3_3 "43"
#define DEFAULT_VAL_3_4 "44"
#define DEFAULT_VAL_3_5 "45"
#define DEFAULT_VAL_3_6 "46"
#define DEFAULT_VAL_3_7 "47"
#define DEFAULT_VAL_3_8 "48"
#define DEFAULT_VAL_3_9 "49"
#define DEFAULT_VAL_3_10 "50"

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

int cmd_i2c_write_regs(int argc, char **argv)
{
    for (int i = 0; i < argc; i++) {
        puts(argv[i]);
    }
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

int main(void)
{
    char * i2c_acquire_args[] = {"i2c_acquire", DEFAULT_DEV_STR};
    char * i2c_release_args[] = {"i2c_release", DEFAULT_DEV_STR};




    char * one_byte_should_succeed_args[] = {"i2c_write_regs",
                                DEFAULT_DEV_STR,
                                DEFAULT_ADDR_STR,
                                DEFAULT_REG_STR,
                                DEFAULT_FLAG_STR,
                                DEFAULT_VAL_1_1};
    cmd_i2c_acquire(sizeof(i2c_acquire_args)/sizeof(char *), i2c_acquire_args);
    cmd_i2c_write_regs(sizeof(one_byte_should_succeed_args)/sizeof(char *), one_byte_should_succeed_args);
    cmd_i2c_release(sizeof(i2c_release_args)/sizeof(char *), i2c_release_args);

#if (AMOUNT_OF_TEST_CASES > 1)
    char * two_bytes_should_succeed_args[] = {"i2c_write_regs",
                                DEFAULT_DEV_STR,
                                DEFAULT_ADDR_STR,
                                DEFAULT_REG_STR,
                                DEFAULT_FLAG_STR,
                                DEFAULT_VAL_2_1,
                                DEFAULT_VAL_2_2};
    cmd_i2c_acquire(sizeof(i2c_acquire_args)/sizeof(char *), i2c_acquire_args);
    cmd_i2c_write_regs(sizeof(two_bytes_should_succeed_args)/sizeof(char *), two_bytes_should_succeed_args);
    cmd_i2c_release(sizeof(i2c_release_args)/sizeof(char *), i2c_release_args);
#endif

#if (AMOUNT_OF_TEST_CASES > 2)
    char * ten_bytes_should_succeed_args[] = {"i2c_write_regs",
                                DEFAULT_DEV_STR,
                                DEFAULT_ADDR_STR,
                                DEFAULT_REG_STR,
                                DEFAULT_FLAG_STR,
                                DEFAULT_VAL_3_1,
                                DEFAULT_VAL_3_2,
                                DEFAULT_VAL_3_3,
                                DEFAULT_VAL_3_4,
                                DEFAULT_VAL_3_5,
                                DEFAULT_VAL_3_6,
                                DEFAULT_VAL_3_7,
                                DEFAULT_VAL_3_8,
                                DEFAULT_VAL_3_9,
                                DEFAULT_VAL_3_10};

    cmd_i2c_acquire(sizeof(i2c_acquire_args)/sizeof(char *), i2c_acquire_args);
    cmd_i2c_write_regs(sizeof(ten_bytes_should_succeed_args)/sizeof(char *), ten_bytes_should_succeed_args);
    cmd_i2c_release(sizeof(i2c_release_args)/sizeof(char *), i2c_release_args);
#endif
    while (1);
    return 0;
}
