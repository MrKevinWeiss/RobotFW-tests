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
#include <errno.h>

#include "periph_conf.h"
#include "periph/i2c.h"

#include "stdio_uart.h"

#ifndef I2C_ACK
#define I2C_ACK         (0)
#endif

#define IN_BUFSIZE         (10U)
#define OUT_BUFSIZE         (128U)
#define PERIPH_I2C_MIN_FW_ID 42

/* i2c_buf is global to reduce stack memory consumtion */
static uint8_t in_buf[IN_BUFSIZE];
static uint8_t out_buf[OUT_BUFSIZE];

enum I2C_ENUM {
    I2C_ACQUIRE,
    I2C_RELEASE,
    I2C_READ_REG,
    I2C_READ_REGS,
    I2C_READ_BYTE,
    I2C_READ_BYTES,
    I2C_WRITE_BYTE,
    I2C_WRITE_BYTES,
    I2C_WRITE_REG,
    I2C_WRITE_REGS,
    I2C_GET_DEVS,
    GET_METADATA,
};

void _read_buf(uint8_t* buf, size_t size) {
    size_t index = 0;
    uint32_t timeout = 0;
    while (index < size) {
        if (index) {
            timeout++;
            if (timeout == 0xFFFF) {
                timeout = 0;
                index = 0;
            }
        }
        index += stdio_read(buf + index, size);
    }
}

int main(void)
{
    while (1) {

        _read_buf(in_buf, IN_BUFSIZE);

        out_buf[0] = 0x80;
        size_t index = 1;
        uint8_t dev = in_buf[1];
        uint16_t addr = in_buf[2] + ((uint16_t)in_buf[3] << 8);
        uint16_t reg = in_buf[4] + ((uint16_t)in_buf[5] << 8);
        uint16_t len = in_buf[6] + ((uint16_t)in_buf[7] << 8);
        uint8_t flags = in_buf[8];
        uint8_t write_byte = in_buf[9];
        int res = -1;

        switch (in_buf[0]) {
            case I2C_ACQUIRE:
                res = i2c_acquire(dev);
                index = 1;
                break;
            case I2C_RELEASE:
                i2c_release(dev);
                res = 0;
                index = 1;
                break;
            case I2C_READ_REG:
                res = i2c_read_reg(dev, addr, reg, &out_buf[1], flags);
                index = 2;
                break;
            case I2C_READ_REGS:
                res = i2c_read_regs(dev, addr, reg, &out_buf[1], len, flags);
                index = 1+len;
                break;
            case I2C_READ_BYTE:
                res = i2c_read_byte(dev, addr, &out_buf[1], flags);
                index = 2;
                break;
            case I2C_READ_BYTES:
                res = i2c_read_bytes(dev, addr, &out_buf[1], len, flags);
                index = 1+len;
                break;
            case I2C_GET_DEVS:
                res = 0;
                out_buf[1] = I2C_NUMOF;
                index = 2;
                break;
            case GET_METADATA:
                res = 0;
                out_buf[1] = PERIPH_I2C_MIN_FW_ID;

                index = 2;
                break;
            case I2C_WRITE_BYTE:
                res = i2c_write_byte(dev, addr, write_byte, flags);
                index = 1;
                break;
            case I2C_WRITE_REG:
                res = i2c_write_reg(dev, addr, reg, write_byte, flags);
                index = 1;
                break;
            case I2C_WRITE_BYTES:
                index = 0;
                _read_buf(out_buf, len);
                res = i2c_write_bytes(dev, addr, out_buf, len, flags);
                index = 1;
                break;
            case I2C_WRITE_REGS:
                _read_buf(out_buf, len);
                res = i2c_write_regs(dev, addr, reg, out_buf, len, flags);
                index = 1;
                break;
            default:
                break;
        }
        if (res == -EIO) {
            out_buf[0] = 0x81;
        }
        else if (res == -ENXIO) {
            out_buf[0] = 0x82;
        }
        else {
            out_buf[0] = res;
        }
        stdio_write(out_buf, index);
    }

    return 0;
}
