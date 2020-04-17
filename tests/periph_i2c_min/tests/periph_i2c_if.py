#! /usr/bin/env python3
# Copyright (C) 2018 Kevin Weiss <kevin.weiss@haw-hamburg.de>
#
# This file is subject to the terms and conditions of the GNU Lesser
# General Public License v2.1. See the file LICENSE in the top level
# directory for more details.
"""@package PyToAPI
This module handles parsing of information from RIOT periph_i2c test.
"""
import logging

from serial import Serial


class PeriphI2cIf():
    """Interface to the a node with periph_i2c firmware."""

    CMDS = {"I2C_ACQUIRE": 0,
            "I2C_RELEASE": 1,
            "I2C_READ_REG": 2,
            "I2C_READ_REGS": 3,
            "I2C_READ_BYTE": 4,
            "I2C_READ_BYTES": 5,
            "I2C_WRITE_BYTE": 6,
            "I2C_WRITE_BYTES": 7,
            "I2C_WRITE_REG": 8,
            "I2C_WRITE_REGS": 9,
            "I2C_GET_DEVS": 10,
            "GET_METADATA": 11}
    FW_ID = 42
    DEFAULT_DEV = 0
    DEFAULT_ADDR = 85
    DEFAULT_REG = 0
    DEFAULT_LEN = 1
    DEFAULT_DATA = 0
    DEFAULT_FLAGS = 0
    DEFAULT_PORT = "/dev/ttyACM0"

    def __init__(self, port=DEFAULT_PORT):
        self.logger = logging.getLogger(self.__class__.__name__)
        self.dev = Serial(port, baudrate=115200, timeout=1)


    def _send_cmd(self, cmd, dev=DEFAULT_DEV, addr=DEFAULT_ADDR, reg=DEFAULT_REG,
                  leng=DEFAULT_LEN, send_data=DEFAULT_DATA, flags=DEFAULT_FLAGS):
        self.logger.debug("_send_cmd(%r)", cmd)
        dat = send_data
        if isinstance(send_data, list):
            leng = len(send_data)
            dat = 0

        send_list = [self.CMDS[cmd], dev,
                     addr & 0xFF, addr >> 8,
                     reg & 0xFF, reg >> 8,
                     leng & 0xFF, leng >> 8,
                     flags, dat]
        self.logger.debug("send_list=%r", send_list)
        self.logger.debug("send_bytes=%r", bytearray(send_list))
        self.dev.reset_input_buffer()
        self.dev.reset_output_buffer()
        self.dev.write(bytearray(send_list))

        if isinstance(send_data, list):
            self.logger.debug("send_data=%r", bytearray(send_data))
            self.dev.write(send_data)
        expected_bytes = {"I2C_ACQUIRE": 1,
                          "I2C_RELEASE": 1,
                          "I2C_READ_REG": 2,
                          "I2C_READ_REGS": 1+leng,
                          "I2C_READ_BYTE": 2,
                          "I2C_READ_BYTES": 1+leng,
                          "I2C_WRITE_BYTE": 1,
                          "I2C_WRITE_BYTES": 1,
                          "I2C_WRITE_REG": 1,
                          "I2C_WRITE_REGS": 1,
                          "I2C_GET_DEVS": 2,
                          "GET_METADATA": 2}

        bytes_read = list(self.dev.read(size=expected_bytes[cmd]))
        self.logger.debug("read_bytes=%r", bytes_read)
        cmd_info = {"cmd": send_list}
        if len(bytes_read) == 0:
            cmd_info['result'] = "Timeout"
        elif bytes_read[0] == 0:
            cmd_info['result'] = "Success"
            if len(bytes_read) > 1:
                cmd_info['data'] = bytes_read[1:]
        else:
            cmd_info['result'] = "Error"
            if bytes_read[0] == 0x81:
                cmd_info['msg'] = "EIO"
            if bytes_read[0] == 0x82:
                cmd_info['msg'] = "ENXIO"
        self.logger.debug("cmd_info=%r", cmd_info)
        return cmd_info

    def i2c_acquire(self, dev=DEFAULT_DEV):
        """Get access to the I2C bus."""
        return self._send_cmd("I2C_ACQUIRE", dev=dev)

    def i2c_release(self, dev=DEFAULT_DEV):
        """Release to the I2C bus."""
        return self._send_cmd("I2C_RELEASE", dev=dev)

    def i2c_read_reg(self, dev=DEFAULT_DEV, addr=DEFAULT_ADDR, reg=DEFAULT_REG,
                     flag=0):
        """Read byte from register."""
        return self._send_cmd("I2C_READ_REG", dev=dev, addr=addr, reg=reg, flags=flag)

    def i2c_read_regs(self, dev=DEFAULT_DEV, addr=DEFAULT_ADDR, reg=DEFAULT_REG,
                      leng=DEFAULT_LEN, flag=0):
        """Read bytes from registers."""
        return self._send_cmd("I2C_READ_REGS", dev=dev, addr=addr, reg=reg, leng=leng, flags=flag)

    def i2c_read_byte(self, dev=DEFAULT_DEV, addr=DEFAULT_ADDR, flag=0):
        """Read byte from the I2C device."""
        return self._send_cmd("I2C_READ_BYTE", dev=dev, addr=addr, flags=flag)

    def i2c_read_bytes(self, dev=DEFAULT_DEV, addr=DEFAULT_ADDR, leng=DEFAULT_LEN,
                       flag=0):
        """Read bytes from the I2C device."""
        return self._send_cmd("I2C_READ_BYTES", dev=dev, addr=addr, leng=leng, flags=flag)

    def i2c_write_reg(self, dev=DEFAULT_DEV, addr=DEFAULT_ADDR, reg=DEFAULT_REG,
                      data=DEFAULT_DATA, flag=0):
        """Write byte to the I2C device."""
        y = data
        if isinstance(data, list):
            y = data[0]
        if isinstance(data, str):
            y = int(data)
        return self._send_cmd("I2C_WRITE_REG", dev=dev, addr=addr, reg=reg, send_data=y, flags=flag)

    def i2c_write_regs(self, dev=DEFAULT_DEV, addr=DEFAULT_ADDR, reg=DEFAULT_REG,
                       data=DEFAULT_DATA, flag=0):
        """Write byte to register."""
        y = data
        if isinstance(y, int):
            y = [y]
        if isinstance(data, str):
            y = []
            x = data.split(" ")
            for _ in range(len(x)):
                y.append(int(x[_]))

        return self._send_cmd("I2C_WRITE_REGS", dev=dev, addr=addr, reg=reg, send_data=y, flags=flag)

    def i2c_write_byte(self, dev=DEFAULT_DEV, addr=DEFAULT_ADDR, data=DEFAULT_DATA,
                       flag=0):
        """Write bytes to registers."""
        y = data
        if isinstance(y, list):
            y = y[0]
        if isinstance(y, str):
            y = int(y)

        return self._send_cmd("I2C_WRITE_BYTE", dev=dev, addr=addr, send_data=y, flags=flag)

    def i2c_write_bytes(self, dev=DEFAULT_DEV, addr=DEFAULT_ADDR,
                        data=DEFAULT_DATA, flag=0):
        """Write bytes to registers."""
        y = data
        if isinstance(data, int):
            data = [data]
        if isinstance(data, str):
            y = []
            x = data.split(" ")
            for _ in range(len(x)):
                y.append(int(x[_]))

        return self._send_cmd("I2C_WRITE_BYTES", dev=dev, addr=addr, send_data=y, flags=flag)

    def i2c_get_devs(self):
        """Gets amount of supported i2c devices."""
        return self._send_cmd("I2C_GET_DEVS")

    def get_metadata(self):
        """Get the metadata of the firmware."""
        res = self._send_cmd("GET_METADATA")
        if "data" in res:
            if res['data'][0] == 42:
                res['msg'] = "tests_periph_i2c_min"
        else:
            res['result'] = "Timeout"
        return res

    def get_command_list(self):
        """List of all commands."""
        cmds = list()
        cmds.append(self.get_metadata)
        cmds.append(self.i2c_get_devs)
        cmds.append(self.i2c_acquire)
        cmds.append(self.i2c_read_reg)
        cmds.append(self.i2c_read_regs)
        cmds.append(self.i2c_read_byte)

        cmds.append(self.i2c_read_bytes)
        cmds.append(self.i2c_write_reg)
        cmds.append(self.i2c_write_regs)
        cmds.append(self.i2c_write_byte)
        cmds.append(self.i2c_write_bytes)
        cmds.append(self.i2c_release)
        return cmds


def main():
    """Test for PeriphI2cIf."""

    logging.getLogger("PeriphI2cIf").setLevel(logging.DEBUG)
    i2c = PeriphI2cIf()
    print(i2c.get_metadata())
    print(i2c.get_metadata())
    print(i2c.get_metadata())
    print(i2c.get_metadata())
    print(i2c.get_metadata())
    return 0
    try:
        i2c = PeriphI2cIf()
        cmds = i2c.get_command_list()
        logging.debug("======================================================")
        for cmd in cmds:
            cmd()
            logging.debug("--------------------------------------------------")
        logging.debug("======================================================")
    except Exception as exc:
        logging.debug(exc)


if __name__ == "__main__":
    main()
