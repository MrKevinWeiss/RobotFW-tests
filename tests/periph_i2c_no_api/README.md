# Periph I2C Test

This application enables you to test all available low-level I2C functions
using the RIOT shell. The `robot-test` feature requires a target board to be
connected to the hardware reference board PHiLIP.

## Automated Test Setup

Before executing the automated test, the DUT and reference hardware PHiLIP
must be setup.

### Robot Framework Setup

Follow the [setup instructions](../../README.md) and install all python
packages needed.

### PHiLIP Setup

To setup PHiLIP follow instructions on the
[PHiLIP repo](https://github.com/riot-appstore/PHiLIP#setup).

### DUT and PHiLIP Wiring Setup

Each test will need to be wired in a specific way. To wire the DUT follow
the [guide to board pinouts](http://doc.riot-os.org/group__boards.html#pinout_guide).
To wire PHiLIP check the [pinout](https://github.com/riot-appstore/PHiLIP#pinb)
for the PHiLIP version used.

This test will require that both `sda` and `scl` pins of the `i2c.dev 0` to
be wired to the `DUT_SDA` and `DUT_SCL` pins on PHiLIP. If the device has a
reset pin then the optional `DUT_RST` pin on PHiLIP can be connected.

DUT pin name | PHiLIP pin name | Required
-------------|-----------------|---------
sda          | DUT_SDA         | Yes
scl          | DUT_SCL         | Yes
nrst         | DUT_RST         | No
gnd          | GND             | Yes _(sometimes usb grounds are enough)_


## Running Automated Tests

After the setup is complete use the following command to execute the robot test:

`PHILIP_PORT=<philip_serial_port> BOARD=<DUT_BOARD_NAME> make flash robot-test`

### Configuring run parameters

A number of settings can be adjusted such as changing timeouts or using the
PHiLIP reset instead of using `make reset`. This information can be found at
in the [robot framework make overrides](../README.md).
