include ../Makefile.tests_common

FEATURES_REQUIRED = periph_i2c

FEATURES_REQUIRED += periph_spi

FEATURES_REQUIRED += periph_timer
FEATURES_REQUIRED += periph_gpio

FEATURES_REQUIRED += periph_uart
FEATURES_OPTIONAL += periph_lpuart  # STM32 L0 and L4 provides lpuart support
FEATURES_OPTIONAL += periph_uart_modecfg

USEMODULE += shell
USEMODULE += xtimer

export HIL_I2C_DEV

export HIL_SPI_DEV

export HIL_PERIPH_TIMER_DEV
export HIL_PERIPH_TIMER_HZ

ifeq ($(HIL_UART_DEV), )
$(error ERROR: There is no specified uart device. This test requires 2 or more uarts)
endif

export HIL_UART_DEV

export HIL_DUT_GPIO0_PORT
export HIL_DUT_GPIO0_PIN
export HIL_DUT_GPIO1_PORT
export HIL_DUT_GPIO1_PIN
export HIL_DUT_GPIO2_PORT
export HIL_DUT_GPIO2_PIN
export HIL_DUT_RX_PORT
export HIL_DUT_RX_PIN
export HIL_DUT_TX_PORT
export HIL_DUT_TX_PIN
export HIL_DUT_NSS_PORT
export HIL_DUT_NSS_PIN
export HIL_DUT_SCK_PORT
export HIL_DUT_SCK_PIN
export HIL_DUT_MISO_PORT
export HIL_DUT_MISO_PIN
export HIL_DUT_MOSI_PORT
export HIL_DUT_MOSI_PIN
export HIL_DUT_SCL_PORT
export HIL_DUT_SCL_PIN
export HIL_DUT_SDA_PORT
export HIL_DUT_SDA_PIN
export HIL_DUT_CTS_PORT
export HIL_DUT_CTS_PIN
export HIL_DUT_RTS_PORT
export HIL_DUT_RTS_PIN
export HIL_DUT_PWM_PORT
export HIL_DUT_PWM_PIN
export HIL_DUT_DAC_PORT
export HIL_DUT_DAC_PIN
export HIL_DUT_ADC_PORT
export HIL_DUT_ADC_PIN
export HIL_DUT_IC_PORT
export HIL_DUT_IC_PIN

include $(RIOTBASE)/Makefile.include
