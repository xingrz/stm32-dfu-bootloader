#pragma once

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#define rcc_gpio_enable(gpion) \
	rcc_peripheral_enable_clock(&RCC_APB2ENR, (1 << (gpion + 2)));

#define GPIO_PORT_N(n) (GPIO_PORT_A_BASE + n * 0x400)
#define GPIO_PIN_N(n) (1 << n)

#if defined(ENABLE_GPIO_DFU_BOOT) && defined(GPIO_DFU_BOOT_CUSTOM)
int force_dfu_gpio();
#endif
