#include "gpio.h"

#define PIN_COL 14  // C0
#define PIN_ROW 8   // R3

int force_dfu_gpio() {
    rcc_periph_clock_enable(RCC_GPIOB);

    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO_PIN_N(PIN_COL));
    gpio_set(GPIOB, GPIO_PIN_N(PIN_COL));

    gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO_PIN_N(PIN_ROW));
    gpio_clear(GPIOB, GPIO_PIN_N(PIN_ROW));

    for (unsigned int i = 0; i < 512; i++)
        __asm__("nop");

    uint16_t val = gpio_get(GPIOB, GPIO_PIN_N(PIN_ROW));

    // Reset all pins
    gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO_PIN_N(PIN_COL));
    gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO_PIN_N(PIN_ROW));

    return val;
}
