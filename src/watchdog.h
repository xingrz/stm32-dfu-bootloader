#pragma once

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/iwdg.h>

#ifdef ENABLE_WATCHDOG

// Enables the watchdog using a period of 1/(40Khz / 256 / 4095) = 26.2s
static void enable_iwdg(uint16_t rldval) {
	// First start LSI oscillator
	rcc_osc_on(RCC_LSI);
	rcc_wait_for_osc_ready(RCC_LSI);

	while (iwdg_prescaler_busy());
	IWDG_KR  = IWDG_KR_UNLOCK;  // Unlock PR/RLR
	IWDG_PR  = 7;       // 256 prescaler

	while (iwdg_reload_busy());
	IWDG_KR  = IWDG_KR_UNLOCK;  // Unlock PR/RLR
	IWDG_RLR = rldval;  // 4095 reload value

	// Starts the watchdog
	iwdg_reset();
}

static int reset_due_to_watchdog() {
	return (RCC_CSR & RCC_CSR_IWDGRSTF);
}

#endif
