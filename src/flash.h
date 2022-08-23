#pragma once

#include <libopencm3/stm32/flash.h>

static int _flash_page_is_erased(uint32_t addr) {
	volatile uint32_t *_ptr32 = (uint32_t*)addr;
	for (unsigned i = 0; i < 1024/sizeof(uint32_t); i++)
		if (_ptr32[i] != 0xffffffffU)
			return 0;
	return 1;
}

static void _flash_program_buffer(uint32_t address, uint16_t *data, unsigned len) {
	flash_wait_for_last_operation();

	// Enable programming
	FLASH_CR |= FLASH_CR_PG;

	volatile uint16_t *addr_ptr = (uint16_t*)address;
	for (unsigned i = 0; i < len/2; i++) {
		addr_ptr[i] = data[i];
		flash_wait_for_last_operation();
	}

	// Disable programming
	FLASH_CR &= ~FLASH_CR_PG;
}

#ifdef ENABLE_SAFEWRITE
static void check_do_erase() {
	// For protection reasons, we do not allow reading the flash using DFU
	// and also we make sure to wipe the entire flash on an ERASE/WRITE command
	// just to guarantee that nobody is able to extract the data by flashing a
	// stub and executing it.

	static int erased = 0;
	if (erased) return;

	/* Change usb_strings accordingly */
	const uint32_t start_addr = 0x08000000 + (FLASH_BOOTLDR_SIZE_KB*1024);
	const uint32_t end_addr   = 0x08000000 + (        FLASH_SIZE_KB*1024);
	for (uint32_t addr = start_addr; addr < end_addr; addr += 1024)
		if (!_flash_page_is_erased(addr))
			flash_erase_page(addr);

	erased = 1;
}
#endif
