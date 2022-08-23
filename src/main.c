/*
 * Copyright (C) 2018 David Guillen Fandos <david@davidgf.net>
 *
 * This program is free software: you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

#include <string.h>
#include "usb.h"
#include "reboot.h"
#include "flash.h"
#include "watchdog.h"
#include "gpio.h"

#include <libopencm3/cm3/scb.h>

/* Commands sent with wBlockNum == 0 as per ST implementation. */
#define CMD_SETADDR	0x21
#define CMD_ERASE	0x41

// Payload/app comes inmediately after Bootloader
#define APP_ADDRESS (FLASH_BASE_ADDR + (FLASH_BOOTLDR_SIZE_KB) * 1024)

// USB control data buffer
uint8_t usbd_control_buffer[DFU_TRANSFER_SIZE];

// DFU state
static enum dfu_state usbdfu_state = STATE_DFU_IDLE;
static struct {
	uint8_t buf[sizeof(usbd_control_buffer)];
	uint16_t len;
	uint32_t addr;
	uint16_t blocknum;
} prog;

// Serial number to expose via USB
static char serial_no[25];

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)
const char * const _usb_strings[5] = {
	"davidgf.net (libopencm3 based)", // iManufacturer
	"DFU bootloader [" VERSION "]", // iProduct
	serial_no, // iSerialNumber
	// Interface desc string
	/* This string is used by ST Microelectronics' DfuSe utility. */
	/* Change check_do_erase() accordingly */
	"@Internal Flash /0x08000000/"
	  STR(FLASH_BOOTLDR_SIZE_KB) "*001Ka,"
	  STR(FLASH_BOOTLDR_PAYLOAD_SIZE_KB) "*001Kg",
	// Config desc string
	"Bootloader config: "
	#ifdef ENABLE_WATCHDOG
	"WtDg[" STR(ENABLE_WATCHDOG) "s] "
	#endif
	#ifdef ENABLE_SAFEWRITE
	"SafeWr "
	#endif
	#ifdef ENABLE_PROTECTIONS
	"RDO/DBG "
	#endif
	#ifdef ENABLE_CHECKSUM
	"FW-CRC "
	#endif
};

static const char hcharset[16] = "0123456789abcdef";
static void get_dev_unique_id(char *s) {
	volatile uint8_t *unique_id = (volatile uint8_t *)DESIG_UNIQUE_ID_BASE;
	/* Fetch serial number from chip's unique ID */
	for (int i = 0; i < 24; i += 2) {
		s[i]   = hcharset[(*unique_id >> 4) & 0xF];
		s[i+1] = hcharset[*unique_id++ & 0xF];
	}
}

static uint8_t usbdfu_getstatus(uint32_t *bwPollTimeout) {
	switch (usbdfu_state) {
	case STATE_DFU_DNLOAD_SYNC:
		usbdfu_state = STATE_DFU_DNBUSY;
		*bwPollTimeout = 100;
		return DFU_STATUS_OK;
	case STATE_DFU_MANIFEST_SYNC:
		// Device will reset when read is complete.
		usbdfu_state = STATE_DFU_MANIFEST;
		return DFU_STATUS_OK;
	case STATE_DFU_ERROR:
		return STATE_DFU_ERROR;
	default:
		return DFU_STATUS_OK;
	}
}

static void usbdfu_getstatus_complete(usbd_device *usbd_dev,
		struct usb_setup_data *req) {
	(void)usbd_dev;
	(void)req;

	// Protect the flash by only writing to the valid flash area
	const uint32_t start_addr = FLASH_BASE_ADDR + (FLASH_BOOTLDR_SIZE_KB * 1024);
	const uint32_t end_addr   = FLASH_BASE_ADDR + (        FLASH_SIZE_KB * 1024);

	switch (usbdfu_state) {
	case STATE_DFU_DNBUSY:
		flash_unlock();
		if (prog.blocknum == 0) {
			switch (prog.buf[0]) {
			case CMD_ERASE: {
				#ifdef ENABLE_SAFEWRITE
				check_do_erase();
				#endif

				// Clear this page here.
				uint32_t baseaddr = *(uint32_t *)(prog.buf + 1);
				if (baseaddr >= start_addr && baseaddr + DFU_TRANSFER_SIZE <= end_addr) {
					if (!_flash_page_is_erased(baseaddr))
						flash_erase_page(baseaddr);
				}
				} break;
			case CMD_SETADDR:
				// Assuming little endian here.
				prog.addr = *(uint32_t *)(prog.buf + 1);
				break;
			}
		} else {
			#ifdef ENABLE_SAFEWRITE
			check_do_erase();
			#endif

			// From formula Address_Pointer + ((wBlockNum - 2)*wTransferSize)
			uint32_t baseaddr = prog.addr + ((prog.blocknum - 2) * DFU_TRANSFER_SIZE);

			if (baseaddr >= start_addr && baseaddr + prog.len <= end_addr) {
				// Program buffer in one go after erasing.
				if (!_flash_page_is_erased(baseaddr))
					flash_erase_page(baseaddr);
				_flash_program_buffer(baseaddr, (uint16_t*)prog.buf, prog.len);
			}
		}
		flash_lock();

		/* Jump straight to dfuDNLOAD-IDLE, skipping dfuDNLOAD-SYNC. */
		usbdfu_state = STATE_DFU_DNLOAD_IDLE;
		return;
	case STATE_DFU_MANIFEST:
		return;  // Reset placed in main loop.
	default:
		return;
	}
}

static enum usbd_request_return_codes usbdfu_control_request(
		usbd_device *usbd_dev,
		struct usb_setup_data *req, uint8_t **buf, uint16_t *len,
		usbd_control_complete_callback *complete) {
	(void)usbd_dev;
	(void)buf;
	switch (req->bRequest) {
	case DFU_DNLOAD:
		if ((len == NULL) || (*len == 0)) {
			// wLength = 0 means leave DFU
			usbdfu_state = STATE_DFU_MANIFEST_SYNC;
			return USBD_REQ_HANDLED;
		} else {
			/* Copy download data for use on GET_STATUS. */
			prog.blocknum = req->wValue;
			// Beware overflows!
			prog.len = *len;
			if (prog.len > sizeof(prog.buf))
				prog.len = sizeof(prog.buf);
			memcpy(prog.buf, usbd_control_buffer, prog.len);
			usbdfu_state = STATE_DFU_DNLOAD_SYNC;
			return USBD_REQ_HANDLED;
		}
	case DFU_CLRSTATUS:
		// Just clears errors.
		if (usbdfu_state == STATE_DFU_ERROR)
			usbdfu_state = STATE_DFU_IDLE;
		return USBD_REQ_HANDLED;
	case DFU_ABORT:
		// Abort just returns to IDLE state.
		usbdfu_state = STATE_DFU_IDLE;
		return USBD_REQ_HANDLED;
	case DFU_DETACH:
		usbdfu_state = STATE_DFU_MANIFEST;
		return USBD_REQ_HANDLED;
	case DFU_UPLOAD:
		// Send data back to host by reading the image.
		usbdfu_state = STATE_DFU_UPLOAD_IDLE;
		if (!req->wValue) {
			// Send back supported commands.
			usbd_control_buffer[0] = 0x00;
			usbd_control_buffer[1] = CMD_SETADDR;
			usbd_control_buffer[2] = CMD_ERASE;
			*len = 3;
			return USBD_REQ_HANDLED;
		} else {
			// Send back data if only if we enabled that.
			#ifndef ENABLE_DFU_UPLOAD
			usbdfu_state = STATE_DFU_ERROR;
			*len = 0;
			#else
			// From formula Address_Pointer + ((wBlockNum - 2)*wTransferSize)
			uint32_t baseaddr = prog.addr + ((req->wValue - 2) * DFU_TRANSFER_SIZE);
			const uint32_t start_addr = FLASH_BASE_ADDR + (FLASH_BOOTLDR_SIZE_KB * 1024);
			const uint32_t end_addr   = FLASH_BASE_ADDR + (        FLASH_SIZE_KB * 1024);
			if (baseaddr >= start_addr && baseaddr + DFU_TRANSFER_SIZE <= end_addr) {
				memcpy(usbd_control_buffer, (void*)baseaddr, DFU_TRANSFER_SIZE);
				*len = DFU_TRANSFER_SIZE;
			} else {
				usbdfu_state = STATE_DFU_ERROR;
				*len = 0;
			}
			#endif
		}
		return USBD_REQ_HANDLED;
	case DFU_GETSTATUS: {
		// Perfom the action and register complete callback.
		uint32_t bwPollTimeout = 0; /* 24-bit integer in DFU class spec */
		usbd_control_buffer[0] = usbdfu_getstatus(&bwPollTimeout);
		usbd_control_buffer[1] = bwPollTimeout & 0xFF;
		usbd_control_buffer[2] = (bwPollTimeout >> 8) & 0xFF;
		usbd_control_buffer[3] = (bwPollTimeout >> 16) & 0xFF;
		usbd_control_buffer[4] = usbdfu_state;
		usbd_control_buffer[5] = 0; /* iString not used here */
		*len = 6;
		*complete = usbdfu_getstatus_complete;
		return USBD_REQ_HANDLED;
		}
	case DFU_GETSTATE:
		// Return state with no state transision.
		usbd_control_buffer[0] = usbdfu_state;
		*len = 1;
		return USBD_REQ_HANDLED;
	}

	return USBD_REQ_NEXT_CALLBACK;
}

#ifdef ENABLE_GPIO_DFU_BOOT
#ifndef GPIO_DFU_BOOT_CUSTOM
int force_dfu_gpio() {
	rcc_gpio_enable(GPIO_DFU_BOOT_PORT);
	gpio_set_mode(
		GPIO_PORT_N(GPIO_DFU_BOOT_PORT),
		GPIO_MODE_INPUT,
	#ifdef GPIO_DFU_BOOT_PULL
		GPIO_CNF_INPUT_PULL_UPDOWN,
	#else
		GPIO_CNF_INPUT_FLOAT,
	#endif
		GPIO_PIN_N(GPIO_DFU_BOOT_PIN));
	#ifdef GPIO_DFU_BOOT_PULL
	#if GPIO_DFU_BOOT_PULL == 0
	gpio_clear(GPIO_PORT_N(GPIO_DFU_BOOT_PORT), GPIO_PIN_N(GPIO_DFU_BOOT_PIN));
	#else
	gpio_set(GPIO_PORT_N(GPIO_DFU_BOOT_PORT), GPIO_PIN_N(GPIO_DFU_BOOT_PIN));
	#endif
	#endif
	for (unsigned int i = 0; i < 512; i++)
		__asm__("nop");
	uint16_t val = gpio_get(GPIO_PORT_N(GPIO_DFU_BOOT_PORT), GPIO_PIN_N(GPIO_DFU_BOOT_PIN));
	gpio_set_mode(
		GPIO_PORT_N(GPIO_DFU_BOOT_PORT),
		GPIO_MODE_INPUT,
		GPIO_CNF_INPUT_FLOAT,
		GPIO_PIN_N(GPIO_DFU_BOOT_PIN));
	return val == GPIO_DFU_BOOT_VAL;
}
#endif // GPIO_DFU_BOOT_CUSTOM
#else  // ENABLE_GPIO_DFU_BOOT
#define force_dfu_gpio()  (0)
#endif // ENABLE_GPIO_DFU_BOOT

#ifdef ENABLE_PINRST_DFU_BOOT
static inline int reset_due_to_pin() {
	return (RCC_CSR & RCC_CSR_PINRSTF) &&
	       !(RCC_CSR & (RCC_CSR_LPWRRSTF | RCC_CSR_WWDGRSTF |
	       RCC_CSR_IWDGRSTF | RCC_CSR_SFTRSTF | RCC_CSR_PORRSTF));
}
#endif

int main(void) {
	/* Boot the application if it seems valid and we haven't been
	 * asked to reboot into DFU mode. This should make the CPU to
	 * boot into DFU if the user app has been erased. */

	#ifdef ENABLE_PROTECTIONS
	// Check for RDP protection, and in case it's not enabled, do it!
	if (!(FLASH_OBR & FLASH_OBR_RDPRT_EN)) {
		// Read protection NOT enabled ->

		// Unlock option bytes
		flash_unlock();
		flash_unlock_option_bytes();

		// Delete them all
		flash_erase_option_bytes();

		// Now write a pair of bytes that are complentary [RDP, nRDP]
		flash_program_option_bytes(FLASH_OPTION_BYTE_0, 0x33CC);

		// Now reset, for RDP to take effect. We should not re-enter this path
		scb_reset_system();
	}

	// Disable JTAG and SWD to prevent debugging/readout
	gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_OFF, 0);
	#endif

	const uint32_t start_addr = FLASH_BASE_ADDR + (FLASH_BOOTLDR_SIZE_KB * 1024);
	const uint32_t * const base_addr = (uint32_t*)start_addr;

	#ifdef ENABLE_CHECKSUM
	uint32_t imagesize = base_addr[0x20 / 4];
	#else
	uint32_t imagesize = 0;
	#endif

	int go_dfu = rebooted_into_dfu() ||
	#ifdef ENABLE_PINRST_DFU_BOOT
	             reset_due_to_pin() ||
	#endif
	#ifdef ENABLE_WATCHDOG
	             reset_due_to_watchdog() ||
	#endif
	             imagesize > FLASH_BOOTLDR_PAYLOAD_SIZE_KB * 1024 / 4 ||
	             force_dfu_gpio();

	RCC_CSR |= RCC_CSR_RMVF;

	if (!go_dfu &&
	   (MMIO32(APP_ADDRESS) & 0x2FFE0000) == 0x20000000) {

		// Do some simple XOR checking
		uint32_t xorv = 0;
		for (unsigned i = 0; i < imagesize; i++)
			xorv ^= base_addr[i];

		if (xorv == 0) {  // Matches!
			// Clear flags
			clear_reboot_flags();
			#ifdef ENABLE_WATCHDOG
			// Enable the watchdog
			enable_iwdg(4096 * ENABLE_WATCHDOG / 26);
			#endif
			// Set vector table base address.
			SCB_VTOR = APP_ADDRESS & 0xFFFF;
			// Initialise master stack pointer.
			__asm__ volatile("msr msp, %0"::"g"
					 (*(volatile uint32_t *)APP_ADDRESS));
			// Jump to application.
			(*(void (**)())(APP_ADDRESS + 4))();
		}
	}

	rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);

	/* Disable USB peripheral as it overrides GPIO settings */
	usb_pwdn();
	/*
	 * Vile hack to reenumerate, physically _drag_ d+ low.
	 * (need at least 2.5us to trigger usb disconnect)
	 */
	rcc_periph_clock_enable(RCC_GPIOA);
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO12);
	gpio_clear(GPIOA, GPIO12);
	for (unsigned int i = 0; i < 100000; i++)
		__asm__("nop");

	get_dev_unique_id(serial_no);
	usb_init(usbdfu_control_request);

	while (1) {
		// Poll based approach
		do_usb_poll();
		if (usbdfu_state == STATE_DFU_MANIFEST) {
			// USB device must detach, we just reset...
			clear_reboot_flags();
			scb_reset_system();
		}
	}
}

// Implement this here to save space, quite minimalistic :D
void *memcpy(void * dst, const void * src, size_t count) {
	uint8_t * dstb = (uint8_t*)dst;
	uint8_t * srcb = (uint8_t*)src;
	while (count--)
		*dstb++ = *srcb++;
	return dst;
}
