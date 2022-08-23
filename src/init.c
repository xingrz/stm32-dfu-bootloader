#include <stdint.h>
#include <libopencm3/cm3/vector.h>

// A handler that does nothing, we use no interrupts
void null_handler(void) {
	while (1);
}

/* Less common symbols exported by the linker script(s): */
typedef void (*funcp_t) (void);

void main(void);

void __attribute__ ((naked)) reset_handler(void) {
	volatile unsigned *src, *dest;

	for (src = &_data_loadaddr, dest = &_data;
		dest < &_edata;
		src++, dest++) {
		*dest = *src;
	}

	while (dest < &_ebss)
		*dest++ = 0;

	/* Ensure 8-byte alignment of stack pointer on interrupts */
	/* Enabled by default on most Cortex-M parts, but not M3 r1 */
	volatile uint32_t *_scb_ccr = (uint32_t*)0xE000ED14U;
	*_scb_ccr |= (1 << 9);

	/* Call the application's entry point. */
	main();
}

// Vector table (bare minimal one)
__attribute__ ((section(".vectors")))
vector_table_t vector_table = {
	.initial_sp_value = &_stack,
	.reset = reset_handler,
	.nmi = null_handler,
	.hard_fault = null_handler,
	.memory_manage_fault = null_handler,
	.bus_fault = null_handler,
	.usage_fault = null_handler,
	.debug_monitor = null_handler,
	.sv_call = null_handler,
	.pend_sv = null_handler,
	.systick = null_handler,
};
