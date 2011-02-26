/*
 * Fault handlers, stolen from FreeRTOS web (www.FreeRTOS.org)
 *
 * 2009-2010 Michal Demin
 *
 */
#include "stm32f10x.h"

void MemManage_Handler(void) __attribute__((naked));
void BusFault_Handler(void) __attribute__ ((naked));
void UsageFault_Handler(void) __attribute__ ((naked));

struct stack_t {
	uint32_t r0;
	uint32_t r1;
	uint32_t r2;
	uint32_t r3;
	uint32_t r12;
	uint32_t lr;
	uint32_t pc;
	uint32_t psr;
};

static inline void assert_break(void)
{
	/* Stop debugger if attached */
	if (CoreDebug->DHCSR & 1) {
		__asm volatile("bkpt 0");
	}

	while(1);
}

void assert_failed(uint8_t *function, uint32_t line)
{
	assert_break();
}

void fault_halt(struct stack_t *stack)
{
	(void)stack;

	/* Inspect stack->pc to locate the offending instruction. */
	assert_break();
}

void MemManage_Handler(void)
{
	__asm volatile
	(
		" tst lr, #4                     \n"
		" ite eq                         \n"
		" mrseq r0, msp                  \n"
		" mrsne r0, psp                  \n"
		" ldr r1, [r0, #24]              \n"
		" ldr r2, mem_handler_const      \n"
		" bx r2                          \n"
		" mem_handler_const: .word fault_halt\n"
	);
}

void BusFault_Handler(void)
{
	__asm volatile
	(
		" tst lr, #4                     \n"
		" ite eq                         \n"
		" mrseq r0, msp                  \n"
		" mrsne r0, psp                  \n"
		" ldr r1, [r0, #24]              \n"
		" ldr r2, bus_handler_const      \n"
		" bx r2                          \n"
		" bus_handler_const: .word fault_halt\n"
	);
}

void UsageFault_Handler(void)
{
	__asm volatile
	(
		" tst lr, #4                     \n"
		" ite eq                         \n"
		" mrseq r0, msp                  \n"
		" mrsne r0, psp                  \n"
		" ldr r1, [r0, #24]              \n"
		" ldr r2, usage_handler_const     \n"
		" bx r2                          \n"
		" usage_handler_const: .word fault_halt\n"
	);
}
