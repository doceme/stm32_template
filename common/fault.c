/*
 * Fault handlers, stolen from FreeRTOS web (www.FreeRTOS.org)
 *
 * 2009-2010 Michal Demin
 *
 */
#include "stm32f10x.h"

void mem_manage_handler(void) __attribute__((naked));
void bus_fault_handler(void) __attribute__ ((naked));
void usage_fault_handler(void) __attribute__ ((naked));

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

#define assert_break() do { if (CoreDebug->DHCSR & 1) { __asm volatile("bkpt 0"); } while(1); } while (0)

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

void hard_fault_handler(void)
{
	assert_break();
}

void mem_manage_handler(void)
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

void bus_fault_handler(void)
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

void usage_fault_handler(void)
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
