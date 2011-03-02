/**
 ******************************************************************************
 *
 * @file       main.c
 * @author     Stephen Caudle Copyright (C) 2010
 * @brief      Main implementation
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

/* STM32 includes */
#include <stm32f10x.h>
#include <stm32f10x_conf.h>

#ifdef FREERTOS
/* FreeRTOS includes */
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#endif

/* Includes */
#include "common.h"
#include "blink.h"
#include "tprintf.h"

/* Function Prototypes */
static void RCC_Configuration(void);
static void GPIO_Configuration(void);
static void EXTI_Configuration(void);
static void USART_Configuration(void);
static void NVIC_Configuration(void);
static void main_noreturn(void) NORETURN;

static BitAction led_user = Bit_SET;
static BitAction led_rtc = Bit_SET;

/**
 * Main function
 */
int main(void)
{
	main_noreturn();
}

inline void main_noreturn(void)
{
	RCC_Configuration();
	GPIO_Configuration();
	EXTI_Configuration();
	NVIC_Configuration();
	USART_Configuration();

	blink_start();
	while (1);
}

/**
 * @brief  Configures the different system clocks.
 * @param  None
 * @retval None
 */
void RCC_Configuration(void)
{
	/* Enable PWR clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);

	/* Enable GPIOA and GPIOC clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |
			RCC_APB2Periph_GPIOC |
			RCC_APB2Periph_USART1 |
			RCC_APB2Periph_AFIO, ENABLE);
}

/**
 * @brief  Configures the different GPIO ports.
 * @param  None
 * @retval None
 */
void GPIO_Configuration(void)
{
	GPIO_InitTypeDef gpio_init;

	/* Configure button input floating */
	gpio_init.GPIO_Pin = GPIO_Pin_0;
	gpio_init.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpio_init);

	/* Configure UART tx pin */
	gpio_init.GPIO_Pin = GPIO_Pin_9;
	gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &gpio_init);

	/* Configure PC5 (ADC Channel15) as analog input */
	GPIO_WriteBit(GPIOC, GPIO_Pin_8 | GPIO_Pin_9, Bit_SET);
	gpio_init.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio_init.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOC, &gpio_init);

	/* Connect Button EXTI Line to Button GPIO Pin */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);
}

/**
  * @brief  Configures EXTI Lines
  * @param  None
  * @retval None
  */
void EXTI_Configuration(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;

	EXTI_ClearITPendingBit(EXTI_Line0);
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
}

/**
  * @brief  Configure the nested vectored interrupt controller.
  * @param  None
  * @retval None
  */
void NVIC_Configuration(void)
{
	NVIC_InitTypeDef nvic_init;

	/* Set the Vector Table base address as specified in .ld file */
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);

	/* 4 bits for Interupt priorities so no sub priorities */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	/* Configure HCLK clock as SysTick clock source. */
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);

	/* Configure EXTI interrupt */
	nvic_init.NVIC_IRQChannel = EXTI0_IRQn;
	nvic_init.NVIC_IRQChannelPreemptionPriority = 0xc;
	nvic_init.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic_init);
}

/**
  * @brief  Configures USART controller
  * @param  None
  * @retval None
  */
void USART_Configuration(void)
{
	USART_InitTypeDef usart_init;

	usart_init.USART_BaudRate = 115200;
	usart_init.USART_WordLength = USART_WordLength_8b;
	usart_init.USART_StopBits = 1;
	usart_init.USART_Parity = USART_Parity_No;
	usart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	usart_init.USART_Mode = USART_Mode_Tx;
	USART_Init(USART1, &usart_init);

	/* Enable the USART */
	USART_Cmd(USART1, ENABLE);
}

/**
  * @brief  This function handles External line 0 interrupt request.
  * @param  None
  * @retval None
  */
void exti0_isr(void)
{
	/* Clear pending bit */
	EXTI_ClearITPendingBit(EXTI_Line0);

	/* TODO: Add debounce logic */
	GPIO_WriteBit(GPIOC, GPIO_Pin_8, led_user);
	led_user ^= 1;
}

void blink_toggle()
{
	GPIO_WriteBit(GPIOC, GPIO_Pin_9, led_rtc);
	led_rtc ^= 1;
}
