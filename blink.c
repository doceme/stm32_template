/**
 ******************************************************************************
 *
 * @file       blink.c
 * @author     Stephen Caudle Copyright (C) 2010
 * @brief      Blink code without using an RTOS
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

/* Includes */
#include "common.h"
#include "blink.h"
#include "tprintf.h"

static void RTC_Configuration(void);
static void NVIC_Configuration(void);

static uint32_t tick;

/**
 * @brief  Retargets the C library printf function to the USART.
 * @param  ch The character to print
 * @retval The character printed
 */
int outbyte(int ch)
{
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
	USART_SendData(USART1, (uint8_t)ch);
	return ch;
}

void blink_start(void)
{
	RTC_Configuration();
	NVIC_Configuration();

	tprintf("stm32_template\r\n");
	while (1);
}

/**
  * @brief  Configures RTC clock source and prescaler
  * @param  None
  * @retval None
  */
void RTC_Configuration(void)
{
	/* RTC clock source configuration ------------------------------------------*/
	/* Allow access to BKP Domain */
	PWR_BackupAccessCmd(ENABLE);

	/* Reset Backup Domain */
	BKP_DeInit();

	/* Enable the LSE OSC */
	RCC_LSEConfig(RCC_LSE_ON);

	/* Wait till LSE is ready */
	while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
	{
	}

	/* Select the RTC Clock Source */
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);

	/* Enable the RTC Clock */
	RCC_RTCCLKCmd(ENABLE);

	/* RTC configuration -------------------------------------------------------*/
	/* Wait for RTC APB registers synchronisation */
	RTC_WaitForSynchro();

	/* Set the RTC time base to 1min */
	RTC_SetPrescaler(32767);

	/* Wait until last write operation on RTC registers has finished */
	RTC_WaitForLastTask();

	/* Enable the RTC Alarm interrupt */
	//RTC_ITConfig(RTC_IT_ALR, ENABLE);

	/* Enable the RTC second interrupt */
	RTC_ITConfig(RTC_IT_SEC, ENABLE);

	/* Wait until last write operation on RTC registers has finished */
	RTC_WaitForLastTask();
}

/**
  * @brief  Configure the nested vectored interrupt controller.
  * @param  None
  * @retval None
  */
void NVIC_Configuration(void)
{
	NVIC_InitTypeDef nvic_init;

	SysTick_Config(SystemCoreClock / 1000);

	/* Configure RTC interrupt */
	nvic_init.NVIC_IRQChannel = RTC_IRQn;
	nvic_init.NVIC_IRQChannelPreemptionPriority = 0;
	nvic_init.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic_init);
}

/**
  * @brief  This function handles SysTick interrupt request
  * @param  None
  * @retval None
  */
void sys_tick_handler(void)
{
	tick++;
}

/**
  * @brief  This function handles RTC interrupt request
  * @param  None
  * @retval None
  */
void rtc_isr(void)
{
	if(RTC_GetITStatus(RTC_IT_SEC) != RESET)
	{
		uint32_t counter = RTC_GetCounter();

		/* Wait until last write operation on RTC registers has finished */
		RTC_WaitForLastTask();

		/* Clear RTC Alarm interrupt pending bit */
		RTC_ClearITPendingBit(RTC_IT_SEC);

		blink_toggle();
		tprintf("tick=%d\r\n", tick);

		/* Reset RTC Counter when Time is 23:59:59 */
		if (counter == 0x00015180)
		{
			RTC_SetCounter(0);
			/* Wait until last write operation on RTC registers has finished */
			RTC_WaitForLastTask();
		}
	}
}
