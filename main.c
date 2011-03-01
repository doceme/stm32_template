/**
 ******************************************************************************
 *
 * @file       discovery.c
 * @author     Stephen Caudle Copyright (C) 2010
 * @brief      Main marquee implementation
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

/* Includes */
#include <stm32f10x.h>
#include <stm32f10x_conf.h>
#include "common.h"

#ifdef FREERTOS
/* FreeRTOS includes */
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#endif

/* Other includes */
#include "tprintf.h"

#define MS_PER_SEC		1000

#ifdef FREERTOS
#define TPRINTF_QUEUE_SIZE	16
#endif

/* Function Prototypes */
static void RCC_Configuration(void);
static void GPIO_Configuration(void);
static void EXTI_Configuration(void);
#ifndef FREERTOS
static void RTC_Configuration(void);
#endif
static void USART_Configuration(void);
static void NVIC_Configuration(void);
static void main_noreturn(void) NORETURN;

#ifdef FREERTOS
static void main_task(void *pvParameters) NORETURN;
static xQueueHandle tprintf_queue;
#endif

static BitAction led_user = Bit_SET;
static BitAction led_rtc = Bit_SET;
#ifndef FREERTOS
static uint32_t tick;
#endif

/**
 * @brief  Retargets the C library printf function to the USART.
 * @param  ch The character to print
 * @retval The character printed
 */
int outbyte(int ch)
{
#ifdef FREERTOS
	/* Enable USART TXE interrupt */
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
	xQueueSendToBack(tprintf_queue, &ch, 0);
#else
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
	USART_SendData(USART1, (uint8_t)ch);
#endif

	return ch;
}


/**
 * Main function
 */
int main(void)
{
	main_noreturn();
}

inline void main_noreturn(void)
{
#ifdef FREERTOS
	xTaskHandle main_task_handle;
#endif

	RCC_Configuration();
	GPIO_Configuration();
	EXTI_Configuration();
#ifndef FREERTOS
	RTC_Configuration();
#endif
	NVIC_Configuration();
	USART_Configuration();

#ifdef FREERTOS
	/* Create the main task */
	xTaskCreate(main_task, (signed portCHAR *)"main", configMINIMAL_STACK_SIZE , NULL, tskIDLE_PRIORITY + 1, &main_task_handle);
	assert_param(main_task_handle);

	/* Start the FreeRTOS scheduler */
	vTaskStartScheduler();

	assert_param(NULL);
#else
	tprintf("stm32_template\r\n");
#endif

	while (1);
}

/**
 * @brief  Configures the different system clocks.
 * @param  None
 * @retval None
 */
void RCC_Configuration(void)
{
	SystemInit();
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

#ifndef FREERTOS
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
#endif

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

#ifndef FREERTOS
	SysTick_Config(SystemCoreClock / 1000);
#endif

	/* Configure EXTI interrupt */
	nvic_init.NVIC_IRQChannel = EXTI0_IRQn;
	nvic_init.NVIC_IRQChannelPreemptionPriority = 0xc;
	nvic_init.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic_init);

#ifdef FREERTOS
	/* Configure USART interrupt */
	nvic_init.NVIC_IRQChannel = USART1_IRQn;
	nvic_init.NVIC_IRQChannelPreemptionPriority = 0xf;
	nvic_init.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic_init);
#else
	/* Configure RTC interrupt */
	nvic_init.NVIC_IRQChannel = RTC_IRQn;
	nvic_init.NVIC_IRQChannelPreemptionPriority = 0;
	nvic_init.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic_init);
#endif
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

#ifdef FREERTOS
/**
  * @brief  This function handles USART interrupt request.
  * @param  None
  * @retval None
  */
void usart1_isr(void)
{
	portBASE_TYPE task_woken;

	if (USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
	{
		unsigned char ch;

		/* Clear pending bit */
		USART_ClearITPendingBit(USART1, USART_IT_TXE);

		if (xQueueReceiveFromISR(tprintf_queue, &ch, &task_woken))
		{
			USART_SendData(USART1, ch);

			while (xQueueReceiveFromISR(tprintf_queue, &ch, &task_woken))
			{
				while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
				USART_SendData(USART1, ch);
			}
		}

		/* Disable USART TXE interrupt */
		USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
	}

	portEND_SWITCHING_ISR(task_woken);
}
#endif

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

static inline void update_led()
{
	GPIO_WriteBit(GPIOC, GPIO_Pin_9, led_rtc);
	led_rtc ^= 1;
}

#ifndef FREERTOS
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

		update_led();
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
#endif

#ifdef FREERTOS
void main_task(void *pvParameters)
{
	portTickType last_wake = xTaskGetTickCount();;

	/*
	 * If using FreeRTOS, tprintf must not be called until after this queue
	 * has been created
	 */
	tprintf_queue = xQueueCreate(TPRINTF_QUEUE_SIZE, sizeof(unsigned char));
	assert_param(tprintf_queue);

	tprintf("stm32_template\r\n");

	for (;;)
	{
		vTaskDelayUntil(&last_wake, (1 * MS_PER_SEC) / portTICK_RATE_MS);
		update_led();
		tprintf("tick=%d\r\n", last_wake);
	}
}
#endif
