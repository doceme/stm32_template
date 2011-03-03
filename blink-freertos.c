/**
 ******************************************************************************
 *
 * @file       blink-freertos.c
 * @author     Stephen Caudle Copyright (C) 2010
 * @brief      Blink code implemented with FreeRTOS
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

/* FreeRTOS includes */
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

/* Includes */
#include "common.h"
#include "tprintf.h"
#include "blink.h"

#define MS_PER_SEC		1000
#define TPRINTF_QUEUE_SIZE	16

static void NVIC_Configuration(void);

static void main_task(void *pvParameters) NORETURN;
static xQueueHandle tprintf_queue;

/**
 * @brief  Retargets the C library printf function to the USART.
 * @param  ch The character to print
 * @retval The character printed
 */
int outbyte(int ch)
{
	/* Enable USART TXE interrupt */
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
	xQueueSendToBack(tprintf_queue, &ch, 0);
	return ch;
}

void blink_start(void)
{
	xTaskHandle main_task_handle;

	/* Create the main task */
	xTaskCreate(main_task, (signed portCHAR *)"main", configMINIMAL_STACK_SIZE , NULL, tskIDLE_PRIORITY + 1, &main_task_handle);
	assert_param(main_task_handle);

	/* Start the FreeRTOS scheduler */
	vTaskStartScheduler();

	assert_param(NULL);

	while (1);
}

/**
  * @brief  Configure the nested vectored interrupt controller.
  * @param  None
  * @retval None
  */
void NVIC_Configuration(void)
{
	NVIC_InitTypeDef nvic_init;

	/* Configure USART interrupt */
	nvic_init.NVIC_IRQChannel = USART1_IRQn;
	nvic_init.NVIC_IRQChannelPreemptionPriority = 0xf;
	nvic_init.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic_init);
}

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

void main_task(void *pvParameters)
{
	portTickType last_wake = xTaskGetTickCount();;

	NVIC_Configuration();

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
		blink_toggle();
		tprintf("tick=%d\r\n", last_wake);
	}
}
