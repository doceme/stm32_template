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
#include "tprintf.h"

#define MS_PER_SEC		1000
#define DEBOUNCE_DELAY		40
#ifdef FREERTOS
#define TPRINTF_QUEUE_SIZE	16
#endif

/* Function Prototypes */
static void setup_rcc(void);
static void setup_gpio(void);
static void setup_exti(void);
#ifndef FREERTOS
static void setup_rtc(void);
static void setup_timer(void);
#endif
static void setup_usart(void);
static void setup_nvic(void);
static void main_noreturn(void) NORETURN;

#ifdef FREERTOS
static void blink_task(void *pvParameters) NORETURN;
static void debounce_task(void *pvParameters) NORETURN;
#endif
static void setup(void);
static void blink_toggle_blue(void);
static void blink_toggle_green(void);

enum button_state
{
	BUTTON_STATE_UP,
	BUTTON_STATE_DOWN
};

#ifdef FREERTOS
static xQueueHandle tprintf_queue;
static xSemaphoreHandle debounce_sem;
#else
static uint32_t tick;
static uint8_t debounce;
#endif

static enum button_state button_state;
static uint8_t led_blue = 1;
static uint8_t led_green = 1;

/**
 * @brief  Retargets the C library printf function to the USART.
 * @param  ch The character to print
 * @retval The character printed
 */
int outbyte(int ch)
{
#ifdef FREERTOS
	/* Enable USART TXE interrupt */
	xQueueSendToBack(tprintf_queue, &ch, 0);
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
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
	xTaskHandle task;

	/* Create the blink task */
	xTaskCreate(blink_task, (signed portCHAR *)"blink", configMINIMAL_STACK_SIZE , NULL, tskIDLE_PRIORITY + 1, &task);
	assert_param(task);

	/* Start the FreeRTOS scheduler */
	vTaskStartScheduler();
	assert_param(NULL);
#else
	setup();
#endif

	while (1);
}

static inline void setup()
{
	setup_rcc();
	setup_gpio();
	setup_exti();
#ifndef FREERTOS
	setup_rtc();
	setup_timer();
#endif
	setup_usart();
	setup_nvic();

	tprintf("stm32_template\r\n");
}

/**
 * @brief  Configures the peripheral clocks
 * @param  None
 * @retval None
 */
void setup_rcc(void)
{
	/* Enable PWR clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR |
#ifndef FREERTOS
			RCC_APB1Periph_TIM4 |
#endif
			RCC_APB1Periph_BKP, ENABLE);

	/* Enable GPIOA and GPIOC clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |
			RCC_APB2Periph_GPIOC |
			RCC_APB2Periph_USART1 |
			RCC_APB2Periph_AFIO, ENABLE);
}

/**
 * @brief  Configures the different GPIO ports
 * @param  None
 * @retval None
 */
void setup_gpio(void)
{
	GPIO_InitTypeDef gpio_init;

	/* Configure UART tx pin */
	gpio_init.GPIO_Pin = GPIO_Pin_9;
	gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpio_init);

	/* Configure PC5 (ADC Channel15) as analog input */
	gpio_init.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &gpio_init);

	/* Configure button input floating */
	gpio_init.GPIO_Pin = GPIO_Pin_0;
	gpio_init.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &gpio_init);

	/* Connect Button EXTI Line to Button GPIO Pin */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);
}

/**
  * @brief  Configures EXTI Lines
  * @param  None
  * @retval None
  */
void setup_exti(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;

	EXTI_ClearITPendingBit(EXTI_Line0);
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
}

#ifndef FREERTOS
/**
  * @brief  Configures RTC clock source and prescaler
  * @param  None
  * @retval None
  */
void setup_rtc(void)
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
  * @brief  Configures timer controller
  * @param  None
  * @retval None
  */
void setup_timer(void)
{
	TIM_TimeBaseInitTypeDef tim_init;

	/* Configure timer */
	tim_init.TIM_Prescaler = DEBOUNCE_DELAY - 1;
	tim_init.TIM_CounterMode = TIM_CounterMode_Up;
	tim_init.TIM_Period = (SystemCoreClock / 1000 ) - 1;
	tim_init.TIM_ClockDivision = 0;
	TIM_TimeBaseInit(TIM4, &tim_init);
}
#endif

/**
  * @brief  Configures USART controller
  * @param  None
  * @retval None
  */
void setup_usart(void)
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
  * @brief  Configure the nested vectored interrupt controller
  * @param  None
  * @retval None
  */
void setup_nvic(void)
{
	NVIC_InitTypeDef nvic_init;

	/* Set the Vector Table base address as specified in .ld file */
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);

	/* 4 bits for Interupt priorities so no sub priorities */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	/* Configure HCLK clock as SysTick clock source. */
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);

#ifdef FREERTOS
	/* Configure USART interrupt */
	nvic_init.NVIC_IRQChannel = USART1_IRQn;
	nvic_init.NVIC_IRQChannelPreemptionPriority = 0xf;
	nvic_init.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic_init);
#else
	SysTick_Config(SystemCoreClock / 1000);
	NVIC_SetPriority (SysTick_IRQn, 1);

	/* Configure RTC interrupt */
	nvic_init.NVIC_IRQChannel = RTC_IRQn;
	nvic_init.NVIC_IRQChannelPreemptionPriority = 2;
	nvic_init.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic_init);

	/* Configure timer interrupt */
	nvic_init.NVIC_IRQChannel = TIM4_IRQn;
	nvic_init.NVIC_IRQChannelPreemptionPriority = 3;
	nvic_init.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic_init);
#endif

	/* Configure EXTI interrupt */
	nvic_init.NVIC_IRQChannel = EXTI0_IRQn;
	nvic_init.NVIC_IRQChannelPreemptionPriority = 0xc;
	nvic_init.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic_init);
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

		if (xQueueReceiveFromISR(tprintf_queue, &ch, &task_woken))
			USART_SendData(USART1, ch);
		else
			USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
	}

	portEND_SWITCHING_ISR(task_woken);
}
#else
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

		blink_toggle_green();
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

/**
  * @brief  This function handles timer interrupt request
  * @param  None
  * @retval None
  */
void tim4_isr(void)
{
	volatile uint8_t button = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0);

	if (button_state == BUTTON_STATE_UP && button)
	{
		button_state = BUTTON_STATE_DOWN;
		blink_toggle_blue();
		tprintf("button press\r\n");
	}
	else if (button_state == BUTTON_STATE_DOWN && !button)
	{
		button_state = BUTTON_STATE_UP;
		tprintf("button release\r\n");
	}

	/* Disable the timer */
	TIM_Cmd(TIM4, DISABLE);

	debounce = 0;
}
#endif

/**
  * @brief  This function handles External line 0 interrupt request.
  * @param  None
  * @retval None
  */
void exti0_isr(void)
{
#ifdef FREERTOS
	static signed portBASE_TYPE task_woken;
#endif

	/* Clear pending bit */
	EXTI_ClearITPendingBit(EXTI_Line0);

#ifdef FREERTOS
	xSemaphoreGiveFromISR(debounce_sem, &task_woken);
#else
	/* Disable timer interrupt */
	TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);

	/* Reset timer counter */
	TIM_SetCounter(TIM4, 0);

	/* Clear the debouce timer update pending bit */
	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);

	/* Enable timer interrupt */
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

	if (!debounce)
	{
		debounce = 1;

		/* Enable the timer */
		TIM_Cmd(TIM4, ENABLE);
	}
#endif

#ifdef FREERTOS
	portEND_SWITCHING_ISR(task_woken);
#endif
}

void blink_toggle_blue()
{
	GPIO_WriteBit(GPIOC, GPIO_Pin_8, led_blue);
	led_blue ^= 1;
}

void blink_toggle_green()
{
	GPIO_WriteBit(GPIOC, GPIO_Pin_9, led_green);
	led_green ^= 1;
}

#ifdef FREERTOS
void blink_task(void *pvParameters)
{
	portTickType last_wake = xTaskGetTickCount();
	xTaskHandle task;

	/*
	 * If using FreeRTOS, tprintf must not be called until after this queue
	 * has been created
	 */
	tprintf_queue = xQueueCreate(TPRINTF_QUEUE_SIZE, sizeof(unsigned char));
	assert_param(tprintf_queue);

	vSemaphoreCreateBinary(debounce_sem);
	assert_param(debounce_sem);

	setup();

	/* Create the button task */
	xTaskCreate(debounce_task, (signed portCHAR *)"debounce", configMINIMAL_STACK_SIZE , NULL, tskIDLE_PRIORITY + 2, &task);
	assert_param(task);

	for (;;)
	{
		vTaskDelayUntil(&last_wake, (1 * MS_PER_SEC) / portTICK_RATE_MS);
		blink_toggle_green();
		tprintf("tick=%d\r\n", last_wake);
	}
}

void debounce_task(void *pvParameters)
{
	portTickType delay = portMAX_DELAY;
	uint8_t debounce = 0;

	for (;;)
	{
		if (xSemaphoreTake(debounce_sem, delay) == pdTRUE)
		{
			if (!debounce)
			{
				debounce = 1;
				delay = DEBOUNCE_DELAY;
			}
		}
		else
		{
			volatile uint8_t button = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0);

			if (button_state == BUTTON_STATE_UP && button)
			{
				button_state = BUTTON_STATE_DOWN;
				blink_toggle_blue();
				tprintf("button press\r\n");
			}
			else if (button_state == BUTTON_STATE_DOWN && !button)
			{
				button_state = BUTTON_STATE_UP;
				tprintf("button release\r\n");
			}

			debounce = 0;
			delay = portMAX_DELAY;
		}
	}
}
#endif
