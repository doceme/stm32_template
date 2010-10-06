/**
 ******************************************************************************
 *
 * @file       common.h
 * @author     Stephen Caudle Copyright (C) 2010.
 * @brief      Main header.
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


#ifndef COMMON_H
#define COMMON_H

#define SUCCESS(e) (e >= 0)
#define FAILURE(e) (e < 0)

#define ERR_GENERIC	1 /* Generic error */
#define ERR_EXIST	2 /* Already exists */
#define ERR_NOINIT	3 /* Not initialized */
#define ERR_NOIMP	4 /* Not implemented */
#define ERR_PARAM	5 /* Invalid parameter */
#define ERR_TIMEOUT	6 /* Timeout */
#define ERR_OVERFLOW	7 /* Overflow */
#define ERR_NOCONNECT	8 /* Not connected */

#define IRQ_PRIO_LOW				12		// lower than RTOS
#define IRQ_PRIO_MID				8		// higher than RTOS
#define IRQ_PRIO_HIGH				5		// for SPI, ADC, I2C etc...
#define IRQ_PRIO_HIGHEST			4 		// for USART etc...

#define NORETURN __attribute__((noreturn))

#endif /* COMMON_H */

