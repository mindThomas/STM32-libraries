/* Copyright (C) 2021- Thomas Jespersen, TKJ Electronics. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the MIT License
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the MIT License for further details.
 *
 * Contact information
 * ------------------------------------------
 * Thomas Jespersen, TKJ Electronics
 * Web      :  http://www.tkjelectronics.dk
 * e-mail   :  thomasj@tkjelectronics.dk
 * ------------------------------------------
 */
 
#ifndef PRIORITIES_H
#define PRIORITIES_H

/* OBS! This file only serves as a template file and is supposed to be copied into the project root include folder which will effectively take over the definitions in this template */

/* Define hardware interrupt priorities   (HAL_NVIC_SetPriority)
 * Lower value means higher priority
 * Interrupt priority has to be between 0 and 15
 * Remember to respect the "configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY" in FreeRTOSConfig.h, hence no hardware interrupt calling FreeRTOS functions must have a priority value lower than configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY
 */
#define HIGH_RES_TIMER_TICK_PRIORITY	0		// This timer is also used to generate ticks for any ST HAL library (eg. HAL_Delay)
#define ENCODER_TIMER_OVERFLOW_PRIORITY	1
#define INPUT_CAPTURE_INTERRUPT_PRIORITY	2
#define ADC_DMA_INTERRUPT_PRIORITY		15

// Below are periphiral priorities which uses FreeRTOS functions
#define USB_INTERRUPT_PRIORITY      5
#define UART_INTERRUPT_PRIORITY     6
#define SPI_INTERRUPT_PRIORITY      7
#define I2C_INTERRUPT_PRIORITY      8
#define TIMER_INTERRUPT_PRIORITY    9
#define IO_INTERRUPT_PRIORITY       10
#define SMBUS_INTERRUPT_PRIORITY    11



/* Define task priorities in FreeRTOS format
 * Higher values means higher priority
 * Hence low values denote low priority tasks. The idle task has priority 0 (tskIDLE_PRIORITY)
 * Task priority has to be between 0 and (configMAX_PRIORITIES-1)
 */
#define MAIN_TASK_PRIORITY				3
#define APPLICATION_TEMPLATE_PRIORITY	0
#define CPULOAD_PRIORITY				1
#define CALIBRATION_SWEEP_PRIORITY		2
#define DEBUG_MESSAGE_PRIORITY			4
#define TIMER_SOFT_CALLBACK_PRIORITY    5
#define CAN_RECEIVER_PRIORITY			9
#define UART_RECEIVER_PRIORITY			10
#define LSPC_RECEIVER_PRIORITY			11
#define LSPC_TRANSMITTER_PRIORITY		13  // important that this has the highest priority



#endif
