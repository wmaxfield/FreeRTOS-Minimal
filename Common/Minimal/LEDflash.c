/*
 * FreeRTOS Kernel V10.1.1
 * Copyright (C) 2018 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/**
 * This version of flash .c is for use on systems that have limited stack space
 * and no display facilities.  The complete version can be found in the
 * Demo/Common/Full directory.
 *
 * Three tasks are created, each of which flash an LED at a different rate.  The first
 * LED flashes every 200ms, the second every 400ms, the third every 600ms.
 *
 * The LED flash tasks provide instant visual feedback.  They show that the scheduler
 * is still operational.
 *
 */


#include <stdlib.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"

#include "main.h"

#define ledSTACK_SIZE		configMINIMAL_STACK_SIZE

static volatile UBaseType_t uxFlashTaskNumber = 0;

/* The task that is created three times. */
static portTASK_FUNCTION_PROTO( vLEDFlashTask, pvParameters );

/*-----------------------------------------------------------*/
void vStartLEDFlashTask( UBaseType_t uxPriority )
{

		/* Spawn the task. */
		xTaskCreate( vLEDFlashTask, "LEDFlash", ledSTACK_SIZE, NULL, uxPriority, ( TaskHandle_t * ) &taskInfo[LED_FLASH_TASK].taskHandle );
}

/*-----------------------------------------------------------*/
// This is the lowest priority task, so if the LED is flashing
// then the system is still functioning properly (no task is stuck)
/*-----------------------------------------------------------*/
static portTASK_FUNCTION( vLEDFlashTask, pvParameters )
{
static TaskInfo_t *TaskInfo = &taskInfo[LED_FLASH_TASK];
	/* The parameters are not used. */
	( void ) pvParameters;


	for(;;)
	{
		/* Delay for half the flash period then turn the LED on. */
        // one flash per second == 500ms delay
		vTaskDelay( pdMS_TO_TICKS(500) );
        TaskInfo->runCounter++;// for debug purposes, coarse profiler information
		P2_1_LED_0_Write(!P2_1_LED_0_Read()); // invert the LED.
	}
} /*lint !e715 !e818 !e830 Function definition must be standard for task creation. */

