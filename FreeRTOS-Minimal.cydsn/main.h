#ifndef _MAIN_H_
    #define _MAIN_H_
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
#include <project.h>
/* RTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

    #define _DEBUGGING_ 1  /* set to 0 for production*/
    

    #define VERSION_STRING ("Version 1.0, FreeRTOS Basic Implementation\n\r" __DATE__" " __TIME__ "\n\r")
    
    // the comma ',' in front of the enum definition allows for easy insertion
    // of new enums.
enum TaskList {
    TERMINAL_DEBUG_TASK=0
    ,LED_FLASH_TASK
    ,USB_SERIAL_TASK
    ,UART_SERIAL_TASK
    ,CONFIG_MANAGER_TASK    // handle permanent configuration data into EEPROM
    ,NUMBER_OF_TASKS // add task ID's before this enum value
};
    
/* The time between cycles of the 'check' functionality (defined within the
tick hook. */
#define mainCOM_LED							( 3 )

//-----------------------------------------------------
/* The number of nano seconds between each processor clock. */
//-----------------------------------------------------
#define mainNS_PER_CLOCK ( ( unsigned long ) ( ( 1.0 / ( double ) configCPU_CLOCK_HZ ) * 1000000000.0 ) )

//-----------------------------------------------------
/* Task priorities. see  FreeRTOSConfig.h*/
//-----------------------------------------------------
#define mainTERMINAL_DEBUG_TASK_PRIORITY    ( tskIDLE_PRIORITY + 4 ) //highest priority, allow us to look around
#define mainUSBSERIAL_TASK_PRIORITY	        ( tskIDLE_PRIORITY + 4 )//highest priority, allow to look around
#define mainUART_SERIAL_TASK_Priority       ( tskIDLE_PRIORITY + 3 )// move this around as needed
#define mainFLASH_TEST_TASK_PRIORITY		( tskIDLE_PRIORITY + 0 )// lowest priority


//-----------------------------------------------------
// the following structure allows for coarse profiling
typedef struct task_info {
    TaskHandle_t taskHandle;   
    uint32_t     runCounter; // number of times has executed for loop.
} TaskInfo_t;

extern TaskInfo_t taskInfo[NUMBER_OF_TASKS];


// defines used many places
// YES and NO are basically TRUE and FALSE
// But can be more intuitive.
#ifndef YES
    #define YES pdTRUE
#endif
#ifndef NO
    #define NO pdFALSE
#endif

#ifndef true
    #define true pdTRUE
#endif
#ifndef false
    #define false pdFALSE
#endif

#endif
/* [] END OF FILE */
