/*

  Copyright 2020, Wade Maxfield
  Written by Wade Maxfield.

   This follows other works which ported the DEMO to PSOC.  This is tuned for
   the CY8CKit-059, but changing the project build CPU and the pin assignments
   will make it work for the other PSOC 5 versions.

   This version provides access to a serial port (P12[6], P12[7]) and the USBUART
   with instructions on using the USBUART on Mac, Linux, and Windows. (see
   TerminalDebug.c for instructions)

 * Use with FreeRTOS Kernel V10.1.1
 * Copyright (C) 2020. Wade Maxfield.  All Rights Reserved.
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
 

*/
#include <device.h>
#include "main.h"


/* Common Demo includes. */
#include "serial.h"
#include "USBSerial.h"
#include "printf.h"  //re-entrant

#define debugPort_putString usbserial_putString


// a task monitoring structure for each task created
TaskInfo_t taskInfo[NUMBER_OF_TASKS];


void prvHardwareSetup( void )
{
/* Port layer functions that need to be copied into the vector table. */
extern void xPortPendSVHandler( void );
extern void xPortSysTickHandler( void );
extern void vPortSVCHandler( void );
extern cyisraddress CyRamVectors[];

	/* Install the OS Interrupt Handlers. */
	CyRamVectors[ 11 ] = ( cyisraddress ) vPortSVCHandler;
	CyRamVectors[ 14 ] = ( cyisraddress ) xPortPendSVHandler;
	CyRamVectors[ 15 ] = ( cyisraddress ) xPortSysTickHandler;

	/* Start-up the peripherals. */


}


/*---------------------------------------------------------------------------*/
// Start all the system tasks here.  Of course, you can change this, but
// this is a very good place to keep the list of tasks started
/*---------------------------------------------------------------------------*/
int main( void )
{
    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
	prvHardwareSetup();
    
    extern void vStartLEDFlashTask( UBaseType_t uxPriority );
    // led flasher
	vStartLEDFlashTask( mainFLASH_TEST_TASK_PRIORITY );
    
    // USB SERIAL TASK
    extern void vStartUSBSerialTasks( UBaseType_t uxPriority );
    vStartUSBSerialTasks( mainUSBSERIAL_TASK_PRIORITY );

    // UART SERIAL TASK
    extern void vStartUARTSerialTask( UBaseType_t uxPriority );
    vStartUARTSerialTask( mainUART_SERIAL_TASK_Priority );

    
    // the terminal debugger task.
    extern void vStartTerminalDebugTask( int16_t priority );
    // debug terminal
    vStartTerminalDebugTask(mainTERMINAL_DEBUG_TASK_PRIORITY);

    
    
    
	/* Will only get here if there was sufficient memory to create the idle
    task.  The idle task is created within vTaskStartScheduler(). */
	vTaskStartScheduler();

	/* Should never reach here as the kernel will now be running.  If
	vTaskStartScheduler() does return then it is very likely that there was
	insufficient (FreeRTOS) heap space available to create all the tasks,
	including the idle task that is created within vTaskStartScheduler() itself. */
	for( ;; );
}
/*---------------------------------------------------------------------------*/




/*---------------------------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
    (void)pxTask;
    debugPort_putString("\r\n\nstack overflow!!:");
    debugPort_putString(pcTaskName);
    debugPort_putString("\n\r\n");
	/* The stack space has been execeeded for a task, considering allocating more. */
	taskDISABLE_INTERRUPTS();
}/*---------------------------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	/* The heap space has been execeeded. */
	taskDISABLE_INTERRUPTS();
}
/*---------------------------------------------------------------------------*/
