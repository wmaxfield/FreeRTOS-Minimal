/*
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
 
    THIS TASK MONITORS AND KEEPS THE USB SERIAL PORT IN OPERATION, HANDLING
     PLUGS AND UNPLUGS.
 */

#include "project.h"

#include <stdlib.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "USBSerial.h"
#include "main.h"

#define USBFS_DEVICE (0)  // not sure why needed

#define USBSerialSTACK_SIZE		configMINIMAL_STACK_SIZE

const TickType_t xDelay = pdMS_TO_TICKS(1);
const TickType_t mDelay = pdMS_TO_TICKS(5000);
    
SerialMessage *USBRxBuffer[NUMBER_OF_USB_BUFFERS];

int16_t currentBuffer;
    
/* The task that is created three times. */
static portTASK_FUNCTION_PROTO( vUSBSerialTask, pvParameters );

SemaphoreHandle_t USBMutex;
QueueHandle_t     USBMessageQ;
int16_t           qError;


/*-----------------------------------------------------------*/
// the following code was patterned from Cypress example
// and modified so as to not lock up when untoward
// events occur in the USB system
// Uses semaphores to serialize access.  Multiple tasks can
// use this routine, but printing results will be interleaved
// Silently fails and returns upon error
/*-----------------------------------------------------------*/
void usbserial_putString(const char msg[])
{
int16 waitCount=0;

    if(0 == USBUART_GetConfiguration()) 
        return;// return silently
    
    // the following has to be checked more than once,
    // since at ~80mhz bus clock the USB can report
    // no activity between calls.
    // if 20 milliseconds have passed, the USB is 
    // probably unplugged.
    while (0==USBUART_CheckActivity()){
        if (++waitCount>20){
            xSemaphoreGive(USBMutex);
            return;// return silently
        }
        vTaskDelay(xDelay);// wait 1 millisecond
    }
    waitCount=0;
  
    xSemaphoreTake(USBMutex,portMAX_DELAY);
    while(0 == USBUART_CDCIsReady()){
        // if an error in the USB, then leave
        // rather than hang here forever.
        // ignore the "transmission" in that case
        if (++waitCount>20){
            xSemaphoreGive(USBMutex);
            return;// return silently
        }
        vTaskDelay(xDelay);
    }

    if (USBUART_CDCIsReady())
        USBUART_PutString(msg);
        
    xSemaphoreGive(USBMutex);
     
}

/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/
void vStartUSBSerialTasks( UBaseType_t uxPriority )
{    
    /*Setup the mutex to control port access*/
    USBMutex = xSemaphoreCreateMutex();
	/* Spawn the task. */
	xTaskCreate( vUSBSerialTask, 
                "USBSerial", 
                USBSerialSTACK_SIZE, 
                NULL,   
                uxPriority, 
                ( TaskHandle_t * ) &taskInfo[USB_SERIAL_TASK].taskHandle);
}
/*-----------------------------------------------------------*/
// the following code was patterned from Cypress example
// and modified so as to not lock up when untoward
// events occur in the USB system, and to send data
// to the receiving task under FreeRTOS
/*-----------------------------------------------------------*/
static portTASK_FUNCTION( vUSBSerialTask, pvParameters )
{
    uint16_t rxCount;
    uint16_t bufferNumber=0;
    TaskInfo_t *TaskInfoPtr = &taskInfo[USB_SERIAL_TASK];

    
   	/* The parameters are not used. */
	( void ) pvParameters;
    
    int index;
    
    for (index=0; index < NUMBER_OF_USB_BUFFERS; index++) {
        USBRxBuffer[index]=(SerialMessage *) pvPortMalloc( sizeof(SerialMessage));   
    }
    
    //--------------------------------------------------------------------------
    // create the queue we will send our USB data to.  If there is a 
    // listener on the queue, it will get these messages and process them.
    //--------------------------------------------------------------------------
    USBMessageQ = xQueueCreate( NUMBER_OF_USB_BUFFERS, sizeof( SerialMessage* ) );
	if( USBMessageQ == 0 )
	{
		// Queue was not created and must not be used.
        // panic here.
        // we can't tell anyone anything, the system is completely broken
        while(1);// maybe a hang will get debugging user's attention
	}
    //--------------------------------------------------------------------------


    /* Start the USB_UART */
    /* Start USBFS operation with 5-V operation. */
    USBUART_Start(USBFS_DEVICE, USBUART_5V_OPERATION);

    
	for(;;)
	{
        TaskInfoPtr->runCounter++; // rough profiler
        
        /* Host can send double SET_INTERFACE request. */
        if (USBUART_IsConfigurationChanged())
        {
            /* Initialize IN endpoints when device is configured. */
            if ( USBUART_GetConfiguration())
            {
                /* Enumeration is done, enable OUT endpoint to receive data 
                 * from host. */
                USBUART_CDC_Init();
            }
        }
        
        if( USBUART_GetConfiguration()) {   
            /* Check for input data from host. */
            if ( USBUART_DataIsReady()){
                
                SerialMessage *ptr = USBRxBuffer[bufferNumber];
                /* Read received data and re-enable OUT endpoint. */
                rxCount = USBUART_GetAll(ptr->msg);// most we can get is 64 bytes
                // now post these bytes to the Debug Terminal Task
                ptr->ucMessageID = bufferNumber++;
                ptr->size=rxCount;
                ptr->msg[rxCount]=0;// terminate string just in case
                
                // block if fails (queue full)
                if( xQueueSend( USBMessageQ,&ptr, pdMS_TO_TICKS(100) ) != pdPASS )
            		{
            			qError ++;
                        //try one more time
                        xQueueSend( USBMessageQ,ptr, pdMS_TO_TICKS(100));
            		}
                    
                // fail or not, continue on.    
                if (bufferNumber >= NUMBER_OF_USB_BUFFERS)
                    bufferNumber=0; // reset the ring
            }// if USBUART_DataIsReady

        }// if USBUART_GetConfiguration
        //--------------------------------------------------------------------
        //  wait 100 milliseconds, we are a high priority task
        // this is fast enough to not bother the user too much.
        // but for communications purposes, it might be a bit slow.
        // You can change to be 1 millisecond, or you can lower the priority
        // to the lowest. In that case, the USB could be ignored if system
        // locks.  For a Debug Task to be useful, it needs to be the highest 
        // priority so it can check on other tasks.
        //--------------------------------------------------------------------
        vTaskDelay(pdMS_TO_TICKS(100));


    } /*lint !e715 !e818 !e830 Function definition must be standard for task creation. */

}

// end of file
