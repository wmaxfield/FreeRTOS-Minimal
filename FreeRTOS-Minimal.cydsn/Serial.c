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
 */
// The PSOC CY8CKit-059 is an 80mzh processor with built in USB.
// It also has a USB finger stub which can be broken off.
//--Finger STUB:
//  *IF YOU KEEP IT ATTACHED* you can communicate over the finger stub with an
//  additional serial port.  With this port, that can be UART usable with 
//  TeraTerm or Putty, or and Linux or Mac machine.
//  Under Windows, the signed driver for the stub will automatically install
//
/*---------------------------------------------------------------------------*/
// this version does not use as much in FreeRTOS resources as their example
// This code is *much* less demanding of CPU resources
// This will avoid activating Receiving task until terminating char is received;
// therefore in its current form it is not suitable for character by character
// i/o.  That can be changed by rewriting vUARTSerialToDestinationTask
/*---------------------------------------------------------------------------*/

#include <device.h>
#include "main.h"
#include "FreeRTOS.h"
#include "USBSerial.h"
#include "serial.h"
#include "TerminalDebug.h"
/*---------------------------------------------------------------------------*/
// if the following is true, then you can test this task by connecting
// with a terminal program. It will echo the characters typed on
// the keyboard.
/*---------------------------------------------------------------------------*/
#define EchoCharactersFromKeyboardImmediately pdFALSE

#define NUMBER_OF_MSG_BUFFERS 3

#define UART_BUFFER_SIZE (int16_t)(128)

#define UARTSerialSTACK_SIZE		configMINIMAL_STACK_SIZE

#define serialSTRING_DELAY_TICKS		( portMAX_DELAY )
/*---------------------------------------------------------------------------*/

CY_ISR_PROTO( vUartRxISR );
/*---------------------------------------------------------------------------*/

QueueHandle_t       xSerialTxQueue = NULL;
QueueHandle_t       UARTMessageQ;  // for sending serial data to destination task
SemaphoreHandle_t   UARTMutex;
SemaphoreHandle_t   UARTDataAvailableMutex;
static int16_t      errorStatus;

static int16        iSerialRxBufferHead,iSerialRxBufferTail; // for rx wrap buffer

static char         cSerialRxBuffer[UART_BUFFER_SIZE+2];
 

/*---------------------------------------------------------------------------*/
// put a character out to the port, delay task if outbound fifo is full
/*---------------------------------------------------------------------------*/
void UARTputChar(char c){
    while((UART_1_TXSTATUS_REG & UART_1_TX_STS_FIFO_FULL)) {
        // FIFO is full 
        // wait 1 millisecond, which allows for ~9 chars to exit buffer
        // at 115200 baud.  Since it is only 4 chars deep, will be empty
        // for a while after this comes back.  Average time in the delay is
        // probably around 0.5 milliseconds for first visit, and
        // around 1 millisecond for subsequent visits for each string printed.
        // At the cost of cpu cycles, a ring buffer could be implemented
        // here, along with a TX interrupt.
        vTaskDelay(pdMS_TO_TICKS(1)); 
    }

    /* Add directly to the hardware FIFO */
    UART_1_TXDATA_REG = c;
}
/*---------------------------------------------------------------------------*/
// Put a string to the UART named UART_1
//  requires null terminated string
//  will delay task to allow characters to clear fifo.
/*---------------------------------------------------------------------------*/
//------------------------
// HARDWARE SPECIFIC BEGIN
//------------------------
void  UARTport_putstring(char *cPtr){

    /* If not Initialized then skip this function */
    if(UART_1_initVar)
    {
        /* This is a blocking function, it will not exit until all data is sent */
        // the task will be suspended when the TXFifo becomes full
        while(*cPtr) {
           UARTputChar(*cPtr++);
        }
    }
}
//------------------------
// HARDWARE SPECIFIC END
//------------------------

/*---------------------------------------------------------------------------*/
// get a character from the UART Ring Buffer into cPtr.
// if cPtr is null, return 0
// NOTE: This function reads directly from the Ring Buffer and does not
// involve the 
/*---------------------------------------------------------------------------*/
int16 getRxChar(char *cPtr) {
    
    if (!cPtr)
        return 0;
    
    if (iSerialRxBufferHead != iSerialRxBufferTail){
        *cPtr = cSerialRxBuffer[iSerialRxBufferTail++];
        if (iSerialRxBufferTail >=UART_BUFFER_SIZE)
            iSerialRxBufferTail=0;
        return 1;
    }
    
    return 0;
}
//---------------------------
int16_t iPeekSerialRxBufferTail;
//------------------------------------------------------------------------------
// return 0 if no character found.  return 1 and place character in *cPtr
//   unless cPtr == NULL; if so, don't write to *cPtr, just return char avail
//   status.
//------------------------------------------------------------------------------
int16_t peekNextRxChar(char *cPtr){
    char c;
    
    if (iSerialRxBufferHead != iPeekSerialRxBufferTail){
        c = cSerialRxBuffer[iPeekSerialRxBufferTail++];
        if (iPeekSerialRxBufferTail >=UART_BUFFER_SIZE)
            iPeekSerialRxBufferTail=0;
        if (cPtr)
            *cPtr=c;
        return 1;
    }
    
    return 0;
   
}

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
xComPortHandle xSerialPortInitMinimal( unsigned long ulWantedBaud, unsigned portBASE_TYPE uxQueueLength )
{
    (void)ulWantedBaud; // not used at the moment
    (void)uxQueueLength;// not used at the moment

//------------------------
// HARDWARE SPECIFIC START
//------------------------
	/* Configure Rx. for interrupt Only */
    /* Tx is handled in Cypress API */
    UART_1_Start();
    UART_1_ClearTxBuffer();
    UART_1_ClearRxBuffer();
    isr_UART1_RX_BYTE_RECEIVED_StartEx(vUartRxISR);

//------------------------
// HARDWARE SPECIFIC END
//------------------------
    // Even if Task is not created, these Semaphores are still needed
    /*Setup the mutexes to control port access*/
    UARTMutex = xSemaphoreCreateMutex();
    UARTDataAvailableMutex = xSemaphoreCreateMutex();


	return ( xComPortHandle )( 1 );
}

/*---------------------------------------------------------------------------*/
// put a string to the serial port.  This is a blocking function
/*---------------------------------------------------------------------------*/
void vSerialPutString( xComPortHandle pxPort, const signed char *  pcString, unsigned short usStringLength )
{
    (void)pxPort; // not used for now
//------------------------
// HARDWARE SPECIFIC START
//------------------------

    /* If hardware not Initialized then skip this function */
    if(UART_1_initVar) {
        /* This is a blocking function, it will not exit until all data is sent */
        while(usStringLength) {
            UARTputChar(*pcString++);
            usStringLength--;
        }
    }
//------------------------
// HARDWARE SPECIFIC END
//------------------------

}
/*---------------------------------------------------------------------------*/
// only use this if you can have a task waiting on a character for a short
// period of time.  Pass 0 for xBlockTime to have it return immediately
// wsm rewritten from FreeRTOS example to be more efficient.
// Will block for xBlockTime and then try again.
// returns non-zero if character received
/*---------------------------------------------------------------------------*/
signed portBASE_TYPE xSerialGetChar( xComPortHandle pxPort, signed char *pcRxedChar, TickType_t xBlockTime )
{

    (void)pxPort;

    if (!getRxChar((char*)pcRxedChar)){
        if (xBlockTime)
            vTaskDelay(xBlockTime);
        return (getRxChar((char*)pcRxedChar));
    }
    // no character returned
	return 0;
}
/*---------------------------------------------------------------------------*/
// return FALSE if error on send, TRUE if sent
// Delay task a Maximum of xBlockTime milliseconds
/*---------------------------------------------------------------------------*/
signed portBASE_TYPE xSerialPutChar( xComPortHandle pxPort, //ignored
                                    signed char cOutChar,   //character to send
                                    TickType_t xBlockTime ) // ms maximum wait time
{
portBASE_TYPE xReturn = pdFALSE;
    (void)pxPort;

//------------------------
// HARDWARE SPECIFIC START
//------------------------

    /* If not Initialized then skip this function */
    if(UART_1_initVar) {
    /* This is a blocking function, it will not exit until all data is sent */
        while((UART_1_TXSTATUS_REG & UART_1_TX_STS_FIFO_FULL)) {
            /* FIFO is full */
            if (!xBlockTime)
                return xReturn;

            vTaskDelay(pdMS_TO_TICKS(1)); // wait 1 millisecond at a time
            xBlockTime--;
        }

        /* Add directly to the FIFO */
        UART_1_TXDATA_REG = cOutChar;
        xReturn = pdTRUE;
    }

//------------------------
// HARDWARE SPECIFIC END
//------------------------
	
	return xReturn;
}
/*---------------------------------------------------------------------------*/
// receive characters and put into ring buffer for getRxChar() to pull out.
// NOTE: The UART Rx task will not wake until either \r or \n is received.
//       Since *all* commands must be terminated with \r or \n, this prevents
//       cpu resources from being consumed by a lot of unnecessary task switching.
//       If you *must* look at every character coming in, 
//       do it here, or post ever character to rx task.
/*---------------------------------------------------------------------------*/
CY_ISR(vUartRxISR)
{
    int16_t dataRcvd=0;
    static BaseType_t xHigherPriorityTaskWoken;
    
    uint8 rxStatus=0;         
    uint8 rxData=0;      
    
    if(iSerialRxBufferTail <0 || iSerialRxBufferTail > UART_BUFFER_SIZE)
        iSerialRxBufferTail=0; // failsafe
    
    do
    {
        /* Read receiver status register */
        rxStatus = UART_1_RXSTATUS_REG;

        if((rxStatus & (UART_1_RX_STS_BREAK      | UART_1_RX_STS_PAR_ERROR |
                        UART_1_RX_STS_STOP_ERROR | UART_1_RX_STS_OVERRUN)) != 0u)
        {
            /* ERROR handling. */
            errorStatus |= rxStatus & ( UART_1_RX_STS_BREAK      | UART_1_RX_STS_PAR_ERROR | 
                                        UART_1_RX_STS_STOP_ERROR | UART_1_RX_STS_OVERRUN);
            
        }
        
        if((rxStatus & UART_1_RX_STS_FIFO_NOTEMPTY))
        {
            /* Read data from the RX data register */
            rxData = UART_1_RXDATA_REG;
            cSerialRxBuffer[iSerialRxBufferHead++]= rxData;

            // Wait for carriage return or line feed to give to 
            // character processing task.
            if (rxData == '\n' || rxData =='\r') 
                dataRcvd=1;
            
            // NOTE this will interleave with characters being
            // printed, but sometimes will fail spectacularly
            if (EchoCharactersFromKeyboardImmediately)                         
                UART_1_TXDATA_REG = rxData;

            // no protection against data overrun, tail can hit head
            if (iSerialRxBufferHead >= (int16_t)UART_BUFFER_SIZE)
                iSerialRxBufferHead = 0;
            
        }
    }while((rxStatus & UART_1_RX_STS_FIFO_NOTEMPTY));

    if (dataRcvd){
        /* The event has occurred, use the semaphore to unblock the task so the task
        can process the event. */
        xSemaphoreGiveFromISR( UARTDataAvailableMutex, &xHigherPriorityTaskWoken );
        /* Now the task has been unblocked a context switch should be performed if
        xHigherPriorityTaskWoken is equal to pdTRUE. NOTE: The syntax required to perform
        a context switch from an ISR varies from port to port, and from compiler to
        compiler. Check the web documentation and examples for the port being used to
        find the syntax required for your application. */
        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );//either this or portEND_SWITCHING_ISR() will work
    }

}

static int16_t qError; // error count for debug
static SerialMessage *UARTSerialRxBuffer[NUMBER_OF_MSG_BUFFERS];
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
static portTASK_FUNCTION( vUARTSerialToDestinationTask, pvParameters )
{
    uint16_t rxCount;
    uint16_t myTaskMsgBufferNumber=0;
    static TaskInfo_t *TaskInfoPtr = &taskInfo[UART_SERIAL_TASK];
    char * ptr1;
    
   	/* The parameters are not used. */
	( void ) pvParameters;
    
    int index;
    // initialize serial receive buffers if not initialized elsewhere
    for (index=0; index < NUMBER_OF_MSG_BUFFERS; index++) {
        if (!UARTSerialRxBuffer[index]) // did someone else take care of this?
            UARTSerialRxBuffer[index]=(SerialMessage *) pvPortMalloc( sizeof(SerialMessage));   
    }
    
    xSerialPortInitMinimal(-1,-1 );// initialize the serial port.  options not used

    UARTport_putstring("UART Serial Task Ready\n\r");
    
	for(;;)
	{
        TaskInfoPtr->runCounter++;// rough profiler
        
        // hang here until ISR gives mutex, if we are the Debug Task Serial Port
        // The Semaphore is not posted until a \n or \r is seen in the data
        // stream.
        xSemaphoreTake(UARTDataAvailableMutex,portMAX_DELAY);
        
        // we are here because isr has data for us.
        SerialMessage *ptr = UARTSerialRxBuffer[myTaskMsgBufferNumber];
        rxCount =0;
        ptr1 =(char*) ptr->msg;
                  
        while(getRxChar(ptr1++) && rxCount < (MSG_BUFFER_SIZE-1))
            rxCount++;
                           
        // now post these bytes to the Task pointed to by the UARTMessageQ
        ptr->ucMessageID = myTaskMsgBufferNumber++;
        ptr->size=rxCount;
        ptr->msg[rxCount]=0;// terminate string just in case
        
       
        
        if(rxCount && UARTMessageQ ) {        
            // block if fails (queue full), wait 100ms and try again.
            if( xQueueSend( UARTMessageQ,&ptr, pdMS_TO_TICKS(100) ) != pdPASS ){
        			qError ++;
                    //try one more time
                    xQueueSend( UARTMessageQ,&ptr, pdMS_TO_TICKS(100));
            }
        }
        
        if (myTaskMsgBufferNumber >= NUMBER_OF_MSG_BUFFERS)
            myTaskMsgBufferNumber=0; // reset the ring
        
    }
}
/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/
void vStartUARTSerialTask( UBaseType_t uxPriority )
{
    
	/* Spawn the task. */
	xTaskCreate( vUARTSerialToDestinationTask, "UARTSerial", UARTSerialSTACK_SIZE, NULL, uxPriority, ( TaskHandle_t * ) &taskInfo[UART_SERIAL_TASK].taskHandle);
}

