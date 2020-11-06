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
//--MICRO-USB:    
//  Macintosh 
//  The USB UART (micro-USB) connector on the CY8C-Kit059 is under /dev/tty.usbmodemxxxxx
//  or /dev/cu.usbmodemxxxxx as a tty device.  
//---------
//  Linux:
//  the USB UART (micro-USB) connector on the CY8C-Kit059 is usually under /dev/ttyACM0. 
//  It can be under ttyACM1, ttyACM2,...ttyACM9.
//  To make world usable under Linux, then
//  create the following file as root: /etc/udev/rules.d/50-local.rules
//  using "sudo nano" to write the file,
//  with the following 2 lines
//
//   KERNEL=="ttyUSB[0-9]*",MODE="0666"
//   KERNEL=="ttyACM[0-9]*",MODE="0666"
//
//  this will make the plugin of the /dev/ttyACMx world usable without
//  running as root. Must reboot the machine for this to work.
//----------
// Windows
//  The USB UART (micro-USB) driver created by PSOC Creator, and can be installed
//  under Windows 7 (unsigned driver) or under Windows 10 (signed).  See
//  the following information:  
//  https://socmaker.com/?p=168 for a full discussion of Windows 7 Driver,
//--    
//  or, for Windows 10 driver info 
//   see https://community.cypress.com/message/57726#57726
//   The knowledge base article is here:
//     ( https://community.cypress.com/docs/DOC-10894 )
//--
//  or (Windows 7) driver info:
//  After programming the PSoC, plug in the USBUART using a USB cable. 
//  You can interrupt the install, but that takes as long as waiting for the 
//  installer to give up talking to Windows Update Server.  Select the local 
//  file location option, and browse to the driver file in the PSOC directory 
//  in your project. It will be at:  PROJECTNAME.cysdn\Generated_Source\PSOC5, 
//  assuming your project is named “PROJECTNAME.” Install that unsigned driver 
//  (under windows 7). You may have to Google allowing unsigned drivers for 
//  Windows 7. It should be functional after that. 
// If not, if you are running a Virtual Machine, try rebooting your VM 
// when running under Mac or Linux 
// or 
//  reboot your Windows machine if running natively.
//------------------------------------------------------------------------------

#include "main.h"
#include "TerminalDebug.h"
#include "USBSerial.h" // for receiving serial port from debug monitor
#include "printf.h"
 

//                              printf,sprintf requires a lot of stack space
#define USBDebugSTACK_SIZE		configMINIMAL_STACK_SIZE*5

SerialMessage *msgPtr;
static portTASK_FUNCTION_PROTO( vTerminalDebugTask, pvParameters );


/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/
void vStartTerminalDebugTask( int16_t priority ) {
 
   
	/* Spawn the task. */
	xTaskCreate( vTerminalDebugTask, 
                    "TerminalDebugTask", 
                    USBDebugSTACK_SIZE, 
                    NULL, // pvParameters (none for now)
                    priority,
                    (TaskHandle_t *)&taskInfo[TERMINAL_DEBUG_TASK].taskHandle);

}

/*-----------------------------------------------------------*/
#define CMD_BUFFER_SIZE 128
char currentCmd[CMD_BUFFER_SIZE];
int16_t sizeOfCmd;
int16_t cmdIndex;
TaskStatus_t xTaskStatus;
BaseType_t xGetFreeStackSpace;
eTaskState eState;
char *printBuffer;


/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/
static portTASK_FUNCTION( vTerminalDebugTask, pvParameters )
{
    int16_t i;
    char c;
    static TaskInfo_t *TaskInfoPtr =& taskInfo[TERMINAL_DEBUG_TASK];
    int16_t queryCount=0;
    
    (void) pvParameters; // not currently used
    //usbserial_putString("USB Terminal Ready.\n\r>");
    printBuffer = pvPortMalloc(256);// 

    // wait for the queue to be created, give the user time to 
    // connect the terminal to the USB
    while (!USBMessageQ)    vTaskDelay(pdMS_TO_TICKS(100));
       
    // change the following timeout for production, is a nuisance waiting around
    // but for testing, gives the user a chance to see the signon message
    
    vTaskDelay(pdMS_TO_TICKS(6000));
    usbserial_putString("Debug Terminal Started..\n\r");
    
    for(;;){
        TaskInfoPtr->runCounter++;// rough profiler
        // wake up every second to process information for debug purposes
        // if there is no message to us
        if ( xQueueReceive( USBMessageQ, &msgPtr, 1000) == pdPASS ){
             usbserial_putString((const char*)(msgPtr->msg));// is already terminated by poster
              
            for (i=0 ; cmdIndex < 128  && i < msgPtr->size; cmdIndex++,sizeOfCmd++,i++){
                c =currentCmd[cmdIndex]=msgPtr->msg[i];                        
            }
                           
                // if user pressed enter
            if (c == '\r' || c == '\n' ) {
                ++queryCount;
                usbserial_putString("\n"); // force line feed
                
                
                cmdIndex=0;
                
                switch((enum CmdEnums)currentCmd[0]) {
                    default:
                    // command not recognized, print out the command list by
                    // falling through to the command list state
                    
                     case CMD_LIST:
                        sprintf(printBuffer,"#%d\n\rUptime: %ld\n\r",queryCount, xTaskGetTickCount());
                        usbserial_putString(printBuffer);
                        // 256 bytes maximum, need to start another string after 4 lines (to be safe)
                        usbserial_putString("\n\r-------------------------------\n\r"
                                            "?  -- list commands\n\r"                        
                                            "t  -- print tasks, time of operation, and stack space left\n\r"
                        );
                        usbserial_putString("v  -- print version & date\n\r"
                                            "\n"
                        );
                    
                    case version:
                    case VERSION:
                        usbserial_putString(VERSION_STRING );
                    break;
                        
                        
                   
                    
                    case nullCmd:
                        usbserial_putString("Command not understood\n\r");
                    break;
                        
                    case cmdTasks:
                    case CMDTASKS:{
                        #if configUSE_TRACE_FACILITY
                        
                        sprintf(printBuffer,"---------------\n\r"
                                            "Task Info\n\r"
                                            "---------------\n\r"
                                            "Uptime: %ld\n\r"
                                            "---------------\n\r",
                                            xTaskGetTickCount());
                        usbserial_putString(printBuffer);
                            
                        xGetFreeStackSpace=pdTRUE;
                        eState = eInvalid;
                        for (i=0 ; i < NUMBER_OF_TASKS; i++) {
                            if (taskInfo[i].taskHandle){
                                vTaskGetTaskInfo(taskInfo[i].taskHandle,
                                                 &xTaskStatus,
                                                 xGetFreeStackSpace,
                                                 eState );   
                                
                                // now print the state information
                                sprintf(printBuffer,"Task: %s\n\r"
                                                    "Runs: %ld\n\r"
                                                    "Stack:%d\n\r"
                                                    "------\n\r",
                                    xTaskStatus.pcTaskName,
                                    taskInfo[i].runCounter,
                                    xTaskStatus.usStackHighWaterMark);
                                
                                usbserial_putString(printBuffer);
                            }
                        }
                        #else
                            usbserial_putString("Must define configUSE_TRACE_FACILITY as 1 in FreeRTOSConfig.h to use this feature.\n\r");
                        #endif
                            
                        
                    }break;
                        
                    
                }
            // here we must parse the message
            usbserial_putString("\n\rTerminal Debug Task Ready\n\rPress Enter for list of available commands\n\r>");
            currentCmd[0]=nullCmd;
            } 
        }
        
    }
     
    
}



/* [] END OF FILE */
