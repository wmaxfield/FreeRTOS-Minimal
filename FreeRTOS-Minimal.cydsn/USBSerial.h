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
#ifndef USBSERIAL_H
#define USBSERIAL_H
#ifndef ___int8_t_defined
    #include "project.h"// needed for uint8_t,uint32_t
#endif

#ifndef UBaseType_t
    #include "portmacro.h"
#endif

#ifndef QueueHandle_t     
    #include "queue.h"
#endif

QueueHandle_t     USBMessageQ;

#define MSG_BUFFER_SIZE 64

typedef struct SerialMessageStruct
 {
	uint16_t ucMessageID;
    uint16_t size;
	uint8_t  msg[ MSG_BUFFER_SIZE +2];// allow terminating zero just in case
 } SerialMessage;

#define NUMBER_OF_USB_BUFFERS 8
extern SerialMessage *USBRxBuffer[NUMBER_OF_USB_BUFFERS];

    
    

void usbserial_putString(const char msg[]);

#endif

