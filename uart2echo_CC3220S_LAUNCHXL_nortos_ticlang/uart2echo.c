/*
 * Copyright (c) 2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== uart2echo.c ========
 */
#include <stdint.h>
#include <stddef.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART2.h>

/* Driver configuration */
#include "ti_drivers_config.h"

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    char input; // one byte input
    const char echoPrompt[] = "Echoing characters:\r\n"; //one byte char

    GPIO_init();

    // configure LED pin
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF); // initially OFF

    UART2_Handle uart;
    UART2_Params uartParams;
    uint32_t status = UART2_STATUS_SUCCESS;

    // set up UART without data processing
    // UART_DATA_BINARY mode is default
    UART2_Params_init(&uartParams);
    uartParams.baudRate = 115200;
    uartParams.readReturnMode = UART2_ReadReturnMode_FULL; //specify only return when buffer is full (one byte buffer)

    uart = UART2_open(CONFIG_UART2_0, &uartParams);

    if (uart == NULL)
    {
        while (1) {}
    }

    size_t bytesRead;

    UART2_write(uart, echoPrompt, sizeof(echoPrompt), &bytesRead);

    /*
     *  LED init waits for input, moves to next state when 'O' is entered
     *  State1 waits for either N or F, N-> ON : F->State2
     *  State2 waits for second F and turns LED OFF
     */
    enum LED_States {LED0_Init, LED0_State1, LED0_State2, LED0_ON, LED0_OFF} LED_State = LED0_Init; //start in init

    while (1)
    {
        UART2_read(uart, &input, 1, NULL); // read one byte at a time, unbuffered

        switch(LED_State) {
        case LED0_Init:
            if (input == 'O') {
                LED_State = LED0_State1;
            }
            break;
        case LED0_State1: // waiting to either turn on or prime for second 'F'
            if (input == 'N') {
                LED_State = LED0_ON; // turn on LED and return to init state
            }
            else if (input == 'F') {
                LED_State = LED0_State2;
            }
            break;
        case LED0_State2:
            if (input == 'F') { // ready to turn off on second 'F'
                LED_State = LED0_OFF; // turn LED off and return to init state
            }
            break;
        default:
            LED_State = LED0_Init;
            break;
        }

        switch(LED_State) {
        case LED0_ON:
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON); // turn LED ON
            LED_State = LED0_Init; // go back to initial state
            break;
        case LED0_OFF:
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF); // turn LED OFF
            LED_State = LED0_Init; // go back to initial state
            break;
        }

        UART2_write(uart, &input, 1, &bytesRead);
    }
}
