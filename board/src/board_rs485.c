/* Copyright 2017, DSI FCEIA UNR - Sistemas Digitales 2
 *    DSI: http://www.dsi.fceia.unr.edu.ar/
 * Copyright 2017, Diego Alegrechi
 * Copyright 2017, Gustavo Muro
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*==================[inclusions]=============================================*/
#include "board.h"
#include "MKL46Z4.h"
#include "fsl_port_hal.h"
#include "fsl_gpio_hal.h"
#include "fsl_sim_hal.h"
#include "fsl_uart_hal.h"
#include "fsl_clock_manager.h"

/*==================[macros and definitions]=================================*/

#define RS485_TX_PORT       PORTE
#define RS485_TX_PIN        0

#define RS485_RX_PORT       PORTE
#define RS485_RX_PIN        1

#define RS485_UART                  UART1
#define RS485_UART_IRQn             UART1_IRQn
#define RS485_kSimClockGateUart     kSimClockGateUart1

/*==================[internal data declaration]==============================*/
static const board_gpioInfo_type board_gpioContLine[] =
{
    {PORTA, GPIOA, 16},    /* RE */
    {PORTA, GPIOA, 17},    /* DE */
};

static bool dataAvailable;
static uint8_t byteRec;

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

static void rs485_RE(bool est)
{
    if (est)
        GPIO_HAL_SetPinOutput(board_gpioContLine[0].gpio, board_gpioContLine[0].pin);
    else
        GPIO_HAL_ClearPinOutput(board_gpioContLine[0].gpio, board_gpioContLine[0].pin);
}

static void rs485_DE(bool est)
{
    if (est)
        GPIO_HAL_SetPinOutput(board_gpioContLine[1].gpio, board_gpioContLine[1].pin);
    else
        GPIO_HAL_ClearPinOutput(board_gpioContLine[1].gpio, board_gpioContLine[1].pin);
}

/*==================[external functions definition]==========================*/
void board_rs485_init(void)
{
    int32_t i;
    uint32_t frecPerifUart;

    dataAvailable = false;

    /* inicialización de pines de control */
    for (i = 0 ; i < 2 ; i++)
    {
        PORT_HAL_SetMuxMode(board_gpioContLine[i].port, board_gpioContLine[i].pin, kPortMuxAsGpio);
        GPIO_HAL_SetPinDir(board_gpioContLine[i].gpio, board_gpioContLine[i].pin, kGpioDigitalOutput);
    }

    rs485_RE(false);
    rs485_DE(false);

    PORT_HAL_SetMuxMode(RS485_TX_PORT, RS485_TX_PIN, kPortMuxAlt3);
    PORT_HAL_SetMuxMode(RS485_RX_PORT, RS485_RX_PIN, kPortMuxAlt3);

    SIM_HAL_EnableClock(SIM, RS485_kSimClockGateUart);

    UART_HAL_Init(RS485_UART);

    UART_HAL_EnableTransmitter(RS485_UART);

    UART_HAL_EnableReceiver(RS485_UART);

    CLOCK_SYS_GetFreq(kBusClock, &frecPerifUart);

    UART_HAL_SetBaudRate(RS485_UART, frecPerifUart, 9600);

    UART_HAL_SetBitCountPerChar(RS485_UART, kUart8BitsPerChar);

    UART_HAL_SetParityMode(RS485_UART, kUartParityDisabled);

    UART_HAL_SetStopBitCount(RS485_UART, kUartOneStopBit);

    UART_HAL_SetIntMode(RS485_UART, kUartIntRxDataRegFull, true);

    NVIC_ClearPendingIRQ(RS485_UART_IRQn);
    NVIC_EnableIRQ(RS485_UART_IRQn);
}

void board_rs485_sendByte(uint8_t dato)
{
    rs485_RE(true);
    rs485_DE(true);

    UART_HAL_Putchar(RS485_UART, dato);

    UART_HAL_SetIntMode(RS485_UART, kUartIntTxDataRegEmpty, true);
    UART_HAL_SetIntMode(RS485_UART, kUartIntTxComplete, true);

}

bool board_rs485_isDataAvailable(void)
{
    return dataAvailable;
}

uint8_t board_rs485_readByte(void)
{
    dataAvailable = false;
    return byteRec;
}

void UART1_IRQHandler(void)
{
    if (UART_HAL_GetStatusFlag(RS485_UART, kUartRxDataRegFull) &&
        UART_HAL_GetIntMode(RS485_UART, kUartIntRxDataRegFull))
    {
        UART_HAL_Getchar(RS485_UART, &byteRec);
        dataAvailable = true;
        UART_HAL_ClearStatusFlag(RS485_UART, kUartRxDataRegFull);
    }

    if (UART_HAL_GetStatusFlag(RS485_UART, kUartTxDataRegEmpty) &&
        UART_HAL_GetIntMode(RS485_UART, kUartIntTxDataRegEmpty))
    {
        UART_HAL_SetIntMode(RS485_UART, kUartIntTxDataRegEmpty, false);
        UART_HAL_ClearStatusFlag(RS485_UART, kUartTxDataRegEmpty);
    }

    if (UART_HAL_GetStatusFlag(RS485_UART, kUartTxComplete) &&
        UART_HAL_GetIntMode(RS485_UART, kUartIntTxComplete))
    {
        UART_HAL_ClearStatusFlag(RS485_UART, kUartTxComplete);
        UART_HAL_SetIntMode(RS485_UART, kUartIntTxComplete, false);
        rs485_RE(false);
        rs485_DE(false);
    }
}

/*==================[end of file]============================================*/
