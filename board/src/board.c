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

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/
static const board_gpioInfo_type board_gpioLeds[] =
{
    {PORTE, GPIOE, 29},     /* LED ROJO */
    {PORTD, GPIOD, 5},      /* LED VERDE */
};

static const board_gpioInfo_type board_gpioSw[] =
{
    {PORTC, GPIOC, 3},      /* SW1 */
    {PORTC, GPIOC, 12},     /* SW3 */
};

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/
void board_init(void)
{
    int32_t i;

    /* Activación de clock para los puertos utilizados */
    SIM_HAL_EnableClock(SIM, kSimClockGatePortA);
    SIM_HAL_EnableClock(SIM, kSimClockGatePortC);
    SIM_HAL_EnableClock(SIM, kSimClockGatePortD);
    SIM_HAL_EnableClock(SIM, kSimClockGatePortE);

    /* inicialización de leds */
    for (i = 0 ; i < BOARD_LED_ID_TOTAL ; i++)
    {
        PORT_HAL_SetMuxMode(board_gpioLeds[i].port, board_gpioLeds[i].pin, kPortMuxAsGpio);
        board_setLed(i, BOARD_LED_MSG_OFF);
        GPIO_HAL_SetPinDir(board_gpioLeds[i].gpio, board_gpioLeds[i].pin, kGpioDigitalOutput);
    }

    /* inicialización de SWs */
    for (i = 0 ; i < BOARD_SW_ID_TOTAL ; i++)
    {
        PORT_HAL_SetMuxMode(board_gpioSw[i].port, board_gpioSw[i].pin, kPortMuxAsGpio);
        GPIO_HAL_SetPinDir(board_gpioSw[i].gpio, board_gpioSw[i].pin, kGpioDigitalInput);
        PORT_HAL_SetPullCmd(board_gpioSw[i].port, board_gpioSw[i].pin, true);
        PORT_HAL_SetPullMode(board_gpioSw[i].port, board_gpioSw[i].pin, kPortPullUp);
    }
}

void board_setLed(board_ledId_enum id, board_ledMsg_enum msg)
{
    switch (msg)
    {
        case BOARD_LED_MSG_OFF:
            GPIO_HAL_SetPinOutput(board_gpioLeds[id].gpio, board_gpioLeds[id].pin);
            break;

        case BOARD_LED_MSG_ON:
            GPIO_HAL_ClearPinOutput(board_gpioLeds[id].gpio, board_gpioLeds[id].pin);
            break;

        case BOARD_LED_MSG_TOGGLE:
            GPIO_HAL_TogglePinOutput(board_gpioLeds[id].gpio, board_gpioLeds[id].pin);
            break;

        default:
            break;
    }
}

bool board_getSw(board_swId_enum id)
{
    return !GPIO_HAL_ReadPinInput(board_gpioSw[id].gpio, board_gpioSw[id].pin);
}

/*==================[end of file]============================================*/
