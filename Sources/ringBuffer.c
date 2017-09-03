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
#include "ringBuffer.h"
#include "stdlib.h"

/*==================[macros and definitions]=================================*/

typedef struct
{
    int32_t indexRead;
    int32_t indexWrite;
    int32_t count;
    int32_t size;
    uin8_t *pBuf;
}ringBuferData_struct;

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/

void *ringBuffer_init(int32_t size)
{
    ringBuferData_struct *rb;

    /* TODO: verificar puntero null*/
    rb = malloc(sizeof(ringBuferData_struct));

    /* TODO: verificar puntero null*/
    rb->pBuf = malloc(size);

    rb->indexRead = 0;
    rb->indexWrite = 0;
    rb->count = 0;
    rb->size = size;
}

void ringBuffer_deInit(void *rb)
{
    // TODO: implementar
}

bool ringBuffer_putData(void *pRb, uint8_t data)
{
    ringBuferData_struct *rb = pRb;
    bool ret = false;

    rb->pBuf[rb->indexWrite] = data;

    rb->indexWrite++;
    rb->indexWrite = rb->indexWrite % rb->size;

    if (rb->count < rb->size)
    {
        rb->count++;
    }
    else
    {
        /* si el buffer está lleno incrementa en uno indexRead
         * haciendo que se pierda el dato má viejo y devuelve
         * true para indicar que se estan perdiendo datos */
        rb->indexRead++;
        rb->indexRead = rb->indexRead % rb->size;
        ret = true;
    }

    return ret;
}

bool ringBuffer_getData(void *pRb, uint8_t *data)
{
    ringBuferData_struct *rb = pRb;
    bool ret = true;

    if (rb->count)
    {
        *data = rb->pBuf[rb->indexRead];

        rb->indexRead++;
        rb->indexRead = rb->indexRead % rb->size;
        rb->count--;
    }
    else
    {
        ret = false;
    }

    return ret;
}

/*==================[end of file]============================================*/