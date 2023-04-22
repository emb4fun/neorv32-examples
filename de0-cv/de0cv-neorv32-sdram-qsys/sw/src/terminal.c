/**************************************************************************
*  Copyright (c) 2021 by Michael Fischer (www.emb4fun.de).
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*  1. Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*
*  2. Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in the
*     documentation and/or other materials provided with the distribution.
*
*  3. Neither the name of the author nor the names of its contributors may
*     be used to endorse or promote products derived from this software
*     without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
*  THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
*  OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
*  AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
*  THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
*  SUCH DAMAGE.
*
**************************************************************************/
#define __TERMINAL_C__

/*=======================================================================*/
/*  Includes                                                             */
/*=======================================================================*/
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "neorv32.h"

#include "terminal.h"

/*=======================================================================*/
/*  All Structures and Common Constants                                  */
/*=======================================================================*/

#define BAUD_RATE             19200

#if !defined(PRINTF_BUFFER_SIZE)
#define PRINTF_BUFFER_SIZE    128
#endif


/*=======================================================================*/
/*  Definition of all global Data                                        */
/*=======================================================================*/

/*=======================================================================*/
/*  Definition of all local Data                                         */
/*=======================================================================*/

static uint8_t PrintfBuffer[PRINTF_BUFFER_SIZE];

/*=======================================================================*/
/*  Definition of all local Procedures                                   */
/*=======================================================================*/

/*=======================================================================*/
/*  All code exported                                                    */
/*=======================================================================*/

/*************************************************************************/
/*  term_Start                                                           */
/*                                                                       */
/*  Start the terminal task.                                             */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void term_Start (void)
{
   neorv32_uart_setup(NEORV32_UART0, BAUD_RATE, 0);

} /* term_Start */

/*************************************************************************/
/*  term_printf                                                          */
/*                                                                       */
/*  This is a printf like output function.                               */
/*                                                                       */
/*  In    : fmt                                                          */
/*  Out   : none                                                         */
/*  Return: Number of characters transmitted                             */
/*************************************************************************/
int term_printf (const char *fmt, ...)
{
   int     n = 0; /* Number of characters transmitted */
   va_list ap;

   va_start(ap, fmt);
   n = vsnprintf((char*)PrintfBuffer, sizeof(PrintfBuffer), fmt, ap);
   va_end(ap);
   (void)ap;

   for (int i = 0; i < n; i++)
   {
      neorv32_uart_putc(NEORV32_UART0, PrintfBuffer[i]);
   }

   return(n);
} /* term_printf */

/*************************************************************************/
/*  term_puts                                                            */
/*                                                                       */
/*  Send a string over the UART.                                         */
/*                                                                       */
/*  In    : string                                                       */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void term_puts (char *string)
{
   neorv32_uart_printf(NEORV32_UART0, string);
   neorv32_uart_putc(NEORV32_UART0, '\r');
   neorv32_uart_putc(NEORV32_UART0, '\n');

} /* term_puts */

/*************************************************************************/
/*  term_putchar                                                         */
/*                                                                       */
/*  Send a character over the UART.                                      */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: Return the char which was written or EOF                     */
/*************************************************************************/
int term_putchar (int ch)
{
   neorv32_uart_putc(NEORV32_UART0, ch);

   return(ch);
} /* term_putchar */

/*** EOF ***/


