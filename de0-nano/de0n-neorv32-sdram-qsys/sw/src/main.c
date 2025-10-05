/**************************************************************************
*  Copyright (c) 2021 by Michael Fischer (www.emb4fun.de).
*  All rights reserved.
*
*  Example is based on the blink_led example from NEORV32.
*  Copyright (c) 2021, Stephan Nolting.
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
***************************************************************************
*
*  RAM test see Embedded System Academy
*
*  (http://www.esacademy.com/en/library/technical-articles-and-documents/
*   miscellaneous/software-based-memory-testing.html)
*
**************************************************************************/
#define __MAIN_C__

/*=======================================================================*/
/*  Includes                                                             */
/*=======================================================================*/
#include "neorv32.h"
#include "terminal.h"
#include "memtest.h"

/*=======================================================================*/
/*  All Structures and Common Constants                                  */
/*=======================================================================*/

#define BASE_ADDRESS ((volatile datum*)0x90000000)
#define NUM_BYTES    (32*1024*1024)


/*=======================================================================*/
/*  Definition of all global Data                                        */
/*=======================================================================*/

/*=======================================================================*/
/*  Definition of all extern Data                                        */
/*=======================================================================*/

/*=======================================================================*/
/*  Definition of all local Data                                         */
/*=======================================================================*/

/*************************************************************************/
/*  OutputBootMessage                                                    */
/*                                                                       */
/*  Output boot message.                                                 */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void OutputBootMessage (void)
{
   const char ResetScreen[] = { 0x1B, 'c', 0 };
   
   term_printf("%s", ResetScreen);
   neorv32_aux_delay_ms(neorv32_sysinfo_get_clk(), 200);   

   term_printf("\r\n");
   term_printf("*********************************\r\n");
   term_printf("  Memory test and running light\r\n");
   term_printf("*********************************\r\n");
   term_printf("\r\n");
   
} /* OutputBootMessage */

/*************************************************************************/
/*  MemoryTest                                                           */
/*                                                                       */
/*  Test the external SDRAM.                                             */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: 0 = OK / -1 = ERROR                                          */
/*************************************************************************/
static int MemoryTest (void)
{
   int rc = -1;
   
   term_printf("Memory test for %d bytes is started.\r\n", NUM_BYTES);

   term_printf("  Data Bus Test... ");
   if (memTestDataBus(BASE_ADDRESS) != 0)
   {
      term_printf("ERROR\r\n");
   }
   else
   {
      term_printf("OK\r\n");
      term_printf("  Address Bus Test... ");
      if (memTestAddressBus(BASE_ADDRESS, NUM_BYTES) != NULL)
      {
         term_printf("ERROR\r\n");
      }
      else
      {
         term_printf("OK\r\n");
         term_printf("  Device Test... ");
         if (memTestDevice(BASE_ADDRESS, NUM_BYTES) != NULL)
         {
            term_printf("ERROR\r\n");
         }
         else
         {
            term_printf("OK\r\n");
            term_printf("Memory test successfully completed.\r\n");
            
            rc = 0;
         }
      }
   }
   
   return(rc);
} /* MemoryTest */

/*=======================================================================*/
/*  Definition of all local Procedures                                   */
/*=======================================================================*/

/*************************************************************************/
/*  neorv32_Init                                                         */
/*                                                                       */
/*  Initializs the NEORV32 system.                                       */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: 0 = OK / -1 = ERROR                                          */
/*************************************************************************/
static int neorv32_Init (void)
{
   /* Start the Terminal functionality */
   term_Start();

   /* 
    * Check if GPIO unit is implemented at all
    */
   if (0 == neorv32_gpio_available()) 
   {
      term_printf("\r\nError! No GPIO unit synthesized!\r\n");
      return(-1);
   }

   /*
    * Capture all exceptions and give debug info via UART
    * this is not required, but keeps us safe.
    */
   neorv32_rte_setup();

   return(0);   
} /* neorv32_Init */

/*=======================================================================*/
/*  All code exported                                                    */
/*=======================================================================*/

/*************************************************************************/
/*  main                                                                 */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: never                                                        */
/*************************************************************************/
int main (void)
{
   uint8_t    value;
   
   /*
    * Initialize the neorv32 system
    */
   if (neorv32_Init() != 0) return(1);

   /*
    * Output startup messages
    */
   OutputBootMessage();
    
   /* Clear gpio output */
   neorv32_gpio_port_set(0);
   
   /* Memory test */ 
   MemoryTest();
   
   
   term_printf("\r\nScotty! Energie, starting running light...\r\n");

   /*
    * Running light
    */   
   while (1) 
   {
      for (int i = 0; i < 8; i++)
      {
         value = (1<<i);
         neorv32_gpio_port_set(value & 0xFF);
         
         /* Wait 200ms using busy wait */
         neorv32_aux_delay_ms(neorv32_sysinfo_get_clk(), 200);
      }
   }      
  
   /*
    * This return here make no sense.
    * But to prevent the compiler warning:
    *    "return type of 'main' is not 'int'
    * We use an int as return :-)
    */ 
   return(0);
} /* main */

/*** EOF ***/

