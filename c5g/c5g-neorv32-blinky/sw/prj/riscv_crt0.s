// **********************************************************************
// *                    SEGGER Microcontroller GmbH                     *
// *                        The Embedded Experts                        *
// **********************************************************************
// *                                                                    *
// *            (c) 2014 - 2021 SEGGER Microcontroller GmbH             *
// *            (c) 2001 - 2021 Rowley Associates Limited               *
// *                                                                    *
// *           www.segger.com     Support: support@segger.com           *
// *                                                                    *
// **********************************************************************
// *                                                                    *
// * All rights reserved.                                               *
// *                                                                    *
// * Redistribution and use in source and binary forms, with or         *
// * without modification, are permitted provided that the following    *
// * condition is met:                                                  *
// *                                                                    *
// * - Redistributions of source code must retain the above copyright   *
// *   notice, this condition and the following disclaimer.             *
// *                                                                    *
// * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND             *
// * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,        *
// * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF           *
// * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE           *
// * DISCLAIMED. IN NO EVENT SHALL SEGGER Microcontroller BE LIABLE FOR *
// * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR           *
// * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT  *
// * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;    *
// * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF      *
// * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT          *
// * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE  *
// * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH   *
// * DAMAGE.                                                            *
// *                                                                    *
// **********************************************************************

#ifndef ARGSSPACE
#define ARGSSPACE 128
#endif

.global __global_pointer$
__global_pointer$=__sdata_start__+0x800

  .section .init, "ax", %progbits

  .global _start
  .type _start, function
_start:
  .option push
  .option norelax
  la gp, __global_pointer$
  .option pop
#ifdef __nds_execit
  la a0, _ITB_BASE_
  csrrw x0, uitb, a0
#endif
  //la sp, __stack_end__
  lui t0, %hi(__stack_end__)
  addi sp, t0, %lo(__stack_end__)
  la tp, __tbss_start__
  la a0, trap_entry
  csrw mtvec, a0
  csrw mcause, x0

  /* Call _init */
  la t1, _init
  jalr t1

  /* Load fast section */
  la a0, __fast_load_start__
  la a1, __fast_start__
  la a2, __fast_end__
  bgeu a1, a2, 2f
1:
  lw t0, (a0)
  sw t0, (a1)
  addi a0, a0, 4
  addi a1, a1, 4
  bltu a1, a2, 1b
2:

  /* Load data section */
  la a0, __data_load_start__
  la a1, __data_start__
  la a2, __data_end__
  bgeu a1, a2, 2f
1:
  lw t0, (a0)
  sw t0, (a1)
  addi a0, a0, 4
  addi a1, a1, 4
  bltu a1, a2, 1b
2:

  /* Load tdata section */
  la a0, __tdata_load_start__
  la a1, __tdata_start__
  la a2, __tdata_end__
  bgeu a1, a2, 2f
1:
  lw t0, (a0)
  sw t0, (a1)
  addi a0, a0, 4
  addi a1, a1, 4
  bltu a1, a2, 1b
2:

  /* Load sdata section */
  la a0, __sdata_load_start__
  la a1, __sdata_start__
  la a2, __sdata_end__
  bgeu a1, a2, 2f
1:
  lw t0, (a0)
  sw t0, (a1)
  addi a0, a0, 4
  addi a1, a1, 4
  bltu a1, a2, 1b
2:

  /* Load rodata section */
  la a0, __rodata_load_start__
  la a1, __rodata_start__
  beq a0, a1, 2f
  la a2, __rodata_end__
  bgeu a1, a2, 2f
1:
  lw t0, (a0)
  sw t0, (a1)
  addi a0, a0, 4
  addi a1, a1, 4
  bltu a1, a2, 1b
2:

  /* Zero bss section */
  la a0, __bss_start__
  la a1, __bss_end__
  bgeu a0, a1, 2f
1:
  sw zero, (a0)
  addi a0, a0, 4
  bltu a0, a1, 1b
2:

  /* Zero tbss section */
  la a0, __tbss_start__
  la a1, __tbss_end__
  bgeu a0, a1, 2f
1:
  sw zero, (a0)
  addi a0, a0, 4
  bltu a0, a1, 1b
2:

  /* Zero sbss section */
  la a0, __sbss_start__
  la a1, __sbss_end__
  bgeu a0, a1, 2f
1:
  sw zero, (a0)
  addi a0, a0, 4
  bltu a0, a1, 1b
2:

#if !defined(__HEAP_SIZE__) || (__HEAP_SIZE__)
  /* Initialize the heap */
  la a0, __heap_start__
  la a1, __heap_end__
  sub a1, a1, a0
  sw zero, 0(a0)  
  sw a1, 4(a0)
1:
#endif

  .global start
  .type start, function
start:

  /* Call constructors */
  la s0, __ctors_start__
  la s1, __ctors_end__
1:
  beq s0, s1, 2f
  lw t1, 0(s0)
  addi s0, s0, 4
  jalr t1
  j 1b
2:

  /* Jump to application entry point */
#ifdef FULL_LIBRARY
  li a0, ARGSSPACE
  la a1, args
  la t1, debug_getargs  
  jalr t1
  li a0, ARGSSPACE
  la a1, args
#else
  li a0, 0
  li a1, 0
#endif	
  la t1, main
  jalr t1

  .global exit
  .type exit, function
exit:
#ifdef FULL_LIBRARY  
  mv s1, a0 // save the exit parameter/return result

  /* Call destructors */
  la s0, __dtors_start__
1:
  la t0, __dtors_end__
  beq s0, t0, 2f  
  lw t1, 0(s0)
  addi s0, s0, 4
  jalr t1  
  j 1b
2:

  /* Call atexit functions */
  la t1, _execute_at_exit_fns    
  jalr t1

  /* Call debug_exit with return result/exit parameter */
  mv a0, s1
  la t1, debug_exit
  jalr t1
#endif

  /* Call _fini */
  la t1, _fini
  jalr t1

  /* Returned from application entry point, loop forever. */
exit_loop:
  j .

#define MSTATUS_MPP 0x00001800

#define NUM_CALLER_SAVE_REGISTERS (1+3+8+4) // save caller integer registers ra, t0-t2, a0-a7, t3-t6

  .section .text.trap_entry, "ax", %progbits
  .global trap_entry
  .align 2
trap_entry:  
  addi sp, sp, -NUM_CALLER_SAVE_REGISTERS*4
  sw ra,  0*4(sp)
  sw t0,  1*4(sp)
  sw t1,  2*4(sp)
  sw t2,  3*4(sp)
  sw a0,  4*4(sp)
  sw a1,  5*4(sp)
  sw a2,  6*4(sp)
  sw a3,  7*4(sp)
  sw a4,  8*4(sp)
  sw a5,  9*4(sp)
#ifndef __riscv_abi_rve
  sw a6, 10*4(sp)
  sw a7, 11*4(sp)
  sw t3, 12*4(sp)
  sw t4, 13*4(sp)
  sw t5, 14*4(sp)
  sw t6, 15*4(sp)
#endif

  csrr a0, mcause
  csrr a1, mepc 
  /* call handle_trap */
  la t1, handle_trap
  jalr t1
  csrw mepc, a0

  lw ra,  0*4(sp)
  lw t0,  1*4(sp)
  lw t1,  2*4(sp)
  lw t2,  3*4(sp)
  lw a0,  4*4(sp)
  lw a1,  5*4(sp)
  lw a2,  6*4(sp)
  lw a3,  7*4(sp)
  lw a4,  8*4(sp)
  lw a5,  9*4(sp)
#ifndef __riscv_abi_rve
  lw a6, 10*4(sp)
  lw a7, 11*4(sp)
  lw t3, 12*4(sp)
  lw t4, 13*4(sp)
  lw t5, 14*4(sp)
  lw t6, 15*4(sp)
#endif

  addi sp, sp, NUM_CALLER_SAVE_REGISTERS*4
  mret

  .weak handle_trap
handle_trap:
  j .

  .weak _init
 _init:
   ret

  .weak _fini
 _fini:
   ret

// default C/C++ library helpers

.macro HELPER helper_name
  .section .text.\helper_name, "ax", %progbits
  .weak \helper_name
#if __riscv_compressed
  .align 1
#else
  .align 2
#endif
\helper_name:
  .type \helper_name,@function
.endm

HELPER abort
  j .
HELPER __assert
  j .
HELPER __sync_synchronize
  ret
HELPER __getchar
  la t0, debug_getchar
  jr t0
HELPER __putchar
  la t0, debug_putchar
  jr t0
HELPER __open
  la t0, debug_fopen
  jr t0
HELPER __close
  la t0, debug_fclose
  jr t0
HELPER __read
  mv a3, a0
  mv a0, a1
  li a1, 1  
  la t0, debug_fread
  jr t0
HELPER __write
  mv a3, a0
  mv a0, a1
  li a1, 1  
  la t0, debug_fwrite
  jr t0
HELPER __seek
  addi sp, sp, -8
  sw a0, 0*4(sp)
  sw ra, 1*4(sp)  
  la t0, debug_fseek
  jalr t0
  bnez a0, 1f
  lw a0, 0*4(sp)
  la t0, debug_ftell
  jalr t0
  j 2f
1:
  li a0, -1
2:
  lw a4, 0*4(sp)
  lw ra, 1*4(sp)
  addi sp, sp, 8
  ret
  // char __user_locale_name_buffer[];
  .section .bss.__user_locale_name_buffer, "aw", %nobits
  .weak __user_locale_name_buffer
  __user_locale_name_buffer:
  .word 0x0

#ifdef FULL_LIBRARY
  .bss
args:
  .space ARGSSPACE
#endif

  /* Setup attibutes of stack and heap sections so they don't take up room in the elf file */
  .section .stack, "wa", %nobits 
  .section .heap, "wa", %nobits
