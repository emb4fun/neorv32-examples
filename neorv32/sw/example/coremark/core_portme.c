/*
Copyright 2018 Embedded Microprocessor Benchmark Consortium (EEMBC)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

Original Author: Shay Gal-on
*/

/* Modified for the NEORV32 Processor - by Stephan Nolting */

#include "coremark.h"
#include "core_portme.h"

#if VALIDATION_RUN
volatile ee_s32 seed1_volatile = 0x3415;
volatile ee_s32 seed2_volatile = 0x3415;
volatile ee_s32 seed3_volatile = 0x66;
#endif
#if PERFORMANCE_RUN
volatile ee_s32 seed1_volatile = 0x0;
volatile ee_s32 seed2_volatile = 0x0;
volatile ee_s32 seed3_volatile = 0x66;
#endif
#if PROFILE_RUN
volatile ee_s32 seed1_volatile = 0x8;
volatile ee_s32 seed2_volatile = 0x8;
volatile ee_s32 seed3_volatile = 0x8;
#endif
volatile ee_s32 seed4_volatile = ITERATIONS;
volatile ee_s32 seed5_volatile = 0;
/* Porting : Timing functions
        How to capture time and convert to seconds must be ported to whatever is
   supported by the platform. e.g. Read value from on board RTC, read value from
   cpu clock cycles performance counter etc. Sample implementation for standard
   time.h and windows.h definitions included.
*/
CORETIMETYPE
barebones_clock()
{
/*
#error \
    "You must implement a method to measure time in barebones_clock()! This function should return current time.\n"
*/
  return 1;
}
/* Define : TIMER_RES_DIVIDER
        Divider to trade off timer resolution and total time that can be
   measured.

        Use lower values to increase resolution, but make sure that overflow
   does not occur. If there are issues with the return value overflowing,
   increase this value.
        */
#define GETMYTIME(_t)              (*_t = (CORETIMETYPE)neorv32_cpu_get_cycle())
#define MYTIMEDIFF(fin, ini)       ((fin) - (ini))
#define TIMER_RES_DIVIDER          1
#define SAMPLE_TIME_IMPLEMENTATION 1
#define EE_TICKS_PER_SEC           (CLOCKS_PER_SEC / TIMER_RES_DIVIDER)

/** Define Host specific (POSIX), or target specific global time variables. */
static CORETIMETYPE start_time_val, stop_time_val;

/* Function : start_time
        This function will be called right before starting the timed portion of
   the benchmark.

        Implementation may be capturing a system timer (as implemented in the
   example code) or zeroing some system parameters - e.g. setting the cpu clocks
   cycles to 0.
*/
void
start_time(void)
{
    GETMYTIME(&start_time_val);
    neorv32_cpu_csr_write(CSR_MCOUNTINHIBIT, 0); // start all counters
}
/* Function : stop_time
        This function will be called right after ending the timed portion of the
   benchmark.

        Implementation may be capturing a system timer (as implemented in the
   example code) or other system parameters - e.g. reading the current value of
   cpu cycles counter.
*/
void
stop_time(void)
{
    neorv32_cpu_csr_write(CSR_MCOUNTINHIBIT, -1); // stop all counters
    GETMYTIME(&stop_time_val);
}
/* Function : get_time
        Return an abstract "ticks" number that signifies time on the system.

        Actual value returned may be cpu cycles, milliseconds or any other
   value, as long as it can be converted to seconds by <time_in_secs>. This
   methodology is taken to accommodate any hardware or simulated platform. The
   sample implementation returns milliseconds by default, and the resolution is
   controlled by <TIMER_RES_DIVIDER>
*/
CORE_TICKS
get_time(void)
{
    CORE_TICKS elapsed
        = (CORE_TICKS)(MYTIMEDIFF(stop_time_val, start_time_val));
    return elapsed;
}
/* Function : time_in_secs
        Convert the value returned by get_time to seconds.

        The <secs_ret> type is used to accomodate systems with no support for
   floating point. Default implementation implemented by the EE_TICKS_PER_SEC
   macro above.
*/
secs_ret
time_in_secs(CORE_TICKS ticks)
{
    /* NEORV32-specific */
    secs_ret retval = (secs_ret)(((CORE_TICKS)ticks) / ((CORE_TICKS)NEORV32_SYSINFO->CLK));
    return retval;
}

ee_u32 default_num_contexts = 1;

/* Number of available hardware performance monitors */
uint32_t num_hpm_cnts_global = 0;


/* Function : portable_init
        Target specific initialization code
        Test for some common mistakes.
*/
#ifndef RUN_COREMARK
void
__attribute__((__noreturn__))
portable_init(core_portable *p, int *argc, char *argv[])
#else
void
portable_init(core_portable *p, int *argc, char *argv[])
#endif
{
  /* NEORV32-specific */
  neorv32_cpu_csr_write(CSR_MIE, 0); // no interrupt, thanks
  neorv32_rte_setup(); // capture all exceptions and give debug information, no HW flow control

  // setup UART at default baud rate, no interrupts
  neorv32_uart0_setup(BAUD_RATE, 0);


// Disable coremark compilation by default
#ifndef RUN_COREMARK
  #warning COREMARK HAS NOT BEEN COMPILED! Use >>make USER_FLAGS+=-DRUN_COREMARK clean_all exe<< to compile it.

  // inform the user if you are actually executing this
  neorv32_uart0_printf("ERROR! CoreMark has not been compiled. Use >>make USER_FLAGS+=-DRUN_COREMARK clean_all exe<< to compile it.\n");

  while(1);
#endif

  // check available hardware extensions and compare with compiler flags
  neorv32_rte_check_isa(0); // silent = 0 -> show message if isa mismatch

  num_hpm_cnts_global = neorv32_cpu_hpm_get_num_counters();

  // stop all counters for now
  neorv32_cpu_csr_write(CSR_MCOUNTINHIBIT, -1);
  neorv32_cpu_csr_write(CSR_MCOUNTEREN, -1); // enable access to all counters

  // try to setup as many counters/HPMs as possible
  neorv32_cpu_set_mcycle(0);
  neorv32_cpu_csr_write(CSR_MHPMCOUNTER3,  0); neorv32_cpu_csr_write(CSR_MHPMEVENT3,  1 << HPMCNT_EVENT_CIR);
  neorv32_cpu_csr_write(CSR_MHPMCOUNTER4,  0); neorv32_cpu_csr_write(CSR_MHPMEVENT4,  1 << HPMCNT_EVENT_WAIT_IF);
  neorv32_cpu_csr_write(CSR_MHPMCOUNTER5,  0); neorv32_cpu_csr_write(CSR_MHPMEVENT5,  1 << HPMCNT_EVENT_WAIT_II);
  neorv32_cpu_csr_write(CSR_MHPMCOUNTER6,  0); neorv32_cpu_csr_write(CSR_MHPMEVENT6,  1 << HPMCNT_EVENT_WAIT_MC);
  neorv32_cpu_csr_write(CSR_MHPMCOUNTER7,  0); neorv32_cpu_csr_write(CSR_MHPMEVENT7,  1 << HPMCNT_EVENT_LOAD);
  neorv32_cpu_csr_write(CSR_MHPMCOUNTER8,  0); neorv32_cpu_csr_write(CSR_MHPMEVENT8,  1 << HPMCNT_EVENT_STORE);
  neorv32_cpu_csr_write(CSR_MHPMCOUNTER9,  0); neorv32_cpu_csr_write(CSR_MHPMEVENT9,  1 << HPMCNT_EVENT_WAIT_LS);
  neorv32_cpu_csr_write(CSR_MHPMCOUNTER10, 0); neorv32_cpu_csr_write(CSR_MHPMEVENT10, 1 << HPMCNT_EVENT_JUMP);
  neorv32_cpu_csr_write(CSR_MHPMCOUNTER11, 0); neorv32_cpu_csr_write(CSR_MHPMEVENT11, 1 << HPMCNT_EVENT_BRANCH);
  neorv32_cpu_csr_write(CSR_MHPMCOUNTER12, 0); neorv32_cpu_csr_write(CSR_MHPMEVENT12, 1 << HPMCNT_EVENT_TBRANCH);
  neorv32_cpu_csr_write(CSR_MHPMCOUNTER13, 0); neorv32_cpu_csr_write(CSR_MHPMEVENT13, 1 << HPMCNT_EVENT_TRAP);
  neorv32_cpu_csr_write(CSR_MHPMCOUNTER14, 0); neorv32_cpu_csr_write(CSR_MHPMEVENT14, 1 << HPMCNT_EVENT_ILLEGAL);

  neorv32_uart0_printf("NEORV32: Processor running at %u Hz\n", (uint32_t)NEORV32_SYSINFO->CLK);
  neorv32_uart0_printf("NEORV32: Executing coremark (%u iterations). This may take some time...\n\n", (uint32_t)ITERATIONS);

/*
#error \
    "Call board initialization routines in portable init (if needed), in particular initialize UART!\n"
*/
    if (sizeof(ee_ptr_int) != sizeof(ee_u8 *))
    {
        ee_printf(
            "ERROR! Please define ee_ptr_int to a type that holds a "
            "pointer!\n");
    }
    if (sizeof(ee_u32) != 4)
    {
        ee_printf("ERROR! Please define ee_u32 to a 32b unsigned type!\n");
    }
    p->portable_id = 1;

#ifndef RUN_COREMARK
  while(1);
#endif
}


/* Function : portable_fini
        Target specific final code
*/
void
portable_fini(core_portable *p)
{
    p->portable_id = 0;

    neorv32_uart0_printf("\nNEORV32: Hardware Performance Monitors (low words only)\n");
    neorv32_uart0_printf(" > Active clock cycles:          %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MCYCLE));
    neorv32_uart0_printf(" > Retired instructions:         %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MINSTRET));
    if (num_hpm_cnts_global == 0) {neorv32_uart0_printf("no HPMs available\n"); }
    if (num_hpm_cnts_global > 0)  {neorv32_uart0_printf(" > Retired compr. instructions:  %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER3)); }
    if (num_hpm_cnts_global > 1)  {neorv32_uart0_printf(" > Instr.-fetch wait cycles:     %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER4)); }
    if (num_hpm_cnts_global > 2)  {neorv32_uart0_printf(" > Instr.-issue wait cycles:     %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER5)); }
    if (num_hpm_cnts_global > 3)  {neorv32_uart0_printf(" > Multi-cycle ALU wait cycles:  %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER6)); }
    if (num_hpm_cnts_global > 4)  {neorv32_uart0_printf(" > Load operations:              %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER7)); }
    if (num_hpm_cnts_global > 5)  {neorv32_uart0_printf(" > Store operations:             %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER8)); }
    if (num_hpm_cnts_global > 6)  {neorv32_uart0_printf(" > Load/store wait cycles:       %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER9)); }
    if (num_hpm_cnts_global > 7)  {neorv32_uart0_printf(" > Unconditional jumps:          %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER10)); }
    if (num_hpm_cnts_global > 8)  {neorv32_uart0_printf(" > Conditional branches (all):   %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER11)); }
    if (num_hpm_cnts_global > 9)  {neorv32_uart0_printf(" > Conditional branches (taken): %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER12)); }
    if (num_hpm_cnts_global > 10) {neorv32_uart0_printf(" > Entered traps:                %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER13)); }
    if (num_hpm_cnts_global > 11) {neorv32_uart0_printf(" > Illegal operations:           %u\n", (uint32_t)neorv32_cpu_csr_read(CSR_MHPMCOUNTER14)); }
    neorv32_uart0_printf("\n");
}
