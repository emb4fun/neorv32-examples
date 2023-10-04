// #################################################################################################
// # << NEORV32: neorv32_cpu.h - CPU Core Functions HW Driver >>                                   #
// # ********************************************************************************************* #
// # BSD 3-Clause License                                                                          #
// #                                                                                               #
// # Copyright (c) 2023, Stephan Nolting. All rights reserved.                                     #
// #                                                                                               #
// # Redistribution and use in source and binary forms, with or without modification, are          #
// # permitted provided that the following conditions are met:                                     #
// #                                                                                               #
// # 1. Redistributions of source code must retain the above copyright notice, this list of        #
// #    conditions and the following disclaimer.                                                   #
// #                                                                                               #
// # 2. Redistributions in binary form must reproduce the above copyright notice, this list of     #
// #    conditions and the following disclaimer in the documentation and/or other materials        #
// #    provided with the distribution.                                                            #
// #                                                                                               #
// # 3. Neither the name of the copyright holder nor the names of its contributors may be used to  #
// #    endorse or promote products derived from this software without specific prior written      #
// #    permission.                                                                                #
// #                                                                                               #
// # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS   #
// # OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF               #
// # MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE    #
// # COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,     #
// # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE #
// # GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED    #
// # AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING     #
// # NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED  #
// # OF THE POSSIBILITY OF SUCH DAMAGE.                                                            #
// # ********************************************************************************************* #
// # The NEORV32 Processor - https://github.com/stnolting/neorv32              (c) Stephan Nolting #
// #################################################################################################


/**********************************************************************//**
 * @file neorv32_cpu.h
 * @brief CPU Core Functions HW driver header file.
 **************************************************************************/

#ifndef neorv32_cpu_h
#define neorv32_cpu_h


/**********************************************************************//**
 * @name Prototypes
 **************************************************************************/
/**@{*/
void     neorv32_cpu_irq_enable(int irq_sel);
void     neorv32_cpu_irq_disable(int irq_sel);
uint64_t neorv32_cpu_get_cycle(void);
void     neorv32_cpu_set_mcycle(uint64_t value);
uint64_t neorv32_cpu_get_instret(void);
void     neorv32_cpu_set_minstret(uint64_t value);
void     neorv32_cpu_delay_ms(uint32_t time_ms);
uint32_t neorv32_cpu_get_clk_from_prsc(int prsc);
uint32_t neorv32_cpu_pmp_get_num_regions(void);
uint32_t neorv32_cpu_pmp_get_granularity(void);
int      neorv32_cpu_pmp_configure_region(int index, uint32_t addr, uint8_t config);
uint32_t neorv32_cpu_hpm_get_num_counters(void);
uint32_t neorv32_cpu_hpm_get_size(void);
void     neorv32_cpu_goto_user_mode(void);
/**@}*/


// #################################################################################################
// Load/store
// #################################################################################################


/**********************************************************************//**
 * Store unsigned word to address space.
 *
 * @note An unaligned access address will raise an alignment exception.
 *
 * @param[in] addr Address (32-bit).
 * @param[in] wdata Data word (32-bit) to store.
 **************************************************************************/
inline void __attribute__ ((always_inline)) neorv32_cpu_store_unsigned_word(uint32_t addr, uint32_t wdata) {

  uint32_t reg_addr = addr;
  uint32_t reg_data = wdata;

  asm volatile ("sw %[da], 0(%[ad])" : : [da] "r" (reg_data), [ad] "r" (reg_addr));
}


/**********************************************************************//**
 * Store unsigned half-word to address space.
 *
 * @note An unaligned access address will raise an alignment exception.
 *
 * @param[in] addr Address (32-bit).
 * @param[in] wdata Data half-word (16-bit) to store.
 **************************************************************************/
inline void __attribute__ ((always_inline)) neorv32_cpu_store_unsigned_half(uint32_t addr, uint16_t wdata) {

  uint32_t reg_addr = addr;
  uint32_t reg_data = (uint32_t)wdata;

  asm volatile ("sh %[da], 0(%[ad])" : : [da] "r" (reg_data), [ad] "r" (reg_addr));
}


/**********************************************************************//**
 * Store unsigned byte to address space.
 *
 * @param[in] addr Address (32-bit).
 * @param[in] wdata Data byte (8-bit) to store.
 **************************************************************************/
inline void __attribute__ ((always_inline)) neorv32_cpu_store_unsigned_byte(uint32_t addr, uint8_t wdata) {

  uint32_t reg_addr = addr;
  uint32_t reg_data = (uint32_t)wdata;

  asm volatile ("sb %[da], 0(%[ad])" : : [da] "r" (reg_data), [ad] "r" (reg_addr));
}


/**********************************************************************//**
 * Load unsigned word from address space.
 *
 * @note An unaligned access address will raise an alignment exception.
 *
 * @param[in] addr Address (32-bit).
 * @return Read data word (32-bit).
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) neorv32_cpu_load_unsigned_word(uint32_t addr) {

  uint32_t reg_addr = addr;
  uint32_t reg_data;

  asm volatile ("lw %[da], 0(%[ad])" : [da] "=r" (reg_data) : [ad] "r" (reg_addr));

  return reg_data;
}


/**********************************************************************//**
 * Load unsigned half-word from address space.
 *
 * @note An unaligned access address will raise an alignment exception.
 *
 * @param[in] addr Address (32-bit).
 * @return Read data half-word (16-bit).
 **************************************************************************/
inline uint16_t __attribute__ ((always_inline)) neorv32_cpu_load_unsigned_half(uint32_t addr) {

  uint32_t reg_addr = addr;
  uint16_t reg_data;

  asm volatile ("lhu %[da], 0(%[ad])" : [da] "=r" (reg_data) : [ad] "r" (reg_addr));

  return reg_data;
}


/**********************************************************************//**
 * Load signed half-word from address space.
 *
 * @note An unaligned access address will raise an alignment exception.
 *
 * @param[in] addr Address (32-bit).
 * @return Read data half-word (16-bit).
 **************************************************************************/
inline int16_t __attribute__ ((always_inline)) neorv32_cpu_load_signed_half(uint32_t addr) {

  uint32_t reg_addr = addr;
  int16_t reg_data;

  asm volatile ("lh %[da], 0(%[ad])" : [da] "=r" (reg_data) : [ad] "r" (reg_addr));

  return reg_data;
}


/**********************************************************************//**
 * Load unsigned byte from address space.
 *
 * @param[in] addr Address (32-bit).
 * @return Read data byte (8-bit).
 **************************************************************************/
inline uint8_t __attribute__ ((always_inline)) neorv32_cpu_load_unsigned_byte(uint32_t addr) {

  uint32_t reg_addr = addr;
  uint8_t reg_data;

  asm volatile ("lbu %[da], 0(%[ad])" : [da] "=r" (reg_data) : [ad] "r" (reg_addr));

  return reg_data;
}


/**********************************************************************//**
 * Load signed byte from address space.
 *
 * @param[in] addr Address (32-bit).
 * @return Read data byte (8-bit).
 **************************************************************************/
inline int8_t __attribute__ ((always_inline)) neorv32_cpu_load_signed_byte(uint32_t addr) {

  uint32_t reg_addr = addr;
  int8_t reg_data;

  asm volatile ("lb %[da], 0(%[ad])" : [da] "=r" (reg_data) : [ad] "r" (reg_addr));

  return reg_data;
}


// #################################################################################################
// Atomic memory access / load-reservate/store-conditional
// #################################################################################################


/**********************************************************************//**
 * Atomic memory access: load-reservate word.
 *
 * @note The address has to be word-aligned - otherwise an alignment exception will be raised.
 * @warning This function requires the A ISA extension.
 *
 * @param[in] addr Address (32-bit).
 * @return Read data word (32-bit).
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) neorv32_cpu_load_reservate_word(uint32_t addr) {

#if defined __riscv_atomic
  uint32_t amo_addr = addr;
  uint32_t amo_rdata;

  asm volatile ("lr.w %[dst], 0(%[addr])" : [dst] "=r" (amo_rdata) : [addr] "r" (amo_addr));

  return amo_rdata;
#else
  return 0;
#endif
}


/**********************************************************************//**
 * Atomic memory access: store-conditional word.
 *
 * @note The address has to be word-aligned - otherwise an alignment exception will be raised.
 * @warning This function requires the A ISA extension.
 *
 * @param[in] addr Address (32-bit).
 * @param[in] wdata Data word to-be-written conditionally (32-bit).
 * @return Status: 0 = ok, 1 = failed (32-bit).
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) neorv32_cpu_store_conditional_word(uint32_t addr, uint32_t wdata) {

#if defined __riscv_atomic
  uint32_t amo_addr  = addr;
  uint32_t amo_wdata = wdata;
  uint32_t amo_status;

  asm volatile ("sc.w %[dst], %[src], (%[addr])" : [dst] "=r" (amo_status) : [src] "r" (amo_wdata), [addr] "r" (amo_addr));

  return amo_status;
#else
  return 1; // always fail
#endif
}


/**********************************************************************//**
 * Atomic memory access: invalidate (all) current reservation sets
 *
 * @warning This function requires the A ISA extension.
 **************************************************************************/
inline void __attribute__ ((always_inline)) neorv32_cpu_invalidate_reservations(void) {

#if defined __riscv_atomic
  asm volatile ("sc.w zero, zero, (zero)");
#endif
}


// #################################################################################################
// CSR access
// #################################################################################################


/**********************************************************************//**
 * Read data from CPU control and status register (CSR).
 *
 * @param[in] csr_id ID of CSR to read. See #NEORV32_CSR_enum.
 * @return Read data (uint32_t).
 **************************************************************************/
inline uint32_t __attribute__ ((always_inline)) neorv32_cpu_csr_read(const int csr_id) {

  uint32_t csr_data;

  asm volatile ("csrr %[result], %[input_i]" : [result] "=r" (csr_data) : [input_i] "i" (csr_id));

  return csr_data;
}


/**********************************************************************//**
 * Write data to CPU control and status register (CSR).
 *
 * @param[in] csr_id ID of CSR to write. See #NEORV32_CSR_enum.
 * @param[in] data Data to write (uint32_t).
 **************************************************************************/
inline void __attribute__ ((always_inline)) neorv32_cpu_csr_write(const int csr_id, uint32_t data) {

  uint32_t csr_data = data;

  asm volatile ("csrw %[input_i], %[input_j]" :  : [input_i] "i" (csr_id), [input_j] "r" (csr_data));
}


/**********************************************************************//**
 * Set bit(s) in CPU control and status register (CSR).
 *
 * @param[in] csr_id ID of CSR to write. See #NEORV32_CSR_enum.
 * @param[in] mask Bit mask (high-active) to set bits (uint32_t).
 **************************************************************************/
inline void __attribute__ ((always_inline)) neorv32_cpu_csr_set(const int csr_id, uint32_t mask) {

  uint32_t csr_data = mask;

  asm volatile ("csrs %[input_i], %[input_j]" :  : [input_i] "i" (csr_id), [input_j] "r" (csr_data));
}


/**********************************************************************//**
 * Clear bit(s) in CPU control and status register (CSR).
 *
 * @param[in] csr_id ID of CSR to write. See #NEORV32_CSR_enum.
 * @param[in] mask Bit mask (high-active) to clear bits (uint32_t).
 **************************************************************************/
inline void __attribute__ ((always_inline)) neorv32_cpu_csr_clr(const int csr_id, uint32_t mask) {

  uint32_t csr_data = mask;

  asm volatile ("csrc %[input_i], %[input_j]" :  : [input_i] "i" (csr_id), [input_j] "r" (csr_data));
}


// #################################################################################################
// Misc
// #################################################################################################


/**********************************************************************//**
 * Prototype for "after-main handler". This function is called if main() returns.
 *
 * @param[in] return_code Return value of main() function.
 **************************************************************************/
extern void __attribute__ ((weak)) __neorv32_crt0_after_main(int32_t return_code);


/**********************************************************************//**
 * Put CPU into sleep / power-down mode.
 *
 * @note The WFI (wait for interrupt) instruction will make the CPU halt until
 * any enabled interrupt source becomes pending.
 **************************************************************************/
inline void __attribute__ ((always_inline)) neorv32_cpu_sleep(void) {

  asm volatile ("wfi");
}


#endif // neorv32_cpu_h
