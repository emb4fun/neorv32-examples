// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_pwm.c
 * @brief Pulse-Width Modulation Controller (PWM) HW driver source file.
 */

#include <neorv32.h>


/**********************************************************************//**
 * Check if PWM unit was synthesized.
 *
 * @return 0 if PWM was not synthesized, non-zero if PWM is available.
 **************************************************************************/
int neorv32_pwm_available(void) {

  return(int)(NEORV32_SYSINFO->SOC & (1 << SYSINFO_SOC_IO_PWM));
}


/**********************************************************************//**
 * Get number of implemented PWM channels.
 * @warning This function will override all channel configuration registers.
 *
 * @return Number of implemented PWM channels.
 **************************************************************************/
int neorv32_pmw_get_num_channels(void) {

  int i = 0;
  uint32_t cnt = 0;

  for (i=0; i<16; i++) {
    NEORV32_PWM->CHANNEL_CFG[i] = 1;
    cnt += NEORV32_PWM->CHANNEL_CFG[i];
  }

  return (int)cnt;
}


/**********************************************************************//**
 * Enable PWM channel.
 *
 * @param[in] channel Channel select (0..15).
 **************************************************************************/
void neorv32_pwm_ch_enable(int channel) {

  channel &= 0xf; // constrain range

  NEORV32_PWM->CHANNEL_CFG[channel] |= ((uint32_t)(1 << PWM_CFG_EN));
}


/**********************************************************************//**
 * Disable PWM channel.
 *
 * @param[in] channel Channel select (0..15).
 **************************************************************************/
void neorv32_pwm_ch_disable(int channel) {

  channel &= 0xf; // constrain range

  NEORV32_PWM->CHANNEL_CFG[channel] &= ~((uint32_t)(1 << PWM_CFG_EN));
}


/**********************************************************************//**
 * Set PWM channel's polarity configuration.
 *
 * @param[in] channel Channel select (0..15).
 * @param[in] normal polarity if false (default), inverted polarity if true
 **************************************************************************/
void neorv32_pwm_ch_set_polarity(int channel, bool inverted) {

  channel &= 0xf; // constrain range

  if (inverted) {
    NEORV32_PWM->CHANNEL_CFG[channel] |= ((uint32_t)(1 << PWM_CFG_POL));
  } else {
    NEORV32_PWM->CHANNEL_CFG[channel] &= ~((uint32_t)(1 << PWM_CFG_POL));
  }
}


/**********************************************************************//**
 * Set PWM channel's clock configuration.
 *
 * @param[in] channel Channel select (0..15).
 * @param[in] prsc Coarse clock prescaler select (3-bit, LSB-aligned). See #NEORV32_CLOCK_PRSC_enum.
 * @param[in] cdiv Fine clock divider value (10-bit, LSB-aligned).
 **************************************************************************/
void neorv32_pwm_ch_set_clock(int channel, int prsc, int cdiv) {

  channel &= 0xf; // constrain range

  uint32_t tmp = NEORV32_PWM->CHANNEL_CFG[channel];
  tmp &= 0x880000ffU; // clear current prsc and cdiv, keep enable, polarity, and duty
  tmp |= ((uint32_t)(prsc & 0x7U))   << PWM_CFG_PRSC_LSB;
  tmp |= ((uint32_t)(cdiv & 0x3ffU)) << PWM_CFG_CDIV_LSB;
  NEORV32_PWM->CHANNEL_CFG[channel] = tmp;
}


/**********************************************************************//**
 * Set PWM channel's duty cycle.
 *
 * @param[in] channel Channel select (0..15).
 * @param[in] duty Duty cycle (8-bit, LSB-aligned).
 **************************************************************************/
void neorv32_pwm_ch_set_duty(int channel, int duty) {

  channel &= 0xf; // constrain range

  uint32_t tmp = NEORV32_PWM->CHANNEL_CFG[channel];
  tmp &= 0xffffff00U; // clear current duty cycle configuration
  tmp |= (uint32_t)(duty & 0x000000ffU); // set new configuration
  NEORV32_PWM->CHANNEL_CFG[channel] = tmp;
}
