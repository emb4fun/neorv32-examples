// #################################################################################################
// # << NEORV32 - SPI Bus Explorer Demo Program >>                                                 #
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
 * @file demo_spi/main.c
 * @author Stephan Nolting
 * @brief SPI bus explorer (execute SPI transactions by hand).
 **************************************************************************/

#include <neorv32.h>
#include <string.h>


/**********************************************************************//**
 * @name User configuration
 **************************************************************************/
/**@{*/
/** UART BAUD rate */
#define BAUD_RATE 19200
/**@}*/


// Global variables
uint32_t spi_configured;

// Prototypes
void spi_cs(uint32_t type);
void spi_trans(void);
void spi_setup(void);
uint32_t hexstr_to_uint(char *buffer, uint8_t length);
void aux_print_hex_byte(uint8_t byte);


/**********************************************************************//**
 * This program provides an interactive console to communicate with SPI devices.
 *
 * @note This program requires the UART and the SPI to be synthesized.
 *
 * @return Irrelevant.
 **************************************************************************/
int main() {

  char buffer[8];
  int length = 0;

  // capture all exceptions and give debug info via UART
  // this is not required, but keeps us safe
  neorv32_rte_setup();

  // setup UART at default baud rate, no interrupts
  neorv32_uart0_setup(BAUD_RATE, 0);


  // check if UART0 unit is implemented at all
  if (neorv32_uart0_available() == 0) {
    return 1;
  }

  // intro
  neorv32_uart0_printf("\n<<< SPI Bus Explorer >>>\n\n");

  // check if SPI unit is implemented at all
  if (neorv32_spi_available() == 0) {
    neorv32_uart0_printf("ERROR! No SPI unit implemented.");
    return 1;
  }

  // info
  neorv32_uart0_printf("This program allows to create SPI transfers by hand.\n"
                       "Type 'help' to see the help menu.\n\n");

  // disable and reset SPI module
  neorv32_spi_disable();
  spi_configured = 0; // SPI not configured yet


  // Main menu
  for (;;) {
    neorv32_uart0_printf("SPI_EXPLORER:> ");
    length = neorv32_uart0_scan(buffer, 15, 1);
    neorv32_uart0_printf("\n");

    if (!length) // nothing to be done
     continue;

    // decode input and execute command
    if (!strcmp(buffer, "help")) {
      neorv32_uart0_printf("Available commands:\n"
                          " help  - show this text\n"
                          " setup - configure SPI module\n"
                          " en    - enable single chip-select line (set low)\n"
                          " dis   - disable all chip-select lines (set high)\n"
                          " trans - SPI data transmission (write & read to/from SPI)\n"
                          "\n"
                          "Configure the SPI module using 'setup'. Enable a certain module using 'cs-en',\n"
                          "then transfer data using 'trans' and disable the module again using 'cs-dis'.\n"
                          "\n");
    }
    else if (!strcmp(buffer, "setup")) {
      spi_setup();
    }
    else if (!strcmp(buffer, "en")) {
      spi_cs(1);
    }
    else if (!strcmp(buffer, "dis")) {
      spi_cs(0);
    }
    else if (!strcmp(buffer, "trans")) {
      spi_trans();
    }
    else {
      neorv32_uart0_printf("Invalid command. Type 'help' to see all commands.\n");
    }
  }

  return 0;
}


/**********************************************************************//**
 * Enable or disable chip-select line
 *
 * @param[in] type 0=disable, 1=enable
 **************************************************************************/
void spi_cs(uint32_t type) {

  char terminal_buffer[2];
  uint8_t channel;

  if (type) {
    neorv32_uart0_printf("Chip-select line to ENABLE (set low) [0..7]: ");
    while (1) {
      neorv32_uart0_scan(terminal_buffer, 2, 1); // 1 hex char plus '\0'
      channel = (uint8_t)hexstr_to_uint(terminal_buffer, strlen(terminal_buffer));
      if (channel > 7) {
        neorv32_uart0_printf("\nInvalid channel selection!\n");
        return;
      }
      else {
        neorv32_uart0_printf("\n");
        break;
      }
    }
    neorv32_spi_cs_en(channel);
  }
  else {
    neorv32_uart0_printf("Disabling chip select lines.\n");
    neorv32_spi_cs_dis();
  }
}


/**********************************************************************//**
 * SPI data transfer
 **************************************************************************/
void spi_trans(void) {

  char terminal_buffer[4];

  if (spi_configured == 0) {
    neorv32_uart0_printf("SPI module not configured yet! Use 'setup' to configure SPI module.\n");
    return;
  }

  neorv32_uart0_printf("Enter TX data (2 hex chars): 0x");
  neorv32_uart0_scan(terminal_buffer, 2+1, 1);
  uint32_t tx_data = (uint32_t)hexstr_to_uint(terminal_buffer, strlen(terminal_buffer));

  uint32_t rx_data = neorv32_spi_trans(tx_data);

  neorv32_uart0_printf("\nTX data: 0x");
  aux_print_hex_byte(tx_data);
  neorv32_uart0_printf("\nRX data: 0x");
  aux_print_hex_byte(rx_data);
  neorv32_uart0_printf("\n");
}


/**********************************************************************//**
 * Configure SPI module
 **************************************************************************/
void spi_setup(void) {

  const uint32_t PRSC_LUT[8] = {2, 4, 8, 64, 128, 1024, 2048, 4096};

  char terminal_buffer[9];
  uint8_t spi_prsc, clk_div, clk_phase, clk_pol;
  uint32_t tmp;

  // ---- SPI clock ----

  while (1) {
    neorv32_uart0_printf("Select SPI clock prescaler (0..7): ");
    neorv32_uart0_scan(terminal_buffer, 2, 1);
    tmp = (uint32_t)hexstr_to_uint(terminal_buffer, strlen(terminal_buffer));
    if (tmp > 8) {
      neorv32_uart0_printf("\nInvalid selection!\n");
    }
    else {
      spi_prsc = (uint8_t)tmp;
      break;
    }
  }

  neorv32_uart0_printf("\nEnter clock divider (0..15, as one hex char): ");
  neorv32_uart0_scan(terminal_buffer, 2, 1);
  clk_div = (uint8_t)hexstr_to_uint(terminal_buffer, strlen(terminal_buffer));

  uint32_t clock = NEORV32_SYSINFO->CLK / (2 * PRSC_LUT[spi_prsc] * (1 + clk_div));
  neorv32_uart0_printf("\n+ New SPI clock speed = %u Hz\n", clock);

  // ---- SPI clock mode ----

  while (1) {
    neorv32_uart0_printf("Select SPI clock mode (0..3): ");
    neorv32_uart0_scan(terminal_buffer, 2, 1);
    tmp = (uint32_t)hexstr_to_uint(terminal_buffer, strlen(terminal_buffer));
    if (tmp > 4) {
      neorv32_uart0_printf("\nInvalid selection!\n");
    }
    else {
      clk_pol   = (uint8_t)((tmp >> 1) & 1);
      clk_phase = (uint8_t)(tmp & 1);
      break;
    }
  }
  neorv32_uart0_printf("\n+ New SPI clock mode = %u\n\n", tmp);

  neorv32_spi_setup(spi_prsc, clk_div, clk_phase, clk_pol, 0);
  spi_configured = 1; // SPI is configured now
}


/**********************************************************************//**
 * Helper function to convert N hex chars string into uint32_T
 *
 * @param[in,out] buffer Pointer to array of chars to convert into number.
 * @param[in,out] length Length of the conversion string.
 * @return Converted number.
 **************************************************************************/
uint32_t hexstr_to_uint(char *buffer, uint8_t length) {

  uint32_t res = 0, d = 0;
  char c = 0;

  while (length--) {
    c = *buffer++;

    if ((c >= '0') && (c <= '9'))
      d = (uint32_t)(c - '0');
    else if ((c >= 'a') && (c <= 'f'))
      d = (uint32_t)((c - 'a') + 10);
    else if ((c >= 'A') && (c <= 'F'))
      d = (uint32_t)((c - 'A') + 10);
    else
      d = 0;

    res = res + (d << (length*4));
  }

  return res;
}


/**********************************************************************//**
 * Print HEX byte.
 *
 * @param[in] byte Byte to be printed as 2-cahr hex value.
 **************************************************************************/
void aux_print_hex_byte(uint8_t byte) {

  static const char symbols[] = "0123456789abcdef";

  neorv32_uart0_putc(symbols[(byte >> 4) & 0x0f]);
  neorv32_uart0_putc(symbols[(byte >> 0) & 0x0f]);
}
