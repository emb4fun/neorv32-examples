-- ****************************************************************************
-- *  Copyright (c) 2021-2023 by Michael Fischer (www.emb4fun.de)
-- *  All rights reserved.
-- *
-- *  Redistribution and use in source and binary forms, with or without
-- *  modification, are permitted provided that the following conditions
-- *  are met:
-- *
-- *  1. Redistributions of source code must retain the above copyright
-- *     notice, this list of conditions and the following disclaimer.
-- *
-- *  2. Redistributions in binary form must reproduce the above copyright
-- *     notice, this list of conditions and the following disclaimer in the
-- *     documentation and/or other materials provided with the distribution.
-- *
-- *  3. Neither the name of the author nor the names of its contributors may
-- *     be used to endorse or promote products derived from this software
-- *     without specific prior written permission.
-- *
-- *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
-- *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
-- *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
-- *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
-- *  THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
-- *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
-- *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
-- *  OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
-- *  AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
-- *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
-- *  THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
-- *  SUCH DAMAGE.
-- ****************************************************************************

-- ****************************************************************************
-- *  DEFINE: Library                                                         *
-- ****************************************************************************

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.std_logic_unsigned.ALL;

library neorv32;
use neorv32.neorv32_package.all;


-- ****************************************************************************
-- *  DEFINE: Entity                                                          *
-- ****************************************************************************

entity top is
   port (
      --
      -- Input clock
      --
      CLOCK_50     : in  std_logic;

      --
      -- Reset
      --
      FPGA_RESET_N : in  std_logic;

      --
      -- JTAG TAP
      --
      nTRST_i      : in  std_logic;
      TCK_i        : in  std_logic;
      TDI_i        : in  std_logic;
      TDO_o        : out std_logic;
      TMS_i        : in  std_logic;

      --
      -- SDRAM interface,
      --
      SDRAM_CLK    : out   std_logic;                          -- Master Clock
      SDRAM_CKE    : out   std_logic;                          -- Clock Enable
      SDRAM_CS_N   : out   std_logic;                          -- Chip Select
      SDRAM_RAS_N  : out   std_logic;                          -- Row Address Strobe
      SDRAM_CAS_N  : out   std_logic;                          -- Column Address Strobe
      SDRAM_WE_N   : out   std_logic;                          -- Write Enable
      SDRAM_DQ     : inout std_logic_vector(15 downto 0);      -- Data I/O (16 bits)
      SDRAM_DQML   : out   std_logic;                          -- Output Disable / Write Mask (low)
      SDRAM_DQMU   : out   std_logic;                          -- Output Disable / Write Mask (high)
      SDRAM_ADDR   : out   std_logic_vector(12 downto 0);      -- Address Input (12 bits)
      SDRAM_BA_0   : out   std_logic;                          -- Bank Address 0
      SDRAM_BA_1   : out   std_logic;                          -- Bank Address 1

      --
      -- User LEDs
      --
      LED          : out std_logic_vector(9 downto 0);

      --
      -- UART
      --
      UART0_TXD    : out std_logic;
      UART0_RXD    : in  std_logic
   );
end entity top;


-- ****************************************************************************
-- *  DEFINE: Architecture                                                    *
-- ****************************************************************************

architecture syn of top is

   --------------------------------------------------------
   -- Define all constants here
   --------------------------------------------------------

   constant CLOCK_FREQUENCY   : natural := 100000000; -- clock frequency of clk_i in Hz
   constant MEM_INT_IMEM_SIZE : natural := 128*1024;  -- size of processor-internal instruction memory in bytes
   constant MEM_INT_DMEM_SIZE : natural := 64*1024;   -- size of processor-internal data memory in bytes


   --------------------------------------------------------
   -- Define all components which are included here
   --------------------------------------------------------

   --
   -- PLL
   --
   component pll_sys
      port (
         refclk   : in  std_logic := '0';
         rst      : in  std_logic := '0';
         outclk_0 : out std_logic;
         outclk_1 : out std_logic;
         locked   : out std_logic
      );
   end component pll_sys;


   --
   -- Wishbone Intercon
   --
   component wb_intercon is
      port (
         -- Syscon
         clk_i      : in  std_logic := '0';
         rst_i      : in  std_logic := '0';

         -- Wishbone Master
         wbm_stb_i  : in  std_logic := '0';
         wbm_cyc_i  : in  std_logic := '0';
         wbm_we_i   : in  std_logic := '0';
         wbm_ack_o  : out std_logic;
         wbm_adr_i  : in  std_logic_vector(31 downto 0) := (others => '0');
         wbm_dat_i  : in  std_logic_vector(31 downto 0) := (others => '0');
         wbm_dat_o  : out std_logic_vector(31 downto 0);
         wbm_sel_i  : in  std_logic_vector(03 downto 0) := (others => '0');

         -- Wishbone Slave x
         wbs_we_o   : out std_logic;
         wbs_dat_o  : out std_logic_vector(31 downto 0);
         wbs_sel_o  : out std_logic_vector(03 downto 0);

         -- Wishbone Slave 1
         wbs1_stb_o : out std_logic;
         wbs1_ack_i : in  std_logic := '0';
         wbs1_adr_o : out std_logic_vector(27 downto 0);
         wbs1_dat_i : in  std_logic_vector(31 downto 0) := (others => '0')

      );
   end component wb_intercon;


   --
   -- Wishbone to Avalon Master
   --
   component wb_av_master is
      port (
         -- System
         clk_i             : in  std_logic := '0';
         rst_i             : in  std_logic := '0';

         -- Wishbone
         wbs_stb_i         : in  std_logic                      := '0';
         wbs_we_i          : in  std_logic                      := '0';
         wbs_sel_i         : in  std_logic_vector(03 downto 0)  := (others => '0');
         wbs_adr_i         : in  std_logic_vector(27 downto 0)  := (others => '0');
         wbs_dat_i         : in  std_logic_vector(31 downto 0)  := (others => '0');
         wbs_dat_o         : out std_logic_vector(31 downto 0);
         wbs_ack_o         : out std_logic;

         -- Avalon Master
         avm_read_o        : out std_logic;
         avm_write_o       : out std_logic;
         avm_waitrequest_i : in  std_logic                        := '0';
         avm_byteenable_o  : out std_logic_vector(03 downto 0);
         avm_address_o     : out std_logic_vector(27 downto 0);
         avm_writedata_o   : out std_logic_vector(31 downto 0);
         avm_readdata_i    : in  std_logic_vector(31 downto 0)    := (others => '0')
      );
   end component wb_av_master;


   --
   -- Qsys
   --
   component qsys_core is
      port (
         clk_clk           : in  std_logic                     := 'X';               -- clk
         reset_reset_n     : in  std_logic                     := 'X';               -- reset_n

         sdram_addr        : out   std_logic_vector(12 downto 0);                    -- addr
         sdram_ba          : out   std_logic_vector(1 downto 0);                     -- ba
         sdram_cas_n       : out   std_logic;                                        -- cas_n
         sdram_cke         : out   std_logic;                                        -- cke
         sdram_cs_n        : out   std_logic;                                        -- cs_n
         sdram_dq          : inout std_logic_vector(15 downto 0) := (others => 'X'); -- dq
         sdram_dqm         : out   std_logic_vector(1 downto 0);                     -- dqm
         sdram_ras_n       : out   std_logic;                                        -- ras_n
         sdram_we_n        : out   std_logic;

         avm_address_i     : in  std_logic_vector(27 downto 0) := (others => 'X');   -- address_i
         avm_read_i        : in  std_logic                     := 'X';               -- read_i
         avm_waitrequest_o : out std_logic;                                          -- waitrequest_o
         avm_readdata_o    : out std_logic_vector(31 downto 0);                      -- readdata_o
         avm_write_i       : in  std_logic                     := 'X';               -- write_i
         avm_writedata_i   : in  std_logic_vector(31 downto 0) := (others => 'X');   -- writedata_i
         avm_byteenable_i  : in  std_logic_vector(3 downto 0)  := (others => 'X')    -- byteenable_i
      );
   end component qsys_core;


   --
   -- neorv32 top
   --
   component neorv32_top is
     generic (
       -- General --
       CLOCK_FREQUENCY              : natural;           -- clock frequency of clk_i in Hz
       HART_ID                      : std_ulogic_vector(31 downto 0) := x"00000000"; -- hardware thread ID
       VENDOR_ID                    : std_ulogic_vector(31 downto 0) := x"00000000"; -- vendor's JEDEC ID
       INT_BOOTLOADER_EN            : boolean := false;  -- boot configuration: true = boot explicit bootloader; false = boot from int/ext (I)MEM

       -- On-Chip Debugger (OCD) --
       ON_CHIP_DEBUGGER_EN          : boolean := false;  -- implement on-chip debugger
       DM_LEGACY_MODE               : boolean := false;                              -- debug module spec version: false = v1.0, true = v0.13

       -- RISC-V CPU Extensions --
       CPU_EXTENSION_RISCV_A        : boolean := false;                              -- implement atomic memory operations extension?
       CPU_EXTENSION_RISCV_B        : boolean := false;  -- implement bit-manipulation extension?
       CPU_EXTENSION_RISCV_C        : boolean := false;  -- implement compressed extension?
       CPU_EXTENSION_RISCV_E        : boolean := false;  -- implement embedded RF extension?
       CPU_EXTENSION_RISCV_M        : boolean := false;  -- implement mul/div extension?
       CPU_EXTENSION_RISCV_U        : boolean := false;  -- implement user mode extension?
       CPU_EXTENSION_RISCV_Zfinx    : boolean := false;  -- implement 32-bit floating-point extension (using INT regs!)
       CPU_EXTENSION_RISCV_Zicntr   : boolean := true;   -- implement base counters?
       CPU_EXTENSION_RISCV_Zihpm    : boolean := false;  -- implement hardware performance monitors?
       CPU_EXTENSION_RISCV_Zifencei : boolean := false;  -- implement instruction stream sync.?
       CPU_EXTENSION_RISCV_Zmmul    : boolean := false;  -- implement multiply-only M sub-extension?
       CPU_EXTENSION_RISCV_Zxcfu    : boolean := false;  -- implement custom (instr.) functions unit?

       -- Tuning Options --
       FAST_MUL_EN                  : boolean := false;  -- use DSPs for M extension's multiplier
       FAST_SHIFT_EN                : boolean := false;  -- use barrel shifter for shift operations

       -- Physical Memory Protection (PMP) --
       PMP_NUM_REGIONS              : natural range 0 to 16 := 0;                    -- number of regions (0..16)
       PMP_MIN_GRANULARITY          : natural := 4;      -- minimal region granularity in bytes, has to be a power of 2, min 4 bytes

       -- Hardware Performance Monitors (HPM) --
       HPM_NUM_CNTS                 : natural range 0 to 13 := 0;                    -- number of implemented HPM counters (0..13)
       HPM_CNT_WIDTH                : natural range 0 to 64 := 40;                   -- total size of HPM counters (0..64)

       -- Atomic Memory Access - Reservation Set Granularity --
       AMO_RVS_GRANULARITY          : natural := 4;                                  -- size in bytes, has to be a power of 2, min 4

       -- Internal Instruction memory (IMEM) --
       MEM_INT_IMEM_EN              : boolean := false;  -- implement processor-internal instruction memory
       MEM_INT_IMEM_SIZE            : natural := 16*1024;                            -- size of processor-internal instruction memory in bytes (use a power of 2)

       -- Internal Data memory (DMEM) --
       MEM_INT_DMEM_EN              : boolean := false;  -- implement processor-internal data memory
       MEM_INT_DMEM_SIZE            : natural := 8*1024;                             -- size of processor-internal data memory in bytes (use a power of 2)

       -- Internal Instruction Cache (iCACHE) --
       ICACHE_EN                    : boolean := false;  -- implement instruction cache
       ICACHE_NUM_BLOCKS            : natural range 1 to 256   := 4;                 -- i-cache: number of blocks (min 1), has to be a power of 2
       ICACHE_BLOCK_SIZE            : natural range 4 to 2**16 := 64;                -- i-cache: block size in bytes (min 4), has to be a power of 2
       ICACHE_ASSOCIATIVITY         : natural range 1 to 2     := 1;                 -- i-cache: associativity / number of sets (1=direct_mapped), has to be a power of 2

       -- Internal Data Cache (dCACHE) --
       DCACHE_EN                    : boolean := false;  -- implement data cache
       DCACHE_NUM_BLOCKS            : natural range 1 to 256   := 4;                 -- d-cache: number of blocks (min 1), has to be a power of 2
       DCACHE_BLOCK_SIZE            : natural range 4 to 2**16 := 64;                -- d-cache: block size in bytes (min 4), has to be a power of 2

       -- External memory interface (WISHBONE) --
       MEM_EXT_EN                   : boolean := false;  -- implement external memory bus interface?
       MEM_EXT_TIMEOUT              : natural := 255;    -- cycles after a pending bus access auto-terminates (0 = disabled)
       MEM_EXT_PIPE_MODE            : boolean := false;  -- protocol: false=classic/standard wishbone mode, true=pipelined wishbone mode
       MEM_EXT_BIG_ENDIAN           : boolean := false;  -- byte order: true=big-endian, false=little-endian
       MEM_EXT_ASYNC_RX             : boolean := false;  -- use register buffer for RX data when false
       MEM_EXT_ASYNC_TX             : boolean := false;  -- use register buffer for TX data when false

       -- External Interrupts Controller (XIRQ) --
       XIRQ_NUM_CH                  : natural range 0 to 32          := 0;           -- number of external IRQ channels (0..32)
       XIRQ_TRIGGER_TYPE            : std_ulogic_vector(31 downto 0) := x"ffffffff"; -- trigger type: 0=level, 1=edge
       XIRQ_TRIGGER_POLARITY        : std_ulogic_vector(31 downto 0) := x"ffffffff"; -- trigger polarity: 0=low-level/falling-edge, 1=high-level/rising-edge

       -- Processor peripherals --
       IO_GPIO_NUM                  : natural range 0 to 64          := 0;           -- number of GPIO input/output pairs (0..64)
       IO_MTIME_EN                  : boolean := false;  -- implement machine system timer (MTIME)?
       IO_UART0_EN                  : boolean := false;  -- implement primary universal asynchronous receiver/transmitter (UART0)?
       IO_UART0_RX_FIFO             : natural range 1 to 2**15       := 1;           -- RX fifo depth, has to be a power of two, min 1
       IO_UART0_TX_FIFO             : natural range 1 to 2**15       := 1;           -- TX fifo depth, has to be a power of two, min 1
       IO_UART1_EN                  : boolean := false;  -- implement secondary universal asynchronous receiver/transmitter (UART1)?
       IO_UART1_RX_FIFO             : natural range 1 to 2**15       := 1;           -- RX fifo depth, has to be a power of two, min 1
       IO_UART1_TX_FIFO             : natural range 1 to 2**15       := 1;           -- TX fifo depth, has to be a power of two, min 1
       IO_SPI_EN                    : boolean := false;  -- implement serial peripheral interface (SPI)?
       IO_SPI_FIFO                  : natural range 1 to 2**15       := 1;           -- RTX fifo depth, has to be a power of two, min 1
       IO_SDI_EN                    : boolean := false;  -- implement serial data interface (SDI)?
       IO_SDI_FIFO                  : natural range 1 to 2**15       := 1;           -- RTX fifo depth, has to be zero or a power of two, min 1
       IO_TWI_EN                    : boolean := false;  -- implement two-wire interface (TWI)?
       IO_PWM_NUM_CH                : natural range 0 to 12          := 0;           -- number of PWM channels to implement (0..12); 0 = disabled
       IO_WDT_EN                    : boolean := false;  -- implement watch dog timer (WDT)?
       IO_TRNG_EN                   : boolean := false;  -- implement true random number generator (TRNG)?
       IO_TRNG_FIFO                 : natural range 1 to 2**15       := 1;           -- data fifo depth, has to be a power of two, min 1
       IO_CFS_EN                    : boolean := false;  -- implement custom functions subsystem (CFS)?
       IO_CFS_CONFIG                : std_ulogic_vector(31 downto 0) := x"00000000"; -- custom CFS configuration generic
       IO_CFS_IN_SIZE               : natural := 32;     -- size of CFS input conduit in bits
       IO_CFS_OUT_SIZE              : natural := 32;     -- size of CFS output conduit in bits
       IO_NEOLED_EN                 : boolean := false;  -- implement NeoPixel-compatible smart LED interface (NEOLED)?
       IO_NEOLED_TX_FIFO            : natural range 1 to 2**15       := 1;           -- NEOLED FIFO depth, has to be a power of two, min 1
       IO_GPTMR_EN                  : boolean := false;  -- implement general purpose timer (GPTMR)?
       IO_XIP_EN                    : boolean := false;  -- implement execute in place module (XIP)?
       IO_ONEWIRE_EN                : boolean                        := false;       -- implement 1-wire interface (ONEWIRE)?
       IO_DMA_EN                    : boolean                        := false;       -- implement direct memory access controller (DMA)?
       IO_SLINK_EN                  : boolean                        := false;       -- implement stream link interface (SLINK)?
       IO_SLINK_RX_FIFO             : natural range 1 to 2**15       := 1;           -- RX fifo depth, has to be a power of two, min 1
       IO_SLINK_TX_FIFO             : natural range 1 to 2**15       := 1;           -- TX fifo depth, has to be a power of two, min 1
       IO_CRC_EN                    : boolean                        := false        -- implement cyclic redundancy check unit (CRC)?
     );
     port (
       -- Global control --
       clk_i          : in  std_ulogic; -- global clock, rising edge
       rstn_i         : in  std_ulogic; -- global reset, low-active, async

       -- JTAG on-chip debugger interface (available if ON_CHIP_DEBUGGER_EN = true) --
       jtag_trst_i    : in  std_ulogic := 'U'; -- low-active TAP reset (optional)
       jtag_tck_i     : in  std_ulogic := 'U'; -- serial clock
       jtag_tdi_i     : in  std_ulogic := 'U'; -- serial data input
       jtag_tdo_o     : out std_ulogic;        -- serial data output
       jtag_tms_i     : in  std_ulogic := 'U'; -- mode select

       -- Wishbone bus interface (available if MEM_EXT_EN = true) --
       wb_tag_o       : out std_ulogic_vector(02 downto 0); -- request tag
       wb_adr_o       : out std_ulogic_vector(31 downto 0); -- address
       wb_dat_i       : in  std_ulogic_vector(31 downto 0) := (others => 'U'); -- read data
       wb_dat_o       : out std_ulogic_vector(31 downto 0); -- write data
       wb_we_o        : out std_ulogic; -- read/write
       wb_sel_o       : out std_ulogic_vector(03 downto 0); -- byte enable
       wb_stb_o       : out std_ulogic; -- strobe
       wb_cyc_o       : out std_ulogic; -- valid cycle
       wb_ack_i       : in  std_ulogic := 'L'; -- transfer acknowledge
       wb_err_i       : in  std_ulogic := 'L'; -- transfer error

       -- Stream Link Interface (available if IO_SLINK_EN = true) --
       slink_rx_dat_i : in  std_ulogic_vector(31 downto 0) := (others => 'U'); -- RX input data
       slink_rx_val_i : in  std_ulogic := 'L'; -- RX valid input
       slink_rx_rdy_o : out std_ulogic; -- RX ready to receive
       slink_tx_dat_o : out std_ulogic_vector(31 downto 0); -- TX output data
       slink_tx_val_o : out std_ulogic; -- TX valid output
       slink_tx_rdy_i : in  std_ulogic := 'L';  -- TX ready to send

       -- Advanced memory control signals --
       fence_o        : out std_ulogic; -- indicates an executed FENCE operation
       fencei_o       : out std_ulogic; -- indicates an executed FENCEI operation

       -- XIP (execute in place via SPI) signals (available if IO_XIP_EN = true) --
       xip_csn_o      : out std_ulogic; -- chip-select, low-active
       xip_clk_o      : out std_ulogic; -- serial clock
       xip_dat_i      : in  std_ulogic := 'L'; -- device data input
       xip_dat_o      : out std_ulogic; -- controller data output

       -- GPIO (available if IO_GPIO_NUM > 0) --
       gpio_o         : out std_ulogic_vector(63 downto 0); -- parallel output
       gpio_i         : in  std_ulogic_vector(63 downto 0) := (others => 'U'); -- parallel input

       -- primary UART0 (available if IO_UART0_EN = true) --
       uart0_txd_o    : out std_ulogic; -- UART0 send data
       uart0_rxd_i    : in  std_ulogic := 'U'; -- UART0 receive data
       uart0_rts_o    : out std_ulogic; -- HW flow control: UART0.RX ready to receive ("RTR"), low-active, optional
       uart0_cts_i    : in  std_ulogic := 'L'; -- HW flow control: UART0.TX allowed to transmit, low-active, optional

       -- secondary UART1 (available if IO_UART1_EN = true) --
       uart1_txd_o    : out std_ulogic; -- UART1 send data
       uart1_rxd_i    : in  std_ulogic := 'U'; -- UART1 receive data
       uart1_rts_o    : out std_ulogic; -- HW flow control: UART1.RX ready to receive ("RTR"), low-active, optional
       uart1_cts_i    : in  std_ulogic := 'L'; -- HW flow control: UART1.TX allowed to transmit, low-active, optional

       -- SPI (available if IO_SPI_EN = true) --
       spi_clk_o      : out std_ulogic; -- SPI serial clock
       spi_dat_o      : out std_ulogic; -- controller data out, peripheral data in
       spi_dat_i      : in  std_ulogic := 'U'; -- controller data in, peripheral data out
       spi_csn_o      : out std_ulogic_vector(07 downto 0); -- chip-select

       -- SDI (available if IO_SDI_EN = true) --
       sdi_clk_i      : in  std_ulogic := 'U'; -- SDI serial clock
       sdi_dat_o      : out std_ulogic; -- controller data out, peripheral data in
       sdi_dat_i      : in  std_ulogic := 'U'; -- controller data in, peripheral data out
       sdi_csn_i      : in  std_ulogic := 'H'; -- chip-select

       -- TWI (available if IO_TWI_EN = true) --
       twi_sda_i      : in  std_ulogic := 'H'; -- serial data line sense input
       twi_sda_o      : out std_ulogic; -- serial data line output (pull low only)
       twi_scl_i      : in  std_ulogic := 'H'; -- serial clock line sense input
       twi_scl_o      : out std_ulogic; -- serial clock line output (pull low only)

       -- 1-Wire Interface (available if IO_ONEWIRE_EN = true) --
       onewire_i      : in  std_ulogic := 'H'; -- 1-wire bus sense input
       onewire_o      : out std_ulogic; -- 1-wire bus output (pull low only)

       -- PWM (available if IO_PWM_NUM_CH > 0) --
       pwm_o          : out std_ulogic_vector(11 downto 0); -- pwm channels

       -- Custom Functions Subsystem IO (available if IO_CFS_EN = true) --
       cfs_in_i       : in  std_ulogic_vector(IO_CFS_IN_SIZE-1 downto 0) := (others => 'U'); -- custom CFS inputs conduit
       cfs_out_o      : out std_ulogic_vector(IO_CFS_OUT_SIZE-1 downto 0); -- custom CFS outputs conduit

       -- NeoPixel-compatible smart LED interface (available if IO_NEOLED_EN = true) --
       neoled_o       : out std_ulogic; -- async serial data line

       -- External platform interrupts (available if XIRQ_NUM_CH > 0) --
       xirq_i         : in  std_ulogic_vector(31 downto 0) := (others => 'L'); -- IRQ channels

       -- CPU interrupts --
       mtime_irq_i    : in  std_ulogic := 'L'; -- machine timer interrupt, available if IO_MTIME_EN = false
       msw_irq_i      : in  std_ulogic := 'L'; -- machine software interrupt
       mext_irq_i     : in  std_ulogic := 'L'  -- machine external interrupt
     );
   end component neorv32_top;


   --------------------------------------------------------
   -- Define all local signals here
   --------------------------------------------------------

   signal sys_clk          : std_logic := '0';
   signal pll_locked       : std_logic := '0';
   signal reset            : std_logic := '0';
   signal reset_s1         : std_logic := '1';
   signal reset_s2         : std_logic := '1';
   signal reset_s3         : std_logic := '1';
   signal sys_rst          : std_logic;
   signal fpga_reset       : std_logic;

   signal clk_i            : std_logic;
   signal rstn_i           : std_logic;

   -- SDRAM
   signal sdram_ba         : std_logic_vector(1 downto 0);
   signal sdram_dqm        : std_logic_vector(1 downto 0);

   signal gpio             : std_ulogic_vector(63 downto 0);

   -- Wishbone bus interface (available if MEM_EXT_EN = true) --
   signal wb_adr           : std_ulogic_vector(31 downto 0);  -- address
   signal wb_dat_read_u    : std_ulogic_vector(31 downto 0);  -- read data
   signal wb_dat_write     : std_ulogic_vector(31 downto 0);  -- write data
   signal wb_we            : std_ulogic;                      -- read/write
   signal wb_sel           : std_ulogic_vector(03 downto 0);  -- byte enable
   signal wb_stb           : std_ulogic;                      -- strobe
   signal wb_cyc           : std_ulogic;
   signal wb_ack           : std_ulogic;                      -- transfer acknowledge

   signal wb_sel_int       : std_logic_vector(03 downto 0);   -- byte enable
   signal wb_adr_int       : std_logic_vector(31 downto 0);   -- address
   signal wb_dat_read      : std_logic_vector(31 downto 0);
   signal wb_dat_write_int : std_logic_vector(31 downto 0);   -- write data

   -- Wishbone slave
   signal wbs_we_i         : std_logic;
   signal wbs_dat_i        : std_logic_vector(31 downto 0);
   signal wbs_sel_i        : std_logic_vector(03 downto 0);
   signal wbs1_stb_i       : std_logic;
   signal wbs1_ack_o       : std_logic;
   signal wbs1_adr_i       : std_logic_vector(27 downto 0);
   signal wbs1_dat_o       : std_logic_vector(31 downto 0);

   -- AvalonMM interface
   signal read             : std_logic;
   signal write            : std_logic;
   signal waitrequest      : std_logic;
   signal byteenable       : std_logic_vector(03 downto 0);
   signal address          : std_logic_vector(27 downto 0);
   signal writedata        : std_logic_vector(31 downto 0);
   signal readdata         : std_logic_vector(31 downto 0);


begin

   --
   -- PLL
   --
   inst_pll_sys : pll_sys
      port map (
         refclk   => CLOCK_50,
         rst      => not FPGA_RESET_N,
         outclk_0 => sys_clk,
         outclk_1 => SDRAM_CLK,
         locked   => pll_locked
      );

   --
   -- In general it is a bad idea to use an asynchhronous Reset signal.
   -- But it is only a bad idea in case of asynchhronous deasserting.
   -- Therefore the deasserting of the Reset signal must be synchronized.
   --

   -- Asynchronous assert
   reset <= '1' when ((FPGA_RESET_N = '0') OR (pll_locked = '0')) else '0';

   -- Synchronize deassert
   process (sys_clk, reset)
   begin
      if (reset = '1') then
         reset_s1 <= '1';
         reset_s2 <= '1';
         reset_s3 <= '1';
      elsif rising_edge(sys_clk) then
         reset_s1 <= '0';
         reset_s2 <= reset_s1;
         reset_s3 <= reset_s2;
      end if;
   end process;

   -- The deassert edge is now synchronized
   sys_rst <= reset_s3;

   clk_i  <= sys_clk;
   rstn_i <= not sys_rst;


   --
   -- neorv32
   --
   neorv32_top_inst: neorv32_top
      generic map (
         -- General --
         CLOCK_FREQUENCY              => CLOCK_FREQUENCY,   -- clock frequency of clk_i in Hz
         INT_BOOTLOADER_EN            => true,              -- boot configuration: true = boot explicit bootloader; false = boot from int/ext (I)MEM

         -- On-Chip Debugger (OCD) --
         ON_CHIP_DEBUGGER_EN          => true,              -- implement on-chip debugger

         -- RISC-V CPU Extensions --
         CPU_EXTENSION_RISCV_C        => true,              -- implement compressed extension?
         CPU_EXTENSION_RISCV_M        => true,              -- implement mul/div extension?
         CPU_EXTENSION_RISCV_Zicntr   => true,              -- implement base counters?
         CPU_EXTENSION_RISCV_Zifencei => true,              -- implement instruction stream sync.?

         -- Internal Instruction memory --
         MEM_INT_IMEM_EN              => true,              -- implement processor-internal instruction memory
         MEM_INT_IMEM_SIZE            => MEM_INT_IMEM_SIZE, -- size of processor-internal instruction memory in bytes

         -- Internal Data memory --
         MEM_INT_DMEM_EN              => true,              -- implement processor-internal data memory
         MEM_INT_DMEM_SIZE            => MEM_INT_DMEM_SIZE, -- size of processor-internal data memory in bytes

         -- External memory interface --
         MEM_EXT_EN                   => true,              -- implement external memory bus interface?
         MEM_EXT_TIMEOUT              => 255,               -- cycles after a pending bus access auto-terminates (0 = disabled)
         MEM_EXT_PIPE_MODE            => false,             -- protocol: false=classic/standard wishbone mode, true=pipelined wishbone mode
         MEM_EXT_BIG_ENDIAN           => false,             -- byte order: true=big-endian, false=little-endian
         MEM_EXT_ASYNC_RX             => false,             -- use register buffer for RX data when false

         -- Processor peripherals --
         IO_GPIO_NUM                  => 8,                 -- number of GPIO input/output pairs (0..64)
         IO_MTIME_EN                  => true,              -- implement machine system timer (MTIME)?
         IO_UART0_EN                  => true               -- implement primary universal asynchronous receiver/transmitter (UART0)?
      )
      port map (
         -- Global control --
         clk_i         => clk_i,                            -- global clock, rising edge
         rstn_i        => rstn_i,                           -- global reset, low-active, async

         -- JTAG on-chip debugger interface (available if ON_CHIP_DEBUGGER_EN = true) --
         jtag_trst_i   => nTRST_i,                          -- low-active TAP reset (optional)
         jtag_tck_i    => TCK_i,                            -- serial clock
         jtag_tdi_i    => TDI_i,                            -- serial data input
         jtag_tdo_o    => TDO_o,                            -- serial data output
         jtag_tms_i    => TMS_i,                            -- mode select

         -- Wishbone bus interface (available if MEM_EXT_EN = true) --
         wb_tag_o      => open,                             -- tag
         wb_adr_o      => wb_adr,                           -- address
         wb_dat_i      => wb_dat_read_u,                    -- read data
         wb_dat_o      => wb_dat_write,                     -- write data
         wb_we_o       => wb_we,                            -- read/write
         wb_sel_o      => wb_sel,                           -- byte enable
         wb_stb_o      => wb_stb,                           -- strobe
         wb_cyc_o      => wb_cyc,                           -- valid cycle
         wb_ack_i      => wb_ack,                           -- transfer acknowledge
         wb_err_i      => '0',                              -- transfer error

         -- GPIO (available if IO_GPIO_EN = true) --
         gpio_o        => gpio,                             -- parallel output
         gpio_i        => open,                             -- parallel input

         -- primary UART0 (available if IO_UART0_EN = true) --
         uart0_txd_o   => UART0_TXD,                        -- UART0 send data
         uart0_rxd_i   => UART0_RXD                         -- UART0 receive data
      );

   wb_sel_int       <= To_StdLogicVector( wb_sel );
   wb_adr_int       <= To_StdLogicVector( wb_adr );
   wb_dat_read_u    <= std_ulogic_vector( wb_dat_read );
   wb_dat_write_int <= To_StdLogicVector( wb_dat_write );


   --
   -- Wishbone Intercon
   --
   inst_wb_intercon : wb_intercon
      port map (
         -- Syscon
         clk_i      => sys_clk,
         rst_i      => sys_rst,

         -- Wishbone Master
         wbm_stb_i  => wb_stb,
         wbm_cyc_i  => wb_cyc,
         wbm_we_i   => wb_we,
         wbm_ack_o  => wb_ack,
         wbm_adr_i  => wb_adr_int,
         wbm_dat_i  => wb_dat_write_int,
         wbm_dat_o  => wb_dat_read,
         wbm_sel_i  => wb_sel_int,

         -- Wishbone Slave x
         wbs_we_o   => wbs_we_i,
         wbs_dat_o  => wbs_dat_i,
         wbs_sel_o  => wbs_sel_i,

         -- Wishbone Slave 1
         wbs1_stb_o => wbs1_stb_i,
         wbs1_ack_i => wbs1_ack_o,
         wbs1_adr_o => wbs1_adr_i,
         wbs1_dat_i => wbs1_dat_o
      );


   --
   -- Wishbone to Avalon Master
   --
   inst_wb_av_master : wb_av_master
      port map (
         -- System
         clk_i             => sys_clk,
         rst_i             => sys_rst,

         -- Wishbone
         wbs_stb_i         => wbs1_stb_i,
         wbs_we_i          => wbs_we_i,
         wbs_sel_i         => wbs_sel_i,
         wbs_adr_i         => wbs1_adr_i,
         wbs_dat_i         => wbs_dat_i,
         wbs_dat_o         => wbs1_dat_o,
         wbs_ack_o         => wbs1_ack_o,

         -- Avalon Master
         avm_read_o        => read,
         avm_write_o       => write,
         avm_waitrequest_i => waitrequest,
         avm_byteenable_o  => byteenable,
         avm_address_o     => address,
         avm_writedata_o   => writedata,
         avm_readdata_i    => readdata
      );


   --
   -- qsys
   --
   inst_qsys : qsys_core
      port map (
         clk_clk           => sys_clk,
         reset_reset_n     => not sys_rst,

         -- SDRAM
         sdram_addr        => SDRAM_ADDR,
         sdram_ba          => sdram_ba,
         sdram_cas_n       => SDRAM_CAS_N,
         sdram_cke         => SDRAM_CKE,
         sdram_cs_n        => SDRAM_CS_N,
         sdram_dq          => SDRAM_DQ,
         sdram_dqm         => sdram_dqm,
         sdram_ras_n       => SDRAM_RAS_N,
         sdram_we_n        => SDRAM_WE_N,

         avm_address_i     => address,
         avm_read_i        => read,
         avm_waitrequest_o => waitrequest,
         avm_readdata_o    => readdata,
         avm_write_i       => write,
         avm_writedata_i   => writedata,
         avm_byteenable_i  => byteenable
      );

   SDRAM_BA_1 <= sdram_ba(1);
   SDRAM_BA_0 <= sdram_ba(0);

   SDRAM_DQMU <= sdram_dqm(1);
   SDRAM_DQML <= sdram_dqm(0);


   --------------------------------------------------------
   -- Output
   --------------------------------------------------------

   LED <= To_StdLogicVector( gpio(9 downto 0) );

end architecture syn;

-- *** EOF ***

