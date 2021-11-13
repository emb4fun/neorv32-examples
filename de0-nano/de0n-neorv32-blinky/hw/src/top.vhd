-- ****************************************************************************
-- *  Copyright (c) 2021 by Michael Fischer (www.emb4fun.de) 
-- *  All rights reserved.
-- *
-- *  Redistribution and use in source and binary forms, with or without 
-- *  modification, are permitted provided that the following conditions 
-- *  are met:
-- *  
-- *  1. Redistributions of source code must retain the above copyright 
-- *     notice, this list of conditions and the following disclaimer.
-- *  2. Redistributions in binary form must reproduce the above copyright
-- *     notice, this list of conditions and the following disclaimer in the 
-- *     documentation and/or other materials provided with the distribution.
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
-- *
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
      CLOCK_50  : in  std_logic;

      --
      -- JTAG
      --
      nTRST_i   : in  std_logic;
      TCK_i     : in  std_logic;
      TDI_i     : in  std_logic; 
      TDO_o     : out std_logic;
      TMS_i     : in  std_logic;

      --
      -- LEDs
      --
      LED       : out std_logic_vector(7 downto 0);
      
      --
      -- Keys
      --
      KEY       : in  std_logic_vector(1 downto 0);
      
      --
      -- UART       
      --
      UART0_TXD : out std_logic;
      UART0_RXD : in  std_logic
   );
end entity top;


-- ****************************************************************************
-- *  DEFINE: Architecture                                                    *
-- ****************************************************************************

architecture syn of top is

   --------------------------------------------------------
   -- Define all constants here
   --------------------------------------------------------

   constant CLOCK_FREQUENCY   : natural := 96000000;  -- clock frequency of clk_i in Hz
   constant MEM_INT_IMEM_SIZE : natural := 32*1024;   -- size of processor-internal instruction memory in bytes
   constant MEM_INT_DMEM_SIZE : natural := 16*1024;   -- size of processor-internal data memory in bytes

   
   --------------------------------------------------------
   -- Define all components which are included here
   --------------------------------------------------------

   --
   -- PLL
   --   
   component pll_sys
      port ( 
         inclk0 : in  std_logic := '0';
         c0     : out std_logic;
         locked : out std_logic 
      );
   end component pll_sys;

   
   --
   -- neorv32 top
   --
   component neorv32_top is
     generic (
       -- General --
       CLOCK_FREQUENCY              : natural;           -- clock frequency of clk_i in Hz
       HW_THREAD_ID                 : natural := 0;      -- hardware thread id (32-bit)
       INT_BOOTLOADER_EN            : boolean := false;  -- boot configuration: true = boot explicit bootloader; false = boot from int/ext (I)MEM

       -- On-Chip Debugger (OCD) --
       ON_CHIP_DEBUGGER_EN          : boolean := false;  -- implement on-chip debugger

       -- RISC-V CPU Extensions --
       CPU_EXTENSION_RISCV_A        : boolean := false;  -- implement atomic extension?
       CPU_EXTENSION_RISCV_B        : boolean := false;  -- implement bit-manipulation extension?
       CPU_EXTENSION_RISCV_C        : boolean := false;  -- implement compressed extension?
       CPU_EXTENSION_RISCV_E        : boolean := false;  -- implement embedded RF extension?
       CPU_EXTENSION_RISCV_M        : boolean := false;  -- implement mul/div extension?
       CPU_EXTENSION_RISCV_U        : boolean := false;  -- implement user mode extension?
       CPU_EXTENSION_RISCV_Zfinx    : boolean := false;  -- implement 32-bit floating-point extension (using INT regs!)
       CPU_EXTENSION_RISCV_Zicsr    : boolean := true;   -- implement CSR system?
       CPU_EXTENSION_RISCV_Zifencei : boolean := false;  -- implement instruction stream sync.?
       CPU_EXTENSION_RISCV_Zmmul    : boolean := false;  -- implement multiply-only M sub-extension?

       -- Extension Options --
       FAST_MUL_EN                  : boolean := false;  -- use DSPs for M extension's multiplier
       FAST_SHIFT_EN                : boolean := false;  -- use barrel shifter for shift operations
       CPU_CNT_WIDTH                : natural := 64;     -- total width of CPU cycle and instret counters (0..64)
       CPU_IPB_ENTRIES              : natural := 2;      -- entries is instruction prefetch buffer, has to be a power of 2

       -- Physical Memory Protection (PMP) --
       PMP_NUM_REGIONS              : natural := 0;      -- number of regions (0..64)
       PMP_MIN_GRANULARITY          : natural := 64*1024; -- minimal region granularity in bytes, has to be a power of 2, min 8 bytes

       -- Hardware Performance Monitors (HPM) --
       HPM_NUM_CNTS                 : natural := 0;      -- number of implemented HPM counters (0..29)
       HPM_CNT_WIDTH                : natural := 40;     -- total size of HPM counters (0..64)

       -- Internal Instruction memory (IMEM) --
       MEM_INT_IMEM_EN              : boolean := false;  -- implement processor-internal instruction memory
       MEM_INT_IMEM_SIZE            : natural := 16*1024; -- size of processor-internal instruction memory in bytes

       -- Internal Data memory (DMEM) --
       MEM_INT_DMEM_EN              : boolean := false;  -- implement processor-internal data memory
       MEM_INT_DMEM_SIZE            : natural := 8*1024; -- size of processor-internal data memory in bytes

       -- Internal Cache memory (iCACHE) --
       ICACHE_EN                    : boolean := false;  -- implement instruction cache
       ICACHE_NUM_BLOCKS            : natural := 4;      -- i-cache: number of blocks (min 1), has to be a power of 2
       ICACHE_BLOCK_SIZE            : natural := 64;     -- i-cache: block size in bytes (min 4), has to be a power of 2
       ICACHE_ASSOCIATIVITY         : natural := 1;      -- i-cache: associativity / number of sets (1=direct_mapped), has to be a power of 2

       -- External memory interface (WISHBONE) --
       MEM_EXT_EN                   : boolean := false;  -- implement external memory bus interface?
       MEM_EXT_TIMEOUT              : natural := 255;    -- cycles after a pending bus access auto-terminates (0 = disabled)
       MEM_EXT_PIPE_MODE            : boolean := false;  -- protocol: false=classic/standard wishbone mode, true=pipelined wishbone mode
       MEM_EXT_BIG_ENDIAN           : boolean := false;  -- byte order: true=big-endian, false=little-endian
       MEM_EXT_ASYNC_RX             : boolean := false;  -- use register buffer for RX data when false

       -- Stream link interface (SLINK) --
       SLINK_NUM_TX                 : natural := 0;      -- number of TX links (0..8)
       SLINK_NUM_RX                 : natural := 0;      -- number of TX links (0..8)
       SLINK_TX_FIFO                : natural := 1;      -- TX fifo depth, has to be a power of two
       SLINK_RX_FIFO                : natural := 1;      -- RX fifo depth, has to be a power of two

       -- External Interrupts Controller (XIRQ) --
       XIRQ_NUM_CH                  : natural := 0;      -- number of external IRQ channels (0..32)
       XIRQ_TRIGGER_TYPE            : std_ulogic_vector(31 downto 0) := x"ffffffff"; -- trigger type: 0=level, 1=edge
       XIRQ_TRIGGER_POLARITY        : std_ulogic_vector(31 downto 0) := x"ffffffff"; -- trigger polarity: 0=low-level/falling-edge, 1=high-level/rising-edge

       -- Processor peripherals --
       IO_GPIO_EN                   : boolean := false;  -- implement general purpose input/output port unit (GPIO)?
       IO_MTIME_EN                  : boolean := false;  -- implement machine system timer (MTIME)?
       IO_UART0_EN                  : boolean := false;  -- implement primary universal asynchronous receiver/transmitter (UART0)?
       IO_UART0_RX_FIFO             : natural := 1;      -- RX fifo depth, has to be a power of two, min 1
       IO_UART0_TX_FIFO             : natural := 1;      -- TX fifo depth, has to be a power of two, min 1
       IO_UART1_EN                  : boolean := false;  -- implement secondary universal asynchronous receiver/transmitter (UART1)?
       IO_UART1_RX_FIFO             : natural := 1;      -- RX fifo depth, has to be a power of two, min 1
       IO_UART1_TX_FIFO             : natural := 1;      -- TX fifo depth, has to be a power of two, min 1
       IO_SPI_EN                    : boolean := false;  -- implement serial peripheral interface (SPI)?
       IO_TWI_EN                    : boolean := false;  -- implement two-wire interface (TWI)?
       IO_PWM_NUM_CH                : natural := 0;      -- number of PWM channels to implement (0..60); 0 = disabled
       IO_WDT_EN                    : boolean := false;  -- implement watch dog timer (WDT)?
       IO_TRNG_EN                   : boolean := false;  -- implement true random number generator (TRNG)?
       IO_CFS_EN                    : boolean := false;  -- implement custom functions subsystem (CFS)?
       IO_CFS_CONFIG                : std_ulogic_vector(31 downto 0) := x"00000000"; -- custom CFS configuration generic
       IO_CFS_IN_SIZE               : positive := 32;    -- size of CFS input conduit in bits
       IO_CFS_OUT_SIZE              : positive := 32;    -- size of CFS output conduit in bits
       IO_NEOLED_EN                 : boolean := false;  -- implement NeoPixel-compatible smart LED interface (NEOLED)?
       IO_NEOLED_TX_FIFO            : natural := 1       -- NEOLED TX FIFO depth, 1..32k, has to be a power of two
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
       wb_lock_o      : out std_ulogic; -- exclusive access request
       wb_ack_i       : in  std_ulogic := 'L'; -- transfer acknowledge
       wb_err_i       : in  std_ulogic := 'L'; -- transfer error

       -- Advanced memory control signals (available if MEM_EXT_EN = true) --
       fence_o        : out std_ulogic; -- indicates an executed FENCE operation
       fencei_o       : out std_ulogic; -- indicates an executed FENCEI operation

       -- TX stream interfaces (available if SLINK_NUM_TX > 0) --
       slink_tx_dat_o : out sdata_8x32_t; -- output data
       slink_tx_val_o : out std_ulogic_vector(7 downto 0); -- valid output
       slink_tx_rdy_i : in  std_ulogic_vector(7 downto 0) := (others => 'L'); -- ready to send

       -- RX stream interfaces (available if SLINK_NUM_RX > 0) --
       slink_rx_dat_i : in  sdata_8x32_t := (others => (others => 'U')); -- input data
       slink_rx_val_i : in  std_ulogic_vector(7 downto 0) := (others => 'L'); -- valid input
       slink_rx_rdy_o : out std_ulogic_vector(7 downto 0); -- ready to receive

       -- GPIO (available if IO_GPIO_EN = true) --
       gpio_o         : out std_ulogic_vector(63 downto 0); -- parallel output
       gpio_i         : in  std_ulogic_vector(63 downto 0) := (others => 'U'); -- parallel input

       -- primary UART0 (available if IO_UART0_EN = true) --
       uart0_txd_o    : out std_ulogic; -- UART0 send data
       uart0_rxd_i    : in  std_ulogic := 'U'; -- UART0 receive data
       uart0_rts_o    : out std_ulogic; -- hw flow control: UART0.RX ready to receive ("RTR"), low-active, optional
       uart0_cts_i    : in  std_ulogic := 'L'; -- hw flow control: UART0.TX allowed to transmit, low-active, optional

       -- secondary UART1 (available if IO_UART1_EN = true) --
       uart1_txd_o    : out std_ulogic; -- UART1 send data
       uart1_rxd_i    : in  std_ulogic := 'U'; -- UART1 receive data
       uart1_rts_o    : out std_ulogic; -- hw flow control: UART1.RX ready to receive ("RTR"), low-active, optional
       uart1_cts_i    : in  std_ulogic := 'L'; -- hw flow control: UART1.TX allowed to transmit, low-active, optional

       -- SPI (available if IO_SPI_EN = true) --
       spi_sck_o      : out std_ulogic; -- SPI serial clock
       spi_sdo_o      : out std_ulogic; -- controller data out, peripheral data in
       spi_sdi_i      : in  std_ulogic := 'U'; -- controller data in, peripheral data out
       spi_csn_o      : out std_ulogic_vector(07 downto 0); -- chip-select

       -- TWI (available if IO_TWI_EN = true) --
       twi_sda_io     : inout std_logic := 'U'; -- twi serial data line
       twi_scl_io     : inout std_logic := 'U'; -- twi serial clock line

       -- PWM (available if IO_PWM_NUM_CH > 0) --
       pwm_o          : out std_ulogic_vector(IO_PWM_NUM_CH-1 downto 0); -- pwm channels

       -- Custom Functions Subsystem IO (available if IO_CFS_EN = true) --
       cfs_in_i       : in  std_ulogic_vector(IO_CFS_IN_SIZE-1  downto 0) := (others => 'U'); -- custom CFS inputs conduit
       cfs_out_o      : out std_ulogic_vector(IO_CFS_OUT_SIZE-1 downto 0); -- custom CFS outputs conduit

       -- NeoPixel-compatible smart LED interface (available if IO_NEOLED_EN = true) --
       neoled_o       : out std_ulogic; -- async serial data line

       -- System time --
       mtime_i        : in  std_ulogic_vector(63 downto 0) := (others => 'U'); -- current system time from ext. MTIME (if IO_MTIME_EN = false)
       mtime_o        : out std_ulogic_vector(63 downto 0); -- current system time from int. MTIME (if IO_MTIME_EN = true)

       -- External platform interrupts (available if XIRQ_NUM_CH > 0) --
       xirq_i         : in  std_ulogic_vector(XIRQ_NUM_CH-1 downto 0) := (others => 'L'); -- IRQ channels

       -- CPU interrupts --
       mtime_irq_i    : in  std_ulogic := 'L'; -- machine timer interrupt, available if IO_MTIME_EN = false
       msw_irq_i      : in  std_ulogic := 'L'; -- machine software interrupt
       mext_irq_i     : in  std_ulogic := 'L'  -- machine external interrupt
     );
   end component neorv32_top;

   
   --------------------------------------------------------
   -- Define all local signals here
   --------------------------------------------------------

   signal sys_clk    : std_logic := '0';
   signal pll_locked : std_logic := '0';
   signal reset      : std_logic := '0';
   signal reset_s1   : std_logic := '1';
   signal reset_s2   : std_logic := '1';
   signal reset_s3   : std_logic := '1';
   signal sys_rst    : std_logic;
   signal fpga_reset : std_logic;

   signal clk_i      : std_logic;
   signal rstn_i     : std_logic;
   
   signal gpio       : std_ulogic_vector(63 downto 0);


begin

   --
   -- PLL
   --
   inst_pll_sys : pll_sys
      port map ( 
         inclk0 => CLOCK_50,
         c0     => sys_clk,
         locked => pll_locked
      );

   --
   -- In general it is a bad idea to use an asynchhronous Reset signal.
   -- But it is only a bad idea in case of asynchhronous deasserting. 
   -- Therefore the deasserting of the Reset signal must be synchronized.
   --               
                              
   -- Asynchronous assert  
   fpga_reset <= '1' when ((KEY(1) = '0') OR (KEY(0) = '0')) else '0';
   reset      <= '1' when ((fpga_reset = '1') OR (pll_locked = '0')) else '0';

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
         INT_BOOTLOADER_EN            => true,             -- boot configuration: true = boot explicit bootloader; false = boot from int/ext (I)MEM

         -- On-Chip Debugger (OCD) --
         ON_CHIP_DEBUGGER_EN          => true,              -- implement on-chip debugger
         
         -- RISC-V CPU Extensions --
         CPU_EXTENSION_RISCV_A        => true,              -- implement atomic extension?
         CPU_EXTENSION_RISCV_C        => true,              -- implement compressed extension?
         CPU_EXTENSION_RISCV_M        => true,              -- implement mul/div extension?
         CPU_EXTENSION_RISCV_Zicsr    => true,              -- implement CSR system?
         CPU_EXTENSION_RISCV_Zifencei => true,              -- implement instruction stream sync.?

         -- Internal Instruction memory --
         MEM_INT_IMEM_EN              => true,              -- implement processor-internal instruction memory
         MEM_INT_IMEM_SIZE            => MEM_INT_IMEM_SIZE, -- size of processor-internal instruction memory in bytes
         
         -- Internal Data memory --
         MEM_INT_DMEM_EN              => true,              -- implement processor-internal data memory
         MEM_INT_DMEM_SIZE            => MEM_INT_DMEM_SIZE, -- size of processor-internal data memory in bytes

         -- Processor peripherals --
         IO_GPIO_EN                   => true,              -- implement general purpose input/output port unit (GPIO)?
         IO_MTIME_EN                  => true,              -- implement machine system timer (MTIME)?
         IO_UART0_EN                  => true               -- implement primary universal asynchronous receiver/transmitter (UART0)?
      )
      port map (
         -- Global control --
         clk_i        => clk_i,                             -- global clock, rising edge
         rstn_i       => rstn_i,                            -- global reset, low-active, async

         -- JTAG on-chip debugger interface (available if ON_CHIP_DEBUGGER_EN = true) --
         jtag_trst_i  => nTRST_i,                           -- low-active TAP reset (optional)
         jtag_tck_i   => TCK_i,                             -- serial clock
         jtag_tdi_i   => TDI_i,                             -- serial data input
         jtag_tdo_o   => TDO_o,                             -- serial data output
         jtag_tms_i   => TMS_i,                             -- mode select
                                                            
         -- GPIO (available if IO_GPIO_EN = true) --
         gpio_o       => gpio,                              -- parallel output
         
         -- primary UART0 (available if IO_UART0_EN = true) --
         uart0_txd_o  => UART0_TXD,                         -- UART0 send data
         uart0_rxd_i  => UART0_RXD                          -- UART0 receive data
      );

   
   --------------------------------------------------------
   -- Output
   --------------------------------------------------------

   LED <= To_StdLogicVector( gpio(7 downto 0) );

end architecture syn;

-- *** EOF ***

