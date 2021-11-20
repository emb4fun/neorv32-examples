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
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;


-- ****************************************************************************
-- *  DEFINE: Entity                                                          *
-- ****************************************************************************

entity wb_sdram is
   port (  
      -- System
      clk_i        : in  std_logic                      := '0';
      rst_i        : in  std_logic                      := '0';

      -- Wishbone
      wbs_stb_i    : in  std_logic                      := '0';
      wbs_we_i     : in  std_logic                      := '0';
      wbs_sel_i    : in  std_logic_vector(03 downto 0)  := (others => '0');
      wbs_adr_i    : in  std_logic_vector(27 downto 0)  := (others => '0');
      wbs_dat_i    : in  std_logic_vector(31 downto 0)  := (others => '0');
      wbs_dat_o    : out std_logic_vector(31 downto 0);  
      wbs_ack_o    : out std_logic;
      
      -- SDRAM
      sdram_addr   : out   std_logic_vector(12 downto 0);                    -- addr
      sdram_ba     : out   std_logic_vector(1 downto 0);                     -- ba
      sdram_cas_n  : out   std_logic;                                        -- cas_n
      sdram_cke    : out   std_logic;                                        -- cke
      sdram_cs_n   : out   std_logic;                                        -- cs_n
      sdram_dq     : inout std_logic_vector(15 downto 0) := (others => 'X'); -- dq
      sdram_dqm    : out   std_logic_vector(1 downto 0);                     -- dqm
      sdram_ras_n  : out   std_logic;                                        -- ras_n
      sdram_we_n   : out   std_logic                                         -- we_n
   );
end entity wb_sdram;


-- ****************************************************************************
-- *  DEFINE: Architecture                                                    *
-- ****************************************************************************

architecture syn of wb_sdram is

   --------------------------------------------------------
   -- Define all constants here
   --------------------------------------------------------

   --------------------------------------------------------
   -- Define all components which are included here
   --------------------------------------------------------

   --
   -- SDRAM Controller from nullobject with modification for "byteenable".
   -- The original project can be find here: https://github.com/nullobject/sdram-fpga
   --
   component sdram is
      generic (
         -- clock frequency (in MHz)
         --
         -- This value must be provided, as it is used to calculate the number of
         -- clock cycles required for the other timing values.
         CLK_FREQ : real;

         -- 32-bit controller interface
         ADDR_WIDTH : natural := 23;
         DATA_WIDTH : natural := 32;

         -- SDRAM interface
         SDRAM_ADDR_WIDTH : natural := 13;
         SDRAM_DATA_WIDTH : natural := 16;
         SDRAM_COL_WIDTH  : natural := 9;
         SDRAM_ROW_WIDTH  : natural := 13;
         SDRAM_BANK_WIDTH : natural := 2;

         -- The delay in clock cycles, between the start of a read command and the
         -- availability of the output data.
         CAS_LATENCY : natural := 2; -- 2=below 133MHz, 3=above 133MHz

         -- The number of 16-bit words to be bursted during a read/write.
         BURST_LENGTH : natural := 2;

         -- timing values (in nanoseconds)
         --
         -- These values can be adjusted to match the exact timing of your SDRAM
         -- chip (refer to the datasheet).
         T_DESL : real := 200000.0; -- startup delay
         T_MRD  : real :=     12.0; -- mode register cycle time
         T_RC   : real :=     60.0; -- row cycle time
         T_RCD  : real :=     18.0; -- RAS to CAS delay
         T_RP   : real :=     18.0; -- precharge to activate delay
         T_WR   : real :=     12.0; -- write recovery time
         T_REFI : real :=   7800.0  -- average refresh interval
      );
      port (
         -- reset
         reset : in std_logic := '0';

         -- clock
         clk : in std_logic;

         -- address bus
         addr : in unsigned(ADDR_WIDTH-1 downto 0);

         -- input data bus
         data : in std_logic_vector(DATA_WIDTH-1 downto 0);
         
         -- input data mask
         mask : in std_logic_vector(3 downto 0);
         
         -- When the write enable signal is asserted, a write operation will be performed.
         we : in std_logic;

         -- When the request signal is asserted, an operation will be performed.
         req : in std_logic;

         -- The acknowledge signal is asserted by the SDRAM controller when
         -- a request has been accepted.
         ack : out std_logic;

         -- The valid signal is asserted when there is a valid word on the output
         -- data bus.
         valid : out std_logic;

         -- output data bus
         q : out std_logic_vector(DATA_WIDTH-1 downto 0);

         -- SDRAM interface (e.g. AS4C16M16SA-6TCN, IS42S16400F, etc.)
         sdram_a     : out unsigned(SDRAM_ADDR_WIDTH-1 downto 0);
         sdram_ba    : out unsigned(SDRAM_BANK_WIDTH-1 downto 0);
         sdram_dq    : inout std_logic_vector(SDRAM_DATA_WIDTH-1 downto 0);
         sdram_cke   : out std_logic;
         sdram_cs_n  : out std_logic;
         sdram_ras_n : out std_logic;
         sdram_cas_n : out std_logic;
         sdram_we_n  : out std_logic;
         sdram_dqml  : out std_logic;
         sdram_dqmh  : out std_logic
      );
   end component sdram;
   
   
   --------------------------------------------------------
   -- Define all local signals here
   --------------------------------------------------------

   signal read_sig       : std_logic;
   signal write_sig      : std_logic;
   signal read_ack       : std_logic;
   signal write_ack      : std_logic;
   signal read_wait_cnt  : integer range 0 to 8;
   signal write_wait_cnt : integer range 0 to 8;

   signal sdram_we       : std_logic;
   signal sdram_mask     : std_logic_vector(3 downto 0);
   signal sdram_req      : std_logic;
   signal sdram_ack      : std_logic;
   signal sdram_valid    : std_logic;

   signal sdram_adr      : unsigned(23-1 downto 0);
   signal sdram_data     : std_logic_vector(32-1 downto 0);
   signal sdram_q        : std_logic_vector(32-1 downto 0);
   signal sdram_addr_int : unsigned(13-1 downto 0);
   signal sdram_ba_int   : unsigned(2-1 downto 0);

   signal sdram_dqml     : std_logic;
   signal sdram_dqmh     : std_logic;
   
   --
   -- FSM defines
   --
   type   fsm_state_t is (STATE_IDLE, STATE_WRITE_1, STATE_WRITE_2, STATE_READ_1, STATE_READ_2, STATE_READ_3, STATE_DONE);
   signal fsm_state   : fsm_state_t := STATE_IDLE;

   
begin

   -- Convert wishbone signals             
   read_sig  <= '1' when (wbs_stb_i = '1' and wbs_we_i = '0') else '0';
   write_sig <= '1' when (wbs_stb_i = '1' and wbs_we_i = '1') else '0';


   --
   -- SDRAM
   --
   inst_sdram : sdram
      generic map (
         -- clock frequency (in MHz)
         --
         -- This value must be provided, as it is used to calculate the number of
         -- clock cycles required for the other timing values.
         CLK_FREQ         => 100.0,

         -- 32-bit controller interface
         ADDR_WIDTH       => 23,
         DATA_WIDTH       => 32,

         -- SDRAM interface
         SDRAM_ADDR_WIDTH => 13,
         SDRAM_DATA_WIDTH => 16,
         SDRAM_COL_WIDTH  => 9,
         SDRAM_ROW_WIDTH  => 13,
         SDRAM_BANK_WIDTH => 2,

         -- The delay in clock cycles, between the start of a read command and the
         -- availability of the output data.
         CAS_LATENCY      => 2,  -- 2=below 133MHz, 3=above 133MHz

         -- The number of 16-bit words to be bursted during a read/write.
         BURST_LENGTH     => 2,

         -- timing values (in nanoseconds)
         --
         -- These values can be adjusted to match the exact timing of your SDRAM
         -- chip (refer to the datasheet).
         T_DESL =>  50000.0,     -- startup delay
         T_MRD  =>     12.0,     -- mode register cycle time
         T_RC   =>     60.0,     -- row cycle time
         T_RCD  =>     18.0,     -- RAS to CAS delay
         T_RP   =>     18.0,     -- precharge to activate delay
         T_WR   =>     12.0,     -- write recovery time
         T_REFI =>   7800.0      -- average refresh interval
      )
      port map (
         -- reset
         reset => rst_i,

         -- clock
         clk   => clk_i,

         -- address bus
         addr  => sdram_adr,

         -- input data bus
         data  => sdram_data,
         
         -- input data mask
         mask  => sdram_mask,

         -- When the write enable signal is asserted, a write operation will be performed.
         we    => sdram_we,

         -- When the request signal is asserted, an operation will be performed.
         req   => sdram_req,

         -- The acknowledge signal is asserted by the SDRAM controller when
         -- a request has been accepted.
         ack   => sdram_ack,

         -- The valid signal is asserted when there is a valid word on the output
         -- data bus.
         valid => sdram_valid,

         -- output data bus
         q     => sdram_q,

         -- SDRAM interface (e.g. AS4C16M16SA-6TCN, IS42S16400F, etc.)
         sdram_a     => sdram_addr_int,
         sdram_ba    => sdram_ba_int,   
         sdram_dq    => sdram_dq,   
         sdram_cke   => sdram_cke,  
         sdram_cs_n  => sdram_cs_n, 
         sdram_ras_n => sdram_ras_n,
         sdram_cas_n => sdram_cas_n,
         sdram_we_n  => sdram_we_n, 
         sdram_dqml  => sdram_dqml,
         sdram_dqmh  => sdram_dqmh
      );
      
      sdram_addr <= std_logic_vector( sdram_addr_int );
      sdram_ba   <= std_logic_vector( sdram_ba_int );      
      
      sdram_dqm(0) <= sdram_dqml;
      sdram_dqm(1) <= sdram_dqmh;


   --
   -- Single Process FSM
   --
   process (clk_i, rst_i)     
   begin
      if (rst_i = '1') then
         fsm_state <= STATE_IDLE;
         sdram_we   <= '0';
         sdram_mask <= (others => '0');
         sdram_req  <= '0';
         write_ack  <= '0';
         read_ack   <= '0';
         
      elsif rising_edge(clk_i) then
                     
         -- Default
         sdram_we   <= sdram_we;
         sdram_mask <= sdram_mask;
         sdram_req  <= sdram_req;
         write_ack  <= '0';
         read_ack   <= '0';
         
         case (fsm_state) is

            --
            -- STATE_IDLE
            --
            when STATE_IDLE =>
               sdram_adr  <= unsigned( wbs_adr_i(24 downto 2) );
               sdram_data <= wbs_dat_i;
               
               if (write_sig = '1') then
                  sdram_we   <= '1';   
                  sdram_mask <= wbs_sel_i;
                  sdram_req  <= '1';
                  fsm_state  <= STATE_WRITE_1;
               end if;
               if (read_sig = '1') then
                  sdram_we   <= '0';   
                  sdram_mask <= "1111";
                  sdram_req  <= '1';
                  fsm_state  <= STATE_READ_1;
               end if;
            -- STATE_IDLE

            
            --
            -- STATE_WRITE_1
            -- 
            when STATE_WRITE_1 =>
               if (sdram_ack = '1') then
                  sdram_we  <= '0';   
                  sdram_req <= '0';
                  write_ack <= '1';
                  fsm_state <= STATE_WRITE_2;
               else   
                  fsm_state <= STATE_WRITE_1;
               end if;
            -- STATE_WRITE_1


            --
            -- STATE_WRITE_2
            -- 
            when STATE_WRITE_2 =>
               write_ack <= '0';
               fsm_state <= STATE_IDLE;
            -- STATE_WRITE_2


            --
            -- STATE_READ_1
            -- 
            when STATE_READ_1 =>
               if (sdram_ack = '1') then
                  sdram_req <= '0';
                  fsm_state <= STATE_READ_2;
               else   
                  fsm_state <= STATE_READ_1;
               end if;
            -- STATE_WRITE_1


            --
            -- STATE_READ_2
            -- 
            when STATE_READ_2 =>
               if (sdram_valid = '1') then
                  wbs_dat_o <= sdram_q;
                  read_ack  <= '1';
                  fsm_state <= STATE_READ_3;
               else   
                  fsm_state <= STATE_READ_2;
               end if;
            -- STATE_WRITE_1


            --
            -- STATE_READ_3
            -- 
            when STATE_READ_3 =>
               read_ack  <= '0';
               fsm_state <= STATE_DONE;
            -- STATE_WRITE_3


            --
            -- others
            --
            when others =>
               fsm_state <= STATE_IDLE;
            -- others
               
         end case;
         
      end if;
   end process;
   
   
   --------------------------------------------------------
   -- Output
   --------------------------------------------------------
   
   wbs_ack_o <= '1' when ((wbs_stb_i = '1') and ((read_ack = '1' or write_ack = '1'))) else '0';

end architecture syn;

-- *** EOF ***

