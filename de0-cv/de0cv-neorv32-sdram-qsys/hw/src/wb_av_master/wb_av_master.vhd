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

entity wb_av_master is
   port (  
      -- System
      clk_i             : in  std_logic := '0';
      rst_i             : in  std_logic := '0';

      -- Wishbone
      wbs_stb_i    : in  std_logic                      := '0';
      wbs_we_i     : in  std_logic                      := '0';
      wbs_sel_i    : in  std_logic_vector(03 downto 0)  := (others => '0');
      wbs_adr_i    : in  std_logic_vector(27 downto 0)  := (others => '0');
      wbs_dat_i    : in  std_logic_vector(31 downto 0)  := (others => '0');
      wbs_dat_o    : out std_logic_vector(31 downto 0);  
      wbs_ack_o    : out std_logic;

      -- Avalon Master
      avm_read_o        : out std_logic;
      avm_write_o       : out std_logic;
      avm_waitrequest_i : in  std_logic                        := '0';
      avm_byteenable_o  : out std_logic_vector(03 downto 0);
      avm_address_o     : out std_logic_vector(27 downto 0);
      avm_writedata_o   : out std_logic_vector(31 downto 0);
      avm_readdata_i    : in  std_logic_vector(31 downto 0)    := (others => '0')
   );
end entity wb_av_master;


-- ****************************************************************************
-- *  DEFINE: Architecture                                                    *
-- ****************************************************************************

architecture syn of wb_av_master is

   --------------------------------------------------------
   -- Define all constants here
   --------------------------------------------------------

   --------------------------------------------------------
   -- Define all components which are included here
   --------------------------------------------------------
   
   --------------------------------------------------------
   -- Define all local signals here
   --------------------------------------------------------

   -- Wishbone
   signal wb_read     : std_logic := '0';
   signal wb_write    : std_logic := '0';
   signal wb_ack      : std_logic := '0';
   
   --
   -- FSM defines
   --
   type   fsm_state_t is (STATE_IDLE, STATE_READ_1, STATE_READ_2, STATE_WRITE_1, STATE_WRITE_2, STATE_DONE);
   signal fsm_state   : fsm_state_t := STATE_IDLE;

   signal wait_cnt    : std_logic_vector(5 downto 0) := (others => '0');  

   signal av_read       : std_logic := '0';
   signal av_write      : std_logic := '0';
   signal av_readdata   : std_logic_vector(31 downto 0);
   signal av_byteenable : std_logic_vector(03 downto 0);
   
begin

   -- Convert wishbone signals             
   wb_read   <= '1' when (wbs_stb_i = '1' and wbs_we_i = '0') else '0';
   wb_write  <= '1' when (wbs_stb_i = '1' and wbs_we_i = '1') else '0';
   
   
   --
   -- Single Process FSM
   -- Used to support the following master signals:
   --
   --    avm_read_o       
   --    avm_write_o      
   --    avm_waitrequest_i
   --
   process (clk_i, rst_i)     
   begin
      if (rst_i = '1') then
         fsm_state <= STATE_IDLE;
         av_read  <= '0';
         av_write <= '0';
         wb_ack   <= '0';
         
      elsif rising_edge(clk_i) then
                     
         -- Default
         av_read  <= av_read;
         av_write <= av_write;
         wb_ack   <= wb_ack;
         
         case (fsm_state) is
         
            --
            -- STATE_IDLE
            --
            when STATE_IDLE =>
               av_read    <= '0';
               av_write   <= '0';
               wait_cnt   <= (others =>'0');
               wb_ack     <= '0';
               
               if (wb_read = '1') then          -- Check for CPU read access
                  fsm_state <= STATE_READ_1;
                  av_read   <= '1';             -- Signal a read cycle on the Avalon bus
                  av_byteenable <= "1111";
               end if;
               if (wb_write = '1') then         -- Check for CPU write access
                  fsm_state <= STATE_WRITE_1;
                  av_write  <= '1';             -- Signal a write cycle on the Avalon bus
                  av_byteenable <= wbs_sel_i;
               end if;

            --
            -- STATE_READ_1
            --
            when STATE_READ_1 =>               
               fsm_state <= STATE_READ_2;

            --
            -- STATE_READ_2
            --
            when STATE_READ_2 =>               
               if (avm_waitrequest_i = '0') then   -- Wait for a normal termination of a Avalon bus cycle                  
                  fsm_state   <= STATE_DONE;
                  av_read     <= '0';              -- Signal end of read cycle on the Avalon bus
                  av_readdata <= avm_readdata_i;   -- Avalon data can be stored
                  wb_ack      <= '1';
               else               
                  fsm_state <= STATE_READ_2;
                  wait_cnt  <= std_logic_vector(unsigned(wait_cnt) + 1);
               end if;

            --
            -- STATE_WRITE_1
            --
            when STATE_WRITE_1 =>                    
               fsm_state <= STATE_WRITE_2;

            --
            -- STATE_WRITE_2
            --
            when STATE_WRITE_2 =>                    
               if (avm_waitrequest_i = '0') then   -- Wait for a normal termination of a Avalon bus cycle
                  fsm_state <= STATE_DONE;
                  av_write  <= '0';                -- Signal end of write cycle on the Avalon bus
                  wb_ack    <= '1';
               else
                  fsm_state <= STATE_WRITE_2;
                  wait_cnt  <= std_logic_vector(unsigned(wait_cnt) + 1);
               end if;

            --
            -- STATE_DONE
            --
            when STATE_DONE =>
               wb_ack <= '0';
               if (wbs_stb_i = '0') then            -- Wait for the end of the CPU bus cycle 
                  fsm_state <= STATE_IDLE;
               else
                  fsm_state <= STATE_DONE;
               end if;

               
            when others =>
               fsm_state <= STATE_IDLE;
               
         end case;
         
         --
         -- Abort the cycle if the bus is occupied too long.
         --
         if (wait_cnt = "111111") then
            fsm_state <= STATE_IDLE;
         end if;
         
      end if;
   end process;
   
   
   --------------------------------------------------------
   -- Output
   --------------------------------------------------------
   
   -- Control
   avm_read_o       <= av_read;
   avm_write_o      <= av_write;
	avm_byteenable_o <= av_byteenable;
   
   -- Address 
   avm_address_o   <= wbs_adr_i(27 downto 2) & "00";
   
   -- Data
   wbs_dat_o       <= av_readdata;
   avm_writedata_o <= wbs_dat_i;
   
   -- ACK
   wbs_ack_o <= wb_ack;
                                
end architecture syn;

-- *** EOF ***

