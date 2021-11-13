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

entity wb_intercon is
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
end entity wb_intercon;


-- ****************************************************************************
-- *  DEFINE: Architecture                                                    *
-- ****************************************************************************

architecture syn of wb_intercon is

   --------------------------------------------------------
   -- Define all constants here
   --------------------------------------------------------

   --------------------------------------------------------
   -- Define all components which are included here
   --------------------------------------------------------
   
   --------------------------------------------------------
   -- Define all local signals here
   --------------------------------------------------------

   signal wbs1_enable : std_logic := '0';
   
begin
   
   --
   -- Coarse Address decoder
   --
   -- wbs1_enable is valid in the range from 0x90000000 to 0x9FFFFFFF
   wbs1_enable <= '1' when (wbm_adr_i(31 downto 28) = x"9") else '0';
   
   wbs1_adr_o  <= wbm_adr_i(27 downto 0);

   --
   -- Master to Slave signals
   --
   
   -- Slave x
   wbs_we_o  <= wbm_we_i;
   wbs_dat_o <= wbm_dat_i; 
   wbs_sel_o <= wbm_sel_i; 
   
   -- Slave 1
   wbs1_stb_o <= wbs1_enable and wbm_stb_i;

   --
   -- Slave to Master signals
   --
   wbm_ack_o <= wbs1_ack_i;
   
   wbm_dat_o <= wbs1_dat_i when (wbs1_enable = '1') else (others => '0');

end architecture syn;

-- *** EOF ***

