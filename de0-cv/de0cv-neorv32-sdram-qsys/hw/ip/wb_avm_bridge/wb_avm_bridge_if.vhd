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

entity wb_avm_bridge_if is
	port (
		clk             : in  std_logic                     := '0';                            --   clock.clk
		reset           : in  std_logic                     := '0';                            --   reset.reset
      
		avm_cs          : out std_logic;                                                       --      m0.chipselect
      avm_address     : out std_logic_vector(31 downto 0);                                   --        .address
		avm_read        : out std_logic;                                                       --        .read
		avm_waitrequest : in  std_logic                     := '0';                            --        .waitrequest
      avm_readdata    : in  std_logic_vector(31 downto 0) := (others => '0');                --        .readdata
		avm_write       : out std_logic;                                                       --        .write
      avm_writedata   : out std_logic_vector(31 downto 0);                                   --        .writedata
      avm_byteenable  : out std_logic_vector(3 downto 0);                                    --        .byteenable
      
		cs_i            : in  std_logic                     := '0';                            -- conduit.export
      address_i       : in  std_logic_vector(31 downto 0) := (others => '0');                --        .export
		read_i          : in  std_logic                     := '0';                            --        .export
		waitrequest_o   : out std_logic;                                                       --        .export
		readdata_o      : out std_logic_vector(31 downto 0);                                   --        .export
		write_i         : in  std_logic                     := '0';                            --        .export
      writedata_i     : in  std_logic_vector(31 downto 0) := (others => '0');                --        .export
      byteenable_i    : in  std_logic_vector(3 downto 0)  := (others => '0')                 --        .export
	);
end entity wb_avm_bridge_if;

-- ****************************************************************************
-- *  DEFINE: Architecture                                                    *
-- ****************************************************************************

architecture syn of wb_avm_bridge_if is
begin

	avm_cs         <= cs_i;
	avm_address    <= address_i;
	avm_read       <= read_i;
	avm_write      <= write_i;
	avm_writedata  <= writedata_i;
	avm_byteenable <= byteenable_i;
   
   waitrequest_o  <= avm_waitrequest;
   readdata_o     <= avm_readdata;

end architecture syn;

-- *** EOF ***
