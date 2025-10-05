-- ================================================================================ --
-- NEORV32 CPU - Load/Store Unit                                                    --
-- -------------------------------------------------------------------------------- --
-- The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              --
-- Copyright (c) NEORV32 contributors.                                              --
-- Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  --
-- Licensed under the BSD-3-Clause license, see LICENSE for details.                --
-- SPDX-License-Identifier: BSD-3-Clause                                            --
-- ================================================================================ --

library ieee;
use ieee.std_logic_1164.all;

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_cpu_lsu is
  port (
    -- global control --
    clk_i       : in  std_ulogic; -- global clock, rising edge
    rstn_i      : in  std_ulogic; -- global reset, low-active, async
    ctrl_i      : in  ctrl_bus_t; -- main control bus
    -- cpu data access interface --
    addr_i      : in  std_ulogic_vector(XLEN-1 downto 0); -- access address
    wdata_i     : in  std_ulogic_vector(XLEN-1 downto 0); -- write data
    rdata_o     : out std_ulogic_vector(XLEN-1 downto 0); -- read data
    mar_o       : out std_ulogic_vector(XLEN-1 downto 0); -- current memory address register
    wait_o      : out std_ulogic; -- wait for access to complete
    err_o       : out std_ulogic_vector(3 downto 0); -- alignment/access errors
    pmp_fault_i : in  std_ulogic; -- PMP read/write access fault
    -- data bus --
    dbus_req_o  : out bus_req_t; -- request
    dbus_rsp_i  : in  bus_rsp_t  -- response
  );
end neorv32_cpu_lsu;

architecture neorv32_cpu_lsu_rtl of neorv32_cpu_lsu is

  signal mar        : std_ulogic_vector(XLEN-1 downto 0); -- memory address register
  signal misaligned : std_ulogic; -- misaligned address
  signal pending    : std_ulogic; -- pending bus request
  signal pmp_err    : std_ulogic; -- PMP access violation
  signal amo_cmd    : std_ulogic_vector(3 downto 0); -- atomic memory operation type
  signal exc_rd     : std_ulogic; -- for exceptions: this is a read operation
  signal exc_wr     : std_ulogic; -- for exceptions: this is a write operation

begin

  -- Access Address -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  mem_addr_reg: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      mar        <= (others => '0');
      misaligned <= '0';
    elsif rising_edge(clk_i) then
      if (ctrl_i.lsu_mo_we = '1') then
        mar <= addr_i; -- memory address register
        case ctrl_i.ir_funct3(1 downto 0) is -- alignment check
          when "00"   => misaligned <= '0'; -- byte
          when "01"   => misaligned <= addr_i(0); -- half-word
          when others => misaligned <= addr_i(1) or addr_i(0); -- word
        end case;
      end if;
    end if;
  end process mem_addr_reg;

  -- address output --
  dbus_req_o.addr <= mar; -- bus address
  mar_o           <= mar; -- for MTVAL CSR


  -- Data Output: Alignment, Byte Enable and Type Identifiers -------------------------------
  -- -------------------------------------------------------------------------------------------
  mem_do_reg: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      dbus_req_o.rw    <= '0';
      dbus_req_o.priv  <= '0';
      dbus_req_o.debug <= '0';
      dbus_req_o.amo   <= '0';
      dbus_req_o.amoop <= (others => '0');
      dbus_req_o.data  <= (others => '0');
      dbus_req_o.ben   <= (others => '0');
    elsif rising_edge(clk_i) then
      if (ctrl_i.lsu_mo_we = '1') then
        -- type identifiers --
        dbus_req_o.rw    <= ctrl_i.lsu_rw; -- read/write
        dbus_req_o.priv  <= ctrl_i.lsu_priv; -- privilege level
        dbus_req_o.debug <= ctrl_i.cpu_debug; -- debug-mode access
        dbus_req_o.amo   <= ctrl_i.lsu_rmw or ctrl_i.lsu_rvs; -- atomic memory operation
        dbus_req_o.amoop <= amo_cmd;
        -- data alignment + byte-enable --
        case ctrl_i.ir_funct3(1 downto 0) is
          when "00" => -- byte
            dbus_req_o.data   <= wdata_i(7 downto 0) & wdata_i(7 downto 0) & wdata_i(7 downto 0) & wdata_i(7 downto 0);
            dbus_req_o.ben(0) <= (not addr_i(1)) and (not addr_i(0));
            dbus_req_o.ben(1) <= (not addr_i(1)) and (    addr_i(0));
            dbus_req_o.ben(2) <= (    addr_i(1)) and (not addr_i(0));
            dbus_req_o.ben(3) <= (    addr_i(1)) and (    addr_i(0));
          when "01" => -- half-word
            dbus_req_o.data <= wdata_i(15 downto 0) & wdata_i(15 downto 0);
            dbus_req_o.ben  <= addr_i(1) & addr_i(1) & (not addr_i(1)) & (not addr_i(1));
          when others => -- word
            dbus_req_o.data <= wdata_i;
            dbus_req_o.ben  <= (others => '1');
        end case;
      end if;
    end if;
  end process mem_do_reg;

  -- hardwired signals --
  dbus_req_o.src   <= '0'; -- always data access
  dbus_req_o.burst <= '0'; -- only single-access

  -- out-of-band signals --
  dbus_req_o.fence <= ctrl_i.lsu_fence;

  -- atomic memory access operation encoding --
  amo_encode: process(ctrl_i.ir_funct12)
  begin
    case ctrl_i.ir_funct12(11 downto 7) is
      when "00010" => amo_cmd <= "1000"; -- Zalrsc.LR
      when "00011" => amo_cmd <= "1001"; -- Zalrsc.SC
      when "00000" => amo_cmd <= "0001"; -- Zaamo.ADD
      when "00100" => amo_cmd <= "0010"; -- Zaamo.XOR
      when "01100" => amo_cmd <= "0011"; -- Zaamo.AND
      when "01000" => amo_cmd <= "0100"; -- Zaamo.OR
      when "10000" => amo_cmd <= "1110"; -- Zaamo.MIN
      when "10100" => amo_cmd <= "1111"; -- Zaamo.MAX
      when "11000" => amo_cmd <= "0110"; -- Zaamo.MINU
      when "11100" => amo_cmd <= "0111"; -- Zaamo.MAXU
      when others  => amo_cmd <= "0000"; -- Zaamo.SWAP
    end case;
  end process;


  -- Bus-Locking (for atomic read-modify-write operations) ----------------------------------
  -- -------------------------------------------------------------------------------------------
  bus_lock: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      dbus_req_o.lock <= '0';
    elsif rising_edge(clk_i) then
      if (ctrl_i.lsu_mo_we = '1') and (ctrl_i.lsu_rmw = '1') and (ctrl_i.ir_funct12(8) = '0') then
        dbus_req_o.lock <= '1'; -- set if Zaamo instruction
      elsif (dbus_rsp_i.ack = '1') or (ctrl_i.cpu_trap = '1') then
        dbus_req_o.lock <= '0'; -- clear at the end of the bus access
      end if;
    end if;
  end process bus_lock;


  -- Data Input: Alignment and Sign-Extension -----------------------------------------------
  -- -------------------------------------------------------------------------------------------
  mem_di_reg: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      rdata_o <= (others => '0');
    elsif rising_edge(clk_i) then
      rdata_o <= (others => '0'); -- output zero if there is no memory access
      if (pending = '1') then -- pending request
        case ctrl_i.ir_funct3(1 downto 0) is
          when "00" => -- byte
            case mar(1 downto 0) is
              when "00"   => rdata_o <= replicate_f((not ctrl_i.ir_funct3(2)) and dbus_rsp_i.data(7),  24) & dbus_rsp_i.data(7 downto 0);
              when "01"   => rdata_o <= replicate_f((not ctrl_i.ir_funct3(2)) and dbus_rsp_i.data(15), 24) & dbus_rsp_i.data(15 downto 8);
              when "10"   => rdata_o <= replicate_f((not ctrl_i.ir_funct3(2)) and dbus_rsp_i.data(23), 24) & dbus_rsp_i.data(23 downto 16);
              when others => rdata_o <= replicate_f((not ctrl_i.ir_funct3(2)) and dbus_rsp_i.data(31), 24) & dbus_rsp_i.data(31 downto 24);
            end case;
          when "01" => -- half-word
            if (mar(1) = '0') then -- low half-word
              rdata_o <= replicate_f((not ctrl_i.ir_funct3(2)) and dbus_rsp_i.data(15), 16) & dbus_rsp_i.data(15 downto 0);
            else -- high half-word
              rdata_o <= replicate_f((not ctrl_i.ir_funct3(2)) and dbus_rsp_i.data(31), 16) & dbus_rsp_i.data(31 downto 16);
            end if;
          when others => -- word
            rdata_o <= dbus_rsp_i.data;
        end case;
      end if;
    end if;
  end process mem_di_reg;


  -- Access Arbiter -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  access_arbiter: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      pmp_err <= '0';
      pending <= '0';
    elsif rising_edge(clk_i) then
      pmp_err <= pmp_fault_i;
      if (pending = '0') then -- idle
        pending <= ctrl_i.lsu_req;
      elsif (dbus_rsp_i.ack = '1') or (ctrl_i.cpu_trap = '1') then -- bus response or start of trap handling
        pending <= '0';
      end if;
    end if;
  end process access_arbiter;

  -- wait for bus response --
  wait_o <= not dbus_rsp_i.ack;

  -- filter exceptions: RMW-AMOs only cause STORE exceptions --
  exc_rd <= '0' when (ctrl_i.lsu_rmw = '1') else not ctrl_i.lsu_rw;
  exc_wr <= '1' when (ctrl_i.lsu_rmw = '1') else     ctrl_i.lsu_rw;

  -- output access/alignment errors to control unit --
  err_o(0) <= pending and exc_rd and misaligned; -- misaligned load
  err_o(1) <= pending and exc_rd and (dbus_rsp_i.err or pmp_err); -- load bus access error
  err_o(2) <= pending and exc_wr and misaligned; -- misaligned store
  err_o(3) <= pending and exc_wr and (dbus_rsp_i.err or pmp_err); -- store bus access error

  -- access request (all source signals are driven by registers) --
  dbus_req_o.stb <= ctrl_i.lsu_req and (not misaligned) and (not pmp_fault_i);


end neorv32_cpu_lsu_rtl;
