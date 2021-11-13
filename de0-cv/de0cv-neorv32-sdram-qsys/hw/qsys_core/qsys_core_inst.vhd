	component qsys_core is
		port (
			avm_cs_i          : in    std_logic                     := 'X';             -- cs_i
			avm_address_i     : in    std_logic_vector(31 downto 0) := (others => 'X'); -- address_i
			avm_read_i        : in    std_logic                     := 'X';             -- read_i
			avm_waitrequest_o : out   std_logic;                                        -- waitrequest_o
			avm_readdata_o    : out   std_logic_vector(31 downto 0);                    -- readdata_o
			avm_write_i       : in    std_logic                     := 'X';             -- write_i
			avm_writedata_i   : in    std_logic_vector(31 downto 0) := (others => 'X'); -- writedata_i
			avm_byteenable_i  : in    std_logic_vector(3 downto 0)  := (others => 'X'); -- byteenable_i
			clk_clk           : in    std_logic                     := 'X';             -- clk
			reset_reset_n     : in    std_logic                     := 'X';             -- reset_n
			sdram_addr        : out   std_logic_vector(12 downto 0);                    -- addr
			sdram_ba          : out   std_logic_vector(1 downto 0);                     -- ba
			sdram_cas_n       : out   std_logic;                                        -- cas_n
			sdram_cke         : out   std_logic;                                        -- cke
			sdram_cs_n        : out   std_logic;                                        -- cs_n
			sdram_dq          : inout std_logic_vector(15 downto 0) := (others => 'X'); -- dq
			sdram_dqm         : out   std_logic_vector(1 downto 0);                     -- dqm
			sdram_ras_n       : out   std_logic;                                        -- ras_n
			sdram_we_n        : out   std_logic                                         -- we_n
		);
	end component qsys_core;

	u0 : component qsys_core
		port map (
			avm_cs_i          => CONNECTED_TO_avm_cs_i,          --   avm.cs_i
			avm_address_i     => CONNECTED_TO_avm_address_i,     --      .address_i
			avm_read_i        => CONNECTED_TO_avm_read_i,        --      .read_i
			avm_waitrequest_o => CONNECTED_TO_avm_waitrequest_o, --      .waitrequest_o
			avm_readdata_o    => CONNECTED_TO_avm_readdata_o,    --      .readdata_o
			avm_write_i       => CONNECTED_TO_avm_write_i,       --      .write_i
			avm_writedata_i   => CONNECTED_TO_avm_writedata_i,   --      .writedata_i
			avm_byteenable_i  => CONNECTED_TO_avm_byteenable_i,  --      .byteenable_i
			clk_clk           => CONNECTED_TO_clk_clk,           --   clk.clk
			reset_reset_n     => CONNECTED_TO_reset_reset_n,     -- reset.reset_n
			sdram_addr        => CONNECTED_TO_sdram_addr,        -- sdram.addr
			sdram_ba          => CONNECTED_TO_sdram_ba,          --      .ba
			sdram_cas_n       => CONNECTED_TO_sdram_cas_n,       --      .cas_n
			sdram_cke         => CONNECTED_TO_sdram_cke,         --      .cke
			sdram_cs_n        => CONNECTED_TO_sdram_cs_n,        --      .cs_n
			sdram_dq          => CONNECTED_TO_sdram_dq,          --      .dq
			sdram_dqm         => CONNECTED_TO_sdram_dqm,         --      .dqm
			sdram_ras_n       => CONNECTED_TO_sdram_ras_n,       --      .ras_n
			sdram_we_n        => CONNECTED_TO_sdram_we_n         --      .we_n
		);

