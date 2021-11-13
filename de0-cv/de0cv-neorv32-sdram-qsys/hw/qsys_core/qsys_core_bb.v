
module qsys_core (
	avm_cs_i,
	avm_address_i,
	avm_read_i,
	avm_waitrequest_o,
	avm_readdata_o,
	avm_write_i,
	avm_writedata_i,
	avm_byteenable_i,
	clk_clk,
	reset_reset_n,
	sdram_addr,
	sdram_ba,
	sdram_cas_n,
	sdram_cke,
	sdram_cs_n,
	sdram_dq,
	sdram_dqm,
	sdram_ras_n,
	sdram_we_n);	

	input		avm_cs_i;
	input	[31:0]	avm_address_i;
	input		avm_read_i;
	output		avm_waitrequest_o;
	output	[31:0]	avm_readdata_o;
	input		avm_write_i;
	input	[31:0]	avm_writedata_i;
	input	[3:0]	avm_byteenable_i;
	input		clk_clk;
	input		reset_reset_n;
	output	[12:0]	sdram_addr;
	output	[1:0]	sdram_ba;
	output		sdram_cas_n;
	output		sdram_cke;
	output		sdram_cs_n;
	inout	[15:0]	sdram_dq;
	output	[1:0]	sdram_dqm;
	output		sdram_ras_n;
	output		sdram_we_n;
endmodule
