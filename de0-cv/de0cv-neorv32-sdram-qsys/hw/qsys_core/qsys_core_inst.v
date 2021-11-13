	qsys_core u0 (
		.avm_cs_i          (<connected-to-avm_cs_i>),          //   avm.cs_i
		.avm_address_i     (<connected-to-avm_address_i>),     //      .address_i
		.avm_read_i        (<connected-to-avm_read_i>),        //      .read_i
		.avm_waitrequest_o (<connected-to-avm_waitrequest_o>), //      .waitrequest_o
		.avm_readdata_o    (<connected-to-avm_readdata_o>),    //      .readdata_o
		.avm_write_i       (<connected-to-avm_write_i>),       //      .write_i
		.avm_writedata_i   (<connected-to-avm_writedata_i>),   //      .writedata_i
		.avm_byteenable_i  (<connected-to-avm_byteenable_i>),  //      .byteenable_i
		.clk_clk           (<connected-to-clk_clk>),           //   clk.clk
		.reset_reset_n     (<connected-to-reset_reset_n>),     // reset.reset_n
		.sdram_addr        (<connected-to-sdram_addr>),        // sdram.addr
		.sdram_ba          (<connected-to-sdram_ba>),          //      .ba
		.sdram_cas_n       (<connected-to-sdram_cas_n>),       //      .cas_n
		.sdram_cke         (<connected-to-sdram_cke>),         //      .cke
		.sdram_cs_n        (<connected-to-sdram_cs_n>),        //      .cs_n
		.sdram_dq          (<connected-to-sdram_dq>),          //      .dq
		.sdram_dqm         (<connected-to-sdram_dqm>),         //      .dqm
		.sdram_ras_n       (<connected-to-sdram_ras_n>),       //      .ras_n
		.sdram_we_n        (<connected-to-sdram_we_n>)         //      .we_n
	);

