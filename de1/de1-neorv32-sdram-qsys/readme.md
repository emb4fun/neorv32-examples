## de1-neorv32-sdram-qsys

#### Project information:

| Board   | [Altera DE1](https://www.terasic.com.tw/cgi-bin/page/archive.pl?Language=English&CategoryNo=53&No=83) |
| :------ | :---------- |
| FPGA    | Cyclone II `EP2C20F484C7` |
| Quartus | 11.1sp2     |
| clk_i   | 50 MHz      |

**Note:** The JTAG signal TCK_i is not connected to GPIO-1 but to EXT_CLK. 

#### Memory Address Map:

| Region  | Start Address | End Address | Size (bytes) | Description |
| :------ | :------------ | :---------- | :----------- | :---------- |
| IMEM    | 0x00000000    | 0x00003FFF  | 16K          | TCM On-chip RAM |
| DMEM    | 0x80000000    | 0x80001FFF  | 8K           | TCM On-chip RAM |
| QSYS    | 0x90000000    | 0x9FFFFFFF  | 256MB        | Accessible via the Wishbone to Avalon Master |

#### Qsys Address Map:

| Region  | Start Address | End Address | Size (bytes) | Description |
| :------ | :------------ | :---------- | :----------- | :---------- |
| SDRAM   | 0x00000000    | 0x007FFFFF  | 8MB          |  |

The SDRAM starts in the Qsys area from address 0x00000000. But since the Qsys area 
in the CPU is starting at 0x90000000, the SDRAM can be reached by the CPU at address 
0x90000000 + 0x0000000.

## Description

This is a simple example using on-chip RAM as memory for IMEM and DMEM. In addition, 
there is the SDRAM which can be addressed by the Wishbone to Avalon Master in the Qsys area.

A memory test was also implemented. The terminal output looks like:

<img src="./doc/terminal.png" width="215">

## Embedded Studio for RISC-V

A description of how to use Embedded Studio for RISC-V can be find [here](https://www.emb4fun.de/riscv/ses4rv/index.html).