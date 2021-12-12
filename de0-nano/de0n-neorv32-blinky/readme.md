## de0n-neorv32-blinky (reference)

#### Project information:

| Board    | [Terasic DE0-Nano](https://www.terasic.com.tw/cgi-bin/page/archive.pl?Language=English&CategoryNo=139&No=593) |
| :------- | :------------- |
| FPGA     | Cyclone IV `EP4CE22F17C6N` |
| Quartus  | 15.0.2         |
| clk_i    | 100 MHz        |
| Terminal | 19200, 8, N, 1 |

#### Memory Address Map:

| Region  | Start Address | End Address | Size (bytes) | Description |
| :------ | :------------ | :---------- | :----------- | :---------- |
| IMEM    | 0x00000000    | 0x00007FFF  | 32K          | TCM On-chip RAM |
| DMEM    | 0x80000000    | 0x80003FFF  | 16K          | TCM On-chip RAM |


## Description

In terms of functionality, this project can be seen as a **reference** for testing
the Setup, Toolchain  and Debugging. 

This is a simple example using on-chip RAM as memory. Compared to the original
blinky example, not the value of the counter but a running light is displayed
on the LEDs. 

The description for the original blinky can be found [here](https://github.com/emb4fun/neorv32/tree/master/setups/quartus/de0-nano-test-setup). 

## Credits
All credit must go to Stephan Nolting who has providing the [NEORV32](https://github.com/stnolting/neorv32) project. I've only made minor changes against the original blinky project here. 

## Embedded Studio for RISC-V

A description of how to use Embedded Studio for RISC-V can be find [here](https://www.emb4fun.de/riscv/ses4rv/index.html).