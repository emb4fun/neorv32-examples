## de0n-neorv32-sdram-direct

#### Equipment information:

| Board   | [Terasic DE0-Nano](https://www.terasic.com.tw/cgi-bin/page/archive.pl?Language=English&CategoryNo=139&No=593) |
| :------ | :---------- |
| FPGA    | Cyclone IV `EP4CE22F17C6N` |
| Quartus | 11.1sp2     |

#### Memory Address Map:

| Region  | Start Address | End Address | Size (bytes) | Description |
| :------ | :------------ | :---------- | :----------- | :---------- |
| IMEM    | 0x00000000    | 0x00003FFF  | 16K          | TCM On-chip RAM |
| DMEM    | 0x80000000    | 0x80001FFF  | 8K           | TCM On-chip RAM |
| SDRAM   | 0x90000000    | 0x91FFFFFF  | 32MB         | The SDRAM is accessed via the Wishbone bus and the SDRAM controller from nullobject  |

## Description

This is a simple example using on-chip RAM as memory for IMEM and DMEM. In addition, there is the SDRAM which can be addressed directly via the Wishbone bus.

A memory test was also implemented. The terminal output looks like:

><dl>
*****************************<br>
&nbsp;&nbsp;&nbsp;&nbsp;Memory test and running light<br>
*****************************<br>
<br>
Memory test for 33554432 bytes is started.<br>
&nbsp;&nbsp;Data Bus Test... OK<br>
&nbsp;&nbsp;Address Bus Test... OK<br>
&nbsp;&nbsp;Device Test... OK<br>
Memory test successfully completed.<br>
<br>
Scotty! Energie, starting running light...<br>
</dl>

## Embedded Studio for RISC-V
A description of how to use Embedded Studio for RISC-V will be available soon.