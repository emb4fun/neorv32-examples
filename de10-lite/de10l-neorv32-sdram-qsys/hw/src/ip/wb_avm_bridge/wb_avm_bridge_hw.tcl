# TCL File Generated by Component Editor 11.1sp2
# Sat Nov 07 13:08:58 CET 2015
# DO NOT MODIFY


# +-----------------------------------
# | 
# | cpu_avm "Wishbone to Avalon Bridge (32 bit)" v1.0
# | Michael Fischer 2015.11.07.13:08:58
# | 
# | 
# | C:/my_design/neorv32-de0n-sdram-qsys/ip/wb_avm_bridge/wb_avm_bridge_if.vhd
# | 
# |    ./wb_avm_bridge_if.vhd syn, sim
# | 
# +-----------------------------------

# +-----------------------------------
# | request TCL package from ACDS 11.0
# | 
package require -exact sopc 11.0
# | 
# +-----------------------------------

# +-----------------------------------
# | module wb_avm_bridge
# | 
set_module_property NAME wb_avm_bridge
set_module_property VERSION 1.0
set_module_property INTERNAL false
set_module_property OPAQUE_ADDRESS_MAP true
set_module_property AUTHOR "Michael Fischer"
set_module_property DISPLAY_NAME "Wishbone to Avalon Bridge (32 bit)"
set_module_property TOP_LEVEL_HDL_FILE wb_avm_bridge_if.vhd
set_module_property TOP_LEVEL_HDL_MODULE wb_avm_bridge_if
set_module_property INSTANTIATE_IN_SYSTEM_MODULE true
set_module_property EDITABLE true
set_module_property ANALYZE_HDL AUTO
set_module_property STATIC_TOP_LEVEL_MODULE_NAME wb_avm_bridge_if
set_module_property FIX_110_VIP_PATH false
# | 
# +-----------------------------------

# +-----------------------------------
# | files
# | 
add_file wb_avm_bridge_if.vhd {SYNTHESIS SIMULATION}
# | 
# +-----------------------------------

# +-----------------------------------
# | parameters
# | 
# | 
# +-----------------------------------

# +-----------------------------------
# | display items
# | 
# | 
# +-----------------------------------

# +-----------------------------------
# | connection point clock
# | 
add_interface clock clock end
set_interface_property clock clockRate 0

set_interface_property clock ENABLED true

add_interface_port clock clk clk Input 1
# | 
# +-----------------------------------

# +-----------------------------------
# | connection point reset
# | 
add_interface reset reset end
set_interface_property reset associatedClock clock
set_interface_property reset synchronousEdges DEASSERT

set_interface_property reset ENABLED true

add_interface_port reset reset reset Input 1
# | 
# +-----------------------------------

# +-----------------------------------
# | connection point m0
# | 
add_interface m0 avalon start
set_interface_property m0 addressUnits SYMBOLS
set_interface_property m0 associatedClock clock
set_interface_property m0 associatedReset reset
set_interface_property m0 bitsPerSymbol 8
set_interface_property m0 burstOnBurstBoundariesOnly false
set_interface_property m0 burstcountUnits WORDS
set_interface_property m0 doStreamReads false
set_interface_property m0 doStreamWrites false
set_interface_property m0 holdTime 0
set_interface_property m0 linewrapBursts false
set_interface_property m0 maximumPendingReadTransactions 0
set_interface_property m0 readLatency 0
set_interface_property m0 readWaitTime 1
set_interface_property m0 setupTime 0
set_interface_property m0 timingUnits Cycles
set_interface_property m0 writeWaitTime 0

set_interface_property m0 ENABLED true

add_interface_port m0 avm_cs chipselect Output 1
add_interface_port m0 avm_address address Output 32
add_interface_port m0 avm_read read Output 1
add_interface_port m0 avm_waitrequest waitrequest Input 1
add_interface_port m0 avm_readdata readdata Input 32
add_interface_port m0 avm_write write Output 1
add_interface_port m0 avm_writedata writedata Output 32
add_interface_port m0 avm_byteenable byteenable Output 4
# | 
# +-----------------------------------

# +-----------------------------------
# | connection point conduit
# | 
add_interface conduit conduit end

set_interface_property conduit ENABLED true

add_interface_port conduit cs_i cs_i Input 1
add_interface_port conduit address_i address_i Input 32
add_interface_port conduit read_i read_i Input 1
add_interface_port conduit waitrequest_o waitrequest_o Output 1
add_interface_port conduit readdata_o readdata_o Output 32
add_interface_port conduit write_i write_i Input 1
add_interface_port conduit writedata_i writedata_i Input 32
add_interface_port conduit byteenable_i byteenable_i Input 4
# | 
# +-----------------------------------
