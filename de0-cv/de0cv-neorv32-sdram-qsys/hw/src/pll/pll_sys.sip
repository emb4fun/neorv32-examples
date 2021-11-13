set_global_assignment -entity "pll_sys" -library "lib_pll_sys" -name IP_TOOL_NAME "altera_pll"
set_global_assignment -entity "pll_sys" -library "lib_pll_sys" -name IP_TOOL_VERSION "15.0"
set_global_assignment -entity "pll_sys" -library "lib_pll_sys" -name IP_TOOL_ENV "mwpim"
set_global_assignment -library "lib_pll_sys" -name SPD_FILE [file join $::quartus(sip_path) "pll_sys.spd"]

set_global_assignment -library "lib_pll_sys" -name MISC_FILE [file join $::quartus(sip_path) "pll_sys_sim/pll_sys.vho"]
