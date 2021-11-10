#***************************************************************************
#  Copyright (c) 2015 by Michael Fischer (www.emb4fun.de)
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without 
#  modification, are permitted provided that the following conditions 
#  are met:
#  
#  1. Redistributions of source code must retain the above copyright 
#     notice, this list of conditions and the following disclaimer.
#  2. Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the 
#     documentation and/or other materials provided with the distribution.
#  3. Neither the name of the author nor the names of its contributors may 
#     be used to endorse or promote products derived from this software 
#     without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL 
#  THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS 
#  OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED 
#  AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
#  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF 
#  THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF 
#  SUCH DAMAGE.
#
#***************************************************************************
#  History:
#
#  12.07.2015  mifi  First version
#***************************************************************************


#***************************************************************************
# Create Clock
#***************************************************************************
create_clock -period 50MHz  [get_ports CLOCK_50]
#create_clock -period 10MHz -name {altera_reserved_tck} {altera_reserved_tck}

#***************************************************************************
# Create Generated Clock
#***************************************************************************
derive_pll_clocks

#***************************************************************************
# Set Clock Latency
#***************************************************************************

#***************************************************************************
# Set Clock Uncertainty
#***************************************************************************
derive_clock_uncertainty

#***************************************************************************
# Set Input Delay
#***************************************************************************

#***************************************************************************
# Set Output Delay
#***************************************************************************

#***************************************************************************
# Set Clock Groups
#***************************************************************************

#***************************************************************************
# Set False Path
#***************************************************************************
#set_false_path -from [get_clocks {altera_reserved_tck}] -to [get_clocks {altera_reserved_tck}]

#***************************************************************************
# Set Multicycle Path
#***************************************************************************

#***************************************************************************
# Set Maximum Delay
#***************************************************************************

#***************************************************************************
# Set Minimum Delay
#***************************************************************************

#***************************************************************************
# Set Input Transition
#***************************************************************************

#***************************************************************************
# Set Load
#***************************************************************************

#*** EOF ***
