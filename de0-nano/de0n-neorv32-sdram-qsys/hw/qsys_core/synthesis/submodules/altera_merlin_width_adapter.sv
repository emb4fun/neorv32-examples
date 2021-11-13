// (C) 2001-2012 Altera Corporation. All rights reserved.
// Your use of Altera Corporation's design tools, logic functions and other 
// software and tools, and its AMPP partner logic functions, and any output 
// files any of the foregoing (including device programming or simulation 
// files), and any associated documentation or information are expressly subject 
// to the terms and conditions of the Altera Program License Subscription 
// Agreement, Altera MegaCore Function License Agreement, or other applicable 
// license agreement, including, without limitation, that your use is for the 
// sole purpose of programming logic devices manufactured by Altera and sold by 
// Altera or its authorized distributors.  Please refer to the applicable 
// agreement for further details.


// $Id: //acds/rel/11.1sp2/ip/merlin/altera_merlin_width_adapter/altera_merlin_width_adapter.sv#1 $
// $Revision: #1 $
// $Date: 2011/11/10 $
// $Author: max $

// -----------------------------------------------------
// Merlin Width Adapter
// -----------------------------------------------------

`timescale 1 ns / 1 ns

module altera_merlin_width_adapter
#(
    parameter IN_PKT_ADDR_L      = 0,
    parameter IN_PKT_ADDR_H      = 31,
    parameter IN_PKT_DATA_L      = 32,
    parameter IN_PKT_DATA_H      = 63,
    parameter IN_PKT_BYTEEN_L    = 64,
    parameter IN_PKT_BYTEEN_H    = 67,
    parameter IN_PKT_TRANS_COMPRESSED_READ = 72,
    parameter IN_PKT_BYTE_CNT_L  = 73,
    parameter IN_PKT_BYTE_CNT_H  = 77,
    parameter IN_PKT_BURSTWRAP_L = 78,
    parameter IN_PKT_BURSTWRAP_H = 82,
    parameter IN_ST_DATA_W       = 110,

    parameter OUT_PKT_ADDR_L     = 0,
    parameter OUT_PKT_ADDR_H     = 31,
    parameter OUT_PKT_DATA_L     = 32,
    parameter OUT_PKT_DATA_H     = 47,
    parameter OUT_PKT_BYTEEN_L   = 48,
    parameter OUT_PKT_BYTEEN_H   = 49,
    parameter OUT_PKT_TRANS_COMPRESSED_READ = 54,
    parameter OUT_PKT_BYTE_CNT_L = 55,
    parameter OUT_PKT_BYTE_CNT_H = 59,
    parameter OUT_ST_DATA_W      = 92,

    parameter ST_CHANNEL_W       = 32,
    parameter OPTIMIZE_FOR_RSP   = 0
)
( 
    input                            clk,
    input                            reset,
    output reg                       in_ready,
    input                            in_valid,
    input      [ST_CHANNEL_W-1:0]    in_channel,
    input      [IN_ST_DATA_W-1:0]    in_data,
    input                            in_startofpacket,
    input                            in_endofpacket,
    input                            out_ready,
    output reg                       out_valid,
    output reg [ST_CHANNEL_W-1:0]    out_channel,
    output reg [OUT_ST_DATA_W-1:0]   out_data,
    output reg                       out_startofpacket,
    output reg                       out_endofpacket
);

    // ------------------------------------------------------------
    // Utility Functions
    // ------------------------------------------------------------
    function integer clogb2;
        input [31:0] value;
        begin
            for (clogb2=0; value>0; clogb2=clogb2+1)
                value = value >> 1;
            clogb2 = clogb2 - 1;
        end
    endfunction // clogb2

    function integer min;
        input [31:0] a;
        input [31:0] b;
        begin
            return (a < b) ? a : b;
        end
    endfunction

    function integer max;
        input [31:0] a;
        input [31:0] b;
        begin
            return (a > b) ? a : b;
        end
    endfunction
  
    // ------------------------------------------------------------
    // Local Parameters
    // ------------------------------------------------------------
    localparam IN_NUMSYMBOLS   = IN_PKT_BYTEEN_H - IN_PKT_BYTEEN_L + 1;
    localparam IN_DATA_W       = IN_PKT_DATA_H   - IN_PKT_DATA_L   + 1;
    localparam IN_BYTEEN_W     = IN_NUMSYMBOLS;

    localparam OUT_NUMSYMBOLS  = OUT_PKT_BYTEEN_H - OUT_PKT_BYTEEN_L + 1;
    localparam OUT_DATA_W      = OUT_PKT_DATA_H   - OUT_PKT_DATA_L   + 1;
    localparam OUT_BYTEEN_W    = OUT_NUMSYMBOLS;

    localparam SYMBOL_W        = IN_DATA_W / IN_NUMSYMBOLS;
    localparam ADDRESS_W       = IN_PKT_ADDR_H     - IN_PKT_ADDR_L     + 1;
    localparam BYTE_CNT_W      = IN_PKT_BYTE_CNT_H - IN_PKT_BYTE_CNT_L + 1;
    localparam BWRAP_W         = IN_PKT_BURSTWRAP_H - IN_PKT_BURSTWRAP_L + 1;

    localparam RATIO           = (IN_NUMSYMBOLS > OUT_NUMSYMBOLS ? 
                                    IN_NUMSYMBOLS / OUT_NUMSYMBOLS : 
                                    OUT_NUMSYMBOLS / IN_NUMSYMBOLS );
    localparam WIDE_NUMSYMBOLS = (IN_NUMSYMBOLS > OUT_NUMSYMBOLS ? 
                                    IN_NUMSYMBOLS : OUT_NUMSYMBOLS );
    localparam WIDE_DATA       = (IN_NUMSYMBOLS > OUT_NUMSYMBOLS ? 
                                    IN_DATA_W - (OUT_NUMSYMBOLS*SYMBOL_W) : 
                                    OUT_DATA_W - (IN_NUMSYMBOLS*SYMBOL_W));
    localparam OUT_SEGMENT_W   = OUT_NUMSYMBOLS * SYMBOL_W;
  
    localparam NW_BITFORSELECT_R = clogb2(IN_NUMSYMBOLS);
    localparam NW_BITFORSELECT_L = clogb2(OUT_NUMSYMBOLS) - 1;
    localparam ALIGNED_BITS_L    = clogb2(OUT_NUMSYMBOLS) - 1;
    localparam WN_ADDR_LSBS      = clogb2(IN_NUMSYMBOLS);

    // ------------------------------------------------------------
    // Pseudo-field Parameters
    //
    // The width adapter widens the data and byteenable fields in the 
    // output packet, thus changing the output packet format. By using 
    // pseudo-fields, we can avoid remapping each individual field to 
    // the output, which is a non-scalable solution. 
    //
    // How? Assume the packet format is { FIRST, byteen, MID, data, LAST },
    // where byteen and data positions are interchangeable. FIRST, MID and
    // LAST are pseudo-fields that represent the collection of fields in
    // those positions.
    // 
    // Not all the pseudo-fields may exist for a given packet format. A
    // non-existent field has reversed indices, so we have to be careful 
    // when using them.
    // ------------------------------------------------------------

    localparam IN_FIRST_L = 0,
               IN_FIRST_H = min(IN_PKT_BYTEEN_L, IN_PKT_DATA_L) - 1,
               IN_MID_L   = min(IN_PKT_DATA_H, IN_PKT_BYTEEN_H) + 1,
               IN_MID_H   = max(IN_PKT_DATA_L, IN_PKT_BYTEEN_L) - 1,
               IN_LAST_L  = max(IN_PKT_BYTEEN_H, IN_PKT_DATA_H) + 1,
               IN_LAST_H  = IN_ST_DATA_W - 1,
 
               FIRST_EXISTS = (IN_FIRST_H >= IN_FIRST_L),
               MID_EXISTS   = (IN_MID_H   >= IN_MID_L),
               LAST_EXISTS  = (IN_LAST_H  >= IN_LAST_L),

               FIRST_W      = IN_FIRST_H - IN_FIRST_L + 1,
               MID_W        = IN_MID_H   - IN_MID_L   + 1,
               LAST_W       = IN_LAST_H  - IN_LAST_L  + 1,

               // -------------------------------------------------
               // We cannot split the output map into generate blocks as we
               // do for the inputs because address and size are mapped over
               // the pseudo-fields. We ensure that the indices are always
               // legal, even if the field is unused later on.

               OUT_FIRST_L = 0,
               OUT_FIRST_H = FIRST_EXISTS ? 
                                min(OUT_PKT_BYTEEN_L, OUT_PKT_DATA_L) - 1 :
                                OUT_FIRST_L,
               OUT_MID_L   = min(OUT_PKT_DATA_H, OUT_PKT_BYTEEN_H) + 1,
               OUT_MID_H   = MID_EXISTS ? 
                                max(OUT_PKT_DATA_L, OUT_PKT_BYTEEN_L) - 1 : 
                                OUT_MID_L,
               OUT_LAST_L  = max(OUT_PKT_BYTEEN_H, OUT_PKT_DATA_H) + 1,
               OUT_LAST_H  = LAST_EXISTS ? 
                                OUT_ST_DATA_W - 1 :
                                OUT_LAST_L;
  
    // ------------------------------------------------------------
    // Signals
    // ------------------------------------------------------------
    reg [IN_DATA_W-1:0]        in_data_field; 
    reg [IN_BYTEEN_W-1:0]      in_byteen_field;
    reg [ADDRESS_W-1:0]        in_address_field;
    reg [BYTE_CNT_W-1:0]       in_byte_cnt_field;
    reg [BWRAP_W-1:0]          in_burstwrap_field;
    reg                        in_cmpr_read;
    reg [BYTE_CNT_W-1:0]       quantized_byte_cnt_field;

    reg [OUT_DATA_W-1:0]       out_data_field;
    reg [OUT_BYTEEN_W-1:0]     out_byteen_field;
    reg [ADDRESS_W-1:0]        out_address_field;
    reg                        out_cmpr_read;
    reg [BYTE_CNT_W-1:0]       out_byte_cnt_field;

    reg [FIRST_W-1:0]          in_first_field;
    reg [FIRST_W-1:0]          out_first_field;
    reg [MID_W-1:0]            in_mid_field;
    reg [MID_W-1:0]            out_mid_field;
    reg [LAST_W-1:0]           in_last_field;
    reg [LAST_W-1:0]           out_last_field;
    
    reg [WIDE_DATA-1:0]        data_reg;
    reg [WIDE_NUMSYMBOLS-1:0]  byteen_reg;
    reg [ADDRESS_W-1:0]        address_reg;
    reg [BYTE_CNT_W-1:0]       byte_cnt_reg;
    reg                        use_reg;
    reg                        startofpacket_reg;
    reg 			           endofpacket_reg;
    reg [OUT_SEGMENT_W-1:0]    mask;

    // In narrow-to-wide adaptation, each input datum/byteenable bit maps to
    // one of OUT_NUMSYMBOLS/IN_NUMSYMBOLS subfields in the wider output
    // packet. (Call these subfields "segments".)  A subfield of the input
    // address, in_bitforselect, selects the segment. Examples:
    // 8-16 adaptation:  in_bitforselect = in_address_field[0]
    // 8-32 adaptation:  in_bitforselect = in_address_field[1:0]
    // 8-64 adaptation:  in_bitforselect = in_address_field[2:0]
    // 16-32 adaptation: in_bitforselect = in_address_field[1]
    // 32-64 adaptation: in_bitforselect = in_address_field[2]

    // The width of in_bitforselect is 
    //  log2(OUT_NUM_SYMBOLS) - log2(IN_NUM_SYMBOLS) =
    //  log2(RATIO)
    
    // The msb of in_bitforselect is driven by
    //   in_adress_field[log2(OUT_NUMSYMBOLS) - 1]
    // The lsb of in_adress_field is driven by
    //   in_adress_field[log2(IN_NUMSYMBOLS)]


    reg [clogb2(RATIO)-1:0] in_bitforselect;
    integer i, j;
   
    // ----------------------------------------
    // Input Field Mapping
    // ----------------------------------------
    always @* begin
        in_data_field      = in_data[IN_PKT_DATA_H    :IN_PKT_DATA_L    ];
        in_byteen_field    = in_data[IN_PKT_BYTEEN_H  :IN_PKT_BYTEEN_L  ];
        in_address_field   = in_data[IN_PKT_ADDR_H    :IN_PKT_ADDR_L    ];
        in_byte_cnt_field  = in_data[IN_PKT_BYTE_CNT_H:IN_PKT_BYTE_CNT_L];
        in_cmpr_read       = in_data[IN_PKT_TRANS_COMPRESSED_READ];
    end

    generate begin
        if (FIRST_EXISTS) begin
            always @* begin
                in_first_field = in_data[IN_FIRST_H:IN_FIRST_L];
            end
        end else begin
            always @* begin
                in_first_field = '0;
            end
        end
        if (MID_EXISTS) begin
            always @* begin
                in_mid_field = in_data[IN_MID_H:IN_MID_L];
            end
        end else begin
            always @* begin
                in_mid_field = '0;
            end
        end
        if (LAST_EXISTS) begin
            always @* begin
                in_last_field = in_data[IN_LAST_H:IN_LAST_L];
            end
        end
    end
    endgenerate

   generate
      
      //-------------------------------------------------------
      //-------------------------------------------------------
      // Wide-to-Narrow
      //
      // For every input cycle, we drive out a bunch'o'output 
      // cycles.  Nothing fancier.  Yes, it could be more
      // optimal, but we'll leave that for another day.
      //-------------------------------------------------------
      //-------------------------------------------------------
      if (IN_NUMSYMBOLS > OUT_NUMSYMBOLS)  begin
         // Below mess is just to avoid Quartus warnings about
         // mis-sized assignments.
         wire [31:0] int_out_numsymbols = OUT_NUMSYMBOLS;
         wire [clogb2(OUT_NUMSYMBOLS):0] sized_out_numsymbols = 
           int_out_numsymbols[clogb2(OUT_NUMSYMBOLS):0];
         wire [31:0] int_ratio_minus_1 = RATIO-1;
         wire [clogb2(RATIO)-1:0] sized_ratio_minus_1 = 
           int_ratio_minus_1[clogb2(RATIO)-1:0];
 
         reg [clogb2(RATIO)-1:0] count;

         always @(posedge clk, posedge reset) begin
            if (reset) begin
               
               data_reg     <= '0; 
               byteen_reg   <= '0;
               address_reg  <= '0;
               byte_cnt_reg <= '0;
               count        <= '0;
               use_reg      <= '0;
               endofpacket_reg	<= '0;
            end else begin
               
               // If we're not working on a wide datum, 
               // then wait until one arrives.
               if (~use_reg) begin

                  // Capture the input data we need (all but least-significant 
                  // narrow data segment).
                  data_reg    <=  in_data_field[IN_DATA_W-1:OUT_NUMSYMBOLS*SYMBOL_W];
                  byteen_reg  <= in_byteen_field >> OUT_NUMSYMBOLS;
                  address_reg[ADDRESS_W - 1 : WN_ADDR_LSBS] <=
                    in_address_field[ADDRESS_W - 1 : WN_ADDR_LSBS];
                  address_reg[WN_ADDR_LSBS - 1 : 0] <= sized_out_numsymbols;

                  byte_cnt_reg <= in_byte_cnt_field - sized_out_numsymbols;
                  endofpacket_reg <= in_endofpacket;
		  
                  if (in_valid && out_ready && !in_cmpr_read) begin
                     // Data has arrived!
                     count   <= sized_ratio_minus_1;
                     use_reg <= 1'b1;
                  end
                  
               end else begin // if (count == 0)
                  // We have a wide datum in progress.  Just wait until 
                  // the previous datum is taken, and then set 
                  // up the next transfer.
                  if (out_ready) begin
                     data_reg     <= data_reg    >> (OUT_NUMSYMBOLS * SYMBOL_W);
                     byteen_reg   <= byteen_reg  >> (OUT_NUMSYMBOLS);
                     address_reg[ADDRESS_W - 1 : WN_ADDR_LSBS] <=
                       in_address_field[ADDRESS_W - 1 : WN_ADDR_LSBS];
                     address_reg[WN_ADDR_LSBS - 1 : 0] <=
                       address_reg[WN_ADDR_LSBS - 1 : 0] + sized_out_numsymbols;

                     byte_cnt_reg <= byte_cnt_reg - sized_out_numsymbols;
                     
                     count <= count - 1'b1;
                     
                     if (count == 1'b1)
                       // We're at the end of this word.
                       use_reg <= '0;

                  end // if (out_ready)
               end // else: !if(count == 0)
            end // if (posedge clk)
         end // always @ (clk, reset)

   
         always @* begin
	        // Calculate in_ready.
            // If count is 0, then we don't have data underway, and we 
            // definitely won't be ready for it the first time 'round.
            // If count is '1', then we're finishing a set, and we're 
            //   ready if the output is.
            // If count > 1, then we're mid set, and certainly 
            //   not ready.
            in_ready = 0;
            if (count == 1 || in_cmpr_read)
               in_ready = out_ready;
	    
            out_valid         = in_valid;
            out_channel       = in_channel;
            out_startofpacket = in_startofpacket;
            out_endofpacket   = 0;
            if (in_cmpr_read)
                out_endofpacket = 1;
            
            out_byteen_field   = in_byteen_field[OUT_NUMSYMBOLS-1:0];
            out_data_field     = in_data_field[OUT_NUMSYMBOLS * SYMBOL_W-1:0];
            out_address_field  = in_address_field;
            out_byte_cnt_field = in_byte_cnt_field;
            out_first_field    = in_first_field;
            out_mid_field      = in_mid_field;
            out_last_field     = in_last_field;
            out_cmpr_read      = in_cmpr_read;
            
            if (use_reg) begin

               out_startofpacket = 0;
               // If it's the Last cycle, or if there's no more data, 
               // we can allow an endofpacket.

               if ((count==1)) 
                  out_endofpacket = endofpacket_reg;
               
               out_data_field     = data_reg[(OUT_NUMSYMBOLS * SYMBOL_W)-1:0];
               out_byteen_field   = byteen_reg[OUT_NUMSYMBOLS-1:0];
               out_address_field  = address_reg;
               out_byte_cnt_field = byte_cnt_reg;
            end

            //-----------------------------------------
            // Optimization for non-bursting wide-narrow response.
            //
            // Only one segment of the wide word will have asserted
            // byteenables. Just pass that segment through and drop
            // the rest. This should synthesize to an and-or mux.
            //-----------------------------------------
            if (OPTIMIZE_FOR_RSP) begin
                out_startofpacket  = in_startofpacket;
                out_endofpacket    = in_endofpacket;
                in_ready           = out_ready;
                //-----------------------------------------
                // Not correct, but nothing in the response path looks
                // at these today (10.1). Must be corrected when we allow
                // multiple width adapters on a path.
                //-----------------------------------------
                out_address_field  = in_address_field;
                out_byte_cnt_field = in_byte_cnt_field;

                out_data_field   = '0;
                out_byteen_field = '0;
                for (i = 0; i < RATIO; i=i+1) begin
                    mask = '0;
                    for (j = 0; j < OUT_NUMSYMBOLS; j=j+1) begin
                        mask |= {SYMBOL_W{in_byteen_field[i*OUT_NUMSYMBOLS+j]}} << (j*SYMBOL_W);
                    end
    
                    out_data_field |= mask & in_data_field[i*OUT_SEGMENT_W +: OUT_SEGMENT_W];
                    out_byteen_field |= in_byteen_field[i*OUT_NUMSYMBOLS +: OUT_NUMSYMBOLS];
                end
            end

         end // always @ *
         
         //-------------------------------------------------------
         // Configuration Error Checking
         //-------------------------------------------------------
         // synthesis translate_off
         initial begin
            if (RATIO * OUT_NUMSYMBOLS != IN_NUMSYMBOLS) begin
               $display("%m : The ratio of input symbols to output symbols must be an integer.");
               $stop();
            end
         end
         // synthesis translate_on
         
      end // if (IN_NUMSYMBOLS > OUT_NUMSYMBOLS)

      
      //-------------------------------------------------------
      //-------------------------------------------------------
      // Narrow-to-Wide
      //-------------------------------------------------------
      //-------------------------------------------------------
      if (OUT_NUMSYMBOLS > IN_NUMSYMBOLS)  begin
         wire                    p0_valid;
         reg                     p0_startofpacket;
         reg                     p0_endofpacket;
         reg [IN_DATA_W-1:0]     p0_data_field; 
         reg [IN_BYTEEN_W-1:0]   p0_byteen_field;
         reg [ADDRESS_W-1:0]     p0_address_field;
         reg [BWRAP_W-1:0]       p0_bwrap_field;
         reg [BYTE_CNT_W-1:0]    p0_byte_cnt_field;
         reg [clogb2(RATIO)-1:0] p0_bitforselect;
         reg                     p0_cmpr_read;
         reg [FIRST_W-1:0]       p0_first_field;
         reg [MID_W-1:0]         p0_mid_field;
         reg [LAST_W-1:0]        p0_last_field;
         reg                     p0_use_reg;
         reg [ST_CHANNEL_W-1:0]  p0_channel;

         reg                     p0_reg_startofpacket;
         reg                     p0_reg_endofpacket;
         reg [IN_DATA_W-1:0]     p0_reg_data_field; 
         reg [IN_BYTEEN_W-1:0]   p0_reg_byteen_field;
         reg [ADDRESS_W-1:0]     p0_reg_address_field;
         reg [BWRAP_W-1:0]       p0_reg_bwrap_field;
         reg [BYTE_CNT_W-1:0]    p0_reg_byte_cnt_field;
         reg [clogb2(RATIO)-1:0] p0_reg_bitforselect;
         reg                     p0_reg_cmpr_read;
         reg [FIRST_W-1:0]       p0_reg_first_field;
         reg [MID_W-1:0]         p0_reg_mid_field;
         reg [LAST_W-1:0]        p0_reg_last_field;
         reg [ST_CHANNEL_W-1:0]  p0_reg_channel;

         wire                    p1_valid;
         reg                     p1_ready;
         reg                     p1_startofpacket;
         reg                     p1_endofpacket;
         reg [IN_DATA_W-1:0]     p1_data_field; 
         reg [IN_BYTEEN_W-1:0]   p1_byteen_field;
         reg [ADDRESS_W-1:0]     p1_address_field;
         reg [BYTE_CNT_W-1:0]    p1_byte_cnt_field;
         reg [clogb2(RATIO)-1:0] p1_bitforselect;
         reg                     p1_cmpr_read;

         reg                     unc_sink_valid;
         wire                    unc_sink_ready;
         wire                    unc_src_startofpacket;
         wire                    unc_src_endofpacket;
         wire                    unc_src_valid;
         wire [ADDRESS_W-1:0]    unc_src_addr;
         wire [BYTE_CNT_W-1:0]   unc_src_byte_cnt;

         wire                    aligned_addr;
         wire                    aligned_byte_cnt;
         wire                    unaligned_read;

         reg [BYTE_CNT_W-1:0]       count;
         reg                        count_eq_zero;

         wire [31:0] int_in_numsymbols = IN_NUMSYMBOLS;
         wire [BYTE_CNT_W-1:0] byte_cnt_sized_in_num_symbols = 
            int_in_numsymbols[BYTE_CNT_W-1:0];

         always @* begin
            in_burstwrap_field = in_data[IN_PKT_BURSTWRAP_H:IN_PKT_BURSTWRAP_L];
         end

         // --------------------------------------------
         // Stage 0: buffer the input cycle if read burst 
         // uncompression is going to happen.
         //
         // This avoids the possibility of a master receiving a premature
         // response while its read burst is still being waitrequested.
         // --------------------------------------------
         always @(posedge clk, posedge reset) begin
            if (reset) begin
               p0_use_reg            <= 1'b0;
               p0_reg_startofpacket  <= 1'b0;
               p0_reg_endofpacket    <= 1'b0;
               p0_reg_data_field     <= '0;
               p0_reg_bwrap_field    <= '0;
               p0_reg_byteen_field   <= '0;
               p0_reg_address_field  <= '0;
               p0_reg_byte_cnt_field <= '0;
               p0_reg_cmpr_read      <= 1'b0;
               p0_reg_first_field    <= '0;
               p0_reg_mid_field      <= '0;
               p0_reg_last_field     <= '0;
               p0_reg_channel        <= '0;
            end else begin
               if (unaligned_read & in_valid)
                  p0_use_reg <= 1'b1;
               if (unc_src_endofpacket & p1_ready)
                  p0_use_reg <= 1'b0;

               if (unaligned_read) begin
                  p0_reg_startofpacket  <= p0_startofpacket;
                  p0_reg_endofpacket    <= p0_endofpacket;   
                  p0_reg_data_field     <= p0_data_field;    
                  p0_reg_bwrap_field    <= p0_bwrap_field;
                  p0_reg_byteen_field   <= p0_byteen_field;  
                  p0_reg_address_field  <= p0_address_field; 
                  p0_reg_byte_cnt_field <= p0_byte_cnt_field;
                  p0_reg_cmpr_read      <= p0_cmpr_read;     
                  p0_reg_first_field    <= p0_first_field;   
                  p0_reg_mid_field      <= p0_mid_field;     
                  p0_reg_last_field     <= p0_last_field;    
                  p0_reg_channel        <= p0_channel;
               end
            end
         end

         always @* begin
            in_ready = p1_ready;
   
            // accept on the first cycle 
            if (unaligned_read & in_valid & ~p0_use_reg)
                in_ready = 1;

            if (p0_use_reg)
                in_ready = 0;
         end

         always @* begin
            p0_startofpacket  = in_startofpacket;
            p0_endofpacket    = in_endofpacket;
            p0_data_field     = in_data_field;  
            p0_bwrap_field    = in_burstwrap_field;
            p0_byteen_field   = in_byteen_field;
            p0_address_field  = in_address_field;  
            p0_byte_cnt_field = in_byte_cnt_field;
            p0_cmpr_read      = in_cmpr_read;     
            p0_first_field    = in_first_field;   
            p0_mid_field      = in_mid_field;     
            p0_last_field     = in_last_field;
            p0_channel        = in_channel;

            if (p0_use_reg) begin
               p0_startofpacket  = p0_reg_startofpacket;
               p0_endofpacket    = p0_reg_endofpacket;
               p0_data_field     = p0_reg_data_field;  
               p0_bwrap_field    = p0_reg_bwrap_field;
               p0_byteen_field   = p0_reg_byteen_field;
               p0_address_field  = p0_reg_address_field;  
               p0_byte_cnt_field = p0_reg_byte_cnt_field;
               p0_cmpr_read      = p0_reg_cmpr_read;     
               p0_first_field    = p0_reg_first_field;   
               p0_mid_field      = p0_reg_mid_field;     
               p0_last_field     = p0_reg_last_field;
               p0_channel        = p0_reg_channel;
            end
         end

         assign p0_valid = in_valid | p0_use_reg;

         // --------------------------------------------
         // Stage 1: uncompress the input packet if necessary
         // --------------------------------------------
         assign p1_valid = (unaligned_read) ? unc_src_valid : p0_valid;
         assign aligned_addr     = (p0_address_field[ALIGNED_BITS_L:0] == 0);
         assign aligned_byte_cnt = (p0_byte_cnt_field[ALIGNED_BITS_L:0] == 0);
         assign unaligned_read   = p0_cmpr_read & (~aligned_addr || ~aligned_byte_cnt);

         always @* begin
            p1_data_field     = p0_data_field;
            p1_byteen_field   = p0_byteen_field;
            p1_startofpacket  = p0_startofpacket;
            p1_endofpacket    = p0_endofpacket;
            p1_address_field  = p0_address_field;
            p1_byte_cnt_field = p0_byte_cnt_field;
            p1_cmpr_read      = p0_cmpr_read;
            
            unc_sink_valid    = 0;

            if (unaligned_read) begin
               unc_sink_valid    = p0_valid;

               p1_startofpacket  = unc_src_startofpacket;
               p1_endofpacket    = unc_src_endofpacket;
               p1_address_field  = unc_src_addr;
               p1_byte_cnt_field = unc_src_byte_cnt;
               p1_cmpr_read      = 0;
            end
         end

         altera_merlin_burst_uncompressor
         #(
            .ADDR_W      (ADDRESS_W),
            .BURSTWRAP_W (BWRAP_W),
            .BYTE_CNT_W  (BYTE_CNT_W),
            .PKT_SYMBOLS (IN_NUMSYMBOLS)
         ) uncompressor (
            .clk                  (clk),
            .reset                (reset),

            .sink_startofpacket   (p0_startofpacket),
            .sink_endofpacket     (p0_endofpacket),
            .sink_valid           (unc_sink_valid),
            .sink_ready           (unc_sink_ready),
            .sink_addr            (p0_address_field),
            .sink_burstwrap       (p0_bwrap_field),
            .sink_byte_cnt        (p0_byte_cnt_field),
            .sink_is_compressed   (1'b1),   // should always be compressed

            .source_startofpacket (unc_src_startofpacket),
            .source_endofpacket   (unc_src_endofpacket),
            .source_valid         (unc_src_valid),
            .source_ready         (p1_ready),
            .source_addr          (unc_src_addr),
            .source_burstwrap     (),
            .source_byte_cnt      (unc_src_byte_cnt),

            .source_is_compressed ()
         );

         // --------------------------------------------
         // Stage 2: perform narrow to wide adaptation on the beats
         // --------------------------------------------
         always @(posedge clk, posedge reset) begin
            if (reset) begin
               data_reg          <= '0;
               byteen_reg        <= '0;
               startofpacket_reg <= '0;
               count             <= '0;
               count_eq_zero     <= '1;
            end else begin
               
               if (p1_valid && (out_ready || ~out_valid)) begin
                  // Lay input data & input byte enables into 
                  // the temp registers
                  data_reg     <= data_reg   | (p1_data_field   << (p1_bitforselect *IN_NUMSYMBOLS*SYMBOL_W));
                  byteen_reg   <= byteen_reg | (p1_byteen_field << (p1_bitforselect *IN_NUMSYMBOLS));

                  // Capture the stuff that's to be held constant
                  if (count_eq_zero) begin
                     startofpacket_reg <= p1_startofpacket;
                     if (~p1_endofpacket) begin
                        count <= p1_byte_cnt_field - byte_cnt_sized_in_num_symbols;
                        count_eq_zero <=
                          ~|(p1_byte_cnt_field - byte_cnt_sized_in_num_symbols);
                     end
                  end else begin
                     count <= count - byte_cnt_sized_in_num_symbols;
                     count_eq_zero <= ~|(count - byte_cnt_sized_in_num_symbols);
                  end

                  if (p1_endofpacket || (p1_bitforselect == '1)) begin
                     data_reg     <= '0;
                     byteen_reg   <= '0;
                  end

                  if (out_valid && out_ready) begin
                     startofpacket_reg <= '0;
                  end
                  
               end // if (p1_valid && (out_ready || ~out_valid))
            end // if (posedge clk)
         end // always @ (clk, reset)
   
         always @* begin
            p1_bitforselect = p1_address_field[NW_BITFORSELECT_L:NW_BITFORSELECT_R];
            
            // We push data to the output whenever the input is 
            // an endofpacket, or the input drives the most-significant
            // segment of the wider output word.
            out_valid = 0;
            if (p1_endofpacket || (p1_bitforselect == '1))
               out_valid = p1_valid;

            out_startofpacket = p1_startofpacket || startofpacket_reg;
            out_endofpacket   = p1_endofpacket;
            out_data_field    = data_reg   | (p1_data_field << (p1_bitforselect *IN_NUMSYMBOLS*SYMBOL_W));

            // Compressed read with byte_cnt > input interface width: 
            // this is a read burst spanning more than the originating
            // interface of data, so all byteenables must be asserted.
            if (p1_cmpr_read && (p1_byte_cnt_field > IN_NUMSYMBOLS)) begin
               out_byteen_field = '1;
            end else begin
               out_byteen_field = byteen_reg |
                    (p1_byteen_field << (p1_bitforselect*IN_NUMSYMBOLS));
            end

            out_byte_cnt_field = quantized_byte_cnt_field;
            out_cmpr_read      = p1_cmpr_read;
            
            out_address_field = p1_address_field;
            out_address_field[NW_BITFORSELECT_L:0] = 0;

            // nothing touches these fields, so assign them
            // directly from the input fields
            out_first_field   = p0_first_field;
            out_mid_field     = p0_mid_field;
            out_last_field    = p0_last_field;

            out_channel       = p0_channel;
         end // always @ *

        //-------------------------------------------------------
        // output byte_cnt, rounded up to alignment with the output-side 
        // address map footprint implied by the input-side access.
        //
        // See "option 3" in Appendix C of
        // merlin_interconnect_architecture_fd_91.doc.
        //-------------------------------------------------------
        reg [NW_BITFORSELECT_L:0] low_addr_bits;

        always @* begin
           low_addr_bits = p1_address_field[NW_BITFORSELECT_L:0];
           
           quantized_byte_cnt_field = low_addr_bits + 
               p1_byte_cnt_field + 
               {clogb2(OUT_NUMSYMBOLS){1'b1}};
           quantized_byte_cnt_field[NW_BITFORSELECT_L:0] = '0;
        end

         //-------------------------------------------------------
         // Backpressure
         //-------------------------------------------------------
         always @ * begin
            p1_ready = out_ready || ~out_valid;
         end
   
      end // if (OUT_NUMSYMBOLS > IN_NUMSYMBOLS)

      //-------------------------------------------------------
      //-------------------------------------------------------
      // Same Width.  Seems kind of silly, but let's be complete.
      //-------------------------------------------------------
      //-------------------------------------------------------
      if (OUT_NUMSYMBOLS == IN_NUMSYMBOLS)  begin
   
         always @* begin
            in_ready           = out_ready;
            out_valid          = in_valid;
            out_channel        = in_channel;
            out_startofpacket  = in_startofpacket;
            out_endofpacket    = in_endofpacket;
            out_data_field     = in_data_field;
            out_byteen_field   = in_byteen_field;
            out_address_field  = in_address_field;
            out_byte_cnt_field = in_byte_cnt_field;
            out_cmpr_read      = in_cmpr_read;
            out_first_field    = in_first_field;
            out_mid_field      = in_mid_field;
            out_last_field     = in_last_field;
         end // always @ *
   
      end // if (OUT_NUMSYMBOLS == IN_NUMSYMBOLS)

   endgenerate

   // ---------------------------------------
   // Output Field Mapping
   //
   // Conditionally assign the pseudo-fields. Assign address and size 
   // last, because they partly override the pseudo-fields.
   // ---------------------------------------
   always @* begin
      if (FIRST_EXISTS)
          out_data[OUT_FIRST_H:OUT_FIRST_L] = out_first_field;
      if (MID_EXISTS)
          out_data[OUT_MID_H:OUT_MID_L]     = out_mid_field;
      if (LAST_EXISTS)
          out_data[OUT_LAST_H:OUT_LAST_L]   = out_last_field;

      out_data[OUT_PKT_DATA_H    :OUT_PKT_DATA_L    ]  = out_data_field;
      out_data[OUT_PKT_BYTEEN_H  :OUT_PKT_BYTEEN_L  ]  = out_byteen_field;
      out_data[OUT_PKT_ADDR_H    :OUT_PKT_ADDR_L    ]  = out_address_field;
      out_data[OUT_PKT_BYTE_CNT_H:OUT_PKT_BYTE_CNT_L]  = out_byte_cnt_field;
      out_data[OUT_PKT_TRANS_COMPRESSED_READ        ]  = out_cmpr_read;
   end // always @ *


endmodule // width_adapter

