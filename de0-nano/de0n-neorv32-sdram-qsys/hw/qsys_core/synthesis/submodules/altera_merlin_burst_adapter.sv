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


// $Id: //acds/rel/11.1sp2/ip/merlin/altera_merlin_burst_adapter/altera_merlin_burst_adapter.sv#1 $
// $Revision: #1 $
// $Date: 2011/11/10 $
// $Author: max $

// -------------------------------------------------------
// Merlin Burst Adapter
// -------------------------------------------------------

`timescale 1 ns / 1 ns

// <burstwrap value> + 1
// By definition, burstwrap values are of the form 2^n - 1; adding 1 is a non-ripple operation.
module altera_merlin_burst_adapter_burstwrap_increment #(parameter WIDTH = 8)
  (
    input [WIDTH - 1:0] mask,
    output [WIDTH - 1:0] inc
  );
    genvar i;
    generate begin : burstwrap_increment_gen
        assign inc[0] = ~mask[0];
        for (i = 1; i < WIDTH; i = i+1) begin : burstwrap_increment_loop
          assign inc[i] = mask[i - 1] & ~mask[i];
        end
      end
    endgenerate
endmodule

module altera_merlin_burst_adapter_adder #(parameter WIDTH = 8) (
    input cin,
    input  [WIDTH-1 : 0] a,
    input  [WIDTH-1 : 0] b,
    output [WIDTH-1 : 0] sum
  );

  genvar i;
  generate begin : full_adder
      wire [WIDTH-1:0] carry;
      assign sum[0]  = a[0] ^ b[0] ^ cin;
      assign carry[0] = a[0] & b[0] | a[0] & cin | b[0] & cin;

      for (i = 1; i < WIDTH; i = i+1) begin : full_adder_loop
          assign sum[i] = a[i] ^ b[i] ^ carry[i-1];
          assign carry[i] = a[i] & b[i] | a[i] & carry[i-1] | b[i] & carry[i-1];
      end

  end endgenerate
endmodule

// a - b = a + ~b + 1
module altera_merlin_burst_adapter_subtractor #(parameter WIDTH = 8) (
    input  [WIDTH-1 : 0] a,
    input  [WIDTH-1 : 0] b,
    output [WIDTH-1 : 0] diff
  );

  altera_merlin_burst_adapter_adder #(.WIDTH (WIDTH)) subtract (
    .cin (1'b1),
    .a (a),
    .b (~b),
    .sum (diff)
  );
endmodule

// Pipeline position:
//   0: register module inputs
//   1: register module output
// I would have expected that with register retiming/duplication turned on, the
// pipeline position parameter would have no effect.  Not so,
// PIPELINE_POSITION=1 is significantly better than PIPELINE_POSITION=0.
module altera_merlin_burst_adapter_min #(parameter PKT_BYTE_CNT_W=8, PKT_BURSTWRAP_W=8, PIPELINE_POSITION = 1) 
  (
    input clk,
    input clken,
    input reset,
    input [PKT_BYTE_CNT_W - 1 : 0] a,
    input [PKT_BYTE_CNT_W - 1 : 0] b,
    input [PKT_BURSTWRAP_W - 1 : 0] c,
    input c_enable,
    input [PKT_BYTE_CNT_W - 1 : 0] d,
    output reg [PKT_BYTE_CNT_W - 1 : 0] result
  );

    wire [PKT_BYTE_CNT_W : 0] ab_diff;
    wire [PKT_BYTE_CNT_W : 0] ac_diff;
    wire [PKT_BYTE_CNT_W : 0] bc_diff;
    wire a_lt_b;
    wire a_lt_c;
    wire b_lt_c;

    reg [PKT_BYTE_CNT_W - 1 : 0] a_reg;
    reg [PKT_BYTE_CNT_W - 1 : 0] b_reg;
    reg [PKT_BURSTWRAP_W - 1 : 0] c_reg;
    reg c_enable_reg;
    reg [PKT_BYTE_CNT_W - 1 : 0] d_reg;

    generate
      if (PIPELINE_POSITION == 0) begin
        always_ff @(posedge clk or posedge reset) begin
          if (reset) begin
            a_reg <= '0;
            b_reg <= '0;
            c_reg <= '0;
            c_enable_reg <= '0;
            d_reg <= '0;
          end
          else if (clken) begin
            a_reg <= a;
            b_reg <= b;
            c_reg <= c;
            c_enable_reg <= c_enable;
            d_reg <= d;
          end
        end
      end
      else begin
        always @* begin
            a_reg = a;
            b_reg = b;
            c_reg = c;
            c_enable_reg = c_enable;
            d_reg = d;
        end
      end
    endgenerate

    altera_merlin_burst_adapter_subtractor #(.WIDTH (PKT_BYTE_CNT_W + 1)) ab_sub (
      .a ({1'b0, a_reg}),
      .b ({1'b0, b_reg}),
      .diff (ab_diff)
    );
    assign a_lt_b = ab_diff[PKT_BYTE_CNT_W];

    altera_merlin_burst_adapter_subtractor #(.WIDTH (PKT_BYTE_CNT_W + 1)) ac_sub (
      .a ({1'b0, a_reg}),
      .b ({{(PKT_BYTE_CNT_W - PKT_BURSTWRAP_W + 1) {1'b0}}, c_reg}),
      .diff (ac_diff)
    );
    assign a_lt_c = ac_diff[PKT_BYTE_CNT_W];

    altera_merlin_burst_adapter_subtractor #(.WIDTH (PKT_BYTE_CNT_W + 1)) bc_sub (
      .a ({1'b0, b_reg}),
      .b ({ {(PKT_BYTE_CNT_W - PKT_BURSTWRAP_W + 1) {1'b0}}, c_reg}),
      .diff (bc_diff)
    );
    assign b_lt_c = bc_diff[PKT_BYTE_CNT_W];

    // If d is greater than any of the values, it'll be greater than the min,
    // certainly.  If d is greater than the min, use d.  Of course, ignore c if
    // !c_enable.
    
    // Note: d is "number-of-symbols", of width PKT_BYTE_CNT_W. So, a constant,
    // and a power of 2 (until we support non-power-of-2 symbols/interface
    // here).
    // wire use_d = (d > a) || (d > b) || ( (d > c) && c_enable);
    // I think there's something clever I can do with masks, but my head hurts,
    // so try something simpler.
    // wire use_d = 
    //   (&(~a[PKT_BYTE_CNT_W - 1:LOG2_NUMSYMBOLS])) || 
    //   (&(~b[PKT_BYTE_CNT_W-1:LOG2_NUMSYMBOLS])) || 
    //   ((&(~c[PKT_BURSTWRAP_W-1:LOG2_NUMSYMBOLS])) && c_enable
    // );
    wire [PKT_BYTE_CNT_W : 0] da_diff;
    wire [PKT_BYTE_CNT_W : 0] db_diff;
    wire [PKT_BYTE_CNT_W : 0] dc_diff;
    wire d_gt_a;
    wire d_gt_b;
    wire d_gt_c;

    altera_merlin_burst_adapter_subtractor #(.WIDTH (PKT_BYTE_CNT_W + 1)) da_sub (
      .a ({1'b0, d_reg}),
      .b ({1'b0, a_reg}),
      .diff (da_diff)
    );
    assign d_gt_a = ~da_diff[PKT_BYTE_CNT_W];

    altera_merlin_burst_adapter_subtractor #(.WIDTH (PKT_BYTE_CNT_W + 1)) db_sub (
      .a ({1'b0, d_reg}),
      .b ({1'b0, b_reg}),
      .diff (db_diff)
    );
    assign d_gt_b = ~db_diff[PKT_BYTE_CNT_W];

    altera_merlin_burst_adapter_subtractor #(.WIDTH (PKT_BYTE_CNT_W + 1)) dc_sub (
      .a ({1'b0, d_reg}),
      .b ({ {(PKT_BYTE_CNT_W - PKT_BURSTWRAP_W + 1) {1'b0}}, c_reg}),
      .diff (dc_diff)
    );
    assign d_gt_c = ~dc_diff[PKT_BYTE_CNT_W];

    wire use_d = d_gt_a || d_gt_b || (d_gt_c && c_enable_reg);

    wire [4:0] cmp = {a_lt_b, a_lt_c, b_lt_c, c_enable_reg, use_d};

    reg [PKT_BYTE_CNT_W - 1 : 0] p1_result;
    always @(a_reg or b_reg or c_reg or d_reg or cmp) begin
      casex (cmp)
        5'b00010: p1_result = c_reg;
        5'b00110: p1_result = b_reg;
        5'b01110: p1_result = b_reg;
        5'b10010: p1_result = c_reg;
        5'b11010: p1_result = a_reg;
        5'b11110: p1_result = a_reg;

        5'b00000: p1_result = b_reg;
        5'b00100: p1_result = b_reg;
        5'b01100: p1_result = b_reg;
        5'b10000: p1_result = a_reg;
        5'b11000: p1_result = a_reg;
        5'b11100: p1_result = a_reg;

        5'b????1: p1_result = d_reg;

        default: p1_result = 'X; // don't-care
      endcase
    end

    generate
      if (PIPELINE_POSITION == 1) begin
        always_ff @(posedge clk or posedge reset) begin
          if (reset) begin
            result <= '0;
          end
          else if (clken) begin
            result <= p1_result;
          end
        end
      end
      else begin
        always @* begin
          result = p1_result;
        end
      end
    endgenerate
endmodule

module altera_merlin_burst_adapter
#(
  parameter // Merlin packet parameters
    PKT_BEGIN_BURST = 81,
    PKT_ADDR_H      = 79,
    PKT_ADDR_L      = 48,
    PKT_BYTE_CNT_H  = 5,
    PKT_BYTE_CNT_L  = 0,
    PKT_BURSTWRAP_H = 11,
    PKT_BURSTWRAP_L = 6,
    PKT_TRANS_COMPRESSED_READ = 14,
    PKT_TRANS_WRITE = 13,
    PKT_TRANS_READ  = 12,
    PKT_BYTEEN_H    = 83,
    PKT_BYTEEN_L    = 80,
    ST_DATA_W       = 84,
    ST_CHANNEL_W    = 8,

    // Component-specific parameters
    BURSTWRAP_CONST_MASK = 0,
    BURSTWRAP_CONST_VALUE = -1,
    OUT_BYTE_CNT_H  = 5,
    OUT_BURSTWRAP_H = 11,
    COMPRESSED_READ_SUPPORT= 1
)
(
    input clk,
    input reset,

    // -------------------
    // Command Sink (Input)
    // -------------------
    input                       sink0_valid,
    input  [ST_DATA_W-1 : 0]    sink0_data,
    input  [ST_CHANNEL_W-1 : 0] sink0_channel,
    input                       sink0_startofpacket,
    input                       sink0_endofpacket,
    output reg                  sink0_ready,

    // -------------------
    // Command Source (Output)
    // -------------------
    output reg                      source0_valid,
    output reg [ST_DATA_W-1    : 0] source0_data,
    output reg [ST_CHANNEL_W-1 : 0] source0_channel,
    output reg                      source0_startofpacket,
    output reg                      source0_endofpacket,
    input                           source0_ready
);
  localparam PKT_BURSTWRAP_W = PKT_BURSTWRAP_H - PKT_BURSTWRAP_L + 1;

  generate if (COMPRESSED_READ_SUPPORT == 1) begin : altera_merlin_burst_adapter_full
    altera_merlin_burst_adapter_full #(
      .PKT_BEGIN_BURST           (PKT_BEGIN_BURST),
      .PKT_ADDR_H                (PKT_ADDR_H ),
      .PKT_ADDR_L                (PKT_ADDR_L),
      .PKT_BYTE_CNT_H            (PKT_BYTE_CNT_H),
      .PKT_BYTE_CNT_L            (PKT_BYTE_CNT_L ),
      .PKT_BURSTWRAP_H           (PKT_BURSTWRAP_H),
      .PKT_BURSTWRAP_L           (PKT_BURSTWRAP_L),
      .PKT_TRANS_COMPRESSED_READ (PKT_TRANS_COMPRESSED_READ),
      .PKT_TRANS_WRITE           (PKT_TRANS_WRITE),
      .PKT_TRANS_READ            (PKT_TRANS_READ),
      .PKT_BYTEEN_H              (PKT_BYTEEN_H),
      .PKT_BYTEEN_L              (PKT_BYTEEN_L),
      .ST_DATA_W                 (ST_DATA_W),
      .ST_CHANNEL_W              (ST_CHANNEL_W),
      .BURSTWRAP_CONST_MASK      (BURSTWRAP_CONST_MASK),
      .BURSTWRAP_CONST_VALUE     (BURSTWRAP_CONST_VALUE),
      .OUT_BYTE_CNT_H            (OUT_BYTE_CNT_H),
      .OUT_BURSTWRAP_H           (OUT_BURSTWRAP_H)
    ) the_ba(
      .clk                   (clk),
      .reset                 (reset),
      .sink0_valid           (sink0_valid),
      .sink0_data            (sink0_data),
      .sink0_channel         (sink0_channel),
      .sink0_startofpacket   (sink0_startofpacket),
      .sink0_endofpacket     (sink0_endofpacket),
      .sink0_ready           (sink0_ready),
      .source0_valid         (source0_valid),
      .source0_data          (source0_data),
      .source0_channel       (source0_channel),
      .source0_startofpacket (source0_startofpacket),
      .source0_endofpacket   (source0_endofpacket),
      .source0_ready         (source0_ready)
    );
  end
  else begin : altera_merlin_burst_adapter_uncompressed_only
    altera_merlin_burst_adapter_uncompressed_only #(
      .PKT_BYTE_CNT_H            (PKT_BYTE_CNT_H),
      .PKT_BYTE_CNT_L            (PKT_BYTE_CNT_L ),
      .PKT_BYTEEN_H              (PKT_BYTEEN_H),
      .PKT_BYTEEN_L              (PKT_BYTEEN_L),
      .ST_DATA_W                 (ST_DATA_W),
      .ST_CHANNEL_W              (ST_CHANNEL_W)
    ) the_ba(
      .clk                   (clk),
      .reset                 (reset),
      .sink0_valid           (sink0_valid),
      .sink0_data            (sink0_data),
      .sink0_channel         (sink0_channel),
      .sink0_startofpacket   (sink0_startofpacket),
      .sink0_endofpacket     (sink0_endofpacket),
      .sink0_ready           (sink0_ready),
      .source0_valid         (source0_valid),
      .source0_data          (source0_data),
      .source0_channel       (source0_channel),
      .source0_startofpacket (source0_startofpacket),
      .source0_endofpacket   (source0_endofpacket),
      .source0_ready         (source0_ready)
    );
  end endgenerate 

  // synthesis translate_off
  // Check for incoming burstwrap values inconsistent with
  // BURSTWRAP_CONST_MASK.
  always @(posedge clk or posedge reset) begin
    if (~reset && sink0_valid &&
        BURSTWRAP_CONST_MASK[PKT_BURSTWRAP_W - 1:0] &
        (BURSTWRAP_CONST_VALUE[PKT_BURSTWRAP_W - 1:0] ^ sink0_data[PKT_BURSTWRAP_H : PKT_BURSTWRAP_L])
      ) begin
      $display("%t: %m: Error: burstwrap value %X is inconsistent with BURSTWRAP_CONST_MASK value %X", $time(), sink0_data[PKT_BURSTWRAP_H : PKT_BURSTWRAP_L], BURSTWRAP_CONST_MASK[PKT_BURSTWRAP_W - 1:0]);
    end
  end
  // synthesis translate_on
endmodule

module altera_merlin_burst_adapter_uncompressed_only
#(
  parameter // Merlin packet parameters
    PKT_BYTE_CNT_H  = 5,
    PKT_BYTE_CNT_L  = 0,
    PKT_BYTEEN_H    = 83,
    PKT_BYTEEN_L    = 80,
    ST_DATA_W       = 84,
    ST_CHANNEL_W    = 8
)
(
    input clk,
    input reset,

    // -------------------
    // Command Sink (Input)
    // -------------------
    input                       sink0_valid,
    input  [ST_DATA_W-1 : 0]    sink0_data,
    input  [ST_CHANNEL_W-1 : 0] sink0_channel,
    input                       sink0_startofpacket,
    input                       sink0_endofpacket,
    output reg                  sink0_ready,

    // -------------------
    // Command Source (Output)
    // -------------------
    output reg                      source0_valid,
    output reg [ST_DATA_W-1    : 0] source0_data,
    output reg [ST_CHANNEL_W-1 : 0] source0_channel,
    output reg                      source0_startofpacket,
    output reg                      source0_endofpacket,
    input                           source0_ready
);
  localparam 
    PKT_BYTE_CNT_W    = PKT_BYTE_CNT_H - PKT_BYTE_CNT_L + 1,
    NUM_SYMBOLS       = PKT_BYTEEN_H - PKT_BYTEEN_L + 1;

  wire [PKT_BYTE_CNT_W - 1 : 0] num_symbols_sig =
    NUM_SYMBOLS[PKT_BYTE_CNT_W - 1 : 0];

  always_comb begin : source0_data_assignments
    source0_valid = sink0_valid;
    source0_channel = sink0_channel;
    source0_startofpacket = sink0_startofpacket;
    source0_endofpacket = sink0_endofpacket;
    sink0_ready = source0_ready;

    source0_data = sink0_data;
    source0_data[PKT_BYTE_CNT_H : PKT_BYTE_CNT_L] = num_symbols_sig;
  end

endmodule


module altera_merlin_burst_adapter_full
#(
  parameter // Merlin packet parameters
    PKT_BEGIN_BURST = 81,
    PKT_ADDR_H      = 79,
    PKT_ADDR_L      = 48,
    PKT_BYTE_CNT_H  = 5,
    PKT_BYTE_CNT_L  = 0,
    PKT_BURSTWRAP_H = 11,
    PKT_BURSTWRAP_L = 6,
    PKT_TRANS_COMPRESSED_READ = 14,
    PKT_TRANS_WRITE = 13,
    PKT_TRANS_READ  = 12,
    PKT_BYTEEN_H    = 83,
    PKT_BYTEEN_L    = 80,
    ST_DATA_W       = 84,
    ST_CHANNEL_W    = 8,

    // Component-specific parameters
    BURSTWRAP_CONST_MASK = 0,
    BURSTWRAP_CONST_VALUE = -1,
    OUT_BYTE_CNT_H  = 5,
    OUT_BURSTWRAP_H = 11
)
(

    input clk,
    input reset,

    // -------------------
    // Command Sink (Input)
    // -------------------
    input                       sink0_valid,
    input  [ST_DATA_W-1 : 0]    sink0_data,
    input  [ST_CHANNEL_W-1 : 0] sink0_channel,
    input                       sink0_startofpacket,
    input                       sink0_endofpacket,
    output reg                  sink0_ready,

    // -------------------
    // Command Source (Output)
    // -------------------
    output reg                      source0_valid,
    output reg [ST_DATA_W-1    : 0] source0_data,
    output reg [ST_CHANNEL_W-1 : 0] source0_channel,
    output reg                      source0_startofpacket,
    output reg                      source0_endofpacket,
    input                           source0_ready
);
  localparam 
    PKT_BYTE_CNT_W    = PKT_BYTE_CNT_H - PKT_BYTE_CNT_L + 1,
    PKT_ADDR_W        = PKT_ADDR_H - PKT_ADDR_L + 1,
    OUT_BYTE_CNT_W    = OUT_BYTE_CNT_H - PKT_BYTE_CNT_L + 1,
    OUT_BURSTWRAP_W   = OUT_BURSTWRAP_H - PKT_BURSTWRAP_L + 1,
    PKT_BURSTWRAP_W   = PKT_BURSTWRAP_H - PKT_BURSTWRAP_L + 1,
    OUT_MAX_BYTE_CNT  = 1 << (OUT_BYTE_CNT_W - 1),
    OUT_MAX_BURSTWRAP = (1 << OUT_BURSTWRAP_W) - 1,
    NUM_SYMBOLS       = PKT_BYTEEN_H - PKT_BYTEEN_L + 1;

  // "min" operation on burstwrap values is a bitwise AND.
  // Todo: one input is always set to constant OUT_MAX_BURSTWRAP; this is a
  // number of the form 2^n.  Does this fact yield an optimization?
  function [PKT_BURSTWRAP_W - 1 : 0] altera_merlin_burst_adapter_burstwrap_min(
    input [PKT_BURSTWRAP_W - 1 : 0] a, b
  );
    altera_merlin_burst_adapter_burstwrap_min = a & b;
  endfunction

  typedef enum int unsigned {
    ST_RESET,
    ST_IDLE,
    ST_WR_BEGIN_SUBBURST,
    ST_WR_CONTINUE_SUBBURST,
    ST_UNCST_RD_BEGIN_SUBBURST,
    ST_UNCRD_CONTINUE_SUBBURST,
    ST_RD_BEGIN_SUBBURST
  } t_state;
  t_state state, next_state;

  // Registered sink signals
  reg [ST_DATA_W - 1 : 0] sink0_data_reg;
  reg [ST_CHANNEL_W-1 : 0] sink0_channel_reg;
  reg [ST_CHANNEL_W-1 : 0] p1_sink0_channel_reg;

  // Handy sub-field signal versions of registered input st data.
  reg [PKT_ADDR_W - 1 : 0 ] sink0_addr_reg;
  reg [PKT_ADDR_W - 1 : 0 ] p1_sink0_addr_reg;
  reg [PKT_BYTE_CNT_W - 1 : 0] sink0_byte_cnt_reg;
  reg sink0_byte_cnt_reg_zero;
  reg [PKT_BYTE_CNT_W - 1 : 0] p1_sink0_byte_cnt_reg;
  reg [PKT_BURSTWRAP_W - 1 : 0] in_burstwrap_reg;

  // Output values.
  reg [PKT_BYTE_CNT_W - 1 : 0] source0_byte_cnt;
  reg p1_source0_valid;
  reg p1_source0_startofpacket;
  reg d1_sink0_startofpacket;
  reg d1_sink0_endofpacket;

  // burstwrap
  reg [PKT_BURSTWRAP_W - 1 : 0] in_burstwrap;
  reg [PKT_BURSTWRAP_W - 1 : 0] source0_burstwrap;
  reg [PKT_BURSTWRAP_W - 1 : 0] p1_source0_burstwrap;

  // wrap_mask extends to '1 when the input burst wrap value (in_burstwrap) is
  // equal to the output maximum burstwrap value (OUT_MAX_BURSTWRAP), to
  // prevent address-dependent wrapping.  The direct expression is:
  //   disable_wrap = (in_burstwrap == OUT_MAX_BURSTWRAP)
  // Optimization: take advantage of the fact that burstwrap values are of the form (2^n) - 1.
  wire disable_wrap;
  generate
    if (OUT_BURSTWRAP_W == 0) begin
      // Special case: 1-symbol, fixed-burst slave.
      assign disable_wrap = ~in_burstwrap[OUT_BURSTWRAP_W];
    end
    else begin
      if (OUT_BURSTWRAP_W == PKT_BURSTWRAP_W)
        assign disable_wrap = in_burstwrap[OUT_BURSTWRAP_W - 1];
      else
        assign disable_wrap = ~in_burstwrap[OUT_BURSTWRAP_W] & in_burstwrap[OUT_BURSTWRAP_W - 1];
    end
  endgenerate
  wire [PKT_BURSTWRAP_W - 1 : 0] wrap_mask = disable_wrap ? '1 : p1_source0_burstwrap;

  // wrap_boundary_distance
  // Must be valid in the cycle prior to each begin-subburst, for calculating
  // begin_subburst_byte_cnt.
  reg [PKT_BURSTWRAP_W - 1 : 0] wrap_boundary_addr;
  wire [PKT_BURSTWRAP_W - 1 : 0] incremented_wrap_mask;
  altera_merlin_burst_adapter_burstwrap_increment #(.WIDTH (PKT_BURSTWRAP_W)) the_burstwrap_increment
  (
    .mask (wrap_mask),
    .inc (incremented_wrap_mask)
  );
  wire [PKT_BURSTWRAP_W - 1 : 0] wrap_boundary_distance;
  assign wrap_boundary_distance = incremented_wrap_mask - (wrap_boundary_addr & wrap_mask);

  // in-process packet burstwrap, extended to full address width.
  reg [PKT_ADDR_W - 1 : 0 ] addr_width_burstwrap;

  // Aliases for sink0 data fields.
  wire [PKT_ADDR_W - 1 : 0 ] sink0_addr = sink0_data[PKT_ADDR_H : PKT_ADDR_L];
  // cmd              PKT_TRANS_COMPRESSED_READ  PKT_TRANS_READ PKT_TRANS_WRITE
  // read                                     0               1               0
  // compressed read                          1               1               0
  // write                                    0               0               1
  // N.b. The fabric sets both PKT_TRANS_COMPRESSED_READ and PKT_TRANS_READ 
  // bits for compressed reads
  wire sink0_compressed_read = sink0_data[PKT_TRANS_COMPRESSED_READ];
  wire sink0_write = sink0_data[PKT_TRANS_WRITE];
  wire sink0_read = sink0_data[PKT_TRANS_READ];
  wire is_uncompressed_read = 
    sink0_read & ~sink0_data[PKT_TRANS_COMPRESSED_READ];
  wire [PKT_BYTE_CNT_W - 1 : 0] sink0_byte_cnt = sink0_data[PKT_BYTE_CNT_H : PKT_BYTE_CNT_L];
  wire [PKT_BURSTWRAP_W - 1 : 0] sink0_burstwrap;

  genvar i;
  generate begin : constant_or_variable_burstwrap
    for (i = 0; i < PKT_BURSTWRAP_W; i = i + 1) begin : assign_burstwrap_bit
      if (BURSTWRAP_CONST_MASK[i]) begin
        assign sink0_burstwrap[i] = BURSTWRAP_CONST_VALUE[i];
      end
      else begin
        assign sink0_burstwrap[i] = sink0_data[PKT_BURSTWRAP_L + i];
      end
    end
  end endgenerate

  // sub-burst initial byte_cnt value
  wire [PKT_BYTE_CNT_W - 1:0] begin_subburst_byte_cnt;
  wire [PKT_BYTE_CNT_W - 1 : 0] num_symbols_sig =
    NUM_SYMBOLS[PKT_BYTE_CNT_W - 1 : 0];
  wire [PKT_BYTE_CNT_W - 1 : 0] out_max_byte_cnt_sig =
    OUT_MAX_BYTE_CNT[PKT_BYTE_CNT_W - 1 : 0];
  // bytes_remaining: bytes left to send in the current packet; counts from
  // initial bytecount down to num_symbols_sig.
  // Alternate way of thinking: bytes_remaining is computed one cycle ahead of
  // the actual downstream cycle, in order to allow begin_subburst_byte_cnt to
  // be computed with one cycle of latency.
  reg [PKT_BYTE_CNT_W - 1 : 0] bytes_remaining;
  // subburst_bytes_remaining: bytes left to send in the current subburst, not
  // counting the current cycle.  Counts from <subburst size> - num_symbols_sig to 0.
  // Used only during uncompressed bursts.
  // There may be an optimization here: subburst_bytes_remaining is just a
  // down-counter, used to determine when the current subburst has ended (when
  // it reaches 0).  1) Can I drop the log2(num_symbols_sig) LSBs of this counter?
  // 2) is it possible to compute |subburst_bytes_remaining one cycle early?
  reg [PKT_BYTE_CNT_W - 1 : 0] d1_subburst_bytes_remaining;
  reg [PKT_BYTE_CNT_W - 1 : 0] subburst_bytes_remaining;

  wire [PKT_BYTE_CNT_W - 1 : 0] begin_subburst_subburst_bytes_remaining =
    (source0_valid & source0_ready) ? (begin_subburst_byte_cnt - num_symbols_sig) :
    d1_subburst_bytes_remaining;

  wire [PKT_BYTE_CNT_W - 1 : 0] continue_subburst_subburst_bytes_remaining =
    d1_subburst_bytes_remaining - (
      (source0_valid & source0_ready) ? num_symbols_sig : '0);

  wire the_min_clken =
    (sink0_valid & sink0_ready) || ((state == ST_RD_BEGIN_SUBBURST) && source0_ready) ;

  altera_merlin_burst_adapter_min #(
    .PKT_BYTE_CNT_W (PKT_BYTE_CNT_W), 
    .PKT_BURSTWRAP_W (PKT_BURSTWRAP_W),
    .PIPELINE_POSITION (1)
    )
  the_min (
    .clk (clk),
    .clken (the_min_clken),
    .reset (reset),
    .a (bytes_remaining),
    .b (out_max_byte_cnt_sig), 
    .c (wrap_boundary_distance),
    .c_enable (~wrap_mask[PKT_BURSTWRAP_W - 1]),
    .d (num_symbols_sig),
    .result (begin_subburst_byte_cnt)
  );

  always_comb begin : state_transition
    // default assignments
    next_state = ST_IDLE;
    sink0_ready = 1'b0;
    p1_source0_valid = 1'b0;
    source0_channel = sink0_channel_reg;
    source0_byte_cnt = 'x;
    p1_source0_startofpacket = 'x;
    source0_endofpacket = 'x;
    p1_sink0_channel_reg = 'x;
    in_burstwrap = 'x;
    wrap_boundary_addr = 'x;
    bytes_remaining = 'x;
    subburst_bytes_remaining = 'x;
    p1_sink0_byte_cnt_reg = 'x;
    p1_sink0_addr_reg = 'x;
    p1_source0_burstwrap = 'x;

    case (state)
      ST_RESET: begin
        next_state = ST_IDLE;
      end 
      ST_IDLE: begin
        next_state = ST_IDLE;
        p1_sink0_addr_reg = sink0_addr;
        p1_source0_valid = sink0_valid;
        p1_source0_startofpacket = sink0_startofpacket;
        p1_sink0_byte_cnt_reg = sink0_byte_cnt;

        if (sink0_valid) begin
          if (sink0_write) begin
            next_state = ST_WR_BEGIN_SUBBURST;
          end
          if (sink0_compressed_read) begin
            next_state = ST_RD_BEGIN_SUBBURST;
            // To calculate endofpacket, must know if this burst will be
            // adapted or not.
          end
          if (is_uncompressed_read) begin
            next_state = ST_UNCST_RD_BEGIN_SUBBURST;
          end
        end

        sink0_ready = 1'b1;
        p1_sink0_channel_reg = sink0_channel;
        in_burstwrap = sink0_burstwrap;

        // input values to the_min
        wrap_boundary_addr = sink0_addr[PKT_BURSTWRAP_W - 1 : 0];
        bytes_remaining = sink0_byte_cnt;
   
        // output burstwrap
        p1_source0_burstwrap = altera_merlin_burst_adapter_burstwrap_min(OUT_MAX_BURSTWRAP, in_burstwrap);
      end

      ST_WR_BEGIN_SUBBURST: begin
        p1_sink0_addr_reg = sink0_addr;
 
        p1_source0_valid = sink0_valid;
        p1_source0_startofpacket = sink0_startofpacket;
        source0_endofpacket = d1_sink0_endofpacket;
        sink0_ready = source0_ready | ~source0_valid;
        p1_sink0_channel_reg = sink0_channel;
        p1_sink0_byte_cnt_reg = sink0_byte_cnt;

        // input values to the_min
        wrap_boundary_addr = sink0_addr[PKT_BURSTWRAP_W - 1 : 0];
        bytes_remaining = sink0_byte_cnt;

        subburst_bytes_remaining = begin_subburst_subburst_bytes_remaining;

        if (source0_endofpacket) begin
          if (sink0_valid) begin
            in_burstwrap = sink0_burstwrap;
          end
        end
        else begin
          in_burstwrap = in_burstwrap_reg;
        end
     
        if (source0_endofpacket) begin
          if (~sink0_valid) begin
            next_state = ST_IDLE;
          end
          else begin
            if (sink0_write) begin
              next_state = ST_WR_BEGIN_SUBBURST;
            end
            if (sink0_compressed_read) begin
              next_state = ST_RD_BEGIN_SUBBURST;
            end
            if (is_uncompressed_read) begin
              next_state = ST_UNCST_RD_BEGIN_SUBBURST;
            end
          end
        end
        else begin
          if (|subburst_bytes_remaining)
            next_state = ST_WR_CONTINUE_SUBBURST;
          else
            next_state = ST_WR_BEGIN_SUBBURST;
        end

        p1_source0_burstwrap = altera_merlin_burst_adapter_burstwrap_min(OUT_MAX_BURSTWRAP, in_burstwrap);
        source0_byte_cnt = begin_subburst_byte_cnt;
      end

      ST_WR_CONTINUE_SUBBURST: begin
        p1_sink0_addr_reg = sink0_addr;
        p1_source0_valid = sink0_valid;
        p1_source0_startofpacket = sink0_startofpacket;
        source0_endofpacket = d1_sink0_endofpacket;
        sink0_ready = source0_ready | ~source0_valid;
        p1_sink0_channel_reg = sink0_channel;
        p1_sink0_byte_cnt_reg = sink0_byte_cnt;

        subburst_bytes_remaining = continue_subburst_subburst_bytes_remaining;

        // input values to the_min
        wrap_boundary_addr = sink0_addr[PKT_BURSTWRAP_W - 1 : 0];
        bytes_remaining = sink0_byte_cnt;

        if (source0_endofpacket) begin
          if (~sink0_valid) begin
            next_state = ST_IDLE;
          end
          else begin
            in_burstwrap = sink0_burstwrap;
            if (sink0_write) begin
              next_state = ST_WR_BEGIN_SUBBURST;
            end
            if (sink0_compressed_read) begin
              next_state = ST_RD_BEGIN_SUBBURST;
            end
            if (is_uncompressed_read) begin
              next_state = ST_UNCST_RD_BEGIN_SUBBURST;
            end
          end
        end
        else begin
          in_burstwrap = in_burstwrap_reg;
          if (|subburst_bytes_remaining)
            next_state = ST_WR_CONTINUE_SUBBURST;
          else
            next_state = ST_WR_BEGIN_SUBBURST;
        end

        p1_source0_burstwrap = altera_merlin_burst_adapter_burstwrap_min(OUT_MAX_BURSTWRAP, in_burstwrap);
        source0_byte_cnt = d1_subburst_bytes_remaining;
      end

      ST_RD_BEGIN_SUBBURST: begin
        // burstwrap doesn't change throughout a packet, so re-use the initial burstwrap value.
        p1_source0_burstwrap = source0_burstwrap;
        in_burstwrap = in_burstwrap_reg;

        bytes_remaining = sink0_byte_cnt_reg;
        p1_source0_startofpacket = source0_startofpacket;
        p1_sink0_byte_cnt_reg = sink0_byte_cnt_reg - begin_subburst_byte_cnt;
        // Todo: are the additions here done at full-address-width?  That would
        // be unnecessary.
        p1_sink0_addr_reg = 
          (sink0_addr_reg & ~addr_width_burstwrap) + (addr_width_burstwrap & (sink0_addr_reg + begin_subburst_byte_cnt));
        if (source0_ready) begin
          wrap_boundary_addr = sink0_addr_reg[PKT_BURSTWRAP_W - 1 : 0] + begin_subburst_byte_cnt[PKT_BURSTWRAP_W - 1 : 0];
          bytes_remaining = sink0_byte_cnt_reg - begin_subburst_byte_cnt;
          p1_source0_startofpacket = 1'b0;
        end
        else begin
          wrap_boundary_addr = sink0_addr_reg[PKT_BURSTWRAP_W - 1 : 0];
        end

        source0_byte_cnt = begin_subburst_byte_cnt;

        source0_endofpacket = 
         (source0_ready) ? (sink0_byte_cnt_reg == begin_subburst_byte_cnt) :
          sink0_byte_cnt_reg_zero;
          
        sink0_ready = (source0_ready | ~source0_valid) & source0_endofpacket;

        if (source0_endofpacket) begin
          p1_source0_valid = sink0_valid;
          if (~sink0_valid) begin
            next_state = ST_IDLE;
          end
          else begin
            // Transition to the next transaction.
            // To do: I've used the previous value bytes_remaining to compute
            // source0_endofpacket.   More reason to try to pre-compute
            // (bytes_remaining == 0)
            in_burstwrap = sink0_burstwrap;
            bytes_remaining = sink0_byte_cnt;
            wrap_boundary_addr = sink0_addr[PKT_BURSTWRAP_W - 1 : 0];

            p1_sink0_channel_reg = sink0_channel;
            p1_source0_burstwrap = altera_merlin_burst_adapter_burstwrap_min(OUT_MAX_BURSTWRAP, sink0_burstwrap);
            p1_sink0_addr_reg = sink0_addr;
            p1_source0_startofpacket = sink0_startofpacket;
            if (sink0_write) begin
              next_state = ST_WR_BEGIN_SUBBURST;
            end
            if (sink0_compressed_read) begin
              next_state = ST_RD_BEGIN_SUBBURST;
              // To calculate endofpacket, must know if this burst will be
              // adapted or not.
              p1_sink0_byte_cnt_reg = sink0_byte_cnt;
            end
            if (is_uncompressed_read) begin
              next_state = ST_UNCST_RD_BEGIN_SUBBURST;
            end
          end
        end
        else begin
          next_state = ST_RD_BEGIN_SUBBURST;
          p1_source0_valid = 1'b1;
        end
      end

      ST_UNCST_RD_BEGIN_SUBBURST: begin
        p1_sink0_addr_reg = sink0_addr;
 
        p1_source0_valid = sink0_valid;
        p1_source0_startofpacket = sink0_startofpacket;
        source0_endofpacket = d1_sink0_endofpacket;
        sink0_ready = source0_ready | ~source0_valid;
        p1_sink0_channel_reg = sink0_channel;
        p1_sink0_byte_cnt_reg = sink0_byte_cnt;

        // input values to the_min
        wrap_boundary_addr = sink0_addr[PKT_BURSTWRAP_W - 1 : 0];
        bytes_remaining = sink0_byte_cnt;

        subburst_bytes_remaining = begin_subburst_subburst_bytes_remaining;

        if (source0_endofpacket) begin
          if (~sink0_valid) begin
            next_state = ST_IDLE;
          end
          else begin
            in_burstwrap = sink0_burstwrap;
            if (sink0_write) begin
              next_state = ST_WR_BEGIN_SUBBURST;
            end
            if (sink0_compressed_read) begin
              next_state = ST_RD_BEGIN_SUBBURST;
            end
            if (is_uncompressed_read) begin
              next_state = ST_UNCST_RD_BEGIN_SUBBURST;
            end
          end
        end
        else begin
          in_burstwrap = in_burstwrap_reg;
          if (|subburst_bytes_remaining)
            next_state = ST_UNCRD_CONTINUE_SUBBURST;
          else
            next_state = ST_UNCST_RD_BEGIN_SUBBURST;
        end
        p1_source0_burstwrap = altera_merlin_burst_adapter_burstwrap_min(OUT_MAX_BURSTWRAP, in_burstwrap);
        source0_byte_cnt = num_symbols_sig;
      end

      ST_UNCRD_CONTINUE_SUBBURST: begin
        p1_sink0_addr_reg = sink0_addr;
        p1_source0_valid = sink0_valid;
        p1_source0_startofpacket = sink0_startofpacket;
        source0_endofpacket = d1_sink0_endofpacket;
        sink0_ready = source0_ready | ~source0_valid;
        p1_sink0_channel_reg = sink0_channel;
        p1_sink0_byte_cnt_reg = sink0_byte_cnt;

        subburst_bytes_remaining = continue_subburst_subburst_bytes_remaining;

        // input values to the_min
        wrap_boundary_addr = sink0_addr[PKT_BURSTWRAP_W - 1 : 0];
        bytes_remaining = sink0_byte_cnt;

        if (source0_endofpacket) begin
          if (~sink0_valid) begin
            next_state = ST_IDLE;
          end
          else begin
            in_burstwrap = sink0_burstwrap;
            if (sink0_write) begin
              next_state = ST_WR_BEGIN_SUBBURST;
            end
            if (sink0_compressed_read) begin
              next_state = ST_RD_BEGIN_SUBBURST;
            end
            if (is_uncompressed_read) begin
              next_state = ST_UNCST_RD_BEGIN_SUBBURST;
            end
          end
        end
        else begin
          in_burstwrap = in_burstwrap_reg;
          if (|subburst_bytes_remaining)
            next_state = ST_UNCRD_CONTINUE_SUBBURST;
          else
              next_state = ST_UNCST_RD_BEGIN_SUBBURST;
        end
        p1_source0_burstwrap = altera_merlin_burst_adapter_burstwrap_min(OUT_MAX_BURSTWRAP, in_burstwrap);
        source0_byte_cnt = num_symbols_sig;
      end
    endcase
  end

  always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
      state <= ST_IDLE;
      sink0_channel_reg <= '0;
      sink0_data_reg <= '0;
      source0_valid <= '0;
      source0_startofpacket <= '0;
      source0_burstwrap <= '0;
      d1_subburst_bytes_remaining <= '0;
      d1_sink0_endofpacket <= '0;
      sink0_byte_cnt_reg <= '0;
      sink0_byte_cnt_reg_zero <= '1;
      sink0_addr_reg <= '0;
      in_burstwrap_reg <= 'x;
    end
    else begin

      // Capture some sink signals as each incoming cycle is accepted.
      if (sink0_valid & sink0_ready) begin
        sink0_data_reg <= sink0_data;
        sink0_channel_reg <= p1_sink0_channel_reg;
        in_burstwrap_reg <= sink0_burstwrap;
        addr_width_burstwrap <= in_burstwrap[PKT_BURSTWRAP_W - 1] ? '1 : in_burstwrap;
        d1_sink0_endofpacket <= sink0_endofpacket;

        source0_burstwrap <= p1_source0_burstwrap;
      end

      // address, sink0_byte_cnt_reg are special - they're normally as-presented by the sink, but for
      // compressed reads they're computed throughout the downstream transaction.
      if ((sink0_valid & sink0_ready) || ((state == ST_RD_BEGIN_SUBBURST) && source0_ready)) begin
        sink0_addr_reg <= p1_sink0_addr_reg;
        sink0_byte_cnt_reg <= p1_sink0_byte_cnt_reg;
        sink0_byte_cnt_reg_zero <= ~|p1_sink0_byte_cnt_reg;
      end

      // Some things can change only when the source is not backpressuring.
      if (~source0_valid | source0_ready) begin
        state <= next_state;
        source0_valid <= p1_source0_valid;
        source0_startofpacket <= p1_source0_startofpacket;
      end

      // Some things can change only when a downstream cycle is accepted.
      if (source0_valid & source0_ready) begin
        d1_subburst_bytes_remaining <= subburst_bytes_remaining;
      end

    end
  end

  always_comb begin : source0_data_assignments
    // Generic assignment - assigns all fields, even the ones this component is
    // unaware of.
    source0_data = sink0_data_reg;

    // Override fields the component is aware of.
    source0_data[PKT_BYTE_CNT_H : PKT_BYTE_CNT_L] = source0_byte_cnt;
    source0_data[PKT_ADDR_H : PKT_ADDR_L] = sink0_addr_reg;
    source0_data[PKT_BURSTWRAP_H : PKT_BURSTWRAP_L] = source0_burstwrap;
  end

endmodule


