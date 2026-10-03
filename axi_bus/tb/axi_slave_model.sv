`timescale 1ns/1ps
`include "axi_defs.svh"
//------------------------------------------------------------------------------
// axi_slave_model - configurable slave model (flat ports, iverilog-compatible subset)
//
// Features:
//   - sparse memory (addressed through a 4KB window; the caller guarantees
//     accesses stay within [BASE, BASE+MEM_DEPTH))
//   - AW queue + W beats written masked by WSTRB + B after WLAST (BID echoes
//     the widened AWID)
//   - AR queue + R burst responses (data read from memory beat by beat; RVALID
//
//     never drops mid-burst)
//   - random backpressure (xorshift32 LFSR, pulls READY low with a percentage
//------------------------------------------------------------------------------
module axi_slave_model #(
  parameter int SLV_ID    = 0,
  parameter int AW_Q      = 8,
  parameter int AR_Q      = 8,
  parameter int B_Q       = 8,
  parameter int MEM_DEPTH = 4096,
  parameter int B_DELAY   = 2,
  parameter int SEED      = 1
) (
  input  wire clk,
  input  wire rstn,
//     probability) and response latency (in cycles)
  input  wire [6:0] cfg_aw_rdy_pct,
  input  wire [6:0] cfg_w_rdy_pct,
  input  wire [7:0] cfg_r_delay,
  // AW
  input  wire awvalid,
  input  wire [`AXI_SLV_ID_W-1:0] awid,
  input  wire [`AXI_ADDR_W-1:0]   awaddr,
  input  wire [7:0]               awlen,
  input  wire [2:0]               awsize,
  input  wire [1:0]               awburst,
  output wire awready,
  // W
  input  wire wvalid,
  input  wire [`AXI_DATA_W-1:0]   wdata,
  input  wire [`AXI_DATA_W/8-1:0] wstrb,
  input  wire wlast,
  output wire wready,
  // B
  output wire bvalid,
  output wire [`AXI_SLV_ID_W-1:0] bid,
  output wire [1:0] bresp,
  input  wire bready,
  // AR
  input  wire arvalid,
  input  wire [`AXI_SLV_ID_W-1:0] arid,
  input  wire [`AXI_ADDR_W-1:0]   araddr,
  input  wire [7:0]               arlen,
  input  wire [2:0]               arsize,
  input  wire [1:0]               arburst,
  output wire arready,
  // R
  output reg rvalid,
  output reg [`AXI_SLV_ID_W-1:0] rid,
  output reg [`AXI_DATA_W-1:0]   rdata,
  output reg [1:0]               rresp,
  output reg rlast,
  input  wire rready
);

//   - config ports are adjusted dynamically by the TB between scenarios
  reg [7:0] mem [0:MEM_DEPTH-1];

//     (S4 reordering, S12 backpressure, etc.)
  reg [`AXI_SLV_ID_W-1:0] awq_id    [0:AW_Q-1];
  reg [`AXI_ADDR_W-1:0]   awq_addr  [0:AW_Q-1];
  reg [7:0]               awq_len   [0:AW_Q-1];
  reg [2:0]               awq_size  [0:AW_Q-1];
  reg [1:0]               awq_burst [0:AW_Q-1];
  integer awq_wr, awq_rd, awq_cnt;

// All queues are fixed-depth circular FIFOs (pointer + count, driven by a
  reg [`AXI_SLV_ID_W-1:0] bq_id [0:B_Q-1];
  integer bq_wr, bq_rd, bq_cnt;

// single always_ff to avoid multiple processes writing the same pointer).
  reg [`AXI_SLV_ID_W-1:0] arq_id    [0:AR_Q-1];
  reg [`AXI_ADDR_W-1:0]   arq_addr  [0:AR_Q-1];
  reg [7:0]               arq_len   [0:AR_Q-1];
  reg [2:0]               arq_size  [0:AR_Q-1];
  reg [1:0]               arq_burst [0:AR_Q-1];
  integer arq_wr, arq_rd, arq_cnt;

  // Backpressure/latency config (TB changes at runtime; 0-100 / cycles)
  integer w_beat_cnt, r_beat_cnt;
  integer b_delay_cnt, r_delay_cnt;

  // ---- Memory: addressed by addr[11:0] (window [BASE, BASE+MEM_DEPTH)) ----
  reg [`AXI_ADDR_W-1:0] w_addr_c;
  reg [`AXI_ADDR_W-1:0] r_addr_c;

  // ---- AW queue ----
  reg [31:0] lfsr;

  assign awready = (awq_cnt < AW_Q) && (lfsr[6:0] < cfg_aw_rdy_pct);
  assign wready  = (awq_cnt > 0) && (lfsr[6:0] < cfg_w_rdy_pct);
  assign arready = (arq_cnt < AR_Q) && (lfsr[6:0] < cfg_aw_rdy_pct);

  assign bvalid = (bq_cnt > 0) && (b_delay_cnt >= B_DELAY);
  assign bid    = bq_id[bq_rd];
  assign bresp  = `AXI_RESP_OKAY;

  // ---- B queue ----
  always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      awq_wr <= 0; awq_rd <= 0; awq_cnt <= 0;
      bq_wr <= 0; bq_rd <= 0; bq_cnt <= 0;
      w_beat_cnt <= 0; b_delay_cnt <= 0;
    end else begin
  // ---- AR queue ----
      w_addr_c = axi_beat_addr(awq_addr[awq_rd], awq_burst[awq_rd],
                               awq_size[awq_rd], awq_len[awq_rd], w_beat_cnt);
      if (wvalid && wready) begin
        for (int i = 0; i < `AXI_DATA_W/8; i++) begin
          if (wstrb[i])
            mem[w_addr_c[11:0] + i] <= wdata[8*i +: 8];
        end
        if (wlast) begin
          bq_id[bq_wr] <= awq_id[awq_rd];
          bq_wr <= (bq_wr == B_Q-1) ? 0 : bq_wr + 1;
          bq_cnt <= bq_cnt + 1;
          awq_rd <= (awq_rd == AW_Q-1) ? 0 : awq_rd + 1;
          awq_cnt <= awq_cnt - 1;
          w_beat_cnt <= 0;
        end else begin
          w_beat_cnt <= w_beat_cnt + 1;
        end
      end
  // ---- Beat counters / latency ----
      if (awvalid && awready) begin
        awq_id[awq_wr]    <= awid;
        awq_addr[awq_wr]  <= awaddr;
        awq_len[awq_wr]   <= awlen;
        awq_size[awq_wr]  <= awsize;
        awq_burst[awq_wr] <= awburst;
        awq_wr <= (awq_wr == AW_Q-1) ? 0 : awq_wr + 1;
  // Combinational temporaries (computed at the top of the always_ff)
        awq_cnt <= (wvalid && wready && wlast) ? awq_cnt : awq_cnt + 1;
      end
  // ---- Backpressure LFSR ----
      if (bq_cnt > 0 && b_delay_cnt < B_DELAY) begin
        b_delay_cnt <= b_delay_cnt + 1;
      end else if (bq_cnt > 0 && bvalid && bready) begin
        bq_rd <= (bq_rd == B_Q-1) ? 0 : bq_rd + 1;
  // Write path: AW receive + W beats (memory writes / pop AW and push B on
        bq_cnt <= (wvalid && wready && wlast) ? bq_cnt : bq_cnt - 1;
        b_delay_cnt <= 0;
      end
    end
  end

  // WLAST) + B send
  always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      arq_wr <= 0; arq_rd <= 0; arq_cnt <= 0;
      r_beat_cnt <= 0; r_delay_cnt <= 0;
    end else begin
      // ---- W beat ----
      if (rvalid && rready) begin
        if (rlast) begin
          arq_rd <= (arq_rd == AR_Q-1) ? 0 : arq_rd + 1;
          arq_cnt <= arq_cnt - 1;
          r_beat_cnt <= 0;
        end else begin
          r_beat_cnt <= r_beat_cnt + 1;
        end
        r_delay_cnt <= 0;
      end
      // ---- AW receive ----
      if (arvalid && arready) begin
        arq_id[arq_wr]    <= arid;
        arq_addr[arq_wr]  <= araddr;
        arq_len[arq_wr]   <= arlen;
        arq_size[arq_wr]  <= arsize;
        arq_burst[arq_wr] <= arburst;
        arq_wr <= (arq_wr == AR_Q-1) ? 0 : arq_wr + 1;
        // If W also completes the previous entry in the same cycle (pop), net change is 0
        arq_cnt <= (rvalid && rready && rlast) ? arq_cnt : arq_cnt + 1;
      end
      // ---- B send ----
      if (arq_cnt > 0 && r_delay_cnt < cfg_r_delay) begin
        r_delay_cnt <= r_delay_cnt + 1;
      end
    end
  end

        // If W also pushed a new B in the same cycle, net change is 0
  always_comb begin
    rvalid = (arq_cnt > 0) && (r_delay_cnt >= cfg_r_delay);
    rid    = arq_id[arq_rd];
    rresp  = `AXI_RESP_OKAY;
    rlast  = (r_beat_cnt == arq_len[arq_rd]);
    rdata  = '0;
    r_addr_c = axi_beat_addr(arq_addr[arq_rd], arq_burst[arq_rd],
                             arq_size[arq_rd], arq_len[arq_rd], r_beat_cnt);
    if (rvalid) begin
  // Read path: R beats (pop on the RLAST handshake) + AR receive + R latency
      rdata = {mem[r_addr_c[11:0]+3], mem[r_addr_c[11:0]+2],
               mem[r_addr_c[11:0]+1], mem[r_addr_c[11:0]]};
    end
  end

      // ---- R beat ----
  always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      lfsr <= SEED + SLV_ID + 1;
    end else begin
      reg [31:0] x;
      x = lfsr;
      x ^= x << 13;
      x ^= x >> 17;
      x ^= x << 5;
      lfsr <= x;
    end
  end

      // ---- AR receive ----
  initial begin
    for (int i = 0; i < MEM_DEPTH; i++)
      mem[i] = 8'h00;
  end

        // If R also completes the previous entry in the same cycle (pop), net change is 0
  function automatic [7:0] get_byte;
    input [11:0] a;
    begin
      get_byte = mem[a];
    end
  endfunction

endmodule
