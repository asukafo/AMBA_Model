`include "axi_defs.svh"
//------------------------------------------------------------------------------
// axi_slave_ram - synthesizable AXI4 RAM slave (reference design)
//
// True RAM storage (DEPTH bytes, power of 2), multiple outstanding
// transactions (fixed-depth queues):
//   - an AW queue receives write addresses; W beats write the current entry
//     masked by WSTRB (in AW order); after WLAST the widened AWID is pushed
//     into the B queue and echoed back on B without delay
//   - an AR queue receives read addresses; the R burst reads the RAM beat by
//     beat (combinational read port); RVALID is held until RREADY and never
//     drops mid-burst
//   - burst address computation supports INCR/WRAP/FIXED (axi_beat_addr,
//     synthesizable)
//
// Address window: memory is indexed by addr[$clog2(DEPTH)-1:0] (the caller
// guarantees accesses stay within the window).
//
// Port style: flat packed vectors (iverilog-compatible subset), fully
// synthesizable.
//------------------------------------------------------------------------------
module axi_slave_ram #(
  parameter int SLV_ID = 0,
  parameter int DEPTH  = 4096,   // bytes, power of 2
  parameter int AW_Q   = 4,
  parameter int AR_Q   = 4
) (
  input  wire clk,
  input  wire rstn,
  // ---- AW ----
  input  wire awvalid,
  input  wire [`AXI_SLV_ID_W-1:0] awid,
  input  wire [`AXI_ADDR_W-1:0]   awaddr,
  input  wire [7:0]               awlen,
  input  wire [2:0]               awsize,
  input  wire [1:0]               awburst,
  input  wire                     awlock,   // exclusive write (AWLOCK=1)
  output wire awready,
  // ---- W ----
  input  wire wvalid,
  input  wire [`AXI_DATA_W-1:0]   wdata,
  input  wire [`AXI_DATA_W/8-1:0] wstrb,
  input  wire wlast,
  output wire wready,
  // ---- B ----
  output wire bvalid,
  output wire [`AXI_SLV_ID_W-1:0] bid,
  output wire [1:0] bresp,
  input  wire bready,
  // ---- AR ----
  input  wire arvalid,
  input  wire [`AXI_SLV_ID_W-1:0] arid,
  input  wire [`AXI_ADDR_W-1:0]   araddr,
  input  wire [7:0]               arlen,
  input  wire [2:0]               arsize,
  input  wire [1:0]               arburst,
  input  wire                     arlock,   // exclusive read (ARLOCK=1)
  output wire arready,
  // ---- R ----
  output reg rvalid,
  output reg [`AXI_SLV_ID_W-1:0] rid,
  output reg [`AXI_DATA_W-1:0]   rdata,
  output reg [1:0]               rresp,
  output reg rlast,
  input  wire rready,
  // ---- Debug read port (TB memory checking, combinational) ----
  input  wire [`AXI_ADDR_W-1:0]   dbg_addr,
  output wire [7:0]               dbg_byte
);

  localparam A_W = $clog2(DEPTH);

  reg [7:0] mem [0:DEPTH-1];
  initial begin
    for (int i = 0; i < DEPTH; i++)
      mem[i] = 8'h00;
  end

  // ---- AW queue ----
  reg [`AXI_SLV_ID_W-1:0] awq_id    [0:AW_Q-1];
  reg [`AXI_ADDR_W-1:0]   awq_addr  [0:AW_Q-1];
  reg [7:0]               awq_len   [0:AW_Q-1];
  reg [2:0]               awq_size  [0:AW_Q-1];
  reg [1:0]               awq_burst [0:AW_Q-1];
  reg                     awq_exok  [0:AW_Q-1];  // exclusive-write success flag
  reg [$clog2(AW_Q)-1:0]  awq_wr, awq_rd;
  reg [$clog2(AW_Q+1)-1:0] awq_cnt;

  // ---- B queue (ID + exclusive success flag) ----
  reg [`AXI_SLV_ID_W-1:0] bq_id  [0:AW_Q-1];
  reg                     bq_exok [0:AW_Q-1];
  reg [$clog2(AW_Q)-1:0]  bq_wr, bq_rd;
  reg [$clog2(AW_Q+1)-1:0] bq_cnt;

  // ---- AR queue ----
  reg [`AXI_SLV_ID_W-1:0] arq_id    [0:AR_Q-1];
  reg [`AXI_ADDR_W-1:0]   arq_addr  [0:AR_Q-1];
  reg [7:0]               arq_len   [0:AR_Q-1];
  reg [2:0]               arq_size  [0:AR_Q-1];
  reg [1:0]               arq_burst [0:AR_Q-1];
  reg                     arq_excl  [0:AR_Q-1];  // exclusive-read flag
  reg [$clog2(AR_Q)-1:0]  arq_wr, arq_rd;
  reg [$clog2(AR_Q+1)-1:0] arq_cnt;

  // ---- Exclusive monitor (simplified: one global monitor point, not per-address) ----
  reg                     excl_own;
  reg [`AXI_SLV_ID_W-1:0] excl_id;

  // ---- Beat counters / combinational addresses ----
  reg [7:0]               w_beat_cnt, r_beat_cnt;
  reg [`AXI_ADDR_W-1:0]   w_addr_c, r_addr_c;

  assign awready = (awq_cnt < AW_Q);
  assign wready  = (awq_cnt > 0);
  assign arready = (arq_cnt < AR_Q);
  assign bvalid  = (bq_cnt > 0);
  assign bid     = bq_id[bq_rd];
  assign bresp   = bq_exok[bq_rd] ? `AXI_RESP_EXOKAY : `AXI_RESP_OKAY;

  //==========================================================================
  // Write path: W beats write the RAM (in AW order) / AW receive / B send
  //==========================================================================
  always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      awq_wr <= '0; awq_rd <= '0; awq_cnt <= '0;
      bq_wr  <= '0; bq_rd  <= '0; bq_cnt  <= '0;
      w_beat_cnt <= 8'd0;
      excl_own <= 1'b0;
      excl_id  <= '0;
    end else begin
      // ---- W beat ----
      w_addr_c = axi_beat_addr(awq_addr[awq_rd], awq_burst[awq_rd],
                               awq_size[awq_rd], awq_len[awq_rd], w_beat_cnt);
      if (wvalid && wready) begin
        for (int i = 0; i < `AXI_DATA_W/8; i++) begin
          if (wstrb[i])
            // Lane mapping: lane i writes byte at word-aligned base + i (AXI narrow-transfer rule)
            mem[{w_addr_c[A_W-1:2], 2'b00} + i] <= wdata[8*i +: 8];
        end
        if (wlast) begin
          bq_id[bq_wr]  <= awq_id[awq_rd];
          bq_exok[bq_wr] <= awq_exok[awq_rd];
          bq_wr  <= (bq_wr == AW_Q-1) ? '0 : bq_wr + 1'b1;
          bq_cnt <= bq_cnt + 1'b1;
          awq_rd  <= (awq_rd == AW_Q-1) ? '0 : awq_rd + 1'b1;
          awq_cnt <= awq_cnt - 1'b1;
          w_beat_cnt <= 8'd0;
        end else begin
          w_beat_cnt <= w_beat_cnt + 8'd1;
        end
      end
      // ---- AW receive ----
      if (awvalid && awready) begin
        awq_id[awq_wr]    <= awid;
        awq_addr[awq_wr]  <= awaddr;
        awq_len[awq_wr]   <= awlen;
        awq_size[awq_wr]  <= awsize;
        awq_burst[awq_wr] <= awburst;
        // Exclusive-write decision: EXOKAY if the monitor point is still owned
        // by this master and AWLOCK=1; any write (including normal ones)
        // consumes/clears the monitor point
        awq_exok[awq_wr]  <= awlock && excl_own && (excl_id === awid);
        excl_own <= 1'b0;
        awq_wr <= (awq_wr == AW_Q-1) ? '0 : awq_wr + 1'b1;
        // If W also completes the previous entry in the same cycle (pop), net change is 0
        awq_cnt <= (wvalid && wready && wlast) ? awq_cnt : awq_cnt + 1'b1;
      end
      // ---- B send ----
      if (bq_cnt > 0 && bvalid && bready) begin
        bq_rd <= (bq_rd == AW_Q-1) ? '0 : bq_rd + 1'b1;
        // If W also pushes a new entry in the same cycle, net change is 0
        bq_cnt <= (wvalid && wready && wlast) ? bq_cnt : bq_cnt - 1'b1;
      end
    end
  end

  //==========================================================================
  // Read path: AR receive / R beats (pop on the RLAST handshake)
  //==========================================================================
  always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      arq_wr <= '0; arq_rd <= '0; arq_cnt <= '0;
      r_beat_cnt <= 8'd0;
    end else begin
      // ---- R beat ----
      if (rvalid && rready) begin
        if (rlast) begin
          arq_rd  <= (arq_rd == AR_Q-1) ? '0 : arq_rd + 1'b1;
          arq_cnt <= arq_cnt - 1'b1;
          r_beat_cnt <= 8'd0;
        end else begin
          r_beat_cnt <= r_beat_cnt + 8'd1;
        end
      end
      // ---- AR receive ----
      if (arvalid && arready) begin
        arq_id[arq_wr]    <= arid;
        arq_addr[arq_wr]  <= araddr;
        arq_len[arq_wr]   <= arlen;
        arq_size[arq_wr]  <= arsize;
        arq_burst[arq_wr] <= arburst;
        // Exclusive read: establish the monitor point (simplified global one); respond EXOKAY
        arq_excl[arq_wr] <= arlock;
        if (arlock) begin
          excl_own <= 1'b1;
          excl_id  <= arid;
        end
        arq_wr <= (arq_wr == AR_Q-1) ? '0 : arq_wr + 1'b1;
        // If R also completes the previous entry in the same cycle (pop), net change is 0
        arq_cnt <= (rvalid && rready && rlast) ? arq_cnt : arq_cnt + 1'b1;
      end
    end
  end

  // ---- R output (combinational): RVALID held until READY, never drops mid-burst ----
  always_comb begin
    rvalid = (arq_cnt > 0);
    rid    = arq_id[arq_rd];
    rresp  = arq_excl[arq_rd] ? `AXI_RESP_EXOKAY : `AXI_RESP_OKAY;
    rlast  = (r_beat_cnt == arq_len[arq_rd]);
    r_addr_c = axi_beat_addr(arq_addr[arq_rd], arq_burst[arq_rd],
                             arq_size[arq_rd], arq_len[arq_rd], r_beat_cnt);
    // Lane mapping: lane L reads byte at word-aligned base + L (AXI narrow-transfer rule);
    // little-endian word assembly
    rdata = {mem[{r_addr_c[A_W-1:2], 2'b00}+3], mem[{r_addr_c[A_W-1:2], 2'b00}+2],
             mem[{r_addr_c[A_W-1:2], 2'b00}+1], mem[{r_addr_c[A_W-1:2], 2'b00}]};
  end

  // ---- Debug read port ----
  assign dbg_byte = mem[dbg_addr[A_W-1:0]];

endmodule
