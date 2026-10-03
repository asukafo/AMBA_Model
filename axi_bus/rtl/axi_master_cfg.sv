`include "axi_defs.svh"
//------------------------------------------------------------------------------
// axi_master_cfg - synthesizable AXI4 master (reference design)
//
// Register-configured, FSM-driven: a write transaction = AW -> W burst ->
// receive B; a read transaction = AR -> receive the R burst. The write and
// read FSMs are independent and may run concurrently.
//
// Deterministic transaction data (easy to verify):
//   beat b write data = cfg_wdata0 + b (WSTRB all ones)
//   read checksum = XOR accumulation of all read-beat rdata
//   (rd_checksum), recomputed and compared on the TB side
//
// Config convention: cfg_* must be written while busy=0 and must stay
// stable for the whole transaction (the transaction takes ownership of
// the config values on the cycle after the start pulse).
//
// Port style: flat packed vectors (iverilog-compatible subset, see the
// header comment of axi_interconnect.sv), fully synthesizable.
//------------------------------------------------------------------------------
module axi_master_cfg #(
  parameter int MST_ID     = 0,
  parameter int ADDR_WIDTH = `AXI_ADDR_W,
  parameter int DATA_WIDTH = `AXI_DATA_W,
  parameter int ID_WIDTH   = `AXI_ID_W
) (
  input  wire clk,
  input  wire rstn,
  // ---- Config / status (software side) ----
  input  wire                  wr_start,   // write-transaction start pulse
  input  wire                  rd_start,   // read-transaction start pulse
  input  wire [ADDR_WIDTH-1:0] cfg_addr,
  input  wire [7:0]            cfg_len,    // burst length (len+1 beats)
  input  wire [2:0]            cfg_size,
  input  wire [1:0]            cfg_burst,
  input  wire [ID_WIDTH-1:0]   cfg_id,
  input  wire [DATA_WIDTH-1:0] cfg_wdata0, // beat-0 write data; later beats increment
  input  wire [DATA_WIDTH/8-1:0] cfg_wstrb, // write byte strobes (all ones by default)
  output wire                  busy,
  output wire                  wr_done,    // write-transaction done pulse
  output wire                  rd_done,    // read-transaction done pulse
  output reg  [1:0]            wr_status,  // write response code
  output reg  [1:0]            rd_status,  // read response code (last beat)
  output wire [DATA_WIDTH-1:0] rd_checksum,// read-data XOR accumulation
  // ---- AXI master ports (AW) ----
  output reg awvalid,
  output reg [ID_WIDTH-1:0]   awid,
  output reg [ADDR_WIDTH-1:0] awaddr,
  output reg [7:0]            awlen,
  output reg [2:0]            awsize,
  output reg [1:0]            awburst,
  input  wire awready,
  // ---- W ----
  output reg wvalid,
  output reg [DATA_WIDTH-1:0]   wdata,
  output reg [DATA_WIDTH/8-1:0] wstrb,
  output reg wlast,
  input  wire wready,
  // ---- B ----
  input  wire bvalid,
  input  wire [ID_WIDTH-1:0] bid,
  input  wire [1:0] bresp,
  output reg bready,
  // ---- AR ----
  output reg arvalid,
  output reg [ID_WIDTH-1:0]   arid,
  output reg [ADDR_WIDTH-1:0] araddr,
  output reg [7:0]            arlen,
  output reg [2:0]            arsize,
  output reg [1:0]            arburst,
  input  wire arready,
  // ---- R ----
  input  wire rvalid,
  input  wire [ID_WIDTH-1:0] rid,
  input  wire [DATA_WIDTH-1:0] rdata,
  input  wire [1:0] rresp,
  input  wire rlast,
  output reg rready
);

  //==========================================================================
  // Write FSM: IDLE -> AW -> DATA -> RESP -> DONE
  //==========================================================================
  localparam W_IDLE = 3'd0, W_AW = 3'd1, W_DATA = 3'd2,
             W_RESP = 3'd3, W_DONE = 3'd4;
  reg [2:0] w_state;
  reg [7:0] w_beat;

  always_comb begin
    awvalid = (w_state == W_AW);
    awid    = cfg_id;
    awaddr  = cfg_addr;
    awlen   = cfg_len;
    awsize  = cfg_size;
    awburst = cfg_burst;
    wvalid  = (w_state == W_DATA);
    wdata   = cfg_wdata0 + w_beat;   // beat b data = base + b
    wstrb   = cfg_wstrb;
    wlast   = (w_beat == cfg_len);
    bready  = (w_state == W_RESP);
  end

  always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      w_state <= W_IDLE;
      w_beat  <= 8'd0;
      wr_status <= 2'b00;
    end else begin
      case (w_state)
        W_IDLE: if (wr_start) w_state <= W_AW;
        W_AW:   if (awvalid && awready) begin
                  w_beat  <= 8'd0;
                  w_state <= W_DATA;
                end
        W_DATA: if (wvalid && wready) begin
                  if (wlast) w_state <= W_RESP;
                  else w_beat <= w_beat + 8'd1;
                end
        W_RESP: if (bvalid && bready) begin
                  wr_status <= bresp;
                  w_state   <= W_DONE;
                end
        // DONE state lasts one cycle (wr_done visible), then back to IDLE
        W_DONE: w_state <= W_IDLE;
        default: w_state <= W_IDLE;
      endcase
    end
  end

  //==========================================================================
  // Read FSM: IDLE -> AR -> DATA -> DONE
  //==========================================================================
  localparam R_IDLE = 2'd0, R_AR = 2'd1, R_DATA = 2'd2, R_DONE = 2'd3;
  reg [1:0] r_state;
  reg [DATA_WIDTH-1:0] chk_q;

  always_comb begin
    arvalid = (r_state == R_AR);
    arid    = cfg_id;
    araddr  = cfg_addr;
    arlen   = cfg_len;
    arsize  = cfg_size;
    arburst = cfg_burst;
    rready  = (r_state == R_DATA);
  end

  always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      r_state  <= R_IDLE;
      rd_status <= 2'b00;
      chk_q    <= '0;
    end else begin
      case (r_state)
        R_IDLE: if (rd_start) begin
                  chk_q   <= '0;
                  r_state <= R_AR;
                end
        R_AR:   if (arvalid && arready) r_state <= R_DATA;
        R_DATA: if (rvalid && rready) begin
                  chk_q <= chk_q ^ rdata;
                  if (rlast) begin
                    rd_status <= rresp;
                    r_state   <= R_DONE;
                  end
                end
        // DONE state lasts one cycle (rd_done visible), then back to IDLE
        R_DONE: r_state <= R_IDLE;
        default: r_state <= R_IDLE;
      endcase
    end
  end

  // done is the combinational DONE-state output (visible in the same cycle as status)
  assign wr_done = (w_state == W_DONE);
  assign rd_done = (r_state == R_DONE);
  assign rd_checksum = chk_q;
  assign busy = (w_state != W_IDLE) || (r_state != R_IDLE);

endmodule
