`include "axi_defs.svh"
//------------------------------------------------------------------------------
// axi_master_excl - synthesizable exclusive-access AXI4 master (reference design)
//
// Typical spinlock flow: after start, issue an exclusive read with ARLOCK=1;
// if the response is EXOKAY, follow up with a conditional exclusive write with
// AWLOCK=1 (W data = cfg_wdata0 + beat index, full WSTRB) and record the
// write response (EXOKAY = still held / OKAY = lost); if the read response
//
// is not EXOKAY, skip the write phase and finish directly (wr_issued=0).
// Verifies pass-through of AWLOCK/ARLOCK/EXOKAY through the interconnect
//
// and the monitor semantics in the slave (companion slave: the exclusive
// monitor of axi_slave_ram).
//
// Port style: flat packed vectors (iverilog-compatible subset), fully
// synthesizable.
//------------------------------------------------------------------------------
module axi_master_excl #(
  parameter int MST_ID     = 0,
  parameter int ADDR_WIDTH = `AXI_ADDR_W,
  parameter int DATA_WIDTH = `AXI_DATA_W,
  parameter int ID_WIDTH   = `AXI_ID_W
) (
  input  wire clk,
  input  wire rstn,
  input  wire start,
  input  wire [ADDR_WIDTH-1:0] cfg_addr,
  input  wire [7:0]            cfg_len,
  input  wire [2:0]            cfg_size,
  input  wire [1:0]            cfg_burst,
  input  wire [ID_WIDTH-1:0]   cfg_id,
  input  wire [DATA_WIDTH-1:0] cfg_wdata0,
  output wire busy,
  // ---- Config / status (software side) ----
  input  wire [7:0]            cfg_wr_delay, // cycles between exclusive read and conditional write (0=immediate)
  output wire done,            // one-cycle DONE-state output (combinational)
  output reg [1:0] rd_resp,   // exclusive read response
  output wire [DATA_WIDTH-1:0] rd_checksum,
  output reg [1:0] wr_resp,   // conditional write response (0 when no write was issued)
  output reg wr_issued,       // a conditional write was issued after an EXOKAY read
  output reg awvalid,
  output reg [ID_WIDTH-1:0]   awid,
  output reg [ADDR_WIDTH-1:0] awaddr,
  output reg [7:0]            awlen,
  output reg [2:0]            awsize,
  output reg [1:0]            awburst,
  output reg                  awlock,
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
  output reg                  arlock,
  input  wire arready,
  // ---- R ----
  input  wire rvalid,
  input  wire [ID_WIDTH-1:0] rid,
  input  wire [DATA_WIDTH-1:0] rdata,
  input  wire [1:0] rresp,
  input  wire rlast,
  output reg rready
);

  localparam E_IDLE = 3'd0, E_AR = 3'd1, E_RD = 3'd2, E_DLY = 3'd3,
             E_AW = 3'd4, E_WD = 3'd5, E_B = 3'd6, E_DONE = 3'd7;
  reg [2:0] e_state;
  reg [7:0] w_beat;
  reg [7:0] dly_cnt;
  reg [DATA_WIDTH-1:0] chk_q;
  reg [1:0] rd_resp_q, wr_resp_q;
  reg wr_issued_q;

  always_comb begin
    awvalid = (e_state == E_AW);
    awid    = cfg_id;
    awaddr  = cfg_addr;
    awlen   = cfg_len;
    awsize  = cfg_size;
    awburst = cfg_burst;
    awlock  = 1'b1;               // conditional exclusive write always carries AWLOCK=1
    wvalid  = (e_state == E_WD);
    wdata   = cfg_wdata0 + w_beat;
    wstrb   = {(`AXI_DATA_W/8){1'b1}};
    wlast   = (w_beat == cfg_len);
    bready  = (e_state == E_B);
    arvalid = (e_state == E_AR);
    arid    = cfg_id;
    araddr  = cfg_addr;
    arlen   = cfg_len;
    arsize  = cfg_size;
    arburst = cfg_burst;
    arlock  = 1'b1;               // exclusive read always carries ARLOCK=1
    rready  = (e_state == E_RD);
  end

  always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      e_state   <= E_IDLE;
      w_beat    <= 8'd0;
      chk_q     <= '0;
      rd_resp_q <= 2'b00;
      wr_resp_q <= 2'b00;
      wr_issued_q <= 1'b0;
    end else begin
      case (e_state)
        E_IDLE: if (start) begin
                  chk_q   <= '0;
                  rd_resp_q <= 2'b00;
                  wr_resp_q <= 2'b00;
                  wr_issued_q <= 1'b0;
                  w_beat  <= 8'd0;
                  e_state <= E_AR;
                end
        E_AR:   if (arvalid && arready) e_state <= E_RD;
        E_RD:   if (rvalid && rready) begin
                  chk_q <= chk_q ^ rdata;
                  if (rlast) begin
                    rd_resp_q <= rresp;
                    if (rresp == `AXI_RESP_EXOKAY) begin
                      wr_issued_q <= 1'b1;
                      dly_cnt <= 8'd0;
                      e_state <= E_DLY;   // configurable delay (leaves a window for interfering writes)
                    end else begin
                      e_state <= E_DONE;   // exclusive lost, skip the write
                    end
                  end
                end
        E_DLY:  begin
                  if (dly_cnt >= cfg_wr_delay) e_state <= E_AW;
                  else dly_cnt <= dly_cnt + 8'd1;
                end
        E_AW:   if (awvalid && awready) begin
                  w_beat  <= 8'd0;
                  e_state <= E_WD;
                end
        E_WD:   if (wvalid && wready) begin
                  if (wlast) e_state <= E_B;
                  else w_beat <= w_beat + 8'd1;
                end
        E_B:    if (bvalid && bready) begin
                  wr_resp_q <= bresp;
                  e_state   <= E_DONE;
                end
        E_DONE: e_state <= E_IDLE;
        default: e_state <= E_IDLE;
      endcase
    end
  end

  assign done        = (e_state == E_DONE);
  assign busy        = (e_state != E_IDLE);
  assign rd_resp     = rd_resp_q;
  assign wr_resp     = wr_resp_q;
  assign wr_issued   = wr_issued_q;
  assign rd_checksum = chk_q;

endmodule
