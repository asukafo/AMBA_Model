`include "axi_defs.svh"
//------------------------------------------------------------------------------
// axi_master_lite - synthesizable AXI4-Lite-style master (reference design)
//
// Strict AXI4-Lite subset: single beat (len=0), fixed 32b (size=2), INCR,
// ID always 0, no bursts - the typical master shape for register access.
// Verifies interconnect compatibility with Lite-subset masters (the TB
//
// ties len/size/burst/id to constants on the interconnect side).
// Single outstanding, blocking: write = AW->W(1 beat)->B; read = AR->R(1 beat);
// write and read share busy, AW wins when both compete in the same cycle.
//
// WSTRB comes from configuration (for partial register writes).
// Port style: flat packed vectors (iverilog-compatible subset), fully
// synthesizable.
//------------------------------------------------------------------------------
module axi_master_lite #(
  parameter int MST_ID     = 0,
  parameter int ADDR_WIDTH = `AXI_ADDR_W,
  parameter int DATA_WIDTH = `AXI_DATA_W
) (
  input  wire clk,
  input  wire rstn,
  input  wire start_wr,
  input  wire start_rd,
  input  wire [ADDR_WIDTH-1:0] cfg_addr,
  input  wire [DATA_WIDTH-1:0] cfg_wdata,
  input  wire [DATA_WIDTH/8-1:0] cfg_wstrb,
  input  wire [2:0] cfg_prot,
  output wire busy,
  output wire wr_done,           // one-cycle DONE-state output (combinational)
  output wire rd_done,
  output reg [1:0] wr_status,
  output reg [1:0] rd_status,
  output reg [DATA_WIDTH-1:0] rd_data,
  // ---- AXI ports (Lite subset: no len/size/burst/id) ----
  output reg awvalid,
  output reg [ADDR_WIDTH-1:0] awaddr,
  output reg [2:0]            awprot,
  input  wire awready,
  output reg wvalid,
  output reg [DATA_WIDTH-1:0]   wdata,
  output reg [DATA_WIDTH/8-1:0] wstrb,
  input  wire wready,
  input  wire bvalid,
  input  wire [1:0] bresp,
  output reg bready,
  output reg arvalid,
  output reg [ADDR_WIDTH-1:0] araddr,
  output reg [2:0]            arprot,
  input  wire arready,
  input  wire rvalid,
  input  wire [DATA_WIDTH-1:0] rdata,
  input  wire [1:0] rresp,
  output reg rready
);

  localparam L_IDLE = 2'd0, L_AW = 2'd1, L_WD = 2'd2, L_B = 2'd3;
  localparam R_IDLE = 2'd0, R_AR = 2'd1, R_RD = 2'd2;
  reg [1:0] l_state, r_state;
  reg [1:0] wr_status_q, rd_status_q;
  reg [DATA_WIDTH-1:0] rd_data_q;
  reg wr_done_q, rd_done_q;   // registered done pulses (visible to polling on the next cycle)

  always_comb begin
    awvalid = (l_state == L_AW);
    awaddr  = cfg_addr;
    awprot  = cfg_prot;
    wvalid  = (l_state == L_WD);
    wdata   = cfg_wdata;
    wstrb   = cfg_wstrb;
    bready  = (l_state == L_B);
    arvalid = (r_state == R_AR) && (l_state == L_IDLE);
    araddr  = cfg_addr;
    arprot  = cfg_prot;
    rready  = (r_state == R_RD);
  end

  always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      l_state <= L_IDLE;
      r_state <= R_IDLE;
      wr_status_q <= 2'b00;
      rd_status_q <= 2'b00;
      rd_data_q   <= '0;
      wr_done_q   <= 1'b0;
      rd_done_q   <= 1'b0;
    end else begin
      wr_done_q <= 1'b0;
      rd_done_q <= 1'b0;
      // ---- Write FSM ----
      case (l_state)
        L_IDLE: if (start_wr) l_state <= L_AW;
        L_AW:   if (awvalid && awready) l_state <= L_WD;
        L_WD:   if (wvalid && wready) l_state <= L_B;
        L_B:    if (bvalid && bready) begin
                  wr_status_q <= bresp;
                  wr_done_q   <= 1'b1;
                  l_state <= L_IDLE;
                end
        default: l_state <= L_IDLE;
      endcase
      // ---- Read FSM ----
      case (r_state)
        R_IDLE: if (start_rd && (l_state == L_IDLE)) r_state <= R_AR;
        R_AR:   if (arvalid && arready) r_state <= R_RD;
        R_RD:   if (rvalid && rready) begin
                  rd_status_q <= rresp;
                  rd_data_q   <= rdata;
                  rd_done_q   <= 1'b1;
                  r_state <= R_IDLE;
                end
        default: r_state <= R_IDLE;
      endcase
    end
  end

  assign wr_done  = wr_done_q;
  assign rd_done  = rd_done_q;
  assign busy     = (l_state != L_IDLE) || (r_state != R_IDLE);
  assign wr_status = wr_status_q;
  assign rd_status = rd_status_q;
  assign rd_data   = rd_data_q;

endmodule
