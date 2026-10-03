`include "axi_defs.svh"
//------------------------------------------------------------------------------
// axi_decerr - internal DECERR responder (one per master in the
// interconnect), flat ports
//
// Write path: 3-state FSM IDLE -> DRAIN -> BRESP
//   IDLE   accepts an unmatched AW (awready=1)
//   DRAIN  drains W beats with wready=1 unconditionally (w_drain_done
//          pulses on the WLAST handshake; the interconnect uses it to
//          clear w_pending)
//   BRESP  bvalid=1, b_id=aw_id, b_resp=DECERR, waits for bready
//   AXI requires B after WLAST; the FSM guarantees this naturally.
//
// Read path: an AR ID FIFO (reuses axi_owner_fifo); each transaction gets
// one beat of R=DECERR with RLAST=1.
//
// Mutual exclusion with the W mux is guaranteed by the interconnect's
// "one in-flight write data stream per master" rule: while this module is
// in DRAIN, that master's W beats can never be selected by any slave's
// W mux (AW arbitration requests are masked while w_pending is set).
//------------------------------------------------------------------------------
module axi_decerr #(
  parameter int AR_Q_DEPTH = `AXI_DECERR_Q
) (
  input  wire                  clk,
  input  wire                  rstn,
  // Write address (interconnect already gates awvalid with unmatched && !w_pending)
  input  wire                  awvalid,
  input  wire [`AXI_ID_W-1:0]  aw_id,
  output wire                  awready,
  // Write data (drained directly from the master's W channel; only valid/last matter)
  input  wire                  wvalid,
  input  wire                  w_last,
  output wire                  wready,
  output wire                  w_drain_done,
  // Write response
  output wire                  bvalid,
  output wire [`AXI_ID_W-1:0]  b_id,
  output wire [1:0]            b_resp,
  input  wire                  bready,
  // Read address (interconnect already gates arvalid with unmatched)
  input  wire                  arvalid,
  input  wire [`AXI_ID_W-1:0]  ar_id,
  output wire                  arready,
  // Read data (single beat, DECERR, RLAST=1)
  output wire                  rvalid,
  output wire [`AXI_ID_W-1:0]  r_id,
  output wire [1:0]            r_resp,
  input  wire                  rready
);

  // ---- Write-path FSM ----
  localparam W_IDLE = 2'd0, W_DRAIN = 2'd1, W_BRESP = 2'd2;
  reg [1:0]           wstate;
  reg [`AXI_ID_W-1:0] aw_id_q;

  assign awready      = (wstate == W_IDLE);
  assign wready       = (wstate == W_DRAIN);
  assign w_drain_done = (wstate == W_DRAIN) && wvalid && wready && w_last;

  assign bvalid = (wstate == W_BRESP);
  assign b_id   = aw_id_q;
  assign b_resp = `AXI_RESP_DECERR;

  always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      wstate  <= W_IDLE;
      aw_id_q <= '0;
    end else begin
      case (wstate)
        W_IDLE: begin
          if (awvalid && awready) begin
            aw_id_q <= aw_id;
            wstate  <= W_DRAIN;
          end
        end
        W_DRAIN: begin
          if (wvalid && wready && w_last)
            wstate <= W_BRESP;
        end
        W_BRESP: begin
          if (bvalid && bready)
            wstate <= W_IDLE;
        end
        default: wstate <= W_IDLE;
      endcase
    end
  end

  // ---- Read path: AR ID FIFO, one DECERR beat per transaction ----
  wire                 rf_empty, rf_full;
  wire [`AXI_ID_W-1:0] rf_head;

  assign arready = !rf_full;
  assign rvalid  = !rf_empty;
  assign r_id    = rf_head;
  assign r_resp  = `AXI_RESP_DECERR;

  axi_owner_fifo #(
    .DEPTH (AR_Q_DEPTH),
    .TAG_W (`AXI_ID_W)
  ) rid_fifo (
    .clk   (clk),
    .rstn  (rstn),
    .push  (arvalid && arready),
    .din   (ar_id),
    .pop   (rvalid && rready),
    .head  (rf_head),
    .empty (rf_empty),
    .full  (rf_full),
    .count ()
  );

endmodule
