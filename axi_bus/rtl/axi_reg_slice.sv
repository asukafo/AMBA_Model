`include "axi_defs.svh"
//------------------------------------------------------------------------------
// axi_reg_slice - forward register slice (skid buffer) for any single
// AXI channel
//
// Behavior: the output is registered (breaking the forward combinational
// path); under backpressure one beat is captured in the skid register
// without loss:
//   - empty: pass-through (dout = din, din_ready = dout_ready)
//   - holding skid data: dout = q, din_ready = 0 (drain first)
//   - capture: store din in q when din_valid && !dout_ready
//
// Protocol-transparent: can be inserted on any valid/payload/ready
// channel (all 5 AXI channels, any handshake sequence); adds one cycle
// of latency only.
//
// Port style: flat packed vectors (iverilog-compatible subset), fully
// synthesizable.
//------------------------------------------------------------------------------
module axi_reg_slice #(
  parameter int W = 64    // payload width
) (
  input  wire clk,
  input  wire rstn,
  input  wire        din_valid,
  input  wire [W-1:0] din,
  output reg         din_ready,
  output reg         dout_valid,
  output reg  [W-1:0] dout,
  input  wire        dout_ready
);

  reg         q_valid;
  reg  [W-1:0] q_data;

  // Key: din_ready must equal dout_ready - the drain beat and the upstream
  // handshake complete on the same edge, so the upstream "completing the
  // current beat" and the downstream "consuming that beat" stay strictly in
  // sync. This prevents the same beat from being replayed through the
  // pass-through path. (The upstream advances to the next beat only after
  // its handshake, so while draining, its current beat is the same beat
  // that sits in q.)
  always_comb begin
    if (q_valid) begin
      dout_valid = 1'b1;
      dout       = q_data;
    end else begin
      dout_valid = din_valid;
      dout       = din;
    end
    din_ready = dout_ready;
  end

  always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      q_valid <= 1'b0;
      q_data  <= '0;
    end else begin
      if (q_valid) begin
        // Drain the skid data
        if (dout_ready) q_valid <= 1'b0;
      end else begin
        // Pass-through stalled: capture
        if (din_valid && !dout_ready) begin
          q_valid <= 1'b1;
          q_data  <= din;
        end
      end
    end
  end

endmodule
