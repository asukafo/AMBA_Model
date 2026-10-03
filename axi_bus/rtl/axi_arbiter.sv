`include "axi_defs.svh"
//------------------------------------------------------------------------------
// axi_arbiter - generic arbiter: req[N] -> onehot grant
//   POLICY = 0: fixed priority (lowest index wins)
//   POLICY = 1: round-robin (rr_ptr scans cyclically starting after the
//               last granted requester)
//
// Timing:
//   - grant is a registered output; once issued it is locked until ack
//     (the handshake of the granted beat)
//   - the RR pointer rotates only on handshake (ack)
//   - there is one bubble cycle between consecutive grants (the next
//     request is picked one cycle after grant release)
//   - req changes are ignored while a grant is locked; a requester must
//     hold req until the handshake (under AXI this means VALID must be
//     held until READY, guaranteed jointly by the interconnect and BFM)
//------------------------------------------------------------------------------
module axi_arbiter #(
  parameter int N      = 2,
  parameter int POLICY = 1
) (
  input  wire         clk,
  input  wire         rstn,
  input  wire [N-1:0] req,
  input  wire         ack,
  output wire [N-1:0] grant
);

  localparam PTR_W = (N > 1) ? $clog2(N) : 1;

  reg [N-1:0]      grant_q, pick;
  reg [PTR_W-1:0]  rr_ptr;

  function automatic integer onehot_idx;
    input [N-1:0] v;
    integer i;
    begin
      onehot_idx = 0;
      for (i = 0; i < N; i = i + 1)
        if (v[i]) onehot_idx = i;
    end
  endfunction

  // Combinational pick (sampled only when no grant is held)
  reg found;
  always_comb begin
    pick  = '0;
    found = 1'b0;
    if (POLICY == 0) begin
      // Fixed priority: lowest set bit
      for (int i = 0; i < N; i++) begin
        if (req[i] && !found) begin
          pick[i] = 1'b1;
          found   = 1'b1;
        end
      end
    end else begin
      // RR: scan cyclically starting from rr_ptr
      for (int k = 0; k < N; k++) begin
        int unsigned i;
        i = rr_ptr + k;
        if (i >= N) i -= N;
        if (req[i] && !found) begin
          pick[i] = 1'b1;
          found   = 1'b1;
        end
      end
    end
  end

  always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      grant_q <= '0;
      rr_ptr  <= '0;
    end else begin
      if (|grant_q) begin
        if (ack) begin
          if (POLICY == 1) begin
            if (onehot_idx(grant_q) == N - 1) rr_ptr <= '0;
            else rr_ptr <= onehot_idx(grant_q) + 1'b1;
          end
          grant_q <= '0;
        end
      end else begin
        grant_q <= pick;
      end
    end
  end

  assign grant = grant_q;

endmodule
