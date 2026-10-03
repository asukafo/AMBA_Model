`include "axi_defs.svh"
//------------------------------------------------------------------------------
// axi_owner_fifo - W-channel ownership tag FIFO (shift-register style,
// combinational head output)
//
// One instance per slave inside the interconnect: push the granted master's
// tag on the AW handshake, pop on the WLAST handshake; the W mux is driven
// by head. Simultaneous push/pop is legal.
//
// full is unreachable in interconnect usage (each master has at most one
// write data stream in flight, and at push time that master's w_pending is
// 0, so at most N_MASTER-1 entries are occupied - a depth of N_MASTER
// suffices). The full output is kept for assertions/debug only.
//------------------------------------------------------------------------------
module axi_owner_fifo #(
  parameter int DEPTH = 2,
  parameter int TAG_W = 1
) (
  input  wire                    clk,
  input  wire                    rstn,
  input  wire                    push,
  input  wire [TAG_W-1:0]        din,
  input  wire                    pop,
  output wire [TAG_W-1:0]        head,
  output wire                    empty,
  output wire                    full,
  output wire [$clog2(DEPTH+1)-1:0] count
);

  localparam PTR_W = (DEPTH > 1) ? $clog2(DEPTH) : 1;

  reg [TAG_W-1:0]            mem [DEPTH];
  reg [PTR_W-1:0]            wr_ptr, rd_ptr;
  reg [$clog2(DEPTH+1)-1:0]  count_q;

  assign head  = mem[rd_ptr];
  assign empty = (count_q == 0);
  assign full  = (count_q == DEPTH);
  assign count = count_q;

  always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      wr_ptr  <= '0;
      rd_ptr  <= '0;
      count_q <= '0;
    end else begin
      case ({push, pop})
        2'b10: begin
          mem[wr_ptr] <= din;
          wr_ptr <= (wr_ptr == DEPTH - 1) ? '0 : wr_ptr + 1'b1;
          count_q <= count_q + 1'b1;
        end
        2'b01: begin
          rd_ptr <= (rd_ptr == DEPTH - 1) ? '0 : rd_ptr + 1'b1;
          count_q <= count_q - 1'b1;
        end
        2'b11: begin
          mem[wr_ptr] <= din;
          wr_ptr <= (wr_ptr == DEPTH - 1) ? '0 : wr_ptr + 1'b1;
          rd_ptr <= (rd_ptr == DEPTH - 1) ? '0 : rd_ptr + 1'b1;
        end
        default: ;
      endcase
    end
  end

endmodule
