`include "axi_defs.svh"
//------------------------------------------------------------------------------
// axi_owner_fifo — W 通道归属 tag FIFO（移位寄存器式，head 组合输出）
//
// 互联里每 slave 一个实例：AW 握手时 push 被授权 master 的 tag，
// WLAST 握手时 pop；W mux 由 head 驱动。push/pop 同拍合法。
//
// full 在互联用法中不可达（每 master 至多一条在途写流，且 push 时该
// master 的 w_pending=0，故占用者 ≤ N_MASTER-1，深度取 N_MASTER 即可），
// 保留 full 输出仅用于断言/调试。
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
