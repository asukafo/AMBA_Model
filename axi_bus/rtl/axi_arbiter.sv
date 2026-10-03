`include "axi_defs.svh"
//------------------------------------------------------------------------------
// axi_arbiter — 通用仲裁器：req[N] -> onehot grant
//   POLICY = 0: 固定优先级（编号小者优先）
//   POLICY = 1: Round-Robin（rr_ptr 从上次授权者的下一位起循环扫描）
//
// 时序约定：
//   - grant 寄存输出，授权后锁存直到 ack（被授权拍的握手）完成
//   - RR 指针仅在握手（ack）时旋转
//   - 相邻两次授权之间有一个空拍（释放 grant 的下一拍才选新请求）
//   - 授权锁存期间 req 变化被忽略；请求方必须保持 req 直到握手
//     （AXI 下即 VALID 必须保持到 READY，由互联与 BFM 共同保证）
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

  // 组合挑选（仅在无授权时被采样）
  reg found;
  always_comb begin
    pick  = '0;
    found = 1'b0;
    if (POLICY == 0) begin
      // 固定优先级：最低编号有效位
      for (int i = 0; i < N; i++) begin
        if (req[i] && !found) begin
          pick[i] = 1'b1;
          found   = 1'b1;
        end
      end
    end else begin
      // RR：从 rr_ptr 起循环扫描
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
