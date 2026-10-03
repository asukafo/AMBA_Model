`include "axi_defs.svh"
//------------------------------------------------------------------------------
// axi_reg_slice — 前向寄存器片（skid buffer），AXI 任意单通道打拍
//
// 行为：输出寄存一拍（切断前向组合路径），背压时用 skid 寄存器捕获
// 一拍数据（不丢拍）：
//   - 空：直通（dout = din，din_ready = dout_ready）
//   - 有 skid 数据：dout = q，din_ready = 0（先排空）
//   - 捕获：din_valid && !dout_ready 时存入 q
//
// 协议透明：任意 valid/payload/ready 单通道均可插入（AXI 的 5 通道、
// 任意握手序列语义不变，只增加一拍延迟）。
//
// 端口风格：拍平 packed 向量（iverilog 兼容子集），全部可综合。
//------------------------------------------------------------------------------
module axi_reg_slice #(
  parameter int W = 64    // payload 宽度
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

  // 关键：din_ready 恒等于 dout_ready —— 排空拍与上游握手在同一沿完成，
  // 上游"完成当前拍"与下游"消费该拍"严格同步，杜绝同一拍被直通重放
  //（上游只在握手后前进到下一拍，排空时其当前拍与 q 中拍是同一拍）
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
        // skid 数据排空
        if (dout_ready) q_valid <= 1'b0;
      end else begin
        // 直通被打断：捕获
        if (din_valid && !dout_ready) begin
          q_valid <= 1'b1;
          q_data  <= din;
        end
      end
    end
  end

endmodule
