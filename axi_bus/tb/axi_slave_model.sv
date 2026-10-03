`timescale 1ns/1ps
`include "axi_defs.svh"
//------------------------------------------------------------------------------
// axi_slave_model — 可配置 slave 模型（扁平端口，iverilog 兼容子集）
//
// 功能：
//   - 稀疏内存（按 4KB 窗口寻址，窗口由调用方保证落在 [BASE, BASE+MEM_DEPTH)）
//   - AW 队列 + W 拍按 WSTRB 掩码写入 + WLAST 后回 B（BID 回显加宽后的 AWID）
//   - AR 队列 + R 突发响应（数据按拍从内存读出，RVALID 中途不掉）
//   - 随机背压（xorshift32 LFSR，按 % 概率拉低 READY）与响应延迟（拍数）
//   - 配置端口由 TB 在场景间动态调整（S4 乱序、S12 背压等）
//
// 队列全部为定深环形 FIFO（指针 + 计数，单 always_ff 驱动，避免
// 多进程写同一指针）。
//------------------------------------------------------------------------------
module axi_slave_model #(
  parameter int SLV_ID    = 0,
  parameter int AW_Q      = 8,
  parameter int AR_Q      = 8,
  parameter int B_Q       = 8,
  parameter int MEM_DEPTH = 4096,
  parameter int B_DELAY   = 2,
  parameter int SEED      = 1
) (
  input  wire clk,
  input  wire rstn,
  // 背压/延迟配置（TB 运行时改，0-100 / 拍数）
  input  wire [6:0] cfg_aw_rdy_pct,
  input  wire [6:0] cfg_w_rdy_pct,
  input  wire [7:0] cfg_r_delay,
  // AW
  input  wire awvalid,
  input  wire [`AXI_SLV_ID_W-1:0] awid,
  input  wire [`AXI_ADDR_W-1:0]   awaddr,
  input  wire [7:0]               awlen,
  input  wire [2:0]               awsize,
  input  wire [1:0]               awburst,
  output wire awready,
  // W
  input  wire wvalid,
  input  wire [`AXI_DATA_W-1:0]   wdata,
  input  wire [`AXI_DATA_W/8-1:0] wstrb,
  input  wire wlast,
  output wire wready,
  // B
  output wire bvalid,
  output wire [`AXI_SLV_ID_W-1:0] bid,
  output wire [1:0] bresp,
  input  wire bready,
  // AR
  input  wire arvalid,
  input  wire [`AXI_SLV_ID_W-1:0] arid,
  input  wire [`AXI_ADDR_W-1:0]   araddr,
  input  wire [7:0]               arlen,
  input  wire [2:0]               arsize,
  input  wire [1:0]               arburst,
  output wire arready,
  // R
  output reg rvalid,
  output reg [`AXI_SLV_ID_W-1:0] rid,
  output reg [`AXI_DATA_W-1:0]   rdata,
  output reg [1:0]               rresp,
  output reg rlast,
  input  wire rready
);

  // ---- 内存：按 addr[11:0] 寻址（窗口 [BASE, BASE+MEM_DEPTH)）----
  reg [7:0] mem [0:MEM_DEPTH-1];

  // ---- AW 队列 ----
  reg [`AXI_SLV_ID_W-1:0] awq_id    [0:AW_Q-1];
  reg [`AXI_ADDR_W-1:0]   awq_addr  [0:AW_Q-1];
  reg [7:0]               awq_len   [0:AW_Q-1];
  reg [2:0]               awq_size  [0:AW_Q-1];
  reg [1:0]               awq_burst [0:AW_Q-1];
  integer awq_wr, awq_rd, awq_cnt;

  // ---- B 队列 ----
  reg [`AXI_SLV_ID_W-1:0] bq_id [0:B_Q-1];
  integer bq_wr, bq_rd, bq_cnt;

  // ---- AR 队列 ----
  reg [`AXI_SLV_ID_W-1:0] arq_id    [0:AR_Q-1];
  reg [`AXI_ADDR_W-1:0]   arq_addr  [0:AR_Q-1];
  reg [7:0]               arq_len   [0:AR_Q-1];
  reg [2:0]               arq_size  [0:AR_Q-1];
  reg [1:0]               arq_burst [0:AR_Q-1];
  integer arq_wr, arq_rd, arq_cnt;

  // ---- 拍计数 / 延迟 ----
  integer w_beat_cnt, r_beat_cnt;
  integer b_delay_cnt, r_delay_cnt;

  // 组合中间量（always_ff 内先算好再使用）
  reg [`AXI_ADDR_W-1:0] w_addr_c;
  reg [`AXI_ADDR_W-1:0] r_addr_c;

  // ---- 背压 LFSR ----
  reg [31:0] lfsr;

  assign awready = (awq_cnt < AW_Q) && (lfsr[6:0] < cfg_aw_rdy_pct);
  assign wready  = (awq_cnt > 0) && (lfsr[6:0] < cfg_w_rdy_pct);
  assign arready = (arq_cnt < AR_Q) && (lfsr[6:0] < cfg_aw_rdy_pct);

  assign bvalid = (bq_cnt > 0) && (b_delay_cnt >= B_DELAY);
  assign bid    = bq_id[bq_rd];
  assign bresp  = `AXI_RESP_OKAY;

  // 写路径：AW 接收 + W 拍（写内存 / WLAST 时 pop AW、push B）+ B 发送
  always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      awq_wr <= 0; awq_rd <= 0; awq_cnt <= 0;
      bq_wr <= 0; bq_rd <= 0; bq_cnt <= 0;
      w_beat_cnt <= 0; b_delay_cnt <= 0;
    end else begin
      // ---- W 拍 ----
      w_addr_c = axi_beat_addr(awq_addr[awq_rd], awq_burst[awq_rd],
                               awq_size[awq_rd], awq_len[awq_rd], w_beat_cnt);
      if (wvalid && wready) begin
        for (int i = 0; i < `AXI_DATA_W/8; i++) begin
          if (wstrb[i])
            mem[w_addr_c[11:0] + i] <= wdata[8*i +: 8];
        end
        if (wlast) begin
          bq_id[bq_wr] <= awq_id[awq_rd];
          bq_wr <= (bq_wr == B_Q-1) ? 0 : bq_wr + 1;
          bq_cnt <= bq_cnt + 1;
          awq_rd <= (awq_rd == AW_Q-1) ? 0 : awq_rd + 1;
          awq_cnt <= awq_cnt - 1;
          w_beat_cnt <= 0;
        end else begin
          w_beat_cnt <= w_beat_cnt + 1;
        end
      end
      // ---- AW 接收 ----
      if (awvalid && awready) begin
        awq_id[awq_wr]    <= awid;
        awq_addr[awq_wr]  <= awaddr;
        awq_len[awq_wr]   <= awlen;
        awq_size[awq_wr]  <= awsize;
        awq_burst[awq_wr] <= awburst;
        awq_wr <= (awq_wr == AW_Q-1) ? 0 : awq_wr + 1;
        // 若同拍 W 完成上一笔（pop），净变化为 0
        awq_cnt <= (wvalid && wready && wlast) ? awq_cnt : awq_cnt + 1;
      end
      // ---- B 发送 ----
      if (bq_cnt > 0 && b_delay_cnt < B_DELAY) begin
        b_delay_cnt <= b_delay_cnt + 1;
      end else if (bq_cnt > 0 && bvalid && bready) begin
        bq_rd <= (bq_rd == B_Q-1) ? 0 : bq_rd + 1;
        // 若同拍 W 完成新 push 了一个 B，净变化为 0
        bq_cnt <= (wvalid && wready && wlast) ? bq_cnt : bq_cnt - 1;
        b_delay_cnt <= 0;
      end
    end
  end

  // 读路径：R 拍（pop 时机在 RLAST 握手）+ AR 接收 + R 延迟
  always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      arq_wr <= 0; arq_rd <= 0; arq_cnt <= 0;
      r_beat_cnt <= 0; r_delay_cnt <= 0;
    end else begin
      // ---- R 拍 ----
      if (rvalid && rready) begin
        if (rlast) begin
          arq_rd <= (arq_rd == AR_Q-1) ? 0 : arq_rd + 1;
          arq_cnt <= arq_cnt - 1;
          r_beat_cnt <= 0;
        end else begin
          r_beat_cnt <= r_beat_cnt + 1;
        end
        r_delay_cnt <= 0;
      end
      // ---- AR 接收 ----
      if (arvalid && arready) begin
        arq_id[arq_wr]    <= arid;
        arq_addr[arq_wr]  <= araddr;
        arq_len[arq_wr]   <= arlen;
        arq_size[arq_wr]  <= arsize;
        arq_burst[arq_wr] <= arburst;
        arq_wr <= (arq_wr == AR_Q-1) ? 0 : arq_wr + 1;
        // 若同拍 R 完成上一笔（pop），净变化为 0
        arq_cnt <= (rvalid && rready && rlast) ? arq_cnt : arq_cnt + 1;
      end
      // ---- R 首拍延迟 ----
      if (arq_cnt > 0 && r_delay_cnt < cfg_r_delay) begin
        r_delay_cnt <= r_delay_cnt + 1;
      end
    end
  end

  // R 输出（组合）：RVALID 保持到 READY，突发中途不掉
  always_comb begin
    rvalid = (arq_cnt > 0) && (r_delay_cnt >= cfg_r_delay);
    rid    = arq_id[arq_rd];
    rresp  = `AXI_RESP_OKAY;
    rlast  = (r_beat_cnt == arq_len[arq_rd]);
    rdata  = '0;
    r_addr_c = axi_beat_addr(arq_addr[arq_rd], arq_burst[arq_rd],
                             arq_size[arq_rd], arq_len[arq_rd], r_beat_cnt);
    if (rvalid) begin
      // little-endian 拼字
      rdata = {mem[r_addr_c[11:0]+3], mem[r_addr_c[11:0]+2],
               mem[r_addr_c[11:0]+1], mem[r_addr_c[11:0]]};
    end
  end

  // 背压 LFSR（xorshift32）
  always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      lfsr <= SEED + SLV_ID + 1;
    end else begin
      reg [31:0] x;
      x = lfsr;
      x ^= x << 13;
      x ^= x >> 17;
      x ^= x << 5;
      lfsr <= x;
    end
  end

  // 内存清零 + 指针复位
  initial begin
    for (int i = 0; i < MEM_DEPTH; i++)
      mem[i] = 8'h00;
  end

  // scoreboard 比对入口
  function automatic [7:0] get_byte;
    input [11:0] a;
    begin
      get_byte = mem[a];
    end
  endfunction

endmodule
