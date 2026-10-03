`include "axi_defs.svh"
//------------------------------------------------------------------------------
// axi_slave_ram — 可综合 AXI4 RAM slave（reference design）
//
// 真 RAM 存储（DEPTH 字节，2 的幂），支持多笔 outstanding（定深队列）：
//   - AW 队列接收写地址；W 拍按 WSTRB 掩码写入当前条目（按 AW 顺序），
//     WLAST 后把加宽 AWID 压入 B 队列，B 无延迟回显
//   - AR 队列接收读地址；R 突发按拍从 RAM 读出（组合读口），RVALID
//     保持到 RREADY，中途不掉
//   - 突发地址计算支持 INCR/WRAP/FIXED（axi_beat_addr，可综合）
//
// 地址窗口：addr[$clog2(DEPTH)-1:0] 索引内存（调用方保证窗口内）。
//
// 端口风格：拍平 packed 向量（iverilog 兼容子集），全部可综合。
//------------------------------------------------------------------------------
module axi_slave_ram #(
  parameter int SLV_ID = 0,
  parameter int DEPTH  = 4096,   // 字节数，2 的幂
  parameter int AW_Q   = 4,
  parameter int AR_Q   = 4
) (
  input  logic clk,
  input  logic rstn,
  // ---- AW ----
  input  logic awvalid,
  input  logic [`AXI_SLV_ID_W-1:0] awid,
  input  logic [`AXI_ADDR_W-1:0]   awaddr,
  input  logic [7:0]               awlen,
  input  logic [2:0]               awsize,
  input  logic [1:0]               awburst,
  input  logic                     awlock,   // 互斥写（AWLOCK=1）
  output logic awready,
  // ---- W ----
  input  logic wvalid,
  input  logic [`AXI_DATA_W-1:0]   wdata,
  input  logic [`AXI_DATA_W/8-1:0] wstrb,
  input  logic wlast,
  output logic wready,
  // ---- B ----
  output logic bvalid,
  output logic [`AXI_SLV_ID_W-1:0] bid,
  output logic [1:0] bresp,
  input  logic bready,
  // ---- AR ----
  input  logic arvalid,
  input  logic [`AXI_SLV_ID_W-1:0] arid,
  input  logic [`AXI_ADDR_W-1:0]   araddr,
  input  logic [7:0]               arlen,
  input  logic [2:0]               arsize,
  input  logic [1:0]               arburst,
  input  logic                     arlock,   // 互斥读（ARLOCK=1）
  output logic arready,
  // ---- R ----
  output logic rvalid,
  output logic [`AXI_SLV_ID_W-1:0] rid,
  output logic [`AXI_DATA_W-1:0]   rdata,
  output logic [1:0]               rresp,
  output logic rlast,
  input  logic rready,
  // ---- 调试读口（TB 校验内存，组合读出）----
  input  logic [`AXI_ADDR_W-1:0]   dbg_addr,
  output logic [7:0]               dbg_byte
);

  localparam A_W = $clog2(DEPTH);

  logic [7:0] mem [0:DEPTH-1];
  initial begin
    for (int i = 0; i < DEPTH; i++)
      mem[i] = 8'h00;
  end

  // ---- AW 队列 ----
  logic [`AXI_SLV_ID_W-1:0] awq_id    [0:AW_Q-1];
  logic [`AXI_ADDR_W-1:0]   awq_addr  [0:AW_Q-1];
  logic [7:0]               awq_len   [0:AW_Q-1];
  logic [2:0]               awq_size  [0:AW_Q-1];
  logic [1:0]               awq_burst [0:AW_Q-1];
  logic                     awq_exok  [0:AW_Q-1];  // 互斥写成功标记
  logic [$clog2(AW_Q)-1:0]  awq_wr, awq_rd;
  logic [$clog2(AW_Q+1)-1:0] awq_cnt;

  // ---- B 队列（ID + 互斥成功标记）----
  logic [`AXI_SLV_ID_W-1:0] bq_id  [0:AW_Q-1];
  logic                     bq_exok [0:AW_Q-1];
  logic [$clog2(AW_Q)-1:0]  bq_wr, bq_rd;
  logic [$clog2(AW_Q+1)-1:0] bq_cnt;

  // ---- AR 队列 ----
  logic [`AXI_SLV_ID_W-1:0] arq_id    [0:AR_Q-1];
  logic [`AXI_ADDR_W-1:0]   arq_addr  [0:AR_Q-1];
  logic [7:0]               arq_len   [0:AR_Q-1];
  logic [2:0]               arq_size  [0:AR_Q-1];
  logic [1:0]               arq_burst [0:AR_Q-1];
  logic                     arq_excl  [0:AR_Q-1];  // 互斥读标记
  logic [$clog2(AR_Q)-1:0]  arq_wr, arq_rd;
  logic [$clog2(AR_Q+1)-1:0] arq_cnt;

  // ---- 互斥监视器（简化版：全局单一监视点，非按地址）----
  logic                     excl_own;
  logic [`AXI_SLV_ID_W-1:0] excl_id;

  // ---- 拍计数 / 组合地址 ----
  logic [7:0]               w_beat_cnt, r_beat_cnt;
  logic [`AXI_ADDR_W-1:0]   w_addr_c, r_addr_c;

  assign awready = (awq_cnt < AW_Q);
  assign wready  = (awq_cnt > 0);
  assign arready = (arq_cnt < AR_Q);
  assign bvalid  = (bq_cnt > 0);
  assign bid     = bq_id[bq_rd];
  assign bresp   = bq_exok[bq_rd] ? `AXI_RESP_EXOKAY : `AXI_RESP_OKAY;

  //==========================================================================
  // 写路径：W 拍写 RAM（AW 顺序）/ AW 接收 / B 发送
  //==========================================================================
  always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      awq_wr <= '0; awq_rd <= '0; awq_cnt <= '0;
      bq_wr  <= '0; bq_rd  <= '0; bq_cnt  <= '0;
      w_beat_cnt <= 8'd0;
      excl_own <= 1'b0;
      excl_id  <= '0;
    end else begin
      // ---- W 拍 ----
      w_addr_c = axi_beat_addr(awq_addr[awq_rd], awq_burst[awq_rd],
                               awq_size[awq_rd], awq_len[awq_rd], w_beat_cnt);
      if (wvalid && wready) begin
        for (int i = 0; i < `AXI_DATA_W/8; i++) begin
          if (wstrb[i])
            // lane 映射：lane i 写到字对齐基址 + i（AXI 窄传输规则）
            mem[{w_addr_c[A_W-1:2], 2'b00} + i] <= wdata[8*i +: 8];
        end
        if (wlast) begin
          bq_id[bq_wr]  <= awq_id[awq_rd];
          bq_exok[bq_wr] <= awq_exok[awq_rd];
          bq_wr  <= (bq_wr == AW_Q-1) ? '0 : bq_wr + 1'b1;
          bq_cnt <= bq_cnt + 1'b1;
          awq_rd  <= (awq_rd == AW_Q-1) ? '0 : awq_rd + 1'b1;
          awq_cnt <= awq_cnt - 1'b1;
          w_beat_cnt <= 8'd0;
        end else begin
          w_beat_cnt <= w_beat_cnt + 8'd1;
        end
      end
      // ---- AW 接收 ----
      if (awvalid && awready) begin
        awq_id[awq_wr]    <= awid;
        awq_addr[awq_wr]  <= awaddr;
        awq_len[awq_wr]   <= awlen;
        awq_size[awq_wr]  <= awsize;
        awq_burst[awq_wr] <= awburst;
        // 互斥写判定：监视点仍归本 master 且 AWLOCK=1 → EXOKAY；
        // 任何写（含普通写）都会消耗/清除监视点
        awq_exok[awq_wr]  <= awlock && excl_own && (excl_id === awid);
        excl_own <= 1'b0;
        awq_wr <= (awq_wr == AW_Q-1) ? '0 : awq_wr + 1'b1;
        // 若同拍 W 完成上一笔（pop），净变化为 0
        awq_cnt <= (wvalid && wready && wlast) ? awq_cnt : awq_cnt + 1'b1;
      end
      // ---- B 发送 ----
      if (bq_cnt > 0 && bvalid && bready) begin
        bq_rd <= (bq_rd == AW_Q-1) ? '0 : bq_rd + 1'b1;
        // 若同拍 W 完成新压入一笔，净变化为 0
        bq_cnt <= (wvalid && wready && wlast) ? bq_cnt : bq_cnt - 1'b1;
      end
    end
  end

  //==========================================================================
  // 读路径：AR 接收 / R 拍（RLAST 握手时 pop）
  //==========================================================================
  always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      arq_wr <= '0; arq_rd <= '0; arq_cnt <= '0;
      r_beat_cnt <= 8'd0;
    end else begin
      // ---- R 拍 ----
      if (rvalid && rready) begin
        if (rlast) begin
          arq_rd  <= (arq_rd == AR_Q-1) ? '0 : arq_rd + 1'b1;
          arq_cnt <= arq_cnt - 1'b1;
          r_beat_cnt <= 8'd0;
        end else begin
          r_beat_cnt <= r_beat_cnt + 8'd1;
        end
      end
      // ---- AR 接收 ----
      if (arvalid && arready) begin
        arq_id[arq_wr]    <= arid;
        arq_addr[arq_wr]  <= araddr;
        arq_len[arq_wr]   <= arlen;
        arq_size[arq_wr]  <= arsize;
        arq_burst[arq_wr] <= arburst;
        // 互斥读：建立监视点（简化全局单点），响应回 EXOKAY
        arq_excl[arq_wr] <= arlock;
        if (arlock) begin
          excl_own <= 1'b1;
          excl_id  <= arid;
        end
        arq_wr <= (arq_wr == AR_Q-1) ? '0 : arq_wr + 1'b1;
        // 若同拍 R 完成上一笔（pop），净变化为 0
        arq_cnt <= (rvalid && rready && rlast) ? arq_cnt : arq_cnt + 1'b1;
      end
    end
  end

  // ---- R 输出（组合）：RVALID 保持到 READY，突发中途不掉 ----
  always_comb begin
    rvalid = (arq_cnt > 0);
    rid    = arq_id[arq_rd];
    rresp  = arq_excl[arq_rd] ? `AXI_RESP_EXOKAY : `AXI_RESP_OKAY;
    rlast  = (r_beat_cnt == arq_len[arq_rd]);
    r_addr_c = axi_beat_addr(arq_addr[arq_rd], arq_burst[arq_rd],
                             arq_size[arq_rd], arq_len[arq_rd], r_beat_cnt);
    // little-endian 拼字
    // lane 映射：lane L 读字对齐基址 + L（AXI 窄传输规则）
    rdata = {mem[{r_addr_c[A_W-1:2], 2'b00}+3], mem[{r_addr_c[A_W-1:2], 2'b00}+2],
             mem[{r_addr_c[A_W-1:2], 2'b00}+1], mem[{r_addr_c[A_W-1:2], 2'b00}]};
  end

  // ---- 调试读口 ----
  assign dbg_byte = mem[dbg_addr[A_W-1:0]];

endmodule
