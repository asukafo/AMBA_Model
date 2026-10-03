`include "axi_defs.svh"
//------------------------------------------------------------------------------
// axi_slave_lat — 可综合高延迟 + 错误注入 AXI4 slave（reference design）
//
// 与 axi_slave_ram（流水式内存）互补：
//   - 可配置 B/R 响应延迟（拍数，运行时寄存器可改）——制造 B/R 仲裁竞争
//   - 可配置 SLVERR 注入：mode 0=正常；1=全部 SLVERR；2=每第 N 笔 SLVERR
//     ——验证错误响应经互联直通
//   - 其余行为与 axi_slave_ram 相同（RAM 存储、定深队列、多笔 outstanding、
//     BID/RID 回显加宽 ID、WSTRB 掩码写入、组合调试读口）
//
// 端口风格：拍平 packed 向量（iverilog 兼容子集），全部可综合。
//------------------------------------------------------------------------------
module axi_slave_lat #(
  parameter int SLV_ID = 0,
  parameter int DEPTH  = 4096,   // 字节数，2 的幂
  parameter int AW_Q   = 4,
  parameter int AR_Q   = 4
) (
  input  wire clk,
  input  wire rstn,
  // ---- 配置（软件侧，运行时生效）----
  input  wire [7:0] cfg_b_delay,    // B 响应延迟（拍）
  input  wire [7:0] cfg_r_delay,    // R 首拍延迟（拍）
  input  wire [1:0] cfg_err_mode,   // 0 正常 / 1 全 SLVERR / 2 每第 N 笔
  input  wire [7:0] cfg_err_period, // mode 2 的周期 N
  // ---- AW ----
  input  wire awvalid,
  input  wire [`AXI_SLV_ID_W-1:0] awid,
  input  wire [`AXI_ADDR_W-1:0]   awaddr,
  input  wire [7:0]               awlen,
  input  wire [2:0]               awsize,
  input  wire [1:0]               awburst,
  output wire awready,
  // ---- W ----
  input  wire wvalid,
  input  wire [`AXI_DATA_W-1:0]   wdata,
  input  wire [`AXI_DATA_W/8-1:0] wstrb,
  input  wire wlast,
  output wire wready,
  // ---- B ----
  output wire bvalid,
  output wire [`AXI_SLV_ID_W-1:0] bid,
  output wire [1:0] bresp,
  input  wire bready,
  // ---- AR ----
  input  wire arvalid,
  input  wire [`AXI_SLV_ID_W-1:0] arid,
  input  wire [`AXI_ADDR_W-1:0]   araddr,
  input  wire [7:0]               arlen,
  input  wire [2:0]               arsize,
  input  wire [1:0]               arburst,
  output wire arready,
  // ---- R ----
  output reg rvalid,
  output reg [`AXI_SLV_ID_W-1:0] rid,
  output reg [`AXI_DATA_W-1:0]   rdata,
  output reg [1:0]               rresp,
  output reg rlast,
  input  wire rready,
  // ---- 调试读口（TB 校验内存，组合读出）----
  input  wire [`AXI_ADDR_W-1:0]   dbg_addr,
  output wire [7:0]               dbg_byte
);

  localparam A_W = $clog2(DEPTH);

  reg [7:0] mem [0:DEPTH-1];
  initial begin
    for (int i = 0; i < DEPTH; i++)
      mem[i] = 8'h00;
  end

  // ---- AW 队列 ----
  reg [`AXI_SLV_ID_W-1:0] awq_id    [0:AW_Q-1];
  reg [`AXI_ADDR_W-1:0]   awq_addr  [0:AW_Q-1];
  reg [7:0]               awq_len   [0:AW_Q-1];
  reg [2:0]               awq_size  [0:AW_Q-1];
  reg [1:0]               awq_burst [0:AW_Q-1];
  reg [$clog2(AW_Q)-1:0]  awq_wr, awq_rd;
  reg [$clog2(AW_Q+1)-1:0] awq_cnt;

  // ---- B 队列（ID + 错误标记）----
  reg [`AXI_SLV_ID_W-1:0] bq_id   [0:AW_Q-1];
  reg                     bq_err  [0:AW_Q-1];
  reg [$clog2(AW_Q)-1:0]  bq_wr, bq_rd;
  reg [$clog2(AW_Q+1)-1:0] bq_cnt;

  // ---- AR 队列 ----
  reg [`AXI_SLV_ID_W-1:0] arq_id    [0:AR_Q-1];
  reg [`AXI_ADDR_W-1:0]   arq_addr  [0:AR_Q-1];
  reg [7:0]               arq_len   [0:AR_Q-1];
  reg [2:0]               arq_size  [0:AR_Q-1];
  reg [1:0]               arq_burst [0:AR_Q-1];
  reg [$clog2(AR_Q)-1:0]  arq_wr, arq_rd;
  reg [$clog2(AR_Q+1)-1:0] arq_cnt;

  // ---- 拍计数 / 延迟 / 错误注入 ----
  reg [7:0]               w_beat_cnt, r_beat_cnt;
  reg [7:0]               b_delay_cnt, r_delay_cnt;
  reg [7:0]               b_txn_cnt, r_txn_cnt;   // 已完成事务计数（错误周期）
  reg [`AXI_ADDR_W-1:0]   w_addr_c, r_addr_c;

  // 错误判定（组合）：mode 1 全错；mode 2 每第 N 笔错（计数从 1 起）
  reg b_err_c, r_err_c;
  always_comb begin
    if (cfg_err_mode == 2'd1) begin
      b_err_c = 1'b1;
      r_err_c = 1'b1;
    end else if (cfg_err_mode == 2'd2) begin
      b_err_c = (b_txn_cnt == cfg_err_period - 1);
      r_err_c = (r_txn_cnt == cfg_err_period - 1);
    end else begin
      b_err_c = 1'b0;
      r_err_c = 1'b0;
    end
  end

  assign awready = (awq_cnt < AW_Q);
  assign wready  = (awq_cnt > 0);
  assign arready = (arq_cnt < AR_Q);
  assign bvalid  = (bq_cnt > 0) && (b_delay_cnt >= cfg_b_delay);
  assign bid     = bq_id[bq_rd];
  assign bresp   = bq_err[bq_rd] ? `AXI_RESP_SLVERR : `AXI_RESP_OKAY;

  //==========================================================================
  // 写路径：W 拍写 RAM（AW 顺序）/ AW 接收 / B 发送（延迟 + 错误）
  //==========================================================================
  always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      awq_wr <= '0; awq_rd <= '0; awq_cnt <= '0;
      bq_wr  <= '0; bq_rd  <= '0; bq_cnt  <= '0;
      w_beat_cnt <= 8'd0;
      b_delay_cnt <= 8'd0;
      b_txn_cnt <= 8'd0;
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
          bq_err[bq_wr] <= b_err_c;
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
        awq_wr <= (awq_wr == AW_Q-1) ? '0 : awq_wr + 1'b1;
        awq_cnt <= (wvalid && wready && wlast) ? awq_cnt : awq_cnt + 1'b1;
      end
      // ---- B 发送（延迟 + 完成计数）----
      if (bq_cnt > 0 && b_delay_cnt < cfg_b_delay) begin
        b_delay_cnt <= b_delay_cnt + 8'd1;
      end else if (bq_cnt > 0 && bvalid && bready) begin
        bq_rd <= (bq_rd == AW_Q-1) ? '0 : bq_rd + 1'b1;
        bq_cnt <= (wvalid && wready && wlast) ? bq_cnt : bq_cnt - 1'b1;
        b_delay_cnt <= 8'd0;
        if (b_txn_cnt == cfg_err_period - 1) b_txn_cnt <= 8'd0;
        else b_txn_cnt <= b_txn_cnt + 8'd1;
      end
    end
  end

  //==========================================================================
  // 读路径：AR 接收 / R 拍（延迟 + 错误，RLAST 握手时 pop）
  //==========================================================================
  always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      arq_wr <= '0; arq_rd <= '0; arq_cnt <= '0;
      r_beat_cnt <= 8'd0;
      r_delay_cnt <= 8'd0;
      r_txn_cnt <= 8'd0;
    end else begin
      // ---- R 拍 ----
      if (rvalid && rready) begin
        if (rlast) begin
          arq_rd  <= (arq_rd == AR_Q-1) ? '0 : arq_rd + 1'b1;
          arq_cnt <= arq_cnt - 1'b1;
          r_beat_cnt <= 8'd0;
          if (r_txn_cnt == cfg_err_period - 1) r_txn_cnt <= 8'd0;
          else r_txn_cnt <= r_txn_cnt + 8'd1;
        end else begin
          r_beat_cnt <= r_beat_cnt + 8'd1;
        end
        r_delay_cnt <= 8'd0;
      end
      // ---- AR 接收 ----
      if (arvalid && arready) begin
        arq_id[arq_wr]    <= arid;
        arq_addr[arq_wr]  <= araddr;
        arq_len[arq_wr]   <= arlen;
        arq_size[arq_wr]  <= arsize;
        arq_burst[arq_wr] <= arburst;
        arq_wr <= (arq_wr == AR_Q-1) ? '0 : arq_wr + 1'b1;
        arq_cnt <= (rvalid && rready && rlast) ? arq_cnt : arq_cnt + 1'b1;
      end
      // ---- R 首拍延迟 ----
      if (arq_cnt > 0 && r_delay_cnt < cfg_r_delay) begin
        r_delay_cnt <= r_delay_cnt + 8'd1;
      end
    end
  end

  // ---- R 输出（组合）：延迟后置起，RVALID 保持到 READY，突发中途不掉 ----
  always_comb begin
    rvalid = (arq_cnt > 0) && (r_delay_cnt >= cfg_r_delay);
    rid    = arq_id[arq_rd];
    rresp  = r_err_c ? `AXI_RESP_SLVERR : `AXI_RESP_OKAY;
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
