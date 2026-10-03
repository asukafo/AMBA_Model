`include "axi_defs.svh"
//------------------------------------------------------------------------------
// axi_master_lfsr — 可综合 LFSR 随机流量生成 master（soak 测试源）
//
// 与 cfg/pipe 的差异：事务参数由内部 xorshift32 LFSR 自动生成，无需
// 软件逐笔配置——enable 后持续发出随机方向/地址/长度/宽度的背靠背
// 事务，直到完成 cfg_tx_max 笔。
//
// 事务参数生成（确定性，TB 可复算）：
//   dir = lfsr[0]；addr = cfg_base | (lfsr & cfg_mask)；
//   len = lfsr[7:5]；size = 2（32b）；burst = INCR；
//   id = 已发笔数 % 8（单 outstanding，无碰撞）；wdata0 = lfsr
// 每笔生成时 tx_vld 脉冲一拍，tx_* 输出本笔参数（TB 监听更新参考模型）。
// 写 WSTRB 全 1；读校验和 = 所有读拍 XOR 累加。
//
// 单 outstanding 设计（写完成才发下一笔，读完成才发下一笔）——
// outstanding 流水由 axi_master_pipe 覆盖，本模块专注随机性 soak。
//
// 端口风格：拍平 packed 向量（iverilog 兼容子集），全部可综合。
//------------------------------------------------------------------------------
module axi_master_lfsr #(
  parameter int MST_ID     = 0,
  parameter int ADDR_WIDTH = `AXI_ADDR_W,
  parameter int DATA_WIDTH = `AXI_DATA_W,
  parameter int ID_WIDTH   = `AXI_ID_W
) (
  input  logic clk,
  input  logic rstn,
  // ---- 控制（软件侧）----
  input  logic enable,            // 高电平运行，完成 cfg_tx_max 笔后 done
  input  logic [15:0] cfg_tx_max,
  input  logic [ADDR_WIDTH-1:0] cfg_base,
  input  logic [ADDR_WIDTH-1:0] cfg_mask,   // 地址扰动窗口
  input  logic [31:0] cfg_seed,             // LFSR 初值
  // ---- 状态（软件侧）----
  output logic busy,
  output logic done,              // 完成脉冲（DONE 状态一拍）
  output logic [15:0] tx_cnt,     // 已完成事务数
  output logic resp_err,
  output logic [1:0] last_err_resp,
  output logic [15:0] gen_cnt,     // G_GEN 进入次数（调试/统计）
  output logic [DATA_WIDTH-1:0] rd_checksum,
  // ---- 事务日志（本笔参数，tx_vld 拍有效）----
  output logic tx_vld,
  output logic tx_dir,
  output logic [ADDR_WIDTH-1:0] tx_addr,
  output logic [7:0]            tx_len,
  output logic [2:0]            tx_size,
  output logic [1:0]            tx_burst,
  output logic [ID_WIDTH-1:0]   tx_id,
  output logic [DATA_WIDTH-1:0] tx_wdata0,
  // ---- AXI master 端口（AW）----
  output logic awvalid,
  output logic [ID_WIDTH-1:0]   awid,
  output logic [ADDR_WIDTH-1:0] awaddr,
  output logic [7:0]            awlen,
  output logic [2:0]            awsize,
  output logic [1:0]            awburst,
  input  logic awready,
  // ---- W ----
  output logic wvalid,
  output logic [DATA_WIDTH-1:0]   wdata,
  output logic [DATA_WIDTH/8-1:0] wstrb,
  output logic wlast,
  input  logic wready,
  // ---- B ----
  input  logic bvalid,
  input  logic [ID_WIDTH-1:0] bid,
  input  logic [1:0] bresp,
  output logic bready,
  // ---- AR ----
  output logic arvalid,
  output logic [ID_WIDTH-1:0]   arid,
  output logic [ADDR_WIDTH-1:0] araddr,
  output logic [7:0]            arlen,
  output logic [2:0]            arsize,
  output logic [1:0]            arburst,
  input  logic arready,
  // ---- R ----
  input  logic rvalid,
  input  logic [ID_WIDTH-1:0] rid,
  input  logic [DATA_WIDTH-1:0] rdata,
  input  logic [1:0] rresp,
  input  logic rlast,
  output logic rready
);

  localparam G_IDLE = 3'd0, G_GEN = 3'd1, G_AW = 3'd2, G_WD = 3'd3,
             G_B = 3'd4, G_AR = 3'd5, G_RD = 3'd6, G_DONE = 3'd7;
  logic [2:0] g_state;
  logic [7:0] w_beat;
  logic [31:0] lfsr;
  logic [15:0] tx_cnt_q;
  logic [DATA_WIDTH-1:0] chk_q;
  logic resp_err_q;
  logic [1:0] last_err_resp_q;
  logic run_done_q;   // 本批完成锁存（撤 enable 解锁，防自动重跑）
  logic [15:0] gen_cnt_q;

  // 本笔事务参数（GEN 拍捕获，执行期间保持）
  logic tx_dir_q;
  logic [ADDR_WIDTH-1:0] tx_addr_q;
  logic [7:0]            tx_len_q;
  logic [ID_WIDTH-1:0]   tx_id_q;
  logic [DATA_WIDTH-1:0] tx_wdata0_q;

  always_comb begin
    awvalid = (g_state == G_AW);
    awid    = tx_id_q;
    awaddr  = tx_addr_q;
    awlen   = tx_len_q;
    awsize  = 3'd2;
    awburst = `AXI_BURST_INCR;
    wvalid  = (g_state == G_WD);
    wdata   = tx_wdata0_q + w_beat;
    wstrb   = {(`AXI_DATA_W/8){1'b1}};
    wlast   = (w_beat == tx_len_q);
    bready  = (g_state == G_B);
    arvalid = (g_state == G_AR);
    arid    = tx_id_q;
    araddr  = tx_addr_q;
    arlen   = tx_len_q;
    arsize  = 3'd2;
    arburst = `AXI_BURST_INCR;
    rready  = (g_state == G_RD);
  end

  // 事务日志输出 = 当前参数（tx_vld 为 GEN 拍的寄存一拍脉冲，
  // 与捕获后的 tx_* 寄存器对齐——捕获值与 GEN 拍取同一 lfsr 值）
  logic tx_vld_q;
  assign tx_vld    = tx_vld_q;
  assign tx_dir    = tx_dir_q;
  assign tx_addr   = tx_addr_q;
  assign tx_len    = tx_len_q;
  assign tx_size   = 3'd2;
  assign tx_burst  = `AXI_BURST_INCR;
  assign tx_id     = tx_id_q;
  assign tx_wdata0 = tx_wdata0_q;

  always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      g_state  <= G_IDLE;
      w_beat   <= 8'd0;
      lfsr     <= 32'd1;
      tx_cnt_q <= 16'd0;
      chk_q    <= '0;
      resp_err_q <= 1'b0;
      last_err_resp_q <= 2'b00;
      tx_vld_q <= 1'b0;
      run_done_q <= 1'b0;
      gen_cnt_q <= 16'd0;
    end else begin
      tx_vld_q <= (g_state == G_GEN);
      // 撤 enable 解锁本批完成锁存
      if (!enable) run_done_q <= 1'b0;
      // LFSR 每拍推进
      begin
        logic [31:0] x;
        x = lfsr;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        lfsr <= x;
      end
      case (g_state)
        G_IDLE: if (enable && !run_done_q) begin
                  lfsr     <= cfg_seed;
                  tx_cnt_q <= 16'd0;
                  gen_cnt_q <= 16'd0;
                  chk_q    <= '0;
                  resp_err_q <= 1'b0;
                  last_err_resp_q <= 2'b00;
                  g_state  <= G_GEN;
                end
        G_GEN: begin
                 // 从当前 LFSR 捕获本笔参数
                 gen_cnt_q <= gen_cnt_q + 16'd1;
                 tx_dir_q     <= lfsr[0];
                 tx_addr_q    <= cfg_base | (lfsr & cfg_mask);
                 tx_len_q     <= lfsr[7:5];
                 tx_id_q      <= tx_cnt_q[ID_WIDTH-1:0];
                 tx_wdata0_q  <= lfsr;
                 w_beat       <= 8'd0;
                 g_state      <= lfsr[0] ? G_AR : G_AW;
               end
        G_AW: if (awvalid && awready) g_state <= G_WD;
        G_WD: if (wvalid && wready) begin
                if (wlast) g_state <= G_B;
                else w_beat <= w_beat + 8'd1;
              end
        G_B:  if (bvalid && bready) begin
                if (bresp != `AXI_RESP_OKAY) begin
                  resp_err_q      <= 1'b1;
                  last_err_resp_q <= bresp;
                end
                g_state <= G_DONE;
              end
        G_AR: if (arvalid && arready) g_state <= G_RD;
        G_RD: if (rvalid && rready) begin
                chk_q <= chk_q ^ rdata;
                if (rlast) begin
                  if (rresp != `AXI_RESP_OKAY) begin
                    resp_err_q      <= 1'b1;
                    last_err_resp_q <= rresp;
                  end
                  g_state <= G_DONE;
                end
              end
        G_DONE: begin
                  tx_cnt_q <= tx_cnt_q + 16'd1;
                  if (tx_cnt_q + 16'd1 >= cfg_tx_max) begin
                    run_done_q <= 1'b1;   // 本批完成，锁存
                    g_state <= G_IDLE;
                  end else begin
                    g_state <= G_GEN;
                  end
                end
        default: g_state <= G_IDLE;
      endcase
    end
  end

  // 全部事务完成回到 IDLE（TB 在 done 后撤 enable）
  assign done        = (g_state == G_IDLE) && (tx_cnt_q >= cfg_tx_max);
  assign busy        = (g_state != G_IDLE);
  assign tx_cnt      = tx_cnt_q;
  assign resp_err    = resp_err_q;
  assign last_err_resp = last_err_resp_q;
  assign rd_checksum = chk_q;
  assign gen_cnt = gen_cnt_q;

endmodule
