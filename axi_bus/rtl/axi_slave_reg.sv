`include "axi_defs.svh"
//------------------------------------------------------------------------------
// axi_slave_reg — 可综合阻塞式寄存器外设 AXI4 slave（reference design）
//
// 典型外设风格，与 axi_slave_ram / axi_slave_lat（流水式队列）互补：
//   - 单事务阻塞：读写共用 busy，AW/AR 仅空闲时接收（AW 与 AR 同拍竞争
//     时 AW 优先），上一笔完成才接下一笔——考验互联的授权锁与背压
//   - 32b 寄存器堆（N_REG 个），WSTRB 字节使能部分写（读改写合并）
//   - 支持多拍突发（拍地址逐拍递增，INCR/WRAP/FIXED 均按 axi_beat_addr）
//   - B/R 无额外延迟；BID/RID 原样回显加宽后的 ID
//   - 组合调试读口（TB 校验寄存器内容）
//
// 端口风格：拍平 packed 向量（iverilog 兼容子集），全部可综合。
//------------------------------------------------------------------------------
module axi_slave_reg #(
  parameter int SLV_ID = 0,
  parameter int N_REG  = 64     // 32b 寄存器数，2 的幂
) (
  input  wire clk,
  input  wire rstn,
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
  output wire rvalid,
  output wire [`AXI_SLV_ID_W-1:0] rid,
  output reg [`AXI_DATA_W-1:0]   rdata,
  output wire [1:0]               rresp,
  output wire rlast,
  input  wire rready,
  // ---- 调试读口（TB 校验寄存器）----
  input  wire [$clog2(N_REG)-1:0] dbg_sel,
  output wire [`AXI_DATA_W-1:0]   dbg_val
);

  localparam IDX_W = $clog2(N_REG) + 2;   // 字节地址窗口宽度

  reg [`AXI_DATA_W-1:0] regs [0:N_REG-1];

  // ---- 事务寄存器 ----
  reg [`AXI_SLV_ID_W-1:0] aw_id_q;
  reg [`AXI_ADDR_W-1:0]   aw_addr_q;
  reg [7:0]               aw_len_q;
  reg [2:0]               aw_size_q;
  reg [1:0]               aw_burst_q;
  reg [7:0]               w_beat_q;
  reg [`AXI_SLV_ID_W-1:0] ar_id_q;
  reg [`AXI_ADDR_W-1:0]   ar_addr_q;
  reg [7:0]               ar_len_q;
  reg [2:0]               ar_size_q;
  reg [1:0]               ar_burst_q;
  reg [7:0]               r_beat_q;

  reg [`AXI_ADDR_W-1:0]   w_addr_c, r_addr_c;
  reg [`AXI_DATA_W-1:0]   w_merged;

  // ---- FSM ----
  localparam W_IDLE = 2'd0, W_AW = 2'd1, W_DATA = 2'd2, W_B = 2'd3;
  localparam R_IDLE = 2'd0, R_AR = 2'd1, R_DATA = 2'd2;
  reg [1:0] w_state, r_state;

  // 读写共用 busy：AW 与 AR 同拍竞争时 AW 优先
  assign awready = (w_state == W_IDLE) && (r_state == R_IDLE);
  assign arready = (r_state == R_IDLE) && (w_state == W_IDLE) && !awvalid;
  assign wready  = (w_state == W_DATA);
  assign bvalid  = (w_state == W_B);
  assign bid     = aw_id_q;
  assign bresp   = `AXI_RESP_OKAY;
  assign rvalid  = (r_state == R_DATA);
  assign rid     = ar_id_q;
  assign rresp   = `AXI_RESP_OKAY;
  assign rlast   = (r_beat_q == ar_len_q);

  always_comb begin
    w_addr_c = axi_beat_addr(aw_addr_q, aw_burst_q, aw_size_q, aw_len_q, w_beat_q);
    r_addr_c = axi_beat_addr(ar_addr_q, ar_burst_q, ar_size_q, ar_len_q, r_beat_q);
    rdata = regs[r_addr_c[IDX_W-1:2]];
  end

  //==========================================================================
  // 单一 always_ff：写 FSM（含寄存器写）+ 读 FSM
  //==========================================================================
  always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      w_state <= W_IDLE;
      r_state <= R_IDLE;
      for (int i = 0; i < N_REG; i++)
        regs[i] <= '0;
      w_beat_q <= 8'd0;
      r_beat_q <= 8'd0;
    end else begin
      // ---- 写 FSM ----
      case (w_state)
        W_IDLE: if (awvalid && awready) begin
                  aw_id_q    <= awid;
                  aw_addr_q  <= awaddr;
                  aw_len_q   <= awlen;
                  aw_size_q  <= awsize;
                  aw_burst_q <= awburst;
                  w_beat_q   <= 8'd0;
                  w_state    <= W_DATA;
                end
        W_DATA: if (wvalid && wready) begin
                  // 读改写合并字节使能（先读旧值，合并后整字写回）
                  w_merged = regs[w_addr_c[IDX_W-1:2]];
                  for (int i = 0; i < `AXI_DATA_W/8; i++)
                    if (wstrb[i]) w_merged[8*i +: 8] = wdata[8*i +: 8];
                  regs[w_addr_c[IDX_W-1:2]] <= w_merged;
                  if (wlast) w_state <= W_B;
                  else w_beat_q <= w_beat_q + 8'd1;
                end
        W_B:   if (bvalid && bready) w_state <= W_IDLE;
        default: w_state <= W_IDLE;
      endcase
      // ---- 读 FSM ----
      case (r_state)
        R_IDLE: if (arvalid && arready) begin
                  ar_id_q    <= arid;
                  ar_addr_q  <= araddr;
                  ar_len_q   <= arlen;
                  ar_size_q  <= arsize;
                  ar_burst_q <= arburst;
                  r_beat_q   <= 8'd0;
                  r_state    <= R_DATA;
                end
        R_DATA: if (rvalid && rready) begin
                  if (rlast) r_state <= R_IDLE;
                  else r_beat_q <= r_beat_q + 8'd1;
                end
        default: r_state <= R_IDLE;
      endcase
    end
  end

  // ---- 调试读口 ----
  assign dbg_val = regs[dbg_sel];

endmodule
