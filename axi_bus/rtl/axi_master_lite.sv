`include "axi_defs.svh"
//------------------------------------------------------------------------------
// axi_master_lite — 可综合 AXI4-Lite 风格 master（reference design）
//
// 严格 AXI4-Lite 子集：单拍（len=0）、固定 32b（size=2）、INCR、
// ID 恒 0、无突发——寄存器访问的典型主端形态。验证互联对 Lite 子集
// master 的兼容性（互联侧 len/size/burst/id 由 TB 接常量）。
//
// 单 outstanding 阻塞式：写 = AW→W(1 拍)→B；读 = AR→R(1 拍)；
// 读写共用 busy，AW 与 AR 同拍竞争时 AW 优先。WSTRB 由配置给出
// （部分写寄存器用）。
//
// 端口风格：拍平 packed 向量（iverilog 兼容子集），全部可综合。
//------------------------------------------------------------------------------
module axi_master_lite #(
  parameter int MST_ID     = 0,
  parameter int ADDR_WIDTH = `AXI_ADDR_W,
  parameter int DATA_WIDTH = `AXI_DATA_W
) (
  input  logic clk,
  input  logic rstn,
  // ---- 配置 / 状态（软件侧）----
  input  logic start_wr,
  input  logic start_rd,
  input  logic [ADDR_WIDTH-1:0] cfg_addr,
  input  logic [DATA_WIDTH-1:0] cfg_wdata,
  input  logic [DATA_WIDTH/8-1:0] cfg_wstrb,
  input  logic [2:0] cfg_prot,
  output logic busy,
  output logic wr_done,           // DONE 状态一拍（组合输出）
  output logic rd_done,
  output logic [1:0] wr_status,
  output logic [1:0] rd_status,
  output logic [DATA_WIDTH-1:0] rd_data,
  // ---- AXI 端口（Lite 子集，无 len/size/burst/id）----
  output logic awvalid,
  output logic [ADDR_WIDTH-1:0] awaddr,
  output logic [2:0]            awprot,
  input  logic awready,
  output logic wvalid,
  output logic [DATA_WIDTH-1:0]   wdata,
  output logic [DATA_WIDTH/8-1:0] wstrb,
  input  logic wready,
  input  logic bvalid,
  input  logic [1:0] bresp,
  output logic bready,
  output logic arvalid,
  output logic [ADDR_WIDTH-1:0] araddr,
  output logic [2:0]            arprot,
  input  logic arready,
  input  logic rvalid,
  input  logic [DATA_WIDTH-1:0] rdata,
  input  logic [1:0] rresp,
  output logic rready
);

  localparam L_IDLE = 2'd0, L_AW = 2'd1, L_WD = 2'd2, L_B = 2'd3;
  localparam R_IDLE = 2'd0, R_AR = 2'd1, R_RD = 2'd2;
  logic [1:0] l_state, r_state;
  logic [1:0] wr_status_q, rd_status_q;
  logic [DATA_WIDTH-1:0] rd_data_q;
  logic wr_done_q, rd_done_q;   // 寄存完成脉冲（轮询在次拍可见）

  // AW 与 AR 同拍竞争时 AW 优先（阻塞式，一次一笔）
  always_comb begin
    awvalid = (l_state == L_AW);
    awaddr  = cfg_addr;
    awprot  = cfg_prot;
    wvalid  = (l_state == L_WD);
    wdata   = cfg_wdata;
    wstrb   = cfg_wstrb;
    bready  = (l_state == L_B);
    arvalid = (r_state == R_AR) && (l_state == L_IDLE);
    araddr  = cfg_addr;
    arprot  = cfg_prot;
    rready  = (r_state == R_RD);
  end

  always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      l_state <= L_IDLE;
      r_state <= R_IDLE;
      wr_status_q <= 2'b00;
      rd_status_q <= 2'b00;
      rd_data_q   <= '0;
      wr_done_q   <= 1'b0;
      rd_done_q   <= 1'b0;
    end else begin
      wr_done_q <= 1'b0;
      rd_done_q <= 1'b0;
      // ---- 写 FSM ----
      case (l_state)
        L_IDLE: if (start_wr) l_state <= L_AW;
        L_AW:   if (awvalid && awready) l_state <= L_WD;
        L_WD:   if (wvalid && wready) l_state <= L_B;
        L_B:    if (bvalid && bready) begin
                  wr_status_q <= bresp;
                  wr_done_q   <= 1'b1;
                  l_state <= L_IDLE;
                end
        default: l_state <= L_IDLE;
      endcase
      // ---- 读 FSM ----
      case (r_state)
        R_IDLE: if (start_rd && (l_state == L_IDLE)) r_state <= R_AR;
        R_AR:   if (arvalid && arready) r_state <= R_RD;
        R_RD:   if (rvalid && rready) begin
                  rd_status_q <= rresp;
                  rd_data_q   <= rdata;
                  rd_done_q   <= 1'b1;
                  r_state <= R_IDLE;
                end
        default: r_state <= R_IDLE;
      endcase
    end
  end

  assign wr_done  = wr_done_q;
  assign rd_done  = rd_done_q;
  assign busy     = (l_state != L_IDLE) || (r_state != R_IDLE);
  assign wr_status = wr_status_q;
  assign rd_status = rd_status_q;
  assign rd_data   = rd_data_q;

endmodule
