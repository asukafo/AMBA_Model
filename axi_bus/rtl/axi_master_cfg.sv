`include "axi_defs.svh"
//------------------------------------------------------------------------------
// axi_master_cfg — 可综合 AXI4 master（reference design）
//
// 寄存器配置 + 状态机驱动：写事务 = AW -> W 突发 -> 收 B；
// 读事务 = AR -> 收 R 突发。读写两路 FSM 独立，可同时进行。
//
// 事务数据生成（确定性，便于验证）：
//   写第 b 拍数据 = cfg_wdata0 + b（WSTRB 全 1）
//   读校验 = 所有读拍的 rdata 异或累加（rd_checksum），TB 侧可复算比对
//
// 配置约定：cfg_* 必须在 busy=0 时写入，并在整个事务期间保持稳定
// （start 脉冲的下一拍起事务拥有配置值）。
//
// 端口风格：拍平 packed 向量（iverilog 兼容子集，见 axi_interconnect.sv
// 头部注释），全部可综合。
//------------------------------------------------------------------------------
module axi_master_cfg #(
  parameter int MST_ID     = 0,
  parameter int ADDR_WIDTH = `AXI_ADDR_W,
  parameter int DATA_WIDTH = `AXI_DATA_W,
  parameter int ID_WIDTH   = `AXI_ID_W
) (
  input  logic clk,
  input  logic rstn,
  // ---- 配置 / 状态（软件侧）----
  input  logic                  wr_start,   // 写事务启动脉冲
  input  logic                  rd_start,   // 读事务启动脉冲
  input  logic [ADDR_WIDTH-1:0] cfg_addr,
  input  logic [7:0]            cfg_len,    // 突发长度（len+1 拍）
  input  logic [2:0]            cfg_size,
  input  logic [1:0]            cfg_burst,
  input  logic [ID_WIDTH-1:0]   cfg_id,
  input  logic [DATA_WIDTH-1:0] cfg_wdata0, // 拍 0 写数据，后续拍递增
  input  logic [DATA_WIDTH/8-1:0] cfg_wstrb, // 写字节使能（默认全 1）
  output logic                  busy,
  output logic                  wr_done,    // 写事务完成脉冲
  output logic                  rd_done,    // 读事务完成脉冲
  output logic [1:0]            wr_status,  // 写响应码
  output logic [1:0]            rd_status,  // 读响应码（末拍）
  output logic [DATA_WIDTH-1:0] rd_checksum,// 读数据异或累加
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

  //==========================================================================
  // 写 FSM：IDLE -> AW -> DATA -> RESP -> DONE
  //==========================================================================
  localparam W_IDLE = 3'd0, W_AW = 3'd1, W_DATA = 3'd2,
             W_RESP = 3'd3, W_DONE = 3'd4;
  logic [2:0] w_state;
  logic [7:0] w_beat;

  always_comb begin
    awvalid = (w_state == W_AW);
    awid    = cfg_id;
    awaddr  = cfg_addr;
    awlen   = cfg_len;
    awsize  = cfg_size;
    awburst = cfg_burst;
    wvalid  = (w_state == W_DATA);
    wdata   = cfg_wdata0 + w_beat;   // 拍 b 数据 = base + b
    wstrb   = cfg_wstrb;
    wlast   = (w_beat == cfg_len);
    bready  = (w_state == W_RESP);
  end

  always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      w_state <= W_IDLE;
      w_beat  <= 8'd0;
      wr_status <= 2'b00;
    end else begin
      case (w_state)
        W_IDLE: if (wr_start) w_state <= W_AW;
        W_AW:   if (awvalid && awready) begin
                  w_beat  <= 8'd0;
                  w_state <= W_DATA;
                end
        W_DATA: if (wvalid && wready) begin
                  if (wlast) w_state <= W_RESP;
                  else w_beat <= w_beat + 8'd1;
                end
        W_RESP: if (bvalid && bready) begin
                  wr_status <= bresp;
                  w_state   <= W_DONE;
                end
        // DONE 状态保持一拍（wr_done 组合输出可见），随后回 IDLE
        W_DONE: w_state <= W_IDLE;
        default: w_state <= W_IDLE;
      endcase
    end
  end

  //==========================================================================
  // 读 FSM：IDLE -> AR -> DATA -> DONE
  //==========================================================================
  localparam R_IDLE = 2'd0, R_AR = 2'd1, R_DATA = 2'd2, R_DONE = 2'd3;
  logic [1:0] r_state;
  logic [DATA_WIDTH-1:0] chk_q;

  always_comb begin
    arvalid = (r_state == R_AR);
    arid    = cfg_id;
    araddr  = cfg_addr;
    arlen   = cfg_len;
    arsize  = cfg_size;
    arburst = cfg_burst;
    rready  = (r_state == R_DATA);
  end

  always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      r_state  <= R_IDLE;
      rd_status <= 2'b00;
      chk_q    <= '0;
    end else begin
      case (r_state)
        R_IDLE: if (rd_start) begin
                  chk_q   <= '0;
                  r_state <= R_AR;
                end
        R_AR:   if (arvalid && arready) r_state <= R_DATA;
        R_DATA: if (rvalid && rready) begin
                  chk_q <= chk_q ^ rdata;
                  if (rlast) begin
                    rd_status <= rresp;
                    r_state   <= R_DONE;
                  end
                end
        // DONE 状态保持一拍（rd_done 组合输出可见），随后回 IDLE
        R_DONE: r_state <= R_IDLE;
        default: r_state <= R_IDLE;
      endcase
    end
  end

  // done 为 DONE 状态组合输出（与 status 同拍可见）
  assign wr_done = (w_state == W_DONE);
  assign rd_done = (r_state == R_DONE);
  assign rd_checksum = chk_q;
  assign busy = (w_state != W_IDLE) || (r_state != R_IDLE);

endmodule
