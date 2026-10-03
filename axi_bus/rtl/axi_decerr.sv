`include "axi_defs.svh"
//------------------------------------------------------------------------------
// axi_decerr — 内部 DECERR 响应器（互联里每 master 一份），扁平端口
//
// 写：IDLE -> DRAIN -> BRESP 三段 FSM
//   IDLE   收未命中的 AW（awready=1）
//   DRAIN  无条件 wready=1 吞 W 拍（w_drain_done 在 WLAST 握手拍脉冲，
//          互联用它清 w_pending）
//   BRESP  bvalid=1，b_id=aw_id，b_resp=DECERR，等 bready
//   AXI 要求 B 必须在 WLAST 之后，FSM 天然保证。
//
// 读：AR ID FIFO（复用 axi_owner_fifo），每笔回一拍 R=DECERR、RLAST=1。
//
// 与 W mux 的互斥由互联的"每 master 至多一条在途写流"规则保证：
// 本模块处于 DRAIN 时，该 master 的 W 拍不可能同时被某个 slave 的
// W mux 选中（w_pending 置位期间 AW 仲裁请求被屏蔽）。
//------------------------------------------------------------------------------
module axi_decerr #(
  parameter int AR_Q_DEPTH = `AXI_DECERR_Q
) (
  input  logic                  clk,
  input  logic                  rstn,
  // 写地址（互联已把 awvalid 门控为 未命中 && !w_pending）
  input  logic                  awvalid,
  input  logic [`AXI_ID_W-1:0]  aw_id,
  output logic                  awready,
  // 写数据（直接从 master 的 W 通道吞拍，只看 valid/last）
  input  logic                  wvalid,
  input  logic                  w_last,
  output logic                  wready,
  output logic                  w_drain_done,
  // 写响应
  output logic                  bvalid,
  output logic [`AXI_ID_W-1:0]  b_id,
  output logic [1:0]            b_resp,
  input  logic                  bready,
  // 读地址（互联已把 arvalid 门控为未命中）
  input  logic                  arvalid,
  input  logic [`AXI_ID_W-1:0]  ar_id,
  output logic                  arready,
  // 读数据（单拍，DECERR，RLAST=1）
  output logic                  rvalid,
  output logic [`AXI_ID_W-1:0]  r_id,
  output logic [1:0]            r_resp,
  input  logic                  rready
);

  // ---- 写路径 FSM ----
  localparam W_IDLE = 2'd0, W_DRAIN = 2'd1, W_BRESP = 2'd2;
  logic [1:0]           wstate;
  logic [`AXI_ID_W-1:0] aw_id_q;

  assign awready      = (wstate == W_IDLE);
  assign wready       = (wstate == W_DRAIN);
  assign w_drain_done = (wstate == W_DRAIN) && wvalid && wready && w_last;

  assign bvalid = (wstate == W_BRESP);
  assign b_id   = aw_id_q;
  assign b_resp = `AXI_RESP_DECERR;

  always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      wstate  <= W_IDLE;
      aw_id_q <= '0;
    end else begin
      case (wstate)
        W_IDLE: begin
          if (awvalid && awready) begin
            aw_id_q <= aw_id;
            wstate  <= W_DRAIN;
          end
        end
        W_DRAIN: begin
          if (wvalid && wready && w_last)
            wstate <= W_BRESP;
        end
        W_BRESP: begin
          if (bvalid && bready)
            wstate <= W_IDLE;
        end
        default: wstate <= W_IDLE;
      endcase
    end
  end

  // ---- 读路径：AR ID FIFO，每笔回一拍 DECERR ----
  logic                 rf_empty, rf_full;
  logic [`AXI_ID_W-1:0] rf_head;

  assign arready = !rf_full;
  assign rvalid  = !rf_empty;
  assign r_id    = rf_head;
  assign r_resp  = `AXI_RESP_DECERR;

  axi_owner_fifo #(
    .DEPTH (AR_Q_DEPTH),
    .TAG_W (`AXI_ID_W)
  ) rid_fifo (
    .clk   (clk),
    .rstn  (rstn),
    .push  (arvalid && arready),
    .din   (ar_id),
    .pop   (rvalid && rready),
    .head  (rf_head),
    .empty (rf_empty),
    .full  (rf_full),
    .count ()
  );

endmodule
