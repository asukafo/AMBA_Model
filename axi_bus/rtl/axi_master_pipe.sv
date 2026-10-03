`include "axi_defs.svh"
//------------------------------------------------------------------------------
// axi_master_pipe — 可综合流水式多 outstanding AXI4 master（reference design）
//
// 与 axi_master_cfg（单事务阻塞式）互补：
//   - 描述符表（N_DESC 条，start 前由软件逐条写入）顺序执行
//   - 读事务：AR 握手后立即发起下一条，不等待 R 响应（outstanding 流水）
//   - 写事务：AW + W 内联完成（同一 master 的 W 流天然串行，满足互联
//     "每 master 至多一条在途写流"规则），WLAST 后立即发起下一条
//   - B/R 响应按 ID 在槽位表中跟踪（写响应等 B，读响应等 RLAST），
//     全部完成才拉 done
//
// 数据生成（确定性，便于验证）：写第 b 拍 = desc_wdata0 + b，WSTRB 全 1；
// 读校验和 = 所有读拍 rdata 的 XOR 累加。
//
// 约束：同时 outstanding 的事务 ID 必须唯一（B/R 靠 ID 匹配槽位）；
// 描述符表在 busy=0 时写入。
//
// 端口风格：拍平 packed 向量（iverilog 兼容子集），全部可综合。
//------------------------------------------------------------------------------
module axi_master_pipe #(
  parameter int MST_ID     = 0,
  parameter int N_DESC     = 8,
  parameter int N_SLOT     = 8,
  parameter int ADDR_WIDTH = `AXI_ADDR_W,
  parameter int DATA_WIDTH = `AXI_DATA_W,
  parameter int ID_WIDTH   = `AXI_ID_W
) (
  input  logic clk,
  input  logic rstn,
  // ---- 控制 / 描述符表写入（软件侧）----
  input  logic start,           // 启动脉冲：顺序执行描述符 0..cfg_ndesc-1
  input  logic [DW:0] cfg_ndesc,// 本批有效描述符数（1..N_DESC）
  input  logic desc_wr,         // 描述符写使能（busy=0 时使用）
  input  logic [$clog2(N_DESC)-1:0] desc_sel,
  input  logic desc_dir,        // 0=写 1=读
  input  logic [ADDR_WIDTH-1:0] desc_addr,
  input  logic [7:0]            desc_len,
  input  logic [2:0]            desc_size,
  input  logic [1:0]            desc_burst,
  input  logic [ID_WIDTH-1:0]   desc_id,
  input  logic [DATA_WIDTH-1:0] desc_wdata0,
  // ---- 状态（软件侧）----
  output logic busy,
  output logic done,            // 全部事务完成（DONE 状态一拍，组合输出）
  output logic resp_err,        // 有任一响应非 OKAY
  output logic [1:0]            last_err_resp,
  output logic [N_SLOT-1:0]     slot_done,   // 槽位已完成（本批次）
  output logic [N_SLOT*2-1:0]   slot_resp,   // 槽位响应码（[i*2 +: 2]）
  output logic [DATA_WIDTH-1:0] rd_checksum, // 读数据 XOR 累加
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

  localparam DW = $clog2(N_DESC);   // 描述符编号宽度

  // ---- 描述符表 ----
  logic             dt_dir    [0:N_DESC-1];
  logic [ADDR_WIDTH-1:0] dt_addr  [0:N_DESC-1];
  logic [7:0]       dt_len    [0:N_DESC-1];
  logic [2:0]       dt_size   [0:N_DESC-1];
  logic [1:0]       dt_burst  [0:N_DESC-1];
  logic [ID_WIDTH-1:0] dt_id  [0:N_DESC-1];
  logic [DATA_WIDTH-1:0] dt_wdata0 [0:N_DESC-1];

  // ---- 响应槽位表（单一 always_ff 驱动）----
  logic             sl_pend    [0:N_SLOT-1];
  logic             sl_is_wr   [0:N_SLOT-1];
  logic [ID_WIDTH-1:0] sl_id   [0:N_SLOT-1];
  logic [1:0]       sl_resp    [0:N_SLOT-1];
  integer           sl_cnt;              // 待决槽数
  logic [N_SLOT-1:0] slot_done_q;
  logic [N_SLOT*2-1:0] slot_resp_q;
  logic             resp_err_q;
  logic [1:0]       last_err_resp_q;
  logic [DATA_WIDTH-1:0] chk_q;

  // ---- 组合查找 ----
  integer slot_free, slot_b, slot_r;
  logic   slot_b_hit, slot_r_hit;

  always_comb begin
    // 空槽
    slot_free = -1;
    begin
      integer fnd;
      fnd = 0;
      for (int i = 0; i < N_SLOT; i++)
        if (!sl_pend[i] && !fnd) begin
          slot_free = i;
          fnd = 1;
        end
    end
    // B 响应匹配槽
    slot_b = -1;
    begin
      integer fnd;
      fnd = 0;
      for (int i = 0; i < N_SLOT; i++)
        if (sl_pend[i] && sl_is_wr[i] && (sl_id[i] === bid) && !fnd) begin
          slot_b = i;
          fnd = 1;
        end
    end
    slot_b_hit = (slot_b >= 0);
    // R 响应匹配槽
    slot_r = -1;
    begin
      integer fnd;
      fnd = 0;
      for (int i = 0; i < N_SLOT; i++)
        if (sl_pend[i] && !sl_is_wr[i] && (sl_id[i] === rid) && !fnd) begin
          slot_r = i;
          fnd = 1;
        end
    end
    slot_r_hit = (slot_r >= 0);
  end

  //==========================================================================
  // issue FSM：顺序走描述符（与响应处理共用同一 always_ff，避免多驱动）
  //==========================================================================
  localparam I_IDLE = 3'd0, I_NEXT = 3'd1, I_WAW = 3'd2,
             I_WDATA = 3'd3, I_RAR = 3'd4, I_WAIT = 3'd5, I_DONE = 3'd6;
  logic [2:0]      i_state;
  // 宽度需能表示 N_DESC 本身（终值哨兵，比较 desc_idx == N_DESC）
  logic [DW:0]     desc_idx;
  logic [7:0]      w_beat;

  always_comb begin
    awvalid = (i_state == I_WAW);
    awid    = dt_id[desc_idx];
    awaddr  = dt_addr[desc_idx];
    awlen   = dt_len[desc_idx];
    awsize  = dt_size[desc_idx];
    awburst = dt_burst[desc_idx];
    wvalid  = (i_state == I_WDATA);
    wdata   = dt_wdata0[desc_idx] + w_beat;
    // 窄传输：WSTRB 仅在 size 字节窗口内置位（按拍地址对齐）
    wstrb   = axi_strb_for_size(dt_size[desc_idx],
               axi_beat_addr(dt_addr[desc_idx], dt_burst[desc_idx],
                             dt_size[desc_idx], dt_len[desc_idx], w_beat));
    wlast   = (w_beat == dt_len[desc_idx]);
    arvalid = (i_state == I_RAR);
    arid    = dt_id[desc_idx];
    araddr  = dt_addr[desc_idx];
    arlen   = dt_len[desc_idx];
    arsize  = dt_size[desc_idx];
    arburst = dt_burst[desc_idx];
  end

  assign bready = 1'b1;
  assign rready = 1'b1;

  // 本拍是否新登记了一个槽（响应的 sl_cnt 更新需与它抵消）
  logic issue_hs;
  always_comb begin
    issue_hs = ((i_state == I_WAW) && awvalid && awready) ||
               ((i_state == I_RAR) && arvalid && arready);
  end

  always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      i_state  <= I_IDLE;
      desc_idx <= '0;
      w_beat   <= 8'd0;
      for (int i = 0; i < N_DESC; i++) begin
        dt_dir[i]    <= 1'b0;
        dt_addr[i]   <= '0;
        dt_len[i]    <= 8'd0;
        dt_size[i]   <= 3'd0;
        dt_burst[i]  <= 2'd0;
        dt_id[i]     <= '0;
        dt_wdata0[i] <= '0;
      end
      for (int i = 0; i < N_SLOT; i++) begin
        sl_pend[i]  <= 1'b0;
        sl_is_wr[i] <= 1'b0;
        sl_id[i]    <= '0;
        sl_resp[i]  <= 2'b00;
      end
      sl_cnt         <= 0;
      slot_done_q    <= '0;
      slot_resp_q    <= '0;
      resp_err_q     <= 1'b0;
      last_err_resp_q <= 2'b00;
      chk_q          <= '0;
    end else begin
      // ---- 描述符写入 ----
      if (desc_wr) begin
        dt_dir[desc_sel]    <= desc_dir;
        dt_addr[desc_sel]   <= desc_addr;
        dt_len[desc_sel]    <= desc_len;
        dt_size[desc_sel]   <= desc_size;
        dt_burst[desc_sel]  <= desc_burst;
        dt_id[desc_sel]     <= desc_id;
        dt_wdata0[desc_sel] <= desc_wdata0;
      end
      // ---- 批次启动复位 ----
      if (start) begin
        resp_err_q      <= 1'b0;
        last_err_resp_q <= 2'b00;
        slot_done_q     <= '0;
        slot_resp_q     <= '0;
        chk_q           <= '0;
      end
      // ---- issue FSM ----
      case (i_state)
        I_IDLE: if (start) begin
                  desc_idx <= '0;
                  i_state  <= I_NEXT;
                end
        I_NEXT: begin
          if (desc_idx == cfg_ndesc)
            i_state <= I_WAIT;
          // 无空槽理论不可达（outstanding 数 <= N_DESC <= N_SLOT，
          // 且写事务 W 内联完成后才发起下一笔），防御性停在原地
          else if (slot_free >= 0) begin
            if (dt_dir[desc_idx] == 1'b0)
              i_state <= I_WAW;
            else
              i_state <= I_RAR;
          end
        end
        I_WAW: if (awvalid && awready) begin
                 // 登记 B 待决槽
                 sl_pend[slot_free]  <= 1'b1;
                 sl_is_wr[slot_free] <= 1'b1;
                 sl_id[slot_free]    <= dt_id[desc_idx];
                 sl_resp[slot_free]  <= 2'b00;
                 w_beat   <= 8'd0;
                 i_state  <= I_WDATA;
               end
        I_WDATA: if (wvalid && wready) begin
                   if (wlast) begin
                     desc_idx <= desc_idx + 1'b1;
                     i_state  <= I_NEXT;
                   end else begin
                     w_beat <= w_beat + 8'd1;
                   end
                 end
        I_RAR: if (arvalid && arready) begin
                 // 登记 R 待决槽
                 sl_pend[slot_free]  <= 1'b1;
                 sl_is_wr[slot_free] <= 1'b0;
                 sl_id[slot_free]    <= dt_id[desc_idx];
                 sl_resp[slot_free]  <= 2'b00;
                 desc_idx <= desc_idx + 1'b1;
                 i_state  <= I_NEXT;
               end
        I_WAIT: begin
                  // 所有响应槽已清（进入此状态时 desc_idx == N_DESC）
                  if (sl_cnt == 0) i_state <= I_DONE;
                end
        I_DONE: i_state <= I_IDLE;
        default: i_state <= I_IDLE;
      endcase
      // ---- B 响应 ----
      if (bvalid && bready) begin
        if (!slot_b_hit) begin
          $error("axi_master_pipe: BID %0h no pending slot", bid);
        end else begin
          sl_pend[slot_b]  <= 1'b0;
          sl_resp[slot_b]  <= bresp;
          slot_resp_q[slot_b*2 +: 2] <= bresp;
          slot_done_q[slot_b] <= 1'b1;
          if (bresp != `AXI_RESP_OKAY) begin
            resp_err_q      <= 1'b1;
            last_err_resp_q <= bresp;
          end
        end
      end
      // ---- R 响应 ----
      if (rvalid && rready) begin
        if (!slot_r_hit) begin
          $error("axi_master_pipe: RID %0h no pending slot", rid);
        end else begin
          chk_q <= chk_q ^ rdata;
          if (rlast) begin
            sl_pend[slot_r]  <= 1'b0;
            sl_resp[slot_r]  <= rresp;
            slot_resp_q[slot_r*2 +: 2] <= rresp;
            slot_done_q[slot_r] <= 1'b1;
            if (rresp != `AXI_RESP_OKAY) begin
              resp_err_q      <= 1'b1;
              last_err_resp_q <= rresp;
            end
          end
        end
      end
      // ---- 槽位计数：净增量单次 NBA（避免同拍 issue/B/R 相互覆盖）----
      begin
        integer d;
        d = 0;
        if (issue_hs) d = d + 1;
        if (bvalid && bready && slot_b_hit) d = d - 1;
        if (rvalid && rready && slot_r_hit && rlast) d = d - 1;
        sl_cnt <= sl_cnt + d;
      end
    end
  end

  assign done        = (i_state == I_DONE);
  assign busy        = (i_state != I_IDLE);
  assign resp_err    = resp_err_q;
  assign last_err_resp = last_err_resp_q;
  assign slot_done   = slot_done_q;
  assign slot_resp   = slot_resp_q;
  assign rd_checksum = chk_q;

endmodule
