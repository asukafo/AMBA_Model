`timescale 1ns/1ps
`include "axi_defs.svh"
//------------------------------------------------------------------------------
// axi_master_bfm — 任务级 master BFM（扁平端口，iverilog 兼容子集）
//
// 驱动时序：negedge 驱动，posedge 采样握手。VALID 保持到 READY。
//
// 写模型：至多一条在途写（与互联 w_pending 规则一致），拆分相位任务
//   aw_only / w_only / wait_b 支撑 W 交错与 B 竞争场景。
// 读模型：outstanding 读表（8 槽），ar_only + collect_all；按 RID 分拣，
//   数据自检（期望值由 seed 确定性生成，见 axi_defs.svh）。
//
// 观测输出（场景通过层级引用检查）：
//   last_resp / w_done_time / w_first_hs_time / ar_done_order /
//   ar_recv_resp / ar_done_cnt
//------------------------------------------------------------------------------
module axi_master_bfm #(
  parameter int MST_ID   = 0,
  parameter int MAX_WAIT = 20000
) (
  input  wire clk,
  input  wire rstn,
  // AW
  output reg awvalid,
  output reg [`AXI_ID_W-1:0]   awid,
  output reg [`AXI_ADDR_W-1:0] awaddr,
  output reg [7:0]             awlen,
  output reg [2:0]             awsize,
  output reg [1:0]             awburst,
  input  wire awready,
  // W
  output reg wvalid,
  output reg [`AXI_DATA_W-1:0]   wdata,
  output reg [`AXI_DATA_W/8-1:0] wstrb,
  output reg wlast,
  input  wire wready,
  // B
  input  wire bvalid,
  input  wire [`AXI_ID_W-1:0] bid,
  input  wire [1:0] bresp,
  output reg bready,
  // AR
  output reg arvalid,
  output reg [`AXI_ID_W-1:0]   arid,
  output reg [`AXI_ADDR_W-1:0] araddr,
  output reg [7:0]             arlen,
  output reg [2:0]             arsize,
  output reg [1:0]             arburst,
  input  wire arready,
  // R
  input  wire rvalid,
  input  wire [`AXI_ID_W-1:0] rid,
  input  wire [`AXI_DATA_W-1:0] rdata,
  input  wire [1:0] rresp,
  input  wire rlast,
  output reg rready
);

  // ---- 观测信号（TB 通过层级引用 g_mst[i].bfm.<name> 访问）----
  // 注意：iverilog 无法驱动 unpacked 数组输出端口（静默 X），
  // 观测信号一律放内部，不做端口
  reg [1:0]  last_resp;
  time       w_done_time;
  time       w_first_hs_time;
  reg [7:0]  ar_done_order [0:7];
  integer    ar_recv_resp [0:7];
  integer    ar_done_cnt;

  // 待发送 W 的写（至多一条，与互联 w_pending 规则一致）
  reg                  pw_active;
  reg [`AXI_ID_W-1:0]  pw_id;
  reg [7:0]            pw_len;

  // outstanding 读表（8 槽）
  reg [7:0]              arq_active;
  reg [`AXI_ID_W-1:0]   arq_id    [0:7];
  reg [`AXI_ADDR_W-1:0] arq_addr  [0:7];
  reg [7:0]             arq_len   [0:7];
  reg [2:0]             arq_size  [0:7];
  reg [1:0]             arq_burst [0:7];
  integer               arq_seed      [0:7];
  integer               arq_strb_seed [0:7];
  integer               arq_strb_mode [0:7];  // 0 全1 / 1 窄 / 2 全0
  integer               arq_exp_mode  [0:7];  // 0 期望生成数据 / 1 期望 0
  integer               arq_exp_resp  [0:7];  // 0 OKAY / 3 DECERR
  integer               arq_beat      [0:7];  // 已收拍数

  // 期望读数据：按写时的 strb 掩码合并（未写字节期望 0）
  function automatic [`AXI_DATA_W-1:0] expected_word;
    input integer slot;
    input integer beat;
    reg [`AXI_ADDR_W-1:0] a;
    reg [`AXI_DATA_W-1:0] d;
    reg [`AXI_DATA_W/8-1:0] st;
    integer i;
    begin
    a = axi_beat_addr(arq_addr[slot], arq_burst[slot], arq_size[slot],
                      arq_len[slot], beat);
    d  = axi_test_data(arq_seed[slot], beat);
    st = axi_test_strb(arq_strb_seed[slot], beat, arq_strb_mode[slot]);
    for (i = 0; i < `AXI_DATA_W/8; i = i + 1)
      if (!st[i]) d[8*i +: 8] = 8'h00;
    expected_word = d;
    end
  endfunction

  //--------------------------------------------------------------------------
  // aw_only：发 AW，等握手，登记待发 W
  //--------------------------------------------------------------------------
  task automatic aw_only(
    input [`AXI_ADDR_W-1:0] addr,
    input [1:0] burst,
    input [2:0] size,
    input [7:0] len,
    input [`AXI_ID_W-1:0] id
  );
    integer cnt;
    begin
      if (pw_active) $fatal(1, "BFM %0d: aw_only while previous W unfinished", MST_ID);
      @(negedge clk);
      awvalid <= 1'b1;
      awid    <= id;
      awaddr  <= addr;
      awlen   <= len;
      awsize  <= size;
      awburst <= burst;
      cnt = 0;
      while (!awready) begin
        @(posedge clk);
        cnt = cnt + 1;
        if (cnt > MAX_WAIT) $fatal(1, "BFM %0d: AW timeout", MST_ID);
      end
      // 阻塞赋值：后续 w_only 在同一时刻立即读取（避免 NBA 竞争）
      pw_active = 1'b1;
      pw_id     = id;
      pw_len    = len;
      @(negedge clk);
      awvalid <= 1'b0;
      awid    <= '0;
      awaddr  <= '0;
      awlen   <= '0;
      awsize  <= '0;
      awburst <= '0;
    end
  endtask

  //--------------------------------------------------------------------------
  // w_only：为 pw_active 的写发送 W 拍（数据由 seed 生成）
  //--------------------------------------------------------------------------
  task automatic w_only(input integer seed,
                        input integer strb_seed,
                        input integer strb_mode);
    integer b, cnt;
    begin
      if (!pw_active) $fatal(1, "BFM %0d: w_only without aw_only", MST_ID);
      for (b = 0; b <= pw_len; b = b + 1) begin
        @(negedge clk);
        wvalid <= 1'b1;
        wdata  <= axi_test_data(seed, b);
        wstrb  <= axi_test_strb(strb_seed, b, strb_mode);
        wlast  <= (b == pw_len) ? 1'b1 : 1'b0;
        cnt = 0;
        while (!wready) begin
          @(posedge clk);
          cnt = cnt + 1;
          if (cnt > MAX_WAIT) $fatal(1, "BFM %0d: W timeout", MST_ID);
        end
        if (b == 0)      w_first_hs_time <= $realtime;
        if (b == pw_len) w_done_time     <= $realtime;
      end
      @(negedge clk);
      wvalid <= 1'b0;
      wdata  <= '0;
      wstrb  <= '0;
      wlast  <= 1'b0;
      // 阻塞赋值：后续 aw_only 在同一时刻立即读取
      pw_active = 1'b0;
    end
  endtask

  //--------------------------------------------------------------------------
  // wait_b：等 B 响应，核对 ID，返回 resp
  //--------------------------------------------------------------------------
  task automatic wait_b(input [`AXI_ID_W-1:0] exp_id,
                        output integer resp);
    integer cnt;
    begin
      @(negedge clk);
      bready <= 1'b1;
      cnt = 0;
      while (!bvalid) begin
        @(posedge clk);
        cnt = cnt + 1;
        if (cnt > MAX_WAIT) $fatal(1, "BFM %0d: B timeout", MST_ID);
      end
      if (bid !== exp_id)
        $fatal(1, "BFM %0d: BID %0h != expected %0h", MST_ID, bid, exp_id);
      resp       = bresp;
      last_resp <= bresp;
      @(negedge clk);
      bready <= 1'b0;
    end
  endtask

  //--------------------------------------------------------------------------
  // write：完整写事务（aw_only + w_only + wait_b）
  //--------------------------------------------------------------------------
  task automatic write(
    input [`AXI_ADDR_W-1:0] addr,
    input [1:0] burst,
    input [2:0] size,
    input [7:0] len,
    input [`AXI_ID_W-1:0] id,
    input integer seed,
    input integer strb_seed,
    input integer strb_mode,
    output integer resp
  );
    begin
      aw_only(addr, burst, size, len, id);
      w_only(seed, strb_seed, strb_mode);
      wait_b(id, resp);
    end
  endtask

  //--------------------------------------------------------------------------
  // ar_only：发 AR，等握手，登记 outstanding 读表
  //--------------------------------------------------------------------------
  task automatic ar_only(
    input [`AXI_ADDR_W-1:0] addr,
    input [1:0] burst,
    input [2:0] size,
    input [7:0] len,
    input [`AXI_ID_W-1:0] id,
    input integer seed,
    input integer strb_seed,
    input integer strb_mode,
    input integer exp_mode,
    input integer exp_resp
  );
    integer cnt, slot;
    begin
      @(negedge clk);
      arvalid <= 1'b1;
      arid    <= id;
      araddr  <= addr;
      arlen   <= len;
      arsize  <= size;
      arburst <= burst;
      cnt = 0;
      while (!arready) begin
        @(posedge clk);
        cnt = cnt + 1;
        if (cnt > MAX_WAIT) $fatal(1, "BFM %0d: AR timeout", MST_ID);
      end
      // 找空槽
      slot = -1;
      begin
        integer fnd;
        fnd = 0;
        for (int i = 0; i < 8; i++)
          if (!arq_active[i] && !fnd) begin slot = i; fnd = 1; end
      end
      if (slot < 0) $fatal(1, "BFM %0d: AR table full", MST_ID);
      arq_active[slot]  <= 1'b1;
      arq_id[slot]      <= id;
      arq_addr[slot]    <= addr;
      arq_len[slot]     <= len;
      arq_size[slot]    <= size;
      arq_burst[slot]   <= burst;
      arq_seed[slot]      <= seed;
      arq_strb_seed[slot] <= strb_seed;
      arq_strb_mode[slot] <= strb_mode;
      arq_exp_mode[slot]  <= exp_mode;
      arq_exp_resp[slot]  <= exp_resp;
      arq_beat[slot]      <= 0;
      @(negedge clk);
      arvalid <= 1'b0;
      arid    <= '0;
      araddr  <= '0;
      arlen   <= '0;
      arsize  <= '0;
      arburst <= '0;
    end
  endtask

  //--------------------------------------------------------------------------
  // collect_all：收 R 直到所有 outstanding 读完成。
  //   stall_after > 0 时，在收满 stall_after 拍后停摆 rready stall_cycles 拍
  //   （测授权锁持有与死锁，S14）
  //--------------------------------------------------------------------------
  task automatic collect_all(input integer stall_after,
                             input integer stall_cycles);
    integer cnt, beats, slot, found;
    reg stalled;
    begin
      beats = 0;
      stalled = 1'b0;
      @(negedge clk);
      rready <= 1'b1;
      ar_done_cnt = 0;
      cnt = 0;
      while (arq_active != 8'h00) begin
        @(posedge clk);
        cnt = cnt + 1;
        if (cnt > MAX_WAIT) $fatal(1, "BFM %0d: R timeout", MST_ID);
        if (rvalid && rready) begin
          // 按 RID 找槽
          slot = -1;
          begin
            integer fnd;
            fnd = 0;
            for (int i = 0; i < 8; i++)
              if (arq_active[i] && (arq_id[i] === rid) && !fnd) begin
                slot = i;
                fnd = 1;
              end
          end
          if (slot < 0)
            $fatal(1, "BFM %0d: unexpected RID %0h", MST_ID, rid);
          // 数据自检
          if (arq_exp_mode[slot] == 0) begin
            if (rdata !== expected_word(slot, arq_beat[slot])) begin
              $display("[%0t] BFM%0d mismatch slot=%0d beat=%0d rdata=%0h exp=%0h addr=%0h",
                       $realtime, MST_ID, slot, arq_beat[slot], rdata,
                       expected_word(slot, arq_beat[slot]),
                       axi_beat_addr(arq_addr[slot], arq_burst[slot],
                                     arq_size[slot], arq_len[slot],
                                     arq_beat[slot]));
              $fatal(1, "BFM %0d: R data mismatch (slot %0d beat %0d)",
                     MST_ID, slot, arq_beat[slot]);
            end
          end else if (arq_exp_mode[slot] == 1) begin
            if (rdata !== '0)
              $fatal(1, "BFM %0d: R data not zero (slot %0d)", MST_ID, slot);
          end
          // 收满 stall_after 拍后停摆一次
          beats = beats + 1;
          if (!stalled && stall_after > 0 && beats >= stall_after) begin
            stalled = 1'b1;
            @(negedge clk);
            rready <= 1'b0;
            repeat (stall_cycles) @(posedge clk);
            @(negedge clk);
            rready <= 1'b1;
          end
          if (rlast) begin
            if (rresp !== arq_exp_resp[slot])
              $fatal(1, "BFM %0d: R resp %0h != expected %0h (slot %0d)",
                     MST_ID, rresp, arq_exp_resp[slot], slot);
            ar_recv_resp[slot] <= rresp;
            arq_active[slot]   <= 1'b0;
            ar_done_order[ar_done_cnt] <= slot;
            ar_done_cnt = ar_done_cnt + 1;
          end else begin
            arq_beat[slot] <= arq_beat[slot] + 1;
          end
        end
      end
      @(negedge clk);
      rready <= 1'b0;
    end
  endtask

  //--------------------------------------------------------------------------
  // read：单笔读（无其他 outstanding 时使用）
  //   exp_mode 0 = 期望生成数据（seed/strb 与写入一致）；1 = 期望 0
  //   exp_resp 0 = OKAY；3 = DECERR
  //--------------------------------------------------------------------------
  task automatic read(
    input [`AXI_ADDR_W-1:0] addr,
    input [1:0] burst,
    input [2:0] size,
    input [7:0] len,
    input [`AXI_ID_W-1:0] id,
    input integer seed,
    input integer strb_seed,
    input integer strb_mode,
    input integer exp_mode,
    input integer exp_resp,
    output integer resp
  );
    begin
      ar_only(addr, burst, size, len, id, seed, strb_seed, strb_mode,
              exp_mode, exp_resp);
      collect_all(0, 0);
      resp = ar_recv_resp[0];
      last_resp <= ar_recv_resp[0];
    end
  endtask

  // 复位默认值
  initial begin
    awvalid = 1'b0; wvalid = 1'b0; bready = 1'b0;
    arvalid = 1'b0; rready = 1'b0;
    awid = '0; awaddr = '0; awlen = '0; awsize = '0; awburst = '0;
    wdata = '0; wstrb = '0; wlast = 1'b0;
    arid = '0; araddr = '0; arlen = '0; arsize = '0; arburst = '0;
    last_resp = '0;
    pw_active = 1'b0;
    arq_active = 8'h00;
    ar_done_cnt = 0;
    w_done_time = 0;
    w_first_hs_time = 0;
  end

endmodule
