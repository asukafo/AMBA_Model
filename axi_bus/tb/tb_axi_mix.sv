`include "axi_defs.svh"
//------------------------------------------------------------------------------
// tb_axi_mix — 混合拓扑 RTL 级联调：
//   master0 = axi_master_cfg（阻塞式单事务）
//   master1 = axi_master_pipe（流水式多 outstanding，描述符表驱动）
//   slave0  = axi_slave_ram（流水式内存）
//   slave1  = axi_slave_lat（高延迟 + SLVERR 注入）
//
// 检查手段：master 状态码 / pipe 槽位响应与错误标记 / 读校验和 /
// 参考内存逐字节比对（两 slave 调试读口）/ 超时看门狗。
//
// 场景：
//   M1 pipe 批量流水（8 描述符读写交错、跨两 slave、唯一 ID）
//   M2 cfg → lat slave：正常写读回 + 全 SLVERR 模式错误检查
//   M3 并发：cfg→ram 写 同时 pipe→lat 读
//   M4 两 master 抢 lat slave（延迟制造 B 竞争）
//   M5 pipe 随机描述符多轮（seeded，读回自检）
//------------------------------------------------------------------------------
module tb_axi_mix;
  timeunit 1ns / 1ps;

  localparam MAX_WAIT = 100000;

  // ---- 时钟 / 复位 ----
  logic clk;
  logic rstn;
  initial clk = 1'b0;
  always #5 clk = ~clk;

  // ---- 互联连线 + DUT（共用）----
  `include "tb_axi_wires.svh"
  // 直通属性默认 0（互斥/窄传输场景由专属 master 驱动对应位）
  initial begin
    s_awlock = '0; s_awcache = '0; s_awprot = '0;
    s_awqos = '0; s_awregion = '0;
    s_arlock = '0; s_arcache = '0; s_arprot = '0;
    s_arqos = '0; s_arregion = '0;
  end


  // ---- master0（cfg）配置（标量连线）----
  logic cfg0_wr_start, cfg0_rd_start;
  logic [`AXI_ADDR_W-1:0]   cfg0_addr;
  logic [7:0]               cfg0_len;
  logic [2:0]               cfg0_size;
  logic [1:0]               cfg0_burst;
  logic [`AXI_ID_W-1:0]     cfg0_id;
  logic [`AXI_DATA_W-1:0]   cfg0_wdata0;
  logic [`AXI_DATA_W/8-1:0] cfg0_wstrb;

  // ---- master1（pipe）描述符连线 ----
  logic pipe_start, desc_wr;
  logic [3:0] pipe_ndesc;
  logic [$clog2(8)-1:0] desc_sel;
  logic desc_dir;
  logic [`AXI_ADDR_W-1:0] desc_addr;
  logic [7:0]            desc_len;
  logic [2:0]            desc_size;
  logic [1:0]            desc_burst;
  logic [`AXI_ID_W-1:0]  desc_id;
  logic [`AXI_DATA_W-1:0] desc_wdata0;

  // ---- lat slave 配置 ----
  logic [7:0] lat_b_delay, lat_r_delay;
  logic [1:0] lat_err_mode;
  logic [7:0] lat_err_period;

  // ---- 调试读口 ----
  logic [`AXI_ADDR_W-1:0] dbg_addr0, dbg_addr1;
  logic [7:0] dbg_byte0_c, dbg_byte1_c;

  // ---- master0：cfg ----
  axi_master_cfg #(
    .MST_ID (0)
  ) mst0 (
    .clk (clk), .rstn (rstn),
    .wr_start (cfg0_wr_start),
    .rd_start (cfg0_rd_start),
    .cfg_addr (cfg0_addr),
    .cfg_len (cfg0_len),
    .cfg_size (cfg0_size),
    .cfg_burst (cfg0_burst),
    .cfg_id (cfg0_id),
    .cfg_wdata0 (cfg0_wdata0),
    .cfg_wstrb (cfg0_wstrb),
    .awvalid (s_awvalid[0]),
    .awid (s_awid[0*`AXI_ID_W +: `AXI_ID_W]),
    .awaddr (s_awaddr[0*`AXI_ADDR_W +: `AXI_ADDR_W]),
    .awlen (s_awlen[0*8 +: 8]),
    .awsize (s_awsize[0*3 +: 3]),
    .awburst (s_awburst[0*2 +: 2]),
    .awready (s_awready[0]),
    .wvalid (s_wvalid[0]),
    .wdata (s_wdata[0*`AXI_DATA_W +: `AXI_DATA_W]),
    .wstrb (s_wstrb[0*`AXI_DATA_W/8 +: `AXI_DATA_W/8]),
    .wlast (s_wlast[0]),
    .wready (s_wready[0]),
    .bvalid (s_bvalid[0]),
    .bid (s_bid[0*`AXI_ID_W +: `AXI_ID_W]),
    .bresp (s_bresp[0*2 +: 2]),
    .bready (s_bready[0]),
    .arvalid (s_arvalid[0]),
    .arid (s_arid[0*`AXI_ID_W +: `AXI_ID_W]),
    .araddr (s_araddr[0*`AXI_ADDR_W +: `AXI_ADDR_W]),
    .arlen (s_arlen[0*8 +: 8]),
    .arsize (s_arsize[0*3 +: 3]),
    .arburst (s_arburst[0*2 +: 2]),
    .arready (s_arready[0]),
    .rvalid (s_rvalid[0]),
    .rid (s_rid[0*`AXI_ID_W +: `AXI_ID_W]),
    .rdata (s_rdata[0*`AXI_DATA_W +: `AXI_DATA_W]),
    .rresp (s_rresp[0*2 +: 2]),
    .rlast (s_rlast[0]),
    .rready (s_rready[0])
  );

  // ---- master1：pipe ----
  axi_master_pipe #(
    .MST_ID (1),
    .N_DESC (8),
    .N_SLOT (8)
  ) mst1 (
    .clk (clk), .rstn (rstn),
    .start (pipe_start),
    .cfg_ndesc (pipe_ndesc),
    .desc_wr (desc_wr),
    .desc_sel (desc_sel),
    .desc_dir (desc_dir),
    .desc_addr (desc_addr),
    .desc_len (desc_len),
    .desc_size (desc_size),
    .desc_burst (desc_burst),
    .desc_id (desc_id),
    .desc_wdata0 (desc_wdata0),
    .awvalid (s_awvalid[1]),
    .awid (s_awid[1*`AXI_ID_W +: `AXI_ID_W]),
    .awaddr (s_awaddr[1*`AXI_ADDR_W +: `AXI_ADDR_W]),
    .awlen (s_awlen[1*8 +: 8]),
    .awsize (s_awsize[1*3 +: 3]),
    .awburst (s_awburst[1*2 +: 2]),
    .awready (s_awready[1]),
    .wvalid (s_wvalid[1]),
    .wdata (s_wdata[1*`AXI_DATA_W +: `AXI_DATA_W]),
    .wstrb (s_wstrb[1*`AXI_DATA_W/8 +: `AXI_DATA_W/8]),
    .wlast (s_wlast[1]),
    .wready (s_wready[1]),
    .bvalid (s_bvalid[1]),
    .bid (s_bid[1*`AXI_ID_W +: `AXI_ID_W]),
    .bresp (s_bresp[1*2 +: 2]),
    .bready (s_bready[1]),
    .arvalid (s_arvalid[1]),
    .arid (s_arid[1*`AXI_ID_W +: `AXI_ID_W]),
    .araddr (s_araddr[1*`AXI_ADDR_W +: `AXI_ADDR_W]),
    .arlen (s_arlen[1*8 +: 8]),
    .arsize (s_arsize[1*3 +: 3]),
    .arburst (s_arburst[1*2 +: 2]),
    .arready (s_arready[1]),
    .rvalid (s_rvalid[1]),
    .rid (s_rid[1*`AXI_ID_W +: `AXI_ID_W]),
    .rdata (s_rdata[1*`AXI_DATA_W +: `AXI_DATA_W]),
    .rresp (s_rresp[1*2 +: 2]),
    .rlast (s_rlast[1]),
    .rready (s_rready[1])
  );

  // ---- slave0：ram ----
  axi_slave_ram #(
    .SLV_ID (0)
  ) slv0 (
    .clk (clk), .rstn (rstn),
    .awvalid (m_awvalid[0]),
    .awid (m_awid[0*`AXI_SLV_ID_W +: `AXI_SLV_ID_W]),
    .awaddr (m_awaddr[0*`AXI_ADDR_W +: `AXI_ADDR_W]),
    .awlen (m_awlen[0*8 +: 8]),
    .awsize (m_awsize[0*3 +: 3]),
    .awburst (m_awburst[0*2 +: 2]),
    .awlock (m_awlock[0*2]),
    .awready (m_awready[0]),
    .wvalid (m_wvalid[0]),
    .wdata (m_wdata[0*`AXI_DATA_W +: `AXI_DATA_W]),
    .wstrb (m_wstrb[0*`AXI_DATA_W/8 +: `AXI_DATA_W/8]),
    .wlast (m_wlast[0]),
    .wready (m_wready[0]),
    .bvalid (m_bvalid[0]),
    .bid (m_bid[0*`AXI_SLV_ID_W +: `AXI_SLV_ID_W]),
    .bresp (m_bresp[0*2 +: 2]),
    .bready (m_bready[0]),
    .arvalid (m_arvalid[0]),
    .arid (m_arid[0*`AXI_SLV_ID_W +: `AXI_SLV_ID_W]),
    .araddr (m_araddr[0*`AXI_ADDR_W +: `AXI_ADDR_W]),
    .arlen (m_arlen[0*8 +: 8]),
    .arsize (m_arsize[0*3 +: 3]),
    .arburst (m_arburst[0*2 +: 2]),
    .arlock (m_arlock[0*2]),
    .arready (m_arready[0]),
    .rvalid (m_rvalid[0]),
    .rid (m_rid[0*`AXI_SLV_ID_W +: `AXI_SLV_ID_W]),
    .rdata (m_rdata[0*`AXI_DATA_W +: `AXI_DATA_W]),
    .rresp (m_rresp[0*2 +: 2]),
    .rlast (m_rlast[0]),
    .rready (m_rready[0]),
    .dbg_addr (dbg_addr0),
    .dbg_byte (dbg_byte0_c)
  );

  // ---- slave1：lat ----
  axi_slave_lat #(
    .SLV_ID (1)
  ) slv1 (
    .clk (clk), .rstn (rstn),
    .cfg_b_delay (lat_b_delay),
    .cfg_r_delay (lat_r_delay),
    .cfg_err_mode (lat_err_mode),
    .cfg_err_period (lat_err_period),
    .awvalid (m_awvalid[1]),
    .awid (m_awid[1*`AXI_SLV_ID_W +: `AXI_SLV_ID_W]),
    .awaddr (m_awaddr[1*`AXI_ADDR_W +: `AXI_ADDR_W]),
    .awlen (m_awlen[1*8 +: 8]),
    .awsize (m_awsize[1*3 +: 3]),
    .awburst (m_awburst[1*2 +: 2]),
    .awready (m_awready[1]),
    .wvalid (m_wvalid[1]),
    .wdata (m_wdata[1*`AXI_DATA_W +: `AXI_DATA_W]),
    .wstrb (m_wstrb[1*`AXI_DATA_W/8 +: `AXI_DATA_W/8]),
    .wlast (m_wlast[1]),
    .wready (m_wready[1]),
    .bvalid (m_bvalid[1]),
    .bid (m_bid[1*`AXI_SLV_ID_W +: `AXI_SLV_ID_W]),
    .bresp (m_bresp[1*2 +: 2]),
    .bready (m_bready[1]),
    .arvalid (m_arvalid[1]),
    .arid (m_arid[1*`AXI_SLV_ID_W +: `AXI_SLV_ID_W]),
    .araddr (m_araddr[1*`AXI_ADDR_W +: `AXI_ADDR_W]),
    .arlen (m_arlen[1*8 +: 8]),
    .arsize (m_arsize[1*3 +: 3]),
    .arburst (m_arburst[1*2 +: 2]),
    .arready (m_arready[1]),
    .rvalid (m_rvalid[1]),
    .rid (m_rid[1*`AXI_SLV_ID_W +: `AXI_SLV_ID_W]),
    .rdata (m_rdata[1*`AXI_DATA_W +: `AXI_DATA_W]),
    .rresp (m_rresp[1*2 +: 2]),
    .rlast (m_rlast[1]),
    .rready (m_rready[1]),
    .dbg_addr (dbg_addr1),
    .dbg_byte (dbg_byte1_c)
  );

  //--------------------------------------------------------------------------
  // 参考内存
  //--------------------------------------------------------------------------
  localparam SB_DEPTH = `AXI_M_SLAVE * 4096;
  logic [7:0] ref_mem [0:SB_DEPTH-1];
  initial begin
    for (int i = 0; i < SB_DEPTH; i++)
      ref_mem[i] = 8'h00;
  end

  function automatic integer slave_of(input logic [`AXI_ADDR_W-1:0] a);
    if ((a & 32'hF000_0000) == 32'h0000_0000) slave_of = 0;
    else if ((a & 32'hF000_0000) == 32'h1000_0000) slave_of = 1;
    else slave_of = -1;
  endfunction

  // 参考内存更新（写第 b 拍 = base + b；字节放置与 slave 的 lane 映射
  // 完全一致：strobe lane i → 字对齐基址 + i 的字节）。
  // use_explicit=1 时用显式 strobe（cfg master 自定义 WSTRB 场景）
  task automatic ref_update(
    input logic [`AXI_ADDR_W-1:0] addr,
    input logic [1:0] burst,
    input logic [2:0] size,
    input logic [7:0] len,
    input logic [`AXI_DATA_W-1:0] base,
    input logic use_explicit,
    input logic [`AXI_DATA_W/8-1:0] explicit_strb
  );
    integer s;
    begin
      s = slave_of(addr);
      if (s >= 0) begin
        for (int b = 0; b <= len; b++) begin
          logic [`AXI_ADDR_W-1:0] a;
          logic [`AXI_DATA_W-1:0] d;
          logic [`AXI_DATA_W/8-1:0] st;
          a  = axi_beat_addr(addr, burst, size, len, b);
          d  = base + b;
          st = use_explicit ? explicit_strb : axi_strb_for_size(size, a);
          for (int i = 0; i < `AXI_DATA_W/8; i++)
            if (st[i])
              ref_mem[s*4096 + ((a[11:0] & 12'hFFC) + i)] = d[8*i +: 8];
        end
      end
    end
  endtask

  // 从 slave 调试口读一个字
  task automatic dbg_word(input integer s, input logic [`AXI_ADDR_W-1:0] a,
                          output logic [`AXI_DATA_W-1:0] w);
    begin
      if (s == 0) begin
        dbg_addr0 = a;     #1; w[7:0]   = dbg_byte0_c;
        dbg_addr0 = a + 1; #1; w[15:8]  = dbg_byte0_c;
        dbg_addr0 = a + 2; #1; w[23:16] = dbg_byte0_c;
        dbg_addr0 = a + 3; #1; w[31:24] = dbg_byte0_c;
      end else begin
        dbg_addr1 = a;     #1; w[7:0]   = dbg_byte1_c;
        dbg_addr1 = a + 1; #1; w[15:8]  = dbg_byte1_c;
        dbg_addr1 = a + 2; #1; w[23:16] = dbg_byte1_c;
        dbg_addr1 = a + 3; #1; w[31:24] = dbg_byte1_c;
      end
    end
  endtask

  // 参考内存 vs 两 slave 调试口逐字节比对
  task automatic mem_compare();
    begin
      for (int s = 0; s < `AXI_M_SLAVE; s++) begin
        for (int i = 0; i < 4096; i++) begin
          logic [`AXI_ADDR_W-1:0] a;
          logic [7:0] b;
          a = (s == 0) ? 32'h0000_0000 + i : 32'h1000_0000 + i;
          if (s == 0) begin dbg_addr0 = a; #1; b = dbg_byte0_c; end
          else        begin dbg_addr1 = a; #1; b = dbg_byte1_c; end
          if (ref_mem[s*4096 + i] !== b) begin
            $display("FAIL: mem_compare slave %0d addr 0x%0h: ref=%0h dut=%0h",
                     s, a, ref_mem[s*4096+i], b);
            $fatal(1);
          end
        end
      end
    end
  endtask

  // 期望校验和（读返回字 = 字对齐基址 + lane，与 slave 的 lane 映射一致）
  task automatic exp_checksum(
    input logic [`AXI_ADDR_W-1:0] addr,
    input logic [1:0] burst, input logic [2:0] size, input logic [7:0] len,
    output logic [`AXI_DATA_W-1:0] chk
  );
    integer s;
    logic [`AXI_DATA_W-1:0] acc;
    begin
      s = slave_of(addr);
      acc = '0;
      if (s >= 0) begin
        for (int b = 0; b <= len; b++) begin
          logic [`AXI_ADDR_W-1:0] a;
          logic [`AXI_ADDR_W-1:0] wb;
          a  = axi_beat_addr(addr, burst, size, len, b);
          wb = a & ~(`AXI_ADDR_W'(3));
          acc = acc ^ {ref_mem[s*4096 + wb[11:0] + 3],
                       ref_mem[s*4096 + wb[11:0] + 2],
                       ref_mem[s*4096 + wb[11:0] + 1],
                       ref_mem[s*4096 + wb[11:0]]};
        end
      end
      chk = acc;
    end
  endtask

  task automatic chk(input integer cond, input [1023:0] msg);
    if (!cond) begin
      $display("FAIL: %s (t=%0t)", msg, $time);
      $fatal(1);
    end
  endtask

  //--------------------------------------------------------------------------
  // 驱动：cfg master 写/读（master0）
  //--------------------------------------------------------------------------
  task automatic mst0_wr(input logic [`AXI_ADDR_W-1:0] addr,
                         input logic [7:0] len, input logic [2:0] size,
                         input logic [1:0] burst, input logic [`AXI_ID_W-1:0] id,
                         input logic [`AXI_DATA_W-1:0] base,
                         output logic [1:0] status);
    integer cnt;
    begin
      cfg0_addr   = addr;
      cfg0_len    = len;
      cfg0_size   = size;
      cfg0_burst  = burst;
      cfg0_id     = id;
      cfg0_wdata0 = base;
      cfg0_wstrb  = {(`AXI_DATA_W/8){1'b1}};
      @(negedge clk);
      cfg0_wr_start = 1'b1;
      @(negedge clk);
      cfg0_wr_start = 1'b0;
      cnt = 0;
      while (!mst0.wr_done) begin
        @(posedge clk);
        cnt = cnt + 1;
        if (cnt > MAX_WAIT) $fatal(1, "MIX: cfg write timeout");
      end
      status = mst0.wr_status;
    end
  endtask

  task automatic mst0_rd(input logic [`AXI_ADDR_W-1:0] addr,
                         input logic [7:0] len, input logic [2:0] size,
                         input logic [1:0] burst, input logic [`AXI_ID_W-1:0] id,
                         output logic [1:0] status,
                         output logic [`AXI_DATA_W-1:0] checksum);
    integer cnt;
    begin
      cfg0_addr  = addr;
      cfg0_len   = len;
      cfg0_size  = size;
      cfg0_burst = burst;
      cfg0_id    = id;
      @(negedge clk);
      cfg0_rd_start = 1'b1;
      @(negedge clk);
      cfg0_rd_start = 1'b0;
      cnt = 0;
      while (!mst0.rd_done) begin
        @(posedge clk);
        cnt = cnt + 1;
        if (cnt > MAX_WAIT) $fatal(1, "MIX: cfg read timeout");
      end
      status   = mst0.rd_status;
      checksum = mst0.rd_checksum;
    end
  endtask

  //--------------------------------------------------------------------------
  // 驱动：pipe master（master1）
  //--------------------------------------------------------------------------
  task automatic desc_put(input integer sel, input logic dir,
                          input logic [`AXI_ADDR_W-1:0] addr,
                          input logic [7:0] len, input logic [2:0] size,
                          input logic [1:0] burst, input logic [`AXI_ID_W-1:0] id,
                          input logic [`AXI_DATA_W-1:0] base);
    begin
      desc_sel    = sel;   // 隐式截断（int 部分选择有 iverilog elab bug）
      desc_dir    = dir;
      desc_addr   = addr;
      desc_len    = len;
      desc_size   = size;
      desc_burst  = burst;
      desc_id     = id;
      desc_wdata0 = base;
      @(negedge clk);
      desc_wr = 1'b1;
      @(negedge clk);
      desc_wr = 1'b0;
    end
  endtask

  task automatic pipe_run();
    integer cnt;
    begin
      @(negedge clk);
      pipe_start = 1'b1;
      @(negedge clk);
      pipe_start = 1'b0;
      cnt = 0;
      while (!mst1.done) begin
        @(posedge clk);
        cnt = cnt + 1;
        if (cnt > MAX_WAIT) begin
          $display("pipe timeout: i_state=%0d desc_idx=%0d sl_cnt=%0d slot_free=%0d",
                   mst1.i_state, mst1.desc_idx, mst1.sl_cnt, mst1.slot_free);
          $fatal(1, "MIX: pipe timeout");
        end
      end
    end
  endtask

  //==========================================================================
  // 场景
  //==========================================================================

  // M1：pipe 批量流水（8 描述符读写交错、跨两 slave、唯一 ID）
  task automatic m1();
    begin
      pipe_ndesc = 8;
      desc_put(0, 1'b0, 32'h0000_0200, 8'd3, 3'd2, `AXI_BURST_INCR, 4'd0, 32'hA000_0000);
      desc_put(1, 1'b1, 32'h0000_0200, 8'd3, 3'd2, `AXI_BURST_INCR, 4'd1, 32'h0);
      desc_put(2, 1'b0, 32'h1000_0400, 8'd1, 3'd2, `AXI_BURST_INCR, 4'd2, 32'hB000_0000);
      desc_put(3, 1'b1, 32'h1000_0400, 8'd1, 3'd2, `AXI_BURST_INCR, 4'd3, 32'h0);
      desc_put(4, 1'b0, 32'h0000_0600, 8'd7, 3'd2, `AXI_BURST_INCR, 4'd4, 32'hC000_0000);
      desc_put(5, 1'b1, 32'h0000_0600, 8'd7, 3'd2, `AXI_BURST_INCR, 4'd5, 32'h0);
      desc_put(6, 1'b0, 32'h1000_0800, 8'd3, 3'd2, `AXI_BURST_INCR, 4'd6, 32'hD000_0000);
      desc_put(7, 1'b1, 32'h1000_0800, 8'd3, 3'd2, `AXI_BURST_INCR, 4'd7, 32'h0);
      // 参考模型
      ref_update(32'h0000_0200, `AXI_BURST_INCR, 3'd2, 8'd3, 32'hA000_0000, 0, 0);
      ref_update(32'h1000_0400, `AXI_BURST_INCR, 3'd2, 8'd1, 32'hB000_0000, 0, 0);
      ref_update(32'h0000_0600, `AXI_BURST_INCR, 3'd2, 8'd7, 32'hC000_0000, 0, 0);
      ref_update(32'h1000_0800, `AXI_BURST_INCR, 3'd2, 8'd3, 32'hD000_0000, 0, 0);
      pipe_run();
      chk(!mst1.resp_err, "M1 resp_err");
      // 读校验和 = 4 个读描述符的 XOR
      begin
        logic [`AXI_DATA_W-1:0] echk, acc;
        logic [`AXI_DATA_W-1:0] e0, e1, e2, e3;
        exp_checksum(32'h0000_0200, `AXI_BURST_INCR, 3'd2, 8'd3, e0);
        exp_checksum(32'h1000_0400, `AXI_BURST_INCR, 3'd2, 8'd1, e1);
        exp_checksum(32'h0000_0600, `AXI_BURST_INCR, 3'd2, 8'd7, e2);
        exp_checksum(32'h1000_0800, `AXI_BURST_INCR, 3'd2, 8'd3, e3);
        acc = e0 ^ e1 ^ e2 ^ e3;
        echk = acc;
        chk(mst1.rd_checksum === echk, "M1 pipe checksum");
      end
      mem_compare();
      $display("[%0t] PASS M1", $time);
    end
  endtask

  // M2：cfg → lat slave：正常写读回 + 全 SLVERR 检查
  task automatic m2();
    logic [1:0] st;
    logic [`AXI_DATA_W-1:0] chk, echk;
    begin
      lat_b_delay = 8'd2;
      lat_r_delay = 8'd2;
      lat_err_mode = 2'd0;
      lat_err_period = 8'd4;
      mst0_wr(32'h1000_0A00, 8'd3, 3'd2, `AXI_BURST_INCR, 4'd0, 32'hE000_0000, st);
      chk(st == 0, "M2 wr status");
      ref_update(32'h1000_0A00, `AXI_BURST_INCR, 3'd2, 8'd3, 32'hE000_0000, 0, 0);
      mst0_rd(32'h1000_0A00, 8'd3, 3'd2, `AXI_BURST_INCR, 4'd0, st, chk);
      chk(st == 0, "M2 rd status");
      exp_checksum(32'h1000_0A00, `AXI_BURST_INCR, 3'd2, 8'd3, echk);
      chk(chk === echk, "M2 checksum");
      // 全 SLVERR 模式：写/读都返回 SLVERR（数据仍写入）
      lat_err_mode = 2'd1;
      mst0_wr(32'h1000_0C00, 8'd1, 3'd2, `AXI_BURST_INCR, 4'd0, 32'hF000_0000, st);
      chk(st == 2, "M2 wr SLVERR status");
      ref_update(32'h1000_0C00, `AXI_BURST_INCR, 3'd2, 8'd1, 32'hF000_0000, 0, 0);
      mst0_rd(32'h1000_0C00, 8'd1, 3'd2, `AXI_BURST_INCR, 4'd0, st, chk);
      chk(st == 2, "M2 rd SLVERR status");
      lat_err_mode = 2'd0;
      mem_compare();
      $display("[%0t] PASS M2", $time);
    end
  endtask

  // M3：并发：cfg→ram 写 同时 pipe→lat 读
  task automatic m3();
    logic [1:0] st0;
    begin
      // 先给 lat 放数据（M2 已写过 0x1000_0A00，直接读回）
      fork
        begin
          mst0_wr(32'h0000_0E00, 8'd3, 3'd2, `AXI_BURST_INCR, 4'd0, 32'h1100_0000, st0);
          chk(st0 == 0, "M3 cfg wr status");
        end
        begin
          pipe_ndesc = 1;
          desc_put(0, 1'b1, 32'h1000_0A00, 8'd3, 3'd2, `AXI_BURST_INCR, 4'd0, 32'h0);
          pipe_run();
          chk(!mst1.resp_err, "M3 pipe resp_err");
        end
      join
      ref_update(32'h0000_0E00, `AXI_BURST_INCR, 3'd2, 8'd3, 32'h1100_0000, 0, 0);
      begin
        logic [`AXI_DATA_W-1:0] echk;
        exp_checksum(32'h1000_0A00, `AXI_BURST_INCR, 3'd2, 8'd3, echk);
        chk(mst1.rd_checksum === echk, "M3 pipe checksum");
      end
      mem_compare();
      $display("[%0t] PASS M3", $time);
    end
  endtask

  // M4：两 master 抢 lat slave（大延迟制造 B 竞争）
  task automatic m4();
    logic [1:0] st0;
    begin
      lat_b_delay = 8'd5;
      lat_r_delay = 8'd3;
      fork
        begin
          mst0_wr(32'h1000_0F00, 8'd3, 3'd2, `AXI_BURST_INCR, 4'd0, 32'h1200_0000, st0);
          chk(st0 == 0, "M4 cfg wr status");
        end
        begin
          pipe_ndesc = 2;
          desc_put(0, 1'b0, 32'h1000_0F80, 8'd1, 3'd2, `AXI_BURST_INCR, 4'd1, 32'h1300_0000);
          desc_put(1, 1'b0, 32'h1000_0FA0, 8'd3, 3'd2, `AXI_BURST_INCR, 4'd2, 32'h1400_0000);
          pipe_run();
          chk(!mst1.resp_err, "M4 pipe resp_err");
        end
      join
      ref_update(32'h1000_0F00, `AXI_BURST_INCR, 3'd2, 8'd3, 32'h1200_0000, 0, 0);
      ref_update(32'h1000_0F80, `AXI_BURST_INCR, 3'd2, 8'd1, 32'h1300_0000, 0, 0);
      ref_update(32'h1000_0FA0, `AXI_BURST_INCR, 3'd2, 8'd3, 32'h1400_0000, 0, 0);
      lat_b_delay = 8'd0;
      lat_r_delay = 8'd0;
      mem_compare();
      $display("[%0t] PASS M4", $time);
    end
  endtask

  // M5：pipe 随机描述符多轮（先写后读，读回自检；4 写 + 4 读 = 8 条）
  task automatic m5(input integer tseed);
    begin
      for (int round = 0; round < 3; round++) begin
        pipe_ndesc = 8;
        // 4 条写（唯一地址、16B 步进不重叠）+ 4 条读（读回前 4 条）
        for (int i = 0; i < 4; i++) begin
          integer ur, base_i, len_i;
          logic [`AXI_ADDR_W-1:0] addr;
          ur = tseed + round*100 + i;
          base_i = 32'h1500_0000 + round*256 + i*16;
          len_i = (tseed + i) % 4;   // 0..3 拍指数（突发 <= 16B，不重叠）
          if ((i + round) % 2 == 0) addr = 32'h0000_0C00 + round*96 + i*16;
          else addr = 32'h1000_0C00 + round*96 + i*16;
          desc_put(i, 1'b0, addr, len_i, 3'd2, `AXI_BURST_INCR,
                   (8 + i), base_i);
          ref_update(addr, `AXI_BURST_INCR, 3'd2, len_i, base_i, 0, 0);
          desc_put(4+i, 1'b1, addr, len_i, 3'd2, `AXI_BURST_INCR,
                   (20 + i), 32'h0);
        end
        pipe_run();
        chk(!mst1.resp_err, "M5 pipe resp_err");
        begin
          logic [`AXI_DATA_W-1:0] acc, echk;
          acc = '0;
          for (int i = 0; i < 4; i++) begin
            logic [`AXI_ADDR_W-1:0] addr;
            integer base_i, len_i;
            base_i = 32'h1500_0000 + round*256 + i*16;
            len_i = (tseed + i) % 4;
            if ((i + round) % 2 == 0) addr = 32'h0000_0C00 + round*96 + i*16;
            else addr = 32'h1000_0C00 + round*96 + i*16;
            exp_checksum(addr, `AXI_BURST_INCR, 3'd2, len_i, echk);
            acc = acc ^ echk;
          end
          chk(mst1.rd_checksum === acc, "M5 pipe checksum");
        end
      end
      mem_compare();
      $display("[%0t] PASS M5", $time);
    end
  endtask

  //==========================================================================
  // M6：pipe 窄传输/非对齐（size 0/1，按 size 对齐的起址，读写交错）
  //==========================================================================
  task automatic m6();
    begin
      pipe_ndesc = 8;
      // 4 条窄写 + 4 条窄读（读回前 4 条）
      desc_put(0, 1'b0, 32'h0000_0B00, 8'd3, 3'd0, `AXI_BURST_INCR, 4'd0, 32'h1600_0000); // 字节宽 len3
      desc_put(1, 1'b0, 32'h1000_0B02, 8'd3, 3'd1, `AXI_BURST_INCR, 4'd1, 32'h1700_0000); // 半字宽 非对齐 len3
      desc_put(2, 1'b0, 32'h0000_0B20, 8'd1, 3'd0, `AXI_BURST_INCR, 4'd2, 32'h1800_0000); // 字节宽 len1
      desc_put(3, 1'b0, 32'h1000_0B24, 8'd1, 3'd1, `AXI_BURST_INCR, 4'd3, 32'h1900_0000); // 半字宽 len1 非对齐
      desc_put(4, 1'b1, 32'h0000_0B00, 8'd3, 3'd0, `AXI_BURST_INCR, 4'd4, 32'h0);
      desc_put(5, 1'b1, 32'h1000_0B02, 8'd3, 3'd1, `AXI_BURST_INCR, 4'd5, 32'h0);
      desc_put(6, 1'b1, 32'h0000_0B20, 8'd1, 3'd0, `AXI_BURST_INCR, 4'd6, 32'h0);
      desc_put(7, 1'b1, 32'h1000_0B24, 8'd1, 3'd1, `AXI_BURST_INCR, 4'd7, 32'h0);
      ref_update(32'h0000_0B00, `AXI_BURST_INCR, 3'd0, 8'd3, 32'h1600_0000, 0, 0);
      ref_update(32'h1000_0B02, `AXI_BURST_INCR, 3'd1, 8'd3, 32'h1700_0000, 0, 0);
      ref_update(32'h0000_0B20, `AXI_BURST_INCR, 3'd0, 8'd1, 32'h1800_0000, 0, 0);
      ref_update(32'h1000_0B24, `AXI_BURST_INCR, 3'd1, 8'd1, 32'h1900_0000, 0, 0);
      pipe_run();
      chk(!mst1.resp_err, "M6 resp_err");
      begin
        logic [`AXI_DATA_W-1:0] acc, echk;
        acc = '0;
        exp_checksum(32'h0000_0B00, `AXI_BURST_INCR, 3'd0, 8'd3, echk);
        acc = acc ^ echk;
        exp_checksum(32'h1000_0B02, `AXI_BURST_INCR, 3'd1, 8'd3, echk);
        acc = acc ^ echk;
        exp_checksum(32'h0000_0B20, `AXI_BURST_INCR, 3'd0, 8'd1, echk);
        acc = acc ^ echk;
        exp_checksum(32'h1000_0B24, `AXI_BURST_INCR, 3'd1, 8'd1, echk);
        acc = acc ^ echk;
        chk(mst1.rd_checksum === acc, "M6 narrow checksum");
      end
      mem_compare();
      $display("[%0t] PASS M6", $time);
    end
  endtask

  //==========================================================================
  // M7：cfg 窄传输（显式 WSTRB 单拍部分写 + 读回）
  //==========================================================================
  task automatic m7();
    logic [1:0] st;
    logic [`AXI_DATA_W-1:0] chk, echk;
    begin
      // 半字写：地址 0x1000_0B40 起 size=1 单拍，WSTRB=1100（高 2 字节）
      cfg0_wstrb = 4'b1100;
      mst0_wr(32'h1000_0B40, 8'd0, 3'd1, `AXI_BURST_INCR, 4'd0, 32'h1A00_0000, st);
      chk(st == 0, "M7 wr status");
      ref_update(32'h1000_0B40, `AXI_BURST_INCR, 3'd1, 8'd0, 32'h1A00_0000, 1, 4'b1100);
      cfg0_wstrb = 4'b1111;
      // 读回（读全字：高 2 字节 = 写的数据，低 2 字节 = 0）
      mst0_rd(32'h1000_0B40, 8'd0, 3'd2, `AXI_BURST_INCR, 4'd0, st, chk);
      chk(st == 0, "M7 rd status");
      begin
        logic [`AXI_DATA_W-1:0] w;
        dbg_word(1, 32'h1000_0B40, w);
        chk(w === 32'h1A00_0000, "M7 narrow write value");
        chk(chk === w, "M7 checksum");
      end
      mem_compare();
      $display("[%0t] PASS M7", $time);
    end
  endtask

  //--------------------------------------------------------------------------
  // 主流程
  //--------------------------------------------------------------------------
  initial begin
    integer tseed;
    $display("=== AXI4 Interconnect Mixed-topology TB ===");
    cfg0_wr_start = 1'b0;
    cfg0_rd_start = 1'b0;
    pipe_start = 1'b0;
    desc_wr    = 1'b0;
    lat_b_delay = 8'd0;
    lat_r_delay = 8'd0;
    lat_err_mode = 2'd0;
    lat_err_period = 8'd4;
    rstn = 1'b0;
    if (!$value$plusargs("+seed=%d", tseed)) tseed = 1;

    $dumpfile("axi_mix.vcd");
    $dumpvars(0, tb_axi_mix);

    repeat (10) @(posedge clk);
    rstn = 1'b1;
    repeat (2) @(posedge clk);

    m1();
    m2();
    m3();
    m4();
    m5(tseed);
    m6();
    m7();

    $display("========================================");
    $display("ALL MIXED SCENARIOS PASS");
    $finish;
  end

  // 全局超时看门狗
  initial begin
    repeat (2000000) @(posedge clk);
    $display("FAIL: global watchdog timeout");
    $fatal(1);
  end

endmodule
