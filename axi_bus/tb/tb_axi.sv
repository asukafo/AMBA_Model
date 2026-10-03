`include "axi_defs.svh"
//------------------------------------------------------------------------------
// tb_axi — AXI4 交叉开关互联验证环境（iverilog）
//
// 拓扑：2 master × 2 slave + 内部 DECERR 响应器
//   地址映射（4KB 窗口内活动）：
//     slave0: base 0x0000_0000 mask 0xF000_0000
//     slave1: base 0x1000_0000 mask 0xF000_0000
//     DECERR: 0x8000_0000 区域（未命中）
//
// 检查手段（三层）：
//   1. BFM 自检：RID 匹配、按 seed 生成的数据比对、响应码
//   2. scoreboard：参考内存与每个 slave 内存逐字节比对
//   3. 监视器：固定优先级不变量 / RR 公平性 / 超时
//
// 场景（S1-S15）：
//   S1  双主并发 INCR 读写不同 slave
//   S2  全主抢同一 slave AW（仅 FIXED 构建：严格优先级不变量）
//   S3  全主抢同一 slave AW（仅 RR 构建：公平计数 ±4）
//   S4  同 master 2 读 2 slave，slave1 先回（乱序，RID 匹配）
//   S5  WRAP 突发非对齐读写
//   S6  窄传输随机 WSTRB（含全 0 拍）
//   S7  DECERR 读写（B=DECERR、单拍 R=DECERR）
//   S8  W 通道交错：两 master 先后获 AW 授权，m1 提前置 WVALID
//   S9  DECERR 3 条 outstanding AR（FIFO 顺序）
//   S10 双主随机混合流量 + 随机背压 + DECERR 点缀
//   S11 随机到达间隔抢同一 slave（策略不变量/公平性）
//   S12 DECERR 多拍写 + 另一 master 真实写并行（slave W 背压）
//   S13 两 slave 同拍 BVALID 竞争同一 master 的 B 仲裁
//   S14 RREADY 停摆下 R 授权锁持有（无死锁）
//   S15 单 master 多 ID 并发读写
//------------------------------------------------------------------------------
module tb_axi;
  timeunit 1ns / 1ps;

  // ---- 时钟 / 复位 ----
  logic clk;
  logic rstn;
  initial clk = 1'b0;
  always #5 clk = ~clk;   // 100MHz

  `include "tb_axi_wires.svh"

  // slave 配置（背压概率 / 读延迟）
  logic [6:0] cfg_aw_rdy_pct [`AXI_M_SLAVE];
  logic [6:0] cfg_w_rdy_pct  [`AXI_M_SLAVE];
  logic [7:0] cfg_r_delay    [`AXI_M_SLAVE];


  // ---- master BFM ----
  for (genvar i = 0; i < `AXI_N_MASTER; i++) begin : g_mst
    axi_master_bfm #(
      .MST_ID (i)
    ) bfm (
      .clk (clk), .rstn (rstn),
      .awvalid (s_awvalid[i]),
      .awid (s_awid[i*`AXI_ID_W +: `AXI_ID_W]),
      .awaddr (s_awaddr[i*`AXI_ADDR_W +: `AXI_ADDR_W]),
      .awlen (s_awlen[i*8 +: 8]),
      .awsize (s_awsize[i*3 +: 3]),
      .awburst (s_awburst[i*2 +: 2]),
      .awready (s_awready[i]),
      .wvalid (s_wvalid[i]),
      .wdata (s_wdata[i*`AXI_DATA_W +: `AXI_DATA_W]),
      .wstrb (s_wstrb[i*`AXI_DATA_W/8 +: `AXI_DATA_W/8]),
      .wlast (s_wlast[i]),
      .wready (s_wready[i]),
      .bvalid (s_bvalid[i]),
      .bid (s_bid[i*`AXI_ID_W +: `AXI_ID_W]),
      .bresp (s_bresp[i*2 +: 2]),
      .bready (s_bready[i]),
      .arvalid (s_arvalid[i]),
      .arid (s_arid[i*`AXI_ID_W +: `AXI_ID_W]),
      .araddr (s_araddr[i*`AXI_ADDR_W +: `AXI_ADDR_W]),
      .arlen (s_arlen[i*8 +: 8]),
      .arsize (s_arsize[i*3 +: 3]),
      .arburst (s_arburst[i*2 +: 2]),
      .arready (s_arready[i]),
      .rvalid (s_rvalid[i]),
      .rid (s_rid[i*`AXI_ID_W +: `AXI_ID_W]),
      .rdata (s_rdata[i*`AXI_DATA_W +: `AXI_DATA_W]),
      .rresp (s_rresp[i*2 +: 2]),
      .rlast (s_rlast[i]),
      .rready (s_rready[i])
    );
  end

  // ---- slave 模型 ----
  for (genvar i = 0; i < `AXI_M_SLAVE; i++) begin : g_slv
    axi_slave_model #(
      .SLV_ID (i),
      .SEED   (100 + i)
    ) slv (
      .clk (clk), .rstn (rstn),
      .cfg_aw_rdy_pct (cfg_aw_rdy_pct[i]),
      .cfg_w_rdy_pct  (cfg_w_rdy_pct[i]),
      .cfg_r_delay    (cfg_r_delay[i]),
      .awvalid (m_awvalid[i]),
      .awid (m_awid[i*`AXI_SLV_ID_W +: `AXI_SLV_ID_W]),
      .awaddr (m_awaddr[i*`AXI_ADDR_W +: `AXI_ADDR_W]),
      .awlen (m_awlen[i*8 +: 8]),
      .awsize (m_awsize[i*3 +: 3]),
      .awburst (m_awburst[i*2 +: 2]),
      .awready (m_awready[i]),
      .wvalid (m_wvalid[i]),
      .wdata (m_wdata[i*`AXI_DATA_W +: `AXI_DATA_W]),
      .wstrb (m_wstrb[i*`AXI_DATA_W/8 +: `AXI_DATA_W/8]),
      .wlast (m_wlast[i]),
      .wready (m_wready[i]),
      .bvalid (m_bvalid[i]),
      .bid (m_bid[i*`AXI_SLV_ID_W +: `AXI_SLV_ID_W]),
      .bresp (m_bresp[i*2 +: 2]),
      .bready (m_bready[i]),
      .arvalid (m_arvalid[i]),
      .arid (m_arid[i*`AXI_SLV_ID_W +: `AXI_SLV_ID_W]),
      .araddr (m_araddr[i*`AXI_ADDR_W +: `AXI_ADDR_W]),
      .arlen (m_arlen[i*8 +: 8]),
      .arsize (m_arsize[i*3 +: 3]),
      .arburst (m_arburst[i*2 +: 2]),
      .arready (m_arready[i]),
      .rvalid (m_rvalid[i]),
      .rid (m_rid[i*`AXI_SLV_ID_W +: `AXI_SLV_ID_W]),
      .rdata (m_rdata[i*`AXI_DATA_W +: `AXI_DATA_W]),
      .rresp (m_rresp[i*2 +: 2]),
      .rlast (m_rlast[i]),
      .rready (m_rready[i])
    );
  end

  //--------------------------------------------------------------------------
  // scoreboard：参考内存（每 slave 一个 4KB 窗口，拍平成 1D）
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

  // 参考内存更新（与 BFM 同一套 seed 生成，掩码与写入一致）
  task automatic sb_write(
    input logic [`AXI_ADDR_W-1:0] addr,
    input logic [1:0] burst,
    input logic [2:0] size,
    input logic [7:0] len,
    input integer seed,
    input integer strb_seed,
    input integer strb_mode
  );
    integer s;
    begin
      s = slave_of(addr);
      if (s >= 0) begin   // DECERR 区无内存
        for (int b = 0; b <= len; b++) begin
          logic [`AXI_ADDR_W-1:0] a;
          logic [`AXI_DATA_W-1:0] d;
          logic [`AXI_DATA_W/8-1:0] st;
          a  = axi_beat_addr(addr, burst, size, len, b);
          d  = axi_test_data(seed, b);
          st = axi_test_strb(strb_seed, b, strb_mode);
          for (int i = 0; i < `AXI_DATA_W/8; i++)
            if (st[i]) ref_mem[s*4096 + a[11:0] + i] = d[8*i +: 8];
        end
      end
    end
  endtask

  // 参考内存与 slave 内存逐字节比对。
  // 注意：iverilog 层级引用的 generate 索引必须为常量，slave 循环显式展开
  task automatic sb_compare();
    begin
      for (int i = 0; i < 4096; i++) begin
        if (ref_mem[0*4096 + i] !== g_slv[0].slv.get_byte(i)) begin
          $display("FAIL: sb_compare slave 0 addr 0x%0h: ref=%0h dut=%0h",
                   i, ref_mem[0*4096+i], g_slv[0].slv.get_byte(i));
          $fatal(1);
        end
        if (ref_mem[1*4096 + i] !== g_slv[1].slv.get_byte(i)) begin
          $display("FAIL: sb_compare slave 1 addr 0x%0h: ref=%0h dut=%0h",
                   i, ref_mem[1*4096+i], g_slv[1].slv.get_byte(i));
          $fatal(1);
        end
      end
    end
  endtask

  task automatic chk(input integer cond, input [1023:0] msg);
    if (!cond) begin
      $display("FAIL: %s (t=%0t)", msg, $time);
      $fatal(1);
    end
  endtask

  //--------------------------------------------------------------------------
  // 监视器：AW 握手计数 + 仲裁策略不变量
  //   arb_cnt[m]：双方请求同时激活（slave0）时的竞争握手归属计数
  //     RR 下应严格交替（|diff|<=2）；FIXED 下 m1 永远不该赢
  //   arb_viol ：FIXED 下授权发给 m1 的瞬间 m0 请求激活 → 违例
  //--------------------------------------------------------------------------
  integer aw_hs_cnt [`AXI_N_MASTER];
  integer arb_cnt [`AXI_N_MASTER];
  integer arb_viol;
  logic   grant1_prev;

  initial begin
    arb_viol = 0;
    grant1_prev = 1'b0;
    for (int m = 0; m < `AXI_N_MASTER; m++) begin
      aw_hs_cnt[m] = 0;
      arb_cnt[m] = 0;
    end
  end

  always @(posedge clk) begin
    if (rstn) begin
      for (int m = 0; m < `AXI_N_MASTER; m++) begin
        if (s_awvalid[m] && s_awready[m] &&
            (slave_of(s_awaddr[m*`AXI_ADDR_W +: `AXI_ADDR_W]) == 0))
          aw_hs_cnt[m] = aw_hs_cnt[m] + 1;
      end
      // 竞争握手归属（slave0：dbg 位 0=m0 请求，位 1=m1 请求）
      if (dbg_aw_req[0] && dbg_aw_req[1]) begin
        if (s_awvalid[0] && s_awready[0]) arb_cnt[0] = arb_cnt[0] + 1;
        if (s_awvalid[1] && s_awready[1]) arb_cnt[1] = arb_cnt[1] + 1;
      end
      // 固定优先级不变量：slave0 的 AW 授权发给 m1 的瞬间，
      // m0 的仲裁请求（已屏蔽 w_pending）不得为 1
      // dbg 拍平位序：位 [s*N_MASTER + m] → slave0.m1 = 位 1，slave0.m0 = 位 0
      if (`AXI_ARB_POLICY == 0) begin
        if (dbg_aw_grant[1] && dbg_aw_req[0] && !grant1_prev)
          arb_viol = 1;
      end
      grant1_prev = dbg_aw_grant[1];
    end
  end

  // 全局超时看门狗
  initial begin
    repeat (2000000) @(posedge clk);
    $display("FAIL: global watchdog timeout");
    $fatal(1);
  end

  //==========================================================================
  // S1：双主并发 INCR 读写不同 slave
  //==========================================================================
  task automatic s1();
    integer r0, r1;
    begin
      sb_write(32'h0000_0100, `AXI_BURST_INCR, 3'd2, 8'd7, 100, 0, 0);
      sb_write(32'h1000_0200, `AXI_BURST_INCR, 3'd2, 8'd7, 200, 0, 0);
      fork
        begin
          g_mst[0].bfm.write(32'h0000_0100, `AXI_BURST_INCR, 3'd2, 8'd7, 4'd0, 100, 0, 0, r0);
          chk(r0 == 0, "S1 m0 write resp");
          g_mst[0].bfm.read(32'h0000_0100, `AXI_BURST_INCR, 3'd2, 8'd7, 4'd0, 100, 0, 0, 0, 0, r0);
          chk(r0 == 0, "S1 m0 read resp");
        end
        begin
          g_mst[1].bfm.write(32'h1000_0200, `AXI_BURST_INCR, 3'd2, 8'd7, 4'd1, 200, 0, 0, r1);
          chk(r1 == 0, "S1 m1 write resp");
          g_mst[1].bfm.read(32'h1000_0200, `AXI_BURST_INCR, 3'd2, 8'd7, 4'd1, 200, 0, 0, 0, 0, r1);
          chk(r1 == 0, "S1 m1 read resp");
        end
      join
      sb_compare();
      $display("[%0t] PASS S1", $time);
    end
  endtask

  //==========================================================================
  // S2/S3：全 master 抢 slave0 AW（按构建选策略检查）
  //==========================================================================
  task automatic s2s3();
    integer r0, r1;
    integer arb_viol_b;
    integer a0_b, a1_b;
    begin
      arb_viol_b = arb_viol;
      a0_b = arb_cnt[0];
      a1_b = arb_cnt[1];
      fork
        begin
          for (int i = 0; i < 20; i++) begin
            g_mst[0].bfm.write(32'h0000_0A00 + 32*i, `AXI_BURST_INCR, 3'd2, 8'd3, 4'd0, 4000+i, 0, 0, r0);
            chk(r0 == 0, "S2S3 m0 write resp");
            sb_write(32'h0000_0A00 + 32*i, `AXI_BURST_INCR, 3'd2, 8'd3, 4000+i, 0, 0);
          end
        end
        begin
          for (int i = 0; i < 20; i++) begin
            g_mst[1].bfm.write(32'h0000_0B00 + 32*i, `AXI_BURST_INCR, 3'd2, 8'd3, 4'd1, 4100+i, 0, 0, r1);
            chk(r1 == 0, "S2S3 m1 write resp");
            sb_write(32'h0000_0B00 + 32*i, `AXI_BURST_INCR, 3'd2, 8'd3, 4100+i, 0, 0);
          end
        end
      join
      if (`AXI_ARB_POLICY == 0) begin
        chk(arb_viol == arb_viol_b, "S2 fixed-priority invariant violated");
        chk((arb_cnt[1] - a1_b) == 0, "S2 fixed: m1 won a contended grant");
        $display("[%0t] PASS S2 (fixed)", $time);
      end else begin
        // RR：竞争握手归属必须严格交替
        begin
          integer d0, d1, d;
          d0 = arb_cnt[0] - a0_b;
          d1 = arb_cnt[1] - a1_b;
          d = (d0 > d1) ? (d0 - d1) : (d1 - d0);
          chk(d <= 2, "S3 RR contended grants not alternating");
        end
        $display("[%0t] PASS S3 (rr, contended m0=%0d m1=%0d)",
                 $time, arb_cnt[0]-a0_b, arb_cnt[1]-a1_b);
      end
      sb_compare();
    end
  endtask

  //==========================================================================
  // S4：同 master 2 读 2 slave，slave1 先回（乱序响应）
  //==========================================================================
  task automatic s4();
    integer r;
    begin
      // 先写好数据
      g_mst[0].bfm.write(32'h0000_0300, `AXI_BURST_INCR, 3'd2, 8'd3, 4'd0, 300, 0, 0, r);
      chk(r == 0, "S4 wr0");
      g_mst[0].bfm.write(32'h1000_0400, `AXI_BURST_INCR, 3'd2, 8'd3, 4'd1, 310, 0, 0, r);
      chk(r == 0, "S4 wr1");
      sb_write(32'h0000_0300, `AXI_BURST_INCR, 3'd2, 8'd3, 300, 0, 0);
      sb_write(32'h1000_0400, `AXI_BURST_INCR, 3'd2, 8'd3, 310, 0, 0);
      // slave0 慢（10 拍），slave1 快（1 拍）→ slave1 的 R 先到
      cfg_r_delay[0] = 8'd10;
      cfg_r_delay[1] = 8'd1;
      g_mst[0].bfm.ar_only(32'h0000_0300, `AXI_BURST_INCR, 3'd2, 8'd3, 4'd0, 300, 0, 0, 0, 0);
      g_mst[0].bfm.ar_only(32'h1000_0400, `AXI_BURST_INCR, 3'd2, 8'd3, 4'd1, 310, 0, 0, 0, 0);
      g_mst[0].bfm.collect_all(0, 0);
      // 完成顺序：槽 1（slave1/id1）先于槽 0（slave0/id0）
      chk(g_mst[0].bfm.ar_done_order[0] == 1 &&
          g_mst[0].bfm.ar_done_order[1] == 0, "S4 out-of-order completion");
      cfg_r_delay[0] = 8'd2;
      cfg_r_delay[1] = 8'd2;
      sb_compare();
      $display("[%0t] PASS S4", $time);
    end
  endtask

  //==========================================================================
  // S5：WRAP 突发非对齐读写（起点 0x224，4 拍 × 4B，回卷边界 0x220）
  //==========================================================================
  task automatic s5();
    integer r;
    begin
      sb_write(32'h0000_0224, `AXI_BURST_WRAP, 3'd2, 8'd3, 500, 0, 0);
      g_mst[0].bfm.write(32'h0000_0224, `AXI_BURST_WRAP, 3'd2, 8'd3, 4'd0, 500, 0, 0, r);
      chk(r == 0, "S5 write resp");
      g_mst[0].bfm.read(32'h0000_0224, `AXI_BURST_WRAP, 3'd2, 8'd3, 4'd0, 500, 0, 0, 0, 0, r);
      chk(r == 0, "S5 read resp");
      sb_compare();
      $display("[%0t] PASS S5", $time);
    end
  endtask

  //==========================================================================
  // S6：窄传输随机 WSTRB（含全 0 拍）
  //==========================================================================
  task automatic s6();
    integer r;
    begin
      sb_write(32'h0000_0500, `AXI_BURST_INCR, 3'd2, 8'd3, 600, 601, 1);
      g_mst[0].bfm.write(32'h0000_0500, `AXI_BURST_INCR, 3'd2, 8'd3, 4'd0, 600, 601, 1, r);
      chk(r == 0, "S6 write resp");
      g_mst[0].bfm.read(32'h0000_0500, `AXI_BURST_INCR, 3'd2, 8'd3, 4'd0, 600, 601, 1, 0, 0, r);
      chk(r == 0, "S6 read resp");
      sb_compare();
      $display("[%0t] PASS S6", $time);
    end
  endtask

  //==========================================================================
  // S7：DECERR 读写
  //==========================================================================
  task automatic s7();
    integer r;
    begin
      g_mst[0].bfm.write(32'h8000_0000, `AXI_BURST_INCR, 3'd2, 8'd1, 4'd0, 700, 0, 0, r);
      chk(r == 3, "S7 write DECERR resp");
      g_mst[0].bfm.read(32'h8000_0010, `AXI_BURST_INCR, 3'd2, 8'd0, 4'd0, 700, 0, 0, 1, 3, r);
      chk(r == 3, "S7 read DECERR resp");
      $display("[%0t] PASS S7", $time);
    end
  endtask

  //==========================================================================
  // S8：W 通道交错 —— 两 master 先后获 AW 授权，m1 提前置 WVALID 而 m0 停摆
  //==========================================================================
  task automatic s8();
    integer r0, r1;
    begin
      g_mst[0].bfm.aw_only(32'h0000_0600, `AXI_BURST_INCR, 3'd2, 8'd3, 4'd0);
      g_mst[1].bfm.aw_only(32'h0000_0700, `AXI_BURST_INCR, 3'd2, 8'd1, 4'd1);
      sb_write(32'h0000_0600, `AXI_BURST_INCR, 3'd2, 8'd3, 800, 0, 0);
      sb_write(32'h0000_0700, `AXI_BURST_INCR, 3'd2, 8'd1, 810, 0, 0);
      fork
        begin
          // m1 提前驱动 W 数据（wready 被互联挡住，直到 m0 流完成）
          g_mst[1].bfm.w_only(810, 0, 0);
        end
        begin
          repeat (10) @(posedge clk);   // m0 停摆 10 拍
          g_mst[0].bfm.w_only(800, 0, 0);
        end
      join
      g_mst[0].bfm.wait_b(4'd0, r0);
      chk(r0 == 0, "S8 m0 B resp");
      g_mst[1].bfm.wait_b(4'd1, r1);
      chk(r1 == 0, "S8 m1 B resp");
      // W 流顺序：m0 的 WLAST 必须早于 m1 的首拍握手
      chk(g_mst[0].bfm.w_done_time < g_mst[1].bfm.w_first_hs_time,
          "S8 W stream order");
      sb_compare();
      $display("[%0t] PASS S8", $time);
    end
  endtask

  //==========================================================================
  // S9：DECERR 3 条 outstanding AR（FIFO 顺序响应）
  //==========================================================================
  task automatic s9();
    begin
      g_mst[0].bfm.ar_only(32'h8000_0100, `AXI_BURST_INCR, 3'd2, 8'd0, 4'd5, 900, 0, 0, 1, 3);
      g_mst[0].bfm.ar_only(32'h8000_0200, `AXI_BURST_INCR, 3'd2, 8'd0, 4'd6, 900, 0, 0, 1, 3);
      g_mst[0].bfm.ar_only(32'h8000_0300, `AXI_BURST_INCR, 3'd2, 8'd0, 4'd7, 900, 0, 0, 1, 3);
      g_mst[0].bfm.collect_all(0, 0);
      chk(g_mst[0].bfm.ar_done_order[0] == 0 &&
          g_mst[0].bfm.ar_done_order[1] == 1 &&
          g_mst[0].bfm.ar_done_order[2] == 2, "S9 DECERR AR FIFO order");
      $display("[%0t] PASS S9", $time);
    end
  endtask

  //==========================================================================
  // S10：双主随机混合流量 + DECERR 点缀
  //==========================================================================
  task automatic s10(input integer tseed);
    integer r;
    begin
      fork
        begin : m0_loop
          for (int i = 0; i < 32; i++) begin
            integer seed, op, ur;
            seed = 2000 + i;
            ur   = tseed + i;
            op   = $urandom(ur) % 10;
            // 写（读回自检的期望数据源）
            begin
              integer mode;
              mode = (i % 3 == 0) ? 1 : 0;
              g_mst[0].bfm.write(32'h0000_0800 + 16*i, `AXI_BURST_INCR, 3'd2, i%4, 4'd0, seed, seed+77, mode, r);
              chk(r == 0, "S10 m0 write resp");
              sb_write(32'h0000_0800 + 16*i, `AXI_BURST_INCR, 3'd2, i%4, seed, seed+77, mode);
              if (op < 3) begin
                g_mst[0].bfm.read(32'h0000_0800 + 16*i, `AXI_BURST_INCR, 3'd2, i%4, 4'd0, seed, seed+77, mode, 0, 0, r);
                chk(r == 0, "S10 m0 read resp");
              end
            end
            if ((i+1) % 5 == 0) begin
              g_mst[0].bfm.write(32'h8000_0800, `AXI_BURST_INCR, 3'd2, 8'd0, 4'd0, seed, 0, 0, r);
              chk(r == 3, "S10 m0 DECERR resp");
            end
          end
        end
        begin : m1_loop
          for (int i = 0; i < 32; i++) begin
            integer seed, op, ur;
            seed = 3000 + i;
            ur   = tseed + 1000 + i;
            op   = $urandom(ur) % 10;
            begin
              integer mode;
              mode = (i % 3 == 0) ? 1 : 0;
              g_mst[1].bfm.write(32'h1000_0900 + 16*i, `AXI_BURST_INCR, 3'd2, i%4, 4'd1, seed, seed+77, mode, r);
              chk(r == 0, "S10 m1 write resp");
              sb_write(32'h1000_0900 + 16*i, `AXI_BURST_INCR, 3'd2, i%4, seed, seed+77, mode);
              if (op < 3) begin
                g_mst[1].bfm.read(32'h1000_0900 + 16*i, `AXI_BURST_INCR, 3'd2, i%4, 4'd1, seed, seed+77, mode, 0, 0, r);
                chk(r == 0, "S10 m1 read resp");
              end
            end
            if ((i+1) % 5 == 0) begin
              g_mst[1].bfm.write(32'h8000_0810, `AXI_BURST_INCR, 3'd2, 8'd0, 4'd1, seed, 0, 0, r);
              chk(r == 3, "S10 m1 DECERR resp");
            end
          end
        end
      join
      sb_compare();
      $display("[%0t] PASS S10", $time);
    end
  endtask

  //==========================================================================
  // S11：随机到达间隔抢同一 slave（策略不变量/公平性）
  //==========================================================================
  task automatic s11(input integer tseed);
    integer r;
    integer arb_viol_b;
    integer a0_b, a1_b;
    begin
      arb_viol_b = arb_viol;
      a0_b = arb_cnt[0];
      a1_b = arb_cnt[1];
      for (int i = 0; i < 30; i++) begin
        integer m, d, ur;
        ur = tseed + 5000 + i;
        m  = $urandom(ur) % 2;
        ur = tseed + 6000 + i;
        d  = $urandom(ur) % 5;
        repeat (d) @(posedge clk);
        // 层级引用索引必须为常量（iverilog），显式展开
        if (m == 0)
          g_mst[0].bfm.write(32'h0000_0C00 + 4*i, `AXI_BURST_INCR, 3'd2, 8'd0, 0, 5000+i, 0, 0, r);
        else
          g_mst[1].bfm.write(32'h0000_0C00 + 4*i, `AXI_BURST_INCR, 3'd2, 8'd0, 1, 5000+i, 0, 0, r);
        chk(r == 0, "S11 write resp");
        sb_write(32'h0000_0C00 + 4*i, `AXI_BURST_INCR, 3'd2, 8'd0, 5000+i, 0, 0);
      end
      if (`AXI_ARB_POLICY == 0) begin
        chk(arb_viol == arb_viol_b, "S11 fixed-priority invariant violated");
        chk((arb_cnt[1] - a1_b) == 0, "S11 fixed: m1 won a contended grant");
      end else begin
        begin
          integer d0, d1, d;
          d0 = arb_cnt[0] - a0_b;
          d1 = arb_cnt[1] - a1_b;
          d = (d0 > d1) ? (d0 - d1) : (d1 - d0);
          chk(d <= 2, "S11 RR contended grants not alternating");
        end
      end
      sb_compare();
      $display("[%0t] PASS S11", $time);
    end
  endtask

  //==========================================================================
  // S12：DECERR 多拍写 + 另一 master 真实写并行（slave W 背压）
  //==========================================================================
  task automatic s12();
    integer r0, r1;
    begin
      cfg_w_rdy_pct[0] = 7'd50;   // slave0 写背压 50%
      fork
        begin
          g_mst[0].bfm.write(32'h8000_0200, `AXI_BURST_INCR, 3'd2, 8'd3, 4'd0, 900, 0, 0, r0);
        end
        begin
          g_mst[1].bfm.write(32'h0000_0D00, `AXI_BURST_INCR, 3'd2, 8'd3, 4'd1, 910, 0, 0, r1);
        end
      join
      chk(r0 == 3, "S12 DECERR resp");
      chk(r1 == 0, "S12 write resp");
      sb_write(32'h0000_0D00, `AXI_BURST_INCR, 3'd2, 8'd3, 910, 0, 0);
      cfg_w_rdy_pct[0] = 7'd100;
      sb_compare();
      $display("[%0t] PASS S12", $time);
    end
  endtask

  //==========================================================================
  // S13：两 slave 同拍 BVALID 竞争同一 master 的 B 仲裁
  //==========================================================================
  task automatic s13();
    integer r0, r1;
    begin
      // 写 1 → slave0；B1 置起后 bready 保持 0（挂起）
      g_mst[0].bfm.aw_only(32'h0000_0E00, `AXI_BURST_INCR, 3'd2, 8'd1, 4'd0);
      g_mst[0].bfm.w_only(1000, 0, 0);
      // 写 2 → slave1（WLAST 完成即可发新 AW；B1 仍挂起）
      g_mst[0].bfm.aw_only(32'h1000_0F00, `AXI_BURST_INCR, 3'd2, 8'd1, 4'd1);
      g_mst[0].bfm.w_only(1010, 0, 0);
      sb_write(32'h0000_0E00, `AXI_BURST_INCR, 3'd2, 8'd1, 1000, 0, 0);
      sb_write(32'h1000_0F00, `AXI_BURST_INCR, 3'd2, 8'd1, 1010, 0, 0);
      // 等 B2 置起：两个 BVALID 同时挂起
      repeat (20) @(posedge clk);
      chk(m_bvalid[0] && m_bvalid[1], "S13 both BVALID pending");
      g_mst[0].bfm.wait_b(4'd0, r0);
      g_mst[0].bfm.wait_b(4'd1, r1);
      chk(r0 == 0 && r1 == 0, "S13 B resp");
      sb_compare();
      $display("[%0t] PASS S13", $time);
    end
  endtask

  //==========================================================================
  // S14：RREADY 停摆下 R 授权锁持有（无死锁）
  //==========================================================================
  task automatic s14();
    integer r;
    begin
      g_mst[0].bfm.write(32'h0000_0080, `AXI_BURST_INCR, 3'd2, 8'd7, 4'd0, 1100, 0, 0, r);
      chk(r == 0, "S14 wr0");
      g_mst[0].bfm.write(32'h1000_0090, `AXI_BURST_INCR, 3'd2, 8'd3, 4'd0, 1110, 0, 0, r);
      chk(r == 0, "S14 wr1");
      sb_write(32'h0000_0080, `AXI_BURST_INCR, 3'd2, 8'd7, 1100, 0, 0);
      sb_write(32'h1000_0090, `AXI_BURST_INCR, 3'd2, 8'd3, 1110, 0, 0);
      // slave0 快（先发长突发），slave1 慢（中途挂起等待）
      cfg_r_delay[0] = 8'd1;
      cfg_r_delay[1] = 8'd5;
      g_mst[0].bfm.ar_only(32'h0000_0080, `AXI_BURST_INCR, 3'd2, 8'd7, 4'd0, 1100, 0, 0, 0, 0);
      g_mst[0].bfm.ar_only(32'h1000_0090, `AXI_BURST_INCR, 3'd2, 8'd3, 4'd1, 1110, 0, 0, 0, 0);
      // 收 1 拍后停摆 100 拍，再继续收完两笔
      g_mst[0].bfm.collect_all(1, 100);
      cfg_r_delay[0] = 8'd2;
      cfg_r_delay[1] = 8'd2;
      sb_compare();
      $display("[%0t] PASS S14", $time);
    end
  endtask

  //==========================================================================
  // S15：单 master 多 ID 并发读写
  //==========================================================================
  task automatic s15();
    integer r;
    begin
      g_mst[0].bfm.write(32'h0000_0040, `AXI_BURST_INCR, 3'd2, 8'd1, 4'd5, 1200, 0, 0, r);
      chk(r == 0, "S15 wr0");
      g_mst[0].bfm.write(32'h1000_0050, `AXI_BURST_INCR, 3'd2, 8'd1, 4'd9, 1210, 0, 0, r);
      chk(r == 0, "S15 wr1");
      sb_write(32'h0000_0040, `AXI_BURST_INCR, 3'd2, 8'd1, 1200, 0, 0);
      sb_write(32'h1000_0050, `AXI_BURST_INCR, 3'd2, 8'd1, 1210, 0, 0);
      // 读用不同 ID 并发（id3 / id7）
      g_mst[0].bfm.ar_only(32'h0000_0040, `AXI_BURST_INCR, 3'd2, 8'd1, 4'd3, 1200, 0, 0, 0, 0);
      g_mst[0].bfm.ar_only(32'h1000_0050, `AXI_BURST_INCR, 3'd2, 8'd1, 4'd7, 1210, 0, 0, 0, 0);
      g_mst[0].bfm.collect_all(0, 0);
      chk(g_mst[0].bfm.ar_done_cnt == 2, "S15 both reads done");
      sb_compare();
      $display("[%0t] PASS S15", $time);
    end
  endtask

  //==========================================================================
  // 主流程
  //==========================================================================
  initial begin
    integer tseed;
    $display("=== AXI4 Interconnect TB ===");
    $display("POLICY=%0d RESP_POLICY=%0d", `AXI_ARB_POLICY, `AXI_RESP_POLICY);
    // slave 默认配置
    for (int i = 0; i < `AXI_M_SLAVE; i++) begin
      cfg_aw_rdy_pct[i] = 7'd100;
      cfg_w_rdy_pct[i]  = 7'd100;
      cfg_r_delay[i]    = 8'd2;
    end
    rstn = 1'b0;
    if (!$value$plusargs("+seed=%d", tseed)) tseed = 1;
    $display("seed=%0d", tseed);

    $dumpfile("axi.vcd");
    $dumpvars(0, tb_axi);

    repeat (10) @(posedge clk);
    rstn = 1'b1;
    repeat (2) @(posedge clk);

    s1();
    s2s3();
    s4();
    s5();
    s6();
    s7();
    s8();
    s9();
    s10(tseed);
    s11(tseed);
    s12();
    s13();
    s14();
    s15();

    $display("========================================");
    $display("ALL SCENARIOS PASS");
    $finish;
  end

endmodule
