`include "axi_defs.svh"
//------------------------------------------------------------------------------
// tb_axi_reg — 寄存器外设 RTL 级联调：
//   master0 = axi_master_cfg（支持 WSTRB 字节使能配置）
//   slave0  = axi_slave_reg（阻塞式寄存器外设）
//   slave1  = axi_slave_ram（占位，本 TB 不使用）
//
// 检查手段：cfg master 状态码 / 读校验和 / 寄存器调试读口比对。
//
// 场景：
//   G1 寄存器写（含 WSTRB 部分写）+ 读回
//   G2 突发读写寄存器
//   G3 DECERR 访问
//------------------------------------------------------------------------------
module tb_axi_reg;
  timeunit 1ns / 1ps;

  localparam MAX_WAIT = 100000;

  // ---- 时钟 / 复位 ----
  logic clk;
  logic rstn;
  initial clk = 1'b0;
  always #5 clk = ~clk;

  // ---- 互联连线 + DUT（共用）----
  `include "tb_axi_wires.svh"
  // 直通属性默认 0。注意：prot 的 master1 切片由 lite master 驱动，
  // 这里只覆盖 master0（cfg master 无 prot 端口）
  initial begin
    s_awlock = '0; s_awcache = '0;
    s_awprot[0*3 +: 3] = '0;
    s_awqos = '0; s_awregion = '0;
    s_arlock = '0; s_arcache = '0;
    s_arprot[0*3 +: 3] = '0;
    s_arqos = '0; s_arregion = '0;
  end


  // ---- master0（cfg）配置 ----
  logic cfg0_wr_start, cfg0_rd_start;
  logic [`AXI_ADDR_W-1:0]   cfg0_addr;
  logic [7:0]               cfg0_len;
  logic [2:0]               cfg0_size;
  logic [1:0]               cfg0_burst;
  logic [`AXI_ID_W-1:0]     cfg0_id;
  logic [`AXI_DATA_W-1:0]   cfg0_wdata0;
  logic [`AXI_DATA_W/8-1:0] cfg0_wstrb;

  // ---- reg slave 调试读口 ----
  logic [$clog2(64)-1:0] dbg_sel;
  logic [`AXI_DATA_W-1:0] dbg_val_c;

  // ---- master1（lite）配置 ----
  logic lite_wr_start, lite_rd_start;
  logic [`AXI_ADDR_W-1:0]   lite_addr;
  logic [`AXI_DATA_W-1:0]   lite_wdata;
  logic [`AXI_DATA_W/8-1:0] lite_wstrb;
  logic [2:0]               lite_prot;

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

  // ---- master1：AXI4-Lite 风格 master ----
  axi_master_lite #(
    .MST_ID (1)
  ) mst1 (
    .clk (clk), .rstn (rstn),
    .start_wr (lite_wr_start),
    .start_rd (lite_rd_start),
    .cfg_addr (lite_addr),
    .cfg_wdata (lite_wdata),
    .cfg_wstrb (lite_wstrb),
    .cfg_prot (lite_prot),
    .awvalid (s_awvalid[1]),
    .awaddr (s_awaddr[1*`AXI_ADDR_W +: `AXI_ADDR_W]),
    .awprot (s_awprot[1*3 +: 3]),
    .awready (s_awready[1]),
    .wvalid (s_wvalid[1]),
    .wdata (s_wdata[1*`AXI_DATA_W +: `AXI_DATA_W]),
    .wstrb (s_wstrb[1*`AXI_DATA_W/8 +: `AXI_DATA_W/8]),
    .wready (s_wready[1]),
    .bvalid (s_bvalid[1]),
    .bresp (s_bresp[1*2 +: 2]),
    .bready (s_bready[1]),
    .arvalid (s_arvalid[1]),
    .araddr (s_araddr[1*`AXI_ADDR_W +: `AXI_ADDR_W]),
    .arprot (s_arprot[1*3 +: 3]),
    .arready (s_arready[1]),
    .rvalid (s_rvalid[1]),
    .rdata (s_rdata[1*`AXI_DATA_W +: `AXI_DATA_W]),
    .rresp (s_rresp[1*2 +: 2]),
    .rready (s_rready[1])
  );

  // ---- master1 缺省 AXI4 字段：Lite 子集恒为 单拍/32b/INCR/ID=0/WLAST=1 ----
  initial begin
    s_awlen[1*8 +: 8]     = 8'd0;
    s_awsize[1*3 +: 3]    = 3'd2;
    s_awburst[1*2 +: 2]   = `AXI_BURST_INCR;
    s_awid[1*`AXI_ID_W +: `AXI_ID_W] = '0;
    s_wlast[1]            = 1'b1;   // Lite 单拍恒 WLAST
    s_arlen[1*8 +: 8]     = 8'd0;
    s_arsize[1*3 +: 3]    = 3'd2;
    s_arburst[1*2 +: 2]   = `AXI_BURST_INCR;
    s_arid[1*`AXI_ID_W +: `AXI_ID_W] = '0;
  end

  // slave0：寄存器外设
  axi_slave_reg #(
    .SLV_ID (0),
    .N_REG (64)
  ) slv0 (
    .clk (clk), .rstn (rstn),
    .awvalid (m_awvalid[0]),
    .awid (m_awid[0*`AXI_SLV_ID_W +: `AXI_SLV_ID_W]),
    .awaddr (m_awaddr[0*`AXI_ADDR_W +: `AXI_ADDR_W]),
    .awlen (m_awlen[0*8 +: 8]),
    .awsize (m_awsize[0*3 +: 3]),
    .awburst (m_awburst[0*2 +: 2]),
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
    .arready (m_arready[0]),
    .rvalid (m_rvalid[0]),
    .rid (m_rid[0*`AXI_SLV_ID_W +: `AXI_SLV_ID_W]),
    .rdata (m_rdata[0*`AXI_DATA_W +: `AXI_DATA_W]),
    .rresp (m_rresp[0*2 +: 2]),
    .rlast (m_rlast[0]),
    .rready (m_rready[0]),
    .dbg_sel (dbg_sel),
    .dbg_val (dbg_val_c)
  );

  // ---- slave1：ram 占位 ----
  logic [`AXI_ADDR_W-1:0] dbg_addr1;
  logic [7:0] dbg_byte1_c;
  axi_slave_ram #(
    .SLV_ID (1)
  ) slv1 (
    .clk (clk), .rstn (rstn),
    .awvalid (m_awvalid[1]),
    .awid (m_awid[1*`AXI_SLV_ID_W +: `AXI_SLV_ID_W]),
    .awaddr (m_awaddr[1*`AXI_ADDR_W +: `AXI_ADDR_W]),
    .awlen (m_awlen[1*8 +: 8]),
    .awsize (m_awsize[1*3 +: 3]),
    .awburst (m_awburst[1*2 +: 2]),
    .awlock (m_awlock[1*2]),
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
    .arlock (m_arlock[1*2]),
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

  task automatic chk(input integer cond, input [1023:0] msg);
    if (!cond) begin
      $display("FAIL: %s (t=%0t)", msg, $time);
      $fatal(1);
    end
  endtask

  //--------------------------------------------------------------------------
  // 驱动：cfg master 写/读（带 WSTRB）
  //--------------------------------------------------------------------------
  task automatic mst0_wr(input logic [`AXI_ADDR_W-1:0] addr,
                         input logic [7:0] len, input logic [2:0] size,
                         input logic [1:0] burst, input logic [`AXI_ID_W-1:0] id,
                         input logic [`AXI_DATA_W-1:0] base,
                         input logic [`AXI_DATA_W/8-1:0] strb,
                         output logic [1:0] status);
    integer cnt;
    begin
      cfg0_addr   = addr;
      cfg0_len    = len;
      cfg0_size   = size;
      cfg0_burst  = burst;
      cfg0_id     = id;
      cfg0_wdata0 = base;
      cfg0_wstrb  = strb;
      @(negedge clk);
      cfg0_wr_start = 1'b1;
      @(negedge clk);
      cfg0_wr_start = 1'b0;
      cnt = 0;
      while (!mst0.wr_done) begin
        @(posedge clk);
        cnt = cnt + 1;
        if (cnt > MAX_WAIT) $fatal(1, "REG: cfg write timeout");
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
        if (cnt > MAX_WAIT) $fatal(1, "REG: cfg read timeout");
      end
      status   = mst0.rd_status;
      checksum = mst0.rd_checksum;
    end
  endtask

  // 读调试口
  task automatic reg_rd(input integer sel, output logic [`AXI_DATA_W-1:0] v);
    begin
      dbg_sel = sel;
      #1;
      v = dbg_val_c;
    end
  endtask

  //==========================================================================
  // 场景
  //==========================================================================

  //--------------------------------------------------------------------------
  // lite 驱动：写 / 读（master1）
  //--------------------------------------------------------------------------
  task automatic lite_wr(input logic [`AXI_ADDR_W-1:0] addr,
                         input logic [`AXI_DATA_W-1:0] data,
                         input logic [`AXI_DATA_W/8-1:0] strb,
                         output logic [1:0] status);
    integer cnt;
    begin
      lite_addr  = addr;
      lite_wdata = data;
      lite_wstrb = strb;
      @(negedge clk);
      lite_wr_start = 1'b1;
      @(negedge clk);
      lite_wr_start = 1'b0;
      cnt = 0;
      while (!mst1.wr_done) begin
        @(posedge clk);
        cnt = cnt + 1;
        if (cnt > MAX_WAIT) $fatal(1, "REG: lite write timeout");
      end
      status = mst1.wr_status;
    end
  endtask

  task automatic lite_rd(input logic [`AXI_ADDR_W-1:0] addr,
                         output logic [1:0] status,
                         output logic [`AXI_DATA_W-1:0] data);
    integer cnt;
    begin
      lite_addr = addr;
      @(negedge clk);
      lite_rd_start = 1'b1;
      @(negedge clk);
      lite_rd_start = 1'b0;
      cnt = 0;
      while (!mst1.rd_done) begin
        @(posedge clk);
        cnt = cnt + 1;
        if (cnt > MAX_WAIT) $fatal(1, "REG: lite read timeout");
      end
      status = mst1.rd_status;
      data   = mst1.rd_data;
    end
  endtask

  // G1：寄存器写（全字 + WSTRB 部分写）+ 读回
  task automatic g1();
    logic [1:0] st;
    logic [`AXI_DATA_W-1:0] v, chk;
    begin
      // 全字写 reg[8]（地址 0x20）
      mst0_wr(32'h0000_0020, 8'd0, 3'd2, `AXI_BURST_FIXED, 4'd0,
              32'h1234_5678, 4'hF, st);
      chk(st == 0, "G1 wr reg8 status");
      // 部分写 reg[9]：只写低 2 字节（wstrb=0011，data 低 2 字节 = 0x2222）
      mst0_wr(32'h0000_0024, 8'd0, 3'd2, `AXI_BURST_FIXED, 4'd0,
              32'h0000_2222, 4'h3, st);
      chk(st == 0, "G1 wr reg9 status");
      // 调试口比对
      reg_rd(8, v);
      chk(v === 32'h1234_5678, "G1 reg8 value");
      reg_rd(9, v);
      chk(v === 32'h0000_2222, "G1 reg9 partial write");
      // 读回校验
      mst0_rd(32'h0000_0020, 8'd0, 3'd2, `AXI_BURST_FIXED, 4'd0, st, chk);
      chk(st == 0 && chk === 32'h1234_5678, "G1 read reg8");
      mst0_rd(32'h0000_0024, 8'd0, 3'd2, `AXI_BURST_FIXED, 4'd0, st, chk);
      chk(st == 0 && chk === 32'h0000_2222, "G1 read reg9");
      $display("[%0t] PASS G1", $time);
    end
  endtask

  // G2：突发写读寄存器（reg[16..23]，数据 = base + 拍号）
  task automatic g2();
    logic [1:0] st;
    logic [`AXI_DATA_W-1:0] chk, echk, v;
    begin
      mst0_wr(32'h0000_0040, 8'd7, 3'd2, `AXI_BURST_INCR, 4'd0,
              32'hAAAA_0000, 4'hF, st);
      chk(st == 0, "G2 wr burst status");
      // 调试口抽查
      reg_rd(16, v); chk(v === 32'hAAAA_0000, "G2 reg16");
      reg_rd(20, v); chk(v === 32'hAAAA_0004, "G2 reg20");
      reg_rd(23, v); chk(v === 32'hAAAA_0007, "G2 reg23");
      // 读回：checksum = XOR(base + b)
      mst0_rd(32'h0000_0040, 8'd7, 3'd2, `AXI_BURST_INCR, 4'd0, st, chk);
      chk(st == 0, "G2 rd burst status");
      echk = 32'hAAAA_0000 ^ 32'hAAAA_0001 ^ 32'hAAAA_0002 ^ 32'hAAAA_0003 ^
             32'hAAAA_0004 ^ 32'hAAAA_0005 ^ 32'hAAAA_0006 ^ 32'hAAAA_0007;
      chk(chk === echk, "G2 checksum");
      $display("[%0t] PASS G2", $time);
    end
  endtask

  // G3：DECERR 访问
  task automatic g3();
    logic [1:0] st;
    logic [`AXI_DATA_W-1:0] chk;
    begin
      mst0_wr(32'h8000_0000, 8'd0, 3'd2, `AXI_BURST_FIXED, 4'd0,
              32'h0, 4'hF, st);
      chk(st == 3, "G3 wr DECERR status");
      mst0_rd(32'h8000_0010, 8'd0, 3'd2, `AXI_BURST_FIXED, 4'd0, st, chk);
      chk(st == 3, "G3 rd DECERR status");
      $display("[%0t] PASS G3", $time);
    end
  endtask

  //==========================================================================
  // G4：Lite master 写读寄存器（部分写）
  //==========================================================================
  task automatic g4();
    logic [1:0] st;
    logic [`AXI_DATA_W-1:0] d, v;
    begin
      // 全字写 reg[10]（地址 0x28）
      lite_wr(32'h0000_0028, 32'hDEAD_BEEF, 4'hF, st);
      chk(st == 0, "G4 lite wr status");
      // 部分写 reg[11]：低 2 字节 = 0x1234
      lite_wr(32'h0000_002C, 32'h0000_1234, 4'h3, st);
      chk(st == 0, "G4 lite partial wr status");
      // 读回
      lite_rd(32'h0000_0028, st, d);
      chk(st == 0 && d === 32'hDEAD_BEEF, "G4 lite rd reg10");
      lite_rd(32'h0000_002C, st, d);
      chk(st == 0 && d === 32'h0000_1234, "G4 lite rd reg11");
      // 调试口交叉比对
      reg_rd(10, v); chk(v === 32'hDEAD_BEEF, "G4 dbg reg10");
      reg_rd(11, v); chk(v === 32'h0000_1234, "G4 dbg reg11");
      $display("[%0t] PASS G4", $time);
    end
  endtask

  //==========================================================================
  // G5：cfg（m0）+ lite（m1）并发访问不同 slave
  //==========================================================================
  task automatic g5();
    logic [1:0] st0, st1;
    logic [`AXI_DATA_W-1:0] v5;
    begin
      fork
        begin
          mst0_wr(32'h0000_0060, 8'd0, 3'd2, `AXI_BURST_FIXED, 4'd0,
                  32'hCAFE_0000, 4'hF, st0);
          chk(st0 == 0, "G5 cfg wr status");
        end
        begin
          // lite 写 slave1（ram 占位实例）：地址 0x1000_0010
          lite_wr(32'h1000_0010, 32'h0102_0304, 4'hF, st1);
          chk(st1 == 0, "G5 lite wr status");
        end
      join
      // 调试口比对（reg[24] = 0x60 >> 2）
      reg_rd(24, v5);
      chk(v5 === 32'hCAFE_0000, "G5 dbg reg24");
      $display("[%0t] PASS G5", $time);
    end
  endtask

  //==========================================================================
  // G6：Lite master DECERR 访问
  //==========================================================================
  task automatic g6();
    logic [1:0] st;
    logic [`AXI_DATA_W-1:0] d;
    begin
      lite_wr(32'h8000_0000, 32'h0, 4'hF, st);
      chk(st == 3, "G6 lite wr DECERR status");
      lite_rd(32'h8000_0010, st, d);
      chk(st == 3, "G6 lite rd DECERR status");
      $display("[%0t] PASS G6", $time);
    end
  endtask

  //--------------------------------------------------------------------------
  // 主流程
  //--------------------------------------------------------------------------
  initial begin
    $display("=== AXI4 Interconnect Register-peripheral TB ===");
    cfg0_wr_start = 1'b0;
    cfg0_rd_start = 1'b0;
    rstn = 1'b0;

    $dumpfile("axi_reg.vcd");
    $dumpvars(0, tb_axi_reg);

    repeat (10) @(posedge clk);
    rstn = 1'b1;
    repeat (2) @(posedge clk);

    g1();
    g2();
    g3();
    g4();
    g5();
    g6();

    $display("========================================");
    $display("ALL REGISTER SCENARIOS PASS");
    $finish;
  end

  // 全局超时看门狗
  initial begin
    repeat (2000000) @(posedge clk);
    $display("FAIL: global watchdog timeout");
    $fatal(1);
  end

endmodule
