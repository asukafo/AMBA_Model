`include "axi_defs.svh"
//------------------------------------------------------------------------------
// tb_axi_rtl — RTL 级联调：可综合 master（axi_master_cfg）× 互联 ×
// 可综合 RAM slave（axi_slave_ram）
//
// 与 tb_axi（BFM 级验证）互补：这里所有总线角色都是真实 RTL，
// TB 只做配置寄存器驱动 + 结果检查，验证互联与真实时序模块的互操作。
//
// 检查手段：
//   1. master 状态机返回的状态码（OKAY/DECERR）
//   2. master 的读校验和（读拍 XOR 累加）vs TB 参考模型复算
//   3. RAM slave 调试读口逐字节比对参考内存
//   4. 全局超时看门狗
//
// 场景：
//   R1 单主写读回（INCR 多拍，校验和比对）
//   R2 双主并发写读不同 slave
//   R3 双主抢同一 slave（仲裁压力 + 内存比对）
//   R4 DECERR 读写（状态码检查）
//   R5 WRAP 突发写读回
//   R6 交叉流量：m0 写 slave0 同时 m1 读 slave1
//------------------------------------------------------------------------------
module tb_axi_rtl;
  timeunit 1ns / 1ps;

  localparam MAX_WAIT = 100000;

  // ---- 时钟 / 复位 ----
  logic clk;
  logic rstn;
  initial clk = 1'b0;
  always #5 clk = ~clk;   // 100MHz

  // ---- 互联连线 + DUT（共用）----
  `include "tb_axi_wires.svh"
  // 直通属性默认 0（互斥/窄传输场景由专属 master 驱动对应位）
  initial begin
    s_awlock = '0; s_awcache = '0; s_awprot = '0;
    s_awqos = '0; s_awregion = '0;
    s_arlock = '0; s_arcache = '0; s_arprot = '0;
    s_arqos = '0; s_arregion = '0;
  end


  // ---- master 配置（每 master 一拍平段，[i*W +: W] 访问）----
  logic [2*`AXI_ADDR_W-1:0]      cfg_addr;
  logic [2*8-1:0]                cfg_len;
  logic [2*3-1:0]                cfg_size;
  logic [2*2-1:0]                cfg_burst;
  logic [2*`AXI_ID_W-1:0]        cfg_id;
  logic [2*`AXI_DATA_W-1:0]      cfg_wdata0;
  logic [2*`AXI_DATA_W/8-1:0]    cfg_wstrb;
  logic [2-1:0]                  cfg_wr_start;
  logic [2-1:0]                  cfg_rd_start;

  // ---- RAM 调试读口 ----
  logic [2*`AXI_ADDR_W-1:0]      ram_dbg_addr;

  // ---- RTL master 实例 ----
  for (genvar i = 0; i < `AXI_N_MASTER; i++) begin : g_mst
    axi_master_cfg #(
      .MST_ID (i)
    ) mst (
      .clk (clk), .rstn (rstn),
      .wr_start (cfg_wr_start[i]),
      .rd_start (cfg_rd_start[i]),
      .cfg_addr (cfg_addr[i*`AXI_ADDR_W +: `AXI_ADDR_W]),
      .cfg_len (cfg_len[i*8 +: 8]),
      .cfg_size (cfg_size[i*3 +: 3]),
      .cfg_burst (cfg_burst[i*2 +: 2]),
      .cfg_id (cfg_id[i*`AXI_ID_W +: `AXI_ID_W]),
      .cfg_wdata0 (cfg_wdata0[i*`AXI_DATA_W +: `AXI_DATA_W]),
      .cfg_wstrb (cfg_wstrb[i*`AXI_DATA_W/8 +: `AXI_DATA_W/8]),
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

  // ---- RTL RAM slave 实例 ----
  for (genvar i = 0; i < `AXI_M_SLAVE; i++) begin : g_ram
    axi_slave_ram #(
      .SLV_ID (i)
    ) ram (
      .clk (clk), .rstn (rstn),
      .awvalid (m_awvalid[i]),
      .awid (m_awid[i*`AXI_SLV_ID_W +: `AXI_SLV_ID_W]),
      .awaddr (m_awaddr[i*`AXI_ADDR_W +: `AXI_ADDR_W]),
      .awlen (m_awlen[i*8 +: 8]),
      .awsize (m_awsize[i*3 +: 3]),
      .awburst (m_awburst[i*2 +: 2]),
      .awlock (m_awlock[i*2]),
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
      .arlock (m_arlock[i*2]),
      .arready (m_arready[i]),
      .rvalid (m_rvalid[i]),
      .rid (m_rid[i*`AXI_SLV_ID_W +: `AXI_SLV_ID_W]),
      .rdata (m_rdata[i*`AXI_DATA_W +: `AXI_DATA_W]),
      .rresp (m_rresp[i*2 +: 2]),
      .rlast (m_rlast[i]),
      .rready (m_rready[i]),
      .dbg_addr (ram_dbg_addr[i*`AXI_ADDR_W +: `AXI_ADDR_W]),
      .dbg_byte (ram_dbg_byte_c[i])
    );
  end

  // RAM 调试读口输出（组合）：TB 内部数组接收实例输出
  logic [7:0] ram_dbg_byte_c [`AXI_M_SLAVE];

  //--------------------------------------------------------------------------
  // 参考内存（与 axi_slave_ram 同步更新，用于逐字节比对）
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

  // 参考内存更新（master 写第 b 拍数据 = base + b，WSTRB 全 1）
  task automatic ref_update(
    input logic [`AXI_ADDR_W-1:0] addr,
    input logic [1:0] burst,
    input logic [2:0] size,
    input logic [7:0] len,
    input logic [`AXI_DATA_W-1:0] base
  );
    integer s;
    begin
      s = slave_of(addr);
      if (s >= 0) begin
        for (int b = 0; b <= len; b++) begin
          logic [`AXI_ADDR_W-1:0] a;
          logic [`AXI_DATA_W-1:0] d;
          a = axi_beat_addr(addr, burst, size, len, b);
          d = base + b;
          for (int i = 0; i < `AXI_DATA_W/8; i++)
            ref_mem[s*4096 + a[11:0] + i] = d[8*i +: 8];
        end
      end
    end
  endtask

  // 从 RAM 调试口读一个字（little-endian）
  task automatic ram_word(input integer s, input logic [`AXI_ADDR_W-1:0] a,
                          output logic [`AXI_DATA_W-1:0] w);
    begin
      if (s == 0) begin
        ram_dbg_addr[0*`AXI_ADDR_W +: `AXI_ADDR_W] = a;
        #1;
        w[7:0]   = ram_dbg_byte_c[0];
        ram_dbg_addr[0*`AXI_ADDR_W +: `AXI_ADDR_W] = a + 1;
        #1;
        w[15:8]  = ram_dbg_byte_c[0];
        ram_dbg_addr[0*`AXI_ADDR_W +: `AXI_ADDR_W] = a + 2;
        #1;
        w[23:16] = ram_dbg_byte_c[0];
        ram_dbg_addr[0*`AXI_ADDR_W +: `AXI_ADDR_W] = a + 3;
        #1;
        w[31:24] = ram_dbg_byte_c[0];
      end else begin
        ram_dbg_addr[1*`AXI_ADDR_W +: `AXI_ADDR_W] = a;
        #1;
        w[7:0]   = ram_dbg_byte_c[1];
        ram_dbg_addr[1*`AXI_ADDR_W +: `AXI_ADDR_W] = a + 1;
        #1;
        w[15:8]  = ram_dbg_byte_c[1];
        ram_dbg_addr[1*`AXI_ADDR_W +: `AXI_ADDR_W] = a + 2;
        #1;
        w[23:16] = ram_dbg_byte_c[1];
        ram_dbg_addr[1*`AXI_ADDR_W +: `AXI_ADDR_W] = a + 3;
        #1;
        w[31:24] = ram_dbg_byte_c[1];
      end
    end
  endtask

  // 参考内存 vs RAM 逐字节比对
  task automatic ram_compare();
    begin
      for (int s = 0; s < `AXI_M_SLAVE; s++) begin
        for (int i = 0; i < 4096; i++) begin
          logic [`AXI_ADDR_W-1:0] a;
          logic [7:0] b;
          a = (s == 0) ? 32'h0000_0000 + i : 32'h1000_0000 + i;
          if (s == 0) begin
            ram_dbg_addr[0*`AXI_ADDR_W +: `AXI_ADDR_W] = a;
            #1;
            b = ram_dbg_byte_c[0];
          end else begin
            ram_dbg_addr[1*`AXI_ADDR_W +: `AXI_ADDR_W] = a;
            #1;
            b = ram_dbg_byte_c[1];
          end
          if (ref_mem[s*4096 + i] !== b) begin
            $display("FAIL: ram_compare slave %0d addr 0x%0h: ref=%0h dut=%0h",
                     s, a, ref_mem[s*4096+i], b);
            $fatal(1);
          end
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
  // 驱动任务：配置 master 并发起写/读事务，等待完成
  //--------------------------------------------------------------------------
  task automatic mst_wr(input integer m, input logic [`AXI_ADDR_W-1:0] addr,
                        input logic [7:0] len, input logic [2:0] size,
                        input logic [1:0] burst, input logic [`AXI_ID_W-1:0] id,
                        input logic [`AXI_DATA_W-1:0] base,
                        output logic [1:0] status);
    integer cnt;
    begin
      // 配置写 TB 侧连线（实例输入端口不能层级驱动）
      cfg_addr[m*`AXI_ADDR_W +: `AXI_ADDR_W] = addr;
      cfg_len[m*8 +: 8]     = len;
      cfg_size[m*3 +: 3]    = size;
      cfg_burst[m*2 +: 2]   = burst;
      cfg_id[m*`AXI_ID_W +: `AXI_ID_W] = id;
      cfg_wdata0[m*`AXI_DATA_W +: `AXI_DATA_W] = base;
      cfg_wstrb[m*`AXI_DATA_W/8 +: `AXI_DATA_W/8] = {(`AXI_DATA_W/8){1'b1}};
      @(negedge clk);
      cfg_wr_start[m] = 1'b1;
      @(negedge clk);
      cfg_wr_start[m] = 1'b0;
      cnt = 0;
      if (m == 0) begin
        while (!g_mst[0].mst.wr_done) begin
          @(posedge clk);
          cnt = cnt + 1;
          if (cnt > MAX_WAIT) $fatal(1, "RTB: mst0 write timeout");
        end
        status = g_mst[0].mst.wr_status;
      end else begin
        while (!g_mst[1].mst.wr_done) begin
          @(posedge clk);
          cnt = cnt + 1;
          if (cnt > MAX_WAIT) $fatal(1, "RTB: mst1 write timeout");
        end
        status = g_mst[1].mst.wr_status;
      end
    end
  endtask

  task automatic mst_rd(input integer m, input logic [`AXI_ADDR_W-1:0] addr,
                        input logic [7:0] len, input logic [2:0] size,
                        input logic [1:0] burst, input logic [`AXI_ID_W-1:0] id,
                        output logic [1:0] status,
                        output logic [`AXI_DATA_W-1:0] checksum);
    integer cnt;
    begin
      cfg_addr[m*`AXI_ADDR_W +: `AXI_ADDR_W] = addr;
      cfg_len[m*8 +: 8]     = len;
      cfg_size[m*3 +: 3]    = size;
      cfg_burst[m*2 +: 2]   = burst;
      cfg_id[m*`AXI_ID_W +: `AXI_ID_W] = id;
      @(negedge clk);
      cfg_rd_start[m] = 1'b1;
      @(negedge clk);
      cfg_rd_start[m] = 1'b0;
      cnt = 0;
      if (m == 0) begin
        while (!g_mst[0].mst.rd_done) begin
          @(posedge clk);
          cnt = cnt + 1;
          if (cnt > MAX_WAIT) $fatal(1, "RTB: mst0 read timeout");
        end
        status   = g_mst[0].mst.rd_status;
        checksum = g_mst[0].mst.rd_checksum;
      end else begin
        while (!g_mst[1].mst.rd_done) begin
          @(posedge clk);
          cnt = cnt + 1;
          if (cnt > MAX_WAIT) $fatal(1, "RTB: mst1 read timeout");
        end
        status   = g_mst[1].mst.rd_status;
        checksum = g_mst[1].mst.rd_checksum;
      end
    end
  endtask

  // 期望校验和：按参考内存复算读突发的 XOR
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
          a = axi_beat_addr(addr, burst, size, len, b);
          acc = acc ^ {ref_mem[s*4096 + a[11:0] + 3],
                       ref_mem[s*4096 + a[11:0] + 2],
                       ref_mem[s*4096 + a[11:0] + 1],
                       ref_mem[s*4096 + a[11:0]]};
        end
      end
      chk = acc;
    end
  endtask

  //--------------------------------------------------------------------------
  // 场景
  //--------------------------------------------------------------------------

  // R1：单主写读回（INCR 多拍）
  task automatic r1();
    logic [1:0] st;
    logic [`AXI_DATA_W-1:0] chk, echk;
    begin
      mst_wr(0, 32'h0000_0100, 8'd7, 3'd2, `AXI_BURST_INCR, 4'd0, 32'h1000_0000, st);
      chk(st == 0, "R1 m0 wr status");
      ref_update(32'h0000_0100, `AXI_BURST_INCR, 3'd2, 8'd7, 32'h1000_0000);
      mst_rd(0, 32'h0000_0100, 8'd7, 3'd2, `AXI_BURST_INCR, 4'd0, st, chk);
      chk(st == 0, "R1 m0 rd status");
      exp_checksum(32'h0000_0100, `AXI_BURST_INCR, 3'd2, 8'd7, echk);
      chk(chk === echk, "R1 m0 checksum");
      mst_wr(1, 32'h1000_0200, 8'd3, 3'd2, `AXI_BURST_INCR, 4'd1, 32'h2000_0000, st);
      chk(st == 0, "R1 m1 wr status");
      ref_update(32'h1000_0200, `AXI_BURST_INCR, 3'd2, 8'd3, 32'h2000_0000);
      mst_rd(1, 32'h1000_0200, 8'd3, 3'd2, `AXI_BURST_INCR, 4'd1, st, chk);
      chk(st == 0, "R1 m1 rd status");
      exp_checksum(32'h1000_0200, `AXI_BURST_INCR, 3'd2, 8'd3, echk);
      chk(chk === echk, "R1 m1 checksum");
      ram_compare();
      $display("[%0t] PASS R1", $time);
    end
  endtask

  // R2：双主并发写读不同 slave
  task automatic r2();
    logic [1:0] st0, st1;
    begin
      fork
        begin
          mst_wr(0, 32'h0000_0400, 8'd7, 3'd2, `AXI_BURST_INCR, 4'd0, 32'h3000_0000, st0);
          chk(st0 == 0, "R2 m0 wr status");
        end
        begin
          mst_wr(1, 32'h1000_0500, 8'd7, 3'd2, `AXI_BURST_INCR, 4'd1, 32'h4000_0000, st1);
          chk(st1 == 0, "R2 m1 wr status");
        end
      join
      ref_update(32'h0000_0400, `AXI_BURST_INCR, 3'd2, 8'd7, 32'h3000_0000);
      ref_update(32'h1000_0500, `AXI_BURST_INCR, 3'd2, 8'd7, 32'h4000_0000);
      ram_compare();
      $display("[%0t] PASS R2", $time);
    end
  endtask

  // R3：双主抢同一 slave（仲裁压力）
  task automatic r3();
    logic [1:0] st0, st1;
    begin
      fork
        begin
          for (int i = 0; i < 16; i++) begin
            mst_wr(0, 32'h0000_0600 + 32*i, 8'd3, 3'd2, `AXI_BURST_INCR, 4'd0, 32'h5000_0000 + i, st0);
            chk(st0 == 0, "R3 m0 wr status");
            ref_update(32'h0000_0600 + 32*i, `AXI_BURST_INCR, 3'd2, 8'd3, 32'h5000_0000 + i);
          end
        end
        begin
          for (int i = 0; i < 16; i++) begin
            mst_wr(1, 32'h0000_0800 + 32*i, 8'd3, 3'd2, `AXI_BURST_INCR, 4'd1, 32'h6000_0000 + i, st1);
            chk(st1 == 0, "R3 m1 wr status");
            ref_update(32'h0000_0800 + 32*i, `AXI_BURST_INCR, 3'd2, 8'd3, 32'h6000_0000 + i);
          end
        end
      join
      ram_compare();
      $display("[%0t] PASS R3", $time);
    end
  endtask

  // R4：DECERR 读写
  task automatic r4();
    logic [1:0] st;
    logic [`AXI_DATA_W-1:0] chk;
    begin
      mst_wr(0, 32'h8000_0000, 8'd1, 3'd2, `AXI_BURST_INCR, 4'd0, 32'h7000_0000, st);
      chk(st == 3, "R4 wr DECERR status");
      mst_rd(0, 32'h8000_0010, 8'd0, 3'd2, `AXI_BURST_INCR, 4'd0, st, chk);
      chk(st == 3, "R4 rd DECERR status");
      $display("[%0t] PASS R4", $time);
    end
  endtask

  // R5：WRAP 突发写读回（起点 0x224，4 拍 × 4B，回卷边界 0x220）
  task automatic r5();
    logic [1:0] st;
    logic [`AXI_DATA_W-1:0] chk, echk;
    begin
      mst_wr(0, 32'h0000_0224, 8'd3, 3'd2, `AXI_BURST_WRAP, 4'd0, 32'h8000_0000, st);
      chk(st == 0, "R5 wr status");
      ref_update(32'h0000_0224, `AXI_BURST_WRAP, 3'd2, 8'd3, 32'h8000_0000);
      mst_rd(0, 32'h0000_0224, 8'd3, 3'd2, `AXI_BURST_WRAP, 4'd0, st, chk);
      chk(st == 0, "R5 rd status");
      exp_checksum(32'h0000_0224, `AXI_BURST_WRAP, 3'd2, 8'd3, echk);
      chk(chk === echk, "R5 checksum");
      ram_compare();
      $display("[%0t] PASS R5", $time);
    end
  endtask

  // R6：交叉流量：m0 写 slave0 同时 m1 读 slave1
  task automatic r6();
    logic [1:0] st0, st1;
    logic [`AXI_DATA_W-1:0] chk1, echk1;
    begin
      // 先给 slave1 放数据（R1 已写过 0x1000_0200，直接读回）
      fork
        begin
          mst_wr(0, 32'h0000_0A00, 8'd7, 3'd2, `AXI_BURST_INCR, 4'd0, 32'h9000_0000, st0);
          chk(st0 == 0, "R6 m0 wr status");
        end
        begin
          mst_rd(1, 32'h1000_0200, 8'd3, 3'd2, `AXI_BURST_INCR, 4'd1, st1, chk1);
          chk(st1 == 0, "R6 m1 rd status");
        end
      join
      ref_update(32'h0000_0A00, `AXI_BURST_INCR, 3'd2, 8'd7, 32'h9000_0000);
      exp_checksum(32'h1000_0200, `AXI_BURST_INCR, 3'd2, 8'd3, echk1);
      chk(chk1 === echk1, "R6 m1 checksum");
      ram_compare();
      $display("[%0t] PASS R6", $time);
    end
  endtask

  //--------------------------------------------------------------------------
  // 主流程
  //--------------------------------------------------------------------------
  initial begin
    $display("=== AXI4 Interconnect RTL-level TB ===");
    cfg_wr_start = 2'b00;
    cfg_rd_start = 2'b00;
    rstn = 1'b0;

    $dumpfile("axi_rtl.vcd");
    $dumpvars(0, tb_axi_rtl);

    repeat (10) @(posedge clk);
    rstn = 1'b1;
    repeat (2) @(posedge clk);

    r1();
    r2();
    r3();
    r4();
    r5();
    r6();

    $display("========================================");
    $display("ALL RTL-LEVEL SCENARIOS PASS");
    $finish;
  end

  // 全局超时看门狗
  initial begin
    repeat (2000000) @(posedge clk);
    $display("FAIL: global watchdog timeout");
    $fatal(1);
  end

endmodule
