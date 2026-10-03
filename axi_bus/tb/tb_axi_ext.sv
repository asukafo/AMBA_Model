`include "axi_defs.svh"
//------------------------------------------------------------------------------
// tb_axi_ext — 互斥访问 + LFSR 流量 RTL 级联调：
//   master0 = axi_master_excl（ARLOCK/AWLOCK 互斥事务，检查 EXOKAY）
//   master1 = axi_master_lfsr（LFSR 随机流量 soak 源）
//   slave0  = axi_slave_ram（带互斥监视器）
//   slave1  = axi_slave_lat
//
// 检查手段：excl 的 rd/wr 响应码与 wr_issued；lfsr 的完成计数、
// resp_err、读校验和（TB 在每笔读发出时按参考内存快照累加期望值——
// 单 outstanding 下发出时 == 读出时）；参考内存逐字节比对。
//
// 场景：
//   E1 互斥成功路径（读 EXOKAY → 条件写 EXOKAY）
//   E2 互斥丢失（读与写之间被另一 master 普通写打断 → 写回 OKAY）
//   E3 LFSR soak（两个地址窗口各 60 笔，校验和与内存比对）
//   E4 互斥 + LFSR 并发
//------------------------------------------------------------------------------
module tb_axi_ext;
  timeunit 1ns / 1ps;

  localparam MAX_WAIT = 200000;

  // ---- 时钟 / 复位 ----
  reg clk;
  reg rstn;
  initial clk = 1'b0;
  always #5 clk = ~clk;

  // ---- 互联连线 + DUT（共用）----
  `include "tb_axi_wires.svh"

  // ---- master0（excl）配置 ----
  reg excl_start;
  reg [`AXI_ADDR_W-1:0] excl_addr;
  reg [7:0] excl_len;
  reg [2:0] excl_size;
  reg [1:0] excl_burst;
  reg [`AXI_ID_W-1:0] excl_id;
  reg [`AXI_DATA_W-1:0] excl_wdata0;
  reg [7:0] excl_wr_delay;

  // ---- master1（lfsr）配置 ----
  reg lfsr_enable;
  reg [15:0] lfsr_tx_max;
  reg [`AXI_ADDR_W-1:0] lfsr_base;
  reg [`AXI_ADDR_W-1:0] lfsr_mask;
  reg [31:0] lfsr_seed;
  reg [15:0] lfsr_gen_cnt;

  // ---- master0：excl ----
  axi_master_excl #(
    .MST_ID (0)
  ) mst0 (
    .clk (clk), .rstn (rstn),
    .start (excl_start),
    .cfg_addr (excl_addr),
    .cfg_len (excl_len),
    .cfg_size (excl_size),
    .cfg_burst (excl_burst),
    .cfg_id (excl_id),
    .cfg_wdata0 (excl_wdata0),
    .cfg_wr_delay (excl_wr_delay),
    .awvalid (s_awvalid[0]),
    .awid (s_awid[0*`AXI_ID_W +: `AXI_ID_W]),
    .awaddr (s_awaddr[0*`AXI_ADDR_W +: `AXI_ADDR_W]),
    .awlen (s_awlen[0*8 +: 8]),
    .awsize (s_awsize[0*3 +: 3]),
    .awburst (s_awburst[0*2 +: 2]),
    .awlock (s_awlock[0*2]),
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
    .arlock (s_arlock[0*2]),
    .arready (s_arready[0]),
    .rvalid (s_rvalid[0]),
    .rid (s_rid[0*`AXI_ID_W +: `AXI_ID_W]),
    .rdata (s_rdata[0*`AXI_DATA_W +: `AXI_DATA_W]),
    .rresp (s_rresp[0*2 +: 2]),
    .rlast (s_rlast[0]),
    .rready (s_rready[0])
  );

  // ---- master1：lfsr ----
  axi_master_lfsr #(
    .MST_ID (1)
  ) mst1 (
    .clk (clk), .rstn (rstn),
    .enable (lfsr_enable),
    .cfg_tx_max (lfsr_tx_max),
    .cfg_base (lfsr_base),
    .cfg_mask (lfsr_mask),
    .cfg_seed (lfsr_seed),
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
    .rready (s_rready[1]),
    .gen_cnt (lfsr_gen_cnt)
  );

  // ---- 直通属性：master1（lfsr）lock 恒 0；excl 驱动 master0 的 lock ----
  assign s_awlock[1*2]    = 1'b0;
  assign s_awlock[1*2+1]  = 1'b0;
  assign s_arlock[1*2]    = 1'b0;
  assign s_arlock[1*2+1]  = 1'b0;
  assign s_awlock[0*2+1]  = 1'b0;   // excl 只驱动 bit0
  assign s_arlock[0*2+1]  = 1'b0;
  assign s_awcache = '0;
  assign s_awprot = '0;
  assign s_awqos = '0;
  assign s_awregion = '0;
  assign s_arcache = '0;
  assign s_arprot = '0;
  assign s_arqos = '0;
  assign s_arregion = '0;

  // ---- slave0：ram（互斥监视器）----
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

  reg [`AXI_ADDR_W-1:0] dbg_addr0, dbg_addr1;
  wire [7:0] dbg_byte0_c, dbg_byte1_c;
  reg [7:0] lat_b_delay, lat_r_delay;
  reg [1:0] lat_err_mode;
  reg [7:0] lat_err_period;

  //--------------------------------------------------------------------------
  // 参考内存
  //--------------------------------------------------------------------------
  localparam SB_DEPTH = `AXI_M_SLAVE * 4096;
  reg [7:0] ref_mem [0:SB_DEPTH-1];
  initial begin
    for (int i = 0; i < SB_DEPTH; i++)
      ref_mem[i] = 8'h00;
  end

  function automatic integer slave_of(input [`AXI_ADDR_W-1:0] a);
    if ((a & 32'hF000_0000) == 32'h0000_0000) slave_of = 0;
    else if ((a & 32'hF000_0000) == 32'h1000_0000) slave_of = 1;
    else slave_of = -1;
  endfunction

  task automatic ref_update(
    input [`AXI_ADDR_W-1:0] addr,
    input [1:0] burst,
    input [2:0] size,
    input [7:0] len,
    input [`AXI_DATA_W-1:0] base
  );
    integer s;
    begin
      s = slave_of(addr);
      if (s >= 0) begin
        for (int b = 0; b <= len; b++) begin
          reg [`AXI_ADDR_W-1:0] a;
          reg [`AXI_DATA_W-1:0] d;
          reg [`AXI_DATA_W/8-1:0] st;
          a  = axi_beat_addr(addr, burst, size, len, b);
          d  = base + b;
          st = axi_strb_for_size(size, a);
          for (int i = 0; i < `AXI_DATA_W/8; i++)
            if (st[i])
              ref_mem[s*4096 + ((a[11:0] & 12'hFFC) + i)] = d[8*i +: 8];
        end
      end
    end
  endtask

  task automatic mem_compare();
    begin
      for (int s = 0; s < `AXI_M_SLAVE; s++) begin
        for (int i = 0; i < 4096; i++) begin
          reg [`AXI_ADDR_W-1:0] a;
          reg [7:0] b;
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

  task automatic chk(input integer cond, input [1023:0] msg);
    if (!cond) begin
      $display("FAIL: %s (t=%0t)", msg, $time);
      $fatal(1);
    end
  endtask

  //--------------------------------------------------------------------------
  // 驱动：excl master
  //--------------------------------------------------------------------------
  task automatic excl_run(input [`AXI_ADDR_W-1:0] addr,
                          input [7:0] len, input [2:0] size,
                          input [1:0] burst, input [`AXI_ID_W-1:0] id,
                          input [`AXI_DATA_W-1:0] base,
                          input [7:0] wr_delay);
    integer cnt;
    begin
      excl_addr     = addr;
      excl_len      = len;
      excl_size     = size;
      excl_burst    = burst;
      excl_id       = id;
      excl_wdata0   = base;
      excl_wr_delay = wr_delay;
      @(negedge clk);
      excl_start = 1'b1;
      @(negedge clk);
      excl_start = 1'b0;
      cnt = 0;
      while (!mst0.done) begin
        @(posedge clk);
        cnt = cnt + 1;
        if (cnt > MAX_WAIT) $fatal(1, "EXT: excl timeout");
      end
    end
  endtask

  //--------------------------------------------------------------------------
  // 驱动：lfsr master
  //--------------------------------------------------------------------------
  task automatic lfsr_run(input [15:0] tx_max,
                          input [`AXI_ADDR_W-1:0] base,
                          input [`AXI_ADDR_W-1:0] mask,
                          input [31:0] seed);
    integer cnt;
    begin
      lfsr_tx_max = tx_max;
      lfsr_base   = base;
      lfsr_mask   = mask;
      lfsr_seed   = seed;
      @(negedge clk);
      lfsr_enable = 1'b1;
      cnt = 0;
      // 上一批的 done 可能仍挂着：先等它清零（本批启动），再等完成置位
      while (mst1.done) begin
        @(posedge clk);
        cnt = cnt + 1;
        if (cnt > MAX_WAIT) $fatal(1, "EXT: lfsr start timeout");
      end
      cnt = 0;
      while (!mst1.done) begin
        @(posedge clk);
        cnt = cnt + 1;
        if (cnt > MAX_WAIT) $fatal(1, "EXT: lfsr timeout");
      end
      @(negedge clk);
      lfsr_enable = 1'b0;
    end
  endtask

  // 找"首笔为写"的种子用（lfsr 首笔参数直接取 cfg_seed 本身）

  //==========================================================================
  // E1：互斥成功路径
  //==========================================================================
  task automatic e1();
    begin
      excl_run(32'h0000_0100, 8'd3, 3'd2, `AXI_BURST_INCR, 4'd0, 32'hA100_0000, 8'd0);
      chk(mst0.rd_resp == 1, "E1 rd EXOKAY");
      chk(mst0.wr_issued, "E1 wr issued");
      chk(mst0.wr_resp == 1, "E1 wr EXOKAY");
      ref_update(32'h0000_0100, `AXI_BURST_INCR, 3'd2, 8'd3, 32'hA100_0000);
      mem_compare();
      $display("[%0t] PASS E1", $time);
    end
  endtask

  //==========================================================================
  // E2：互斥丢失（读写之间被另一 master 普通写打断 → 写回 OKAY）
  //==========================================================================
  task automatic e2();
    reg [31:0] seed;
    begin
      // 找一个 bit0=0（首笔为写）的种子；首笔参数直接取 seed：
      // dir=seed[0]、len=seed[7:5]、wdata0=seed
      seed = 32'd1;
      while (seed[0] != 1'b0)
        seed = seed + 32'd1;
      // 干扰写：单笔写 slave0（普通写清除监视点）
      fork
        begin
          excl_run(32'h0000_0200, 8'd1, 3'd2, `AXI_BURST_INCR, 4'd0, 32'hA200_0000, 8'd40);
        end
        begin
          lfsr_run(16'd1, 32'h0000_0300, 32'h0, seed);
        end
      join
      chk(mst0.rd_resp == 1, "E2 rd EXOKAY");
      chk(mst0.wr_issued, "E2 wr issued");
      chk(mst0.wr_resp == 0, "E2 wr lost (OKAY)");
      chk(mst1.resp_err == 1'b0, "E2 lfsr no err");
      ref_update(32'h0000_0200, `AXI_BURST_INCR, 3'd2, 8'd1, 32'hA200_0000);
      begin
        reg [7:0] ln;
        ln = seed[7:5];
        ref_update(32'h0000_0300, `AXI_BURST_INCR, 3'd2, ln, seed);
      end
      mem_compare();
      $display("[%0t] PASS E2", $time);
    end
  endtask

  //==========================================================================
  // E3：LFSR soak（两个窗口各 60 笔）
  //==========================================================================
  reg [`AXI_DATA_W-1:0] exp_acc;
  initial exp_acc = '0;

  // 读事务发出时按参考内存快照累加期望校验和（单 outstanding：发出时==读出时）
  always @(posedge clk) begin
    if (lfsr_enable && mst1.tx_vld && mst1.tx_dir) begin
      for (int b = 0; b <= mst1.tx_len; b++) begin
        reg [`AXI_ADDR_W-1:0] a, wb;
        integer s;
        a = axi_beat_addr(mst1.tx_addr, `AXI_BURST_INCR, 3'd2, mst1.tx_len, b);
        s = slave_of(a);
        if (s >= 0) begin
          wb = a & ~(`AXI_ADDR_W'(3));
          exp_acc = exp_acc ^ {ref_mem[s*4096 + wb[11:0] + 3],
                               ref_mem[s*4096 + wb[11:0] + 2],
                               ref_mem[s*4096 + wb[11:0] + 1],
                               ref_mem[s*4096 + wb[11:0]]};
        end
      end
    end
    // 写事务发出时更新参考内存
    if (lfsr_enable && mst1.tx_vld && !mst1.tx_dir) begin
      ref_update(mst1.tx_addr, `AXI_BURST_INCR, 3'd2, mst1.tx_len, mst1.tx_wdata0);
    end
  end

  task automatic e3();
    begin
      exp_acc = '0;
      lfsr_run(16'd60, 32'h0000_0000, 32'h0000_0F80, 32'h1234_5678);
      chk(lfsr_gen_cnt == 60, "E3 w0 gen_cnt");
      chk(mst1.resp_err == 1'b0, "E3 w0 resp_err");
      chk(mst1.tx_cnt == 60, "E3 w0 tx_cnt");
      chk(mst1.rd_checksum === exp_acc, "E3 w0 checksum");
      exp_acc = '0;
      lfsr_run(16'd60, 32'h1000_0000, 32'h0000_0F80, 32'h9ABC_DEF0);
      chk(lfsr_gen_cnt == 60, "E3 w1 gen_cnt");
      chk(mst1.resp_err == 1'b0, "E3 w1 resp_err");
      chk(mst1.rd_checksum === exp_acc, "E3 w1 checksum");
      mem_compare();
      $display("[%0t] PASS E3", $time);
    end
  endtask

  //==========================================================================
  // E4：互斥 + LFSR 并发（不同 slave）
  //==========================================================================
  task automatic e4();
    begin
      lat_b_delay = 8'd2;
      lat_r_delay = 8'd2;
      fork
        begin
          excl_run(32'h0000_0400, 8'd3, 3'd2, `AXI_BURST_INCR, 4'd0, 32'hA400_0000, 8'd0);
        end
        begin
          lfsr_run(16'd30, 32'h1000_0000, 32'h0000_0F80, 32'h5555_AAAA);
        end
      join
      chk(mst0.rd_resp == 1 && mst0.wr_resp == 1, "E4 excl EXOKAY");
      chk(mst1.resp_err == 1'b0, "E4 lfsr no err");
      ref_update(32'h0000_0400, `AXI_BURST_INCR, 3'd2, 8'd3, 32'hA400_0000);
      mem_compare();
      lat_b_delay = 8'd0;
      lat_r_delay = 8'd0;
      $display("[%0t] PASS E4", $time);
    end
  endtask

  //--------------------------------------------------------------------------
  // 主流程
  //--------------------------------------------------------------------------
  initial begin
    $display("=== AXI4 Interconnect Exclusive + LFSR TB ===");
    excl_start = 1'b0;
    lfsr_enable = 1'b0;
    lat_b_delay = 8'd0;
    lat_r_delay = 8'd0;
    lat_err_mode = 2'd0;
    lat_err_period = 8'd4;
    rstn = 1'b0;

    $dumpfile("axi_ext.vcd");
    $dumpvars(0, tb_axi_ext);

    repeat (10) @(posedge clk);
    rstn = 1'b1;
    repeat (2) @(posedge clk);

    e1();
    e2();
    e3();
    e4();

    $display("========================================");
    $display("ALL EXCLUSIVE/LFSR SCENARIOS PASS");
    $finish;
  end

  // 全局超时看门狗
  initial begin
    repeat (2000000) @(posedge clk);
    $display("FAIL: global watchdog timeout");
    $fatal(1);
  end

endmodule
