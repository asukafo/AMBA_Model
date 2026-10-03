`include "axi_defs.svh"
//------------------------------------------------------------------------------
// tb_axi_rtl - RTL-level co-verification: synthesizable master (axi_master_cfg) x interconnect x
// synthesizable RAM slave (axi_slave_ram)
//
// Complements tb_axi (BFM-level verification): here every bus role is real RTL;
// the TB only drives configuration registers and checks results, verifying
//
// interoperability between the interconnect and real timed modules.
// Checks:
//   1. status codes returned by the master FSMs (OKAY/DECERR)
//   2. master read checksums (read-beat XOR accumulation) vs the TB reference model
//   3. byte-by-byte comparison of the RAM slave debug read port against the reference memory
//
//   4. global timeout watchdog
// Scenarios:
//   R1 single master write/read-back (multi-beat INCR, checksum comparison)
//   R2 two masters concurrently write/read different slaves
//   R3 two masters hammer one slave (arbitration stress + memory comparison)
//   R4 DECERR write/read (status-code checks)
//   R5 WRAP burst write/read-back
//------------------------------------------------------------------------------
module tb_axi_rtl;
  timeunit 1ns / 1ps;

  localparam MAX_WAIT = 100000;

//   R6 cross traffic: m0 writes slave0 while m1 reads slave1
  reg clk;
  reg rstn;
  initial clk = 1'b0;
  always #5 clk = ~clk;   // 100MHz

  // ---- Clock / reset ----
  `include "tb_axi_wires.svh"
  // ---- Interconnect wires + DUT (shared) ----
  assign s_awlock = '0;
  assign s_awcache = '0;
  assign s_awprot = '0;
  assign s_awqos = '0;
  assign s_awregion = '0;
  assign s_arlock = '0;
  assign s_arcache = '0;
  assign s_arprot = '0;
  assign s_arqos = '0;
  assign s_arregion = '0;


  // Pass-through attributes default to 0 (dedicated masters drive their bits in the exclusive/narrow scenarios)
  reg [2*`AXI_ADDR_W-1:0] cfg_addr;
  reg [2*8-1:0] cfg_len;
  reg [2*3-1:0] cfg_size;
  reg [2*2-1:0] cfg_burst;
  reg [2*`AXI_ID_W-1:0] cfg_id;
  reg [2*`AXI_DATA_W-1:0] cfg_wdata0;
  reg [2*`AXI_DATA_W/8-1:0] cfg_wstrb;
  reg [2-1:0] cfg_wr_start;
  reg [2-1:0] cfg_rd_start;

  // ---- Master config (one flat segment per master, accessed via [i*W +: W]) ----
  reg [2*`AXI_ADDR_W-1:0]      ram_dbg_addr;

  // ---- RAM debug read ports ----
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

  // ---- RTL master instances ----
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

  // ---- RTL RAM slave instances ----
  wire [7:0] ram_dbg_byte_c [`AXI_M_SLAVE];

  //--------------------------------------------------------------------------
  // RAM debug read-port outputs (combinational): TB-internal arrays receive the instance outputs
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

  // Reference memory (updated in lockstep with axi_slave_ram, used for byte-by-byte comparison)
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
          a = axi_beat_addr(addr, burst, size, len, b);
          d = base + b;
          for (int i = 0; i < `AXI_DATA_W/8; i++)
            ref_mem[s*4096 + a[11:0] + i] = d[8*i +: 8];
        end
      end
    end
  endtask

  // Reference-memory update (master writes beat b data = base + b, WSTRB all ones)
  task automatic ram_word(input integer s, input [`AXI_ADDR_W-1:0] a,
                          output [`AXI_DATA_W-1:0] w);
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

  // Read one word from the RAM debug port (little-endian)
  task automatic ram_compare();
    begin
      for (int s = 0; s < `AXI_M_SLAVE; s++) begin
        for (int i = 0; i < 4096; i++) begin
          reg [`AXI_ADDR_W-1:0] a;
          reg [7:0] b;
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
  // Byte-by-byte comparison of the reference memory against the RAM
  //--------------------------------------------------------------------------
  task automatic mst_wr(input integer m, input [`AXI_ADDR_W-1:0] addr,
                        input [7:0] len, input [2:0] size,
                        input [1:0] burst, input [`AXI_ID_W-1:0] id,
                        input [`AXI_DATA_W-1:0] base,
                        output [1:0] status);
    integer cnt;
    begin
  // Driver tasks: configure the master, launch a write/read transaction, wait for completion
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

  task automatic mst_rd(input integer m, input [`AXI_ADDR_W-1:0] addr,
                        input [7:0] len, input [2:0] size,
                        input [1:0] burst, input [`AXI_ID_W-1:0] id,
                        output [1:0] status,
                        output [`AXI_DATA_W-1:0] checksum);
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

      // Config writes go to TB-side wires (instance input ports cannot be driven hierarchically)
  task automatic exp_checksum(
    input [`AXI_ADDR_W-1:0] addr,
    input [1:0] burst, input [2:0] size, input [7:0] len,
    output [`AXI_DATA_W-1:0] chk
  );
    integer s;
    reg [`AXI_DATA_W-1:0] acc;
    begin
      s = slave_of(addr);
      acc = '0;
      if (s >= 0) begin
        for (int b = 0; b <= len; b++) begin
          reg [`AXI_ADDR_W-1:0] a;
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
  // Expected checksum: recompute the read burst's XOR from the reference memory
  //--------------------------------------------------------------------------

  // Scenarios
  task automatic r1();
    reg [1:0] st;
    reg [`AXI_DATA_W-1:0] chk, echk;
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

  // R1: single master write/read-back (multi-beat INCR)
  task automatic r2();
    reg [1:0] st0, st1;
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

  // R2: two masters concurrently write/read different slaves
  task automatic r3();
    reg [1:0] st0, st1;
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

  // R3: two masters hammer one slave (arbitration stress)
  task automatic r4();
    reg [1:0] st;
    reg [`AXI_DATA_W-1:0] chk;
    begin
      mst_wr(0, 32'h8000_0000, 8'd1, 3'd2, `AXI_BURST_INCR, 4'd0, 32'h7000_0000, st);
      chk(st == 3, "R4 wr DECERR status");
      mst_rd(0, 32'h8000_0010, 8'd0, 3'd2, `AXI_BURST_INCR, 4'd0, st, chk);
      chk(st == 3, "R4 rd DECERR status");
      $display("[%0t] PASS R4", $time);
    end
  endtask

  // R4: DECERR write/read
  task automatic r5();
    reg [1:0] st;
    reg [`AXI_DATA_W-1:0] chk, echk;
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

  // R5: WRAP burst write/read-back (start 0x224, 4 beats x 4B, wrap boundary 0x220)
  task automatic r6();
    reg [1:0] st0, st1;
    reg [`AXI_DATA_W-1:0] chk1, echk1;
    begin
  // R6: cross traffic: m0 writes slave0 while m1 reads slave1
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
      // Seed slave1's data first (R1 already wrote 0x1000_0200; read it back)
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

  // Main flow
  initial begin
    repeat (2000000) @(posedge clk);
    $display("FAIL: global watchdog timeout");
    $fatal(1);
  end

endmodule
