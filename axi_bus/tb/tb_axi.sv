`include "axi_defs.svh"
//------------------------------------------------------------------------------
// tb_axi - AXI4 crossbar interconnect verification environment (iverilog)
//
// Topology: 2 masters x 2 slaves + internal DECERR responders
//   Address map (active within 4KB windows):
//     slave0: base 0x0000_0000 mask 0xF000_0000
//     slave1: base 0x1000_0000 mask 0xF000_0000
//     DECERR: the 0x8000_0000 region (unmatched)
//
// Checks (three layers):
//   1. BFM self-checks: RID matching, seed-generated data comparison, response codes
//   2. Scoreboard: byte-by-byte comparison of the reference memory against each slave memory
//   3. Monitors: fixed-priority invariant / RR fairness / timeouts
//
// Scenarios (S1-S15):
//   S1  two masters concurrently do INCR writes/reads to different slaves
//   S2  all masters hammer one slave's AW (FIXED build only: strict priority invariant)
//   S3  all masters hammer one slave's AW (RR build only: fairness count +-4)
//   S4  one master reads 2 slaves; slave1 responds first (out of order, RID matching)
//   S5  WRAP burst unaligned write/read
//   S6  narrow transfers with random WSTRB (including all-zero beats)
//   S7  DECERR write/read (B=DECERR, single-beat R=DECERR)
//   S8  W-channel interleaving: two masters get AW grants back to back; m1 drives WVALID early
//   S9  3 outstanding ARs to DECERR (FIFO order)
//   S10 two-master random mixed traffic + random backpressure + sprinkled DECERR
//   S11 random arrival intervals hammering one slave (policy invariant/fairness)
//   S12 multi-beat DECERR write in parallel with another master's real write (slave W backpressure)
//   S13 two slaves assert BVALID in the same cycle, contending for one master's B arbitration
//   S14 R grant lock held while RREADY stalls (no deadlock)
//   S15 single master with multiple concurrent IDs
//------------------------------------------------------------------------------
module tb_axi;
  timeunit 1ns / 1ps;

  // ---- Clock / reset ----
  reg clk;
  reg rstn;
  initial clk = 1'b0;
  always #5 clk = ~clk;   // 100MHz

  `include "tb_axi_wires.svh"

  // Slave config (backpressure probability / read latency)
  reg [6:0] cfg_aw_rdy_pct [`AXI_M_SLAVE];
  reg [6:0] cfg_w_rdy_pct  [`AXI_M_SLAVE];
  reg [7:0] cfg_r_delay    [`AXI_M_SLAVE];


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

  // ---- Slave models ----
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
  // Scoreboard: reference memory (one 4KB window per slave, flattened to 1D)
  //--------------------------------------------------------------------------
  localparam SB_DEPTH = `AXI_M_SLAVE * 4096;
  reg [7:0] ref_mem [0:SB_DEPTH-1];
  initial begin
    for (int i = 0; i < SB_DEPTH; i++)
      ref_mem[i] = 8'h00;
  end

  function automatic integer slave_of;
    input [`AXI_ADDR_W-1:0] a;
    begin
      if ((a & 32'hF000_0000) == 32'h0000_0000) slave_of = 0;
      else if ((a & 32'hF000_0000) == 32'h1000_0000) slave_of = 1;
      else slave_of = -1;
    end
  endfunction

  // Reference-memory update (same seed generation as the BFM; masking matches the writes)
  task automatic sb_write(
    input [`AXI_ADDR_W-1:0] addr,
    input [1:0] burst,
    input [2:0] size,
    input [7:0] len,
    input integer seed,
    input integer strb_seed,
    input integer strb_mode
  );
    integer s;
    begin
      s = slave_of(addr);
      if (s >= 0) begin   // the DECERR region has no memory
        for (int b = 0; b <= len; b++) begin
          reg [`AXI_ADDR_W-1:0] a;
          reg [`AXI_DATA_W-1:0] d;
          reg [`AXI_DATA_W/8-1:0] st;
          a  = axi_beat_addr(addr, burst, size, len, b);
          d  = axi_test_data(seed, b);
          st = axi_test_strb(strb_seed, b, strb_mode);
          for (int i = 0; i < `AXI_DATA_W/8; i++)
            if (st[i]) ref_mem[s*4096 + a[11:0] + i] = d[8*i +: 8];
        end
      end
    end
  endtask

  // Byte-by-byte comparison of the reference memory against the slave memories.
  // Note: iverilog hierarchical references need constant generate indices, so the slave loop is unrolled explicitly
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
  // Monitors: AW handshake counts + arbitration-policy invariants
  //   arb_cnt[m]: contended-handshake ownership count when both requests are active (slave0)
  //     under RR it should alternate strictly (|diff|<=2); under FIXED m1 must never win
  //   arb_viol : under FIXED, granting m1 while m0's request is active = violation
  //--------------------------------------------------------------------------
  integer aw_hs_cnt [`AXI_N_MASTER];
  integer arb_cnt [`AXI_N_MASTER];
  integer arb_viol;
  reg     grant1_prev;

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
      // Contended-handshake ownership (slave0: dbg bit 0 = m0 request, bit 1 = m1 request)
      if (dbg_aw_req[0] && dbg_aw_req[1]) begin
        if (s_awvalid[0] && s_awready[0]) arb_cnt[0] = arb_cnt[0] + 1;
        if (s_awvalid[1] && s_awready[1]) arb_cnt[1] = arb_cnt[1] + 1;
      end
      // Fixed-priority invariant: at the moment slave0's AW grant goes to m1,
      // m0's arbitration request (already masked by w_pending) must not be 1
      // dbg flattened bit order: bit [s*N_MASTER + m] -> slave0.m1 = bit 1, slave0.m0 = bit 0
      if (`AXI_ARB_POLICY == 0) begin
        if (dbg_aw_grant[1] && dbg_aw_req[0] && !grant1_prev)
          arb_viol = 1;
      end
      grant1_prev = dbg_aw_grant[1];
    end
  end

  // Global timeout watchdog
  initial begin
    repeat (2000000) @(posedge clk);
    $display("FAIL: global watchdog timeout");
    $fatal(1);
  end

  //==========================================================================
  // S1: two masters concurrently do INCR writes/reads to different slaves
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
  // S2/S3: all masters hammer slave0's AW (policy check selected by build)
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
        // RR: contended-handshake ownership must alternate strictly
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
  // S4: one master reads 2 slaves; slave1 responds first (out-of-order responses)
  //==========================================================================
  task automatic s4();
    integer r;
    begin
      // Write the data first
      g_mst[0].bfm.write(32'h0000_0300, `AXI_BURST_INCR, 3'd2, 8'd3, 4'd0, 300, 0, 0, r);
      chk(r == 0, "S4 wr0");
      g_mst[0].bfm.write(32'h1000_0400, `AXI_BURST_INCR, 3'd2, 8'd3, 4'd1, 310, 0, 0, r);
      chk(r == 0, "S4 wr1");
      sb_write(32'h0000_0300, `AXI_BURST_INCR, 3'd2, 8'd3, 300, 0, 0);
      sb_write(32'h1000_0400, `AXI_BURST_INCR, 3'd2, 8'd3, 310, 0, 0);
      // slave0 is slow (10 cycles), slave1 fast (1 cycle) -> slave1's R arrives first
      cfg_r_delay[0] = 8'd10;
      cfg_r_delay[1] = 8'd1;
      g_mst[0].bfm.ar_only(32'h0000_0300, `AXI_BURST_INCR, 3'd2, 8'd3, 4'd0, 300, 0, 0, 0, 0);
      g_mst[0].bfm.ar_only(32'h1000_0400, `AXI_BURST_INCR, 3'd2, 8'd3, 4'd1, 310, 0, 0, 0, 0);
      g_mst[0].bfm.collect_all(0, 0);
      // Completion order: slot 1 (slave1/id1) before slot 0 (slave0/id0)
      chk(g_mst[0].bfm.ar_done_order[0] == 1 &&
          g_mst[0].bfm.ar_done_order[1] == 0, "S4 out-of-order completion");
      cfg_r_delay[0] = 8'd2;
      cfg_r_delay[1] = 8'd2;
      sb_compare();
      $display("[%0t] PASS S4", $time);
    end
  endtask

  //==========================================================================
  // S5: WRAP burst unaligned write/read (start 0x224, 4 beats x 4B, wrap boundary 0x220)
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
  // S6: narrow transfers with random WSTRB (including all-zero beats)
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
  // S7: DECERR write/read
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
  // S8: W-channel interleaving - two masters get AW grants back to back; m1 drives WVALID early while m0 stalls
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
          // m1 drives W data early (wready is blocked by the interconnect until m0's stream completes)
          g_mst[1].bfm.w_only(810, 0, 0);
        end
        begin
          repeat (10) @(posedge clk);   // m0 stalls for 10 cycles
          g_mst[0].bfm.w_only(800, 0, 0);
        end
      join
      g_mst[0].bfm.wait_b(4'd0, r0);
      chk(r0 == 0, "S8 m0 B resp");
      g_mst[1].bfm.wait_b(4'd1, r1);
      chk(r1 == 0, "S8 m1 B resp");
      // W stream order: m0's WLAST must precede m1's first-beat handshake
      chk(g_mst[0].bfm.w_done_time < g_mst[1].bfm.w_first_hs_time,
          "S8 W stream order");
      sb_compare();
      $display("[%0t] PASS S8", $time);
    end
  endtask

  //==========================================================================
  // S9: 3 outstanding ARs to DECERR (FIFO-order responses)
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
  // S10: two-master random mixed traffic + sprinkled DECERR
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
            // Write (the expected-data source for read-back self-checks)
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
  // S11: random arrival intervals hammering one slave (policy invariant/fairness)
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
        // Hierarchical-reference indices must be constant (iverilog); unroll explicitly
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
  // S12: multi-beat DECERR write in parallel with another master's real write (slave W backpressure)
  //==========================================================================
  task automatic s12();
    integer r0, r1;
    begin
      cfg_w_rdy_pct[0] = 7'd50;   // slave0 write backpressure 50%
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
  // S13: two slaves assert BVALID in the same cycle, contending for one master's B arbitration
  //==========================================================================
  task automatic s13();
    integer r0, r1;
    begin
      // Write 1 -> slave0; after B1 asserts, bready stays 0 (pending)
      g_mst[0].bfm.aw_only(32'h0000_0E00, `AXI_BURST_INCR, 3'd2, 8'd1, 4'd0);
      g_mst[0].bfm.w_only(1000, 0, 0);
      // Write 2 -> slave1 (a new AW is allowed once WLAST completes; B1 still pending)
      g_mst[0].bfm.aw_only(32'h1000_0F00, `AXI_BURST_INCR, 3'd2, 8'd1, 4'd1);
      g_mst[0].bfm.w_only(1010, 0, 0);
      sb_write(32'h0000_0E00, `AXI_BURST_INCR, 3'd2, 8'd1, 1000, 0, 0);
      sb_write(32'h1000_0F00, `AXI_BURST_INCR, 3'd2, 8'd1, 1010, 0, 0);
      // Wait for B2 to assert: two BVALIDs pending at once
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
  // S14: R grant lock held while RREADY stalls (no deadlock)
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
      // slave0 fast (long burst first), slave1 slow (waits mid-way)
      cfg_r_delay[0] = 8'd1;
      cfg_r_delay[1] = 8'd5;
      g_mst[0].bfm.ar_only(32'h0000_0080, `AXI_BURST_INCR, 3'd2, 8'd7, 4'd0, 1100, 0, 0, 0, 0);
      g_mst[0].bfm.ar_only(32'h1000_0090, `AXI_BURST_INCR, 3'd2, 8'd3, 4'd1, 1110, 0, 0, 0, 0);
      // Stall for 100 cycles after 1 beat, then collect both bursts
      g_mst[0].bfm.collect_all(1, 100);
      cfg_r_delay[0] = 8'd2;
      cfg_r_delay[1] = 8'd2;
      sb_compare();
      $display("[%0t] PASS S14", $time);
    end
  endtask

  //==========================================================================
  // S15: single master with multiple concurrent IDs
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
      // Reads use different concurrent IDs (id3 / id7)
      g_mst[0].bfm.ar_only(32'h0000_0040, `AXI_BURST_INCR, 3'd2, 8'd1, 4'd3, 1200, 0, 0, 0, 0);
      g_mst[0].bfm.ar_only(32'h1000_0050, `AXI_BURST_INCR, 3'd2, 8'd1, 4'd7, 1210, 0, 0, 0, 0);
      g_mst[0].bfm.collect_all(0, 0);
      chk(g_mst[0].bfm.ar_done_cnt == 2, "S15 both reads done");
      sb_compare();
      $display("[%0t] PASS S15", $time);
    end
  endtask

  //==========================================================================
  // Main flow
  //==========================================================================
  initial begin
    integer tseed;
    $display("=== AXI4 Interconnect TB ===");
    $display("POLICY=%0d RESP_POLICY=%0d", `AXI_ARB_POLICY, `AXI_RESP_POLICY);
    // Slave default config
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
