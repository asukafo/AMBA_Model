`include "axi_defs.svh"
//------------------------------------------------------------------------------
// axi_interconnect - AXI4 full-crossbar interconnect (N_MASTER masters x M_SLAVE slaves)
//
// Structure:
//   - combinational address decode: {addr & ADDR_MASK[s]} == ADDR_BASE[s]
//     selects slave s; if no slave matches, route to the internal DECERR responder
//   - independent per-slave AW/AR arbiters + channel muxes (ID widening point: {tag, id})
//   - one W-ownership FIFO per slave: push the granted master's tag on the
//     AW handshake, pop on the WLAST handshake; the W mux is driven by head
//   - independent per-master B/R response arbiters + muxes (ID narrowing point: strip the tag)
//   - one internal DECERR responder per master
//
// Core correctness rules (must understand before changing any handshake logic):
//   1. Each master has at most one in-flight write data stream: w_pending[m]
//      is set on any AW handshake (including to DECERR) and cleared on the
//      WLAST handshake of that W stream. Both AW arbitration requests and
//      AWREADY are gated by !w_pending[m]. This guarantees a master's W
//      beats can only flow to one destination (the W-ownership FIFO head or
//      the DECERR drain), preventing the data corruption where m0 issues
//      AW->slave0 then AW->slave1 and its W beats get routed to both slaves.
//   2. Arbitration request masking: req = valid && !w_pending && sel, so a
//      grant is never held by a master with an unfinished W stream
//      (head-of-line blocking).
//   3. Grant locking: AW/AR lock until the address-beat handshake; B locks
//      until BVALID&&BREADY; R locks until the RLAST-beat handshake (so two
//      sources never interleave beat-by-beat on one master's R port).
//
//   4. WREADY is shown only to the master at the W-ownership FIFO head (or
//      in the DECERR drain).
//   5. ID widening/narrowing happens only at the m_aw/m_ar outputs and the
//      s_b/s_r outputs; DECERR uses master-width IDs throughout and never
//      enters the slave muxes.
// Tool constraints (iverilog 12, discovered the hard way):
//   - all module-boundary ports are packed vectors (per-channel fields
//
//     flattened together): iverilog silently produces X when driving an
//     unpacked-array output port in any way (assign / instance connection /
//     always block); unpacked arrays are internal-only
//   - port field access uses [i*W +: W] part-selects (verified with both
//     variable and constant indices)
//------------------------------------------------------------------------------
module axi_interconnect #(
  parameter int N_MASTER       = `AXI_N_MASTER,
  parameter int M_SLAVE        = `AXI_M_SLAVE,
  parameter int ADDR_WIDTH     = `AXI_ADDR_W,
  parameter int DATA_WIDTH     = `AXI_DATA_W,
  parameter int ID_WIDTH       = `AXI_ID_W,
  parameter int POLICY         = `AXI_ARB_POLICY,     // AW/AR arbitration
  parameter int RESP_POLICY    = `AXI_RESP_POLICY,    // B/R arbitration
  parameter int W_FIFO_DEPTH   = `AXI_W_FIFO_DEPTH,   // >= N_MASTER
  parameter int REG_SLICE     = `AXI_REG_SLICE,     // 0=combinational 1=register-sliced
  parameter int DECERR_Q_DEPTH = `AXI_DECERR_Q,
  // Address map: one ADDR_WIDTH-bit segment per slave; slave s's segment is
  // [s*ADDR_WIDTH +: ADDR_WIDTH] (1D packed - iverilog crashes on element
  // selects of 2D packed parameters and does not support unpacked-array parameters)
  parameter [(M_SLAVE*ADDR_WIDTH)-1:0] ADDR_BASE = '0,
  parameter [(M_SLAVE*ADDR_WIDTH)-1:0] ADDR_MASK = '0
) (
  input  wire                 clk,
  input  wire                 rstn,
  // ---- Master-side AW (interconnect acts as slave) ----
  input  wire [N_MASTER-1:0]                    s_awvalid,
  input  wire [N_MASTER*ID_WIDTH-1:0]           s_awid,
  input  wire [N_MASTER*ADDR_WIDTH-1:0]         s_awaddr,
  input  wire [N_MASTER*8-1:0]                  s_awlen,
  input  wire [N_MASTER*3-1:0]                  s_awsize,
  input  wire [N_MASTER*2-1:0]                  s_awburst,
  input  wire [N_MASTER*2-1:0]                  s_awlock,
  input  wire [N_MASTER*4-1:0]                  s_awcache,
  input  wire [N_MASTER*3-1:0]                  s_awprot,
  input  wire [N_MASTER*4-1:0]                  s_awqos,
  input  wire [N_MASTER*4-1:0]                  s_awregion,
  output reg  [N_MASTER-1:0]                    s_awready,
  // ---- Master-side W ----
  input  wire [N_MASTER-1:0]                    s_wvalid,
  input  wire [N_MASTER*DATA_WIDTH-1:0]         s_wdata,
  input  wire [N_MASTER*DATA_WIDTH/8-1:0]       s_wstrb,
  input  wire [N_MASTER-1:0]                    s_wlast,
  output reg  [N_MASTER-1:0]                    s_wready,
  // ---- Master-side B ----
  output reg  [N_MASTER-1:0]                    s_bvalid,
  output reg  [N_MASTER*ID_WIDTH-1:0]           s_bid,
  output reg  [N_MASTER*2-1:0]                  s_bresp,
  input  wire [N_MASTER-1:0]                    s_bready,
  // ---- Master-side AR ----
  input  wire [N_MASTER-1:0]                    s_arvalid,
  input  wire [N_MASTER*ID_WIDTH-1:0]           s_arid,
  input  wire [N_MASTER*ADDR_WIDTH-1:0]         s_araddr,
  input  wire [N_MASTER*8-1:0]                  s_arlen,
  input  wire [N_MASTER*3-1:0]                  s_arsize,
  input  wire [N_MASTER*2-1:0]                  s_arburst,
  input  wire [N_MASTER*2-1:0]                  s_arlock,
  input  wire [N_MASTER*4-1:0]                  s_arcache,
  input  wire [N_MASTER*3-1:0]                  s_arprot,
  input  wire [N_MASTER*4-1:0]                  s_arqos,
  input  wire [N_MASTER*4-1:0]                  s_arregion,
  output reg  [N_MASTER-1:0]                    s_arready,
  // ---- Master-side R ----
  output reg  [N_MASTER-1:0]                    s_rvalid,
  output reg  [N_MASTER*ID_WIDTH-1:0]           s_rid,
  output reg  [N_MASTER*DATA_WIDTH-1:0]         s_rdata,
  output reg  [N_MASTER*2-1:0]                  s_rresp,
  output reg  [N_MASTER-1:0]                    s_rlast,
  input  wire [N_MASTER-1:0]                    s_rready,
  // ---- Slave-side AW (interconnect acts as master; IDs already widened) ----
  output wire [M_SLAVE-1:0]                     m_awvalid,
  output wire [M_SLAVE*SLV_ID_W-1:0]            m_awid,
  output wire [M_SLAVE*ADDR_WIDTH-1:0]          m_awaddr,
  output wire [M_SLAVE*8-1:0]                   m_awlen,
  output wire [M_SLAVE*3-1:0]                   m_awsize,
  output wire [M_SLAVE*2-1:0]                   m_awburst,
  output wire [M_SLAVE*2-1:0]                   m_awlock,
  output wire [M_SLAVE*4-1:0]                   m_awcache,
  output wire [M_SLAVE*3-1:0]                   m_awprot,
  output wire [M_SLAVE*4-1:0]                   m_awqos,
  output wire [M_SLAVE*4-1:0]                   m_awregion,
  input  wire [M_SLAVE-1:0]                     m_awready,
  // ---- Slave-side W ----
  output wire [M_SLAVE-1:0]                     m_wvalid,
  output wire [M_SLAVE*DATA_WIDTH-1:0]          m_wdata,
  output wire [M_SLAVE*DATA_WIDTH/8-1:0]        m_wstrb,
  output wire [M_SLAVE-1:0]                     m_wlast,
  input  wire [M_SLAVE-1:0]                     m_wready,
  // ---- Slave-side B ----
  input  wire [M_SLAVE-1:0]                     m_bvalid,
  input  wire [M_SLAVE*SLV_ID_W-1:0]            m_bid,
  input  wire [M_SLAVE*2-1:0]                   m_bresp,
  output wire [M_SLAVE-1:0]                     m_bready,
  // ---- Slave-side AR ----
  output wire [M_SLAVE-1:0]                     m_arvalid,
  output wire [M_SLAVE*SLV_ID_W-1:0]            m_arid,
  output wire [M_SLAVE*ADDR_WIDTH-1:0]          m_araddr,
  output wire [M_SLAVE*8-1:0]                   m_arlen,
  output wire [M_SLAVE*3-1:0]                   m_arsize,
  output wire [M_SLAVE*2-1:0]                   m_arburst,
  output wire [M_SLAVE*2-1:0]                   m_arlock,
  output wire [M_SLAVE*4-1:0]                   m_arcache,
  output wire [M_SLAVE*3-1:0]                   m_arprot,
  output wire [M_SLAVE*4-1:0]                   m_arqos,
  output wire [M_SLAVE*4-1:0]                   m_arregion,
  input  wire [M_SLAVE-1:0]                     m_arready,
  // ---- Slave-side R ----
  input  wire [M_SLAVE-1:0]                     m_rvalid,
  input  wire [M_SLAVE*SLV_ID_W-1:0]            m_rid,
  input  wire [M_SLAVE*DATA_WIDTH-1:0]          m_rdata,
  input  wire [M_SLAVE*2-1:0]                   m_rresp,
  input  wire [M_SLAVE-1:0]                     m_rlast,
  output wire [M_SLAVE-1:0]                     m_rready,
  // ---- Debug observability (for TB fairness/policy checks; flattened 1D: bit [s*N_MASTER + m]) ----
  output wire [M_SLAVE*N_MASTER-1:0]            dbg_aw_grant,
  output wire [M_SLAVE*N_MASTER-1:0]            dbg_ar_grant,
  output wire [M_SLAVE*N_MASTER-1:0]            dbg_aw_req,
  output wire [M_SLAVE*N_MASTER-1:0]            dbg_ar_req
);

  localparam TAG_W    = $clog2(N_MASTER);
  localparam SLV_ID_W = ID_WIDTH + TAG_W;

  // Slave-side channel payload bit layout (for register-slice packing;
  localparam AW_PLD_W  = SLV_ID_W + ADDR_WIDTH + 8 + 3 + 2 + 2 + 4 + 3 + 4 + 4;
  // matches the port field order). Field order (from LSB): region(4) qos(4)
  //                prot(3) cache(4) lock(2) burst(2) size(3) len(8) addr(32) id(5)
  localparam AW_REGION_LSB = 0;
  localparam AW_QOS_LSB    = AW_REGION_LSB + 4;
  localparam AW_PROT_LSB   = AW_QOS_LSB + 4;
  localparam AW_CACHE_LSB  = AW_PROT_LSB + 3;
  localparam AW_LOCK_LSB   = AW_CACHE_LSB + 4;
  localparam AW_BURST_LSB  = AW_LOCK_LSB + 2;
  localparam AW_SIZE_LSB   = AW_BURST_LSB + 2;
  localparam AW_LEN_LSB    = AW_SIZE_LSB + 3;
  localparam AW_ADDR_LSB   = AW_LEN_LSB + 8;
  localparam AW_ID_LSB     = AW_ADDR_LSB + ADDR_WIDTH;
  localparam W_PLD_W   = DATA_WIDTH + DATA_WIDTH/8 + 1;
  localparam W_LAST_LSB = 0;
  localparam W_STRB_LSB = 1;
  localparam W_DATA_LSB = 1 + DATA_WIDTH/8;
  localparam B_PLD_W   = SLV_ID_W + 2;
  localparam B_RESP_LSB = 0;
  localparam B_ID_LSB   = 2;
  localparam R_PLD_W   = SLV_ID_W + DATA_WIDTH + 2 + 1;
  localparam R_LAST_LSB = 0;
  localparam R_RESP_LSB = 1;
  localparam R_DATA_LSB = 3;
  localparam R_ID_LSB   = 3 + DATA_WIDTH;

  // Slave-side internal channels (mux logic decoupled from the ports;
  // routed to the ports through register slices or directly)
  reg  [M_SLAVE-1:0]     m_awvalid_i;
  wire [M_SLAVE-1:0]     m_awready_i;
  reg  [AW_PLD_W-1:0]    m_aw_pld_i [M_SLAVE];
  reg  [M_SLAVE-1:0]     m_wvalid_i;
  wire [M_SLAVE-1:0]     m_wready_i;
  reg  [W_PLD_W-1:0]     m_w_pld_i [M_SLAVE];
  wire [M_SLAVE-1:0]     m_bvalid_i;
  reg  [M_SLAVE-1:0]     m_bready_i;
  wire [B_PLD_W-1:0]     m_b_pld_i [M_SLAVE];
  reg  [M_SLAVE-1:0]     m_arvalid_i;
  wire [M_SLAVE-1:0]     m_arready_i;
  reg  [AW_PLD_W-1:0]    m_ar_pld_i [M_SLAVE];
  wire [M_SLAVE-1:0]     m_rvalid_i;
  reg  [M_SLAVE-1:0]     m_rready_i;
  wire [R_PLD_W-1:0]     m_r_pld_i [M_SLAVE];

  // Struct widths are fixed by the axi_defs.svh macros; parameters must match
  initial begin
    if (N_MASTER < 2)
      $fatal(1, "axi_interconnect: N_MASTER must be >= 2");
    if (N_MASTER != `AXI_N_MASTER || M_SLAVE != `AXI_M_SLAVE ||
        ADDR_WIDTH != `AXI_ADDR_W || DATA_WIDTH != `AXI_DATA_W ||
        ID_WIDTH != `AXI_ID_W)
      $fatal(1, "axi_interconnect: parameters must match axi_defs.svh macros (use -D)");
  end

  function automatic [TAG_W-1:0] idx_of;
    input [N_MASTER-1:0] v;
    reg [TAG_W-1:0] r;
    integer i;
    begin
    r = '0;
    // Note: cannot use the i[TAG_W-1:0] part-select (iverilog 12 elab bug);
    // implicit truncating assignment does the job
    for (i = 0; i < N_MASTER; i = i + 1) if (v[i]) r = i;
    idx_of = r;
    end
  endfunction

  //============================================================================
  // Address decode (combinational)
  //============================================================================
  reg [M_SLAVE-1:0] aw_sel [N_MASTER];   // onehot slave select
  reg [M_SLAVE-1:0] ar_sel [N_MASTER];
  reg               aw_decerr [N_MASTER]; // no slave matches
  reg               ar_decerr [N_MASTER];

  // indices; read later with variable indices)
  wire [ADDR_WIDTH-1:0] addr_base_i [M_SLAVE];
  wire [ADDR_WIDTH-1:0] addr_mask_i [M_SLAVE];
  for (genvar s = 0; s < M_SLAVE; s++) begin : g_cfg
    assign addr_base_i[s] = ADDR_BASE[s*ADDR_WIDTH +: ADDR_WIDTH];
    assign addr_mask_i[s] = ADDR_MASK[s*ADDR_WIDTH +: ADDR_WIDTH];
  end

  always_comb begin
    for (int m = 0; m < N_MASTER; m++) begin
      for (int s = 0; s < M_SLAVE; s++) begin
        aw_sel[m][s] = ((s_awaddr[m*ADDR_WIDTH +: ADDR_WIDTH] & addr_mask_i[s])
                        == addr_base_i[s]);
        ar_sel[m][s] = ((s_araddr[m*ADDR_WIDTH +: ADDR_WIDTH] & addr_mask_i[s])
                        == addr_base_i[s]);
      end
      aw_decerr[m] = (aw_sel[m] == {M_SLAVE{1'b0}});
      ar_decerr[m] = (ar_sel[m] == {M_SLAVE{1'b0}});
    end
  end

  //============================================================================
  // AW path: per-slave arbitration + mux (ID widening point)
  //============================================================================
  reg  [N_MASTER-1:0] req_aw   [M_SLAVE];   // driven by always_comb
  wire [N_MASTER-1:0] grant_aw [M_SLAVE];   // arbiter instance output
  reg                 aw_ack   [M_SLAVE];   // AW handshake beat of that slave
  reg  [TAG_W-1:0]    aw_gidx  [M_SLAVE];   // granted master number

  // stream do not take part in arbitration
  always_comb begin
    for (int s = 0; s < M_SLAVE; s++) begin
      for (int m = 0; m < N_MASTER; m++) begin
        req_aw[s][m] = s_awvalid[m] && !w_pending[m] && aw_sel[m][s];
      end
    end
  end

  for (genvar s = 0; s < M_SLAVE; s++) begin : g_aw
    axi_arbiter #(
      .N      (N_MASTER),
      .POLICY (POLICY)
    ) arb (
      .clk   (clk),
      .rstn  (rstn),
      .req   (req_aw[s]),
      .ack   (aw_ack[s]),
      .grant (grant_aw[s])
    );
  end

  always_comb begin
    for (int s = 0; s < M_SLAVE; s++) begin
      aw_gidx[s] = idx_of(grant_aw[s]);
      // Handshake: the granted master and the slave are both valid && ready
      aw_ack[s] = |grant_aw[s] && m_awready_i[s] && s_awvalid[aw_gidx[s]];
      m_awvalid_i[s] = |grant_aw[s] && s_awvalid[aw_gidx[s]];
      // Channel payload packing (bit layout per AW_*_LSB); ID widened to {tag, id}
      m_aw_pld_i[s] = {aw_gidx[s], s_awid[aw_gidx[s]*ID_WIDTH +: ID_WIDTH],
                       s_awaddr[aw_gidx[s]*ADDR_WIDTH +: ADDR_WIDTH],
                       s_awlen[aw_gidx[s]*8 +: 8],
                       s_awsize[aw_gidx[s]*3 +: 3],
                       s_awburst[aw_gidx[s]*2 +: 2],
                       s_awlock[aw_gidx[s]*2 +: 2],
                       s_awcache[aw_gidx[s]*4 +: 4],
                       s_awprot[aw_gidx[s]*3 +: 3],
                       s_awqos[aw_gidx[s]*4 +: 4],
                       s_awregion[aw_gidx[s]*4 +: 4]};
    end
  end

  //============================================================================
  // AR path: per-slave arbitration + mux (ID widening point); reads may be arbitrarily outstanding
  //============================================================================
  reg  [N_MASTER-1:0] req_ar   [M_SLAVE];
  wire [N_MASTER-1:0] grant_ar [M_SLAVE];
  reg                 ar_ack   [M_SLAVE];
  reg  [TAG_W-1:0]    ar_gidx  [M_SLAVE];

  always_comb begin
    for (int s = 0; s < M_SLAVE; s++) begin
      for (int m = 0; m < N_MASTER; m++) begin
        req_ar[s][m] = s_arvalid[m] && ar_sel[m][s];
      end
    end
  end

  for (genvar s = 0; s < M_SLAVE; s++) begin : g_ar
    axi_arbiter #(
      .N      (N_MASTER),
      .POLICY (POLICY)
    ) arb (
      .clk   (clk),
      .rstn  (rstn),
      .req   (req_ar[s]),
      .ack   (ar_ack[s]),
      .grant (grant_ar[s])
    );
  end

  always_comb begin
    for (int s = 0; s < M_SLAVE; s++) begin
      ar_gidx[s] = idx_of(grant_ar[s]);
      ar_ack[s] = |grant_ar[s] && m_arready_i[s] && s_arvalid[ar_gidx[s]];
      m_arvalid_i[s] = |grant_ar[s] && s_arvalid[ar_gidx[s]];
      m_ar_pld_i[s] = {ar_gidx[s], s_arid[ar_gidx[s]*ID_WIDTH +: ID_WIDTH],
                       s_araddr[ar_gidx[s]*ADDR_WIDTH +: ADDR_WIDTH],
                       s_arlen[ar_gidx[s]*8 +: 8],
                       s_arsize[ar_gidx[s]*3 +: 3],
                       s_arburst[ar_gidx[s]*2 +: 2],
                       s_arlock[ar_gidx[s]*2 +: 2],
                       s_arcache[ar_gidx[s]*4 +: 4],
                       s_arprot[ar_gidx[s]*3 +: 3],
                       s_arqos[ar_gidx[s]*4 +: 4],
                       s_arregion[ar_gidx[s]*4 +: 4]};
    end
  end

  //============================================================================
  // W path: per-slave ownership FIFO + mux
  //============================================================================
  reg              push_w   [M_SLAVE];
  reg              pop_w    [M_SLAVE];
  wire [TAG_W-1:0] w_head   [M_SLAVE];   // fifo instance output
  wire             wf_empty [M_SLAVE];
  wire             wf_full  [M_SLAVE];

  for (genvar s = 0; s < M_SLAVE; s++) begin : g_w
    axi_owner_fifo #(
      .DEPTH (W_FIFO_DEPTH),
      .TAG_W (TAG_W)
    ) wfifo (
      .clk   (clk),
      .rstn  (rstn),
      .push  (push_w[s]),
      .din   (aw_gidx[s]),
      .pop   (pop_w[s]),
      .head  (w_head[s]),
      .empty (wf_empty[s]),
      .full  (wf_full[s]),
      .count ()
    );
  end

  reg w_slv_done [N_MASTER];   // a master's W stream completed WLAST on the slave side

  always_comb begin
    for (int m = 0; m < N_MASTER; m++)
      w_slv_done[m] = 1'b0;
    for (int s = 0; s < M_SLAVE; s++) begin
      push_w[s] = aw_ack[s];   // record the granted master on the AW handshake
      m_wvalid_i[s] = !wf_empty[s] && s_wvalid[w_head[s]];
      m_w_pld_i[s] = {s_wdata[w_head[s]*DATA_WIDTH +: DATA_WIDTH],
                      s_wstrb[w_head[s]*DATA_WIDTH/8 +: DATA_WIDTH/8],
                      s_wlast[w_head[s]]};
      pop_w[s] = m_wvalid_i[s] && m_wready_i[s] && m_w_pld_i[s][W_LAST_LSB];
      if (pop_w[s]) w_slv_done[w_head[s]] = 1'b1;
    end
  end

  // WREADY is shown only to the current owner (FIFO head or DECERR drain)
  always_comb begin
    for (int m = 0; m < N_MASTER; m++) begin
      s_wready[m] = de_wready[m];
      for (int s = 0; s < M_SLAVE; s++) begin
        if (!wf_empty[s] && (w_head[s] == m))
          s_wready[m] = m_wready_i[s];
      end
    end
  end

  // both DECERR and a slave at the same time
  reg de_owns [N_MASTER];
  always_comb begin
    for (int m = 0; m < N_MASTER; m++) begin
      de_owns[m] = de_wready[m];
      for (int s = 0; s < M_SLAVE; s++) begin
        if (de_owns[m] && !wf_empty[s] && (w_head[s] == m))
          $error("axi_interconnect: W ownership conflict on master %0d", m);
      end
    end
  end

  //============================================================================
  // At most one in-flight write stream per master (core rule 1)
  //============================================================================
  reg w_pending [N_MASTER];
  reg aw_hs     [N_MASTER];

  always_comb begin
    for (int m = 0; m < N_MASTER; m++) begin
      // AWREADY merge: DECERR destination and slave grant are mutually exclusive
      s_awready[m] = 1'b0;
      if (aw_decerr[m] && !w_pending[m])
        s_awready[m] = de_awready[m];
      else begin
        for (int s = 0; s < M_SLAVE; s++)
          if (grant_aw[s][m]) s_awready[m] = m_awready_i[s];
      end
      // ARREADY merge
      s_arready[m] = 1'b0;
      if (ar_decerr[m])
        s_arready[m] = de_arready[m];
      else begin
        for (int s = 0; s < M_SLAVE; s++)
          if (grant_ar[s][m]) s_arready[m] = m_arready_i[s];
      end
      aw_hs[m] = s_awvalid[m] && s_awready[m];
    end
  end

  always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      for (int m = 0; m < N_MASTER; m++)
        w_pending[m] <= 1'b0;
    end else begin
      for (int m = 0; m < N_MASTER; m++) begin
        // Setting requires !w_pending (awready already gates it), clearing
        // requires w_pending; the same master can never set and clear in the same cycle
        if (aw_hs[m])               w_pending[m] <= 1'b1;
        else if (de_wdone[m])       w_pending[m] <= 1'b0;
        else if (w_slv_done[m])     w_pending[m] <= 1'b0;
      end
    end
  end

  //============================================================================
  // DECERR responders (one per master)
  //============================================================================
  reg  de_awvalid  [N_MASTER];   // driven by always_comb
  wire de_awready  [N_MASTER];   // decerr instance output
  wire de_wready   [N_MASTER];
  wire de_wdone    [N_MASTER];
  wire de_bvalid   [N_MASTER];
  wire [`AXI_ID_W-1:0] de_bid [N_MASTER];
  wire [1:0]       de_bresp [N_MASTER];
  reg  de_bready   [N_MASTER];   // driven by always_comb
  reg  de_arvalid  [N_MASTER];
  wire de_arready  [N_MASTER];
  wire de_rvalid   [N_MASTER];
  wire [`AXI_ID_W-1:0] de_rid [N_MASTER];
  wire [1:0]       de_rresp [N_MASTER];
  reg  de_rready   [N_MASTER];

  always_comb begin
    for (int m = 0; m < N_MASTER; m++) begin
      de_awvalid[m] = s_awvalid[m] && aw_decerr[m] && !w_pending[m];
      de_arvalid[m] = s_arvalid[m] && ar_decerr[m];
    end
  end

  for (genvar m = 0; m < N_MASTER; m++) begin : g_decerr
    axi_decerr #(
      .AR_Q_DEPTH (DECERR_Q_DEPTH)
    ) dec (
      .clk          (clk),
      .rstn         (rstn),
      .awvalid      (de_awvalid[m]),
      .aw_id        (s_awid[m*ID_WIDTH +: ID_WIDTH]),
      .awready      (de_awready[m]),
      .wvalid       (s_wvalid[m]),
      .w_last       (s_wlast[m]),
      .wready       (de_wready[m]),
      .w_drain_done (de_wdone[m]),
      .bvalid       (de_bvalid[m]),
      .b_id         (de_bid[m]),
      .b_resp       (de_bresp[m]),
      .bready       (de_bready[m]),
      .arvalid      (de_arvalid[m]),
      .ar_id        (s_arid[m*ID_WIDTH +: ID_WIDTH]),
      .arready      (de_arready[m]),
      .rvalid       (de_rvalid[m]),
      .r_id         (de_rid[m]),
      .r_resp       (de_rresp[m]),
      .rready       (de_rready[m])
    );
  end

  //============================================================================
  // B path: per-master response arbitration + mux (ID narrowing point)
  // under fixed priority)
  //============================================================================
  reg  [M_SLAVE:0] req_b   [N_MASTER];
  wire [M_SLAVE:0] grant_b [N_MASTER];
  reg              b_ack   [N_MASTER];
  // Chained part-selects are unsupported (iverilog); use intermediate variables.
  // Note: each block must use its own variable - a single variable driven by
  // multiple always_comb blocks forms a zero-delay combinational loop at t=0
  reg [SLV_ID_W-1:0] bid_req_c, bid_mux_c, rid_req_c, rid_mux_c;

  // Requests: the top TAG_W bits of the slave's BID are the master tag
  always_comb begin
    for (int m = 0; m < N_MASTER; m++) begin
      req_b[m][0] = de_bvalid[m];
      for (int s = 0; s < M_SLAVE; s++) begin
        bid_req_c = m_b_pld_i[s][B_ID_LSB +: SLV_ID_W];
        req_b[m][s+1] = m_bvalid_i[s] && (bid_req_c[ID_WIDTH +: TAG_W] == m);
      end
    end
  end

  for (genvar m = 0; m < N_MASTER; m++) begin : g_b
    axi_arbiter #(
      .N      (M_SLAVE + 1),
      .POLICY (RESP_POLICY)
    ) arb (
      .clk   (clk),
      .rstn  (rstn),
      .req   (req_b[m]),
      .ack   (b_ack[m]),
      .grant (grant_b[m])
    );
  end

  always_comb begin
    for (int s = 0; s < M_SLAVE; s++)
      m_bready_i[s] = 1'b0;
    for (int m = 0; m < N_MASTER; m++) begin
      de_bready[m] = 1'b0;
      // BVALID likewise follows the granted source's actual bvalid (the grant
      s_bvalid[m] = 1'b0;
      s_bid[m*ID_WIDTH +: ID_WIDTH] = '0;
      s_bresp[m*2 +: 2] = '0;
      b_ack[m]    = 1'b0;
      if (grant_b[m][0]) begin
  // lock only constrains source switching)
        s_bvalid[m] = de_bvalid[m];
        s_bid[m*ID_WIDTH +: ID_WIDTH] = de_bid[m];
        s_bresp[m*2 +: 2]  = de_bresp[m];
        b_ack[m]    = de_bvalid[m] && s_bready[m];
        de_bready[m] = s_bready[m];
      end else begin
        for (int s = 0; s < M_SLAVE; s++) begin
          if (grant_b[m][s+1]) begin
            bid_mux_c = m_b_pld_i[s][B_ID_LSB +: SLV_ID_W];
            s_bvalid[m] = m_bvalid_i[s];
            s_bid[m*ID_WIDTH +: ID_WIDTH] = bid_mux_c[ID_WIDTH-1:0];
            s_bresp[m*2 +: 2]  = m_b_pld_i[s][B_RESP_LSB +: 2];
            b_ack[m]    = m_bvalid_i[s] && s_bready[m];
            m_bready_i[s] = s_bready[m];
          end
        end
      end
    end
  end

  //============================================================================
        // DECERR source: already master-width ID, pass through
  //============================================================================
  reg  [M_SLAVE:0] req_r   [N_MASTER];
  wire [M_SLAVE:0] grant_r [N_MASTER];
  reg              r_ack   [N_MASTER];

  always_comb begin
    for (int m = 0; m < N_MASTER; m++) begin
      req_r[m][0] = de_rvalid[m];
      for (int s = 0; s < M_SLAVE; s++) begin
        rid_req_c = m_r_pld_i[s][R_ID_LSB +: SLV_ID_W];
        req_r[m][s+1] = m_rvalid_i[s] && (rid_req_c[ID_WIDTH +: TAG_W] == m);
      end
    end
  end

  for (genvar m = 0; m < N_MASTER; m++) begin : g_r
    axi_arbiter #(
      .N      (M_SLAVE + 1),
      .POLICY (RESP_POLICY)
    ) arb (
      .clk   (clk),
      .rstn  (rstn),
      .req   (req_r[m]),
      .ack   (r_ack[m]),
      .grant (grant_r[m])
    );
  end

  always_comb begin
    for (int s = 0; s < M_SLAVE; s++)
      m_rready_i[s] = 1'b0;
    for (int m = 0; m < N_MASTER; m++) begin
      de_rready[m] = 1'b0;
      // R path: per-master response arbitration + mux (ID narrowing point);
      // grant locks until the RLAST handshake
      s_rvalid[m] = 1'b0;
      s_rid[m*ID_WIDTH +: ID_WIDTH] = '0;
      s_rdata[m*DATA_WIDTH +: DATA_WIDTH] = '0;
      s_rresp[m*2 +: 2] = '0;
      s_rlast[m] = 1'b0;
      r_ack[m]    = 1'b0;
      if (grant_r[m][0]) begin
        // DECERR source: single beat, last is always 1
        s_rvalid[m] = de_rvalid[m];
        s_rid[m*ID_WIDTH +: ID_WIDTH] = de_rid[m];
        s_rresp[m*2 +: 2]  = de_rresp[m];
        s_rlast[m]  = 1'b1;
        r_ack[m]    = de_rvalid[m] && s_rready[m];
        de_rready[m] = s_rready[m];
      end else begin
        for (int s = 0; s < M_SLAVE; s++) begin
          if (grant_r[m][s+1]) begin
            rid_mux_c = m_r_pld_i[s][R_ID_LSB +: SLV_ID_W];
            s_rvalid[m] = m_rvalid_i[s];
            s_rid[m*ID_WIDTH +: ID_WIDTH] = rid_mux_c[ID_WIDTH-1:0];
            s_rdata[m*DATA_WIDTH +: DATA_WIDTH] = m_r_pld_i[s][R_DATA_LSB +: DATA_WIDTH];
            s_rresp[m*2 +: 2]  = m_r_pld_i[s][R_RESP_LSB +: 2];
            s_rlast[m]  = m_r_pld_i[s][R_LAST_LSB];
  // RVALID must follow the granted source's actual rvalid - a slave may
            r_ack[m]    = m_rvalid_i[s] && s_rready[m] && m_r_pld_i[s][R_LAST_LSB];
            m_rready_i[s] = s_rready[m];
          end
        end
      end
    end
  end

  //============================================================================
  // legally drop RVALID between burst beats (e.g. with delays); the grant
  // lock only constrains source switching and must not force valid high
  //============================================================================
  for (genvar s = 0; s < M_SLAVE; s++) begin : g_slv_conn
    if (REG_SLICE) begin : g_sl
      wire [AW_PLD_W-1:0] aw_dout;
      wire [W_PLD_W-1:0]  w_dout;
      wire [B_PLD_W-1:0]  b_din;
      wire [AW_PLD_W-1:0] ar_dout;
      wire [R_PLD_W-1:0]  r_din;
            // Lock until the RLAST-beat handshake
      axi_reg_slice #(.W(AW_PLD_W)) u_aw (
        .clk (clk), .rstn (rstn),
        .din_valid (m_awvalid_i[s]),
        .din       (m_aw_pld_i[s]),
        .din_ready (m_awready_i[s]),
        .dout_valid (m_awvalid[s]),
        .dout       (aw_dout),
        .dout_ready (m_awready[s])
      );
      assign m_awid[s*SLV_ID_W +: SLV_ID_W] = aw_dout[AW_ID_LSB +: SLV_ID_W];
      assign m_awaddr[s*ADDR_WIDTH +: ADDR_WIDTH] = aw_dout[AW_ADDR_LSB +: ADDR_WIDTH];
      assign m_awlen[s*8 +: 8]   = aw_dout[AW_LEN_LSB +: 8];
      assign m_awsize[s*3 +: 3]  = aw_dout[AW_SIZE_LSB +: 3];
      assign m_awburst[s*2 +: 2] = aw_dout[AW_BURST_LSB +: 2];
      assign m_awlock[s*2 +: 2]  = aw_dout[AW_LOCK_LSB +: 2];
      assign m_awcache[s*4 +: 4] = aw_dout[AW_CACHE_LSB +: 4];
      assign m_awprot[s*3 +: 3]  = aw_dout[AW_PROT_LSB +: 3];
      assign m_awqos[s*4 +: 4]   = aw_dout[AW_QOS_LSB +: 4];
      assign m_awregion[s*4 +: 4] = aw_dout[AW_REGION_LSB +: 4];
  // Slave-side channel connections: optional register slices
      axi_reg_slice #(.W(W_PLD_W)) u_w (
        .clk (clk), .rstn (rstn),
        .din_valid (m_wvalid_i[s]),
        .din       (m_w_pld_i[s]),
        .din_ready (m_wready_i[s]),
        .dout_valid (m_wvalid[s]),
        .dout       (w_dout),
        .dout_ready (m_wready[s])
      );
      assign m_wdata[s*DATA_WIDTH +: DATA_WIDTH] = w_dout[W_DATA_LSB +: DATA_WIDTH];
      assign m_wstrb[s*DATA_WIDTH/8 +: DATA_WIDTH/8] = w_dout[W_STRB_LSB +: DATA_WIDTH/8];
      assign m_wlast[s] = w_dout[W_LAST_LSB];
  // (REG_SLICE=1 registers each channel, breaking combinational paths) or
      assign b_din = {m_bid[s*SLV_ID_W +: SLV_ID_W], m_bresp[s*2 +: 2]};
      axi_reg_slice #(.W(B_PLD_W)) u_b (
        .clk (clk), .rstn (rstn),
        .din_valid (m_bvalid[s]),
        .din       (b_din),
        .din_ready (m_bready[s]),
        .dout_valid (m_bvalid_i[s]),
        .dout       (m_b_pld_i[s]),
        .dout_ready (m_bready_i[s])
      );
  // direct combinational connections (REG_SLICE=0). Register slices are
      axi_reg_slice #(.W(AW_PLD_W)) u_ar (
        .clk (clk), .rstn (rstn),
        .din_valid (m_arvalid_i[s]),
        .din       (m_ar_pld_i[s]),
        .din_ready (m_arready_i[s]),
        .dout_valid (m_arvalid[s]),
        .dout       (ar_dout),
        .dout_ready (m_arready[s])
      );
      assign m_arid[s*SLV_ID_W +: SLV_ID_W] = ar_dout[AW_ID_LSB +: SLV_ID_W];
      assign m_araddr[s*ADDR_WIDTH +: ADDR_WIDTH] = ar_dout[AW_ADDR_LSB +: ADDR_WIDTH];
      assign m_arlen[s*8 +: 8]   = ar_dout[AW_LEN_LSB +: 8];
      assign m_arsize[s*3 +: 3]  = ar_dout[AW_SIZE_LSB +: 3];
      assign m_arburst[s*2 +: 2] = ar_dout[AW_BURST_LSB +: 2];
      assign m_arlock[s*2 +: 2]  = ar_dout[AW_LOCK_LSB +: 2];
      assign m_arcache[s*4 +: 4] = ar_dout[AW_CACHE_LSB +: 4];
      assign m_arprot[s*3 +: 3]  = ar_dout[AW_PROT_LSB +: 3];
      assign m_arqos[s*4 +: 4]   = ar_dout[AW_QOS_LSB +: 4];
      assign m_arregion[s*4 +: 4] = ar_dout[AW_REGION_LSB +: 4];
  // protocol-transparent and add one cycle of latency only.
      assign r_din = {m_rid[s*SLV_ID_W +: SLV_ID_W],
                      m_rdata[s*DATA_WIDTH +: DATA_WIDTH],
                      m_rresp[s*2 +: 2], m_rlast[s]};
      axi_reg_slice #(.W(R_PLD_W)) u_r (
        .clk (clk), .rstn (rstn),
        .din_valid (m_rvalid[s]),
        .din       (r_din),
        .din_ready (m_rready[s]),
        .dout_valid (m_rvalid_i[s]),
        .dout       (m_r_pld_i[s]),
        .dout_ready (m_rready_i[s])
      );
    end else begin : g_direct
      // AW
      assign m_awvalid[s] = m_awvalid_i[s];
      assign m_awready_i[s] = m_awready[s];
      assign m_awid[s*SLV_ID_W +: SLV_ID_W] = m_aw_pld_i[s][AW_ID_LSB +: SLV_ID_W];
      assign m_awaddr[s*ADDR_WIDTH +: ADDR_WIDTH] = m_aw_pld_i[s][AW_ADDR_LSB +: ADDR_WIDTH];
      assign m_awlen[s*8 +: 8]   = m_aw_pld_i[s][AW_LEN_LSB +: 8];
      assign m_awsize[s*3 +: 3]  = m_aw_pld_i[s][AW_SIZE_LSB +: 3];
      assign m_awburst[s*2 +: 2] = m_aw_pld_i[s][AW_BURST_LSB +: 2];
      assign m_awlock[s*2 +: 2]  = m_aw_pld_i[s][AW_LOCK_LSB +: 2];
      assign m_awcache[s*4 +: 4] = m_aw_pld_i[s][AW_CACHE_LSB +: 4];
      assign m_awprot[s*3 +: 3]  = m_aw_pld_i[s][AW_PROT_LSB +: 3];
      assign m_awqos[s*4 +: 4]   = m_aw_pld_i[s][AW_QOS_LSB +: 4];
      assign m_awregion[s*4 +: 4] = m_aw_pld_i[s][AW_REGION_LSB +: 4];
      // W
      assign m_wvalid[s] = m_wvalid_i[s];
      assign m_wready_i[s] = m_wready[s];
      assign m_wdata[s*DATA_WIDTH +: DATA_WIDTH] = m_w_pld_i[s][W_DATA_LSB +: DATA_WIDTH];
      assign m_wstrb[s*DATA_WIDTH/8 +: DATA_WIDTH/8] = m_w_pld_i[s][W_STRB_LSB +: DATA_WIDTH/8];
      assign m_wlast[s] = m_w_pld_i[s][W_LAST_LSB];
      // B
      assign m_bvalid_i[s] = m_bvalid[s];
      assign m_b_pld_i[s] = {m_bid[s*SLV_ID_W +: SLV_ID_W], m_bresp[s*2 +: 2]};
      assign m_bready[s] = m_bready_i[s];
      // AR
      assign m_arvalid[s] = m_arvalid_i[s];
      assign m_arready_i[s] = m_arready[s];
      assign m_arid[s*SLV_ID_W +: SLV_ID_W] = m_ar_pld_i[s][AW_ID_LSB +: SLV_ID_W];
      assign m_araddr[s*ADDR_WIDTH +: ADDR_WIDTH] = m_ar_pld_i[s][AW_ADDR_LSB +: ADDR_WIDTH];
      assign m_arlen[s*8 +: 8]   = m_ar_pld_i[s][AW_LEN_LSB +: 8];
      assign m_arsize[s*3 +: 3]  = m_ar_pld_i[s][AW_SIZE_LSB +: 3];
      assign m_arburst[s*2 +: 2] = m_ar_pld_i[s][AW_BURST_LSB +: 2];
      assign m_arlock[s*2 +: 2]  = m_ar_pld_i[s][AW_LOCK_LSB +: 2];
      assign m_arcache[s*4 +: 4] = m_ar_pld_i[s][AW_CACHE_LSB +: 4];
      assign m_arprot[s*3 +: 3]  = m_ar_pld_i[s][AW_PROT_LSB +: 3];
      assign m_arqos[s*4 +: 4]   = m_ar_pld_i[s][AW_QOS_LSB +: 4];
      assign m_arregion[s*4 +: 4] = m_ar_pld_i[s][AW_REGION_LSB +: 4];
      // R
      assign m_rvalid_i[s] = m_rvalid[s];
      assign m_r_pld_i[s] = {m_rid[s*SLV_ID_W +: SLV_ID_W],
                             m_rdata[s*DATA_WIDTH +: DATA_WIDTH],
                             m_rresp[s*2 +: 2], m_rlast[s]};
      assign m_rready[s] = m_rready_i[s];
    end
  end

  //============================================================================
      // AW (output direction: interconnect -> slave)
  //============================================================================
  for (genvar s = 0; s < M_SLAVE; s++) begin : g_dbg
    assign dbg_aw_grant[s*N_MASTER +: N_MASTER] = grant_aw[s];
    assign dbg_ar_grant[s*N_MASTER +: N_MASTER] = grant_ar[s];
    assign dbg_aw_req[s*N_MASTER +: N_MASTER]   = req_aw[s];
    assign dbg_ar_req[s*N_MASTER +: N_MASTER]   = req_ar[s];
  end

endmodule
