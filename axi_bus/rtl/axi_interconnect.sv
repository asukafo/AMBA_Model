`include "axi_defs.svh"
//------------------------------------------------------------------------------
// axi_interconnect — AXI4 全交叉开关互联（N_MASTER 主 × M_SLAVE 从）
//
// 结构：
//   - 组合地址译码：{addr & ADDR_MASK[s]} == ADDR_BASE[s] 选中 slave s，
//     全不命中则路由到内部 DECERR 响应器
//   - 每 slave 独立 AW/AR 仲裁器 + 通道 mux（ID 加宽点：{tag, id}）
//   - 每 slave 一个 W 归属 FIFO：AW 握手 push 被授权 master tag，
//     WLAST 握手 pop，W mux 由 head 驱动
//   - 每 master 独立 B/R 响应仲裁器 + mux（ID 剥窄点：去掉 tag）
//   - 内部 DECERR 响应器每 master 一份
//
// 核心正确性规则（改动任何握手逻辑前必须理解）：
//   1. 每 master 同时至多一条未完成的写数据流：w_pending[m] 在任意 AW
//      握手（含 DECERR 目的）时置 1，在对应 W 流 WLAST 握手时清 0。
//      AW 仲裁请求与 AWREADY 都以 !w_pending[m] 门控。这保证一个 master
//      的 W 拍只可能流向唯一目的地（W 归属 FIFO 头部或 DECERR drain），
//      杜绝"m0 先发 AW->slave0 再发 AW->slave1，W 拍被同时路由到两个
//      slave"的数据破坏。
//   2. 仲裁请求屏蔽：req = valid && !w_pending && sel，避免授权被 W 流
//      未完成的 master 占住（head-of-line blocking）。
//   3. 授权锁定：AW/AR 锁到地址拍握手；B 锁到 BVALID&&BREADY；
//      R 锁到 RLAST 拍握手（同一 master 的 R 端口不出现两源按拍交错）。
//   4. WREADY 只发给 W 归属 FIFO 头部（或 DECERR drain 中）的 master。
//   5. ID 加宽/剥窄只在 m_aw/m_ar 输出与 s_b/s_r 输出处发生；
//      DECERR 全程 master 宽度 ID，不进 slave mux。
//
// 工具约束（iverilog 12，实测踩坑）：
//   - 模块边界端口一律用 packed 向量（每通道字段拍平拼接）：
//     iverilog 对 unpacked 数组输出端口的任何驱动（assign/实例连接/
//     always 块）都会静默产出 X；unpacked 数组仅限模块内部使用
//   - 端口字段访问用 [i*W +: W] 部分选择（变量/常量索引均验证可用）
//   - 禁止 struct 数组元素成员访问（elab 崩溃）、2D packed 变量索引写
//   - 参数的部分选择要求常量索引（genvar 展开 OK）
//
// 限制（v1）：
//   - N_MASTER >= 2；所有 master/slave 同宽 DATA/ID（无宽度转换）
//   - 无 USER 信号、无稀疏连接矩阵、无寄存器片；组合直连
//   - 改规模/宽度请用 -D 宏（axi_defs.svh），模块参数默认值必须与宏一致
//     （initial 里有检查兜底）
//------------------------------------------------------------------------------
module axi_interconnect #(
  parameter int N_MASTER       = `AXI_N_MASTER,
  parameter int M_SLAVE        = `AXI_M_SLAVE,
  parameter int ADDR_WIDTH     = `AXI_ADDR_W,
  parameter int DATA_WIDTH     = `AXI_DATA_W,
  parameter int ID_WIDTH       = `AXI_ID_W,
  parameter int POLICY         = `AXI_ARB_POLICY,     // AW/AR 仲裁
  parameter int RESP_POLICY    = `AXI_RESP_POLICY,    // B/R 仲裁
  parameter int W_FIFO_DEPTH   = `AXI_W_FIFO_DEPTH,   // >= N_MASTER
  parameter int DECERR_Q_DEPTH = `AXI_DECERR_Q,
  // 地址映射：每 slave 一段 ADDR_WIDTH 位，slave s 的段为
  // [s*ADDR_WIDTH +: ADDR_WIDTH]（1D packed——iverilog 对 2D packed
  // 参数的元素选择会崩溃，unpacked 数组参数不支持）
  parameter [(M_SLAVE*ADDR_WIDTH)-1:0] ADDR_BASE = '0,
  parameter [(M_SLAVE*ADDR_WIDTH)-1:0] ADDR_MASK = '0
) (
  input  logic                 clk,
  input  logic                 rstn,
  // ---- master 侧 AW（互联作为 slave 收）----
  input  logic [N_MASTER-1:0]                    s_awvalid,
  input  logic [N_MASTER*ID_WIDTH-1:0]           s_awid,
  input  logic [N_MASTER*ADDR_WIDTH-1:0]         s_awaddr,
  input  logic [N_MASTER*8-1:0]                  s_awlen,
  input  logic [N_MASTER*3-1:0]                  s_awsize,
  input  logic [N_MASTER*2-1:0]                  s_awburst,
  input  logic [N_MASTER*2-1:0]                  s_awlock,
  input  logic [N_MASTER*4-1:0]                  s_awcache,
  input  logic [N_MASTER*3-1:0]                  s_awprot,
  input  logic [N_MASTER*4-1:0]                  s_awqos,
  input  logic [N_MASTER*4-1:0]                  s_awregion,
  output logic [N_MASTER-1:0]                    s_awready,
  // ---- master 侧 W ----
  input  logic [N_MASTER-1:0]                    s_wvalid,
  input  logic [N_MASTER*DATA_WIDTH-1:0]         s_wdata,
  input  logic [N_MASTER*DATA_WIDTH/8-1:0]       s_wstrb,
  input  logic [N_MASTER-1:0]                    s_wlast,
  output logic [N_MASTER-1:0]                    s_wready,
  // ---- master 侧 B ----
  output logic [N_MASTER-1:0]                    s_bvalid,
  output logic [N_MASTER*ID_WIDTH-1:0]           s_bid,
  output logic [N_MASTER*2-1:0]                  s_bresp,
  input  logic [N_MASTER-1:0]                    s_bready,
  // ---- master 侧 AR ----
  input  logic [N_MASTER-1:0]                    s_arvalid,
  input  logic [N_MASTER*ID_WIDTH-1:0]           s_arid,
  input  logic [N_MASTER*ADDR_WIDTH-1:0]         s_araddr,
  input  logic [N_MASTER*8-1:0]                  s_arlen,
  input  logic [N_MASTER*3-1:0]                  s_arsize,
  input  logic [N_MASTER*2-1:0]                  s_arburst,
  input  logic [N_MASTER*2-1:0]                  s_arlock,
  input  logic [N_MASTER*4-1:0]                  s_arcache,
  input  logic [N_MASTER*3-1:0]                  s_arprot,
  input  logic [N_MASTER*4-1:0]                  s_arqos,
  input  logic [N_MASTER*4-1:0]                  s_arregion,
  output logic [N_MASTER-1:0]                    s_arready,
  // ---- master 侧 R ----
  output logic [N_MASTER-1:0]                    s_rvalid,
  output logic [N_MASTER*ID_WIDTH-1:0]           s_rid,
  output logic [N_MASTER*DATA_WIDTH-1:0]         s_rdata,
  output logic [N_MASTER*2-1:0]                  s_rresp,
  output logic [N_MASTER-1:0]                    s_rlast,
  input  logic [N_MASTER-1:0]                    s_rready,
  // ---- slave 侧 AW（互联作为 master 发，ID 已加宽）----
  output logic [M_SLAVE-1:0]                     m_awvalid,
  output logic [M_SLAVE*SLV_ID_W-1:0]            m_awid,
  output logic [M_SLAVE*ADDR_WIDTH-1:0]          m_awaddr,
  output logic [M_SLAVE*8-1:0]                   m_awlen,
  output logic [M_SLAVE*3-1:0]                   m_awsize,
  output logic [M_SLAVE*2-1:0]                   m_awburst,
  output logic [M_SLAVE*2-1:0]                   m_awlock,
  output logic [M_SLAVE*4-1:0]                   m_awcache,
  output logic [M_SLAVE*3-1:0]                   m_awprot,
  output logic [M_SLAVE*4-1:0]                   m_awqos,
  output logic [M_SLAVE*4-1:0]                   m_awregion,
  input  logic [M_SLAVE-1:0]                     m_awready,
  // ---- slave 侧 W ----
  output logic [M_SLAVE-1:0]                     m_wvalid,
  output logic [M_SLAVE*DATA_WIDTH-1:0]          m_wdata,
  output logic [M_SLAVE*DATA_WIDTH/8-1:0]        m_wstrb,
  output logic [M_SLAVE-1:0]                     m_wlast,
  input  logic [M_SLAVE-1:0]                     m_wready,
  // ---- slave 侧 B ----
  input  logic [M_SLAVE-1:0]                     m_bvalid,
  input  logic [M_SLAVE*SLV_ID_W-1:0]            m_bid,
  input  logic [M_SLAVE*2-1:0]                   m_bresp,
  output logic [M_SLAVE-1:0]                     m_bready,
  // ---- slave 侧 AR ----
  output logic [M_SLAVE-1:0]                     m_arvalid,
  output logic [M_SLAVE*SLV_ID_W-1:0]            m_arid,
  output logic [M_SLAVE*ADDR_WIDTH-1:0]          m_araddr,
  output logic [M_SLAVE*8-1:0]                   m_arlen,
  output logic [M_SLAVE*3-1:0]                   m_arsize,
  output logic [M_SLAVE*2-1:0]                   m_arburst,
  output logic [M_SLAVE*2-1:0]                   m_arlock,
  output logic [M_SLAVE*4-1:0]                   m_arcache,
  output logic [M_SLAVE*3-1:0]                   m_arprot,
  output logic [M_SLAVE*4-1:0]                   m_arqos,
  output logic [M_SLAVE*4-1:0]                   m_arregion,
  input  logic [M_SLAVE-1:0]                     m_arready,
  // ---- slave 侧 R ----
  input  logic [M_SLAVE-1:0]                     m_rvalid,
  input  logic [M_SLAVE*SLV_ID_W-1:0]            m_rid,
  input  logic [M_SLAVE*DATA_WIDTH-1:0]          m_rdata,
  input  logic [M_SLAVE*2-1:0]                   m_rresp,
  input  logic [M_SLAVE-1:0]                     m_rlast,
  output logic [M_SLAVE-1:0]                     m_rready,
  // ---- 调试观测（TB 公平性/策略检查用，拍平 1D：位 [s*N_MASTER + m]）----
  output logic [M_SLAVE*N_MASTER-1:0]            dbg_aw_grant,
  output logic [M_SLAVE*N_MASTER-1:0]            dbg_ar_grant,
  output logic [M_SLAVE*N_MASTER-1:0]            dbg_aw_req,
  output logic [M_SLAVE*N_MASTER-1:0]            dbg_ar_req
);

  localparam TAG_W    = $clog2(N_MASTER);
  localparam SLV_ID_W = ID_WIDTH + TAG_W;

  // struct 宽度由 axi_defs.svh 宏决定，参数必须与之保持一致
  initial begin
    if (N_MASTER < 2)
      $fatal(1, "axi_interconnect: N_MASTER must be >= 2");
    if (N_MASTER != `AXI_N_MASTER || M_SLAVE != `AXI_M_SLAVE ||
        ADDR_WIDTH != `AXI_ADDR_W || DATA_WIDTH != `AXI_DATA_W ||
        ID_WIDTH != `AXI_ID_W)
      $fatal(1, "axi_interconnect: parameters must match axi_defs.svh macros (use -D)");
  end

  function automatic logic [TAG_W-1:0] idx_of(input logic [N_MASTER-1:0] v);
    logic [TAG_W-1:0] r;
    r = '0;
    // 注意：不能用 i[TAG_W-1:0] 部分选择（iverilog 12 elab bug），
    // 隐式截断赋值即可
    for (int i = 0; i < N_MASTER; i++) if (v[i]) r = i;
    return r;
  endfunction

  //============================================================================
  // 地址译码（组合）
  //============================================================================
  logic [M_SLAVE-1:0] aw_sel [N_MASTER];   // onehot 选中 slave
  logic [M_SLAVE-1:0] ar_sel [N_MASTER];
  logic               aw_decerr [N_MASTER]; // 全不命中
  logic               ar_decerr [N_MASTER];

  // 参数段拷贝成 memory 风格数组（generate 常量索引，供变量索引读取）
  logic [ADDR_WIDTH-1:0] addr_base_i [M_SLAVE];
  logic [ADDR_WIDTH-1:0] addr_mask_i [M_SLAVE];
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
  // AW 路径：每 slave 仲裁 + mux（ID 加宽点）
  //============================================================================
  logic [N_MASTER-1:0] req_aw   [M_SLAVE];
  logic [N_MASTER-1:0] grant_aw [M_SLAVE];
  logic                aw_ack   [M_SLAVE];   // 该 slave 的 AW 握手拍
  logic [TAG_W-1:0]    aw_gidx  [M_SLAVE];   // 被授权 master 编号

  // 仲裁请求：规则 1/2 —— 在途写流未完成的 master 不参与仲裁
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
      // 握手：被授权 master 与 slave 同时 valid && ready
      aw_ack[s] = |grant_aw[s] && m_awready[s] && s_awvalid[aw_gidx[s]];
      m_awvalid[s] = |grant_aw[s] && s_awvalid[aw_gidx[s]];
      // ID 加宽为 {tag, id}
      m_awid[s*SLV_ID_W +: SLV_ID_W] =
        {aw_gidx[s], s_awid[aw_gidx[s]*ID_WIDTH +: ID_WIDTH]};
      m_awaddr[s*ADDR_WIDTH +: ADDR_WIDTH] = s_awaddr[aw_gidx[s]*ADDR_WIDTH +: ADDR_WIDTH];
      m_awlen[s*8 +: 8]     = s_awlen[aw_gidx[s]*8 +: 8];
      m_awsize[s*3 +: 3]    = s_awsize[aw_gidx[s]*3 +: 3];
      m_awburst[s*2 +: 2]   = s_awburst[aw_gidx[s]*2 +: 2];
      m_awlock[s*2 +: 2]    = s_awlock[aw_gidx[s]*2 +: 2];
      m_awcache[s*4 +: 4]   = s_awcache[aw_gidx[s]*4 +: 4];
      m_awprot[s*3 +: 3]    = s_awprot[aw_gidx[s]*3 +: 3];
      m_awqos[s*4 +: 4]     = s_awqos[aw_gidx[s]*4 +: 4];
      m_awregion[s*4 +: 4]  = s_awregion[aw_gidx[s]*4 +: 4];
    end
  end

  //============================================================================
  // AR 路径：每 slave 仲裁 + mux（ID 加宽点），读可任意 outstanding
  //============================================================================
  logic [N_MASTER-1:0] req_ar   [M_SLAVE];
  logic [N_MASTER-1:0] grant_ar [M_SLAVE];
  logic                ar_ack   [M_SLAVE];
  logic [TAG_W-1:0]    ar_gidx  [M_SLAVE];

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
      ar_ack[s] = |grant_ar[s] && m_arready[s] && s_arvalid[ar_gidx[s]];
      m_arvalid[s] = |grant_ar[s] && s_arvalid[ar_gidx[s]];
      m_arid[s*SLV_ID_W +: SLV_ID_W] =
        {ar_gidx[s], s_arid[ar_gidx[s]*ID_WIDTH +: ID_WIDTH]};
      m_araddr[s*ADDR_WIDTH +: ADDR_WIDTH] = s_araddr[ar_gidx[s]*ADDR_WIDTH +: ADDR_WIDTH];
      m_arlen[s*8 +: 8]     = s_arlen[ar_gidx[s]*8 +: 8];
      m_arsize[s*3 +: 3]    = s_arsize[ar_gidx[s]*3 +: 3];
      m_arburst[s*2 +: 2]   = s_arburst[ar_gidx[s]*2 +: 2];
      m_arlock[s*2 +: 2]    = s_arlock[ar_gidx[s]*2 +: 2];
      m_arcache[s*4 +: 4]   = s_arcache[ar_gidx[s]*4 +: 4];
      m_arprot[s*3 +: 3]    = s_arprot[ar_gidx[s]*3 +: 3];
      m_arqos[s*4 +: 4]     = s_arqos[ar_gidx[s]*4 +: 4];
      m_arregion[s*4 +: 4]  = s_arregion[ar_gidx[s]*4 +: 4];
    end
  end

  //============================================================================
  // W 路径：每 slave 归属 FIFO + mux
  //============================================================================
  logic             push_w   [M_SLAVE];
  logic             pop_w    [M_SLAVE];
  logic [TAG_W-1:0] w_head   [M_SLAVE];
  logic             wf_empty [M_SLAVE];
  logic             wf_full  [M_SLAVE];

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

  logic w_slv_done [N_MASTER];   // 某 master 的 W 流在 slave 侧 WLAST 完成

  always_comb begin
    for (int m = 0; m < N_MASTER; m++)
      w_slv_done[m] = 1'b0;
    for (int s = 0; s < M_SLAVE; s++) begin
      push_w[s] = aw_ack[s];   // AW 握手时记录被授权 master
      m_wvalid[s] = !wf_empty[s] && s_wvalid[w_head[s]];
      m_wdata[s*DATA_WIDTH +: DATA_WIDTH] =
        s_wdata[w_head[s]*DATA_WIDTH +: DATA_WIDTH];
      m_wstrb[s*DATA_WIDTH/8 +: DATA_WIDTH/8] =
        s_wstrb[w_head[s]*DATA_WIDTH/8 +: DATA_WIDTH/8];
      m_wlast[s] = s_wlast[w_head[s]];
      pop_w[s] = m_wvalid[s] && m_wready[s] && m_wlast[s];
      if (pop_w[s]) w_slv_done[w_head[s]] = 1'b1;
    end
  end

  // WREADY 只发给当前归属者（FIFO head 或 DECERR drain 中）
  always_comb begin
    for (int m = 0; m < N_MASTER; m++) begin
      s_wready[m] = de_wready[m];
      for (int s = 0; s < M_SLAVE; s++) begin
        if (!wf_empty[s] && (w_head[s] == m))
          s_wready[m] = m_wready[s];
      end
    end
  end

  // 合法性断言：同一 master 不可能同时被 DECERR 与某 slave 拥有 W 流
  logic de_owns [N_MASTER];
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
  // 每 master 至多一条在途写流（核心规则 1）
  //============================================================================
  logic w_pending [N_MASTER];
  logic aw_hs     [N_MASTER];

  always_comb begin
    for (int m = 0; m < N_MASTER; m++) begin
      // AWREADY 合并：DECERR 目的与 slave 授权两者互斥
      s_awready[m] = 1'b0;
      if (aw_decerr[m] && !w_pending[m])
        s_awready[m] = de_awready[m];
      else begin
        for (int s = 0; s < M_SLAVE; s++)
          if (grant_aw[s][m]) s_awready[m] = m_awready[s];
      end
      // ARREADY 合并
      s_arready[m] = 1'b0;
      if (ar_decerr[m])
        s_arready[m] = de_arready[m];
      else begin
        for (int s = 0; s < M_SLAVE; s++)
          if (grant_ar[s][m]) s_arready[m] = m_arready[s];
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
        // 置位要求 !w_pending（awready 已门控），清零要求 w_pending，
        // 同一 master 不可能同拍既置又清
        if (aw_hs[m])               w_pending[m] <= 1'b1;
        else if (de_wdone[m])       w_pending[m] <= 1'b0;
        else if (w_slv_done[m])     w_pending[m] <= 1'b0;
      end
    end
  end

  //============================================================================
  // DECERR 响应器（每 master 一份）
  //============================================================================
  logic de_awvalid  [N_MASTER];
  logic de_awready  [N_MASTER];
  logic de_wready   [N_MASTER];
  logic de_wdone    [N_MASTER];
  logic de_bvalid   [N_MASTER];
  logic [`AXI_ID_W-1:0] de_bid [N_MASTER];
  logic [1:0]       de_bresp [N_MASTER];
  logic de_bready   [N_MASTER];
  logic de_arvalid  [N_MASTER];
  logic de_arready  [N_MASTER];
  logic de_rvalid   [N_MASTER];
  logic [`AXI_ID_W-1:0] de_rid [N_MASTER];
  logic [1:0]       de_rresp [N_MASTER];
  logic de_rready   [N_MASTER];

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
  // B 路径：每 master 响应仲裁 + mux（ID 剥窄点）
  // 源编号：0 = DECERR，s+1 = slave s（固定优先级下 DECERR 最高）
  //============================================================================
  logic [M_SLAVE:0] req_b   [N_MASTER];
  logic [M_SLAVE:0] grant_b [N_MASTER];
  logic             b_ack   [N_MASTER];
  // 链式部分选择不支持（iverilog），中间变量承接。
  // 注意：必须每块独立变量——同一变量被多个 always_comb 驱动会在
  // iverilog 下形成 t=0 零延迟组合环（仿真卡死）
  logic [SLV_ID_W-1:0] bid_req_c, bid_mux_c, rid_req_c, rid_mux_c;

  // 请求：slave 的 BID 最高 TAG_W 位即 master tag
  always_comb begin
    for (int m = 0; m < N_MASTER; m++) begin
      req_b[m][0] = de_bvalid[m];
      for (int s = 0; s < M_SLAVE; s++) begin
        bid_req_c = m_bid[s*SLV_ID_W +: SLV_ID_W];
        req_b[m][s+1] = m_bvalid[s] && (bid_req_c[ID_WIDTH +: TAG_W] == m);
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
      m_bready[s] = 1'b0;
    for (int m = 0; m < N_MASTER; m++) begin
      de_bready[m] = 1'b0;
      // BVALID 同样跟随被授权源的实际 bvalid（授权锁只约束换源）
      s_bvalid[m] = 1'b0;
      s_bid[m*ID_WIDTH +: ID_WIDTH] = '0;
      s_bresp[m*2 +: 2] = '0;
      b_ack[m]    = 1'b0;
      if (grant_b[m][0]) begin
        // DECERR 源：已是 master 宽度 ID，直通
        s_bvalid[m] = de_bvalid[m];
        s_bid[m*ID_WIDTH +: ID_WIDTH] = de_bid[m];
        s_bresp[m*2 +: 2]  = de_bresp[m];
        b_ack[m]    = de_bvalid[m] && s_bready[m];
        de_bready[m] = s_bready[m];
      end else begin
        for (int s = 0; s < M_SLAVE; s++) begin
          if (grant_b[m][s+1]) begin
            bid_mux_c = m_bid[s*SLV_ID_W +: SLV_ID_W];
            s_bvalid[m] = m_bvalid[s];
            s_bid[m*ID_WIDTH +: ID_WIDTH] = bid_mux_c[ID_WIDTH-1:0];
            s_bresp[m*2 +: 2]  = m_bresp[s*2 +: 2];
            b_ack[m]    = m_bvalid[s] && s_bready[m];
            m_bready[s] = s_bready[m];
          end
        end
      end
    end
  end

  //============================================================================
  // R 路径：每 master 响应仲裁 + mux（ID 剥窄点），授权锁到 RLAST 握手
  //============================================================================
  logic [M_SLAVE:0] req_r   [N_MASTER];
  logic [M_SLAVE:0] grant_r [N_MASTER];
  logic             r_ack   [N_MASTER];

  always_comb begin
    for (int m = 0; m < N_MASTER; m++) begin
      req_r[m][0] = de_rvalid[m];
      for (int s = 0; s < M_SLAVE; s++) begin
        rid_req_c = m_rid[s*SLV_ID_W +: SLV_ID_W];
        req_r[m][s+1] = m_rvalid[s] && (rid_req_c[ID_WIDTH +: TAG_W] == m);
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
      m_rready[s] = 1'b0;
    for (int m = 0; m < N_MASTER; m++) begin
      de_rready[m] = 1'b0;
      // RVALID 必须跟随被授权源的实际 rvalid——slave 允许在突发
      // 拍间拉低 RVALID（如加延迟），授权锁只约束"换源"，不能硬拉 valid
      s_rvalid[m] = 1'b0;
      s_rid[m*ID_WIDTH +: ID_WIDTH] = '0;
      s_rdata[m*DATA_WIDTH +: DATA_WIDTH] = '0;
      s_rresp[m*2 +: 2] = '0;
      s_rlast[m] = 1'b0;
      r_ack[m]    = 1'b0;
      if (grant_r[m][0]) begin
        // DECERR 源：单拍，last 恒 1
        s_rvalid[m] = de_rvalid[m];
        s_rid[m*ID_WIDTH +: ID_WIDTH] = de_rid[m];
        s_rresp[m*2 +: 2]  = de_rresp[m];
        s_rlast[m]  = 1'b1;
        r_ack[m]    = de_rvalid[m] && s_rready[m];
        de_rready[m] = s_rready[m];
      end else begin
        for (int s = 0; s < M_SLAVE; s++) begin
          if (grant_r[m][s+1]) begin
            rid_mux_c = m_rid[s*SLV_ID_W +: SLV_ID_W];
            s_rvalid[m] = m_rvalid[s];
            s_rid[m*ID_WIDTH +: ID_WIDTH] = rid_mux_c[ID_WIDTH-1:0];
            s_rdata[m*DATA_WIDTH +: DATA_WIDTH] = m_rdata[s*DATA_WIDTH +: DATA_WIDTH];
            s_rresp[m*2 +: 2]  = m_rresp[s*2 +: 2];
            s_rlast[m]  = m_rlast[s];
            // 锁到 RLAST 拍握手
            r_ack[m]    = m_rvalid[s] && s_rready[m] && m_rlast[s];
            m_rready[s] = s_rready[m];
          end
        end
      end
    end
  end

  //============================================================================
  // 调试输出（拍平 1D，位 [s*N_MASTER + m]）
  //============================================================================
  for (genvar s = 0; s < M_SLAVE; s++) begin : g_dbg
    assign dbg_aw_grant[s*N_MASTER +: N_MASTER] = grant_aw[s];
    assign dbg_ar_grant[s*N_MASTER +: N_MASTER] = grant_ar[s];
    assign dbg_aw_req[s*N_MASTER +: N_MASTER]   = req_aw[s];
    assign dbg_ar_req[s*N_MASTER +: N_MASTER]   = req_ar[s];
  end

endmodule
