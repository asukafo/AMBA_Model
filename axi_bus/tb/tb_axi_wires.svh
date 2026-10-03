//------------------------------------------------------------------------------
// tb_axi_wires.svh — tb_axi / tb_axi_rtl 共用：互联端口连线 + DUT 实例
//
// 地址映射（4KB 窗口内活动）：
//   slave0: base 0x0000_0000 mask 0xF000_0000
//   slave1: base 0x1000_0000 mask 0xF000_0000
//   DECERR: 0x8000_0000 区域（未命中）
//------------------------------------------------------------------------------

  // ---- 互联端口信号（packed 向量——iverilog 无法驱动 unpacked 数组端口）----
  logic [`AXI_N_MASTER-1:0]                    s_awvalid;
  logic [`AXI_N_MASTER*`AXI_ID_W-1:0]          s_awid;
  logic [`AXI_N_MASTER*`AXI_ADDR_W-1:0]        s_awaddr;
  logic [`AXI_N_MASTER*8-1:0]                  s_awlen;
  logic [`AXI_N_MASTER*3-1:0]                  s_awsize;
  logic [`AXI_N_MASTER*2-1:0]                  s_awburst;
  logic [`AXI_N_MASTER*2-1:0]                  s_awlock;
  logic [`AXI_N_MASTER*4-1:0]                  s_awcache;
  logic [`AXI_N_MASTER*3-1:0]                  s_awprot;
  logic [`AXI_N_MASTER*4-1:0]                  s_awqos;
  logic [`AXI_N_MASTER*4-1:0]                  s_awregion;
  logic [`AXI_N_MASTER-1:0]                    s_awready;
  logic [`AXI_N_MASTER-1:0]                    s_wvalid;
  logic [`AXI_N_MASTER*`AXI_DATA_W-1:0]        s_wdata;
  logic [`AXI_N_MASTER*`AXI_DATA_W/8-1:0]      s_wstrb;
  logic [`AXI_N_MASTER-1:0]                    s_wlast;
  logic [`AXI_N_MASTER-1:0]                    s_wready;
  logic [`AXI_N_MASTER-1:0]                    s_bvalid;
  logic [`AXI_N_MASTER*`AXI_ID_W-1:0]          s_bid;
  logic [`AXI_N_MASTER*2-1:0]                  s_bresp;
  logic [`AXI_N_MASTER-1:0]                    s_bready;
  logic [`AXI_N_MASTER-1:0]                    s_arvalid;
  logic [`AXI_N_MASTER*`AXI_ID_W-1:0]          s_arid;
  logic [`AXI_N_MASTER*`AXI_ADDR_W-1:0]        s_araddr;
  logic [`AXI_N_MASTER*8-1:0]                  s_arlen;
  logic [`AXI_N_MASTER*3-1:0]                  s_arsize;
  logic [`AXI_N_MASTER*2-1:0]                  s_arburst;
  logic [`AXI_N_MASTER*2-1:0]                  s_arlock;
  logic [`AXI_N_MASTER*4-1:0]                  s_arcache;
  logic [`AXI_N_MASTER*3-1:0]                  s_arprot;
  logic [`AXI_N_MASTER*4-1:0]                  s_arqos;
  logic [`AXI_N_MASTER*4-1:0]                  s_arregion;
  logic [`AXI_N_MASTER-1:0]                    s_arready;
  logic [`AXI_N_MASTER-1:0]                    s_rvalid;
  logic [`AXI_N_MASTER*`AXI_ID_W-1:0]          s_rid;
  logic [`AXI_N_MASTER*`AXI_DATA_W-1:0]        s_rdata;
  logic [`AXI_N_MASTER*2-1:0]                  s_rresp;
  logic [`AXI_N_MASTER-1:0]                    s_rlast;
  logic [`AXI_N_MASTER-1:0]                    s_rready;

  logic [`AXI_M_SLAVE-1:0]                     m_awvalid;
  logic [`AXI_M_SLAVE*`AXI_SLV_ID_W-1:0]       m_awid;
  logic [`AXI_M_SLAVE*`AXI_ADDR_W-1:0]         m_awaddr;
  logic [`AXI_M_SLAVE*8-1:0]                   m_awlen;
  logic [`AXI_M_SLAVE*3-1:0]                   m_awsize;
  logic [`AXI_M_SLAVE*2-1:0]                   m_awburst;
  logic [`AXI_M_SLAVE*2-1:0]                   m_awlock;
  logic [`AXI_M_SLAVE*4-1:0]                   m_awcache;
  logic [`AXI_M_SLAVE*3-1:0]                   m_awprot;
  logic [`AXI_M_SLAVE*4-1:0]                   m_awqos;
  logic [`AXI_M_SLAVE*4-1:0]                   m_awregion;
  logic [`AXI_M_SLAVE-1:0]                     m_awready;
  logic [`AXI_M_SLAVE-1:0]                     m_wvalid;
  logic [`AXI_M_SLAVE*`AXI_DATA_W-1:0]         m_wdata;
  logic [`AXI_M_SLAVE*`AXI_DATA_W/8-1:0]       m_wstrb;
  logic [`AXI_M_SLAVE-1:0]                     m_wlast;
  logic [`AXI_M_SLAVE-1:0]                     m_wready;
  logic [`AXI_M_SLAVE-1:0]                     m_bvalid;
  logic [`AXI_M_SLAVE*`AXI_SLV_ID_W-1:0]       m_bid;
  logic [`AXI_M_SLAVE*2-1:0]                   m_bresp;
  logic [`AXI_M_SLAVE-1:0]                     m_bready;
  logic [`AXI_M_SLAVE-1:0]                     m_arvalid;
  logic [`AXI_M_SLAVE*`AXI_SLV_ID_W-1:0]       m_arid;
  logic [`AXI_M_SLAVE*`AXI_ADDR_W-1:0]         m_araddr;
  logic [`AXI_M_SLAVE*8-1:0]                   m_arlen;
  logic [`AXI_M_SLAVE*3-1:0]                   m_arsize;
  logic [`AXI_M_SLAVE*2-1:0]                   m_arburst;
  logic [`AXI_M_SLAVE*2-1:0]                   m_arlock;
  logic [`AXI_M_SLAVE*4-1:0]                   m_arcache;
  logic [`AXI_M_SLAVE*3-1:0]                   m_arprot;
  logic [`AXI_M_SLAVE*4-1:0]                   m_arqos;
  logic [`AXI_M_SLAVE*4-1:0]                   m_arregion;
  logic [`AXI_M_SLAVE-1:0]                     m_arready;
  logic [`AXI_M_SLAVE-1:0]                     m_rvalid;
  logic [`AXI_M_SLAVE*`AXI_SLV_ID_W-1:0]       m_rid;
  logic [`AXI_M_SLAVE*`AXI_DATA_W-1:0]         m_rdata;
  logic [`AXI_M_SLAVE*2-1:0]                   m_rresp;
  logic [`AXI_M_SLAVE-1:0]                     m_rlast;
  logic [`AXI_M_SLAVE-1:0]                     m_rready;

  logic [`AXI_M_SLAVE*`AXI_N_MASTER-1:0]       dbg_aw_grant;
  logic [`AXI_M_SLAVE*`AXI_N_MASTER-1:0]       dbg_ar_grant;
  logic [`AXI_M_SLAVE*`AXI_N_MASTER-1:0]       dbg_aw_req;
  logic [`AXI_M_SLAVE*`AXI_N_MASTER-1:0]       dbg_ar_req;

  // ---- DUT ----
  axi_interconnect #(
    .ADDR_BASE ({32'h1000_0000, 32'h0000_0000}),
    .ADDR_MASK ({32'hF000_0000, 32'hF000_0000})
  ) dut (
    .clk (clk), .rstn (rstn),
    .s_awvalid (s_awvalid), .s_awid (s_awid), .s_awaddr (s_awaddr),
    .s_awlen (s_awlen), .s_awsize (s_awsize), .s_awburst (s_awburst),
    .s_awlock (s_awlock), .s_awcache (s_awcache), .s_awprot (s_awprot),
    .s_awqos (s_awqos), .s_awregion (s_awregion), .s_awready (s_awready),
    .s_wvalid (s_wvalid), .s_wdata (s_wdata), .s_wstrb (s_wstrb),
    .s_wlast (s_wlast), .s_wready (s_wready),
    .s_bvalid (s_bvalid), .s_bid (s_bid), .s_bresp (s_bresp), .s_bready (s_bready),
    .s_arvalid (s_arvalid), .s_arid (s_arid), .s_araddr (s_araddr),
    .s_arlen (s_arlen), .s_arsize (s_arsize), .s_arburst (s_arburst),
    .s_arlock (s_arlock), .s_arcache (s_arcache), .s_arprot (s_arprot),
    .s_arqos (s_arqos), .s_arregion (s_arregion), .s_arready (s_arready),
    .s_rvalid (s_rvalid), .s_rid (s_rid), .s_rdata (s_rdata),
    .s_rresp (s_rresp), .s_rlast (s_rlast), .s_rready (s_rready),
    .m_awvalid (m_awvalid), .m_awid (m_awid), .m_awaddr (m_awaddr),
    .m_awlen (m_awlen), .m_awsize (m_awsize), .m_awburst (m_awburst),
    .m_awlock (m_awlock), .m_awcache (m_awcache), .m_awprot (m_awprot),
    .m_awqos (m_awqos), .m_awregion (m_awregion), .m_awready (m_awready),
    .m_wvalid (m_wvalid), .m_wdata (m_wdata), .m_wstrb (m_wstrb),
    .m_wlast (m_wlast), .m_wready (m_wready),
    .m_bvalid (m_bvalid), .m_bid (m_bid), .m_bresp (m_bresp), .m_bready (m_bready),
    .m_arvalid (m_arvalid), .m_arid (m_arid), .m_araddr (m_araddr),
    .m_arlen (m_arlen), .m_arsize (m_arsize), .m_arburst (m_arburst),
    .m_arlock (m_arlock), .m_arcache (m_arcache), .m_arprot (m_arprot),
    .m_arqos (m_arqos), .m_arregion (m_arregion), .m_arready (m_arready),
    .m_rvalid (m_rvalid), .m_rid (m_rid), .m_rdata (m_rdata),
    .m_rresp (m_rresp), .m_rlast (m_rlast), .m_rready (m_rready),
    .dbg_aw_grant (dbg_aw_grant), .dbg_ar_grant (dbg_ar_grant),
    .dbg_aw_req (dbg_aw_req), .dbg_ar_req (dbg_ar_req)
  );
