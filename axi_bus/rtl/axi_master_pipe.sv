`include "axi_defs.svh"
//------------------------------------------------------------------------------
// axi_master_pipe - synthesizable pipelined multi-outstanding AXI4 master (reference design)
//
// Complements axi_master_cfg (single-transaction, blocking):
//   - executes a descriptor table (N_DESC entries, written by software
//     before start) sequentially
//   - read transactions: the next one is issued right after the AR handshake
//     without waiting for R (outstanding pipelining)
//   - write transactions: AW + W complete inline (one master's W streams are
//     naturally serialized, satisfying the interconnect's "one in-flight
//
//     write stream per master" rule); the next one is issued right after WLAST
//   - B/R responses are tracked by ID in a slot table (writes wait for B,
//
//     reads wait for RLAST); done asserts only when all are complete
// Deterministic data (easy to verify): write beat b = desc_wdata0 + b with
//
// full WSTRB; read checksum = XOR accumulation of all read-beat rdata.
//------------------------------------------------------------------------------
module axi_master_pipe #(
  parameter int MST_ID     = 0,
  parameter int N_DESC     = 8,
  parameter int N_SLOT     = 8,
  parameter int ADDR_WIDTH = `AXI_ADDR_W,
  parameter int DATA_WIDTH = `AXI_DATA_W,
  parameter int ID_WIDTH   = `AXI_ID_W
) (
  input  wire clk,
  input  wire rstn,
  // ---- Control / descriptor-table write (software side) ----
  input  wire start,           // start pulse: execute descriptors 0..cfg_ndesc-1 in order
  input  wire [DW:0] cfg_ndesc,// number of valid descriptors in this batch (1..N_DESC)
  input  wire desc_wr,         // descriptor write enable (use while busy=0)
  input  wire [$clog2(N_DESC)-1:0] desc_sel,
  input  wire desc_dir,        // 0=write 1=read
  input  wire [ADDR_WIDTH-1:0] desc_addr,
  input  wire [7:0]            desc_len,
  input  wire [2:0]            desc_size,
  input  wire [1:0]            desc_burst,
  input  wire [ID_WIDTH-1:0]   desc_id,
  input  wire [DATA_WIDTH-1:0] desc_wdata0,
  // ---- Status (software side) ----
  output wire busy,
  output wire done,            // all transactions complete (one-cycle DONE-state output)
  output wire resp_err,        // some response was not OKAY
  output wire [1:0]            last_err_resp,
  output wire [N_SLOT-1:0]     slot_done,   // slot completed (this batch)
  output wire [N_SLOT*2-1:0]   slot_resp,   // slot response code ([i*2 +: 2])
  output wire [DATA_WIDTH-1:0] rd_checksum, // read-data XOR accumulation
  // ---- AXI master ports (AW) ----
  output reg awvalid,
  output reg [ID_WIDTH-1:0]   awid,
  output reg [ADDR_WIDTH-1:0] awaddr,
  output reg [7:0]            awlen,
  output reg [2:0]            awsize,
  output reg [1:0]            awburst,
  input  wire awready,
  // ---- W ----
  output reg wvalid,
  output reg [DATA_WIDTH-1:0]   wdata,
  output reg [DATA_WIDTH/8-1:0] wstrb,
  output reg wlast,
  input  wire wready,
  // ---- B ----
  input  wire bvalid,
  input  wire [ID_WIDTH-1:0] bid,
  input  wire [1:0] bresp,
  output wire bready,
  // ---- AR ----
  output reg arvalid,
  output reg [ID_WIDTH-1:0]   arid,
  output reg [ADDR_WIDTH-1:0] araddr,
  output reg [7:0]            arlen,
  output reg [2:0]            arsize,
  output reg [1:0]            arburst,
  input  wire arready,
  // ---- R ----
  input  wire rvalid,
  input  wire [ID_WIDTH-1:0] rid,
  input  wire [DATA_WIDTH-1:0] rdata,
  input  wire [1:0] rresp,
  input  wire rlast,
  output wire rready
);

  localparam DW = $clog2(N_DESC);   // descriptor index width

  reg              dt_dir    [0:N_DESC-1];
  reg [ADDR_WIDTH-1:0] dt_addr  [0:N_DESC-1];
  reg [7:0]        dt_len    [0:N_DESC-1];
  reg [2:0]        dt_size   [0:N_DESC-1];
  reg [1:0]        dt_burst  [0:N_DESC-1];
  reg [ID_WIDTH-1:0] dt_id   [0:N_DESC-1];
  reg [DATA_WIDTH-1:0] dt_wdata0 [0:N_DESC-1];

  reg              sl_pend    [0:N_SLOT-1];
  reg              sl_is_wr   [0:N_SLOT-1];
  reg [ID_WIDTH-1:0] sl_id    [0:N_SLOT-1];
  reg [1:0]        sl_resp    [0:N_SLOT-1];
  integer          sl_cnt;              // pending slot count
  reg [N_SLOT-1:0] slot_done_q;
  reg [N_SLOT*2-1:0] slot_resp_q;
  reg              resp_err_q;
  reg [1:0]        last_err_resp_q;
  reg [DATA_WIDTH-1:0] chk_q;

  integer slot_free, slot_b, slot_r;
  reg    slot_b_hit, slot_r_hit;

  always_comb begin
    // Free slot
    slot_free = -1;
    begin
      integer fnd;
      fnd = 0;
      for (int i = 0; i < N_SLOT; i++)
        if (!sl_pend[i] && !fnd) begin
          slot_free = i;
          fnd = 1;
        end
    end
    // B-response match slot
    slot_b = -1;
    begin
      integer fnd;
      fnd = 0;
      for (int i = 0; i < N_SLOT; i++)
        if (sl_pend[i] && sl_is_wr[i] && (sl_id[i] === bid) && !fnd) begin
          slot_b = i;
          fnd = 1;
        end
    end
    slot_b_hit = (slot_b >= 0);
    // R-response match slot
    slot_r = -1;
    begin
      integer fnd;
      fnd = 0;
      for (int i = 0; i < N_SLOT; i++)
        if (sl_pend[i] && !sl_is_wr[i] && (sl_id[i] === rid) && !fnd) begin
          slot_r = i;
          fnd = 1;
        end
    end
    slot_r_hit = (slot_r >= 0);
  end

  //==========================================================================
  // Issue FSM: walks the descriptors sequentially (shares the single
  // always_ff with response handling to avoid multiple drivers)
  //==========================================================================
  localparam I_IDLE = 3'd0, I_NEXT = 3'd1, I_WAW = 3'd2,
             I_WDATA = 3'd3, I_RAR = 3'd4, I_WAIT = 3'd5, I_DONE = 3'd6;
  reg [2:0]      i_state;
  // Width must represent N_DESC itself (sentinel value; compare desc_idx == N_DESC)
  reg [DW:0]     desc_idx;
  reg [7:0]      w_beat;

  always_comb begin
    awvalid = (i_state == I_WAW);
    awid    = dt_id[desc_idx];
    awaddr  = dt_addr[desc_idx];
    awlen   = dt_len[desc_idx];
    awsize  = dt_size[desc_idx];
    awburst = dt_burst[desc_idx];
    wvalid  = (i_state == I_WDATA);
    wdata   = dt_wdata0[desc_idx] + w_beat;
  // Issue FSM: walks the descriptors sequentially (shares the single
    wstrb   = axi_strb_for_size(dt_size[desc_idx],
               axi_beat_addr(dt_addr[desc_idx], dt_burst[desc_idx],
                             dt_size[desc_idx], dt_len[desc_idx], w_beat));
    wlast   = (w_beat == dt_len[desc_idx]);
    arvalid = (i_state == I_RAR);
    arid    = dt_id[desc_idx];
    araddr  = dt_addr[desc_idx];
    arlen   = dt_len[desc_idx];
    arsize  = dt_size[desc_idx];
    arburst = dt_burst[desc_idx];
  end

  assign bready = 1'b1;
  assign rready = 1'b1;

  // ---- Combinational lookup ----
  reg issue_hs;
  always_comb begin
    issue_hs = ((i_state == I_WAW) && awvalid && awready) ||
               ((i_state == I_RAR) && arvalid && arready);
  end

  always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      i_state  <= I_IDLE;
      desc_idx <= '0;
      w_beat   <= 8'd0;
      for (int i = 0; i < N_DESC; i++) begin
        dt_dir[i]    <= 1'b0;
        dt_addr[i]   <= '0;
        dt_len[i]    <= 8'd0;
        dt_size[i]   <= 3'd0;
        dt_burst[i]  <= 2'd0;
        dt_id[i]     <= '0;
        dt_wdata0[i] <= '0;
      end
      for (int i = 0; i < N_SLOT; i++) begin
        sl_pend[i]  <= 1'b0;
        sl_is_wr[i] <= 1'b0;
        sl_id[i]    <= '0;
        sl_resp[i]  <= 2'b00;
      end
      sl_cnt         <= 0;
      slot_done_q    <= '0;
      slot_resp_q    <= '0;
      resp_err_q     <= 1'b0;
      last_err_resp_q <= 2'b00;
      chk_q          <= '0;
    end else begin
  // Whether a slot was newly allocated this cycle (response sl_cnt updates
      if (desc_wr) begin
        dt_dir[desc_sel]    <= desc_dir;
        dt_addr[desc_sel]   <= desc_addr;
        dt_len[desc_sel]    <= desc_len;
        dt_size[desc_sel]   <= desc_size;
        dt_burst[desc_sel]  <= desc_burst;
        dt_id[desc_sel]     <= desc_id;
        dt_wdata0[desc_sel] <= desc_wdata0;
      end
  // must cancel it out)
      if (start) begin
        resp_err_q      <= 1'b0;
        last_err_resp_q <= 2'b00;
        slot_done_q     <= '0;
        slot_resp_q     <= '0;
        chk_q           <= '0;
      end
    // Narrow transfers: WSTRB is set only within the size-byte window
      case (i_state)
        I_IDLE: if (start) begin
                  desc_idx <= '0;
                  i_state  <= I_NEXT;
                end
        I_NEXT: begin
          if (desc_idx == cfg_ndesc)
            i_state <= I_WAIT;
            // Allocate a B-pending slot
            // Allocate an R-pending slot
          else if (slot_free >= 0) begin
            if (dt_dir[desc_idx] == 1'b0)
              i_state <= I_WAW;
            else
              i_state <= I_RAR;
          end
        end
        I_WAW: if (awvalid && awready) begin
             // ---- Descriptor write ----
                 sl_pend[slot_free]  <= 1'b1;
                 sl_is_wr[slot_free] <= 1'b1;
                 sl_id[slot_free]    <= dt_id[desc_idx];
                 sl_resp[slot_free]  <= 2'b00;
                 w_beat   <= 8'd0;
                 i_state  <= I_WDATA;
               end
        I_WDATA: if (wvalid && wready) begin
                   if (wlast) begin
                     desc_idx <= desc_idx + 1'b1;
                     i_state  <= I_NEXT;
                   end else begin
                     w_beat <= w_beat + 8'd1;
                   end
                 end
        I_RAR: if (arvalid && arready) begin
             // ---- Batch-start reset ----
                 sl_pend[slot_free]  <= 1'b1;
                 sl_is_wr[slot_free] <= 1'b0;
                 sl_id[slot_free]    <= dt_id[desc_idx];
                 sl_resp[slot_free]  <= 2'b00;
                 desc_idx <= desc_idx + 1'b1;
                 i_state  <= I_NEXT;
               end
        I_WAIT: begin
                  // DONE state lasts one cycle, then back to IDLE
                  if (sl_cnt == 0) i_state <= I_DONE;
                end
        I_DONE: i_state <= I_IDLE;
        default: i_state <= I_IDLE;
      endcase
  // (aligned to the beat address)
      if (bvalid && bready) begin
        if (!slot_b_hit) begin
          $error("axi_master_pipe: BID %0h no pending slot", bid);
        end else begin
          sl_pend[slot_b]  <= 1'b0;
          sl_resp[slot_b]  <= bresp;
          slot_resp_q[slot_b*2 +: 2] <= bresp;
          slot_done_q[slot_b] <= 1'b1;
          if (bresp != `AXI_RESP_OKAY) begin
            resp_err_q      <= 1'b1;
            last_err_resp_q <= bresp;
          end
        end
      end
  // Issue FSM: walks the descriptors sequentially
      if (rvalid && rready) begin
        if (!slot_r_hit) begin
          $error("axi_master_pipe: RID %0h no pending slot", rid);
        end else begin
          chk_q <= chk_q ^ rdata;
          if (rlast) begin
            sl_pend[slot_r]  <= 1'b0;
            sl_resp[slot_r]  <= rresp;
            slot_resp_q[slot_r*2 +: 2] <= rresp;
            slot_done_q[slot_r] <= 1'b1;
            if (rresp != `AXI_RESP_OKAY) begin
              resp_err_q      <= 1'b1;
              last_err_resp_q <= rresp;
            end
          end
        end
      end
  // Response handling: B (single beat) and R (burst) in parallel
      begin
        integer d;
        d = 0;
        if (issue_hs) d = d + 1;
        if (bvalid && bready && slot_b_hit) d = d - 1;
        if (rvalid && rready && slot_r_hit && rlast) d = d - 1;
        sl_cnt <= sl_cnt + d;
      end
    end
  end

  assign done        = (i_state == I_DONE);
  assign busy        = (i_state != I_IDLE);
  assign resp_err    = resp_err_q;
  assign last_err_resp = last_err_resp_q;
  assign slot_done   = slot_done_q;
  assign slot_resp   = slot_resp_q;
  assign rd_checksum = chk_q;

endmodule
