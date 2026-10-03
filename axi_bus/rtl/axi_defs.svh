//------------------------------------------------------------------------------
// axi_defs.svh - global definitions for the AXI4 full-crossbar interconnect
//
// Widths/scale are `ifndef-guarded macros, overridable at compile time via
// -D (e.g. -DAXI_N_MASTER=4). axi_interconnect's module parameters default
// from these macros - change scale/width via -D, not by overriding module
// parameters alone (axi_interconnect has an initial-block check as a
// safety net).
//
// Tool constraints: this project runs on iverilog 12, whose SystemVerilog
// support has serious defects (member access on struct array elements
// crashes elaboration, struct queues / associative arrays are not
// supported). RTL and TB therefore use a conservative subset: flat ports +
// unpacked arrays (memory style) + generate unrolling. Do not introduce
// structs / queues / associative arrays in this project.
//
// ID scheme: slave-side ID = {master tag (TAG_W), master id}; B/R responses
// are routed back to the master by the tag, which is then stripped - no
// transaction tracking tables.
//------------------------------------------------------------------------------
`ifndef AXI_DEFS_SVH
`define AXI_DEFS_SVH

// ---- Width/scale macros (all -D overridable) ----
`ifndef AXI_ADDR_W
`define AXI_ADDR_W 32
`endif
`ifndef AXI_DATA_W
`define AXI_DATA_W 32
`endif
`ifndef AXI_ID_W
`define AXI_ID_W 4
`endif
`ifndef AXI_N_MASTER
`define AXI_N_MASTER 2
`endif
`ifndef AXI_M_SLAVE
`define AXI_M_SLAVE 2
`endif
`ifndef AXI_TAG_W
`define AXI_TAG_W ($clog2(`AXI_N_MASTER))
`endif
// Slave-side ID width = master ID + master tag
`ifndef AXI_SLV_ID_W
`define AXI_SLV_ID_W (`AXI_ID_W + `AXI_TAG_W)
`endif
`ifndef AXI_W_FIFO_DEPTH
`define AXI_W_FIFO_DEPTH 2
`endif
`ifndef AXI_DECERR_Q
`define AXI_DECERR_Q 4
`endif
// Arbitration policy: 0 = fixed priority (lowest index wins), 1 = round-robin
`ifndef AXI_ARB_POLICY
`define AXI_ARB_POLICY 1
`endif
// B/R response-channel arbitration policy, same encoding
`ifndef AXI_RESP_POLICY
`define AXI_RESP_POLICY 1
`endif
// Slave-side output register slices (0 = combinational; 1 = one register
// stage on all 5 channels, breaking the interconnect's internal
// combinational paths; +1 cycle of latency per transaction)
`ifndef AXI_REG_SLICE
`define AXI_REG_SLICE 0
`endif

// ---- Protocol constants ----
`define AXI_RESP_OKAY   2'b00
`define AXI_RESP_EXOKAY 2'b01
`define AXI_RESP_SLVERR 2'b10
`define AXI_RESP_DECERR 2'b11
`define AXI_BURST_FIXED 2'b00
`define AXI_BURST_INCR  2'b01
`define AXI_BURST_WRAP  2'b10

//------------------------------------------------------------------------------
// AXI burst beat-address computation (shared by BFM / scoreboard /
// slave models). WRAP wraps at the boundary aligned to the total burst
// size (len+1)*2**size; the start address must be size-aligned.
//------------------------------------------------------------------------------
function automatic [`AXI_ADDR_W-1:0] axi_beat_addr;
  input [`AXI_ADDR_W-1:0] start;
  input [1:0]             burst;
  input [2:0]             size;
  input [7:0]             len;
  input integer           beat;
  reg [`AXI_ADDR_W-1:0] num_bytes, total_bytes, addr;
  num_bytes   = (1 << size);
  total_bytes = (len + 1) * num_bytes;
  begin
  case (burst)
    `AXI_BURST_INCR: addr = start + beat * num_bytes;
    `AXI_BURST_WRAP: begin
      reg [`AXI_ADDR_W-1:0] lower, upper;
      lower = (start / total_bytes) * total_bytes;  // WRAP total size is a power of 2
      upper = lower + total_bytes;
      addr  = start + beat * num_bytes;
      if (addr >= upper) addr -= total_bytes;
    end
    default: addr = start;  // FIXED
  endcase
  axi_beat_addr = addr;
  end
endfunction

//------------------------------------------------------------------------------
// Address/burst legality check (used as BFM generation constraints)
//   - the burst must not cross a 4KB boundary
//   - INCR/WRAP start address must be size-aligned
//   - for WRAP, len+1 must be in {2,4,8,16}
//------------------------------------------------------------------------------
function automatic axi_addr_legal;
  input [`AXI_ADDR_W-1:0] addr;
  input [1:0]             burst;
  input [2:0]             size;
  input [7:0]             len;
  reg [`AXI_ADDR_W-1:0] num_bytes, total_bytes;
  begin
  num_bytes   = (1 << size);
  total_bytes = (len + 1) * num_bytes;
  axi_addr_legal = 1'b1;
  if ((addr[11:0] + total_bytes) > 4096) axi_addr_legal = 1'b0;
  if (burst != `AXI_BURST_FIXED && (addr & (num_bytes - 1)) != '0)
    axi_addr_legal = 1'b0;
  if (burst == `AXI_BURST_WRAP &&
      (len + 1) != 2 && (len + 1) != 4 && (len + 1) != 8 && (len + 1) != 16)
    axi_addr_legal = 1'b0;
  end
endfunction

//------------------------------------------------------------------------------
// Narrow-transfer WSTRB: the AXI lane-mapping rule - transfer bytes are
// placed on 2^size consecutive lanes starting at addr % lanes (e.g. on a
// 32-bit bus, a 16-bit transfer at address 0x2 carries data on
// WDATA[31:16], WSTRB=1100).
// Constraint: a single beat must not cross the bus-word boundary
// (addr%lanes + 2^size <= lanes) - unaligned narrow transfers crossing
// the word boundary are out of scope for this reference design (allowed
// by AXI but rarely used).
// Shared by RTL and TB reference models to keep them consistent.
//------------------------------------------------------------------------------
function automatic [`AXI_DATA_W/8-1:0] axi_strb_for_size;
  input [2:0]             size;
  input [`AXI_ADDR_W-1:0] addr;
  integer bytes, shift;
  begin
  bytes = 1 << size;   // legal size values keep bytes <= lanes (size=2 is a full word)
  shift = addr % (`AXI_DATA_W/8);
  // Replication counts must be constant (iverilog); use the macro directly
  axi_strb_for_size =
    ({(`AXI_DATA_W/8){1'b1}} >> ((`AXI_DATA_W/8) - bytes)) << shift;
  end
endfunction

//------------------------------------------------------------------------------
// Test data generation (shared by BFM and scoreboard; deterministic and
// uniquely determined by the seed): beat b data = xorshift32 sequence.
// The same seed generates identical data in the BFM and scoreboard,
// avoiding passing arrays around in the TB.
//------------------------------------------------------------------------------
function automatic [`AXI_DATA_W-1:0] axi_test_data;
  input integer seed;
  input integer beat;
  reg [31:0] x;
  begin
  x = seed ^ (beat * 32'h9E3779B9) ^ 32'hA5A5A5A5;
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  axi_test_data = x;
  end
endfunction

//------------------------------------------------------------------------------
// Test WSTRB generation: deterministic; can produce narrow transfers
// (including all-zero beats).
// mode 0 = all ones; mode 1 = random narrow; mode 2 = all zeros
//------------------------------------------------------------------------------
function automatic [`AXI_DATA_W/8-1:0] axi_test_strb;
  input integer strb_seed;
  input integer beat;
  input integer mode;
  reg [31:0] x;
  begin
  x = strb_seed ^ (beat * 32'h85EBCA6B) ^ 32'h5A5A5A5A;
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  case (mode)
    0: axi_test_strb = {(`AXI_DATA_W/8){1'b1}};
    1: axi_test_strb = x[`AXI_DATA_W/8-1:0];
    default: axi_test_strb = '0;
  endcase
  end
endfunction

`endif // AXI_DEFS_SVH
