# AXI4 Multi-Master Multi-Slave Interconnect (Full Crossbar)

A parameterized AXI4 full-crossbar interconnect: **N masters x M slaves**, in synthesizable Verilog/SystemVerilog RTL, with a complete verification environment (iverilog).

## Quick Start

```bash
make sim             # round-robin arbitration build + run (default; 15 BFM-level scenarios)
make sim-fixed       # fixed-priority arbitration build + run (same scenarios, different policy checks)
make sim-sliced      # register-slice build: all 5 slave-side channels registered (same 15 scenarios)
make sim-rtl         # RTL-level co-verification: cfg master + RAM slave + interconnect (R1-R6)
make sim-rtl-sliced  # RTL-level co-verification + register slices (R1-R6)
make sim-mix         # mixed topology: cfg + pipe x ram + lat (M1-M7, incl. narrow transfers)
make sim-reg         # register peripheral: cfg + lite master x reg/ram slave (G1-G6)
make sim-ext         # exclusive access + LFSR traffic: excl + lfsr x ram(excl) + lat (E1-E4)
make SEED=42 sim     # pick a random seed
```

Simulation writes `axi.vcd` / `axi_rtl.vcd` / ... waveforms (open with gtkwave).
Every build must print `ALL ... PASS` to be considered passing.

## Directory Layout

```
rtl/
  axi_interconnect.sv   top-level crossbar (flat packed ports)
  axi_arbiter.sv        generic arbiter (FIXED / RR, grant locked until handshake)
  axi_owner_fifo.sv     W-ownership tag FIFO
  axi_decerr.sv         internal DECERR responder (one per master)
  axi_reg_slice.sv      forward register slice (skid buffer, any single channel)
  axi_master_cfg.sv     synthesizable master: register-configured FSM (single transaction, WSTRB)
  axi_master_pipe.sv    synthesizable master: descriptor-table driven, multi-outstanding pipeline
  axi_master_excl.sv    synthesizable master: exclusive access (ARLOCK/AWLOCK + EXOKAY)
  axi_master_lfsr.sv    synthesizable master: LFSR random-traffic soak source
  axi_master_lite.sv    synthesizable master: AXI4-Lite subset (single-beat register access)
  axi_slave_ram.sv      synthesizable RAM slave: fixed-depth queues + exclusive monitor (EXOKAY)
  axi_slave_lat.sv      synthesizable slave: RAM + configurable latency + SLVERR injection
  axi_slave_reg.sv      synthesizable slave: blocking register peripheral (byte enables)
  axi_defs.svh          width macros / protocol constants / address-lane-data generation functions
tb/
  axi_master_bfm.sv     task-level master BFM (split-phase aw_only/w_only/wait_b)
  axi_slave_model.sv    configurable slave model (random backpressure / latency / memory)
  tb_axi.sv             scoreboard + monitors + 15 scenarios
  tb_axi_rtl.sv         RTL-level co-verification TB (R1-R6, config-driven + reference model)
  tb_axi_mix.sv         mixed-topology TB (M1-M7)
  tb_axi_reg.sv         register-peripheral + Lite TB (G1-G6)
  tb_axi_ext.sv         exclusive + LFSR TB (E1-E4)
  tb_axi_wires.svh      interconnect wires + DUT instance shared by all TBs
```

## Architecture Highlights

- **ID widening routing**: slave-side ID = `{master_tag, id}`; widened at the AW/AR
  mux outputs and narrowed at each master's B/R mux outputs. Responses are routed
  back by the tag - no transaction tracking tables.
- **Independent per-slave AW/AR arbitration**, independent per-master B/R
  arbitration. Grant locking: AW/AR lock until the address-beat handshake, B locks
  until BVALID&&BREADY, **R locks until the RLAST handshake**.
- **At most one in-flight write stream per master** (w_pending gates both the AW
  arbitration requests and AWREADY): guarantees a master's W beats can only flow
  to one destination, preventing W-stream routing ambiguity.
- **W-ownership FIFO** (per slave, depth = N_MASTER): push the granted master on
  the AW handshake, pop on the WLAST handshake; WREADY is shown only to the
  FIFO-head master.
- **DECERR**: unmatched addresses are handled by an internal responder
  (writes: drain W beats then return B=DECERR; reads: queue ARs, one R beat of
  DECERR each), mutually exclusive with the slave W muxes.
- Master-side ports are flattened packed vectors (per-channel fields
  concatenated, accessed via `[i*W +: W]`).

## Register Slices

The interconnect parameter `REG_SLICE` (or `-DAXI_REG_SLICE=1`) inserts a forward
register slice (rtl/axi_reg_slice.sv) on all **5 slave-side channels**, breaking
the combinational paths from the arbiters/muxes to the slave ports at the cost of
one cycle of latency per transaction. Protocol-transparent - the same 15 scenarios
and R1-R6 all pass in the sliced builds.

Design choice: the slices live inside the interconnect rather than in the
reference masters/slaves - users keep their own IP untouched and still benefit
from registered timing (the same approach as Xilinx SmartConnect).
`axi_reg_slice` itself is a generic channel-level building block usable on either
side. The reference masters/slaves intentionally stay combinational to expose the
interconnect's worst-case combinational delay.

Skid-buffer correctness note (learned the hard way): `din_ready` must always
equal `dout_ready`, so the drain beat and the upstream handshake complete on the
same edge - otherwise a backpressured captured beat is replayed through the
pass-through path (the upstream "completing the current beat" and the downstream
"consuming that beat" fall out of sync).

## Configuration (-D macros, see rtl/axi_defs.svh)

`AXI_N_MASTER` / `AXI_M_SLAVE` / `AXI_ADDR_W` / `AXI_DATA_W` / `AXI_ID_W` /
`AXI_ARB_POLICY` (0 = fixed priority, 1 = RR) / `AXI_RESP_POLICY` /
`AXI_W_FIFO_DEPTH` / `AXI_DECERR_Q` / `AXI_REG_SLICE`

When changing the scale, update the address maps and scenarios in the TBs
accordingly (the TBs are hardcoded for a 2x2 topology).

## Reference Design (synthesizable master/slave library)

**Masters**
- **axi_master_cfg**: register-configured (address/length/ID/data base/WSTRB) with
  independent write and read FSMs, single-transaction blocking. Write data is
  generated deterministically (beat b = cfg_wdata0 + b); the read checksum is the
  XOR accumulation of read beats, recomputable by the TB.
- **axi_master_pipe**: descriptor-table driven (N_DESC entries, batch size via
  cfg_ndesc), multi-outstanding pipelining - the next read is issued right after
  its AR handshake; B/R responses are tracked by ID in a slot table; done asserts
  only when all are complete. Write W streams are serialized inline (satisfying
  the interconnect's single-write-stream rule); WSTRB follows the size-based
  narrow-transfer lane rule.
- **axi_master_excl**: exclusive access - an ARLOCK=1 read (waiting for EXOKAY)
  followed by a conditional AWLOCK=1 write (recording EXOKAY/OKAY); skips the
  write if the read was not EXOKAY; configurable read-to-write gap (for testing
  the exclusive-lost window).
- **axi_master_lfsr**: LFSR random-traffic soak source - after enable, issues
  random-direction/address/length transactions back to back until cfg_tx_max;
  tx_vld + tx_* transaction-log outputs let the TB recompute expectations; the
  batch-complete latch prevents auto-rerun.
- **axi_master_lite**: strict AXI4-Lite subset - single beat, fixed 32b, no
  ID/bursts; write and read share busy (AW wins on contention); the typical
  blocking register-access shape.

**Slaves**
- **axi_slave_ram**: true RAM + fixed-depth AW/AR/B queues, multiple outstanding,
  BID/RID echo of the widened ID, WSTRB-masked writes (AXI lane mapping), an
  **exclusive monitor** (exclusive read establishes the monitor point and returns
  EXOKAY; a matching exclusive write returns EXOKAY; any normal write clears it),
  and a combinational debug read port.
- **axi_slave_lat**: on top of the RAM behavior, configurable B/R response
  latency and SLVERR injection (all errors / every Nth transaction) - creates
  response-arbitration contention and verifies error pass-through.
- **axi_slave_reg**: blocking register peripheral - single-transaction
  write/read mutual exclusion (AW wins), a 32-bit register file with WSTRB
  byte-enable partial writes (read-modify-write merge), no queues.

**Narrow-transfer lane mapping** (the consistent rule implemented here): transfer
bytes sit on 2^size consecutive lanes starting at `addr % lanes` (e.g. on a
32-bit bus, a 16-bit transfer at address 0x2 carries data on WDATA[31:16],
WSTRB=1100); slaves read/write at the word-aligned base + lane; a single beat
must not cross the bus-word boundary (out of scope for this reference design).
See `axi_strb_for_size` in axi_defs.svh.

**Verification scenarios**
- tb_axi_rtl.sv (R1-R6): cfg master x ram slave - write/read-back checksums,
  two-master concurrency, same-slave contention, DECERR, WRAP, cross traffic
- tb_axi_mix.sv (M1-M7): cfg + pipe x ram + lat - batch pipelining, SLVERR
  checks, concurrency, latency contention, random descriptor batches,
  narrow/unaligned transfers
- tb_axi_reg.sv (G1-G6): cfg + lite x reg/ram - partial-write read-back, burst
  registers, DECERR, lite concurrency
- tb_axi_ext.sv (E1-E4): excl + lfsr x ram(excl) + lat - exclusive success/lost,
  LFSR soak (two windows x 60 transactions), concurrency

## Roadmap (Plan)

Planned extensions, in order (planned 2026-10):

1. **Hierarchical interconnect (NIC-400 style)** - cascade axi_interconnect with
   itself: 2x2 small switches composed into 4x4 or larger topologies, with
   register slices between levels. This is how real SoCs build interconnect
   fabrics (ARM NIC-400 is a hierarchy of small crossbars):
   - upper-level slave ports connect to lower-level master ports; the address
     map is partitioned per level
   - verification: hierarchical-topology scenarios (cross-level routing,
     same-level contention, inter-level slices preserving semantics)
   - prerequisite: abstract "an interconnect instance is a master/slave
     endpoint" for reuse (the flat packed ports already connect naturally)
2. **Verilator dual-tool verification** - run the same RTL on Verilator 5 to
   rule out iverilog-specific behavior, cross-check correctness, and compare
   simulation speed
3. **yosys synthesis check** - run `synth` for area/resource reports, backing
   the "synthesizable" claim with real synthesis
4. **CI automation** - GitHub Actions running all 8 builds on every change
5. **Standalone protocol checker** - SVA-based AXI4 protocol monitor (VALID
   stability, handshake rules, 4KB boundaries, burst constraints), extracted
   from the TBs' inline checks
6. **Data-width conversion** - 32<->64-bit upsize/downsize adapters (narrow-burst
   lane stitching, WSTRB merging, R splitting); the largest effort, do it last

## Coding Style (wire/reg convention)

This project declares signal types in classic Verilog style (no blanket `logic`):

| Driven by | Declaration |
|---|---|
| `assign` continuous assignment / instance output connection | `wire` |
| `always_ff` / `always_comb` / `initial` / task procedural assignment | `reg` |
| module output port | by its internal driver: assign-driven -> `output wire`, procedurally-driven -> `output reg` |
| module input port | `input wire` |
| unpacked arrays (memories) | `reg` only (Verilog syntax requirement) |

Note: `always_comb` outputs must be declared `reg` - a Verilog syntax
requirement; it does not mean they become registers (combinational logic uses
`reg` for procedural assignment too). Functions use the classic style (the
function name is the return value; inputs are declared inside the body).

## iverilog 12 Compatibility Constraints (hard project rules, discovered the hard way)

These are iverilog 12 defects. **Do not introduce the following patterns into
this project** (they silently produce X, crash elaboration, or fail syntax):

1. **Unpacked arrays cannot be module output ports**: any driver (assign /
   instance connection / always block) silently produces X -> output ports must
   be flattened packed vectors
2. Struct-array-element member access (`arr[i].field`): variable-index reads
   crash, writes are unimplemented, constant-index port connections crash ->
   RTL/TB are fully flattened, no structs
3. Queues of structs (`st_t q[$]`) are syntax errors; associative arrays
   (`bit [7:0] mem[int]`) are unsupported
4. Element selects of 2D packed parameters (`P[0]`) crash -> use 1D packed +
   `[s*W +: W]`
5. Variable-index LHS writes on packed 2D arrays misbehave ("constant selects"
   sorry message)
6. Chained part-selects `x[a +: W][b +: W2]` are unsupported -> intermediate
   variables
7. The same variable driven by multiple always_comb blocks forms a zero-delay
   combinational loop at t=0 (simulation hangs)
8. `arr[genvar].member` anywhere inside a generate block crashes -> copy the
   whole element first
9. `$fatal`'s first argument must be numeric; `$urandom`'s seed must be a
   variable; tasks cannot `return`; hierarchical references need constant
   generate indices
10. Part-selects of parameters require constant indices (genvar unrolling works);
    variable part-selects are not allowed on nets (copy parameters with generate
    assigns)

The usable subset: unpacked arrays (memory style) with arbitrary-index
read/write + generate unrolling + packed-port part-selects + 1D packed bit
writes. These are all standard Verilog-2001 semantics, so the RTL also holds
under other tools (Verilator/VCS etc.).
