//------------------------------------------------------------------------------
// axi_defs.svh — AXI4 全交叉开关互联 全局定义
//
// 宽度/规模通过 `ifndef 宏定义，编译时可用 -D 覆盖（如 -DAXI_N_MASTER=4）。
// axi_interconnect 的模块参数默认值取自这些宏——改规模/宽度请走 -D，
// 不要只改模块参数（axi_interconnect 里有 initial 检查兜底）。
//
// 注意（工具约束）：本项目跑在 iverilog 12 上，其 SystemVerilog 支持有
// 严重缺陷（struct 数组元素的成员访问会崩溃、struct 队列/关联数组不支持），
// 因此 RTL 与 TB 全部采用扁平端口 + unpacked 数组（memory 风格）+
// generate 展开的保守子集。不要在本项目引入 struct/queue/assoc array。
//
// ID 方案：slave 侧 ID = {master tag(TAG_W), master id}，B/R 响应按 tag
// 路由回 master 后再剥掉 tag，无需事务跟踪表。
//------------------------------------------------------------------------------
`ifndef AXI_DEFS_SVH
`define AXI_DEFS_SVH

// ---- 宽度/规模宏（均可 -D 覆盖）----
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
// slave 侧 ID 宽度 = 主 ID + master tag
`ifndef AXI_SLV_ID_W
`define AXI_SLV_ID_W (`AXI_ID_W + `AXI_TAG_W)
`endif
`ifndef AXI_W_FIFO_DEPTH
`define AXI_W_FIFO_DEPTH 2
`endif
`ifndef AXI_DECERR_Q
`define AXI_DECERR_Q 4
`endif
// 仲裁策略：0 = 固定优先级（编号小者优先），1 = Round-Robin
`ifndef AXI_ARB_POLICY
`define AXI_ARB_POLICY 1
`endif
// B/R 响应通道仲裁策略，同上
`ifndef AXI_RESP_POLICY
`define AXI_RESP_POLICY 1
`endif
// slave 侧通道输出寄存器片（0 = 组合直连；1 = 5 通道全部打一拍，
// 切断互联内部组合路径，每事务 +1 拍延迟）
`ifndef AXI_REG_SLICE
`define AXI_REG_SLICE 0
`endif

// ---- 协议常量 ----
`define AXI_RESP_OKAY   2'b00
`define AXI_RESP_EXOKAY 2'b01
`define AXI_RESP_SLVERR 2'b10
`define AXI_RESP_DECERR 2'b11
`define AXI_BURST_FIXED 2'b00
`define AXI_BURST_INCR  2'b01
`define AXI_BURST_WRAP  2'b10

//------------------------------------------------------------------------------
// AXI 突发拍地址计算（BFM / scoreboard / slave model 共享）
// WRAP 边界按突发总大小 (len+1)*2**size 对齐，起点必须按 size 对齐
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
      lower = (start / total_bytes) * total_bytes;  // WRAP 总大小必为 2 的幂
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
// 地址/突发合法性检查（BFM 生成约束用）
//   - 突发不跨 4KB 边界
//   - INCR/WRAP 起点按 size 对齐
//   - WRAP 时 len+1 ∈ {2,4,8,16}
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
// 窄传输 WSTRB：AXI lane 映射规则——传输字节置于 addr % lanes 起的
// 连续 2^size 个 lane（例如 32b 总线、地址 0x2 的 16b 传输 → 数据在
// WDATA[31:16]，WSTRB=1100）。
// 约束：单拍不跨总线字边界（addr%lanes + 2^size <= lanes）——
// 跨字边界的非对齐窄传输超出本参考设计范围（AXI 允许但极少用）。
// RTL 与 TB 参考模型共用，保证一致。
//------------------------------------------------------------------------------
function automatic [`AXI_DATA_W/8-1:0] axi_strb_for_size;
  input [2:0]             size;
  input [`AXI_ADDR_W-1:0] addr;
  integer bytes, shift;
  begin
  bytes = 1 << size;   // size 合法值使 bytes <= lanes（size=2 即全字）
  shift = addr % (`AXI_DATA_W/8);
  // 复制计数必须为常量（iverilog），直接用宏
  axi_strb_for_size =
    ({(`AXI_DATA_W/8){1'b1}} >> ((`AXI_DATA_W/8) - bytes)) << shift;
  end
endfunction

//------------------------------------------------------------------------------
// 测试数据生成（BFM 与 scoreboard 共享，确定性，由 seed 唯一决定）：
// 拍 b 的数据 = xorshift32 序列，保证同一 seed 在 BFM 与 scoreboard
// 生成完全一致，避免在 TB 里传数组
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
// 测试 WSTRB 生成：确定性，可产生窄传输（含全 0 拍）。
// mode 0 = 全 1；mode 1 = 随机窄；mode 2 = 全 0
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
