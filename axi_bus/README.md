# AXI4 多主多从 Interconnect（全交叉开关）

参数化 AXI4 全交叉开关互联：**N master × M slave**，纯 Verilog/SystemVerilog 可综合 RTL + 完整验证环境（iverilog）。

## 快速开始

```bash
make sim             # Round-Robin 仲裁构建 + 运行（默认，15 场景 BFM 级验证）
make sim-fixed       # 固定优先级仲裁构建 + 运行（同场景，策略检查不同）
make sim-sliced      # 寄存器片构建：互联 slave 侧 5 通道打拍（同 15 场景）
make sim-rtl         # RTL 级联调：cfg master + RAM slave + 互联（R1-R6）
make sim-rtl-sliced  # RTL 级联调 + 寄存器片（R1-R6）
make sim-mix         # 混合拓扑：cfg + pipe × ram + lat（M1-M7，含窄传输）
make sim-reg         # 寄存器外设：cfg + lite master × reg/ram slave（G1-G6）
make sim-ext         # 互斥访问 + LFSR 流量：excl + lfsr × ram(excl) + lat（E1-E4）
make SEED=42 sim     # 指定随机种子
```

仿真输出 `axi.vcd` / `axi_rtl.vcd` 波形（gtkwave 可直接打开）。

## 目录

```
rtl/
  axi_interconnect.sv   顶层交叉开关（扁平 packed 端口）
  axi_arbiter.sv        通用仲裁器（FIXED / RR，授权锁到握手）
  axi_owner_fifo.sv     W 归属 tag FIFO
  axi_decerr.sv         内部 DECERR 响应器（每 master 一份）
  axi_reg_slice.sv      前向寄存器片（skid buffer，任意通道打拍）
  axi_master_cfg.sv     可综合 master：寄存器配置 + FSM（单事务，支持 WSTRB）
  axi_master_pipe.sv    可综合 master：描述符表驱动、多 outstanding 流水（窄传输）
  axi_master_excl.sv    可综合 master：互斥访问（ARLOCK/AWLOCK + EXOKAY）
  axi_master_lfsr.sv    可综合 master：LFSR 随机流量 soak 源
  axi_master_lite.sv    可综合 master：AXI4-Lite 子集（单拍寄存器访问）
  axi_slave_ram.sv      可综合 RAM slave：定深队列 + 互斥监视器（EXOKAY）
  axi_slave_lat.sv      可综合 slave：RAM + 可配置延迟 + SLVERR 注入
  axi_slave_reg.sv      可综合 slave：阻塞式寄存器外设（字节使能）
  axi_defs.svh          宽度宏 / 协议常量 / 地址·lane·数据生成函数
tb/
  axi_master_bfm.sv     任务级 master BFM（aw_only/w_only/wait_b 拆分相位）
  axi_slave_model.sv    可配置 slave 模型（随机背压 / 延迟 / 内存）
  tb_axi.sv             scoreboard + 监视器 + 15 场景
  tb_axi_rtl.sv         RTL 级联调 TB（R1-R6，配置驱动 + 参考模型比对）
  tb_axi_mix.sv         混合拓扑 TB（M1-M7）
  tb_axi_reg.sv         寄存器外设 + Lite TB（G1-G6）
  tb_axi_ext.sv         互斥 + LFSR TB（E1-E4）
  tb_axi_wires.svh      各 TB 共用的互联连线 + DUT 实例
```

## Reference design（可综合 master/slave 库）

**Masters**
- **axi_master_cfg**：寄存器配置（地址/长度/ID/数据基值/WSTRB）+ 读写两路
  独立 FSM，单事务阻塞式。写数据确定性生成（第 b 拍 = cfg_wdata0 + b），
  读校验和 = 读拍 XOR 累加，便于 TB 复算比对。
- **axi_master_pipe**：描述符表（N_DESC 条，cfg_ndesc 定批大小）驱动，
  多 outstanding 流水——读事务 AR 握手后立即发下一笔，B/R 按 ID 槽位
  跟踪，全部完成才拉 done。写事务 W 内联串行（满足互联单写流规则）；
  WSTRB 按 size 生成（窄传输 lane 规则）。
- **axi_master_excl**：互斥访问——ARLOCK=1 读（等 EXOKAY）→ AWLOCK=1
  条件写（记录 EXOKAY/OKAY）；读非 EXOKAY 时跳过写；可配读写间隔
  （测试互斥丢失窗口）。
- **axi_master_lfsr**：LFSR 随机流量 soak 源——enable 后自动生成随机
  方向/地址/长度事务直到 cfg_tx_max 笔；tx_vld+tx_* 事务日志输出供
  TB 监听复算；完成锁存（撤 enable 解锁，防自动重跑）。
- **axi_master_lite**：AXI4-Lite 子集——单拍、固定 32b、无 ID/突发；
  读写共用 busy（AW 优先），阻塞式寄存器访问形态。

**Slaves**
- **axi_slave_ram**：真 RAM + 定深 AW/AR/B 队列，多笔 outstanding，
  BID/RID 回显加宽 ID，WSTRB 掩码写入（AXI lane 映射），**互斥监视器**
  （互斥读建监视点回 EXOKAY，互斥写命中回 EXOKAY，普通写清除），
  组合调试读口。
- **axi_slave_lat**：RAM 基础上加可配置 B/R 响应延迟与 SLVERR 注入
  （全错 / 每第 N 笔错），制造响应仲裁竞争、验证错误直通。
- **axi_slave_reg**：阻塞式寄存器外设——单事务读写互斥（AW 优先），
  32b 寄存器堆 + WSTRB 字节使能部分写（读改写合并），无队列。

**窄传输 lane 映射（本库实现的一致规则）**：传输字节置于 addr % lanes
起的连续 2^size 个 lane；slave 按字对齐基址 + lane 读写；单拍不跨总线
字边界（超出参考设计范围，见 axi_defs.svh 的 axi_strb_for_size）。

**验证场景**
- tb_axi_rtl.sv（R1-R6）：cfg master × ram slave——写读回校验和、双主并发、
  同 slave 竞争、DECERR、WRAP、交叉流量
- tb_axi_mix.sv（M1-M7）：cfg + pipe × ram + lat——批量流水、SLVERR 检查、
  并发、延迟竞争、随机描述符多轮、窄传输/非对齐
- tb_axi_reg.sv（G1-G6）：cfg + lite × reg/ram——部分写读回、突发寄存器、
  DECERR、lite 并发
- tb_axi_ext.sv（E1-E4）：excl + lfsr × ram(excl) + lat——互斥成功/丢失、
  LFSR soak（两窗口各 60 笔）、并发

## 架构要点

- **ID 加宽路由**：slave 侧 ID = `{master_tag, id}`，在 AW/AR mux 输出加宽、
  在每 master 的 B/R mux 输出剥窄；响应按 tag 路由，无事务跟踪表
- **每 slave 独立 AW/AR 仲裁**，每 master 独立 B/R 仲裁；授权锁定：
  AW/AR 锁到地址拍握手、B 锁到 BVALID&&BREADY、**R 锁到 RLAST 握手**
- **每 master 至多一条在途写流**（`w_pending` 门控 AW 请求与 AWREADY）：
  保证一个 master 的 W 拍只可能流向唯一目的地，杜绝 W 流路由歧义
- **W 归属 FIFO**（每 slave，深度 = N_MASTER）：AW 握手 push 授权 master，
  WLAST 握手 pop；WREADY 只发给 head 指向的 master
- **DECERR**：未命中地址由内部响应器处理（写：吞 W 拍后回 B=DECERR；
  读：AR 入队、每笔回一拍 R=DECERR），与 slave W mux 互斥
- master 侧端口为拍平 packed 向量（每通道字段拼接，`[i*W +: W]` 访问）

## 寄存器片（register slice）

互联参数 `REG_SLICE`（或 `-DAXI_REG_SLICE=1`）在 **slave 侧 5 个通道**插入
前向寄存器片（rtl/axi_reg_slice.sv），切断仲裁器→mux→slave 端口的组合路径，
每事务 +1 拍延迟。协议透明——同一套 15 场景与 R1-R6 在切片构建下全 PASS。

设计选择：寄存器片放在**互联内**而不是 reference master/slave 里——
使用者挂自己的 IP 无需改动即获得打拍收益（Xilinx SmartConnect 同款做法）；
`axi_reg_slice` 本身是通道级标准件，master/slave 侧需要时同样可复用。
reference master/slave 有意保持纯组合，以暴露互联的最大组合延迟。

skid buffer 关键正确性（踩坑记录）：`din_ready` 必须恒等于 `dout_ready`，
使排空拍与上游握手在同一沿完成——否则背压捕获的拍在排空后会被直通
重放一次（上游"完成当前拍"与下游"消费该拍"不同步）。

## 配置（-D 宏，见 rtl/axi_defs.svh）

`AXI_N_MASTER` / `AXI_M_SLAVE` / `AXI_ADDR_W` / `AXI_DATA_W` / `AXI_ID_W` /
`AXI_ARB_POLICY`（0=固定优先级，1=RR）/ `AXI_RESP_POLICY` /
`AXI_W_FIFO_DEPTH` / `AXI_DECERR_Q` / `AXI_REG_SLICE`

改规模时需同步改 tb/tb_axi.sv 的地址映射与场景（TB 按 2×2 硬编码）。

## 验证场景（tb/tb_axi.sv，两种仲裁构建各跑一遍）

| 场景 | 覆盖点 |
|---|---|
| S1 | 双主并发 INCR 读写不同 slave |
| S2/S3 | 全主抢同一 slave（FIXED 不变量 / RR 竞争交替公平性）|
| S4 | 同 master 双读乱序响应（RID 匹配）|
| S5 | WRAP 突发非对齐读写 |
| S6 | 窄传输随机 WSTRB（含全 0 拍）|
| S7/S9 | DECERR 读写 / 多条 outstanding AR FIFO 顺序 |
| S8 | W 通道交错（两主 AW 先后授权，m1 提前置 WVALID）|
| S10 | 双主随机混合流量 + DECERR 点缀 |
| S11 | 随机到达间隔抢同一 slave |
| S12 | DECERR 多拍写与真实写并行（slave W 背压）|
| S13 | 两 slave 同拍 BVALID 竞争 B 仲裁 |
| S14 | RREADY 停摆下 R 授权锁持有（无死锁）|
| S15 | 单 master 多 ID 并发 |

## 路线图（Plan）

已排序的扩展计划（2026-10 规划）：

1. **层级互联（NIC-400 风格）**——用 `axi_interconnect` 级联自身：2×2 小开关
   组合成 4×4/更大规模，级间插寄存器片。真实 SoC 的拓扑做法（ARM NIC-400
   就是层级化的小交叉开关网络）：
   - 上层互联的 slave 端口接下层互联的 master 端口，地址映射分层划分
   - 验证：层级拓扑场景（跨层路由、同层竞争、级间片不破坏语义）
   - 需要先抽象"互联的实例即 master/slave 端点"的复用方式（端口已是
     扁平 packed，天然可互联互接）
2. **Verilator 双工具验证**——同一套 RTL 在 Verilator 5 上跑通，排除
   iverilog 特有行为，交叉验证正确性并对比仿真速度
3. **yosys 综合验证**——跑 `synth` 出面积/资源报告，验证"可综合"承诺
4. **CI 自动化**——GitHub Actions 跑全部 8 个构建，改动即回归
5. **独立协议检查器**——SVA 断言的 AXI4 protocol monitor（VALID 稳定性、
   握手规则、4KB 边界、burst 约束），从各 TB 内联检查中抽出独立化
6. **数据宽度转换**——32↔64 位 upsize/downsize 适配器（窄突发 lane 拼接、
   WSTRB 合并、R 拆分），工作量大，最后做

## 代码风格（wire/reg 约定）

本项目按经典 Verilog 风格声明信号类型（不使用 `logic` 统称）：

| 驱动方式 | 声明 |
|---|---|
| `assign` 连续赋值 / 实例输出端口连接 | `wire` |
| `always_ff` / `always_comb` / `initial` / task 过程赋值 | `reg` |
| 模块输出端口 | 看内部驱动方式：assign 驱动 → `output wire`，过程驱动 → `output reg` |
| 模块输入端口 | `input wire` |
| unpacked 数组（memory） | 只能 `reg`（Verilog 语法限制） |

注意：`always_comb` 的输出必须声明为 `reg`——这是 Verilog 的语法要求，
不代表它综合成寄存器（组合逻辑的过程赋值同样用 reg）。函数为经典
Verilog 风格（函数名即返回值，输入在函数体内声明）。

## iverilog 12 兼容性约束（本项目硬性规则，实测踩坑）

这些是 iverilog 12 的缺陷，**不要在本项目引入以下写法**（会被静默产出 X、
elab 崩溃或语法错误）：

1. **unpacked 数组不能做模块输出端口**：任何驱动方式（assign / 实例连接 /
   always 块）都静默产出 X → 输出端口一律用拍平 packed 向量
2. struct 数组元素的成员访问（`arr[i].field`）：变量索引读崩溃、写未实现、
   常量索引端口连接崩溃 → RTL/TB 全面扁平化，不用 struct
3. 结构体队列（`st_t q[$]`）语法错误、关联数组（`bit [7:0] mem[int]`）不支持
4. 2D packed 参数的元素选择（`P[0]`）崩溃 → 用 1D packed + `[s*W +: W]`
5. packed 2D 的变量索引写（LHS）行为错误（"constant selects" sorry）
6. 链式部分选择 `x[a +: W][b +: W2]` 不支持 → 中间变量承接
7. 同一变量被多个 always_comb 驱动 → t=0 零延迟组合环（仿真卡死）
8. generate 块内 `arr[genvar].member` 任何用法崩溃 → 先整元素拷贝
9. `$fatal` 第一参数必须是数字；`$urandom` 的 seed 必须是变量；
   task 里不能 `return`；层级引用的 generate 索引必须是常量
10. 参数的部分选择要求常量索引（genvar 展开可以）；变量部分选择不允许
    作用于 net（参数拷贝用 generate assign）

可用子集：unpacked 数组（memory 风格）任意索引读写 + generate 展开 +
packed 端口部分选择 + 1D packed 位写。这些都是标准 Verilog-2001 语义，
因此该 RTL 在其他工具（Verilator/VCS 等）上同样成立。
