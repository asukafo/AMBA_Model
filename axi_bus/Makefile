# AXI4 交叉开关互联 — 仿真（iverilog）
#
#   make sim         : RR 策略构建 + 运行（默认）
#   make sim-fixed   : 固定优先级策略构建 + 运行
#   make clean       : 清理产物
#
# 宽度/规模/策略通过 -D 宏覆盖（见 rtl/axi_defs.svh）：
#   AXI_N_MASTER / AXI_M_SLAVE / AXI_ADDR_W / AXI_DATA_W / AXI_ID_W /
#   AXI_ARB_POLICY / AXI_RESP_POLICY ...
# 注意：改宏时必须同步改 tb/tb_axi.sv 中的地址映射与场景
# （TB 场景按 2 主 × 2 从硬编码）。

IVERILOG := iverilog
VVP      := vvp
FLAGS    := -g2012 -Wall -Irtl -Itb

RTL := rtl/axi_arbiter.sv rtl/axi_owner_fifo.sv rtl/axi_decerr.sv \
       rtl/axi_interconnect.sv
RTL_REF := rtl/axi_master_cfg.sv rtl/axi_master_pipe.sv \
           rtl/axi_master_excl.sv rtl/axi_master_lfsr.sv rtl/axi_master_lite.sv \
           rtl/axi_slave_ram.sv rtl/axi_slave_lat.sv rtl/axi_slave_reg.sv
TB  := tb/axi_master_bfm.sv tb/axi_slave_model.sv tb/tb_axi.sv
TB_RTL := tb/tb_axi_rtl.sv
TB_MIX := tb/tb_axi_mix.sv
TB_REG := tb/tb_axi_reg.sv
TB_EXT := tb/tb_axi_ext.sv

SEED ?= 1

.PHONY: sim sim-fixed sim-rtl sim-mix sim-reg sim-ext clean

# RR（默认策略，AXI_ARB_POLICY=1）
sim: $(RTL) $(TB)
	$(IVERILOG) $(FLAGS) -o sim.vvp $(RTL) $(TB)
	$(VVP) -n sim.vvp +seed=$(SEED)

# 固定优先级（AW/AR 与 B/R 均 FIXED）
sim-fixed: FLAGS += -DAXI_ARB_POLICY=0 -DAXI_RESP_POLICY=0
sim-fixed: sim

# RTL 级联调：可综合 master + RAM slave + 互联（R1-R6 场景）
sim-rtl: $(RTL) $(RTL_REF) $(TB_RTL)
	$(IVERILOG) $(FLAGS) -o sim_rtl.vvp $(RTL) $(RTL_REF) $(TB_RTL)
	$(VVP) -n sim_rtl.vvp +seed=$(SEED)

# 混合拓扑：cfg + pipe master × ram + lat slave（M1-M5 场景）
sim-mix: $(RTL) $(RTL_REF) $(TB_MIX)
	$(IVERILOG) $(FLAGS) -o sim_mix.vvp $(RTL) $(RTL_REF) $(TB_MIX)
	$(VVP) -n sim_mix.vvp +seed=$(SEED)

# 寄存器外设：cfg master + lite master + reg/ram slave（G1-G6 场景）
sim-reg: $(RTL) $(RTL_REF) $(TB_REG)
	$(IVERILOG) $(FLAGS) -o sim_reg.vvp $(RTL) $(RTL_REF) $(TB_REG)
	$(VVP) -n sim_reg.vvp +seed=$(SEED)

# 互斥访问 + LFSR 流量：excl + lfsr master × ram(excl) + lat slave（E1-E4）
sim-ext: $(RTL) $(RTL_REF) $(TB_EXT)
	$(IVERILOG) $(FLAGS) -o sim_ext.vvp $(RTL) $(RTL_REF) $(TB_EXT)
	$(VVP) -n sim_ext.vvp +seed=$(SEED)

clean:
	rm -f sim.vvp sim_rtl.vvp sim_mix.vvp sim_reg.vvp sim_ext.vvp \
	      axi.vcd axi_rtl.vcd axi_mix.vcd axi_reg.vcd axi_ext.vcd
