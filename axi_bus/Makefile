# AXI4 crossbar interconnect - simulation (iverilog)
#
#   make sim         : RR policy build + run (default)
#   make sim-fixed   : fixed-priority policy build + run
#   make clean       : remove build artifacts
#
# Widths/scale/policy are overridden via -D macros (see rtl/axi_defs.svh):
#   AXI_N_MASTER / AXI_M_SLAVE / AXI_ADDR_W / AXI_DATA_W / AXI_ID_W /
#   AXI_ARB_POLICY / AXI_RESP_POLICY ...
# Note: changing the macros requires updating the address maps and scenarios
# in the TBs (scenarios are hardcoded for a 2-master x 2-slave topology).

IVERILOG := iverilog
VVP      := vvp
FLAGS    := -g2012 -Wall -Irtl -Itb

RTL := rtl/axi_arbiter.sv rtl/axi_owner_fifo.sv rtl/axi_decerr.sv \
       rtl/axi_reg_slice.sv rtl/axi_interconnect.sv
RTL_REF := rtl/axi_master_cfg.sv rtl/axi_master_pipe.sv \
           rtl/axi_master_excl.sv rtl/axi_master_lfsr.sv rtl/axi_master_lite.sv \
           rtl/axi_slave_ram.sv rtl/axi_slave_lat.sv rtl/axi_slave_reg.sv
TB  := tb/axi_master_bfm.sv tb/axi_slave_model.sv tb/tb_axi.sv
TB_RTL := tb/tb_axi_rtl.sv
TB_MIX := tb/tb_axi_mix.sv
TB_REG := tb/tb_axi_reg.sv
TB_EXT := tb/tb_axi_ext.sv

SEED ?= 1

.PHONY: sim sim-fixed sim-sliced sim-rtl sim-rtl-sliced sim-mix sim-reg sim-ext clean

# RR (default policy, AXI_ARB_POLICY=1)
sim: $(RTL) $(TB)
	$(IVERILOG) $(FLAGS) -o sim.vvp $(RTL) $(TB)
	$(VVP) -n sim.vvp +seed=$(SEED)

# Fixed priority (both AW/AR and B/R FIXED)
sim-fixed: FLAGS += -DAXI_ARB_POLICY=0 -DAXI_RESP_POLICY=0
sim-fixed: sim

# Register-slice build: all 5 slave-side channels registered (same scenarios verify protocol transparency)
sim-sliced: FLAGS += -DAXI_REG_SLICE=1
sim-sliced: sim

sim-rtl-sliced: FLAGS += -DAXI_REG_SLICE=1
sim-rtl-sliced: sim-rtl

# RTL-level co-verification: synthesizable master + RAM slave + interconnect (R1-R6 scenarios)
sim-rtl: $(RTL) $(RTL_REF) $(TB_RTL)
	$(IVERILOG) $(FLAGS) -o sim_rtl.vvp $(RTL) $(RTL_REF) $(TB_RTL)
	$(VVP) -n sim_rtl.vvp +seed=$(SEED)

# Mixed topology: cfg + pipe masters x ram + lat slaves (M1-M7 scenarios)
sim-mix: $(RTL) $(RTL_REF) $(TB_MIX)
	$(IVERILOG) $(FLAGS) -o sim_mix.vvp $(RTL) $(RTL_REF) $(TB_MIX)
	$(VVP) -n sim_mix.vvp +seed=$(SEED)

# Register peripheral: cfg + lite masters x reg/ram slaves (G1-G6 scenarios)
sim-reg: $(RTL) $(RTL_REF) $(TB_REG)
	$(IVERILOG) $(FLAGS) -o sim_reg.vvp $(RTL) $(RTL_REF) $(TB_REG)
	$(VVP) -n sim_reg.vvp +seed=$(SEED)

# Exclusive access + LFSR traffic: excl + lfsr masters x ram(excl) + lat slaves (E1-E4)
sim-ext: $(RTL) $(RTL_REF) $(TB_EXT)
	$(IVERILOG) $(FLAGS) -o sim_ext.vvp $(RTL) $(RTL_REF) $(TB_EXT)
	$(VVP) -n sim_ext.vvp +seed=$(SEED)

clean:
	rm -f sim.vvp sim_rtl.vvp sim_mix.vvp sim_reg.vvp sim_ext.vvp \
	      axi.vcd axi_rtl.vcd axi_mix.vcd axi_reg.vcd axi_ext.vcd
