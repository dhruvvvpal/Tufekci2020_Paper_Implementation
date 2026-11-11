#!/usr/bin/env bash
# Runs the stand-alone testbench of the Tufekci et al. (2020) architecture.
# Needs Icarus Verilog >= 11.  Windows: run the two commands in Git-Bash/WSL,
# or use Vivado xsim:  xvlog -sv -i rtl rtl/*.sv tb/tb_paper_cordic.sv
#                      xelab tb_paper_cordic -s s ; xsim s -R
cd "$(dirname "$0")"
iverilog -g2012 -Irtl -o sim_out/paper.vvp rtl/*.sv tb/tb_paper_cordic.sv
vvp sim_out/paper.vvp | tee sim_out/tb_paper_cordic.log
