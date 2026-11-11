# =============================================================================
# synth_paper.tcl -- out-of-context Vivado synthesis + implementation of the
# paper's current loop (foc_current_cordic) at 100 MHz on the Zybo Z7-10 part.
#   cd Tufekci2020_Paper_Implementation
#   vivado -mode batch -source vivado/synth_paper.tcl
# Reports: build/paper_util.rpt, build/paper_timing.rpt (LUT/FF/DSP/WNS)
# =============================================================================
set part xc7z010clg400-1
file mkdir build
read_verilog -sv {rtl/cordic_rotate.sv rtl/foc_current_cordic.sv}
synth_design -top foc_current_cordic -part $part -mode out_of_context -include_dirs rtl
create_clock -period 10.0 -name clk [get_ports clk]
opt_design; place_design; route_design
report_utilization    -file build/paper_util.rpt
report_timing_summary -file build/paper_timing.rpt
puts ">>> LUT [llength [get_cells -hier -filter {PRIMITIVE_GROUP == LUT}]]  FF [llength [get_cells -hier -filter {PRIMITIVE_GROUP == FLOP_LATCH}]]  DSP [llength [get_cells -hier -filter {REF_NAME =~ DSP*}]]  WNS [get_property SLACK [get_timing_paths -max_paths 1]] ns"
