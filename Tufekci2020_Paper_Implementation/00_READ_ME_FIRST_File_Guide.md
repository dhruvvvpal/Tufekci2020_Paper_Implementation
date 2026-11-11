# Tufekci et al. (2020): paper implementation package (P11)

This folder holds **only** the parts of P11 that implement the paper below, the paper
itself, its stand-alone testbench, its synthesis script and its results. My own
architecture (resource-shared 1-DSP design) and the 13-DSP baseline are **not** in here.
They live in the main `P11_FOC.zip`.

## 1. The paper
**B. Tufekci, B. Onal, H. Dere, and H. F. Ugurdag**, "Efficient FPGA Implementation of
Field Oriented Control for 3-Phase Machine Drives," *Proc. 2020 IEEE East-West Design &
Test Symposium (EWDTS)*, Varna, Bulgaria, Sep. 2020.
- DOI: https://doi.org/10.1109/EWDTS50664.2020.9224884
- IEEE Xplore: https://ieeexplore.ieee.org/document/9224884
- PDF: `paper/Tufekci2020_EWDTS_FOC_FPGA.pdf`

**Idea implemented.** Park and inverse Park are done by a **CORDIC that rotates the
current/voltage vector directly** ("CORDIC rotate" + "CORDIC scale" blocks in the paper's
Fig. 6), so they need **no multipliers**. The few remaining multiplications (Clarke 1/√3,
the two PI controllers) are **time-shared on one multiply-add unit** (the paper's "MAS"
module). Result: the whole current loop uses **1 DSP48**.

## 2. Folder contents
```
Tufekci2020_Paper_Implementation/
├── 00_READ_ME_FIRST_File_Guide.md   <- this document
├── paper/
│   └── Tufekci2020_EWDTS_FOC_FPGA.pdf
├── rtl/
│   ├── cordic_rotate.sv
│   ├── foc_current_cordic.sv
│   └── foc_fixed_pkg.svh
├── tb/
│   └── tb_paper_cordic.sv
├── vivado/
│   └── synth_paper.tcl
├── run_paper_sim.sh
├── sim_out/
│   ├── tb_paper_cordic.log
│   ├── system_arch2.log
│   ├── p11_arch2.csv
│   └── p11_arch2_fast.csv
└── results/
    ├── overview_arch2.png
    ├── steps_arch2.png
    ├── ripple_arch2.png
    └── metrics_arch2.json
```

## 3. What each file does
| File | What it is | What it does |
|---|---|---|
| `paper/Tufekci2020_EWDTS_FOC_FPGA.pdf` | The paper | Reference being implemented. Sec. II = FOC equations, Sec. III / Fig. 6 = hardware architecture and results table. |
| `rtl/cordic_rotate.sv` | **Paper block: "CORDIC rotate + CORDIC scale"** | Iterative rotation-mode CORDIC. Rotates (x, y) by angle φ: x' = x cosφ − y sinφ, y' = x sinφ + y cosφ. 16 iterations, quadrant pre-fold, 24-bit Q5.18 internal datapath, gain correction 1/K = 0.60725 done with a 7-term shift-add network. **0 DSP**, 18-clock latency, valid/ready handshake. |
| `rtl/foc_current_cordic.sv` | **Paper's FOC current loop** | Clarke → Park → PI(d) + PI(q) with clamping anti-windup → inverse Park, controlled by one FSM. One `cordic_rotate` is **reused** for Park (angle −θ) and inverse Park (angle +θ). One pipelined multiply-add (the "MAS", 1 DSP48) runs Clarke (1 operation) and both PIs (4 operations). 51-clock latency per control sample. |
| `rtl/foc_fixed_pkg.svh` | Shared helper (needed to compile) | Fixed-point formats and round/saturate functions: data Q3.13 (16 bit), coefficients Q4.14 (18 bit), accumulator 48 bit. |
| `tb/tb_paper_cordic.sv` | Stand-alone self-checking testbench | **T1** checks the rotator against ideal math on 4096 random vectors. **T2** checks the full paper current loop against a double-precision model on 2000 random samples. **T3** checks the anti-windup. Prints the latency and PASS/FAIL. |
| `run_paper_sim.sh` | Run script | Compiles and runs the testbench with Icarus Verilog (xsim commands are in its comments). |
| `vivado/synth_paper.tcl` | Vivado script | Out-of-context synthesis, place and route of the paper's loop on the Zybo Z7-10 part at 100 MHz. Prints LUT / FF / DSP / timing slack for the comparison table. |
| `sim_out/tb_paper_cordic.log` | Testbench output | Log of the last run of the stand-alone testbench (PASS). |
| `sim_out/system_arch2.log` | Full-system run log | The paper's loop running inside the complete P11 hardware-in-the-loop system (ARCH = 2): no shoot-through, dead time respected, over-current shut-down in 1 clock, PASS. |
| `sim_out/p11_arch2.csv` | Full-system data (20 kHz) | Speed, currents, voltages and FSM state at every control sample of the 180 ms HIL run. |
| `sim_out/p11_arch2_fast.csv` | Full-system data (1 µs) | Motor currents during the current step and at steady state, used for rise time and PWM ripple. |
| `results/overview_arch2.png` | Plot | Whole run: start-up (align, open loop, closed loop), current step, speed steps, load step, over-current fault, restart, stall fault. HIL vs Python golden model. |
| `results/steps_arch2.png` | Plot | Zoom on the current step, speed steps and load step. |
| `results/ripple_arch2.png` | Plot | Phase currents and id/iq at 20 kHz PWM with 100 ns dead time (current ripple). |
| `results/metrics_arch2.json` | Numbers | All computed metrics (latency, rise time, overshoot, settling, ripple, deviation from golden model). |

## 4. Where the paper's ideas appear in the code
| Paper | Code |
|---|---|
| Eq. (4)–(5) Clarke, simplified with a + b + c = 0 | `foc_current_cordic.sv`: `iab2 = ia + 2·ib`, then × 1/√3 on the shared MAC |
| Eq. (10) Park | `cordic_rotate` with φ = −θ (state `S_PARK_GO` / `S_PARK_WAIT`) |
| Eq. (11) Inverse Park | same `cordic_rotate` with φ = +θ (state `S_IPARK_GO` / `S_IPARK_WAIT`) |
| "Park done by taking only the reverse of the input angle" | `rot_phi = -r_theta` for Park |
| Eq. (12) PI: Pn = Kp·E, In = Ki·Ts·E + In−1, Yn = Pn + In, with anti-windup | MAC instructions `INTD_C`, `VD`, `INTQ_C`, `VQ` + clamping anti-windup in the write-back |
| "CORDIC rotate" + "CORDIC scale" modules (Fig. 6) | `cordic_rotate.sv`: `BUSY` state (rotate) + `SCALE` state (1/K shift-add) |
| "MAS module", "resource sharing", "scheduled" | one DSP48 MAC in `foc_current_cordic.sv`, issued by the FSM (1 Clarke + 4 PI operations) |

## 5. How to run
```bash
./run_paper_sim.sh                                   # stand-alone testbench (~20 s)
vivado -mode batch -source vivado/synth_paper.tcl    # resources + timing on xc7z010
```
On the board and in the full HIL simulation, the paper's loop is selected with **ARCH = 2**
in the main package: `./run_sim.sh system 2` and
`vivado -mode batch -source vivado/build_zybo.tcl -tclargs 2 1`.

## 6. Results of the paper architecture
| Metric | Value |
|---|---|
| Current-loop latency | 51 clk = 0.51 µs (1.02 % of the 50 µs control period) |
| Resources (Yosys estimate, xc7) | 1 DSP48, ~1567 LUT, ~536 FF, 0 BRAM. Replace with Vivado numbers from `synth_paper.tcl` |
| CORDIC rotator accuracy | max error 1.3e-4 pu (≈1 LSB of Q3.13) |
| Loop accuracy vs ideal | id/iq 1.5e-4 pu, vα/vβ 1.9e-4 pu |
| iq step 0 → 0.25 pu | rise 0.30 ms, overshoot 1.6 % |
| Speed step 573 → 1433 rpm | rise 2.79 ms, overshoot 2.2 %, settling 10.9 ms |
| Load step 0.3 pu | dip 0.034 pu, recovery 13.5 ms |
| PWM current ripple (iq pk-pk) | 0.0107 pu |
| HIL vs Python golden model | RMS speed deviation 0.0007 pu |
| Safety in system test | 0 shoot-through, dead time ≥ 10 clk, over-current → gates off in 1 clk |

## 7. Differences from the paper (state these honestly)
- Only the paper's **current-loop core** (Clarke, Park, PI d/q, inverse Park) is reproduced
  here. ADC, encoder interface, speed PI and SVM come from the shared P11 infrastructure
  (main package), identical for all three compared architectures.
- Word lengths: the paper uses 18-bit signals. Here data is 16-bit Q3.13, coefficients are
  18-bit Q4.14 and the CORDIC is 24 bit internally, matching the DSP48 port widths.
- The paper reports 23 clk for Park; this rotator takes 18 clk per rotation (16 iterations +
  load + scale), 51 clk for the whole current loop.
- The paper uses four MAS units; here one MAC is enough because only 5 multiplications
  remain after the CORDIC removes the Park/inverse-Park products.
- The paper reports no DSP count, so the DSP comparison is against the other P11 designs.
