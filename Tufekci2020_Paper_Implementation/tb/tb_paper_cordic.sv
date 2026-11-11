// =============================================================================
// tb_paper_cordic.sv -- stand-alone testbench for the Tufekci et al. (2020)
// architecture ONLY (no other P11 architecture is compiled).
//   T1  cordic_rotate : 4096 random vectors x angles vs ideal rotation
//   T2  foc_current_cordic : 2000 random samples vs ideal double-precision
//       Clarke -> Park -> PI (+clamping anti-windup) -> inverse Park.
//       The ideal model is re-seeded with the DUT's integrator state each
//       sample (one-step-ahead check), so only arithmetic error is measured.
//   T3  anti-windup : impossible reference for 300 samples, integrator bounded
//   Reports latency in clock cycles (budget: 5000 clk @ 100 MHz, 20 kHz loop).
//   Run:  iverilog -g2012 -I../rtl -o sim ../rtl/*.sv tb_paper_cordic.sv && vvp sim
// =============================================================================
`timescale 1ns/1ps
module tb_paper_cordic;
    localparam real PI = 3.14159265358979, Q13 = 8192.0, Q14 = 16384.0, Q27 = 134217728.0;
    logic clk = 0, rst_n = 0;  always #5 clk = ~clk;

    // ---------------- T1: rotator alone
    logic r_iv = 0, r_ir, r_ov;  logic signed [15:0] r_x, r_y, r_xo, r_yo;  logic [15:0] r_phi;
    cordic_rotate u_rot (.clk, .rst_n, .in_valid(r_iv), .in_ready(r_ir), .x_in(r_x), .y_in(r_y),
        .phi(r_phi), .out_valid(r_ov), .out_ready(1'b1), .x_out(r_xo), .y_out(r_yo));

    // ---------------- T2/T3: paper current loop
    logic in_valid = 0, out_ready = 0, clr_int = 0;
    logic signed [15:0] ia, ib, id_ref, iq_ref, vmax;  logic [15:0] theta;
    logic signed [17:0] kp, ki;
    logic in_ready, out_valid, satd, satq;  logic [7:0] lat;
    logic signed [15:0] va, vb, idq_d, idq_q, vd, vq;
    foc_current_cordic dut (.clk, .rst_n, .in_valid, .in_ready, .ia, .ib, .theta, .id_ref, .iq_ref,
        .kp_d(kp), .ki_d(ki), .kp_q(kp), .ki_q(ki), .vmax, .clr_int, .out_valid, .out_ready,
        .valpha(va), .vbeta(vb), .id_o(idq_d), .iq_o(idq_q), .vd_o(vd), .vq_o(vq),
        .sat_d_o(satd), .sat_q_o(satq), .last_latency(lat));

    function real absr(input real x); absr = (x < 0) ? -x : x; endfunction
    function real clampr(input real x, input real m); clampr = (x > m) ? m : (x < -m) ? -m : x; endfunction

    real m_id, m_iq, m_va, m_vb, int_d, int_q;
    task automatic model();
        real fa, fb, th, c, s, ibe, ed, eq, cd, cq, ud, uq, kpr, kir, vm, vdm, vqm;
        fa = ia/Q13; fb = ib/Q13; th = theta*2.0*PI/65536.0; c = $cos(th); s = $sin(th);
        kpr = kp/Q14; kir = ki/Q14; vm = vmax/Q13;
        ibe = (fa + 2.0*fb)/$sqrt(3.0);
        m_id = fa*c + ibe*s;  m_iq = -fa*s + ibe*c;
        ed = id_ref/Q13 - m_id;  eq = iq_ref/Q13 - m_iq;
        cd = int_d + kir*ed;  ud = cd + kpr*ed;  cq = int_q + kir*eq;  uq = cq + kpr*eq;
        vdm = clampr(ud, vm);  vqm = clampr(uq, vm);
        m_va = vdm*c - vqm*s;  m_vb = vdm*s + vqm*c;
    endtask
    task automatic sample();
        @(negedge clk); int_d = dut.int_d/Q27; int_q = dut.int_q/Q27; in_valid = 1;
        wait (in_ready); @(negedge clk); in_valid = 0;
        wait (out_valid); @(negedge clk); out_ready = 1; @(negedge clk); out_ready = 0;
    endtask

    real e, max_r = 0, max_i = 0, max_v = 0;  integer k, n_err = 0;
    initial begin
        kp = 18'sd8192; ki = 18'sd328; vmax = 16'sd4730;
        repeat (4) @(posedge clk); rst_n = 1;

        // T1
        for (k = 0; k < 4096; k++) begin
            real xr, yr, ph;
            @(negedge clk); r_x = $random % 16000; r_y = $random % 16000; r_phi = $random; r_iv = 1;
            @(negedge clk); r_iv = 0; wait (r_ov); #1;
            ph = r_phi*2.0*PI/65536.0;
            xr = (r_x*$cos(ph) - r_y*$sin(ph))/Q13;  yr = (r_x*$sin(ph) + r_y*$cos(ph))/Q13;
            e = absr(r_xo/Q13 - xr); if (e > max_r) max_r = e;
            e = absr(r_yo/Q13 - yr); if (e > max_r) max_r = e;
            @(negedge clk);
        end
        $display("T1 CORDIC rotator : 4096 vectors, max |err| = %e pu (%0.2f LSB of Q3.13)", max_r, max_r*Q13);

        // T2
        for (k = 0; k < 2000; k++) begin
            ia = $random % 6000; ib = $random % 6000; theta = $random;
            id_ref = $random % 2000; iq_ref = $random % 6000;
            sample(); model();
            e = absr(idq_d/Q13 - m_id); if (e > max_i) max_i = e; if (e > 1e-3) n_err++;
            e = absr(idq_q/Q13 - m_iq); if (e > max_i) max_i = e; if (e > 1e-3) n_err++;
            e = absr(va/Q13 - m_va);    if (e > max_v) max_v = e; if (e > 2e-3) n_err++;
            e = absr(vb/Q13 - m_vb);    if (e > max_v) max_v = e; if (e > 2e-3) n_err++;
        end
        $display("T2 paper FOC loop : max |err| id/iq %e pu, valpha/vbeta %e pu, %0d out of tolerance",
                 max_i, max_v, n_err);

        // T3
        for (k = 0; k < 300; k++) begin
            ia = 0; ib = 0; theta = k*200; id_ref = 0; iq_ref = 16'sd16000; sample();
        end
        $display("T3 anti-windup    : |int_q| = %f pu after 300 saturated samples (bound %f)",
                 absr(dut.int_q/Q27), vmax/Q13);
        $display("LATENCY           : %0d clk per control sample (budget 5000 -> %0.2f %%)", lat, lat/50.0);
        $display("RESULT            : %s",
                 (n_err == 0 && absr(dut.int_q/Q27) <= vmax/Q13 + 1e-6) ? "PASS" : "FAIL");
        $finish;
    end
endmodule
