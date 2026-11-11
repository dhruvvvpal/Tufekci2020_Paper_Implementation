// =============================================================================
// foc_current_cordic.sv -- "Paper" architecture (Tufekci, Onal, Dere, Ugurdag,
//   "Efficient FPGA Implementation of Field Oriented Control for 3-Phase
//   Machine Drives", IEEE 2020):
//     * Park / inverse Park done by a CORDIC VECTOR ROTATOR + CORDIC SCALE
//       (no multipliers, one rotator time-shared between Park and inv. Park)
//     * remaining multiplies (Clarke 1/sqrt3, 2x PI = 4) scheduled on ONE
//       shared MAC (the paper's "MAS" unit) -> 1 DSP48
//   Same ports, Q-formats, PI + clamping anti-windup as foc_current_shared,
//   so the three architectures are drop-in interchangeable (ARCH param).
//   Latency ~ 50 clk (two 18-clk rotations dominate).
// =============================================================================
module foc_current_cordic (
    input  logic               clk, rst_n,
    input  logic               in_valid,
    output logic               in_ready,
    input  logic signed [15:0] ia, ib,
    input  logic        [15:0] theta,
    input  logic signed [15:0] id_ref, iq_ref,
    input  logic signed [17:0] kp_d, ki_d, kp_q, ki_q,
    input  logic signed [15:0] vmax,
    input  logic               clr_int,
    output logic               out_valid,
    input  logic               out_ready,
    output logic signed [15:0] valpha, vbeta, id_o, iq_o, vd_o, vq_o,
    output logic               sat_d_o, sat_q_o,
    output logic        [7:0]  last_latency
);
    `include "foc_fixed_pkg.svh"

    typedef enum logic [3:0] {S_IDLE, S_CLARKE, S_PARK_GO, S_PARK_WAIT, S_PI,
                              S_IPARK_GO, S_IPARK_WAIT, S_DONE} st_t;
    st_t st;
    logic [3:0] cnt;
    logic [7:0] lat_cnt;

    logic signed [DW-1:0] r_ia, r_ib, r_idref, r_iqref, r_ibeta, r_id, r_iq, r_vd, r_vq, r_va, r_vb;
    logic        [15:0]   r_theta;
    logic signed [PW-1:0] int_d, int_q, cand_d, cand_q;

    // ------------------------------------------------ shared CORDIC rotator
    logic               rot_in_valid, rot_in_ready, rot_out_valid, rot_out_ready;
    logic signed [15:0] rot_x, rot_y, rot_xo, rot_yo;
    logic        [15:0] rot_phi;
    cordic_rotate u_rot (.clk, .rst_n,
        .in_valid(rot_in_valid), .in_ready(rot_in_ready),
        .x_in(rot_x), .y_in(rot_y), .phi(rot_phi),
        .out_valid(rot_out_valid), .out_ready(rot_out_ready),
        .x_out(rot_xo), .y_out(rot_yo));

    always_comb begin
        rot_in_valid  = (st == S_PARK_GO) || (st == S_IPARK_GO);
        rot_out_ready = (st == S_PARK_WAIT) || (st == S_IPARK_WAIT);
        if (st == S_IPARK_GO) begin rot_x = r_vd; rot_y = r_vq;    rot_phi = r_theta;  end
        else                  begin rot_x = r_ia; rot_y = r_ibeta; rot_phi = -r_theta; end
    end

    assign in_ready = (st == S_IDLE);

    // ------------------------------------------------ shared MAC (1 DSP48)
    localparam logic [2:0] A_IAB2=0, A_ED=1, A_EQ=2;
    localparam logic [2:0] B_K3=0, B_KPD=1, B_KID=2, B_KPQ=3, B_KIQ=4;
    localparam logic [1:0] Z_0=0, Z_P=1, Z_CID=2, Z_CIQ=3;
    localparam logic [3:0] D_NONE=0, D_IBETA=1, D_INTDC=4, D_VD=5, D_INTQC=6, D_VQ=7;

    logic signed [DW-1:0] iab2, e_d, e_q;
    assign iab2 = sat_d($signed({{(PW-DW){r_ia[DW-1]}}, r_ia}) + ($signed({{(PW-DW){r_ib[DW-1]}}, r_ib}) <<< 1));
    assign e_d  = sat_d($signed({{(PW-DW){r_idref[DW-1]}}, r_idref}) - $signed({{(PW-DW){r_id[DW-1]}}, r_id}));
    assign e_q  = sat_d($signed({{(PW-DW){r_iqref[DW-1]}}, r_iqref}) - $signed({{(PW-DW){r_iq[DW-1]}}, r_iq}));

    // issue logic: Clarke = 1 instr; PI = 4 instrs (back-to-back)
    logic       i_v, i_sub; logic [2:0] i_a, i_b; logic [1:0] i_z; logic [3:0] i_dst;
    always_comb begin
        {i_v, i_sub, i_a, i_b, i_z, i_dst} = '0;
        if (st == S_CLARKE && cnt == 0) begin
            i_v = 1; i_a = A_IAB2; i_b = B_K3; i_z = Z_0; i_dst = D_IBETA;
        end else if (st == S_PI && cnt < 4) begin
            i_v = 1;
            case (cnt)
              0: begin i_a = A_ED; i_b = B_KID; i_z = Z_CID; i_dst = D_INTDC; end
              1: begin i_a = A_ED; i_b = B_KPD; i_z = Z_P;   i_dst = D_VD;    end
              2: begin i_a = A_EQ; i_b = B_KIQ; i_z = Z_CIQ; i_dst = D_INTQC; end
              default: begin i_a = A_EQ; i_b = B_KPQ; i_z = Z_P; i_dst = D_VQ; end
            endcase
        end
    end

    logic signed [DW-1:0] a_mux; logic signed [CW-1:0] b_mux;
    always_comb begin
        case (i_a) A_ED: a_mux = e_d; A_EQ: a_mux = e_q; default: a_mux = iab2; endcase
        case (i_b) B_KPD: b_mux = kp_d; B_KID: b_mux = ki_d; B_KPQ: b_mux = kp_q;
                   B_KIQ: b_mux = ki_q; default: b_mux = 18'sd9459; endcase
    end

    (* use_dsp = "yes" *) logic signed [PW-1:0] p_r;
    logic signed [DW-1:0] a_r; logic signed [CW-1:0] b_r; logic signed [DW+CW-1:0] m_r;
    logic [1:0] z1, z2; logic s1, s2, v1, v2, v3; logic [3:0] d1, d2, d3;
    always_ff @(posedge clk) begin
        a_r <= a_mux; b_r <= b_mux; z1 <= i_z; s1 <= i_sub; v1 <= i_v; d1 <= i_dst;
        m_r <= a_r * b_r;  z2 <= z1; s2 <= s1; v2 <= v1; d2 <= d1;
        begin : st3
            logic signed [PW-1:0] zv, mv;
            mv = {{(PW-DW-CW){m_r[DW+CW-1]}}, m_r};
            case (z2) Z_0: zv = '0; Z_P: zv = p_r; Z_CID: zv = int_d; default: zv = int_q; endcase
            if (v2) p_r <= s2 ? (zv - mv) : (zv + mv);
        end
        v3 <= v2; d3 <= d2;
    end

    logic signed [PW-1:0] vmax_p;
    assign vmax_p = d2p(vmax);
    function automatic logic signed [PW-1:0] clamp_p(input logic signed [PW-1:0] v,
                                                      input logic signed [PW-1:0] m);
        clamp_p = (v > m) ? m : (v < -m) ? -m : v;
    endfunction

    // ------------------------------------------------ writeback + FSM
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= S_IDLE; cnt <= '0; out_valid <= 1'b0; lat_cnt <= '0; last_latency <= '0;
            {r_ia, r_ib, r_idref, r_iqref, r_ibeta, r_id, r_iq, r_vd, r_vq, r_va, r_vb} <= '0;
            r_theta <= '0; int_d <= '0; int_q <= '0; cand_d <= '0; cand_q <= '0;
            sat_d_o <= 1'b0; sat_q_o <= 1'b0;
        end else begin
            if (st != S_IDLE && st != S_DONE) lat_cnt <= lat_cnt + 1'b1;
            if (clr_int) begin int_d <= '0; int_q <= '0; end
            // MAC writeback (identical anti-windup to foc_current_shared)
            if (v3) case (d3)
                D_IBETA: r_ibeta <= p2d(p_r);
                D_INTDC: cand_d  <= p_r;
                D_INTQC: cand_q  <= p_r;
                D_VD: begin : wb_vd
                    logic sat; sat = (p_r > vmax_p) || (p_r < -vmax_p);
                    r_vd <= p2d(clamp_p(p_r, vmax_p)); sat_d_o <= sat;
                    if (!(sat && (e_d[DW-1] == p_r[PW-1])) && !clr_int) int_d <= clamp_p(cand_d, vmax_p);
                end
                D_VQ: begin : wb_vq
                    logic sat; sat = (p_r > vmax_p) || (p_r < -vmax_p);
                    r_vq <= p2d(clamp_p(p_r, vmax_p)); sat_q_o <= sat;
                    if (!(sat && (e_q[DW-1] == p_r[PW-1])) && !clr_int) int_q <= clamp_p(cand_q, vmax_p);
                end
                default: ;
            endcase

            case (st)
            S_IDLE: if (in_valid) begin
                r_ia <= ia; r_ib <= ib; r_theta <= theta; r_idref <= id_ref; r_iqref <= iq_ref;
                lat_cnt <= 8'd1; cnt <= '0; st <= S_CLARKE;
            end
            S_CLARKE: begin                       // issue at cnt 0, result readable at 4
                cnt <= cnt + 1'b1;
                if (cnt == 4) st <= S_PARK_GO;
            end
            S_PARK_GO:   if (rot_in_ready) st <= S_PARK_WAIT;
            S_PARK_WAIT: if (rot_out_valid) begin
                r_id <= rot_xo; r_iq <= rot_yo; cnt <= '0; st <= S_PI;
            end
            S_PI: begin                           // issue 0..3, last writeback at 7
                cnt <= cnt + 1'b1;
                if (cnt == 7) st <= S_IPARK_GO;
            end
            S_IPARK_GO:   if (rot_in_ready) st <= S_IPARK_WAIT;
            S_IPARK_WAIT: if (rot_out_valid) begin
                r_va <= rot_xo; r_vb <= rot_yo;
                out_valid <= 1'b1; last_latency <= lat_cnt; st <= S_DONE;
            end
            S_DONE: if (out_ready) begin out_valid <= 1'b0; st <= S_IDLE; end
            default: st <= S_IDLE;
            endcase
        end
    end
    assign valpha = r_va; assign vbeta = r_vb; assign id_o = r_id; assign iq_o = r_iq;
    assign vd_o = r_vd;   assign vq_o = r_vq;
endmodule
