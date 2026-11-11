// =============================================================================
// cordic_rotate.sv -- Iterative rotation-mode CORDIC that ROTATES A VECTOR
//   (x, y) by angle phi:  x' = x cos(phi) - y sin(phi)
//                         y' = x sin(phi) + y cos(phi)
//   Park         : rotate (i_alpha, i_beta) by -theta  -> (i_d, i_q)
//   Inverse Park : rotate (v_d, v_q)        by +theta  -> (v_alpha, v_beta)
//   This is the "CORDIC rotate + CORDIC scale" structure of Tufekci et al.
//   (IEEE 2020): NO multiplier -- the CORDIC gain 1/K = 0.6072529 is applied
//   with a 7-term canonical-signed-digit shift-add network (error 1.1e-6).
//
//   x, y in/out : 16-bit Q3.13 (saturating)   phi : 16-bit BAM (2^16 = 2*pi)
//   Internal    : 24-bit Q5.18, 20-bit angle
//   Latency     : 1 accept + 16 iterations + 1 scale/round = 18 clk, 0 DSP
// =============================================================================
module cordic_rotate #(
    parameter int N  = 16,
    parameter int XW = 24,   // Q5.18
    parameter int ZW = 20
)(
    input  logic               clk, rst_n,
    input  logic               in_valid,
    output logic               in_ready,
    input  logic signed [15:0] x_in, y_in,
    input  logic        [15:0] phi,
    output logic               out_valid,
    input  logic               out_ready,
    output logic signed [15:0] x_out, y_out
);
    function automatic logic signed [ZW-1:0] atan_lut(input logic [4:0] i);
        case (i)
            0: atan_lut = 20'sd131072;  1: atan_lut = 20'sd77376;
            2: atan_lut = 20'sd40884;   3: atan_lut = 20'sd20753;
            4: atan_lut = 20'sd10417;   5: atan_lut = 20'sd5213;
            6: atan_lut = 20'sd2607;    7: atan_lut = 20'sd1304;
            8: atan_lut = 20'sd652;     9: atan_lut = 20'sd326;
           10: atan_lut = 20'sd163;    11: atan_lut = 20'sd81;
           12: atan_lut = 20'sd41;     13: atan_lut = 20'sd20;
           14: atan_lut = 20'sd10;     15: atan_lut = 20'sd5;
           default: atan_lut = '0;
        endcase
    endfunction

    // 1/K = 2^-1 + 2^-3 - 2^-6 - 2^-9 - 2^-13 - 2^-14 + 2^-16  (CSD, 0 DSP)
    function automatic logic signed [XW-1:0] scale_k(input logic signed [XW-1:0] v);
        scale_k = (v >>> 1) + (v >>> 3) - (v >>> 6) - (v >>> 9)
                - (v >>> 13) - (v >>> 14) + (v >>> 16);
    endfunction

    // Q5.18 -> Q3.13 : round half up, saturate to 16 bits
    function automatic logic signed [15:0] to_q13(input logic signed [XW-1:0] v);
        logic signed [XW-1:0] r;
        r = (v + 24'sd16) >>> 5;
        if      (r >  24'sd32767) to_q13 = 16'sh7FFF;
        else if (r < -24'sd32768) to_q13 = 16'sh8000;
        else                      to_q13 = r[15:0];
    endfunction

    typedef enum logic [1:0] {IDLE, BUSY, SCALE, DONE} state_t;
    state_t state;
    logic signed [XW-1:0] x, y;
    logic signed [ZW-1:0] z;
    logic [4:0] it;

    assign in_ready = (state == IDLE);

    logic signed [XW-1:0] xs, ys, xn, yn;
    logic signed [ZW-1:0] zn;
    always_comb begin
        xs = x >>> it;
        ys = y >>> it;
        if (!z[ZW-1]) begin xn = x - ys; yn = y + xs; zn = z - atan_lut(it); end
        else          begin xn = x + ys; yn = y - xs; zn = z + atan_lut(it); end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE; out_valid <= 1'b0; x <= '0; y <= '0; z <= '0; it <= '0;
            x_out <= '0; y_out <= '0;
        end else begin
            case (state)
            IDLE: if (in_valid) begin : accept
                logic signed [XW-1:0] xi, yi;
                xi = $signed({{(XW-21){x_in[15]}}, x_in, 5'b0});   // Q3.13 -> Q5.18
                yi = $signed({{(XW-21){y_in[15]}}, y_in, 5'b0});
                if (phi[15] ^ phi[14]) begin       // |phi| > pi/2 : pre-rotate by pi
                    x <= -xi; y <= -yi;
                    z <= $signed({phi, {(ZW-16){1'b0}}}) + $signed(1 <<< (ZW-1));
                end else begin
                    x <= xi;  y <= yi;
                    z <= $signed({phi, {(ZW-16){1'b0}}});
                end
                it <= '0; state <= BUSY;
            end
            BUSY: begin
                x <= xn; y <= yn; z <= zn;
                it <= it + 1'b1;
                if (it == N-1) state <= SCALE;
            end
            SCALE: begin
                x_out <= to_q13(scale_k(x));
                y_out <= to_q13(scale_k(y));
                out_valid <= 1'b1; state <= DONE;
            end
            DONE: if (out_ready) begin out_valid <= 1'b0; state <= IDLE; end
            endcase
        end
    end
endmodule
