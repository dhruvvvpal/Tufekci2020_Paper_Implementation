// =============================================================================
// foc_fixed_pkg.svh -- shared fixed-point helpers (include inside a module)
//   Data   (A operand) : 16-bit Q3.13   (range +-4 pu, LSB 1.22e-4 pu)
//   Coeff  (B operand) : 18-bit Q4.14   (sin/cos, 1/sqrt3, Kp, Ki*Ts)
//   Product            : 34-bit Q7.27 ; accumulator 48-bit (DSP48 P width)
//   Integrator state   : kept UNROUNDED in accumulator scale (Q.27)
// =============================================================================
localparam int DW = 16;
localparam int CW = 18;
localparam int PW = 48;
localparam int FR = 14;   // fractional bits dropped when going P -> data

function automatic logic signed [DW-1:0] sat_d(input logic signed [PW-1:0] v);
    if      (v >  $signed({{(PW-DW){1'b0}}, 1'b0, {(DW-1){1'b1}}})) sat_d = {1'b0, {(DW-1){1'b1}}};
    else if (v <  $signed({{(PW-DW){1'b1}}, 1'b1, {(DW-1){1'b0}}})) sat_d = {1'b1, {(DW-1){1'b0}}};
    else                                                             sat_d = v[DW-1:0];
endfunction

// P (Q.27) -> data (Q3.13): round half up, then saturate
function automatic logic signed [DW-1:0] p2d(input logic signed [PW-1:0] p);
    p2d = sat_d((p + (48'sd1 <<< (FR-1))) >>> FR);
endfunction

// data (Q3.13) -> P scale (Q.27)
function automatic logic signed [PW-1:0] d2p(input logic signed [DW-1:0] d);
    d2p = $signed({{(PW-DW){d[DW-1]}}, d}) <<< FR;
endfunction
