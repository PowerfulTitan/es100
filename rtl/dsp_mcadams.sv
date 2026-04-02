module dsp_mcadams (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               sample_strobe,

    input  wire signed [23:0] in_left,
    input  wire signed [23:0] in_right,

    input  wire               enable,
    input  wire [3:0]         alpha_sel,
    input  wire               cfg_change,

    output reg  signed [23:0] out_left,
    output reg  signed [23:0] out_right,

    output wire               dbg_busy,
    output wire [15:0]        dbg_rootcount
);

    // Core frame and model parameters
    localparam integer P          = 10;
    localparam integer FRAME_N    = 1024;
    localparam integer HOP        = 512;
    localparam integer GRID_N     = 256;
    localparam integer BISECT_IT  = 8;

    localparam integer COEF_FRAC  = 30;         // Q30 coefficient format

    // Angles in Q28 radians
    localparam signed [31:0] PI_Q28     = 32'sh3243F6A9;
    localparam signed [31:0] PI_2_Q28   = 32'sh1921FB54;
    localparam signed [31:0] STEP_Q28   = 32'sd3294198;

    // 1/pi in Q30
    localparam signed [31:0] INV_PI_Q30 = 32'sh145F306E;

    // CORDIC constants in Q30
    localparam signed [31:0] CORDIC_KINV_Q30 = 32'sh26DD3B6A;
    localparam integer       CORDIC_IT       = 16;

    wire dbg_residual = (alpha_sel == 4'd0);

    // Saturation and absolute value helpers
    function [23:0] sat24;
        input signed [63:0] x;
        begin
            if (x > 64'sd8388607)       sat24 = 24'sh7FFFFF;
            else if (x < -64'sd8388608) sat24 = 24'sh800000;
            else                        sat24 = x[23:0];
        end
    endfunction

    function [63:0] abs64;
        input signed [63:0] x;
        begin
            abs64 = x[63] ? (~x + 64'd1) : x;
        end
    endfunction

    // Warp LUT for u' = u^alpha with u in Q0.16
    // Uses piecewise linear interpolation between 16 points
    // alpha_sel 8 is the identity mapping
    // alpha_sel 0 also uses identity here because residual debug bypasses warping
    function [15:0] warp_y;
        input [3:0] sel;
        input [4:0] idx;
        begin
            warp_y = 16'd0;
            case (sel)
                4'd1: begin // 0.30
                    case (idx)
                        0:warp_y=16'd0; 1:warp_y=16'd28526; 2:warp_y=16'd35103; 3:warp_y=16'd39546;
                        4:warp_y=16'd43014; 5:warp_y=16'd45891; 6:warp_y=16'd48373; 7:warp_y=16'd50571;
                        8:warp_y=16'd52552; 9:warp_y=16'd54355; 10:warp_y=16'd56011; 11:warp_y=16'd57544;
                        12:warp_y=16'd58974; 13:warp_y=16'd60316; 14:warp_y=16'd61582; 15:warp_y=16'd62783;
                        default:warp_y=16'd65535;
                    endcase
                end
                4'd2: begin // 0.40
                    case (idx)
                        0:warp_y=16'd0; 1:warp_y=16'd21649; 2:warp_y=16'd28701; 3:warp_y=16'd33743;
                        4:warp_y=16'd37742; 5:warp_y=16'd41065; 6:warp_y=16'd43930; 7:warp_y=16'd46446;
                        8:warp_y=16'd48697; 9:warp_y=16'd50706; 10:warp_y=16'd52501; 11:warp_y=16'd54100;
                        12:warp_y=16'd55523; 13:warp_y=16'd56789; 14:warp_y=16'd57914; 15:warp_y=16'd58913;
                        default:warp_y=16'd65535;
                    endcase
                end
                4'd3: begin // 0.50
                    case (idx)
                        0:warp_y=16'd0; 1:warp_y=16'd16384; 2:warp_y=16'd23170; 3:warp_y=16'd28378;
                        4:warp_y=16'd32768; 5:warp_y=16'd36636; 6:warp_y=16'd40132; 7:warp_y=16'd43343;
                        8:warp_y=16'd46341; 9:warp_y=16'd49176; 10:warp_y=16'd51874; 11:warp_y=16'd54461;
                        12:warp_y=16'd56954; 13:warp_y=16'd59365; 14:warp_y=16'd61703; 15:warp_y=16'd63976;
                        default:warp_y=16'd65535;
                    endcase
                end
                4'd4: begin // 0.60
                    case (idx)
                        0:warp_y=16'd0; 1:warp_y=16'd12393; 2:warp_y=16'd18728; 3:warp_y=16'd23847;
                        4:warp_y=16'd28378; 5:warp_y=16'd32457; 6:warp_y=16'd36197; 7:warp_y=16'd39668;
                        8:warp_y=16'd42925; 9:warp_y=16'd46008; 10:warp_y=16'd48946; 11:warp_y=16'd51766;
                        12:warp_y=16'd54485; 13:warp_y=16'd57118; 14:warp_y=16'd59676; 15:warp_y=16'd62170;
                        default:warp_y=16'd65535;
                    endcase
                end
                4'd5: begin // 0.70
                    case (idx)
                        0:warp_y=16'd0; 1:warp_y=16'd9365; 2:warp_y=16'd15142; 3:warp_y=16'd20004;
                        4:warp_y=16'd24426; 5:warp_y=16'd28535; 6:warp_y=16'd32418; 7:warp_y=16'd36090;
                        8:warp_y=16'd39574; 9:warp_y=16'd42891; 10:warp_y=16'd46057; 11:warp_y=16'd49087;
                        12:warp_y=16'd51993; 13:warp_y=16'd54788; 14:warp_y=16'd57481; 15:warp_y=16'd60082;
                        default:warp_y=16'd65535;
                    endcase
                end
                4'd6: begin // 0.80
                    case (idx)
                        0:warp_y=16'd0; 1:warp_y=16'd7071; 2:warp_y=16'd12312; 3:warp_y=16'd16887;
                        4:warp_y=16'd21151; 5:warp_y=16'd25193; 6:warp_y=16'd29038; 7:warp_y=16'd32705;
                        8:warp_y=16'd36211; 9:warp_y=16'd39568; 10:warp_y=16'd42787; 11:warp_y=16'd45878;
                        12:warp_y=16'd48850; 13:warp_y=16'd51710; 14:warp_y=16'd54465; 15:warp_y=16'd57123;
                        default:warp_y=16'd65535;
                    endcase
                end
                4'd7: begin // 0.90
                    case (idx)
                        0:warp_y=16'd0; 1:warp_y=16'd5346; 2:warp_y=16'd10018; 3:warp_y=16'd14256;
                        4:warp_y=16'd18319; 5:warp_y=16'd22221; 6:warp_y=16'd25979; 7:warp_y=16'd29609;
                        8:warp_y=16'd33120; 9:warp_y=16'd36521; 10:warp_y=16'd39817; 11:warp_y=16'd43015;
                        12:warp_y=16'd46120; 13:warp_y=16'd49137; 14:warp_y=16'd52070; 15:warp_y=16'd54922;
                        default:warp_y=16'd65535;
                    endcase
                end
                4'd8: begin // 1.00 identity
                    case (idx)
                        0:warp_y=16'd0; 1:warp_y=16'd4096; 2:warp_y=16'd8192; 3:warp_y=16'd12288;
                        4:warp_y=16'd16384; 5:warp_y=16'd20480; 6:warp_y=16'd24576; 7:warp_y=16'd28672;
                        8:warp_y=16'd32768; 9:warp_y=16'd36864; 10:warp_y=16'd40960; 11:warp_y=16'd45056;
                        12:warp_y=16'd49152; 13:warp_y=16'd53248; 14:warp_y=16'd57344; 15:warp_y=16'd61440;
                        default:warp_y=16'd65535;
                    endcase
                end
                4'd9: begin // 1.10
                    case (idx)
                        0:warp_y=16'd0; 1:warp_y=16'd3137; 2:warp_y=16'd6880; 3:warp_y=16'd10855;
                        4:warp_y=16'd15045; 5:warp_y=16'd19395; 6:warp_y=16'd23870; 7:warp_y=16'd28447;
                        8:warp_y=16'd33109; 9:warp_y=16'd37842; 10:warp_y=16'd42636; 11:warp_y=16'd47483;
                        12:warp_y=16'd52377; 13:warp_y=16'd57313; 14:warp_y=16'd62287; 15:warp_y=16'd67295;
                        default:warp_y=16'd65535;
                    endcase
                end
                4'd10: begin // 1.20
                    case (idx)
                        0:warp_y=16'd0; 1:warp_y=16'd2404; 2:warp_y=16'd5789; 3:warp_y=16'd9586;
                        4:warp_y=16'd13727; 5:warp_y=16'd18172; 6:warp_y=16'd22892; 7:warp_y=16'd27862;
                        8:warp_y=16'd33060; 9:warp_y=16'd38466; 10:warp_y=16'd44064; 11:warp_y=16'd49838;
                        12:warp_y=16'd55776; 13:warp_y=16'd61865; 14:warp_y=16'd68097; 15:warp_y=16'd74462;
                        default:warp_y=16'd65535;
                    endcase
                end
                4'd11: begin // 1.30
                    case (idx)
                        0:warp_y=16'd0; 1:warp_y=16'd1843; 2:warp_y=16'd4877; 3:warp_y=16'd8429;
                        4:warp_y=16'd12453; 5:warp_y=16'd16912; 6:warp_y=16'd21769; 7:warp_y=16'd27002;
                        8:warp_y=16'd32592; 9:warp_y=16'd38523; 10:warp_y=16'd44779; 11:warp_y=16'd51348;
                        12:warp_y=16'd58217; 13:warp_y=16'd65376; 14:warp_y=16'd72815; 15:warp_y=16'd80524;
                        default:warp_y=16'd65535;
                    endcase
                end
                4'd12: begin // 1.40
                    case (idx)
                        0:warp_y=16'd0; 1:warp_y=16'd1413; 2:warp_y=16'd4108; 3:warp_y=16'd7411;
                        4:warp_y=16'd11291; 5:warp_y=16'd15693; 6:warp_y=16'd20573; 7:warp_y=16'd25900;
                        8:warp_y=16'd31651; 9:warp_y=16'd37808; 10:warp_y=16'd44355; 11:warp_y=16'd51282;
                        12:warp_y=16'd58580; 13:warp_y=16'd66245; 14:warp_y=16'd74273; 15:warp_y=16'd82662;
                        default:warp_y=16'd65535;
                    endcase
                end
                4'd13: begin // 1.50
                    case (idx)
                        0:warp_y=16'd0; 1:warp_y=16'd1085; 2:warp_y=16'd3465; 3:warp_y=16'd6497;
                        4:warp_y=16'd10175; 5:warp_y=16'd14503; 6:warp_y=16'd19487; 7:warp_y=16'd25137;
                        8:warp_y=16'd31467; 9:warp_y=16'd38500; 10:warp_y=16'd46263; 11:warp_y=16'd54788;
                        12:warp_y=16'd64113; 13:warp_y=16'd74281; 14:warp_y=16'd85338; 15:warp_y=16'd97339;
                        default:warp_y=16'd65535;
                    endcase
                end
                4'd14: begin // 1.60
                    case (idx)
                        0:warp_y=16'd0; 1:warp_y=16'd833; 2:warp_y=16'd2920; 3:warp_y=16'd5660;
                        4:warp_y=16'd9113; 5:warp_y=16'd13287; 6:warp_y=16'd18201; 7:warp_y=16'd23872;
                        8:warp_y=16'd30316; 9:warp_y=16'd37550; 10:warp_y=16'd45591; 11:warp_y=16'd54455;
                        12:warp_y=16'd64162; 13:warp_y=16'd74730; 14:warp_y=16'd86180; 15:warp_y=16'd98531;
                        default:warp_y=16'd65535;
                    endcase
                end
                4'd15: begin // 1.70
                    case (idx)
                        0:warp_y=16'd0; 1:warp_y=16'd588; 2:warp_y=16'd2440; 3:warp_y=16'd4948;
                        4:warp_y=16'd8171; 5:warp_y=16'd12167; 6:warp_y=16'd17006; 7:warp_y=16'd22773;
                        8:warp_y=16'd29532; 9:warp_y=16'd37329; 10:warp_y=16'd46180; 11:warp_y=16'd56101;
                        12:warp_y=16'd67107; 13:warp_y=16'd79209; 14:warp_y=16'd92417; 15:warp_y=16'd106739;
                        default:warp_y=16'd65535;
                    endcase
                end
                default: begin // Default to identity
                    case (idx)
                        0:warp_y=16'd0; 1:warp_y=16'd4096; 2:warp_y=16'd8192; 3:warp_y=16'd12288;
                        4:warp_y=16'd16384; 5:warp_y=16'd20480; 6:warp_y=16'd24576; 7:warp_y=16'd28672;
                        8:warp_y=16'd32768; 9:warp_y=16'd36864; 10:warp_y=16'd40960; 11:warp_y=16'd45056;
                        12:warp_y=16'd49152; 13:warp_y=16'd53248; 14:warp_y=16'd57344; 15:warp_y=16'd61440;
                        default:warp_y=16'd65535;
                    endcase
                end
            endcase
        end
    endfunction

    function [15:0] warp_u_q16;
        input [15:0] u_q16;
        input [3:0]  sel;
        reg [3:0]  seg;
        reg [11:0] frac;
        reg signed [17:0] y0, y1, dy;
        reg signed [31:0] interp;
        reg signed [17:0] tmp18;
        begin
            seg  = u_q16[15:12];
            frac = u_q16[11:0];

            y0 = {2'b00, warp_y(sel, {1'b0, seg})};
            y1 = {2'b00, warp_y(sel, {1'b0, seg} + 5'd1)};
            dy = y1 - y0;

            interp = dy * $signed({1'b0, frac});
            tmp18  = y0 + (interp >>> 12);

            warp_u_q16 = tmp18[15:0];
        end
    endfunction

    // atan lookup table for CORDIC in Q28
    function signed [31:0] atan_q28;
        input integer ii;
        begin
            case (ii)
                0:  atan_q28 = 32'sd210828714;
                1:  atan_q28 = 32'sd124459457;
                2:  atan_q28 = 32'sd65760959;
                3:  atan_q28 = 32'sd33381290;
                4:  atan_q28 = 32'sd16755422;
                5:  atan_q28 = 32'sd8385879;
                6:  atan_q28 = 32'sd4193963;
                7:  atan_q28 = 32'sd2097109;
                8:  atan_q28 = 32'sd1048571;
                9:  atan_q28 = 32'sd524287;
                10: atan_q28 = 32'sd262144;
                11: atan_q28 = 32'sd131072;
                12: atan_q28 = 32'sd65536;
                13: atan_q28 = 32'sd32768;
                14: atan_q28 = 32'sd16384;
                default: atan_q28 = 32'sd8192;
            endcase
        end
    endfunction

    // CORDIC for theta in [0, pi/2]
    // Input is Q28 angle, outputs are Q30 cosine and sine
    task cordic_sincos_0_pi2;
        input  signed [31:0] theta_q28;
        output signed [31:0] cos_q30;
        output signed [31:0] sin_q30;
        reg signed [31:0] x, y, z;
        reg signed [31:0] x_new, y_new, z_new;
        integer ii;
        begin
            x = CORDIC_KINV_Q30;
            y = 32'sd0;
            z = theta_q28;

            for (ii=0; ii<CORDIC_IT; ii=ii+1) begin
                if (z >= 0) begin
                    x_new = x - (y >>> ii);
                    y_new = y + (x >>> ii);
                    z_new = z - atan_q28(ii);
                end else begin
                    x_new = x + (y >>> ii);
                    y_new = y - (x >>> ii);
                    z_new = z + atan_q28(ii);
                end
                x = x_new; y = y_new; z = z_new;
            end
            cos_q30 = x;
            sin_q30 = y;
        end
    endtask

    // Convert stereo input to mono
    reg signed [24:0] mono_sum;
    reg signed [23:0] mono_in;
    always @* begin
        mono_sum = $signed({in_left[23], in_left}) + $signed({in_right[23], in_right});
        mono_in  = mono_sum >>> 1;
    end

    // Ring buffer and overlap-add storage
    reg signed [23:0] ring      [0:1023];
    reg signed [23:0] hop_buf_a [0:511];
    reg signed [23:0] hop_buf_b [0:511];
    reg signed [23:0] tail_buf  [0:511];

    reg [9:0] wptr;
    reg [8:0] hop_ctr;
    reg       hop_boundary;
    reg [9:0] frame_start;

    wire [9:0] wptr_inc = wptr + 10'd1;

    always @(posedge clk) begin
        if (!rst_n) begin
            wptr         <= 10'd0;
            hop_ctr      <= 9'd0;
            hop_boundary <= 1'b0;
            frame_start  <= 10'd0;
        end else begin
            hop_boundary <= 1'b0;

            if (cfg_change) begin
                wptr        <= 10'd0;
                hop_ctr     <= 9'd0;
                frame_start <= 10'd0;
            end else if (sample_strobe) begin
                ring[wptr] <= mono_in;
                wptr <= wptr_inc;

                if (hop_ctr == 9'd511) begin
                    hop_ctr      <= 9'd0;
                    hop_boundary <= 1'b1;
                    frame_start  <= wptr_inc;
                end else begin
                    hop_ctr <= hop_ctr + 9'd1;
                end
            end
        end
    end

    function [9:0] faddr;
        input integer n;
        reg [10:0] tmp;
        begin
            tmp   = {1'b0, frame_start} + n;
            faddr = tmp[9:0];
        end
    endfunction

    // Controls which processed hop is currently being read out
    reg        hop_sel;
    reg        have_buf;
    integer    hop_rd;

    reg        frame_done_pulse;
    reg        hop_ready_latched;

    always @(posedge clk) begin
        integer ii;
        if (!rst_n) begin
            hop_sel           <= 1'b0;
            have_buf          <= 1'b0;
            hop_rd            <= 0;
            hop_ready_latched <= 1'b0;
            for (ii=0;ii<512;ii=ii+1) tail_buf[ii] <= 24'sd0;
        end else if (cfg_change) begin
            hop_sel           <= 1'b0;
            have_buf          <= 1'b0;
            hop_rd            <= 0;
            hop_ready_latched <= 1'b0;
            for (ii=0;ii<512;ii=ii+1) tail_buf[ii] <= 24'sd0;
        end else begin
            if (frame_done_pulse)
                hop_ready_latched <= 1'b1;

            if (sample_strobe) begin
                if (hop_rd == 511) begin
                    hop_rd <= 0;
                    if (hop_ready_latched) begin
                        hop_sel <= ~hop_sel;
                        hop_ready_latched <= 1'b0;
                        have_buf <= 1'b1;
                    end
                end else begin
                    hop_rd <= hop_rd + 1;
                end
            end
        end
    end

    reg signed [23:0] hop_out;
    always @* begin
        if (!enable || !have_buf) hop_out = mono_in;
        else                      hop_out = (hop_sel == 1'b0) ? hop_buf_a[hop_rd] : hop_buf_b[hop_rd];
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            out_left  <= 24'sd0;
            out_right <= 24'sd0;
        end else if (sample_strobe) begin
            out_left  <= hop_out;
            out_right <= hop_out;
        end
    end

    // LPC, LSF, and synthesis state
    reg signed [63:0] r [0:10];

    reg signed [31:0] a_prev  [0:10];  // LPC coefficients from Levinson-Durbin
    reg signed [31:0] a_new   [0:10];

    // Filter state carried across frames
    reg signed [23:0] x_state [1:10];
    reg signed [23:0] y_state [1:10];

    // Working histories used during synthesis
    reg signed [23:0] x_hist [1:10];
    reg signed [23:0] y_hist [1:10];

    reg signed [63:0] E;

    // Levinson-Durbin control
    reg [3:0]  ld_i;
    reg [3:0]  ld_j;
    reg signed [63:0] ld_sum;

    // Divider for |ld_sum| / E in Q30
    reg        div_run;
    reg [5:0]  div_cnt;
    reg [63:0] div_rem;
    reg [63:0] div_den;
    reg [29:0] div_q;
    reg        div_sat;
    reg signed [31:0] kappa_q30;

    // Autocorrelation control
    reg [3:0]  ac_lag;
    reg [9:0]  ac_n;

    // LSF angles in Q28 and their P/Q family type
    reg [31:0] lsf_omega_q28   [0:9];
    reg        lsf_type        [0:9];
    reg [31:0] lsf_warp_q28    [0:9];

    reg [3:0]  roots_found;
    reg        lsf_done;

    // Root scan state
    reg [8:0]  grid_idx;
    reg [31:0] omega_prev_q28, omega_next_q28;

    reg signed [31:0] P_prev, Q_prev;
    reg signed [31:0] P_next, Q_next;

    // Bisection state
    reg        bisect_func; // 0 = P, 1 = Q
    reg [31:0] bis_lo, bis_hi;
    reg signed [31:0] f_lo;
    reg [3:0]  bis_it;

    // Holds a second root if both P and Q flip in the same interval
    reg        pend_valid;
    reg        pend_func;
    reg [31:0] pend_lo, pend_hi;
    reg signed [31:0] pend_flo;

    // LSF to LPC polynomial build state
    reg signed [31:0] polyP     [0:10];
    reg signed [31:0] polyQ     [0:10];
    reg signed [31:0] polyTmp   [0:10];
    reg [3:0]         l2a_idx;

    // Warped LPC coefficients
    reg signed [31:0] a_mod     [0:10];

    // Synthesis control
    reg [9:0]  syn_n;
    reg [3:0]  k_idx;
    reg signed [23:0] cur_x;
    reg signed [63:0] acc_res;
    reg signed [63:0] acc_syn;

    reg fill_sel;

    // Debug latches
    reg frame_toggle;
    reg fallback_used;

    // FSM states
    localparam [5:0]
        S_IDLE        = 6'd0,
        S_AC_INIT     = 6'd1,
        S_AC_RUN      = 6'd2,
        S_LD_INIT     = 6'd3,
        S_LD_SUM_INIT = 6'd4,
        S_LD_SUM_MAC  = 6'd5,
        S_LD_DIV_PRE  = 6'd6,
        S_LD_DIV_RUN  = 6'd7,
        S_LD_UPD_INIT = 6'd8,
        S_LD_UPD_MAC  = 6'd9,
        S_LD_EUPD     = 6'd10,
        S_LD_NEXT     = 6'd11,

        S_LSF_INIT    = 6'd12,
        S_LSF_SCAN    = 6'd13,
        S_LSF_BISECT  = 6'd14,
        S_LSF_STORE   = 6'd15,

        S_WARP_INIT   = 6'd16,
        S_WARP_STEP   = 6'd17,

        S_L2A_INIT    = 6'd18,
        S_L2A_STEP    = 6'd19,
        S_L2A_COMBINE = 6'd20,

        S_SYN_INIT    = 6'd21,
        S_SYN_LOAD    = 6'd22,
        S_RES_INIT    = 6'd23,
        S_RES_MAC     = 6'd24,
        S_SYN_INIT2   = 6'd25,
        S_SYN_MAC     = 6'd26,
        S_SYN_WRITE   = 6'd27,
        S_SAVE_STATE  = 6'd28,
        S_DONE        = 6'd29;

    reg [5:0] st;
    assign dbg_busy = (st != S_IDLE);

    // Stage flags for debug
    wire in_AC   = (st == S_AC_INIT)  || (st == S_AC_RUN);
    wire in_LD   = (st >= S_LD_INIT)  && (st <= S_LD_NEXT);
    wire in_LSF  = (st >= S_LSF_INIT) && (st <= S_LSF_STORE);
    wire in_WARP = (st == S_WARP_INIT) || (st == S_WARP_STEP);
    wire in_L2A  = (st >= S_L2A_INIT) && (st <= S_L2A_COMBINE);
    wire in_SYN  = (st >= S_SYN_INIT) && (st <= S_DONE);

    wire roots_ok = (roots_found == 4'd10);

    // Pack status bits into the debug bus
    assign dbg_rootcount = {
        dbg_residual,     // [15]
        have_buf,         // [14]
        lsf_done,         // [13]
        roots_ok,         // [12]
        roots_found,      // [11:8]
        in_AC,            // [7]
        in_LD,            // [6]
        in_LSF,           // [5]
        in_WARP,          // [4]
        in_L2A,           // [3]
        in_SYN,           // [2]
        frame_toggle,     // [1]
        fallback_used     // [0]
    };

    // Evaluate P(omega) and Q(omega) in Q30
    // Uses D(omega)=A(e^{jω})*e^{j(P+1)ω/2}
    task eval_PQ;
        input  [31:0] omega_q28;
        output signed [31:0] P_out;
        output signed [31:0] Q_out;
        reg signed [31:0] theta_q28;
        reg signed [31:0] cth_q30, sth_q30;

        reg signed [63:0] c2, s2;
        reg signed [31:0] cw_q30, sw_q30;

        reg signed [31:0] cos_k, sin_k;
        reg signed [31:0] cos_n, sin_n;

        reg signed [31:0] cos_m, sin_m;
        reg signed [31:0] cos_mn, sin_mn;

        reg signed [63:0] sumRe, sumS;
        reg signed [63:0] prod;
        integer k;
        integer m;

        reg signed [31:0] c11_q30, s11_q30;
        reg signed [63:0] t1, t2, t3, t4;
        begin
            theta_q28 = $signed({1'b0, omega_q28[31:1]}); // omega/2

            cordic_sincos_0_pi2(theta_q28, cth_q30, sth_q30);

            c2 = $signed(cth_q30) * $signed(cth_q30);
            s2 = $signed(sth_q30) * $signed(sth_q30);

            cw_q30 = $signed((c2 - s2) >>> 30);

            prod   = $signed(sth_q30) * $signed(cth_q30);
            sw_q30 = $signed((prod >>> 29)); // 2*sin(theta)*cos(theta)

            sumRe = 64'sd0;
            sumS  = 64'sd0;

            cos_k = (32'sd1 <<< COEF_FRAC);
            sin_k = 32'sd0;

            for (k=0; k<=P; k=k+1) begin
                prod  = ($signed(a_prev[k]) * $signed(cos_k)) >>> COEF_FRAC;
                sumRe = sumRe + prod;

                prod  = ($signed(a_prev[k]) * $signed(sin_k)) >>> COEF_FRAC;
                sumS  = sumS + prod;

                if (k != P) begin
                    t1 = ($signed(cos_k) * $signed(cw_q30)) >>> COEF_FRAC;
                    t2 = ($signed(sin_k) * $signed(sw_q30)) >>> COEF_FRAC;
                    cos_n = $signed(t1 - t2);

                    t3 = ($signed(sin_k) * $signed(cw_q30)) >>> COEF_FRAC;
                    t4 = ($signed(cos_k) * $signed(sw_q30)) >>> COEF_FRAC;
                    sin_n = $signed(t3 + t4);

                    cos_k = cos_n;
                    sin_k = sin_n;
                end
            end

            cos_m = (32'sd1 <<< COEF_FRAC);
            sin_m = 32'sd0;
            for (m=0; m<11; m=m+1) begin
                t1 = ($signed(cos_m) * $signed(cth_q30)) >>> COEF_FRAC;
                t2 = ($signed(sin_m) * $signed(sth_q30)) >>> COEF_FRAC;
                cos_mn = $signed(t1 - t2);

                t3 = ($signed(sin_m) * $signed(cth_q30)) >>> COEF_FRAC;
                t4 = ($signed(cos_m) * $signed(sth_q30)) >>> COEF_FRAC;
                sin_mn = $signed(t3 + t4);

                cos_m = cos_mn;
                sin_m = sin_mn;
            end
            c11_q30 = cos_m;
            s11_q30 = sin_m;

            t1 = ($signed(sumRe) * $signed(c11_q30)) >>> COEF_FRAC;
            t2 = ($signed(sumS)  * $signed(s11_q30)) >>> COEF_FRAC;
            t3 = ($signed(sumRe) * $signed(s11_q30)) >>> COEF_FRAC;
            t4 = ($signed(sumS)  * $signed(c11_q30)) >>> COEF_FRAC;

            P_out = $signed(t1 + t2);
            Q_out = $signed(t3 - t4);
        end
    endtask

    // Main processing FSM
    integer ii;
    always @(posedge clk) begin
        // Common temporary values
        reg signed [63:0] prod;
        reg signed [63:0] term;
        reg signed [63:0] k2;
        reg signed [63:0] one_minus_k2;

        // Divider temporaries
        reg [63:0] rem2;
        reg        qbit;
        reg [29:0] q_next;

        // Bisection temporaries
        reg [31:0] mid;
        reg signed [31:0] f_mid;
        reg signed [31:0] p_eval, q_eval;

        // Root-scan decision flags
        reg p_change, q_change;

        // Warp temporaries
        reg signed [63:0] mult64;
        reg [15:0] u_q16;
        reg [15:0] up_q16;
        reg [31:0] omega_w;

        // LSF to LPC temporaries
        reg signed [31:0] cth_q30, sth_q30;
        reg signed [31:0] theta_q28;
        reg signed [63:0] c2, s2;
        reg signed [31:0] cosw_q30;
        reg signed [31:0] b1_q30;
        reg signed [63:0] acc;

        // Synthesis temporaries
        reg signed [23:0] syn_samp;
        reg signed [23:0] res_samp;
        reg signed [23:0] use_samp;
        reg signed [63:0] mix_acc;
        reg [15:0] w_new;
        reg [15:0] w_old;

        frame_done_pulse <= 1'b0;

        if (!rst_n) begin
            st <= S_IDLE;

            for (ii=0;ii<=P;ii=ii+1) begin
                r[ii]      <= 64'sd0;
                a_prev[ii] <= (ii==0) ? (32'sd1<<<COEF_FRAC) : 32'sd0;
                a_new[ii]  <= (ii==0) ? (32'sd1<<<COEF_FRAC) : 32'sd0;
                a_mod[ii]  <= (ii==0) ? (32'sd1<<<COEF_FRAC) : 32'sd0;
                polyP[ii]  <= 32'sd0;
                polyQ[ii]  <= 32'sd0;
                polyTmp[ii]<= 32'sd0;
            end
            for (ii=1;ii<=P;ii=ii+1) begin
                x_hist[ii] <= 24'sd0;
                y_hist[ii] <= 24'sd0;
                x_state[ii]<= 24'sd0;
                y_state[ii]<= 24'sd0;
            end
            for (ii=0;ii<10;ii=ii+1) begin
                lsf_omega_q28[ii] <= 32'd0;
                lsf_warp_q28[ii]  <= 32'd0;
                lsf_type[ii]      <= 1'b0;
            end

            E <= 64'sd0;

            ac_lag <= 0;
            ac_n   <= 0;

            ld_i   <= 1;
            ld_j   <= 1;
            ld_sum <= 0;

            div_run <= 1'b0;
            div_cnt <= 0;
            div_rem <= 0;
            div_den <= 0;
            div_q   <= 0;
            div_sat <= 1'b0;
            kappa_q30 <= 32'sd0;

            roots_found <= 0;
            lsf_done    <= 1'b0;
            grid_idx    <= 0;
            omega_prev_q28 <= 32'd0;
            omega_next_q28 <= 32'd0;
            P_prev <= 0; Q_prev <= 0; P_next <= 0; Q_next <= 0;

            bisect_func <= 1'b0;
            bis_lo <= 0; bis_hi <= 0; f_lo <= 0; bis_it <= 0;
            pend_valid <= 1'b0;

            l2a_idx <= 0;

            syn_n <= 0;
            k_idx <= 1;
            cur_x <= 0;
            acc_res <= 0;
            acc_syn <= 0;

            fill_sel <= 1'b0;

            frame_toggle <= 1'b0;
            fallback_used <= 1'b0;

        end else if (cfg_change) begin
            st <= S_IDLE;

            for (ii=1;ii<=P;ii=ii+1) begin
                x_state[ii] <= 24'sd0;
                y_state[ii] <= 24'sd0;
            end

            roots_found <= 0;
            lsf_done    <= 1'b0;
            pend_valid  <= 1'b0;

            fallback_used <= 1'b0;

        end else begin
            if (st == S_IDLE) begin
                if (enable && hop_boundary) begin
                    fill_sel <= ~hop_sel;
                    st <= S_AC_INIT;
                end
            end else begin
                case (st)

                    // Compute autocorrelation r[0] through r[P]
                    S_AC_INIT: begin
                        for (ii=0;ii<=P;ii=ii+1) r[ii] <= 64'sd0;
                        ac_lag <= 0;
                        ac_n   <= 0;
                        st     <= S_AC_RUN;
                    end

                    S_AC_RUN: begin
                        if (ac_n < ac_lag) ac_n <= ac_lag;

                        prod = $signed(ring[faddr(ac_n)]) * $signed(ring[faddr(ac_n - ac_lag)]);
                        r[ac_lag] <= r[ac_lag] + prod;

                        if (ac_n == 10'd1023) begin
                            if (ac_lag == 4'd10) begin
                                st <= S_LD_INIT;
                            end else begin
                                ac_lag <= ac_lag + 1;
                                ac_n   <= 0;
                            end
                        end else begin
                            ac_n <= ac_n + 1;
                        end
                    end

                    // Run Levinson-Durbin to get LPC coefficients
                    S_LD_INIT: begin
                        a_prev[0] <= (32'sd1<<<COEF_FRAC);
                        for (ii=1;ii<=P;ii=ii+1) a_prev[ii] <= 32'sd0;

                        a_new[0] <= (32'sd1<<<COEF_FRAC);
                        for (ii=1;ii<=P;ii=ii+1) a_new[ii] <= 32'sd0;

                        E    <= r[0];
                        ld_i <= 4'd1;
                        st   <= S_LD_SUM_INIT;
                    end

                    S_LD_SUM_INIT: begin
                        ld_sum <= r[ld_i];
                        ld_j   <= 4'd1;
                        if (ld_i == 4'd1) st <= S_LD_DIV_PRE;
                        else              st <= S_LD_SUM_MAC;
                    end

                    S_LD_SUM_MAC: begin
                        term   = ($signed(a_prev[ld_j]) * $signed(r[ld_i - ld_j])) >>> COEF_FRAC;
                        ld_sum <= ld_sum + term;

                        if (ld_j == (ld_i - 1)) st <= S_LD_DIV_PRE;
                        else                    ld_j <= ld_j + 1;
                    end

                    S_LD_DIV_PRE: begin
                        div_run <= 1'b0;
                        div_cnt <= 0;
                        div_q   <= 30'd0;

                        if (E <= 0) begin
                            kappa_q30 <= 32'sd0;
                            st <= S_LD_UPD_INIT;
                        end else begin
                            if (abs64(ld_sum) >= abs64(E)) begin
                                div_sat <= 1'b1;
                                st <= S_LD_DIV_RUN;
                            end else begin
                                div_sat <= 1'b0;
                                div_rem <= abs64(ld_sum);
                                div_den <= abs64(E);
                                div_cnt <= 0;
                                div_q   <= 30'd0;
                                div_run <= 1'b1;
                                st <= S_LD_DIV_RUN;
                            end
                        end
                    end

                    S_LD_DIV_RUN: begin
                        if (div_sat) begin
                            if (ld_sum[63]) kappa_q30 <= 32'sd1073741823;
                            else            kappa_q30 <= -32'sd1073741823;
                            st <= S_LD_UPD_INIT;
                            div_run <= 1'b0;
                        end else if (div_run) begin
                            rem2 = (div_rem << 1);
                            if (rem2 >= div_den) begin
                                rem2 = rem2 - div_den;
                                qbit = 1'b1;
                            end else begin
                                qbit = 1'b0;
                            end
                            q_next = {div_q[28:0], qbit};

                            div_rem <= rem2;
                            div_q   <= q_next;

                            if (div_cnt == 6'd29) begin
                                if (ld_sum[63]) kappa_q30 <= $signed({2'b00, q_next});
                                else            kappa_q30 <= -$signed({2'b00, q_next});
                                st <= S_LD_UPD_INIT;
                                div_run <= 1'b0;
                            end else begin
                                div_cnt <= div_cnt + 1;
                            end
                        end else begin
                            st <= S_LD_UPD_INIT;
                        end
                    end

                    S_LD_UPD_INIT: begin
                        a_new[0] <= (32'sd1<<<COEF_FRAC);
                        for (ii=1;ii<=P;ii=ii+1) a_new[ii] <= 32'sd0;

                        ld_j <= 4'd1;
                        st <= S_LD_UPD_MAC;
                    end

                    S_LD_UPD_MAC: begin
                        if (ld_i == 4'd1) begin
                            a_new[1] <= kappa_q30;
                            st <= S_LD_EUPD;
                        end else begin
                            if (ld_j < ld_i) begin
                                term = ($signed(kappa_q30) * $signed(a_prev[ld_i - ld_j])) >>> COEF_FRAC;
                                a_new[ld_j] <= a_prev[ld_j] + $signed(term[31:0]);

                                if (ld_j == (ld_i - 1)) begin
                                    a_new[ld_i] <= kappa_q30;
                                    st <= S_LD_EUPD;
                                end else begin
                                    ld_j <= ld_j + 1;
                                end
                            end else begin
                                a_new[ld_i] <= kappa_q30;
                                st <= S_LD_EUPD;
                            end
                        end
                    end

                    S_LD_EUPD: begin
                        k2 = ($signed(kappa_q30) * $signed(kappa_q30)) >>> COEF_FRAC;
                        one_minus_k2 = (64'sd1<<<COEF_FRAC) - k2;
                        E <= ($signed(E) * $signed(one_minus_k2)) >>> COEF_FRAC;
                        st <= S_LD_NEXT;
                    end

                    S_LD_NEXT: begin
                        for (ii=0;ii<=P;ii=ii+1) a_prev[ii] <= a_new[ii];

                        if (ld_i == 4'd10) st <= S_LSF_INIT;
                        else begin
                            ld_i <= ld_i + 1;
                            st <= S_LD_SUM_INIT;
                        end
                    end

                    // Find LSF roots from the LPC polynomial
                    S_LSF_INIT: begin
                        roots_found <= 4'd0;
                        lsf_done    <= 1'b0;
                        pend_valid  <= 1'b0;

                        grid_idx       <= 9'd1;
                        omega_prev_q28 <= STEP_Q28;
                        omega_next_q28 <= STEP_Q28 + STEP_Q28;

                        eval_PQ(STEP_Q28, P_prev, Q_prev);
                        eval_PQ(STEP_Q28 + STEP_Q28, P_next, Q_next);

                        st <= S_LSF_SCAN;
                    end

                    S_LSF_SCAN: begin
                        p_change = 1'b0;
                        q_change = 1'b0;

                        if ((P_prev == 0) || (P_next == 0) || (P_prev[31] != P_next[31])) p_change = 1'b1;
                        if ((Q_prev == 0) || (Q_next == 0) || (Q_prev[31] != Q_next[31])) q_change = 1'b1;

                        if ((roots_found >= P) || (grid_idx >= (GRID_N-1))) begin
                            lsf_done <= 1'b1;
                            st <= S_WARP_INIT;
                        end else if (p_change || q_change) begin
                            if (p_change) begin
                                bisect_func <= 1'b0;
                                bis_lo <= omega_prev_q28;
                                bis_hi <= omega_next_q28;
                                f_lo   <= P_prev;
                                bis_it <= 4'd0;

                                if (q_change) begin
                                    pend_valid <= 1'b1;
                                    pend_func  <= 1'b1;
                                    pend_lo    <= omega_prev_q28;
                                    pend_hi    <= omega_next_q28;
                                    pend_flo   <= Q_prev;
                                end else begin
                                    pend_valid <= 1'b0;
                                end
                                st <= S_LSF_BISECT;
                            end else begin
                                bisect_func <= 1'b1;
                                bis_lo <= omega_prev_q28;
                                bis_hi <= omega_next_q28;
                                f_lo   <= Q_prev;
                                bis_it <= 4'd0;
                                pend_valid <= 1'b0;
                                st <= S_LSF_BISECT;
                            end
                        end else begin
                            omega_prev_q28 <= omega_next_q28;
                            P_prev <= P_next;
                            Q_prev <= Q_next;

                            omega_next_q28 <= omega_next_q28 + STEP_Q28;

                            eval_PQ(omega_next_q28 + STEP_Q28, P_next, Q_next);
                            grid_idx <= grid_idx + 1;
                        end
                    end

                    S_LSF_BISECT: begin
                        mid = (bis_lo + bis_hi) >> 1;

                        eval_PQ(mid, p_eval, q_eval);
                        f_mid = (bisect_func == 1'b0) ? p_eval : q_eval;

                        if ((f_mid == 0) || (f_mid[31] != f_lo[31])) begin
                            bis_hi <= mid;
                        end else begin
                            bis_lo <= mid;
                            f_lo   <= f_mid;
                        end

                        if (bis_it == (BISECT_IT-1)) begin
                            st <= S_LSF_STORE;
                        end else begin
                            bis_it <= bis_it + 1;
                        end
                    end

                    S_LSF_STORE: begin
                        reg [31:0] root;
                        root = (bis_lo + bis_hi) >> 1;

                        if (roots_found < P) begin
                            lsf_omega_q28[roots_found] <= root;
                            lsf_type[roots_found]      <= bisect_func;
                            roots_found <= roots_found + 1;
                        end

                        if (pend_valid) begin
                            bisect_func <= pend_func;
                            bis_lo <= pend_lo;
                            bis_hi <= pend_hi;
                            f_lo   <= pend_flo;
                            bis_it <= 4'd0;
                            pend_valid <= 1'b0;
                            st <= S_LSF_BISECT;
                        end else begin
                            omega_prev_q28 <= omega_next_q28;
                            P_prev <= P_next;
                            Q_prev <= Q_next;

                            omega_next_q28 <= omega_next_q28 + STEP_Q28;
                            eval_PQ(omega_next_q28 + STEP_Q28, P_next, Q_next);

                            grid_idx <= grid_idx + 1;
                            st <= S_LSF_SCAN;
                        end
                    end

                    // Warp the LSF angles using the selected alpha
                    S_WARP_INIT: begin
                        l2a_idx <= 4'd0;
                        st <= S_WARP_STEP;
                    end

                    S_WARP_STEP: begin
                        if (l2a_idx == P) begin
                            st <= S_L2A_INIT;
                        end else begin
                            // Residual mode and alpha = 1.00 both use the original angles
                            if ((alpha_sel == 4'd0) || (alpha_sel == 4'd8)) begin
                                lsf_warp_q28[l2a_idx] <= lsf_omega_q28[l2a_idx];
                            end else begin
                                // Convert omega to normalized u = omega/pi in Q0.16
                                mult64 = $signed({1'b0, lsf_omega_q28[l2a_idx]}) * $signed(INV_PI_Q30);
                                u_q16  = mult64[57:42];

                                // Warp using u^alpha
                                up_q16 = warp_u_q16(u_q16, alpha_sel);

                                // Convert back to omega
                                mult64  = $signed({1'b0, up_q16}) * $signed(PI_Q28);
                                omega_w = mult64[43:16];

                                // Keep omega safely inside the search range
                                if ($signed(omega_w) < $signed(STEP_Q28)) omega_w = STEP_Q28;
                                if ($signed(omega_w) > $signed(PI_Q28 - STEP_Q28)) omega_w = (PI_Q28 - STEP_Q28);

                                lsf_warp_q28[l2a_idx] <= omega_w;
                            end
                            l2a_idx <= l2a_idx + 1;
                        end
                    end

                    // Convert warped LSFs back into LPC coefficients
                    S_L2A_INIT: begin
                        if (roots_found != P) begin
                            for (ii=0;ii<=P;ii=ii+1) a_mod[ii] <= a_prev[ii];
                            fallback_used <= 1'b1;
                            st <= S_SYN_INIT;
                        end else begin
                            fallback_used <= 1'b0;

                            for (ii=0;ii<=P;ii=ii+1) begin
                                polyP[ii] <= 32'sd0;
                                polyQ[ii] <= 32'sd0;
                            end
                            polyP[0] <= (32'sd1<<<COEF_FRAC);
                            polyQ[0] <= (32'sd1<<<COEF_FRAC);

                            l2a_idx <= 4'd0;
                            st <= S_L2A_STEP;
                        end
                    end

                    S_L2A_STEP: begin
                        if (l2a_idx == P) begin
                            st <= S_L2A_COMBINE;
                        end else begin
                            theta_q28 = $signed({1'b0, lsf_warp_q28[l2a_idx][31:1]}); // w/2
                            cordic_sincos_0_pi2(theta_q28, cth_q30, sth_q30);

                            c2 = $signed(cth_q30) * $signed(cth_q30);
                            s2 = $signed(sth_q30) * $signed(sth_q30);
                            cosw_q30 = $signed((c2 - s2) >>> 30);

                            b1_q30 = -$signed({cosw_q30[30:0], 1'b0}); // -2cos(w)

                            if (lsf_type[l2a_idx] == 1'b0) begin
                                for (ii=0;ii<=P;ii=ii+1) polyTmp[ii] = 32'sd0;
                                for (ii=0;ii<=P;ii=ii+1) begin
                                    acc = 64'sd0;
                                    acc = acc + $signed(polyP[ii]);
                                    if (ii >= 1) begin
                                        term = ($signed(b1_q30) * $signed(polyP[ii-1])) >>> COEF_FRAC;
                                        acc = acc + term;
                                    end
                                    if (ii >= 2) begin
                                        acc = acc + $signed(polyP[ii-2]);
                                    end
                                    polyTmp[ii] = acc[31:0];
                                end
                                for (ii=0;ii<=P;ii=ii+1) polyP[ii] <= polyTmp[ii];
                            end else begin
                                for (ii=0;ii<=P;ii=ii+1) polyTmp[ii] = 32'sd0;
                                for (ii=0;ii<=P;ii=ii+1) begin
                                    acc = 64'sd0;
                                    acc = acc + $signed(polyQ[ii]);
                                    if (ii >= 1) begin
                                        term = ($signed(b1_q30) * $signed(polyQ[ii-1])) >>> COEF_FRAC;
                                        acc = acc + term;
                                    end
                                    if (ii >= 2) begin
                                        acc = acc + $signed(polyQ[ii-2]);
                                    end
                                    polyTmp[ii] = acc[31:0];
                                end
                                for (ii=0;ii<=P;ii=ii+1) polyQ[ii] <= polyTmp[ii];
                            end

                            l2a_idx <= l2a_idx + 1;
                        end
                    end

                    S_L2A_COMBINE: begin
                        reg signed [63:0] p1, q1, s;
                        for (ii=0;ii<=P;ii=ii+1) begin
                            p1 = $signed(polyP[ii]);
                            if (ii >= 1) p1 = p1 + $signed(polyP[ii-1]);

                            q1 = $signed(polyQ[ii]);
                            if (ii >= 1) q1 = q1 - $signed(polyQ[ii-1]);

                            s  = p1 + q1;
                            a_mod[ii] <= $signed(s >>> 1);
                        end
                        a_mod[0] <= (32'sd1<<<COEF_FRAC);
                        st <= S_SYN_INIT;
                    end

                    // Synthesize the new frame and overlap-add it into the hop buffer
                    S_SYN_INIT: begin
                        syn_n <= 10'd0;

                        for (ii=1;ii<=P;ii=ii+1) begin
                            x_hist[ii] <= x_state[ii];
                            y_hist[ii] <= y_state[ii];
                        end
                        st <= S_SYN_LOAD;
                    end

                    S_SYN_LOAD: begin
                        cur_x <= ring[faddr(syn_n)];
                        st <= S_RES_INIT;
                    end

                    S_RES_INIT: begin
                        acc_res <= $signed(cur_x);
                        k_idx <= 4'd1;
                        st <= S_RES_MAC;
                    end

                    S_RES_MAC: begin
                        term = ($signed(a_prev[k_idx]) * $signed(x_hist[k_idx])) >>> COEF_FRAC;
                        acc_res <= acc_res + term;

                        if (k_idx == 4'd10) st <= S_SYN_INIT2;
                        else                k_idx <= k_idx + 1;
                    end

                    S_SYN_INIT2: begin
                        acc_syn <= acc_res;
                        k_idx <= 4'd1;
                        st <= S_SYN_MAC;
                    end

                    S_SYN_MAC: begin
                        if (dbg_residual)
                            term = ($signed(a_prev[k_idx]) * $signed(y_hist[k_idx])) >>> COEF_FRAC;
                        else
                            term = ($signed(a_mod[k_idx])  * $signed(y_hist[k_idx])) >>> COEF_FRAC;

                        acc_syn <= acc_syn - term;

                        if (k_idx == 4'd10) st <= S_SYN_WRITE;
                        else                k_idx <= k_idx + 1;
                    end

                    S_SYN_WRITE: begin
                        syn_samp = sat24(acc_syn);
                        res_samp = sat24(acc_res);

                        if (dbg_residual)
                            use_samp = sat24($signed({{40{res_samp[23]}},res_samp}) <<< 1);
                        else
                            use_samp = syn_samp;

                        for (ii=P; ii>1; ii=ii-1) begin
                            x_hist[ii] <= x_hist[ii-1];
                            y_hist[ii] <= y_hist[ii-1];
                        end
                        x_hist[1] <= cur_x;
                        y_hist[1] <= syn_samp;

                        if (syn_n < 10'd512) begin
                            w_new = {syn_n[8:0], 7'b0000000};
                            w_old = 16'hFFFF - w_new;

                            mix_acc = ($signed(tail_buf[syn_n]) * $signed({1'b0, w_old})) +
                                      ($signed(use_samp)         * $signed({1'b0, w_new}));

                            if (fill_sel == 1'b0) hop_buf_a[syn_n] <= sat24(mix_acc >>> 16);
                            else                  hop_buf_b[syn_n] <= sat24(mix_acc >>> 16);
                        end else begin
                            tail_buf[syn_n - 10'd512] <= use_samp;
                        end

                        if (syn_n == 10'd1023) begin
                            frame_done_pulse <= 1'b1;
                            frame_toggle <= ~frame_toggle;
                            st <= S_SAVE_STATE;
                        end else begin
                            syn_n <= syn_n + 1;
                            st <= S_SYN_LOAD;
                        end
                    end

                    S_SAVE_STATE: begin
                        for (ii=1;ii<=P;ii=ii+1) begin
                            x_state[ii] <= x_hist[ii];
                            y_state[ii] <= y_hist[ii];
                        end
                        st <= S_DONE;
                    end

                    S_DONE: begin
                        st <= S_IDLE;
                    end

                    default: st <= S_IDLE;

                endcase
            end
        end
    end

endmodule