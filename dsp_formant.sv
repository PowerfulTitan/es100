module dsp_formant #(
    parameter int N_STAGES        = 6,
    parameter int SETTLE_SAMPLES  = 512  // ~10.7 ms at 48k
) (
    input  logic                 clk,
    input  logic                 rst_n,
    input  logic                 sample_strobe,

    input  logic signed [23:0]   in_left,
    input  logic signed [23:0]   in_right,

    input  logic signed [3:0]    step,       // [-7..+7]
    input  logic                 enable,

    input  logic                 cfg_change, // Pulse high when controls change

    output logic signed [23:0]   out_left,
    output logic signed [23:0]   out_right
);

    localparam int INT_W = 28;
    localparam int SET_W = $clog2(SETTLE_SAMPLES+1);

    function automatic logic signed [23:0] sat24(input logic signed [31:0] x);
        if (x > 32'sd8388607)        return 24'sd8388607;
        else if (x < -32'sd8388608)  return -24'sd8388608;
        else                         return x[23:0];
    endfunction

    function automatic logic signed [INT_W-1:0] sat_int(input logic signed [31:0] x);
        localparam logic signed [31:0] MAXI = (32'sd1 <<< (INT_W-1)) - 32'sd1;
        localparam logic signed [31:0] MINI = - (32'sd1 <<< (INT_W-1));
        if (x > MAXI)        return MAXI[INT_W-1:0];
        else if (x < MINI)   return MINI[INT_W-1:0];
        else                 return x[INT_W-1:0];
    endfunction

    function automatic logic [3:0] abs_step(input logic signed [3:0] s);
        return s[3] ? logic'(-s) : logic'(s);
    endfunction

    // Maps the user step setting into the all-pass coefficient
    // Clamp at +/-0.5 to keep the filter stable
    logic signed [15:0] lambda_q15;
    always_comb begin
        lambda_q15 = 16'sd0;
        unique case (step)
            4'sd1:  lambda_q15 = 16'sd3277;   // 0.10
            4'sd2:  lambda_q15 = 16'sd6554;   // 0.20
            4'sd3:  lambda_q15 = 16'sd9830;   // 0.30
            4'sd4:  lambda_q15 = 16'sd13107;  // 0.40
            4'sd5:  lambda_q15 = 16'sd16384;  // 0.50
            4'sd6:  lambda_q15 = 16'sd16384;
            4'sd7:  lambda_q15 = 16'sd16384;

            -4'sd1: lambda_q15 = -16'sd3277;
            -4'sd2: lambda_q15 = -16'sd6554;
            -4'sd3: lambda_q15 = -16'sd9830;
            -4'sd4: lambda_q15 = -16'sd13107;
            -4'sd5: lambda_q15 = -16'sd16384;
            -4'sd6: lambda_q15 = -16'sd16384;
            -4'sd7: lambda_q15 = -16'sd16384;

            default: lambda_q15 = 16'sd0;
        endcase

        if (!enable || (step == 0))
            lambda_q15 = 16'sd0;
    end

    // Reduce input level slightly to leave headroom inside the filter
    logic signed [INT_W-1:0] xL0, xR0;
    always_comb begin
        xL0 = $signed({{(INT_W-24){in_left[23]}},  in_left})  >>> 1;
        xR0 = $signed({{(INT_W-24){in_right[23]}}, in_right}) >>> 1;
    end

    // Per-stage filter state for left and right channels
    logic signed [INT_W-1:0] xprev_l [N_STAGES];
    logic signed [INT_W-1:0] yprev_l [N_STAGES];
    logic signed [INT_W-1:0] xprev_r [N_STAGES];
    logic signed [INT_W-1:0] yprev_r [N_STAGES];

    integer i;

    function automatic logic signed [INT_W-1:0] allpass_step_int(
        input logic signed [INT_W-1:0] x,
        input logic signed [INT_W-1:0] x_prev,
        input logic signed [INT_W-1:0] y_prev,
        input logic signed [15:0]      lam_q15
    );
        logic signed [47:0] m1, m2;
        logic signed [31:0] y32;
        begin
            m1 = $signed(lam_q15) * $signed(x);
            m2 = $signed(lam_q15) * $signed(y_prev);

            y32 = $signed({{(32-INT_W){x_prev[INT_W-1]}}, x_prev})
                - $signed(m1 >>> 15)
                + $signed(m2 >>> 15);

            return sat_int(y32);
        end
    endfunction

    // Run the signal through the cascade combinationally
    logic signed [INT_W-1:0] stage_in_l, stage_in_r;
    logic signed [INT_W-1:0] stage_out_l [N_STAGES];
    logic signed [INT_W-1:0] stage_out_r [N_STAGES];

    always_comb begin
        stage_in_l = xL0;
        stage_in_r = xR0;
        for (i = 0; i < N_STAGES; i = i + 1) begin
            stage_out_l[i] = allpass_step_int(stage_in_l, xprev_l[i], yprev_l[i], lambda_q15);
            stage_out_r[i] = allpass_step_int(stage_in_r, xprev_r[i], yprev_r[i], lambda_q15);
            stage_in_l = stage_out_l[i];
            stage_in_r = stage_out_r[i];
        end
    end

    logic signed [INT_W-1:0] warped_l_int, warped_r_int;

    // After a control change, hold output while the filter state settles
    logic [SET_W-1:0] settle_cnt;
    logic             settling;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            warped_l_int <= '0;
            warped_r_int <= '0;
            settle_cnt   <= '0;
            settling     <= 1'b0;
            for (i = 0; i < N_STAGES; i = i + 1) begin
                xprev_l[i] <= '0; yprev_l[i] <= '0;
                xprev_r[i] <= '0; yprev_r[i] <= '0;
            end
        end else if (sample_strobe) begin
            if (cfg_change) begin
                // Clear the filter state when settings change
                for (i = 0; i < N_STAGES; i = i + 1) begin
                    xprev_l[i] <= '0; yprev_l[i] <= '0;
                    xprev_r[i] <= '0; yprev_r[i] <= '0;
                end
                warped_l_int <= '0;
                warped_r_int <= '0;

                settling   <= 1'b1;
                settle_cnt <= '0;
            end else if (settling) begin
                if (settle_cnt == SETTLE_SAMPLES[SET_W-1:0]) settling <= 1'b0;
                else                                         settle_cnt <= settle_cnt + 1'b1;
            end else begin
                warped_l_int <= stage_out_l[N_STAGES-1];
                warped_r_int <= stage_out_r[N_STAGES-1];

                // Update each stage state for the next sample
                for (i = 0; i < N_STAGES; i = i + 1) begin
                    if (i == 0) begin
                        xprev_l[i] <= xL0;
                        xprev_r[i] <= xR0;
                    end else begin
                        xprev_l[i] <= stage_out_l[i-1];
                        xprev_r[i] <= stage_out_r[i-1];
                    end
                    yprev_l[i] <= stage_out_l[i];
                    yprev_r[i] <= stage_out_r[i];
                end
            end
        end
    end

    // Convert back to 24-bit and restore the earlier gain reduction
    logic signed [23:0] warped_l_24, warped_r_24;
    always_comb begin
        warped_l_24 = sat24($signed(warped_l_int) <<< 1);
        warped_r_24 = sat24($signed(warped_r_int) <<< 1);
    end

    // Wet amount increases with step magnitude
    logic [15:0] wet_q15;
    logic [3:0]  mag;

    always_comb begin
        wet_q15 = 16'd0;
        mag     = abs_step(step);

        if (enable && (step != 0) && !settling) begin
            unique case (mag)
                4'd1: wet_q15 = 16'd8192;     // 0.25
                4'd2: wet_q15 = 16'd9830;     // 0.30
                4'd3: wet_q15 = 16'd11469;    // 0.35
                4'd4: wet_q15 = 16'd13107;    // 0.40
                4'd5: wet_q15 = 16'd14746;    // 0.45
                default: wet_q15 = 16'd16384; // 0.50
            endcase
        end
    end

    function automatic logic signed [23:0] mix_dry_wet(
        input logic signed [23:0] dry,
        input logic signed [23:0] wet,
        input logic [15:0]        w_q15
    );
        logic signed [16:0] w, iw;
        logic signed [47:0] md, mw;
        logic signed [31:0] y32;
        begin
            w  = {1'b0, w_q15};
            iw = 17'sd32767 - w;
            md = $signed(iw) * $signed(dry);
            mw = $signed(w)  * $signed(wet);
            y32 = $signed((md + mw) >>> 15);
            return sat24(y32);
        end
    endfunction

    logic signed [23:0] tgt_l, tgt_r;
    always_comb begin
        if (settling) begin
            tgt_l = in_left;
            tgt_r = in_right;
        end else begin
            tgt_l = mix_dry_wet(in_left,  warped_l_24, wet_q15);
            tgt_r = mix_dry_wet(in_right, warped_r_24, wet_q15);
        end
    end

    // Final output is scaled down another 6 dB
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            out_left  <= 24'sd0;
            out_right <= 24'sd0;
        end else if (sample_strobe) begin
            out_left  <= tgt_l >>> 1;
            out_right <= tgt_r >>> 1;
        end
    end

endmodule