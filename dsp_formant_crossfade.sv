module dsp_formant_crossfade #(
    parameter int FADE_SAMPLES = 2048  // Must be a power of 2
) (
    input  logic               clk,
    input  logic               rst_n,
    input  logic               sample_strobe,
    input  logic               change,      // Pulse high when settings change
    input  logic signed [23:0] in_new_l,
    input  logic signed [23:0] in_new_r,
    output logic signed [23:0] out_l,
    output logic signed [23:0] out_r,
    output logic               dbg_fading   // Debug signal for LED
);

    initial begin
        if ((FADE_SAMPLES & (FADE_SAMPLES-1)) != 0)
            $error("FADE_SAMPLES must be power of two");
    end

    localparam int SHIFT = $clog2(FADE_SAMPLES);
    localparam int CNT_W = $clog2(FADE_SAMPLES+1);

    logic signed [23:0] old_l, old_r;
    logic [CNT_W-1:0]   cnt;
    logic               fading;
    logic [15:0]        a_q15;

    assign dbg_fading = fading;

    function automatic logic signed [23:0] sat24(input logic signed [31:0] x);
        if (x > 32'sd8388607)        return 24'sd8388607;
        else if (x < -32'sd8388608)  return -24'sd8388608;
        else                         return x[23:0];
    endfunction

    function automatic logic signed [23:0] xfade1(
        input logic signed [23:0] oldx,
        input logic signed [23:0] newx,
        input logic [15:0]        a
    );
        logic signed [24:0] diff;
        logic signed [40:0] prod;
        logic signed [31:0] y32;
        begin
            // Interpolate between old and new using alpha
            diff = $signed({newx[23], newx}) - $signed({oldx[23], oldx});
            prod = $signed({1'b0,a}) * $signed(diff);
            y32  = $signed({{8{oldx[23]}}, oldx}) + $signed(prod >>> 15);
            return sat24(y32);
        end
    endfunction

    always_comb begin
        logic [31:0] tmp;
        tmp   = cnt * 32'd32767;
        a_q15 = tmp[SHIFT +: 16];  // Approx. (cnt / FADE_SAMPLES) * 32767

        if (cnt >= FADE_SAMPLES[CNT_W-1:0])
            a_q15 = 16'd32767;
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            out_l   <= 24'sd0;
            out_r   <= 24'sd0;
            old_l   <= 24'sd0;
            old_r   <= 24'sd0;
            cnt     <= '0;
            fading  <= 1'b0;
        end else if (sample_strobe) begin
            if (change) begin
                // Save the current output as the fade starting point
                old_l   <= out_l;
                old_r   <= out_r;
                cnt     <= '0;
                fading  <= 1'b1;
            end else if (fading) begin
                // Advance the fade until it reaches full new signal
                if (cnt == FADE_SAMPLES[CNT_W-1:0]) fading <= 1'b0;
                else                                cnt <= cnt + 1'b1;
            end

            if (fading) begin
                out_l <= xfade1(old_l, in_new_l, a_q15);
                out_r <= xfade1(old_r, in_new_r, a_q15);
            end else begin
                // Once fading is done, pass the new signal directly
                out_l <= in_new_l;
                out_r <= in_new_r;
            end
        end
    end
endmodule