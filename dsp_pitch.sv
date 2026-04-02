module dsp_pitch #(
    // Buffer holds 2^BUF_BITS mono samples
    parameter int BUF_BITS   = 9,   // 512 samples
    parameter int FRAC_BITS  = 16,

    // Read pointer stays behind the write pointer by this many samples.
    // This sets the base delay/latency
    parameter int DELAY_SAMP = (1 << (BUF_BITS-1))
) (
    input  logic                  clk,
    input  logic                  rst_n,          // Active-low reset

    input  logic                  sample_strobe,  // One pulse per stereo sample pair
    input  logic signed [23:0]    in_left,
    input  logic signed [23:0]    in_right,

    input  logic [15:0]           sw,

    output logic signed [23:0]    out_left,
    output logic signed [23:0]    out_right
);

    localparam int BUF_SIZE = (1 << BUF_BITS);
    localparam int PHASE_W  = BUF_BITS + FRAC_BITS;

    // Sized versions of delay-related constants
    localparam logic [BUF_BITS-1:0] DELAY_PTR   = DELAY_SAMP;
    localparam logic [BUF_BITS:0]   DELAY_PLUS2 = (DELAY_SAMP + 2);

    // Exactly one switch should be active for pitch shifting, otherwise, default to bypass (one-hot encoding)
    wire [4:0] ones      = $countones(sw);
    wire       bypass_en = (sw == 16'd0) || (ones != 5'd1);

    // Phase increment in Q16.16 format
    // A value of 65536 corresponds to 1.0x playback rate.
    logic [31:0] inc_q16_16;

    always_comb begin
        inc_q16_16 = 32'd65536;  // 1.0x

        // Upward shifts: sw[8] = +1 semitone through sw[15] = +8 semitones
        if (sw[8])       inc_q16_16 = 32'd69433;
        else if (sw[9])  inc_q16_16 = 32'd73582;
        else if (sw[10]) inc_q16_16 = 32'd77936;
        else if (sw[11]) inc_q16_16 = 32'd82564;
        else if (sw[12]) inc_q16_16 = 32'd87483;
        else if (sw[13]) inc_q16_16 = 32'd92682;
        else if (sw[14]) inc_q16_16 = 32'd98165;
        else if (sw[15]) inc_q16_16 = 32'd104040;

        // Downward shifts: sw[7] = -1 semitone through sw[0] = -8 semitones
        else if (sw[7])  inc_q16_16 = 32'd61817;
        else if (sw[6])  inc_q16_16 = 32'd58386;
        else if (sw[5])  inc_q16_16 = 32'd55173;
        else if (sw[4])  inc_q16_16 = 32'd52042;
        else if (sw[3])  inc_q16_16 = 32'd49100;
        else if (sw[2])  inc_q16_16 = 32'd46341;
        else if (sw[1])  inc_q16_16 = 32'd43712;
        else if (sw[0])  inc_q16_16 = 32'd41282;
    end

    // Clamp a wide signed value into 24-bit signed range
    function automatic logic signed [23:0] sat24(input logic signed [47:0] x);
        logic signed [47:0] max24, min24;
        begin
            max24 = 48'sd8388607;
            min24 = -48'sd8388608;
            if (x > max24)      sat24 = 24'sh7FFFFF;
            else if (x < min24) sat24 = 24'sh800000;
            else                sat24 = x[23:0];
        end
    endfunction

    // Circular buffer for mono samples
    logic signed [23:0] sample_buf [0:BUF_SIZE-1];
    logic [BUF_BITS-1:0] wr_ptr;

    // Read position stored as [integer index | fractional bits]
    logic [PHASE_W-1:0] rd_phase;

    // Tracks how many valid samples have been written so far
    logic [BUF_BITS:0] fill_cnt;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            wr_ptr    <= '0;
            rd_phase  <= '0;
            fill_cnt  <= '0;
            out_left  <= 24'sd0;
            out_right <= 24'sd0;
        end else if (sample_strobe) begin
            // Convert stereo input to mono by averaging left and right
            logic signed [24:0] sum;
            logic signed [24:0] mono25;
            logic signed [23:0] mono_in;

            sum     = $signed({in_left[23],  in_left}) + $signed({in_right[23], in_right});
            mono25  = sum >>> 1;
            mono_in = mono25[23:0];

            // Write the newest mono sample into the buffer
            sample_buf[wr_ptr] <= mono_in;
            wr_ptr             <= wr_ptr + 1'b1;

            // Stop incrementing once the buffer has been fully filled
            if (fill_cnt != BUF_SIZE[BUF_BITS:0])
                fill_cnt <= fill_cnt + 1'b1;

            if (bypass_en) begin
                // In bypass mode, pass the original stereo input through
                out_left  <= in_left;
                out_right <= in_right;

                // Keep the read phase parked at a fixed delay behind the writer
                rd_phase <= { (wr_ptr - DELAY_PTR), {FRAC_BITS{1'b0}} };
            end else begin
                // In pitch mode, output the processed mono signal on both channels
                if (fill_cnt < DELAY_PLUS2) begin
                    // Wait until enough samples exist for interpolation.
                    out_left  <= 24'sd0;
                    out_right <= 24'sd0;
                end else begin
                    logic [BUF_BITS-1:0]  rd_idx0, rd_idx1;
                    logic [FRAC_BITS-1:0] frac;
                    logic signed [23:0]   s0, s1;
                    logic signed [24:0]   diff;
                    logic signed [47:0]   base48;
                    logic signed [47:0]   interp;

                    rd_idx0 = rd_phase[FRAC_BITS + BUF_BITS - 1 : FRAC_BITS];
                    frac    = rd_phase[FRAC_BITS-1 : 0];
                    rd_idx1 = rd_idx0 + 1'b1;

                    s0 = sample_buf[rd_idx0];
                    s1 = sample_buf[rd_idx1];

                    // Linear interpolation between adjacent samples
                    diff   = $signed({s1[23], s1}) - $signed({s0[23], s0});
                    base48 = {{24{s0[23]}}, s0};
                    interp = base48 + (($signed(diff) * $signed({1'b0, frac})) >>> FRAC_BITS);

                    out_left  <= sat24(interp);
                    out_right <= sat24(interp);
                end

                // Move the read phase forward by the selected pitch ratio
                rd_phase <= rd_phase + inc_q16_16[PHASE_W-1:0];
            end
        end
    end

endmodule