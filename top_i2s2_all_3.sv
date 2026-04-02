module top_i2s2_all_3 (
    input  logic        clk100,
    input  logic [15:0] sw,

    // Pmod I2S2 DAC (Line Out)
    output logic da_mclk,
    output logic da_lrck,
    output logic da_sclk,
    output logic da_sdin,

    // Pmod I2S2 ADC (Line In)
    output logic ad_mclk,
    output logic ad_lrck,
    output logic ad_sclk,
    input  logic  ad_sdout,

    // USB-UART TX (to PC)
    output logic uart_txd,

    // Nexys A7 LEDs
    output logic [15:0] led
);

    localparam int USE_16BIT = 1;

    // ----------------------------
    // Buffer clk100
    // ----------------------------
    logic clk100_buf;
    BUFG bufg_clk100 (.I(clk100), .O(clk100_buf));

    // ----------------------------
    // Clock wizard for MCLK (12.288 MHz)
    // ----------------------------
    logic clk_mclk, clk_locked;
    logic rst_wiz = 1'b0;

    clk_wiz_0 u_clk (
        .clk_in1  (clk100_buf),
        .reset    (rst_wiz),
        .clk_out1 (clk_mclk),
        .locked   (clk_locked)
    );

    assign da_mclk = clk_mclk;
    assign ad_mclk = clk_mclk;

    // ----------------------------
    // Clean reset release in clk_mclk domain
    // ----------------------------
    logic [1:0] lock_sync;
    logic       rstn_mclk;

    always_ff @(posedge clk_mclk) begin
        lock_sync <= {lock_sync[0], clk_locked};
        rstn_mclk  <= &lock_sync;
    end

    // ----------------------------
    // Generate SCLK and LRCK using 4-phase divider
    // ----------------------------
    logic [1:0] div4;
    logic       sclk_q;

    wire phase0 = (div4 == 2'd0);
    wire phase1 = (div4 == 2'd1);
    wire phase2 = (div4 == 2'd2);
    wire phase3 = (div4 == 2'd3);

    wire tx_ce  = rstn_mclk && phase0;
    wire rx_ce  = rstn_mclk && phase2;

    logic [5:0] bit_cnt;
    logic       lrck_q;

    always_ff @(posedge clk_mclk) begin
        if (!rstn_mclk) begin
            div4    <= 2'd0;
            sclk_q  <= 1'b0;
            bit_cnt <= 6'd0;
            lrck_q  <= 1'b0;
        end else begin
            if (phase1) sclk_q <= 1'b1;
            if (phase3) sclk_q <= 1'b0;

            div4 <= div4 + 2'd1;

            if (phase3) begin
                if (bit_cnt == 6'd63) bit_cnt <= 6'd0;
                else                  bit_cnt <= bit_cnt + 6'd1;

                if (bit_cnt == 6'd63) lrck_q <= 1'b0;
                else                  lrck_q <= ((bit_cnt + 6'd1) >= 6'd32);
            end
        end
    end

    assign da_sclk = sclk_q;
    assign ad_sclk = sclk_q;
    assign da_lrck = lrck_q;
    assign ad_lrck = lrck_q;

    wire is_left = (bit_cnt < 6'd32);
    wire [5:0] pos_in_chan = is_left ? bit_cnt : (bit_cnt - 6'd32);

    // ----------------------------
    // RX (I2S): pos0 dummy, pos1..pos24 data
    // ----------------------------
    logic [23:0] rx_shift24;
    logic signed [23:0] adc_left, adc_right;
    logic left_valid, right_valid;

    always_ff @(posedge clk_mclk) begin
        if (!rstn_mclk) begin
            rx_shift24  <= 24'd0;
            adc_left    <= 24'sd0;
            adc_right   <= 24'sd0;
            left_valid  <= 1'b0;
            right_valid <= 1'b0;
        end else begin
            left_valid  <= 1'b0;
            right_valid <= 1'b0;

            if (rx_ce) begin
                if (pos_in_chan == 6'd0)
                    rx_shift24 <= 24'd0;

                if (pos_in_chan >= 6'd1 && pos_in_chan <= 6'd24) begin
                    rx_shift24 <= {rx_shift24[22:0], ad_sdout};

                    if (pos_in_chan == 6'd24) begin
                        if (is_left) begin
                            adc_left   <= {rx_shift24[22:0], ad_sdout};
                            left_valid <= 1'b1;
                        end else begin
                            adc_right   <= {rx_shift24[22:0], ad_sdout};
                            right_valid <= 1'b1;
                        end
                    end
                end
            end
        end
    end

    // ----------------------------
    // Hold most recent samples
    // ----------------------------
    logic signed [23:0] play_left, play_right;

    always_ff @(posedge clk_mclk) begin
        if (!rstn_mclk) begin
            play_left  <= 24'sd0;
            play_right <= 24'sd0;
        end else begin
            if (left_valid)  play_left  <= adc_left;
            if (right_valid) play_right <= adc_right;
        end
    end

    wire [23:0] play_left_fmt  = USE_16BIT ? {play_left[23:8],  8'h00} : play_left;
    wire [23:0] play_right_fmt = USE_16BIT ? {play_right[23:8], 8'h00} : play_right;

    wire sample_strobe = right_valid;

    // ------------------------------------------------------------
    // Switch sync + debounce
    // ------------------------------------------------------------
    logic [15:0] sw_ff1, sw_ff2;
    always_ff @(posedge clk_mclk) begin
        sw_ff1 <= sw;
        sw_ff2 <= sw_ff1;
    end
    wire [15:0] sw_sync = sw_ff2;

    localparam int DEB_SAMPLES = 1024;
    localparam int DEB_BITS    = $clog2(DEB_SAMPLES);

    logic [15:0] sw_last;
    logic [15:0] sw_db;
    logic [DEB_BITS-1:0] deb_cnt;

    always_ff @(posedge clk_mclk) begin
        if (!rstn_mclk) begin
            sw_last <= 16'd0;
            sw_db   <= 16'd0;
            deb_cnt <= '0;
        end else if (sample_strobe) begin
            if (sw_sync == sw_last) begin
                if (deb_cnt != DEB_SAMPLES-1)
                    deb_cnt <= deb_cnt + 1'b1;
                else
                    sw_db <= sw_sync;
            end else begin
                sw_last <= sw_sync;
                deb_cnt <= '0;
            end
        end
    end

    // ============================================================
    // Control decode: mode=SW0..SW2, strength=SW3..SW15, midpoint=SW9
    // ============================================================
    logic [2:0] mode;
    assign mode = sw_db[2:0];

    wire [12:0] scale_sw   = sw_db[15:3];
    wire [4:0]  scale_ones = $countones(scale_sw);
    wire        scale_ok   = (scale_sw == 13'd0) || (scale_ones == 5'd1);
    wire        scale_any  = (scale_sw != 13'd0);

    // Base "offset-from-midpoint": SW3..SW8=-6..-1, SW9=0, SW10..SW15=+1..+6
    logic signed [3:0] step;
    always_comb begin
        step = 4'sd0;

        if (sw_db[15])      step = 4'sd6;
        else if (sw_db[14]) step = 4'sd5;
        else if (sw_db[13]) step = 4'sd4;
        else if (sw_db[12]) step = 4'sd3;
        else if (sw_db[11]) step = 4'sd2;
        else if (sw_db[10]) step = 4'sd1;

        else if (sw_db[8])  step = -4'sd1;
        else if (sw_db[7])  step = -4'sd2;
        else if (sw_db[6])  step = -4'sd3;
        else if (sw_db[5])  step = -4'sd4;
        else if (sw_db[4])  step = -4'sd5;
        else if (sw_db[3])  step = -4'sd6;
        // SW9 => 0
    end

    // ============================================================
    // Config-change detect
    // ============================================================
    logic [15:0] sw_db_prev;
    logic        cfg_change;

    always_ff @(posedge clk_mclk) begin
        if (!rstn_mclk) begin
            sw_db_prev <= 16'd0;
            cfg_change <= 1'b0;
        end else if (sample_strobe) begin
            cfg_change <= (sw_db != sw_db_prev);
            sw_db_prev <= sw_db;
        end
    end

    // ============================================================
    // Make SW9 ACTIVE by giving each algorithm a non-identity baseline
    // ============================================================
    localparam logic signed [3:0] PITCH_BASE  = 4'sd1;  // SW9 => +1 semitone
    localparam logic signed [3:0] FORM_BASE   = 4'sd1;  // SW9 => +1 step
    localparam integer            MCA_BASE    = 9;      // SW9 => alpha_sel=9 (~1.1), not 8 (~1.0)

    logic signed [3:0] pitch_step;
    logic signed [3:0] form_step;
    always_comb begin
        pitch_step = step + PITCH_BASE;   // range -5..+7
        form_step  = step + FORM_BASE;    // range -5..+7
    end

    logic [3:0] mca_alpha_sel;
    always_comb begin
        int tmp;
        tmp = MCA_BASE + step;            // step in [-6..+6] => tmp in [3..15]
        if (tmp < 0)  tmp = 0;
        if (tmp > 15) tmp = 15;
        mca_alpha_sel = tmp[3:0];
    end

    // ============================================================
    // Mode plan:
    // 000 passthrough
    // 001 pitch
    // 010 formant
    // 011 mcadams
    // 100 pitch+formant
    // 101 pitch+mcadams
    // 110 formant+mcadams
    // 111 pitch+formant+mcadams
    // ============================================================

    // Processing is allowed whenever a valid one-hot strength switch is chosen.
    // (SW9 counts as a valid strength selection.)
    wire process_gate  = scale_ok && scale_any;

    wire need_pitch = (mode == 3'b001) || (mode == 3'b100) || (mode == 3'b101) || (mode == 3'b111);
    wire need_form  = (mode == 3'b010) || (mode == 3'b100) || (mode == 3'b110) || (mode == 3'b111);
    wire need_mca   = (mode == 3'b011) || (mode == 3'b101) || (mode == 3'b110) || (mode == 3'b111);

    wire en_pitch = process_gate && need_pitch;
    wire en_form  = process_gate && need_form;
    wire en_mca   = process_gate && need_mca;

    wire do_process = en_pitch || en_form || en_mca;

    // ============================================================
    // pitch_sw mapping (one-hot) from pitch_step (NOT raw step)
    // ============================================================
    logic [15:0] pitch_sw;
    always_comb begin
        pitch_sw = 16'd0;
        if (en_pitch) begin
            unique case (pitch_step)
                4'sd1:  pitch_sw[8]  = 1'b1;
                4'sd2:  pitch_sw[9]  = 1'b1;
                4'sd3:  pitch_sw[10] = 1'b1;
                4'sd4:  pitch_sw[11] = 1'b1;
                4'sd5:  pitch_sw[12] = 1'b1;
                4'sd6:  pitch_sw[13] = 1'b1;
                4'sd7:  pitch_sw[14] = 1'b1;

                -4'sd1: pitch_sw[7]  = 1'b1;
                -4'sd2: pitch_sw[6]  = 1'b1;
                -4'sd3: pitch_sw[5]  = 1'b1;
                -4'sd4: pitch_sw[4]  = 1'b1;
                -4'sd5: pitch_sw[3]  = 1'b1;
                -4'sd6: pitch_sw[2]  = 1'b1;
                -4'sd7: pitch_sw[1]  = 1'b1;

                default: pitch_sw = 16'd0;
            endcase
        end
    end

    // ============================================================
    // SINGLE instances of each DSP block + CASCADE muxing
    // ============================================================

    // Stage 0 (dry)
    logic signed [23:0] s0_l, s0_r;
    always_comb begin
        s0_l = play_left_fmt;
        s0_r = play_right_fmt;
    end

    // --- Pitch stage ---
    logic signed [23:0] pitch_l, pitch_r;
    dsp_pitch u_pitch (
        .clk          (clk_mclk),
        .rst_n         (rstn_mclk),
        .sample_strobe (sample_strobe),
        .in_left       (s0_l),
        .in_right      (s0_r),
        .sw            (pitch_sw),
        .out_left      (pitch_l),
        .out_right     (pitch_r)
    );

    logic signed [23:0] s1_l, s1_r;
    always_comb begin
        s1_l = en_pitch ? pitch_l : s0_l;
        s1_r = en_pitch ? pitch_r : s0_r;
    end

    // --- Formant stage (uses form_step baseline) ---
    logic signed [23:0] form_l, form_r;
    dsp_formant u_formant (
        .clk          (clk_mclk),
        .rst_n         (rstn_mclk),
        .sample_strobe (sample_strobe),
        .in_left       (s1_l),
        .in_right      (s1_r),
        .step          (form_step),
        .enable        (en_form),
        .cfg_change    (cfg_change),
        .out_left      (form_l),
        .out_right     (form_r)
    );

    logic signed [23:0] s2_l, s2_r;
    always_comb begin
        s2_l = en_form ? form_l : s1_l;
        s2_r = en_form ? form_r : s1_r;
    end

    // --- McAdams stage ---
    logic signed [23:0] mca_l, mca_r;
    wire        mca_busy;
    wire [15:0] mca_dbg;

    dsp_mcadams u_mcadams (
        .clk           (clk_mclk),
        .rst_n          (rstn_mclk),
        .sample_strobe  (sample_strobe),
        .in_left        (s2_l),
        .in_right       (s2_r),
        .enable         (en_mca),
        .alpha_sel      (mca_alpha_sel),
        .cfg_change     (cfg_change),
        .out_left       (mca_l),
        .out_right      (mca_r),
        .dbg_busy       (mca_busy),
        .dbg_rootcount  (mca_dbg)
    );

    // Bypass mcadams until it has a processed hop buffer (mca_dbg[14] == have_buf)
    wire mca_have_buf = mca_dbg[14];

    logic signed [23:0] proc_left, proc_right;
    always_comb begin
        if (en_mca && mca_have_buf) begin
            proc_left  = mca_l;
            proc_right = mca_r;
        end else begin
            proc_left  = s2_l;
            proc_right = s2_r;
        end
    end

    // ============================================================
    // OUTPUT CROSSFADE (only used when do_process==1)
    // ============================================================
    logic signed [23:0] xf_left, xf_right;
    logic               xf_fading;

    dsp_formant_crossfade #(.FADE_SAMPLES(2048)) u_outfade (
        .clk          (clk_mclk),
        .rst_n         (rstn_mclk),
        .sample_strobe (sample_strobe),
        .change        (cfg_change),
        .in_new_l      (proc_left),
        .in_new_r      (proc_right),
        .out_l         (xf_left),
        .out_r         (xf_right),
        .dbg_fading    (xf_fading)
    );

    // Force digital silence debug
    wire force_silence = sw_db[15] & sw_db[14];

    // FINAL OUTPUT select (hard bypass when not processing)
    logic signed [23:0] final_left, final_right;
    always_comb begin
        if (force_silence) begin
            final_left  = 24'sd0;
            final_right = 24'sd0;
        end else if (!do_process) begin
            final_left  = s0_l;
            final_right = s0_r;
        end else begin
            final_left  = xf_left;
            final_right = xf_right;
        end
    end

    // ----------------------------
    // TX (I2S)
    // ----------------------------
    logic [23:0] tx_word;
    logic        tx_sdin;

    always_ff @(posedge clk_mclk) begin
        if (!rstn_mclk) begin
            tx_word <= 24'd0;
            tx_sdin <= 1'b0;
        end else if (tx_ce) begin
            if (pos_in_chan == 6'd0) begin
                tx_word <= is_left ? final_left : final_right;
                tx_sdin <= 1'b0;
            end else if (pos_in_chan >= 6'd1 && pos_in_chan <= 6'd24) begin
                tx_sdin <= tx_word[23];
                tx_word <= {tx_word[22:0], 1'b0};
            end else begin
                tx_sdin <= 1'b0;
            end
        end
    end

    assign da_sdin = tx_sdin;

    // ============================================================
    // UART LOGGER TAP (mono-left, 16-bit)
    // ============================================================
    logic signed [15:0] log_in16, log_out16;
    always_comb begin
        log_in16  = play_left_fmt[23:8];
        log_out16 = final_left[23:8];
    end

    wire  log_enable = rstn_mclk;
    logic log_overflow;

    audio_uart_logger_pair #(
        .CLK_HZ(12_288_000),
        .BAUD(3_072_000),
        .FIFO_BYTES(8192)
    ) u_logger_pair (
        .clk          (clk_mclk),
        .rst          (!rstn_mclk),
        .enable       (log_enable),
        .in_sample    (log_in16),
        .out_sample   (log_out16),
        .sample_valid (sample_strobe),
        .uart_txd     (uart_txd),
        .fifo_overflow(log_overflow)
    );

    // ----------------------------
    // LEDs (basic visibility)
    // ----------------------------
    logic [26:0] hb;
    always_ff @(posedge clk100_buf) hb <= hb + 27'd1;

    always_comb begin
        led = 16'd0;

        led[0]  = hb[26];
        led[15] = rstn_mclk;
        led[14] = lrck_q;
        led[13] = sclk_q;
        led[12] = ad_sdout;

        led[3:1]  = mode;
        led[8]    = scale_any;
        led[9]    = do_process;
        led[10]   = xf_fading;
        led[11]   = mca_busy;
        led[6]    = cfg_change;
        led[4]    = log_overflow;
    end

endmodule