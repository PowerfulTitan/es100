module audio_uart_logger_pair #(
    parameter int CLK_HZ      = 12_288_000,
    parameter int BAUD        = 3_072_000,
    parameter int FIFO_BYTES  = 8192
) (
    input  logic        clk,
    input  logic        rst,        // active-high synchronous reset

    input  logic        enable,

    input  logic signed [15:0] in_sample,
    input  logic signed [15:0] out_sample,
    input  logic               sample_valid,   // 1 pulse per sample pair (48k)

    output logic        uart_txd,
    output logic        fifo_overflow
);

    // Frame: A5, in_lo, in_hi, out_lo, out_hi
    logic [7:0] b0, b1, b2, b3, b4;
    always_comb begin
        b0 = 8'hA5;
        b1 = in_sample[7:0];
        b2 = in_sample[15:8];
        b3 = out_sample[7:0];
        b4 = out_sample[15:8];
    end

    // FIFO wires
    logic [7:0] fifo_wdata, fifo_rdata;
    logic       fifo_wvalid, fifo_wready;
    logic       fifo_rvalid, fifo_rready;

    byte_fifo #(.DEPTH(FIFO_BYTES)) u_fifo (
        .clk(clk), .rst(rst),
        .wdata(fifo_wdata), .wvalid(fifo_wvalid), .wready(fifo_wready),
        .rdata(fifo_rdata), .rready(fifo_rready), .rvalid(fifo_rvalid),
        .overflow(fifo_overflow)
    );

    // Emit 5 bytes per sample_valid
    logic [2:0] emit_idx;
    logic       emitting;

    always_ff @(posedge clk) begin
        if (rst) begin
            emitting    <= 1'b0;
            emit_idx    <= 3'd0;
            fifo_wvalid <= 1'b0;
            fifo_wdata  <= 8'd0;
        end else begin
            fifo_wvalid <= 1'b0;

            if (!emitting) begin
                if (enable && sample_valid) begin
                    emitting <= 1'b1;
                    emit_idx <= 3'd0;
                end
            end else begin
                if (fifo_wready) begin
                    fifo_wvalid <= 1'b1;
                    case (emit_idx)
                        3'd0: fifo_wdata <= b0;
                        3'd1: fifo_wdata <= b1;
                        3'd2: fifo_wdata <= b2;
                        3'd3: fifo_wdata <= b3;
                        default: fifo_wdata <= b4;
                    endcase

                    if (emit_idx == 3'd4) begin
                        emitting <= 1'b0;
                        emit_idx <= 3'd0;
                    end else begin
                        emit_idx <= emit_idx + 3'd1;
                    end
                end
            end
        end
    end

    // UART drains FIFO
    logic uart_ready;
    wire  uart_valid = fifo_rvalid;
    wire [7:0] uart_data = fifo_rdata;

    assign fifo_rready = uart_ready && fifo_rvalid;

    uart_tx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) u_uart (
        .clk(clk), .rst(rst),
        .tx_data(uart_data),
        .tx_valid(uart_valid),
        .tx_ready(uart_ready),
        .txd(uart_txd)
    );

endmodule
