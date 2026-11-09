`timescale 1ns/1ps
module i2c_master #(
    parameter int CLK_DIV = 10
)(
    input  logic clk,
    input  logic rst_n,
    input  logic start_i,
    input  logic [6:0] addr_i,
    input  logic [7:0] data_i,
    output logic busy_o,
    output logic ack_error_o,
    inout  wire scl_io,
    inout  wire sda_io
);

    typedef enum logic [3:0] {
        ST_IDLE = 4'd0,
        ST_START = 4'd1,
        ST_ADDR = 4'd2,
        ST_ACK_ADDR = 4'd3,
        ST_DATA = 4'd4,
        ST_ACK_DATA = 4'd5,
        ST_STOP = 4'd6
    } state_t;

    state_t c_state, n_state;

    logic [$clog2(CLK_DIV)-1:0] div_cnt;
    logic [1:0] phase;
    logic phase_tick;
    logic [2:0] bit_cnt;
    logic [7:0] shift_reg;
    logic sda_drive_low;
    logic scl_drive_low;
    logic ack_err_r;

    assign sda_io = sda_drive_low ? 1'b0 : 1'bz;
    assign scl_io = scl_drive_low ? 1'b0 : 1'bz;
    assign ack_error_o = ack_err_r;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            div_cnt <= '0;
            phase <= 2'd0;
            phase_tick <= 1'b0;
        end else if (c_state == ST_IDLE) begin
            div_cnt <= '0;
            phase <= 2'd0;
            phase_tick <= 1'b0;
        end else begin
            if (div_cnt == CLK_DIV - 1) begin
                div_cnt <= '0;
                phase <= phase + 2'd1;
                phase_tick <= 1'b1;
            end else begin
                div_cnt <= div_cnt + 1'b1;
                phase_tick <= 1'b0;
            end
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            c_state <= ST_IDLE;
            bit_cnt <= 3'd0;
            shift_reg <= 8'd0;
            ack_err_r <= 1'b0;
        end else begin
            c_state <= n_state;

            if (c_state == ST_IDLE && start_i) begin
                shift_reg <= {addr_i, 1'b0};
                bit_cnt <= 3'd7;
                ack_err_r <= 1'b0;
            end else if (phase_tick && phase == 2'd3) begin
                if (c_state == ST_ADDR || c_state == ST_DATA) begin
                    if (bit_cnt > 3'd0)
                        bit_cnt <= bit_cnt - 3'd1;
                end else if (c_state == ST_ACK_ADDR) begin
                    shift_reg <= data_i;
                    bit_cnt <= 3'd7;
                end
            end else if (phase_tick && phase == 2'd2) begin
                if (c_state == ST_ACK_ADDR || c_state == ST_ACK_DATA) begin
                    if (sda_io == 1'b1)
                        ack_err_r <= 1'b1;
                end
            end
        end
    end

    always_comb begin
        n_state = c_state;

        case (c_state)
            ST_IDLE:
                if (start_i) n_state = ST_START;

            ST_START:
                if (phase_tick && phase == 2'd3)
                    n_state = ST_ADDR;

            ST_ADDR:
                if (phase_tick && phase == 2'd3 && bit_cnt == 3'd0)
                    n_state = ST_ACK_ADDR;

            ST_ACK_ADDR:
                if (phase_tick && phase == 2'd3)
                    n_state = ack_err_r ? ST_STOP : ST_DATA;

            ST_DATA:
                if (phase_tick && phase == 2'd3 && bit_cnt == 3'd0)
                    n_state = ST_ACK_DATA;

            ST_ACK_DATA:
                if (phase_tick && phase == 2'd3)
                    n_state = ST_STOP;

            ST_STOP:
                if (phase_tick && phase == 2'd3)
                    n_state = ST_IDLE;

            default:
                n_state = ST_IDLE;
        endcase
    end

    always_comb begin
        busy_o        = (c_state != ST_IDLE);
        sda_drive_low = 1'b0;
        scl_drive_low = 1'b0;

        case (c_state)
            ST_IDLE: begin
                sda_drive_low = 1'b0;
                scl_drive_low = 1'b0;
            end

            ST_START: begin
                sda_drive_low = (phase >= 2'd2);
                scl_drive_low = 1'b0;
            end

            ST_ADDR, ST_DATA: begin
                sda_drive_low = ~shift_reg[bit_cnt];
                scl_drive_low = (phase == 2'd0 || phase == 2'd3);
            end

            ST_ACK_ADDR, ST_ACK_DATA: begin
                sda_drive_low = 1'b0;
                scl_drive_low = (phase == 2'd0 || phase == 2'd3);
            end

            ST_STOP: begin
                sda_drive_low = (phase < 2'd2);
                scl_drive_low = (phase == 2'd0);
            end

            default: begin
                sda_drive_low = 1'b0;
                scl_drive_low = 1'b0;
            end
        endcase
    end

endmodule
