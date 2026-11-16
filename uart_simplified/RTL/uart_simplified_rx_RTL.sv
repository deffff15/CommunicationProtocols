`timescale 1ns/1ps
module uart_rx #(
    parameter int CLK_DIV = 27
)(
    input logic clk,
    input logic rst_n,
    output logic [7:0] rx_data_o,
    output logic valid_o,
    output logic error_o,
    input logic rx_i
);

    typedef enum logic [1:0] {
        ST_IDLE = 2'd0,
        ST_START = 2'd1,
        ST_DATA = 2'd2,
        ST_STOP = 2'd3
    } state_t;

    state_t c_state, n_state;

    logic [1:0] rx_sync;
    logic rx_fall;
    logic [$clog2(CLK_DIV)-1:0] div_cnt;
    logic [3:0] phase;
    logic phase_tick;
    logic [2:0] bit_cnt;
    logic [7:0] shift_reg;
    logic valid_r;
    logic error_r;

    assign rx_data_o = shift_reg;
    assign valid_o = valid_r;
    assign error_o = error_r;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            rx_sync <= 2'b11;
        else
            rx_sync <= {rx_sync[0], rx_i};
    end

    assign rx_fall = (rx_sync == 2'b10);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            div_cnt <= '0;
            phase <= 4'd0;
            phase_tick <= 1'b0;
        end else if (c_state == ST_IDLE) begin
            div_cnt <= '0;
            phase <= 4'd0;
            phase_tick <= 1'b0;
        end else begin
            if (div_cnt == CLK_DIV - 1) begin
                div_cnt <= '0;
                phase_tick <= 1'b1;

                if (c_state == ST_START && phase == 4'd7)
                    phase <= 4'd0;
                else
                    phase <= phase + 4'd1;
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
            valid_r <= 1'b0;
            error_r <= 1'b0;
        end else begin
            c_state <= n_state;
            valid_r <= 1'b0;

            if (c_state == ST_IDLE && rx_fall) begin
                bit_cnt <= 3'd0;
                error_r <= 1'b0;
            end else if (phase_tick) begin
                if (c_state == ST_DATA && phase == 4'd15) begin
                    shift_reg[bit_cnt] <= rx_sync[0];

                    if (bit_cnt < 3'd7)
                        bit_cnt <= bit_cnt + 3'd1;
                end

                if (c_state == ST_STOP && phase == 4'd15) begin
                    if (rx_sync[0])
                        valid_r <= 1'b1;
                    else
                        error_r <= 1'b1;
                end
            end
        end
    end

    always_comb begin
        n_state = c_state;

        case (c_state)
            ST_IDLE:
                if (rx_fall)
                    n_state = ST_START;

            ST_START:
                if (phase_tick && phase == 4'd7)
                    n_state = (rx_sync[0] == 1'b0) ? ST_DATA : ST_IDLE;

            ST_DATA:
                if (phase_tick && phase == 4'd15 && bit_cnt == 3'd7)
                    n_state = ST_STOP;

            ST_STOP:
                if (phase_tick && phase == 4'd15)
                    n_state = ST_IDLE;

            default:
                n_state = ST_IDLE;
        endcase
    end

endmodule

