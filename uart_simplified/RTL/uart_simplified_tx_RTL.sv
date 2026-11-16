`timescale 1ns/1ps
module uart_tx #(
    parameter int CLK_DIV = 27
)(
    input logic clk,
    input logic rst_n,
    input logic start_i,
    input logic [7:0] data_i,
    output logic busy_o,
    output logic tx_o
);

    typedef enum logic [1:0] {
        ST_IDLE = 2'd0,
        ST_START = 2'd1,
        ST_DATA = 2'd2,
        ST_STOP = 2'd3
    } state_t;

    state_t c_state, n_state;

    logic [$clog2(CLK_DIV)-1:0] div_cnt;
    logic [3:0] phase;
    logic phase_tick;
    logic [2:0] bit_cnt;
    logic [7:0] shift_reg;
    logic tx_r;

    assign tx_o = tx_r;
    assign busy_o = (c_state != ST_IDLE);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            div_cnt <= '0;
            phase <= 4'd15;
            phase_tick <= 1'b0;
        end else if (c_state == ST_IDLE) begin
            div_cnt <= '0;
            phase <= 4'd15;
            phase_tick <= 1'b0;
        end else begin
            if (div_cnt == CLK_DIV - 1) begin
                div_cnt <= '0;
                phase <= phase + 4'd1;
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
        end else begin
            c_state <= n_state;

            if (c_state == ST_IDLE && start_i) begin
                shift_reg <= data_i;
                bit_cnt <= 3'd0;
            end else if (phase_tick && phase == 4'd15) begin
                if (c_state == ST_DATA && bit_cnt < 3'd7)
                    bit_cnt <= bit_cnt + 3'd1;
            end
        end
    end

    always_comb begin
        n_state = c_state;

        case (c_state)
            ST_IDLE:
                if (start_i) n_state = ST_START;

            ST_START:
                if (phase_tick && phase == 4'd15)
                    n_state = ST_DATA;

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

    always_comb begin
        tx_r = 1'b1;

        case (c_state)
            ST_IDLE: tx_r = 1'b1;
            ST_START: tx_r = 1'b0;
            ST_DATA: tx_r = shift_reg[bit_cnt];
            ST_STOP: tx_r = 1'b1;
            default: tx_r = 1'b1;
        endcase
    end

endmodule

