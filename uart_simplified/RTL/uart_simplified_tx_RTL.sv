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

    state_t state;

    logic [$clog2(CLK_DIV)-1:0] div_cnt;
    logic [3:0] phase;
    logic [2:0] bit_cnt;
    logic [7:0] shift_reg;

    assign busy_o = (state != ST_IDLE);

    always_comb begin
        case (state)
            ST_IDLE: tx_o = 1'b1;
            ST_START: tx_o = 1'b0;
            ST_DATA: tx_o = shift_reg[bit_cnt];
            ST_STOP: tx_o = 1'b1;
            default: tx_o = 1'b1;
        endcase
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE;
            div_cnt <= '0;
            phase <= 4'd0;
            bit_cnt <= 3'd0;
            shift_reg <= 8'd0;
        end else begin
            case (state)
                ST_IDLE: begin
                    div_cnt <= '0;
                    phase <= 4'd0;
                    bit_cnt <= 3'd0;

                    if (start_i) begin
                        shift_reg <= data_i;
                        state <= ST_START;
                    end
                end

                ST_START: begin
                    if (div_cnt == CLK_DIV - 1) begin
                        div_cnt <= '0;

                        if (phase == 4'd15) begin
                            phase <= 4'd0;
                            state <= ST_DATA;
                        end else begin
                            phase <= phase + 1'b1;
                        end
                    end else begin
                        div_cnt <= div_cnt + 1'b1;
                    end
                end

                ST_DATA: begin
                    if (div_cnt == CLK_DIV - 1) begin
                        div_cnt <= '0;

                        if (phase == 4'd15) begin
                            phase <= 4'd0;

                            if (bit_cnt == 3'd7)
                                state <= ST_STOP;
                            else
                                bit_cnt <= bit_cnt + 1'b1;
                        end else begin
                            phase <= phase + 1'b1;
                        end
                    end else begin
                        div_cnt <= div_cnt + 1'b1;
                    end
                end

                ST_STOP: begin
                    if (div_cnt == CLK_DIV - 1) begin
                        div_cnt <= '0;

                        if (phase == 4'd15) begin
                            phase <= 4'd0;
                            state <= ST_IDLE;
                        end else begin
                            phase <= phase + 1'b1;
                        end
                    end else begin
                        div_cnt <= div_cnt + 1'b1;
                    end
                end

                default: begin
                    state <= ST_IDLE;
                    div_cnt <= '0;
                    phase <= '0;
                end
            endcase
        end
    end

endmodule

