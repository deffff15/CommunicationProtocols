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

    state_t state;

    logic [1:0] rx_sync;
    logic rx_fall;

    logic [$clog2(CLK_DIV)-1:0] div_cnt;
    logic [3:0] phase;

    logic [2:0] bit_cnt;
    logic [7:0] shift_reg;

    logic valid_r;
    logic error_r;

    assign rx_data_o = shift_reg;
    assign valid_o = valid_r;
    assign error_o = error_r;

    assign rx_fall = (rx_sync == 2'b10);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            rx_sync <= 2'b11;
        else
            rx_sync <= {rx_sync[0], rx_i};
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE;
            div_cnt <= '0;
            phase <= 4'd0;
            bit_cnt <= 3'd0;
            shift_reg <= 8'd0;
            valid_r <= 1'b0;
            error_r <= 1'b0;
        end else begin
            valid_r <= 1'b0;

            case (state)
                ST_IDLE: begin
                    div_cnt <= '0;
                    phase <= 4'd0;
                    bit_cnt <= 3'd0;

                    if (rx_fall) begin
                        state <= ST_START;
                        error_r <= 1'b0;
                    end
                end

                ST_START: begin
                    if (div_cnt == CLK_DIV - 1) begin
                        div_cnt <= '0;

                        if (phase == 4'd7) begin
                            phase <= 4'd0;

                            if (rx_sync[0] == 1'b0)
                                state <= ST_DATA;
                            else
                                state <= ST_IDLE;
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
                            shift_reg[bit_cnt] <= rx_sync[0];

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

                            if (rx_sync[0]) begin
                                valid_r <= 1'b1;
                            end else begin
                                error_r <= 1'b1;
                            end
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

