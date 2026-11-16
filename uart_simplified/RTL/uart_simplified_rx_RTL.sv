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

    localparam int BIT_CYCLES = 16 * CLK_DIV;
    localparam int CNT_WIDTH = (BIT_CYCLES <= 1) ? 1 : $clog2(BIT_CYCLES);

    localparam int HALF_BIT = BIT_CYCLES / 2;

    typedef enum logic [1:0] {
        ST_IDLE,
        ST_START,
        ST_DATA,
        ST_STOP
    } state_t;

    state_t state;

    logic [1:0] rx_sync;
    logic rx_fall;

    logic [CNT_WIDTH-1:0] cnt;
    logic [2:0] bit_cnt;
    logic [7:0] shift_reg;

    assign rx_fall = (rx_sync == 2'b10);
    assign rx_data_o = shift_reg;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_sync <= 2'b11;
        end else begin
            rx_sync <= {rx_sync[0], rx_i};
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE;
            cnt <= '0;
            bit_cnt <= '0;
            shift_reg <= '0;
            valid_o <= 1'b0;
            error_o <= 1'b0;
        end else begin
            valid_o <= 1'b0;
            error_o <= 1'b0;

            case (state)

                ST_IDLE: begin
                    cnt <= '0;

                    if (rx_fall) begin
                        state <= ST_START;
                        cnt <= '0;
                        bit_cnt <= '0;
                    end
                end

                ST_START: begin
                    if (cnt == HALF_BIT - 1) begin
                        cnt <= '0;

                        if (rx_sync[1] == 1'b0) begin
                            state <= ST_DATA;
                        end else begin
                            state <= ST_IDLE;
                        end
                    end else begin
                        cnt <= cnt + 1'b1;
                    end
                end

                ST_DATA: begin
                    if (cnt == BIT_CYCLES - 1) begin
                        cnt <= '0;
                        shift_reg[bit_cnt] <= rx_sync[1];

                        if (bit_cnt == 3'd7) begin
                            state <= ST_STOP;
                        end else begin
                            bit_cnt <= bit_cnt + 1'b1;
                        end
                    end else begin
                        cnt <= cnt + 1'b1;
                    end
                end

                ST_STOP: begin
                    if (cnt == BIT_CYCLES - 1) begin
                        cnt <= '0;
                        state <= ST_IDLE;

                        if (rx_sync[1]) begin
                            valid_o <= 1'b1;
                        end else begin
                            error_o <= 1'b1;
                        end
                    end else begin
                        cnt <= cnt + 1'b1;
                    end
                end

                default: begin
                    state <= ST_IDLE;
                    cnt <= '0;
                end

            endcase
        end
    end

endmodule

