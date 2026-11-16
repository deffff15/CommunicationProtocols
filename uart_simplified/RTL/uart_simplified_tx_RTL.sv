`timescale 1ns/1ps
module uart_tx #(
    parameter int CLK_DIV = 27
)(
    input  logic       clk,
    input  logic       rst_n,
    input  logic       start_i,
    input  logic [7:0] data_i,
    output logic       busy_o,
    output logic       tx_o
);

    localparam int BIT_CYCLES = 16 * CLK_DIV;
    localparam int CNT_WIDTH = (BIT_CYCLES <= 1) ? 1 : $clog2(BIT_CYCLES);
    localparam logic [CNT_WIDTH-1:0] BIT_LAST = BIT_CYCLES - 1;

    typedef enum logic [1:0] {
        ST_IDLE,
        ST_START,
        ST_DATA,
        ST_STOP
    } state_t;

    state_t state;

    logic [CNT_WIDTH-1:0] cnt;
    logic [2:0] bit_cnt;
    logic [7:0] shift_reg;

    assign busy_o = (state != ST_IDLE);

    always_comb begin
        case (state)
            ST_START: tx_o = 1'b0;
            ST_DATA:  tx_o = shift_reg[0];
            default:  tx_o = 1'b1;
        endcase
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= ST_IDLE;
            cnt       <= '0;
            bit_cnt   <= '0;
            shift_reg <= '0;
        end else begin
            case (state)

                ST_IDLE: begin
                    cnt <= '0;

                    if (start_i) begin
                        shift_reg <= data_i;
                        bit_cnt   <= 3'd0;
                        state     <= ST_START;
                    end
                end

                ST_START: begin
                    if (cnt == BIT_LAST) begin
                        cnt   <= '0;
                        state <= ST_DATA;
                    end else begin
                        cnt <= cnt + 1'b1;
                    end
                end

                ST_DATA: begin
                    if (cnt == BIT_LAST) begin
                        cnt <= '0;

                        if (bit_cnt == 3'd7) begin
                            state <= ST_STOP;
                        end else begin
                            bit_cnt   <= bit_cnt + 1'b1;
                            shift_reg <= {1'b0, shift_reg[7:1]};
                        end
                    end else begin
                        cnt <= cnt + 1'b1;
                    end
                end

                ST_STOP: begin
                    if (cnt == BIT_LAST) begin
                        cnt   <= '0;
                        state <= ST_IDLE;
                    end else begin
                        cnt <= cnt + 1'b1;
                    end
                end

                default: begin
                    state <= ST_IDLE;
                    cnt   <= '0;
                end

            endcase
        end
    end

endmodule

