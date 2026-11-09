`timescale 1ns/1ps
module i2c_slave #(
    parameter logic [6:0] SLAVE_ADDR = 7'h3A
)(
    input  logic clk,
    input  logic rst_n,
    output logic [7:0] rx_data_o,
    output logic valid_o,
    inout  wire scl_io,
    inout  wire sda_io
);

    typedef enum logic [2:0] {
        ST_IDLE = 3'd0,
        ST_ADDR = 3'd1,
        ST_ACK_ADDR = 3'd2,
        ST_DATA = 3'd3,
        ST_ACK_DATA = 3'd4
    } state_t;

    state_t c_state, n_state;

    logic [1:0] scl_sync, sda_sync;
    logic scl_rise, scl_fall;
    logic sda_rise, sda_fall;
    logic start_cond, stop_cond;
    logic [2:0] bit_cnt;
    logic [7:0] shift_reg;
    logic sda_drive_low;
    logic first_fall;

    assign sda_io = sda_drive_low ? 1'b0 : 1'bz;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            scl_sync <= 2'b11;
            sda_sync <= 2'b11;
        end else begin
            scl_sync <= {scl_sync[0], scl_io};
            sda_sync <= {sda_sync[0], sda_io};
        end
    end

    assign scl_rise = (scl_sync == 2'b01);
    assign scl_fall = (scl_sync == 2'b10);
    assign sda_rise = (sda_sync == 2'b01);
    assign sda_fall = (sda_sync == 2'b10);

    assign start_cond = (scl_sync[0] == 1'b1) && sda_fall;
    assign stop_cond = (scl_sync[0] == 1'b1) && sda_rise;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            c_state <= ST_IDLE;
            bit_cnt <= 3'd0;
            shift_reg <= 8'd0;
            rx_data_o <= 8'd0;
            valid_o <= 1'b0;
            first_fall <= 1'b0;
        end else begin
            valid_o <= 1'b0;

            if (start_cond) begin
                c_state <= ST_ADDR;
                bit_cnt <= 3'd7;
                first_fall <= 1'b1;
            end else if (stop_cond) begin
                c_state <= ST_IDLE;
            end else begin
                c_state <= n_state;

                if (scl_rise && (c_state == ST_ADDR || c_state == ST_DATA)) begin
                    shift_reg <= {shift_reg[6:0], sda_sync[0]};
                end

                if (scl_fall) begin
                    if (first_fall) begin
                        first_fall <= 1'b0;
                    end else if (c_state == ST_ADDR || c_state == ST_DATA) begin
                        if (bit_cnt > 3'd0)
                            bit_cnt <= bit_cnt - 3'd1;
                    end else if (c_state == ST_ACK_ADDR) begin
                        bit_cnt <= 3'd7;
                    end else if (c_state == ST_ACK_DATA) begin
                        rx_data_o <= shift_reg;
                        valid_o <= 1'b1;
                    end
                end
            end
        end
    end

    always_comb begin
        n_state = c_state;

        case (c_state)
            ST_ADDR: begin
                if (scl_fall && !first_fall && bit_cnt == 3'd0) begin
                    if (shift_reg[7:1] == SLAVE_ADDR && shift_reg[0] == 1'b0)
                        n_state = ST_ACK_ADDR;
                    else
                        n_state = ST_IDLE;
                end
            end

            ST_ACK_ADDR:
                if (scl_fall) n_state = ST_DATA;

            ST_DATA:
                if (scl_fall && bit_cnt == 3'd0)
                    n_state = ST_ACK_DATA;

            ST_ACK_DATA:
                if (scl_fall) n_state = ST_IDLE;

            default:
                n_state = ST_IDLE;
        endcase
    end

    always_comb begin
        sda_drive_low = (c_state == ST_ACK_ADDR || c_state == ST_ACK_DATA);
    end

endmodule
