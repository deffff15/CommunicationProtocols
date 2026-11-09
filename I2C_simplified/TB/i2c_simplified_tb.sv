`timescale 1ns/1ps

// ---------------------------------------------------------------------------
// Parameters
// ---------------------------------------------------------------------------
package i2c_tb_pkg;
    localparam int         CLK_DIV    = 10;
    localparam logic [6:0] SLAVE_ADDR = 7'h3A;
    localparam int         LEN_ACK    = 79 * CLK_DIV + 1;
    localparam int         LEN_NACK   = 43 * CLK_DIV + 1;
endpackage

import i2c_tb_pkg::*;

// ---------------------------------------------------------------------------
// Interface
// ---------------------------------------------------------------------------
interface i2c_if (input logic clk, input wire scl, input wire sda);
    logic       rst_n;
    logic       start;
    logic [6:0] addr;
    logic [7:0] data;
    logic       busy;
    logic       ack_error;
    logic [7:0] rx_data;
    logic       valid;

    clocking mon_cb @(posedge clk);
        default input #1step;
        input rst_n, start, addr, data, busy, ack_error, rx_data, valid, scl, sda;
    endclocking
endinterface

// ---------------------------------------------------------------------------
// Transaction
// ---------------------------------------------------------------------------
class I2cTxn;
    rand bit       rst_n;
    rand bit       start;
    rand bit       use_slave;
    rand bit [6:0] addr;
    rand bit [7:0] data;

    int rst_w   = 1;
    int start_w = 50;
    int slave_w = 500;

    logic       busy;
    logic       ack_error;
    logic [7:0] rx_data;
    logic       valid;
    logic       scl;
    logic       sda;

    constraint c_ctrl {
        rst_n     dist {1'b0 := rst_w,   1'b1 := 1000 - rst_w};
        start     dist {1'b1 := start_w, 1'b0 := 1000 - start_w};
        use_slave dist {1'b1 := slave_w, 1'b0 := 1000 - slave_w};
    }

    function void post_randomize();
        if (use_slave) addr = SLAVE_ADDR;
    endfunction
endclass

// ---------------------------------------------------------------------------
// Scoreboard
// ---------------------------------------------------------------------------
class I2cScoreboard;
    bit        in_txn;
    bit        just_started;
    bit        finished_now;
    int        busy_len;
    bit [6:0]  exp_addr;
    bit [7:0]  exp_data;
    bit        exp_match;
    int        valid_cnt;
    int        starts_seen;
    int        stops_seen;
    int        clk_cnt;
    bit [17:0] bits_s;
    bit        prev_scl = 1'b1;
    bit        prev_sda = 1'b1;

    int checked          = 0;
    int errors           = 0;
    int txns             = 0;
    int completed        = 0;
    int match_txns       = 0;
    int miss_txns        = 0;
    int acked            = 0;
    int nacked           = 0;
    int data_ok          = 0;
    int data_acked       = 0;
    int valid_ok         = 0;
    int start_ignored    = 0;
    int back_to_back     = 0;
    int extra_stops      = 0;
    int extra_clocks     = 0;
    int resets_mid_txn   = 0;
    int resets_idle      = 0;
    int diff_bit [7];

    function new();
        for (int i = 0; i < 7; i++)
            diff_bit[i] = 0;
        reset_model();
    endfunction

    function void reset_model();
        in_txn       = 1'b0;
        just_started = 1'b0;
        busy_len     = 0;
        valid_cnt    = 0;
        starts_seen  = 0;
        stops_seen   = 0;
        clk_cnt      = 0;
        bits_s       = '0;
    endfunction

    function bit finish_txn(I2cTxn tr);
        bit       ok = 1;
        int       exp_len;
        bit       exp_ack_error;
        bit [7:0] dec_addr;
        bit [7:0] dec_data;

        completed++;
        if (exp_match) match_txns++;
        else           miss_txns++;

        exp_ack_error = !exp_match;
        exp_len       = (tr.ack_error === 1'b1) ? LEN_NACK : LEN_ACK;

        if (tr.ack_error !== exp_ack_error) begin
            $error("[SB] ack_error_o mismatch @ %0t: got %0b, expected %0b (addr %h)",
                   $time, tr.ack_error, exp_ack_error, exp_addr);
            ok = 0;
        end

        if (busy_len != exp_len) begin
            $error("[SB] busy length mismatch @ %0t: got %0d cycles, expected %0d (ack_error %0b)",
                   $time, busy_len, exp_len, tr.ack_error);
            ok = 0;
        end

        if (starts_seen != 1) begin
            $error("[SB] expected exactly one START @ %0t, saw %0d", $time, starts_seen);
            ok = 0;
        end

        if (stops_seen == 0) begin
            $error("[SB] no STOP condition seen @ %0t", $time);
            ok = 0;
        end
        else if (stops_seen > 1)
            extra_stops++;

        if (clk_cnt < 9) begin
            $error("[SB] only %0d SCL pulses after START @ %0t", clk_cnt, $time);
            ok = 0;
        end
        else begin
            for (int i = 0; i < 8; i++)
                dec_addr[7 - i] = bits_s[i];

            if (dec_addr !== {exp_addr, 1'b0}) begin
                $error("[SB] address byte on the bus mismatch @ %0t: got %h, expected %h",
                       $time, dec_addr, {exp_addr, 1'b0});
                ok = 0;
            end

            if (bits_s[8]) nacked++;
            else           acked++;

            if (bits_s[8] !== exp_ack_error) begin
                $error("[SB] address ACK bit mismatch @ %0t: got %0b, expected %0b (addr %h)",
                       $time, bits_s[8], exp_ack_error, exp_addr);
                ok = 0;
            end

            if (tr.ack_error !== bits_s[8]) begin
                $error("[SB] ack_error_o does not follow the ACK bit on the bus @ %0t: ack_error %0b, bus %0b",
                       $time, tr.ack_error, bits_s[8]);
                ok = 0;
            end

            if (!bits_s[8]) begin
                if (clk_cnt < 18) begin
                    $error("[SB] data byte incomplete @ %0t: %0d SCL pulses", $time, clk_cnt);
                    ok = 0;
                end
                else begin
                    for (int i = 0; i < 8; i++)
                        dec_data[7 - i] = bits_s[9 + i];

                    if (dec_data !== exp_data) begin
                        $error("[SB] data byte on the bus mismatch @ %0t: got %h, expected %h",
                               $time, dec_data, exp_data);
                        ok = 0;
                    end
                    else
                        data_ok++;

                    if (bits_s[17]) begin
                        $error("[SB] data ACK bit mismatch @ %0t: slave did not ACK the data byte", $time);
                        ok = 0;
                    end
                    else
                        data_acked++;
                end
                if (clk_cnt > 18) extra_clocks++;
            end
            else if (clk_cnt > 9)
                extra_clocks++;
        end

        if (valid_cnt != (exp_match ? 1 : 0)) begin
            $error("[SB] valid_o pulses mismatch @ %0t: got %0d, expected %0d (addr %h)",
                   $time, valid_cnt, (exp_match ? 1 : 0), exp_addr);
            ok = 0;
        end

        return ok;
    endfunction

    function void check(I2cTxn tr);
        bit       ok = 1;
        bit       start_cond;
        bit       stop_cond;
        bit       scl_rise;
        bit [6:0] diff;

        finished_now = 1'b0;
        start_cond   = prev_scl && tr.scl && prev_sda && !tr.sda;
        stop_cond    = prev_scl && tr.scl && !prev_sda && tr.sda;
        scl_rise     = !prev_scl && tr.scl;

        if (!tr.rst_n) begin
            if (in_txn) resets_mid_txn++;
            else        resets_idle++;
            reset_model();
        end
        else begin
            if (tr.valid === 1'b1) begin
                if (!in_txn) begin
                    $error("[SB] unexpected valid_o @ %0t", $time);
                    ok = 0;
                end
                else begin
                    valid_cnt++;
                    if (!exp_match) begin
                        $error("[SB] valid_o for a non-matching address @ %0t (addr %h)", $time, exp_addr);
                        ok = 0;
                    end
                    else if (tr.rx_data !== exp_data) begin
                        $error("[SB] rx_data_o mismatch @ %0t: got %h, expected %h",
                               $time, tr.rx_data, exp_data);
                        ok = 0;
                    end
                    else
                        valid_ok++;
                end
            end

            if (in_txn) begin
                if (tr.busy === 1'b1) begin
                    if (busy_len == 39 * CLK_DIV) exp_data = tr.data;
                    busy_len++;
                    just_started = 1'b0;
                end
                else if (just_started) begin
                    $error("[SB] busy_o did not rise after start_i @ %0t", $time);
                    ok       = 0;
                    in_txn   = 1'b0;
                end
                else begin
                    if (!finish_txn(tr)) ok = 0;
                    in_txn       = 1'b0;
                    finished_now = 1'b1;
                end
            end

            if (in_txn) begin
                if (start_cond) begin
                    starts_seen++;
                    clk_cnt = 0;
                    bits_s  = '0;
                end
                else if (scl_rise && starts_seen > 0) begin
                    if (clk_cnt < 18) bits_s[clk_cnt] = tr.sda;
                    clk_cnt++;
                end
                if (stop_cond) stops_seen++;
                if (tr.start === 1'b1) start_ignored++;
            end

            if (tr.busy !== 1'b1 && tr.start === 1'b1 && !in_txn) begin
                in_txn       = 1'b1;
                just_started = 1'b1;
                busy_len     = 0;
                valid_cnt    = 0;
                starts_seen  = 0;
                stops_seen   = 0;
                clk_cnt      = 0;
                bits_s       = '0;
                exp_addr     = tr.addr;
                exp_data     = tr.data;
                exp_match    = (tr.addr == SLAVE_ADDR);
                txns++;
                if (finished_now) back_to_back++;

                diff = tr.addr ^ SLAVE_ADDR;
                if ($countones(diff) == 1)
                    for (int i = 0; i < 7; i++)
                        if (diff[i]) diff_bit[i]++;
            end
        end

        if (tr.busy !== 1'b1 && !in_txn) begin
            if (tr.scl !== 1'b1 || tr.sda !== 1'b1) begin
                $error("[SB] bus not idle @ %0t: scl %0b, sda %0b", $time, tr.scl, tr.sda);
                ok = 0;
            end
        end

        prev_scl = tr.scl;
        prev_sda = tr.sda;

        checked++;
        if (!ok) errors++;
    endfunction

    function void report();
        $display("---------------- scoreboard ----------------");
        $display("cycles checked          : %0d", checked);
        $display("errors                  : %0d", errors);
        $display("transfers started       : %0d", txns);
        $display("transfers completed     : %0d", completed);
        $display("slave address used      : %0d", match_txns);
        $display("other address used      : %0d", miss_txns);
        $display("address ACKed on bus    : %0d", acked);
        $display("address NACKed on bus   : %0d", nacked);
        $display("data byte correct       : %0d", data_ok);
        $display("data byte ACKed         : %0d", data_acked);
        $display("valid_o with right data : %0d", valid_ok);
        $display("start_i ignored (busy)  : %0d", start_ignored);
        $display("back-to-back transfers  : %0d", back_to_back);
        $display("extra STOP conditions   : %0d", extra_stops);
        $display("transfers with extra SCL: %0d", extra_clocks);
        $display("resets mid-transfer     : %0d", resets_mid_txn);
        $display("resets while idle       : %0d", resets_idle);
        for (int i = 0; i < 7; i++)
            $display("1-bit address diff, bit %0d: %0d", i, diff_bit[i]);
        $display("--------------------------------------------");

        if (completed == 0)
            $warning("[SB] no transfer was completed");
        if (match_txns == 0 || miss_txns == 0)
            $warning("[SB] both the slave address and other addresses must be used");
        if (acked == 0)
            $warning("[SB] the slave never ACKed an address");
        if (nacked == 0)
            $warning("[SB] the slave never NACKed an address");
        if (data_ok == 0 || data_acked == 0 || valid_ok == 0)
            $warning("[SB] no data byte was transferred, ACKed and reported by valid_o");
        for (int i = 0; i < 7; i++)
            if (diff_bit[i] == 0)
                $warning("[SB] address differing only in bit %0d was never used", i);
        if (start_ignored == 0)
            $warning("[SB] start_i while busy was never applied");
        if (back_to_back == 0)
            $warning("[SB] a transfer started right after the previous one was never applied");
        if (resets_mid_txn == 0)
            $warning("[SB] reset was never applied in the middle of a transfer");
        if (resets_idle == 0)
            $warning("[SB] reset was never applied while idle");
        if (extra_stops != 0)
            $warning("[SB] SDA rose while SCL was high before the final STOP in %0d transfers", extra_stops);
        if (extra_clocks != 0)
            $warning("[SB] extra SCL pulses after the last ACK clock in %0d transfers", extra_clocks);
    endfunction
endclass

// ---------------------------------------------------------------------------
// Monitor
// ---------------------------------------------------------------------------
class I2cMonitor;
    virtual i2c_if vif;
    I2cScoreboard  sb;

    function new(virtual i2c_if vif, I2cScoreboard sb);
        this.vif = vif;
        this.sb  = sb;
    endfunction

    task run();
        I2cTxn tr;
        forever begin
            @(vif.mon_cb);
            tr           = new();
            tr.rst_n     = vif.mon_cb.rst_n;
            tr.start     = vif.mon_cb.start;
            tr.addr      = vif.mon_cb.addr;
            tr.data      = vif.mon_cb.data;
            tr.busy      = vif.mon_cb.busy;
            tr.ack_error = vif.mon_cb.ack_error;
            tr.rx_data   = vif.mon_cb.rx_data;
            tr.valid     = vif.mon_cb.valid;
            tr.scl       = vif.mon_cb.scl;
            tr.sda       = vif.mon_cb.sda;
            sb.check(tr);
        end
    endtask
endclass

// ---------------------------------------------------------------------------
// Driver
// ---------------------------------------------------------------------------
class I2cDriver;
    virtual i2c_if vif;

    function new(virtual i2c_if vif);
        this.vif = vif;
    endfunction

    task drive(I2cTxn tr);
        @(negedge vif.clk);
        vif.rst_n <= tr.rst_n;
        vif.start <= tr.start;
        vif.addr  <= tr.addr;
        vif.data  <= tr.data;
    endtask
endclass

// ---------------------------------------------------------------------------
// Generator
// ---------------------------------------------------------------------------
class I2cGenerator;
    I2cDriver drv;
    int       num_random;

    bit [6:0] la = 7'h00;
    bit [7:0] ld = 8'h00;

    function new(I2cDriver drv, int num_random);
        this.drv        = drv;
        this.num_random = num_random;
    endfunction

    task step(bit rst_n, bit start, bit [6:0] addr, bit [7:0] data);
        I2cTxn tr;
        tr       = new();
        tr.rst_n = rst_n;
        tr.start = start;
        tr.addr  = addr;
        tr.data  = data;
        la = addr;
        ld = data;
        drv.drive(tr);
    endtask

    task calm(int n = 1);
        repeat (n) step(1'b1, 1'b0, la, ld);
    endtask

    task xfer(bit [6:0] addr, bit [7:0] data, int tail);
        step(1'b1, 1'b1, addr, data);
        calm(tail);
    endtask

    task directed();
        bit [6:0] a;
        bit [7:0] d;
        int       pts [9] = '{10, 100, 340, 355, 385, 450, 700, 725, 770};

        step(1'b0, 1'b0, 7'h00, 8'h00);
        step(1'b0, 1'b0, 7'h00, 8'h00);
        step(1'b1, 1'b0, 7'h00, 8'h00);
        calm(20);

        xfer(SLAVE_ADDR, 8'hA5, LEN_ACK + 10);
        xfer(SLAVE_ADDR, 8'h00, LEN_ACK + 10);
        xfer(SLAVE_ADDR, 8'hFF, LEN_ACK + 10);
        xfer(SLAVE_ADDR, 8'h5A, LEN_ACK + 10);
        for (int b = 0; b < 8; b++) begin
            d = 8'd1 << b;
            xfer(SLAVE_ADDR, d, LEN_ACK + 10);
        end

        xfer(7'h00, 8'hA5, LEN_NACK + 10);
        xfer(7'h7F, 8'hA5, LEN_NACK + 10);
        for (int b = 0; b < 7; b++) begin
            a = SLAVE_ADDR;
            a[b] = ~a[b];
            xfer(a, 8'h3C, LEN_NACK + 10);
        end
        xfer(7'h74, 8'hA5, LEN_NACK + 10);
        xfer(7'h1D, 8'hA5, LEN_NACK + 10);

        repeat (3 * LEN_ACK) step(1'b1, 1'b1, SLAVE_ADDR, 8'h69);
        calm(LEN_ACK + 10);
        repeat (3 * LEN_NACK) step(1'b1, 1'b1, 7'h11, 8'h22);
        calm(LEN_NACK + 10);

        step(1'b1, 1'b1, SLAVE_ADDR, 8'h77);
        calm(100);
        step(1'b1, 1'b1, 7'h11, 8'h77);
        calm(LEN_ACK + 10);

        step(1'b1, 1'b1, SLAVE_ADDR, 8'hC3);
        calm(100);
        step(1'b1, 1'b0, SLAVE_ADDR, 8'h3C);
        calm(349);
        step(1'b1, 1'b0, SLAVE_ADDR, 8'hF0);
        calm(LEN_ACK);

        foreach (pts[i]) begin
            step(1'b1, 1'b1, SLAVE_ADDR, 8'hA5);
            calm(pts[i]);
            step(1'b0, 1'b0, SLAVE_ADDR, 8'hA5);
            calm(30);
            xfer(SLAVE_ADDR, 8'h5A, LEN_ACK + 10);
        end

        step(1'b0, 1'b1, SLAVE_ADDR, 8'hA5);
        calm(5);
        xfer(SLAVE_ADDR, 8'h96, LEN_ACK + 10);
    endtask

    task random_run();
        I2cTxn tr;
        int style = 0;
        int left  = 0;

        repeat (num_random) begin
            if (left == 0) begin
                style = $urandom_range(0, 3);
                left  = 1000;
            end
            left--;

            tr = new();
            case (style)
                0: begin
                    tr.rst_w = 1; tr.start_w = 50;  tr.slave_w = 500;
                end
                1: begin
                    tr.rst_w = 2; tr.start_w = 200; tr.slave_w = 800;
                end
                2: begin
                    tr.rst_w = 1; tr.start_w = 20;  tr.slave_w = 200;
                end
                default: begin
                    tr.rst_w = 3; tr.start_w = 100; tr.slave_w = 500;
                end
            endcase

            if (!tr.randomize())
                $fatal(1, "[GEN] randomization failed");
            la = tr.addr;
            ld = tr.data;
            drv.drive(tr);
        end
    endtask
endclass

// ---------------------------------------------------------------------------
// Environment
// ---------------------------------------------------------------------------
class I2cEnvironment;
    I2cGenerator  gen;
    I2cDriver     drv;
    I2cMonitor    mon;
    I2cScoreboard sb;

    virtual i2c_if vif;

    function new(virtual i2c_if vif, int num_random = 20000);
        this.vif = vif;
        sb  = new();
        drv = new(vif);
        mon = new(vif, sb);
        gen = new(drv, num_random);
    endfunction

    task run();
        fork
            mon.run();
        join_none

        gen.directed();
        gen.random_run();

        repeat (2) @(posedge vif.clk);
        #1;
        sb.report();

        if (sb.errors != 0)
            $fatal(1, "[ENV] TEST FAILED (%0d errors)", sb.errors);
        else
            $display("[ENV] TEST PASSED");
    endtask
endclass

// ---------------------------------------------------------------------------
// Top
// ---------------------------------------------------------------------------
module tb_i2c;

    logic clk = 1'b0;
    always #5 clk = ~clk;

    tri1 scl;
    tri1 sda;

    i2c_if vif (clk, scl, sda);

    i2c_master #(.CLK_DIV(CLK_DIV)) dut_master (
        .clk         (clk),
        .rst_n       (vif.rst_n),
        .start_i     (vif.start),
        .addr_i      (vif.addr),
        .data_i      (vif.data),
        .busy_o      (vif.busy),
        .ack_error_o (vif.ack_error),
        .scl_io      (scl),
        .sda_io      (sda)
    );

    i2c_slave #(.SLAVE_ADDR(SLAVE_ADDR)) dut_slave (
        .clk       (clk),
        .rst_n     (vif.rst_n),
        .rx_data_o (vif.rx_data),
        .valid_o   (vif.valid),
        .scl_io    (scl),
        .sda_io    (sda)
    );

    I2cEnvironment env;

    initial begin
        $dumpfile("i2c_waveform.vcd");
        $dumpvars(0, tb_i2c);
        #5us;
        $dumpoff;
    end

    initial begin
        vif.rst_n = 1'b0;
        vif.start = 1'b0;
        vif.addr  = 7'h00;
        vif.data  = 8'h00;

        repeat (2) @(negedge clk);

        env = new(vif);
        env.run();
        $finish;
    end

    initial begin
        #10ms;
        $fatal(1, "[TB] timeout");
    end
endmodule //The last line must be empty so vscode doesn't give a mistake

