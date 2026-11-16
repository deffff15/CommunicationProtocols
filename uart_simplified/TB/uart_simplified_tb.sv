`timescale 1ns/1ps

// ---------------------------------------------------------------------------
// Parameters
// ---------------------------------------------------------------------------
package uart_tb_pkg;
    localparam int CLK_DIV   = 27;
    localparam int T_BIT     = 16 * CLK_DIV;
    localparam int FRAME_LEN = 10 * T_BIT;
    localparam int TAIL      = FRAME_LEN + 10;
endpackage

import uart_tb_pkg::*;

// ---------------------------------------------------------------------------
// Interface
// ---------------------------------------------------------------------------
interface uart_if (input logic clk, input wire tx, input wire rx_line);
    logic       rst_n;
    logic       start;
    logic [7:0] data;
    logic       sel;
    logic       rx_ext;
    logic       busy;
    logic [7:0] rx_data;
    logic       valid;
    logic       error;

    clocking mon_cb @(posedge clk);
        default input #1step;
        input rst_n, start, data, sel, rx_ext, busy, rx_data, valid, error, tx, rx_line;
    endclocking
endinterface

// ---------------------------------------------------------------------------
// Transaction
// ---------------------------------------------------------------------------
class UartTxn;
    bit       rst_n;
    bit       start;
    bit [7:0] data;
    bit       sel;
    bit       rx_ext;

    rand bit [2:0] kind;
    rand bit [7:0] rdata;
    rand int       glitch_len;
    rand int       reset_at;
    rand int       extra;

    int loop_w = 40;
    int ext_w  = 25;
    int err_w  = 10;
    int gl_w   = 10;
    int rst_w  = 15;

    logic       busy;
    logic [7:0] rx_data;
    logic       valid;
    logic       error;
    logic       tx;
    logic       rx_line;

    constraint c_op {
        kind dist {3'd0 := loop_w, 3'd1 := ext_w, 3'd2 := err_w, 3'd3 := gl_w, 3'd4 := rst_w};
        glitch_len inside {[1 : 6 * CLK_DIV]};
        reset_at   inside {[1 : FRAME_LEN]};
        extra      inside {[0 : 100]};
    }
endclass

// ---------------------------------------------------------------------------
// Decoder
// ---------------------------------------------------------------------------
class UartDecoder;
    bit       active = 1'b0;
    int       age    = 0;
    bit [7:0] byte_v = 8'h00;
    bit       stop_v = 1'b1;
    bit       prev   = 1'b1;
    bit       ev_start;
    bit       ev_false;
    bit       ev_done;
    bit       ev_end;

    function void reset(bit line);
        active   = 1'b0;
        age      = 0;
        prev     = line;
        ev_start = 1'b0;
        ev_false = 1'b0;
        ev_done  = 1'b0;
        ev_end   = 1'b0;
    endfunction

    function void sample(bit line);
        int j;

        ev_start = 1'b0;
        ev_false = 1'b0;
        ev_done  = 1'b0;
        ev_end   = 1'b0;

        if (!active) begin
            if (prev && !line) begin
                active   = 1'b1;
                age      = 0;
                ev_start = 1'b1;
            end
        end
        else begin
            age++;
            if (age == 8 * CLK_DIV) begin
                if (line) begin
                    ev_false = 1'b1;
                    active   = 1'b0;
                end
            end
            else if (age >= 24 * CLK_DIV && ((age - 24 * CLK_DIV) % T_BIT) == 0) begin
                j = (age - 24 * CLK_DIV) / T_BIT;
                if (j < 8)
                    byte_v[j] = line;
                else if (j == 8) begin
                    stop_v  = line;
                    ev_done = 1'b1;
                end
            end
            else if (age == 155 * CLK_DIV) begin
                active = 1'b0;
                ev_end = 1'b1;
            end
        end

        prev = line;
    endfunction
endclass

// ---------------------------------------------------------------------------
// Scoreboard
// ---------------------------------------------------------------------------
class UartScoreboard;
    UartDecoder tx_dec;
    UartDecoder rx_dec;

    bit         tx_pending;
    bit         tx_just;
    bit         tx_frame_checked;
    bit         fin_now;
    int         tx_busy_len;
    bit [7:0]   tx_exp;

    int         rx_valid_cnt;
    logic [7:0] rx_valid_data;
    bit         rx_sel_frame;

    int checked        = 0;
    int errors         = 0;
    int tx_txns        = 0;
    int tx_ok          = 0;
    int start_ignored  = 0;
    int back_to_back   = 0;
    int busy_off_nom   = 0;
    int busy_len_seen  = 0;
    int rx_frames      = 0;
    int rx_ok_loop     = 0;
    int rx_ok_ext      = 0;
    int rx_err_ok      = 0;
    int false_starts   = 0;
    int frames_loop    = 0;
    int frames_ext     = 0;
    int resets_mid_tx  = 0;
    int resets_mid_rx  = 0;
    int resets_idle    = 0;
    int bit_one [8];
    int bit_zero [8];

    function new();
        tx_dec = new();
        rx_dec = new();
        for (int i = 0; i < 8; i++) begin
            bit_one[i]  = 0;
            bit_zero[i] = 0;
        end
        reset_model(1'b1, 1'b1);
    endfunction

    function void reset_model(bit tx_line, bit rx_line);
        tx_pending       = 1'b0;
        tx_just          = 1'b0;
        tx_frame_checked = 1'b0;
        tx_busy_len      = 0;
        rx_valid_cnt     = 0;
        tx_dec.reset(tx_line);
        rx_dec.reset(rx_line);
    endfunction

    function bit finish_tx();
        bit ok = 1;
        int diff;

        if (!tx_frame_checked) begin
            $error("[SB] no frame decoded on tx_o @ %0t", $time);
            ok = 0;
        end

        busy_len_seen = tx_busy_len;
        diff = tx_busy_len - FRAME_LEN;
        if (diff < 0) diff = -diff;

        if (diff > 8 * CLK_DIV) begin
            $error("[SB] busy length mismatch @ %0t: got %0d cycles, expected about %0d",
                   $time, tx_busy_len, FRAME_LEN);
            ok = 0;
        end
        else if (tx_busy_len != FRAME_LEN)
            busy_off_nom++;

        if (ok) tx_ok++;
        return ok;
    endfunction

    function bit eval_rx(UartTxn tr);
        bit ok = 1;

        if (rx_dec.stop_v) begin
            if (rx_valid_cnt != 1) begin
                $error("[SB] valid_o pulses mismatch @ %0t: got %0d, expected 1 (byte %h)",
                       $time, rx_valid_cnt, rx_dec.byte_v);
                ok = 0;
            end
            else if (rx_valid_data !== rx_dec.byte_v) begin
                $error("[SB] rx_data_o mismatch @ %0t: got %h, expected %h",
                       $time, rx_valid_data, rx_dec.byte_v);
                ok = 0;
            end
            else begin
                if (rx_sel_frame) rx_ok_ext++;
                else              rx_ok_loop++;
            end

            if (tr.error !== 1'b0) begin
                $error("[SB] error_o set for a frame with a valid stop bit @ %0t", $time);
                ok = 0;
            end
        end
        else begin
            if (rx_valid_cnt != 0) begin
                $error("[SB] valid_o pulses for a framing error @ %0t: got %0d, expected 0",
                       $time, rx_valid_cnt);
                ok = 0;
            end

            if (tr.error !== 1'b1) begin
                $error("[SB] error_o not set for a framing error @ %0t", $time);
                ok = 0;
            end
            else
                rx_err_ok++;
        end

        return ok;
    endfunction

    function void check(UartTxn tr);
        bit ok = 1;
        bit txb;
        bit rxb;

        txb     = tr.tx;
        rxb     = tr.rx_line;
        fin_now = 1'b0;

        if (!tr.rst_n) begin
            if (tx_pending)    resets_mid_tx++;
            if (rx_dec.active) resets_mid_rx++;
            if (!tx_pending && !rx_dec.active) resets_idle++;
            reset_model(txb, rxb);

            if (tr.tx !== 1'b1) begin
                $error("[SB] tx_o not idle during reset @ %0t", $time);
                ok = 0;
            end
        end
        else begin
            if (tr.valid === 1'b1) begin
                if (!rx_dec.active) begin
                    $error("[SB] unexpected valid_o @ %0t", $time);
                    ok = 0;
                end
                else begin
                    rx_valid_cnt++;
                    rx_valid_data = tr.rx_data;
                end
            end

            tx_dec.sample(txb);
            rx_dec.sample(rxb);

            if (tx_dec.ev_false) begin
                $error("[SB] tx_o start bit shorter than half a bit @ %0t", $time);
                ok = 0;
            end

            if (tx_dec.ev_done) begin
                if (!tx_pending) begin
                    $error("[SB] unexpected frame on tx_o @ %0t", $time);
                    ok = 0;
                end
                else begin
                    tx_frame_checked = 1'b1;
                    if (tx_dec.byte_v !== tx_exp) begin
                        $error("[SB] tx_o data mismatch @ %0t: got %h, expected %h",
                               $time, tx_dec.byte_v, tx_exp);
                        ok = 0;
                    end
                    if (!tx_dec.stop_v) begin
                        $error("[SB] tx_o stop bit is low @ %0t", $time);
                        ok = 0;
                    end
                end
            end

            if (tx_pending) begin
                if (tr.busy === 1'b1) begin
                    if (tx_busy_len == 0 && tr.tx !== 1'b0) begin
                        $error("[SB] tx_o not low in the first busy cycle @ %0t", $time);
                        ok = 0;
                    end
                    tx_busy_len++;
                    tx_just = 1'b0;
                end
                else if (tx_just) begin
                    $error("[SB] busy_o did not rise after start_i @ %0t", $time);
                    ok         = 0;
                    tx_pending = 1'b0;
                end
                else begin
                    if (!finish_tx()) ok = 0;
                    tx_pending = 1'b0;
                    fin_now    = 1'b1;
                end
            end

            if (tr.start === 1'b1 && tr.busy === 1'b1)
                start_ignored++;

            if (tr.busy !== 1'b1 && tr.start === 1'b1 && !tx_pending) begin
                tx_pending       = 1'b1;
                tx_just          = 1'b1;
                tx_frame_checked = 1'b0;
                tx_busy_len      = 0;
                tx_exp           = tr.data;
                tx_txns++;
                if (fin_now) back_to_back++;
                for (int i = 0; i < 8; i++) begin
                    if (tr.data[i]) bit_one[i]++;
                    else            bit_zero[i]++;
                end
            end

            if (rx_dec.ev_start) begin
                rx_valid_cnt = 0;
                rx_sel_frame = tr.sel;
                rx_frames++;
                if (tr.sel) frames_ext++;
                else        frames_loop++;
            end

            if (rx_dec.ev_false)
                false_starts++;

            if (rx_dec.ev_end) begin
                if (!eval_rx(tr)) ok = 0;
            end

            if (tr.busy !== 1'b1 && !tx_pending && tr.tx !== 1'b1) begin
                $error("[SB] tx_o not idle @ %0t", $time);
                ok = 0;
            end
        end

        checked++;
        if (!ok) errors++;
    endfunction

    function void report();
        $display("---------------- scoreboard ----------------");
        $display("cycles checked          : %0d", checked);
        $display("errors                  : %0d", errors);
        $display("tx transfers started    : %0d", tx_txns);
        $display("tx transfers correct    : %0d", tx_ok);
        $display("start_i ignored (busy)  : %0d", start_ignored);
        $display("back-to-back tx         : %0d", back_to_back);
        $display("busy != 10 bit times    : %0d (last %0d, nominal %0d)", busy_off_nom, busy_len_seen, FRAME_LEN);
        $display("rx frames seen          : %0d", rx_frames);
        $display("  from tx (loopback)    : %0d", frames_loop);
        $display("  from external source  : %0d", frames_ext);
        $display("rx correct, loopback    : %0d", rx_ok_loop);
        $display("rx correct, external    : %0d", rx_ok_ext);
        $display("framing errors flagged  : %0d", rx_err_ok);
        $display("false starts ignored    : %0d", false_starts);
        $display("resets mid tx transfer  : %0d", resets_mid_tx);
        $display("resets mid rx frame     : %0d", resets_mid_rx);
        $display("resets while idle       : %0d", resets_idle);
        for (int i = 0; i < 8; i++)
            $display("tx data bit %0d: ones %0d, zeros %0d", i, bit_one[i], bit_zero[i]);
        $display("--------------------------------------------");

        if (tx_ok == 0)
            $warning("[SB] no tx transfer was completed correctly");
        if (rx_ok_loop == 0)
            $warning("[SB] no frame was received correctly in loopback");
        if (rx_ok_ext == 0)
            $warning("[SB] no frame from the external source was received correctly");
        if (rx_err_ok == 0)
            $warning("[SB] a framing error was never flagged correctly");
        if (false_starts == 0)
            $warning("[SB] a false start was never applied");
        for (int i = 0; i < 8; i++)
            if (bit_one[i] == 0 || bit_zero[i] == 0)
                $warning("[SB] tx data bit %0d was not sent as both 0 and 1", i);
        if (start_ignored == 0)
            $warning("[SB] start_i while busy was never applied");
        if (back_to_back == 0)
            $warning("[SB] a transfer started right after the previous one was never applied");
        if (resets_mid_tx == 0 || resets_mid_rx == 0)
            $warning("[SB] reset was not applied in the middle of a tx transfer and an rx frame");
        if (resets_idle == 0)
            $warning("[SB] reset was never applied while idle");
        if (busy_off_nom != 0)
            $warning("[SB] busy_o lasted %0d cycles instead of %0d (10 bit times) in %0d transfers",
                     busy_len_seen, FRAME_LEN, busy_off_nom);
    endfunction
endclass

// ---------------------------------------------------------------------------
// Monitor
// ---------------------------------------------------------------------------
class UartMonitor;
    virtual uart_if vif;
    UartScoreboard  sb;

    function new(virtual uart_if vif, UartScoreboard sb);
        this.vif = vif;
        this.sb  = sb;
    endfunction

    task run();
        UartTxn tr;
        forever begin
            @(vif.mon_cb);
            tr         = new();
            tr.rst_n   = vif.mon_cb.rst_n;
            tr.start   = vif.mon_cb.start;
            tr.data    = vif.mon_cb.data;
            tr.sel     = vif.mon_cb.sel;
            tr.rx_ext  = vif.mon_cb.rx_ext;
            tr.busy    = vif.mon_cb.busy;
            tr.rx_data = vif.mon_cb.rx_data;
            tr.valid   = vif.mon_cb.valid;
            tr.error   = vif.mon_cb.error;
            tr.tx      = vif.mon_cb.tx;
            tr.rx_line = vif.mon_cb.rx_line;
            sb.check(tr);
        end
    endtask
endclass

// ---------------------------------------------------------------------------
// Driver
// ---------------------------------------------------------------------------
class UartDriver;
    virtual uart_if vif;

    function new(virtual uart_if vif);
        this.vif = vif;
    endfunction

    task drive(UartTxn tr);
        @(negedge vif.clk);
        vif.rst_n  <= tr.rst_n;
        vif.start  <= tr.start;
        vif.data   <= tr.data;
        vif.sel    <= tr.sel;
        vif.rx_ext <= tr.rx_ext;
    endtask
endclass

// ---------------------------------------------------------------------------
// Generator
// ---------------------------------------------------------------------------
class UartGenerator;
    UartDriver drv;
    int        num_random;
    bit        use_ext = 1'b1;

    bit [7:0]  ld   = 8'h00;
    bit        lsel = 1'b0;

    function new(UartDriver drv, int num_random);
        this.drv        = drv;
        this.num_random = num_random;
    endfunction

    task step(bit rst_n, bit start, bit [7:0] data, bit sel, bit rx_ext);
        UartTxn tr;
        tr        = new();
        tr.rst_n  = rst_n;
        tr.start  = start;
        tr.data   = data;
        tr.sel    = sel;
        tr.rx_ext = rx_ext;
        ld   = data;
        lsel = sel;
        drv.drive(tr);
    endtask

    task calm(int n = 1);
        repeat (n) step(1'b1, 1'b0, ld, lsel, 1'b1);
    endtask

    task tx_send(bit [7:0] d, int tail);
        step(1'b1, 1'b1, d, 1'b0, 1'b1);
        calm(tail);
    endtask

    task ext_frame(bit [7:0] d, bit stop, int tail, int limit);
        bit v;
        int cnt = 0;

        for (int i = 0; i < 10; i++) begin
            if (i == 0)      v = 1'b0;
            else if (i == 9) v = stop;
            else             v = d[i - 1];

            repeat (T_BIT) begin
                if (limit >= 0 && cnt == limit) begin
                    step(1'b0, 1'b0, ld, 1'b1, 1'b1);
                    calm(30);
                    return;
                end
                step(1'b1, 1'b0, ld, 1'b1, v);
                cnt++;
            end
        end
        calm(tail);
    endtask

    task false_start(int len);
        repeat (len) step(1'b1, 1'b0, ld, 1'b1, 1'b0);
        calm(2 * T_BIT);
    endtask

    task directed();
        bit [7:0] d;
        bit [7:0] ext_bytes [8] = '{8'h00, 8'hFF, 8'h55, 8'hAA, 8'hA5, 8'h3C, 8'h81, 8'h7E};
        int       pts [6]       = '{50, 300, 1000, 2500, 3900, 4200};
        int       ext_pts [3]   = '{1000, 2500, 3900};

        step(1'b0, 1'b0, 8'h00, 1'b0, 1'b1);
        step(1'b0, 1'b0, 8'h00, 1'b0, 1'b1);
        step(1'b1, 1'b0, 8'h00, 1'b0, 1'b1);
        calm(20);

        tx_send(8'h00, TAIL);
        tx_send(8'hFF, TAIL);
        tx_send(8'h55, TAIL);
        tx_send(8'hAA, TAIL);
        tx_send(8'hA5, TAIL);
        tx_send(8'h5A, TAIL);
        tx_send(8'h81, TAIL);
        tx_send(8'h7E, TAIL);
        for (int b = 0; b < 8; b++) begin
            d = 8'd1 << b;
            tx_send(d, TAIL);
            d = ~d;
            tx_send(d, TAIL);
        end

        repeat (3 * FRAME_LEN) step(1'b1, 1'b1, 8'h3C, 1'b0, 1'b1);
        calm(TAIL);

        step(1'b1, 1'b1, 8'hC3, 1'b0, 1'b1);
        calm(100);
        step(1'b1, 1'b1, 8'h3C, 1'b0, 1'b1);
        calm(100);
        step(1'b1, 1'b0, 8'h96, 1'b0, 1'b1);
        calm(TAIL);

        foreach (pts[i]) begin
            step(1'b1, 1'b1, 8'hA5, 1'b0, 1'b1);
            calm(pts[i]);
            step(1'b0, 1'b0, 8'hA5, 1'b0, 1'b1);
            calm(30);
            tx_send(8'h5A, TAIL);
        end

        if (use_ext) begin
            foreach (ext_bytes[i])
                ext_frame(ext_bytes[i], 1'b1, 50, -1);

            ext_frame(8'hA5, 1'b0, 50, -1);
            ext_frame(8'h00, 1'b0, 50, -1);
            ext_frame(8'hFF, 1'b0, 50, -1);
            ext_frame(8'h5A, 1'b0, 50, -1);

            ext_frame(8'h12, 1'b1, 0, -1);
            ext_frame(8'h34, 1'b1, 0, -1);
            calm(50);

            false_start(1);
            false_start(CLK_DIV);
            false_start(3 * CLK_DIV);
            false_start(6 * CLK_DIV);
            ext_frame(8'hC3, 1'b1, 50, -1);

            foreach (ext_pts[i]) begin
                ext_frame(8'hA5, 1'b1, 50, ext_pts[i]);
                ext_frame(8'h5A, 1'b1, 50, -1);
            end
        end

        step(1'b0, 1'b1, 8'hA5, 1'b0, 1'b1);
        calm(2);
        tx_send(8'h96, TAIL);
    endtask

    task random_run();
        UartTxn tr;
        int     kind;

        repeat (num_random) begin
            tr = new();
            if (!tr.randomize())
                $fatal(1, "[GEN] randomization failed");

            kind = tr.kind;
            if (!use_ext && kind >= 1 && kind <= 3)
                kind = 0;

            case (kind)
                0: begin
                    tx_send(tr.rdata, FRAME_LEN + 10 + tr.extra);
                end
                1: begin
                    ext_frame(tr.rdata, 1'b1, tr.extra, -1);
                end
                2: begin
                    ext_frame(tr.rdata, 1'b0, tr.extra, -1);
                end
                3: begin
                    false_start(tr.glitch_len);
                end
                default: begin
                    step(1'b1, 1'b1, tr.rdata, 1'b0, 1'b1);
                    calm(tr.reset_at);
                    step(1'b0, 1'b0, tr.rdata, 1'b0, 1'b1);
                    calm(30);
                end
            endcase
        end
    endtask
endclass

// ---------------------------------------------------------------------------
// Environment
// ---------------------------------------------------------------------------
class UartEnvironment;
    UartGenerator  gen;
    UartDriver     drv;
    UartMonitor    mon;
    UartScoreboard sb;

    virtual uart_if vif;

    function new(virtual uart_if vif, int num_random = 40);
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
module tb_uart;

    logic clk = 1'b0;
    always #5 clk = ~clk;

    wire tx_w;
    wire rx_w;

    uart_if vif (clk, tx_w, rx_w);

    assign rx_w = vif.sel ? vif.rx_ext : tx_w;

    uart_tx #(.CLK_DIV(CLK_DIV)) dut_tx (
        .clk     (clk),
        .rst_n   (vif.rst_n),
        .start_i (vif.start),
        .data_i  (vif.data),
        .busy_o  (vif.busy),
        .tx_o    (tx_w)
    );

    uart_rx #(.CLK_DIV(CLK_DIV)) dut_rx (
        .clk       (clk),
        .rst_n     (vif.rst_n),
        .rx_data_o (vif.rx_data),
        .valid_o   (vif.valid),
        .error_o   (vif.error),
        .rx_i      (rx_w)
    );

    UartEnvironment env;

    initial begin
        $dumpfile("uart_waveform.vcd");
        $dumpvars(0, tb_uart);
        #5us;
        $dumpoff;
    end

    initial begin
        vif.rst_n  = 1'b0;
        vif.start  = 1'b0;
        vif.data   = 8'h00;
        vif.sel    = 1'b0;
        vif.rx_ext = 1'b1;

        repeat (2) @(negedge clk);

        env = new(vif);
        env.run();
        $finish;
    end

    initial begin
        #50ms;
        $fatal(1, "[TB] timeout");
    end
endmodule //The last line must be empty so vscode doesn't give a mistake

