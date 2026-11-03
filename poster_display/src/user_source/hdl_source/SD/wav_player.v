/*
WAV 解析与播放。扫到 RIFF/WAVE 后读 fmt 块取采样率、位宽、声道数，跳过 data
之前的所有块，再按 I2S 时序串行送出采样。

采样率与 48kHz 不同时用增量累加重采样，避免整除误差累积。
*/

module wav_player #(
    parameter [31:0] SCAN_FROM_SECTOR = 32'd0,
    parameter [31:0] SCAN_MAX_SECTOR  = 32'd1048575,
    parameter [8:0]  FIFO_HI          = 9'd360,
    parameter [31:0] HDR_MAX_BYTES    = 32'd8192
)(
    input  wire        clk,
    input  wire        rst,

    input  wire        sd_init_done,

    output reg         sd_sec_read,
    output reg  [31:0] sd_sec_read_addr,
    input  wire [7:0]  sd_sec_read_data,
    input  wire        sd_sec_read_data_valid,
    input  wire        sd_sec_read_end,

    input  wire        play_en,
    input  wire        count_req,
    input  wire        next_tgl,
    input  wire        prev_tgl,

    input  wire        aclk,
    input  wire        rd_req,
    output wire [15:0] rd_data,
    output wire        rd_empty,

    output reg  [3:0]  state_code,
    output reg         wav_found,
    output wire        count_busy,
    output wire [3:0]  trk_now,
    output wire [3:0]  trk_total,
    output reg  [8:0]  fifo_used,
    output reg  [31:0] dbg_data_ofs,
    output reg  [15:0] dbg_chn,
    output reg  [31:0] dbg_srate,
    output wire [8:0]  dbg_wptr,
    output wire [8:0]  dbg_rptr,
    output wire [7:0]  dbg_pop_k_o,
    output wire [8:0]  dbg_used_o
);

    assign dbg_wptr = wptr;
    assign dbg_rptr = rptr;
    assign trk_now   = {1'b0, trk_idx};
    assign trk_total = {1'b0, trk_cnt};

    reg [15:0] mem [0:511];
    reg [8:0]  wptr;
    reg [8:0]  rptr;
    reg [8:0]  wptr_s1, wptr_s2;
    reg [8:0]  rptr_s1, rptr_s2;

    wire [8:0] used      = wptr - rptr_s2;
    wire       fifo_full = (used >= 9'd496);

    reg [15:0] rd_data_r;
    reg        wr_en;
    reg [15:0] wr_sample;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            wptr <= 9'd0;
            rptr_s1 <= 9'd0;
            rptr_s2 <= 9'd0;
        end else begin
            rptr_s1 <= rptr;
            rptr_s2 <= rptr_s1;
            if (wr_en && !fifo_full) begin
                mem[wptr] <= wr_sample;
                wptr      <= wptr + 9'd1;
            end
        end
    end

    reg [7:0] rd_div;
    reg [8:0] rptr_prev;
    wire      rd_tick = (rd_div == 8'd255);

    always @(posedge aclk or posedge rst) begin
        if (rst) begin
            rptr      <= 9'd0;
            rd_div    <= 8'd0;
            wptr_s1   <= 9'd0;
            wptr_s2   <= 9'd0;
            rd_data_r <= 16'd0;
        end else begin
            wptr_s1   <= wptr;
            wptr_s2   <= wptr_s1;
            rd_data_r <= mem[rptr];
            rd_div    <= rd_tick ? 8'd0 : (rd_div + 8'd1);
            if (rd_tick && (rptr != wptr_s2))
                rptr <= rptr + 9'd1;
        end
    end

    assign rd_data  = rd_data_r;
    assign rd_empty = (rptr == wptr_s2);

    always @(posedge clk or posedge rst) begin
        if (rst) fifo_used <= 9'd0;
        else     fifo_used <= used;
    end

    reg [26:0] win_cnt;
    reg [15:0] wr_acc, pop_acc;
    reg [7:0]  dbg_wr_k, dbg_pop_k;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            win_cnt    <= 27'd0;
            wr_acc     <= 16'd0;
            pop_acc    <= 16'd0;
            rptr_prev  <= 9'd0;
            dbg_wr_k   <= 8'd0;
            dbg_pop_k  <= 8'd0;
        end else begin
            if (wr_en && !fifo_full) wr_acc <= wr_acc + 16'd1;

            if (rptr_s2 != rptr_prev) begin
                pop_acc   <= pop_acc + {7'b0, rptr_s2} - {7'b0, rptr_prev};
                rptr_prev <= rptr_s2;
            end
            if (win_cnt == 27'd99_999_999) begin
                win_cnt   <= 27'd0;
                dbg_wr_k  <= wr_acc[15:10];
                dbg_pop_k <= pop_acc[15:10];
                wr_acc    <= 16'd0;
                pop_acc   <= 16'd0;
            end else begin
                win_cnt <= win_cnt + 27'd1;
            end
        end
    end

    assign dbg_pop_k_o  = dbg_pop_k;
    assign dbg_used_o   = used;

    localparam [3:0] S_IDLE       = 4'd0,
                     S_SCAN_REQ   = 4'd1,
                     S_SCAN       = 4'd2,
                     S_HDR_REQ    = 4'd3,
                     S_HDR        = 4'd4,
                     S_WAIT_LO    = 4'd5,
                     S_STREAM_REQ = 4'd6,
                     S_STREAM     = 4'd7,
                     S_LOOP       = 4'd8,
                     S_VERIFY_REQ = 4'd9,
                     S_VERIFY     = 4'd10;

    reg [3:0]  state;
    reg [31:0] scan_sec;
    reg [31:0] data_sec;
    reg [31:0] data_remain;
    reg [31:0] hdr_sec;
    reg [31:0] data_start;
    reg [31:0] dsize;
    reg [31:0] file_ofs;

    reg [9:0]  bcnt;
    reg [3:0]  scan_stride;
    reg        scan_pass2;

    reg        scan_win;
    reg [11:0] scan_cnt;

    reg        disc_pend;

    reg  [1:0] cr_s;
    reg        cr_d;
    reg        cnt_disc;
    reg        cnt_done;
    wire       cr_evt  = cr_s[1] & ~cr_d;
    wire       cnt_go  = cr_evt && !play_en && !cnt_done && !disc_pend && (state == S_IDLE);

    wire       run_en  = play_en | cnt_disc | cnt_go;

    assign count_busy = cnt_disc;

    reg        resume_valid;
    reg [31:0] ri_sec_r;
    reg [31:0] dstart_r;
    reg [31:0] dsize_r;
    reg [15:0] chn_r;

    reg  [31:0] end_sec_r;
    reg  [31:0] goto_sec;
    reg         goto_pend;
    reg         seek_prev;
    reg  [31:0] prev_cand;
    reg         prev_hit;
    reg  [1:0]  nt_s, pt_s;

    localparam [2:0] TRK_N = 3'd4;
    reg [31:0] trk_sec [0:3];
    reg [2:0]  trk_cnt;
    reg [2:0]  trk_idx;
    reg        trk_full;

    wire trk_next = nt_s[1] ^ nt_s[0];
    wire trk_prev = pt_s[1] ^ pt_s[0];
    wire trk_chg  = trk_next | trk_prev;

    integer k;
    reg        trk_req;
    reg [31:0] trk_req_sec;
    reg        trk_clr;
    reg        trk_go1;
    reg [1:0]  reg_st;
    reg [1:0]  reg_i;
    reg [31:0] reg_sec;
    reg        reg_hit;
    reg [1:0]  reg_hit_i;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            trk_cnt   <= 3'd0;
            trk_idx   <= 3'd0;
            trk_full  <= 1'b0;
            reg_st    <= 2'd0;
            reg_i     <= 2'd0;
            reg_sec   <= 32'd0;
            reg_hit   <= 1'b0;
            reg_hit_i <= 2'd0;
            for (k = 0; k < 4; k = k + 1) trk_sec[k] <= 32'd0;
        end else if (trk_clr) begin
            trk_cnt  <= 3'd0;
            trk_idx  <= 3'd0;
            trk_full <= 1'b0;
            reg_st   <= 2'd0;
        end else if (trk_go1) begin

            trk_idx <= 3'd1;
            reg_st  <= 2'd0;
        end else case (reg_st)
            2'd0: if (trk_req) begin
                reg_sec   <= trk_req_sec;
                reg_i     <= 2'd0;
                reg_hit   <= 1'b0;
                reg_hit_i <= 2'd0;
                reg_st    <= 2'd1;
            end
            2'd1: begin
                if (!reg_hit && (reg_i < trk_cnt) && (trk_sec[reg_i] == reg_sec)) begin
                    reg_hit   <= 1'b1;
                    reg_hit_i <= reg_i;
                end
                if (reg_i == 2'd3) reg_st <= 2'd2;
                else               reg_i  <= reg_i + 2'd1;
            end
            default: begin
                if (reg_hit)
                    trk_idx <= reg_hit_i + 3'd1;
                else if (trk_cnt != TRK_N) begin
                    trk_sec[trk_cnt[1:0]] <= reg_sec;
                    trk_cnt <= trk_cnt + 3'd1;
                    trk_idx <= trk_cnt + 3'd1;
                end else begin
                    trk_full <= 1'b1;
                end
                reg_st <= 2'd0;
            end
        endcase
    end

    wire tab_ok        = (trk_cnt != 3'd0) && !trk_full;
    wire trk_prev_fast = tab_ok && (trk_idx >  3'd1);

    wire trk_ok2   = tab_ok && (trk_cnt >= 3'd2);
    wire [1:0] trk_nxt_i = (trk_idx >= trk_cnt) ? 2'd0 : trk_idx[1:0];

    reg        trk_pend;
    reg [31:0] sig;
    reg [1:0]  ds_cnt;
    reg        ds_get;
    reg [15:0] chn;
    reg [31:0] srate;
    reg [15:0] bits;
    reg [7:0]  asm_lo;
    reg        asm_ph;
    reg        ch_tog;
    reg        riff_seen;
    reg        wave_seen;
    reg        data_seen;

    wire sr_ok  = (srate == 32'd48000) || (srate == 32'd44100);

    wire ds_ok  = (dsize != 32'd0) && (dsize < 32'd536870912);
    wire fmt_ok = (bits == 16'd16) && ((chn == 16'd1) || (chn == 16'd2)) && sr_ok && ds_ok;

    wire [31:0] sig_next = {sig[23:0], sd_sec_read_data};

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            nt_s <= 2'b00;
            pt_s <= 2'b00;
        end else begin
            nt_s <= {nt_s[0], next_tgl};
            pt_s <= {pt_s[0], prev_tgl};
        end
    end

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state            <= S_IDLE;
            sd_sec_read      <= 1'b0;
            sd_sec_read_addr <= 32'd0;
            scan_sec         <= SCAN_FROM_SECTOR;
            data_sec         <= 32'd0;
            data_remain      <= 32'd0;
            hdr_sec          <= 32'd0;
            data_start       <= 32'd0;
            dsize            <= 32'd0;
            file_ofs         <= 32'd0;
            bcnt             <= 10'd0;
            sig              <= 32'd0;
            ds_cnt           <= 2'd0;
            ds_get           <= 1'b0;
            chn              <= 16'd0;
            srate            <= 32'd0;
            bits             <= 16'd0;
            asm_lo           <= 8'd0;
            asm_ph           <= 1'b0;
            ch_tog           <= 1'b0;
            wr_en            <= 1'b0;
            wr_sample        <= 16'd0;
            wav_found        <= 1'b0;
            state_code       <= 4'd0;
            dbg_data_ofs     <= 32'd0;
            dbg_chn          <= 16'd0;
            dbg_srate        <= 32'd0;
            riff_seen        <= 1'b0;
            wave_seen        <= 1'b0;
            data_seen        <= 1'b0;
            scan_stride      <= 4'd8;
            scan_pass2       <= 1'b0;
            scan_win         <= 1'b0;
            scan_cnt         <= 12'd0;
            disc_pend        <= 1'b0;
            trk_go1          <= 1'b0;
            resume_valid     <= 1'b0;
            ri_sec_r         <= 32'd0;
            dstart_r         <= 32'd0;
            dsize_r          <= 32'd0;
            chn_r            <= 16'd0;
            end_sec_r        <= 32'd0;
            goto_sec         <= 32'd0;
            goto_pend        <= 1'b0;
            seek_prev        <= 1'b0;
            prev_cand        <= 32'd0;
            prev_hit         <= 1'b0;
            trk_pend         <= 1'b0;
            cr_s             <= 2'b00;
            cr_d             <= 1'b0;
            cnt_disc         <= 1'b0;
            cnt_done         <= 1'b0;
        end else begin
            wr_en      <= 1'b0;
            state_code <= state;
            trk_req    <= 1'b0;
            trk_clr    <= 1'b0;
            trk_go1    <= 1'b0;

            if (trk_chg) trk_pend <= 1'b1;

            cr_s <= {cr_s[0], count_req};
            cr_d <= cr_s[1];

            if (cnt_go) begin
                cnt_disc    <= 1'b1;
                disc_pend   <= 1'b1;
                scan_sec    <= SCAN_FROM_SECTOR;
                scan_stride <= 4'd8;
                scan_win    <= 1'b0;
                scan_cnt    <= 12'd0;
                scan_pass2  <= 1'b0;
                goto_pend   <= 1'b0;
                state       <= S_SCAN_REQ;
            end

            if (!run_en) begin

                state       <= S_IDLE;
                sd_sec_read <= 1'b0;
                wav_found   <= 1'b0;
                scan_win    <= 1'b0;
                disc_pend   <= 1'b0;
                trk_pend    <= 1'b0;
                cnt_disc    <= 1'b0;
            end else if (!sd_init_done && (state != S_IDLE)) begin

                state       <= S_IDLE;
                sd_sec_read <= 1'b0;
                wav_found   <= 1'b0;
                scan_win    <= 1'b0;
                disc_pend   <= 1'b0;
                trk_pend    <= 1'b0;
            end else if (trk_pend && !sd_sec_read) begin

                trk_pend     <= 1'b0;
                wav_found    <= 1'b0;
                resume_valid <= 1'b0;
                goto_pend    <= 1'b1;

                goto_sec     <= trk_ok2        ? trk_sec[trk_nxt_i]
                              : trk_prev_fast  ? trk_sec[trk_idx[1:0] - 2'd2]
                              : (trk_prev ? SCAN_FROM_SECTOR : end_sec_r);
                seek_prev    <= trk_prev & ~trk_prev_fast & ~trk_ok2;

                scan_win     <= !trk_prev && !trk_ok2;
                scan_cnt     <= 12'd0;
                prev_hit     <= 1'b0;
                disc_pend    <= 1'b0;

                scan_stride  <= 4'd1;
                scan_pass2   <= 1'b0;
                state        <= S_IDLE;
            end else case (state)

            S_IDLE: begin

                if (sd_init_done && play_en) begin

                    if (resume_valid) begin
                        bcnt      <= 10'd0;
                        sig       <= 32'd0;
                        riff_seen <= 1'b0;
                        wave_seen <= 1'b0;
                        state     <= S_VERIFY_REQ;
                    end else begin

                        scan_sec    <= goto_sec;
                        goto_pend   <= 1'b0;
                        state       <= S_SCAN_REQ;
                    end
                end
            end

            S_VERIFY_REQ: begin
                sd_sec_read_addr <= ri_sec_r;
                sd_sec_read      <= 1'b1;
                state            <= S_VERIFY;
            end

            S_VERIFY: begin
                if (sd_sec_read_data_valid) begin
                    if (bcnt <= 10'd3)                   sig  <= sig_next;
                    if (bcnt == 10'd3)                   riff_seen <= (sig_next == 32'h52494646);
                    if (bcnt >= 10'd8 && bcnt <= 10'd11) sig  <= sig_next;
                    if (bcnt == 10'd11)                  wave_seen <= (sig_next == 32'h57415645);
                    bcnt <= bcnt + 10'd1;
                end
                if (sd_sec_read_end) begin
                    sd_sec_read <= 1'b0;
                    if (riff_seen && wave_seen) begin

                        scan_sec    <= ri_sec_r;
                        data_start  <= dstart_r;
                        dsize       <= dsize_r;
                        chn         <= chn_r;
                        data_sec    <= ri_sec_r;
                        file_ofs    <= 32'd0;
                        data_remain <= dsize_r;
                        asm_ph      <= 1'b0;
                        ch_tog      <= 1'b0;
                        wav_found   <= 1'b1;
                        end_sec_r   <= ri_sec_r + ((dstart_r + dsize_r) >> 9);
                        trk_go1     <= 1'b1;
                        state       <= S_STREAM_REQ;
                    end else begin

                        resume_valid <= 1'b0;
                        trk_clr      <= 1'b1;
                        cnt_done     <= 1'b0;
                        disc_pend    <= 1'b0;
                        scan_sec     <= SCAN_FROM_SECTOR;
                        scan_stride  <= 4'd8;
                        scan_pass2   <= 1'b0;

                        scan_win     <= 1'b0;
                        scan_cnt     <= 12'd0;
                        state        <= S_SCAN_REQ;
                    end
                end
            end

            S_SCAN_REQ: begin
                sd_sec_read_addr <= scan_sec;
                bcnt             <= 10'd0;
                sig              <= 32'd0;
                riff_seen        <= 1'b0;
                wave_seen        <= 1'b0;
                sd_sec_read      <= 1'b1;

                if (scan_win) scan_cnt <= scan_cnt + 12'd1;
                state            <= S_SCAN;
            end

            S_SCAN: begin
                if (sd_sec_read_data_valid) begin
                    if (bcnt <= 10'd3)                   sig  <= sig_next;
                    if (bcnt == 10'd3)                   riff_seen <= (sig_next == 32'h52494646);
                    if (bcnt >= 10'd8 && bcnt <= 10'd11) sig  <= sig_next;
                    if (bcnt == 10'd11)                  wave_seen <= (sig_next == 32'h57415645);
                    bcnt <= bcnt + 10'd1;
                end
                if (sd_sec_read_end) begin
                    sd_sec_read <= 1'b0;
                    if (riff_seen && wave_seen && seek_prev) begin

                        if (scan_sec < ri_sec_r) begin
                            prev_cand <= scan_sec;
                            prev_hit  <= 1'b1;
                        end
                        scan_sec <= scan_sec + scan_stride;
                        state    <= S_SCAN_REQ;
                    end else if (seek_prev && (scan_sec >= ri_sec_r)) begin

                        scan_sec  <= prev_hit ? prev_cand : SCAN_FROM_SECTOR;
                        seek_prev <= 1'b0;
                        prev_hit  <= 1'b0;
                        state     <= S_SCAN_REQ;
                    end else if (riff_seen && wave_seen) begin

                        hdr_sec    <= 32'd0;
                        file_ofs   <= 32'd0;
                        sig        <= 32'd0;
                        ds_get     <= 1'b0;
                        data_seen  <= 1'b0;
                        chn        <= 16'd0;
                        srate      <= 32'd0;
                        bits       <= 16'd0;
                        state      <= S_HDR_REQ;
                    end else if (scan_win && (scan_cnt >= 12'd2047)) begin

                        scan_win <= 1'b0;
                        if (disc_pend) begin

                            disc_pend <= 1'b0;
                            cnt_disc  <= 1'b0;
                            cnt_done  <= 1'b1;
                            trk_go1   <= 1'b1;
                            goto_pend <= 1'b0;
                            state     <= S_IDLE;
                        end else if (tab_ok) begin

                            scan_sec <= trk_sec[0];
                            state    <= S_SCAN_REQ;
                        end else begin
                            state <= S_IDLE;
                        end
                    end else if (scan_sec + scan_stride > SCAN_MAX_SECTOR) begin

                        if (disc_pend) begin

                            disc_pend <= 1'b0;
                            cnt_disc  <= 1'b0;
                            cnt_done  <= 1'b1;
                            trk_go1   <= 1'b1;
                            scan_win  <= 1'b0;
                            goto_pend <= 1'b0;
                            state     <= S_IDLE;
                        end else if (!scan_pass2) begin
                            scan_pass2  <= 1'b1;
                            scan_stride <= 4'd1;
                            scan_sec    <= SCAN_FROM_SECTOR;
                            state       <= S_SCAN_REQ;
                        end else begin
                            disc_pend <= 1'b0;
                            cnt_disc  <= 1'b0;
                            cnt_done  <= 1'b1;
                            scan_win  <= 1'b0;
                            scan_cnt  <= 12'd0;
                            state <= S_IDLE;
                        end
                    end else begin
                        scan_sec <= scan_sec + scan_stride;
                        state    <= S_SCAN_REQ;
                    end
                end
            end

            S_HDR_REQ: begin
                sd_sec_read_addr <= scan_sec + hdr_sec;
                bcnt             <= 10'd0;
                sd_sec_read      <= 1'b1;
                state            <= S_HDR;
            end

            S_HDR: begin
                if (sd_sec_read_data_valid) begin
                    sig <= sig_next;
                    if (sig_next == 32'h64617461) begin
                        ds_get       <= 1'b1;
                        ds_cnt       <= 2'd0;
                        data_start   <= file_ofs + 32'd5;
                        dbg_data_ofs <= file_ofs + 32'd5;
                    end
                    if (ds_get) begin
                        case (ds_cnt)
                            2'd0: dsize[7:0]   <= sd_sec_read_data;
                            2'd1: dsize[15:8]  <= sd_sec_read_data;
                            2'd2: dsize[23:16] <= sd_sec_read_data;
                            default: dsize[31:24] <= sd_sec_read_data;
                        endcase
                        if (ds_cnt == 2'd3) begin
                            ds_get    <= 1'b0;
                            data_seen <= 1'b1;
                        end else begin
                            ds_cnt <= ds_cnt + 2'd1;
                        end
                    end

                    case (file_ofs)
                        32'd22: chn[7:0]     <= sd_sec_read_data;
                        32'd23: chn[15:8]    <= sd_sec_read_data;
                        32'd24: srate[7:0]   <= sd_sec_read_data;
                        32'd25: srate[15:8]  <= sd_sec_read_data;
                        32'd26: srate[23:16] <= sd_sec_read_data;
                        32'd27: srate[31:24] <= sd_sec_read_data;
                        32'd34: bits[7:0]    <= sd_sec_read_data;
                        32'd35: bits[15:8]   <= sd_sec_read_data;
                        default: ;
                    endcase
                    file_ofs <= file_ofs + 32'd1;
                    bcnt     <= bcnt + 10'd1;
                end

                if (sd_sec_read_end) begin
                    sd_sec_read <= 1'b0;
                    if (data_seen) begin
                        dbg_chn   <= chn;
                        dbg_srate <= srate;
                        if (fmt_ok) begin

                            data_sec    <= scan_sec;
                            file_ofs    <= 32'd0;
                            data_remain <= dsize;
                            asm_ph      <= 1'b0;
                            ch_tog      <= 1'b0;
                            end_sec_r   <= scan_sec + ((data_start + dsize) >> 9);

                            trk_req      <= 1'b1;
                            trk_req_sec  <= scan_sec;

                            if (disc_pend && (trk_cnt != TRK_N)) begin

                                wav_found <= 1'b0;
                                scan_win  <= 1'b1;
                                scan_cnt  <= 12'd0;
                                scan_sec  <= scan_sec + ((data_start + dsize) >> 9);
                                scan_stride <= 4'd1;
                                state     <= S_SCAN_REQ;
                            end else if (disc_pend) begin

                                disc_pend <= 1'b0;
                                cnt_disc  <= 1'b0;
                                cnt_done  <= 1'b1;
                                trk_go1   <= 1'b1;
                                scan_win  <= 1'b0;
                                goto_pend <= 1'b0;
                                state     <= S_IDLE;
                            end else if (trk_cnt == 3'd0) begin

                                resume_valid <= 1'b1;
                                ri_sec_r     <= scan_sec;
                                dstart_r     <= data_start;
                                dsize_r      <= dsize;
                                chn_r        <= chn;
                                wav_found    <= 1'b0;
                                disc_pend    <= 1'b1;
                                scan_win     <= 1'b1;
                                scan_cnt     <= 12'd0;
                                scan_sec     <= scan_sec + ((data_start + dsize) >> 9);
                                scan_stride  <= 4'd1;
                                scan_pass2   <= 1'b0;
                                state        <= S_SCAN_REQ;
                            end else begin

                                wav_found    <= 1'b1;

                                resume_valid <= 1'b1;
                                ri_sec_r     <= scan_sec;
                                dstart_r     <= data_start;
                                dsize_r      <= dsize;
                                chn_r        <= chn;
                                state        <= S_STREAM_REQ;
                            end
                        end else begin

                            wav_found <= 1'b0;
                            scan_sec  <= scan_sec + scan_stride;
                            state     <= S_SCAN_REQ;
                        end
                    end else if (hdr_sec >= (HDR_MAX_BYTES >> 9)) begin
                        scan_sec <= scan_sec + scan_stride;
                        state    <= S_SCAN_REQ;
                    end else begin
                        hdr_sec <= hdr_sec + 32'd1;
                        state   <= S_HDR_REQ;
                    end
                end
            end

            S_WAIT_LO: begin
                if (used <= FIFO_HI - 9'd256)
                    state <= S_STREAM_REQ;
            end

            S_STREAM_REQ: begin
                sd_sec_read_addr <= data_sec;
                bcnt             <= 10'd0;
                sd_sec_read      <= 1'b1;
                state            <= S_STREAM;
            end

            S_STREAM: begin
                if (sd_sec_read_data_valid) begin
                    if (file_ofs >= data_start) begin
                        if (!asm_ph) begin
                            asm_lo <= sd_sec_read_data;
                            asm_ph <= 1'b1;
                        end else begin
                            asm_ph <= 1'b0;

                            if ((chn == 16'd1) || !ch_tog) begin
                                wr_sample <= {sd_sec_read_data, asm_lo};
                                wr_en     <= 1'b1;
                            end
                            ch_tog <= ~ch_tog;
                        end
                    end
                    file_ofs <= file_ofs + 32'd1;
                    bcnt     <= bcnt + 10'd1;
                end

                if (sd_sec_read_end) begin
                    sd_sec_read <= 1'b0;
                    if (data_remain <= 32'd512) begin
                        state <= S_LOOP;
                    end else begin
                        data_sec    <= data_sec + 32'd1;
                        data_remain <= data_remain - 32'd512;
                        state       <= S_WAIT_LO;
                    end
                end
            end

            S_LOOP: begin
                sd_sec_read  <= 1'b0;
                wav_found    <= 1'b0;
                resume_valid <= 1'b0;
                goto_pend    <= 1'b1;

                goto_sec     <= trk_ok2 ? trk_sec[trk_nxt_i] : end_sec_r;
                scan_win     <= ~trk_ok2;
                scan_cnt     <= 12'd0;
                seek_prev    <= 1'b0;
                prev_hit     <= 1'b0;
                disc_pend    <= 1'b0;

                scan_stride  <= 4'd1;
                scan_pass2   <= 1'b0;
                state        <= S_IDLE;
            end

            default: state <= S_IDLE;
            endcase
        end
    end

endmodule
