/*
图片缓存池与加载调度。维护 N_POOL 个 SDRAM 帧槽，扫描到新图就往后排，
显示指针与写入指针分离，命中缓存即一帧内换图，不用等卡。

扫描与加载共用 bmp_read，靠 scan_pause / scan_resume 在扇区边界让路：
停下时锁存 scan_sector，加载结束后从该处续扫。

状态心跳看门狗：状态超过 50ms 不变即判卡死，中止整帧并自动重发。
*/

module sd_card_bmp #(
    parameter integer CLK_FREQ_HZ       = 100_000_000,
    parameter [31:0]  SCAN_START_SECTOR = 32'd0,
    parameter [31:0]  SCAN_MAX_SECTOR   = 32'd524287,
    parameter [5:0]   SCAN_TARGET_COUNT = 6'd12

)(
    input                       clk,
    input                       rst,
    input                       key_next,
    input                       key_auto,
    output [4:0]                state_code,
    input  [15:0]               bmp_width,
    input  [15:0]               bmp_height,
    output reg                  display_valid,

    input                       write_finish_toggle,
    output reg [2:0]            write_buf_idx,
    output reg [2:0]            disp_buf_idx,

    output                      write_req,
    input                       write_req_ack,
    output                      write_en,
    output [31:0]               write_data,

    input                       write_full,

    input                       sd_init_done,
    output wire                 sd_sec_read,
    output wire [31:0]          sd_sec_read_addr,
    input  wire [7:0]           sd_sec_read_data,
    input  wire                 sd_sec_read_data_valid,
    input  wire                 sd_sec_read_end,

    input  [1:0]                play_mode,
    input  [1:0]                auto_period,
    input                       next_tgl,
    input                       prev_tgl,
    input                       beat_evt,

    input                       disp_adopted,

    output [55:0]               dbg_bus,

    output [31:0]               dbg_geom,

    output [1:0]                o_img_lvl,

    output [15:0]               o_osd_src_w,
    output [15:0]               o_osd_src_h,
    output [10:0]               o_osd_disp_w,
    output [10:0]               o_osd_disp_h
);

wire key_next_press;
wire key_prev_press;
wire key_auto_press;
wire key_next_lvl;
wire key_auto_lvl;
wire key_next_sticky;
wire key_auto_sticky;

reg [2:0] nt_sync, pt_sync, at_sync;
always @(posedge clk or posedge rst) begin
    if (rst) begin
        nt_sync <= 3'b000;
        pt_sync <= 3'b000;
        at_sync <= 3'b000;
    end else begin
        nt_sync <= {nt_sync[1:0], next_tgl};
        pt_sync <= {pt_sync[1:0], prev_tgl};
        at_sync <= {at_sync[1:0], 1'b0};
    end
end
assign key_next_press = nt_sync[2] ^ nt_sync[1];
assign key_prev_press = pt_sync[2] ^ pt_sync[1];
assign key_auto_press = at_sync[2] ^ at_sync[1];
assign key_next_lvl   = 1'b1;
assign key_auto_lvl   = 1'b1;
assign key_next_sticky = 1'b0;
assign key_auto_sticky = 1'b0;

reg [1:0] disp_adopted_sync;
always @(posedge clk or posedge rst) begin
    if (rst) disp_adopted_sync <= 2'b00;
    else     disp_adopted_sync <= {disp_adopted_sync[0], disp_adopted};
end
wire disp_adopted_s = disp_adopted_sync[1];

wire             bmp_data_wr_en;
wire [23:0]      bmp_data;

reg  [9:0]       lum_div;
reg  [16:0]      lum_sum;
reg  [9:0]       lum_cnt;
reg  [1:0]       img_lvl [0:11];
wire [9:0]       px_y3   = {2'b00, bmp_data[23:16]} +
                           {2'b00, bmp_data[15:8]} +
                           {2'b00, bmp_data[7:0]};
wire [7:0]       px_y8   = px_y3[9:2];
wire             lum_smp = bmp_data_wr_en && (lum_div == 10'd0);

wire             bmp_row_timeout;
wire             bmp_load_fail;
wire             bmp_ready;
wire             scan_done;
wire             scan_found_valid;
wire [31:0]      scan_found_sector;
wire [5:0]       scan_found_total;

reg              scan_start_pulse;
reg              load_start_pulse;
reg [31:0]       load_sector;
reg              scan_kicked;
reg              first_image_committed;
reg              auto_play_en;
reg [31:0]       auto_cnt;
reg [5:0]        img_found_count;
reg [5:0]        img_idx;
reg [5:0]        load_idx;
reg [2:0]        pending_buf_idx;
reg [18:0]       img_sector [0:11];

    reg [2:0]        img_buf [0:11];
    reg [11:0]       img_v;
    integer          kb;
    integer          kc;
    reg [5:0]        buf_used;

    always @* begin
        buf_used = 6'd0;
        for (kb = 0; kb < 12; kb = kb + 1)
            if (img_v[kb]) buf_used[img_buf[kb]] = 1'b1;
    end
integer          i_sect;

reg [5:0]        target_idx;

reg              pf_valid;
reg [5:0]        pf_idx;
reg [2:0]        pf_buf;
reg              pf_go;
reg              load_manual;

reg              scan_pause_req;

reg [7:0]        key_cnt;
reg              load_busy;

reg              retry_pend;
reg [1:0]        retry_cnt;

// 看门狗：状态与行号都 50ms 不变即判卡死，拉 stall_abort 中止整帧。
localparam [22:0] STALL_LIMIT = 23'd4_999_999;
reg  [22:0]      stall_cnt;
reg  [4:0]       stall_state_d;
reg  [9:0]       stall_row_d;
reg              stall_abort;
reg              stuck_flag;
reg  [4:0]       stuck_state_r;
reg  [9:0]       stuck_row_r;

wire [4:0]       cur_stall_state = state_code;
wire [9:0]       cur_stall_row   = dbg_geom[22:13];

wire             stall_exp       = (stall_cnt >= STALL_LIMIT);

always @(posedge clk or posedge rst) begin
    if (rst) begin
        stall_cnt         <= 23'd0;
        stall_state_d     <= 5'd0;
        stall_row_d       <= 10'd0;
        stall_abort       <= 1'b0;
        stuck_flag        <= 1'b0;
    end else begin
        stall_state_d <= cur_stall_state;
        stall_row_d   <= cur_stall_row;
        stall_abort   <= 1'b0;
        if (write_finish_pulse)
            stuck_flag <= 1'b0;
        if (!load_busy) begin
            stall_cnt <= 23'd0;
        end else if ((cur_stall_state != stall_state_d) || (cur_stall_row != stall_row_d)) begin
            stall_cnt <= 23'd0;
        end else if (!stall_exp) begin
            stall_cnt <= stall_cnt + 23'd1;
        end else begin
            stall_abort       <= 1'b1;
            stuck_flag        <= 1'b1;
            stuck_state_r     <= cur_stall_state;
            stuck_row_r       <= cur_stall_row;
            stall_cnt         <= 23'd0;
        end
    end
end

reg              scan_resume_r;
reg              scan_resume_pulse;

reg              auto_pend;

wire             list_ready = scan_done || (img_found_count != 3'd0);

wire             auto_tick_use;

wire             beat_switch_now;
wire             auto_go        = auto_play_en && (auto_tick_use || auto_pend);
wire             want_load_now  = (!display_valid && (img_found_count != 3'd0)) ||
                                  (display_valid && (img_found_count != 3'd0) && (target_idx != img_idx)) ||
                                  (display_valid && auto_go && (img_found_count > 3'd1));

wire             want_scan_resume = !scan_done && !scan_pause_req && !load_busy &&
                                    bmp_ready && !want_load_now;

reg              source_done_seen;

reg [2:0]        disp_buf_idx_d;
reg [25:0]       buf_settle;
wire             buf_switch = (disp_buf_idx != disp_buf_idx_d);

always @(posedge clk or posedge rst) begin
    if (rst) begin
        disp_buf_idx_d <= 3'd0;
        buf_settle     <= 26'd0;
    end else begin
        disp_buf_idx_d <= disp_buf_idx;
        if (buf_switch)
            buf_settle <= 26'd0;
        else if (!buf_settle[25])
            buf_settle <= buf_settle + 26'd1;
    end
end

wire buf_ready = buf_settle[25];

reg [31:0]       load_wdog;
wire             load_wdog_exp = (load_wdog == 32'd2_999_999_999);

reg [2:0]        wrfin_tgl_sync;
wire             write_finish_pulse;

wire auto_tick;
wire [5:0] next_from_current;
wire [5:0] next_target_idx;
wire [5:0] prev_target_idx;

reg  [1:0] auto_period_r;
reg  [1:0] auto_period_d;
reg  [3:0] auto_sec;
always @(posedge clk or posedge rst) begin
    if (rst) begin
        auto_period_r <= 2'd0;
        auto_period_d <= 2'd0;
    end else begin
        auto_period_r <= auto_period;
        auto_period_d <= auto_period_r;
    end
end

wire        auto_period_chg = (auto_period_r != auto_period_d);
wire        auto_tick_1s = (auto_cnt == (CLK_FREQ_HZ - 1));
wire [3:0]  auto_lim     = (auto_period_r == 2'd0) ? 4'd0 :
                           (auto_period_r == 2'd1) ? 4'd1 :
                           (auto_period_r == 2'd2) ? 4'd4 : 4'd9;

reg [1:0] play_mode_r;
reg [1:0] play_mode_s;
always @(posedge clk or posedge rst) begin
    if (rst) begin
        play_mode_r <= 2'd0;
        play_mode_s <= 2'd0;
    end else begin
        play_mode_r <= play_mode;
        play_mode_s <= play_mode_r;
    end
end

reg [2:0] beat_sync;
wire      beat_pulse = beat_sync[2] & ~beat_sync[1];
always @(posedge clk or posedge rst) begin
    if (rst) beat_sync <= 3'b000;
    else     beat_sync <= {beat_sync[1:0], beat_evt};
end

localparam [1:0] BEATS_PER_SHOT = 2'd2;

reg  [1:0] beat_div;
always @(posedge clk or posedge rst) begin
    if (rst) beat_div <= 2'd0;
    else if (beat_pulse) begin
        if ((beat_div + 2'd1) >= BEATS_PER_SHOT) beat_div <= 2'd0;
        else                                     beat_div <= beat_div + 2'd1;
    end
end

wire      beat_use = beat_pulse && ((beat_div + 2'd1) >= BEATS_PER_SHOT);

assign write_en   = bmp_data_wr_en;
assign write_data = {bmp_data[23:16], bmp_data[15:8], bmp_data[7:0], 8'b0};
assign auto_tick  = auto_tick_1s && (auto_sec == auto_lim);

reg  pf_ready_r;

reg  auto_trig_r;
always @(posedge clk or posedge rst) begin
    if (rst) auto_trig_r <= 1'b0;
    else     auto_trig_r <= (play_mode_s == 2'd2) ? (beat_use && !pf_ready_r) : auto_tick;
end
assign auto_tick_use = auto_trig_r;

assign beat_switch_now = (play_mode_s == 2'd2) && beat_use && pf_ready_r;

wire   prefetch_ok = (play_mode_s != 2'd1);

function [5:0] next_pool_idx;
    input [5:0] cur;
    input [11:0] vmask;
    reg [3:0] k;
    reg [4:0] idx5;
    reg       found;
    begin
        next_pool_idx = cur;
        found         = 1'b0;
        for (k = 4'd1; k <= 4'd11; k = k + 4'd1) begin
            idx5 = {1'b0, cur[3:0]} + {1'b0, k};
            if (idx5 >= 5'd12) idx5 = idx5 - 5'd12;
            if (!found && vmask[idx5[3:0]]) begin
                next_pool_idx = {2'b00, idx5[3:0]};
                found         = 1'b1;
            end
        end
    end
endfunction

reg [5:0] next_pool_r;
always @(posedge clk or posedge rst) begin
    if (rst) next_pool_r <= 6'd0;
    else     next_pool_r <= next_pool_idx(img_idx, img_v);
end

wire [5:0] next_all_c = next_index_limited(img_idx, img_found_count);
assign next_from_current = (play_mode_s == 2'd2) ? next_pool_r
                                                 : next_index_limited(img_idx, img_found_count);
assign next_target_idx   = next_index_limited(target_idx, img_found_count);

assign prev_target_idx   = prev_index_limited(target_idx, img_found_count);

reg [5:0] next_fc_r;
always @(posedge clk or posedge rst) begin
    if (rst) next_fc_r <= 6'd0;
    else     next_fc_r <= next_from_current;
end

reg        hit_tgt,  hit_prev,  hit_fc,  abs_all;
reg [2:0]  buf_tgt,  buf_prev,  buf_fc;
reg [5:0]  tgt_r,    prev_r,    nfc_r,   nall_r;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        hit_tgt  <= 1'b0;
        buf_tgt  <= 3'd0;
        tgt_r    <= 6'd0;
        hit_prev <= 1'b0;
        buf_prev <= 3'd0;
        prev_r   <= 6'd0;
        hit_fc   <= 1'b0;
        buf_fc   <= 3'd0;
        nfc_r    <= 6'd0;
        abs_all  <= 1'b0;
        nall_r   <= 6'd0;
    end else begin

        tgt_r    <= target_idx;
        hit_tgt  <= img_v[target_idx[3:0]]      && (img_buf[target_idx[3:0]]      != disp_buf_idx);
        buf_tgt  <= img_buf[target_idx[3:0]];

        prev_r   <= prev_target_idx;
        hit_prev <= img_v[prev_target_idx[3:0]] && (img_buf[prev_target_idx[3:0]] != disp_buf_idx);
        buf_prev <= img_buf[prev_target_idx[3:0]];

        nfc_r    <= next_fc_r;
        hit_fc   <= img_v[next_fc_r[3:0]]       && (img_buf[next_fc_r[3:0]]       != disp_buf_idx);
        buf_fc   <= img_buf[next_fc_r[3:0]];

        nall_r   <= next_all_c;
        abs_all  <= !img_v[next_all_c[3:0]];
    end
end

wire        cache_hit_next = hit_fc && (nfc_r != img_idx);
wire        pf_hit_next    = pf_valid && (pf_idx != img_idx) && (img_found_count > 3'd1);
wire        next_ready     = pf_hit_next || cache_hit_next;
wire [2:0]  next_buf_sel   = pf_hit_next ? pf_buf : buf_fc;
wire [5:0]  next_idx_sel   = pf_hit_next ? pf_idx : nfc_r;

always @(posedge clk or posedge rst) begin
    if (rst) pf_ready_r <= 1'b0;
    else     pf_ready_r <= (play_mode_s == 2'd2) && !load_busy &&
                           (img_found_count > 3'd1) && next_ready;
end

assign write_finish_pulse = wrfin_tgl_sync[2] ^ wrfin_tgl_sync[1];
assign o_img_lvl = img_lvl[img_idx[3:0]];
assign dbg_bus = {{stuck_flag, stuck_state_r, stuck_row_r},
                  key_cnt[7:0], bmp_row_timeout, img_idx[2],
                  img_found_count[5:0], target_idx[4:0], {2'b00, pf_go},
                  img_idx[1:0], key_auto_sticky, key_next_sticky, key_auto_lvl, key_next_lvl,
                  first_image_committed, bmp_ready, scan_done,
                  img_found_count[2:0], auto_play_en, load_busy, display_valid, sd_init_done};

key_press_debounce #(
    .CLK_FREQ_HZ (CLK_FREQ_HZ),
    .DEBOUNCE_MS (20)
) u_key_next (
    .clk        (clk),
    .rst        (rst),
    .button_in  (1'b1),
    .press_pulse(),
    .debounced  (),
    .sticky_press()
);

key_press_debounce #(
    .CLK_FREQ_HZ (CLK_FREQ_HZ),
    .DEBOUNCE_MS (20)
) u_key_auto (
    .clk        (clk),
    .rst        (rst),
    .button_in  (1'b1),
    .press_pulse(),
    .debounced  (),
    .sticky_press()
);

function [5:0] next_index_limited;
    input [5:0] cur;
    input [5:0] count;
    begin
        if (count == 3'd0)
            next_index_limited = 6'd0;
        else if ((cur + 6'd1) >= count)
            next_index_limited = 6'd0;
        else
            next_index_limited = cur + 6'd1;
    end
endfunction

function [5:0] prev_index_limited;
    input [5:0] cur;
    input [5:0] count;
    begin
        if (count == 3'd0)
            prev_index_limited = 6'd0;
        else if (cur == 6'd0)
            prev_index_limited = count - 6'd1;
        else if (cur >= count)
            prev_index_limited = count - 6'd1;
        else
            prev_index_limited = cur - 6'd1;
    end
endfunction

function [31:0] sector_lut;
    input [5:0] idx;
    begin
        sector_lut = {13'd0, img_sector[idx[3:0]]};
    end
endfunction

localparam [2:0] N_POOL = 3'd6;

function [2:0] next_buf_lut;
    input [2:0] cur_disp_buf;
    input       valid_now;
    begin

        if (!valid_now)
            next_buf_lut = 3'd0;

        else if (!buf_used[0] && (cur_disp_buf != 3'd0)) next_buf_lut = 3'd0;
        else if (!buf_used[1] && (cur_disp_buf != 3'd1)) next_buf_lut = 3'd1;
        else if (!buf_used[2] && (cur_disp_buf != 3'd2)) next_buf_lut = 3'd2;
        else if (!buf_used[3] && (cur_disp_buf != 3'd3)) next_buf_lut = 3'd3;
        else if ((N_POOL >= 3'd5) && !buf_used[4] && (cur_disp_buf != 3'd4)) next_buf_lut = 3'd4;
        else if ((N_POOL >= 3'd6) && !buf_used[5] && (cur_disp_buf != 3'd5)) next_buf_lut = 3'd5;
        else if (cur_disp_buf != 3'd0) next_buf_lut = 3'd0;
        else if (cur_disp_buf != 3'd1) next_buf_lut = 3'd1;
        else if (cur_disp_buf != 3'd2) next_buf_lut = 3'd2;
        else if (cur_disp_buf != 3'd3) next_buf_lut = 3'd3;
        else if (cur_disp_buf != 3'd4) next_buf_lut = 3'd4;
        else                           next_buf_lut = 3'd5;
    end
endfunction

always @(posedge clk or posedge rst) begin
    if (rst)
        load_wdog <= 32'd0;
    else if (!load_busy)
        load_wdog <= 32'd0;
    else if (!load_wdog_exp)
        load_wdog <= load_wdog + 32'd1;
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        wrfin_tgl_sync        <= 3'b000;
        scan_start_pulse      <= 1'b0;
        load_start_pulse      <= 1'b0;
        load_sector           <= 32'd0;
        scan_kicked           <= 1'b0;
        first_image_committed <= 1'b0;
        auto_play_en          <= 1'b0;
        auto_cnt              <= 32'd0;
        img_found_count       <= 3'd0;
        img_idx               <= 3'd0;
        load_idx              <= 3'd0;
        pending_buf_idx       <= 3'd0;
        write_buf_idx         <= 3'd0;
        disp_buf_idx          <= 3'd0;
        for (i_sect = 0; i_sect < 12; i_sect = i_sect + 1) img_sector[i_sect] <= 19'd0;
        target_idx            <= 3'd0;
        pf_valid              <= 1'b0;
        pf_idx                <= 3'd0;
        pf_buf                <= 3'd0;
        pf_go                 <= 1'b0;
        load_manual           <= 1'b0;
        scan_pause_req        <= 1'b0;
        key_cnt               <= 8'd0;
        scan_resume_r         <= 1'b0;
        scan_resume_pulse     <= 1'b0;
        auto_pend             <= 1'b0;
        load_busy             <= 1'b0;        source_done_seen      <= 1'b0;
        retry_pend            <= 1'b0;
        retry_cnt             <= 2'd0;
        display_valid         <= 1'b0;
        lum_div               <= 10'd0;
        lum_sum               <= 17'd0;
        lum_cnt               <= 10'd0;
        for (i_sect = 0; i_sect < 12; i_sect = i_sect + 1) img_lvl[i_sect] <= 2'd1;
    end else begin
        wrfin_tgl_sync   <= {wrfin_tgl_sync[1:0], write_finish_toggle};
        scan_start_pulse <= 1'b0;
        load_start_pulse <= 1'b0;

        if (load_start_pulse) begin
            lum_div <= 10'd0;
            lum_sum <= 17'd0;
            lum_cnt <= 10'd0;
        end else if (bmp_data_wr_en) begin
            lum_div <= lum_div + 10'd1;
            if (lum_smp) begin
                lum_sum <= lum_sum + {10'd0, px_y8};
                if (lum_cnt != 10'd1023) lum_cnt <= lum_cnt + 10'd1;
            end
        end

        if (!sd_init_done) begin
            scan_kicked           <= 1'b0;
            first_image_committed <= 1'b0;
            auto_play_en          <= 1'b0;
            auto_cnt              <= 32'd0;
            img_found_count       <= 3'd0;
            img_idx               <= 3'd0;
            load_idx              <= 3'd0;
            pending_buf_idx       <= 3'd0;
            write_buf_idx         <= 3'd0;
            disp_buf_idx          <= 3'd0;
            for (i_sect = 0; i_sect < 12; i_sect = i_sect + 1) img_sector[i_sect] <= 19'd0;
            target_idx            <= 3'd0;
            pf_valid              <= 1'b0;
            pf_go                 <= 1'b0;
            scan_pause_req        <= 1'b0;
            load_busy             <= 1'b0;
            source_done_seen      <= 1'b0;
            retry_pend            <= 1'b0;
            retry_cnt             <= 2'd0;
            display_valid         <= 1'b0;
        end else begin

            if (scan_found_valid) begin
                img_sector[img_found_count[3:0]] <= scan_found_sector[18:0];

                if (img_found_count < SCAN_TARGET_COUNT)
                    img_found_count <= img_found_count + 6'd1;
            end

            if (load_busy && bmp_ready)
                source_done_seen <= 1'b1;

            if (load_busy && source_done_seen && write_finish_pulse) begin
                load_busy        <= 1'b0;
                source_done_seen <= 1'b0;

                img_lvl[load_idx[3:0]] <=
                    (lum_sum >  {7'd0, lum_cnt, 7'd0})              ? 2'd2 :
                    (lum_sum <  ({7'd0, lum_cnt} * 17'd48))         ? 2'd0 :
                                                                      2'd1;

                retry_cnt        <= 2'd0;

                if (pf_go) begin

                    pf_go    <= 1'b0;
                    pf_valid <= 1'b1;
                    pf_idx   <= load_idx;
                end else begin
                    disp_buf_idx          <= pending_buf_idx;
                    img_idx               <= load_idx;
                    display_valid         <= 1'b1;
                    first_image_committed <= 1'b1;
                    pf_valid              <= 1'b0;

                    if (!load_manual)
                        target_idx <= load_idx;
                end
            end

            if (!scan_kicked && bmp_ready) begin
                scan_start_pulse      <= 1'b1;
                scan_kicked           <= 1'b1;
                first_image_committed <= 1'b0;
                auto_play_en          <= 1'b0;
                auto_cnt              <= 32'd0;
                img_found_count       <= 3'd0;
                img_idx               <= 3'd0;
                load_idx              <= 3'd0;
                pending_buf_idx       <= 3'd0;
                write_buf_idx         <= 3'd0;
                disp_buf_idx          <= 3'd0;
                target_idx            <= 3'd0;
                pf_valid              <= 1'b0;
                pf_go                 <= 1'b0;
                scan_pause_req        <= 1'b0;
                display_valid         <= 1'b0;
                load_busy             <= 1'b0;
                source_done_seen      <= 1'b0;
            end else begin

                if (load_start_pulse)
                    scan_pause_req <= 1'b0;
                else if (want_load_now)
                    scan_pause_req <= 1'b1;
                else
                    scan_pause_req <= 1'b0;

                if (auto_tick_use && auto_play_en && display_valid && !load_busy &&
                    first_image_committed && (img_found_count > 3'd1))
                    auto_pend <= 1'b1;
                if (load_start_pulse)
                    auto_pend <= 1'b0;

                scan_resume_r <= want_scan_resume;
                if (want_scan_resume && !scan_resume_r)
                    scan_resume_pulse <= 1'b1;
                else
                    scan_resume_pulse <= 1'b0;

                if ((key_next_press || key_prev_press) && (key_cnt < 8'd99))
                    key_cnt <= key_cnt + 8'd1;

                if (key_next_press && list_ready && (img_found_count > 3'd0)) begin
                    retry_pend <= 1'b0;
                    retry_cnt  <= 2'd0;
                    if (pf_valid && !load_busy && (pf_idx == next_target_idx) &&
                        (next_target_idx != img_idx)) begin
                        disp_buf_idx <= pf_buf;
                        img_idx      <= pf_idx;
                        target_idx   <= pf_idx;
                        pf_valid     <= 1'b0;
                    end else begin
                        target_idx <= next_target_idx;
                    end
                end

                if (key_prev_press && list_ready && (img_found_count > 3'd0)) begin
                    retry_pend <= 1'b0;
                    retry_cnt  <= 2'd0;
                    if (hit_prev && (prev_r == prev_target_idx)) begin
                        disp_buf_idx <= buf_prev;
                        img_idx      <= prev_r;
                        target_idx   <= prev_r;
                    end else begin
                        target_idx <= prev_r;
                    end
                end

                if (auto_play_en != (play_mode_s != 2'd0)) begin
                    auto_play_en <= (play_mode_s != 2'd0);
                    auto_cnt     <= 32'd0;
                    auto_sec     <= 4'd0;
                end

                if (list_ready && auto_play_en && display_valid && !load_busy && first_image_committed && (img_found_count > 3'd1)) begin
                    if (auto_tick_1s) begin
                        auto_cnt <= 32'd0;
                        auto_sec <= (auto_sec >= auto_lim) ? 4'd0 : (auto_sec + 4'd1);
                    end else begin
                        auto_cnt <= auto_cnt + 32'd1;
                    end
                end else begin
                    auto_cnt <= 32'd0;
                    auto_sec <= 4'd0;
                end

                if (auto_period_chg) begin
                    auto_cnt <= 32'd0;
                    auto_sec <= 4'd0;
                end

                if ((stall_abort || bmp_load_fail) && load_busy) begin
                    load_busy        <= 1'b0;
                    source_done_seen <= 1'b0;
                    if (retry_cnt != 2'd2) begin
                        retry_cnt  <= retry_cnt + 2'd1;
                        retry_pend <= 1'b1;
                    end else begin
                        retry_pend <= 1'b0;
                        target_idx <= img_idx;
                    end
                end

                if (retry_pend && bmp_ready && !load_busy) begin
                    retry_pend       <= 1'b0;
                    load_start_pulse <= 1'b1;
                    load_busy        <= 1'b1;
                    source_done_seen <= 1'b0;
                end

                else if (list_ready && !display_valid && bmp_ready && !load_busy && (img_found_count != 3'd0)) begin
                    load_idx         <= 3'd0;
                    load_sector      <= img_sector[0];
                    pending_buf_idx  <= 3'd0;
                    write_buf_idx    <= 3'd0;
                    load_start_pulse <= 1'b1;
                    load_busy        <= 1'b1;
                    load_manual      <= 1'b0;
                    pf_valid         <= 1'b0;
                    pf_go            <= 1'b0;
                    source_done_seen <= 1'b0;
                    target_idx       <= 3'd0;
                    auto_cnt         <= 32'd0;
                end

                if (list_ready && bmp_ready && display_valid && !load_busy && buf_ready &&
                    disp_adopted_s && (play_mode_s == 2'd2) && beat_switch_now &&
                    (nfc_r == next_fc_r) && (next_idx_sel != img_idx)) begin
                    disp_buf_idx <= next_buf_sel;
                    img_idx      <= next_idx_sel;
                    target_idx   <= next_idx_sel;

                    pf_valid     <= pf_hit_next ? 1'b0 : pf_valid;
                    auto_pend    <= 1'b0;
                    auto_cnt     <= 32'd0;
                end
                else if (list_ready && bmp_ready && display_valid && !load_busy && buf_ready &&
                         disp_adopted_s && pf_valid && (pf_idx == target_idx) &&
                         (target_idx != img_idx)) begin
                    disp_buf_idx     <= pf_buf;
                    img_idx          <= pf_idx;
                    target_idx       <= pf_idx;
                    pf_valid         <= 1'b0;
                    auto_cnt         <= 32'd0;
                end

                else if (list_ready && bmp_ready && display_valid && !load_busy && buf_ready &&
                         disp_adopted_s && (target_idx != img_idx) && (tgt_r == target_idx) && hit_tgt) begin
                    disp_buf_idx <= buf_tgt;
                    img_idx      <= tgt_r;
                    auto_cnt     <= 32'd0;
                end

                else if (list_ready && bmp_ready && display_valid && !load_busy && buf_ready && disp_adopted_s && (target_idx != img_idx) && (tgt_r == target_idx) && (img_found_count != 3'd0)) begin
                    load_idx         <= tgt_r;
                    load_sector      <= sector_lut(tgt_r);
                    pending_buf_idx  <= next_buf_lut(disp_buf_idx, display_valid);
                    write_buf_idx    <= next_buf_lut(disp_buf_idx, display_valid);
                    load_start_pulse <= 1'b1;
                    load_busy        <= 1'b1;
                    load_manual      <= 1'b1;
                    pf_valid         <= 1'b0;
                    source_done_seen <= 1'b0;
                    auto_cnt         <= 32'd0;
                end

                else if (list_ready && bmp_ready && display_valid && !load_busy && buf_ready && disp_adopted_s &&
                         auto_go && (img_found_count > 3'd1) && (nfc_r == next_fc_r) && (nfc_r != img_idx) && hit_fc) begin
                    disp_buf_idx <= buf_fc;
                    img_idx      <= nfc_r;
                    target_idx   <= nfc_r;
                    auto_cnt     <= 32'd0;

                    auto_pend    <= 1'b0;
                end

                else if (list_ready && bmp_ready && display_valid && !load_busy && buf_ready && disp_adopted_s && auto_go && (img_found_count > 3'd1) && (nfc_r == next_fc_r) && (nfc_r != img_idx)) begin
                    load_idx         <= nfc_r;
                    load_sector      <= sector_lut(nfc_r);
                    pending_buf_idx  <= next_buf_lut(disp_buf_idx, display_valid);
                    write_buf_idx    <= next_buf_lut(disp_buf_idx, display_valid);
                    load_start_pulse <= 1'b1;
                    load_busy        <= 1'b1;
                    load_manual      <= 1'b0;
                    pf_valid         <= 1'b0;
                    source_done_seen <= 1'b0;
                    auto_cnt         <= 32'd0;
                end

                else if (scan_done && bmp_ready && display_valid && first_image_committed &&
                         !key_next_press && !load_busy && buf_ready && !pf_valid && prefetch_ok &&
                         (nall_r != img_idx) &&
                         abs_all &&
                         disp_adopted_s && (img_found_count > 3'd1) && (target_idx == img_idx)) begin
                    load_idx         <= nall_r;
                    load_sector      <= sector_lut(nall_r);
                    pending_buf_idx  <= next_buf_lut(disp_buf_idx, 1'b1);
                    write_buf_idx    <= next_buf_lut(disp_buf_idx, 1'b1);
                    pf_buf           <= next_buf_lut(disp_buf_idx, 1'b1);
                    load_start_pulse <= 1'b1;
                    load_busy        <= 1'b1;
                    pf_go            <= 1'b1;
                    load_manual      <= 1'b0;
                    source_done_seen <= 1'b0;
                end

                if (load_wdog_exp) begin
                    load_busy        <= 1'b0;
                    source_done_seen <= 1'b0;
                    pf_go            <= 1'b0;
                    pf_valid         <= 1'b0;
                    target_idx       <= img_idx;
                end
            end
        end
    end
end

wire commit_now = load_busy && source_done_seen && write_finish_pulse;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        for (kc = 0; kc < 12; kc = kc + 1)
            img_v[kc] <= 1'b0;
    end else begin
        if (load_start_pulse) begin
            for (kc = 0; kc < 12; kc = kc + 1)
                if (img_buf[kc] == write_buf_idx) img_v[kc] <= 1'b0;
        end
        if (commit_now) begin
            img_buf[load_idx[3:0]] <= write_buf_idx;
            img_v[load_idx[3:0]]   <= 1'b1;
        end
    end
end

bmp_read bmp_read_m0(
    .clk                    (clk),
    .rst                    (rst),
    .ready                  (bmp_ready),

    .scan_start             (scan_start_pulse),
    .scan_start_sector      (SCAN_START_SECTOR),
    .scan_max_sector        (SCAN_MAX_SECTOR),
    .scan_target_count      (SCAN_TARGET_COUNT),
    .scan_pause             (scan_pause_req),
    .scan_resume            (scan_resume_pulse),
    .scan_done              (scan_done),
    .scan_found_valid       (scan_found_valid),
    .scan_found_sector      (scan_found_sector),
    .scan_found_total       (scan_found_total),

    .load_start             (load_start_pulse),
    .load_sector            (load_sector),

    .sd_init_done           (sd_init_done),
    .state_code             (state_code),
    .dbg_geom               (dbg_geom),

    .o_osd_src_w            (o_osd_src_w),
    .o_osd_src_h            (o_osd_src_h),
    .o_osd_disp_w           (o_osd_disp_w),
    .o_osd_disp_h           (o_osd_disp_h),
    .bmp_width              (bmp_width),
    .bmp_height             (bmp_height),
    .write_req              (write_req),
    .write_req_ack          (write_req_ack),
    .sd_sec_read            (sd_sec_read),
    .sd_sec_read_addr       (sd_sec_read_addr),
    .sd_sec_read_data       (sd_sec_read_data),
    .sd_sec_read_data_valid (sd_sec_read_data_valid),
    .sd_sec_read_end        (sd_sec_read_end),
    .bmp_data_wr_en         (bmp_data_wr_en),
    .bmp_data               (bmp_data),
    .write_full             (write_full),
    .stall_abort            (stall_abort),
    .row_timeout            (bmp_row_timeout),
    .load_fail              (bmp_load_fail)
);

endmodule

module key_press_debounce #(
    parameter integer CLK_FREQ_HZ = 100_000_000,
    parameter integer DEBOUNCE_MS = 20
)(
    input  wire clk,
    input  wire rst,
    input  wire button_in,
    output reg  press_pulse,
    output wire debounced,
    output reg  sticky_press
);

localparam integer DEBOUNCE_CYCLES = (CLK_FREQ_HZ / 1000) * DEBOUNCE_MS;

localparam integer PRESS_CYCLES = (CLK_FREQ_HZ / 1000) * 3;

reg button_sync0;
reg button_sync1;
reg button_stable;
reg [31:0] cnt;

assign debounced = button_stable;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        button_sync0  <= 1'b1;
        button_sync1  <= 1'b1;
        button_stable <= 1'b1;
        cnt           <= 32'd0;
        press_pulse   <= 1'b0;
        sticky_press  <= 1'b0;
    end else begin
        button_sync0 <= button_in;
        button_sync1 <= button_sync0;
        press_pulse  <= 1'b0;

        if (button_sync1 == button_stable) begin
            cnt <= 32'd0;
        end else if (cnt >= (((button_sync1 == 1'b0) ? PRESS_CYCLES
                                                   : DEBOUNCE_CYCLES) - 1)) begin
            if (button_stable && !button_sync1) begin
                press_pulse  <= 1'b1;
                sticky_press <= 1'b1;
            end
            button_stable <= button_sync1;
            cnt           <= 32'd0;
        end else begin
            cnt <= cnt + 32'd1;
        end
    end
end

endmodule
