/*
顶层。把图像、音频、OSD 三条通路接起来：TF 卡经 sd_card_bmp 取图写入 SDRAM
缓存池，读侧送出 640x480 RGB；音频通路由 wav_player 产生 I2S 帧；osd_layer
在像素流上叠加字幕、频谱与菜单，最后送 HDMI。

三个时钟域：video_clk 25MHz 像素，sd_card_clk 100MHz，audio_mclk 12.288MHz。
跨域信号先打两拍再用，多比特量走握手，不直接采样。

屏幕亮度 = 手动设定 + 音乐包络呼吸 + 环境光补偿，三项位宽不同，相加时每个
操作数都要 $signed()，拼接表达式默认无符号。
*/

module top(
    input                       clk,
    input                       rst_n,
    input                       key1,
    input                       key2,
    input                       key3,
    input                       sw1,
    input                       sw2,
    input                       sw3,
    input                       sw4,

    output [5:0]                seg_sel,
    output [7:0]                seg_data,

    output                      HDMI_CLK_P,
    output                      HDMI_D2_P,
    output                      HDMI_D1_P,
    output                      HDMI_D0_P,

    output                      HDMI_DDC_SCL,
    inout                       HDMI_DDC_SDA,

    output                      sd_ncs,
    output                      sd_dclk,
    output                      sd_mosi,
    input                       sd_miso,

    output [3:0]                led
);

parameter MEM_DATA_BITS = 32;
parameter ADDR_BITS     = 21;
parameter BUSRT_BITS    = 10;
parameter FRAME_PIXELS  = 24'd307200;
parameter BUF0_ADDR     = 24'd0;
parameter BUF1_ADDR     = FRAME_PIXELS;

parameter BUF2_ADDR     = FRAME_PIXELS * 2;
parameter BUF3_ADDR     = FRAME_PIXELS * 3;

parameter BUF4_ADDR     = FRAME_PIXELS * 4;
parameter BUF5_ADDR     = FRAME_PIXELS * 5;

wire Sdr_init_done;
wire Sdr_init_ref_vld;
wire Sdr_busy;

wire sd_card_clk;
wire ext_mem_clk;
wire ext_mem_clk_sft;
wire video_clk;
wire hdmi_5x_clk;

wire hs;
wire vs;
wire de;

wire [23:0] vout_data_raw;
wire [23:0] vout_data;
wire        display_valid;

wire [4:0]  state_code;

wire        video_read_req;
wire        video_read_req_ack;
wire        video_read_en;
wire [31:0] video_read_data;

wire        sd_card_write_en;
wire [31:0] sd_card_write_data;
wire sd_card_write_full;
wire        sd_card_write_req;
wire        sd_card_write_req_ack;
wire        frame_write_finish;
reg         frame_write_toggle_mem;

wire [2:0]  write_buf_idx;
wire [2:0]  disp_buf_idx;

wire        disp_adopted;

wire        music_on;

wire        menu_prev_tgl;
wire        menu_next_tgl;

wire [3:0]  menu_trk_now;
wire [3:0]  menu_trk_total;

wire [1:0]  menu_play_mode;
wire [1:0]  menu_period;
wire        beat_evt;

wire        menu_open;

wire        wav_count_busy;
wire [1:0]  img_lvl;
wire        wipe_mode;
wire [1:0]  trans_sel_eff;

wire        music_hardcut;
wire        music_driving;

wire        trans_busy;

wire        sd_init_done;
wire [7:0]  sd_sec_read_data;
wire        sd_sec_read_data_valid;
wire        sd_sec_read_end;

wire App_rd_en;
wire [ADDR_BITS-1:0] App_rd_addr;
wire Sdr_rd_en;
wire [MEM_DATA_BITS-1:0] Sdr_rd_dout;
wire App_wr_en;
wire [ADDR_BITS-1:0] App_wr_addr;
wire [MEM_DATA_BITS-1:0] App_wr_din;
wire [3:0] App_wr_dm;

wire hs_0;
wire vs_0;
wire de_0;

wire        audio_pll_lock;
wire        audio_mclk;
wire        audio_i2s_bclk;
wire        audio_i2s_lrck;
wire        audio_i2s_dout;
wire        audio_valid;
wire [23:0] audio_left_data;
wire [23:0] audio_right_data;
wire        acr_valid;
wire [19:0] acr_cts;
wire [19:0] acr_n;

wire        axis_s_user;
wire        axis_s_valid;
wire        axis_s_last;
wire [23:0] axis_s_data;
wire        axis_s_ready;

wire        edid_trig;
wire        edid_valid;
wire [7:0]  edid_data;

wire [9:0]  tmds_ch0_data;
wire [9:0]  tmds_ch1_data;
wire [9:0]  tmds_ch2_data;
wire [9:0]  tmds_clk_data;

wire rst_vid = ~rst_n;
wire rst_aud = ~rst_n | ~audio_pll_lock;
wire rst_all = rst_vid;

/*
环境光：光敏模块 AO 接 J1-37，对应 R16 / ADC 通道 7。模块越亮 AO 越低，
AMB_INVERT 取 1 时为暗处提亮。基准用片内 VREF，不能设成外部。
*/
localparam EN_AMBIENT  = 1'b1;
localparam AMB_INVERT  = 1'b1;

reg         vs_bd;
reg  [6:0]  osd_bright_use;

reg  [5:0] adc_div;
always @(posedge audio_mclk or posedge rst_aud) begin
    if (rst_aud) adc_div <= 6'd0;
    else         adc_div <= adc_div + 6'd1;
end
// 每 64 个 audio_mclk 起一次转换，间隔 5.2us，小于手册要求的 10us。
wire adc_soc = (adc_div == 6'd0);

wire        adc_eoc;
wire [11:0] adc_dout;

EG_PHY_ADC #(
    .CH0("DISABLE"), .CH1("DISABLE"), .CH2("DISABLE"), .CH3("DISABLE"),
    .CH4("DISABLE"), .CH5("DISABLE"), .CH6("DISABLE"), .CH7("ENABLE"),
    .VREF ("DISABLE")
) u_amb_adc (
    .clk  (audio_mclk),
    .pd   (1'b0),
    .s    (3'd7),
    .soc  (adc_soc),
    .eoc  (adc_eoc),
    .dout (adc_dout)
);

// 手册没规定 dout 在 eoc 的哪一拍建立，所以 eoc 当拍与后一拍各采一次取大。
reg        adc_eoc_d1, adc_eoc_d2, adc_eoc_d3;
reg [11:0] adc_s1, adc_s2;
always @(posedge audio_mclk or posedge rst_aud) begin
    if (rst_aud) begin
        adc_eoc_d1 <= 1'b0; adc_eoc_d2 <= 1'b0; adc_eoc_d3 <= 1'b0;
        adc_s1     <= 12'd0; adc_s2    <= 12'd0;
    end else begin
        adc_eoc_d1 <= adc_eoc;
        adc_eoc_d2 <= adc_eoc_d1;
        adc_eoc_d3 <= adc_eoc_d2;
        if (adc_eoc)    adc_s1 <= adc_dout;
        if (adc_eoc_d1) adc_s2 <= adc_dout;
    end
end
wire        adc_upd = adc_eoc_d3;
wire [11:0] adc_val = (adc_s1 > adc_s2) ? adc_s1 : adc_s2;

// 一阶 IIR，acc += x - (acc >> 9)，时间常数 512 个样本约 2.7ms。
reg  [21:0] amb_acc;
reg  [11:0] amb_avg;
always @(posedge audio_mclk or posedge rst_aud) begin
    if (rst_aud) begin
        amb_acc <= 22'd0;
        amb_avg <= 12'd2048;
    end else if (adc_upd) begin
        amb_acc <= amb_acc + {10'd0, adc_val} - {10'd0, amb_acc[20:9]};
        amb_avg <= amb_acc[20:9];
    end
end

wire [12:0] amb_pos = {1'b0, amb_avg} - 13'd2048;
wire signed [12:0] amb_d = AMB_INVERT ? $signed(amb_pos) : -$signed(amb_pos);
wire signed [12:0] amb_s = amb_d >>> 6;
wire signed [12:0] amb_b = amb_s + 13'sd20;

reg [5:0] amb_code;
always @(posedge audio_mclk or posedge rst_aud) begin
    if (rst_aud)               amb_code <= 6'd20;
    else if (!EN_AMBIENT)      amb_code <= 6'd20;
    else if (amb_s >  13'sd20) amb_code <= 6'd40;
    else if (amb_s < -13'sd20) amb_code <= 6'd0;
    else                       amb_code <= amb_b[5:0];
end

wire [55:0] dbg_bus;

wire [31:0] dbg_geom;

wire        img_load_busy_vis = dbg_bus[2] & ~dbg_bus[16] & ~music_driving;

reg [24:0] hb_cnt;
reg        hb;

always @(posedge clk or posedge rst_all) begin
    if (rst_all) begin
        hb_cnt <= 25'd0;
        hb     <= 1'b0;
    end else if (hb_cnt == 25'd24_999_999) begin
        hb_cnt <= 25'd0;
        hb     <= ~hb;
    end else begin
        hb_cnt <= hb_cnt + 25'd1;
    end
end

assign led[0] = music_on ? wav_state[0] : ~dbg_bus[10];
assign led[1] = music_on ? wav_state[1] : ~dbg_bus[11];
assign led[2] = music_on ? wav_state[2] : dbg_bus[3];
assign led[3] = music_on ? wav_state[3] : hb;

sys_pll sys_pll_m0(
    .refclk     (clk),
    .clk0_out   (sd_card_clk),
    .clk1_out   (ext_mem_clk),
    .clk2_out   (ext_mem_clk_sft),
    .reset      (1'b0)
);

video_pll video_pll_m0(
    .refclk     (clk),
    .clk0_out   (video_clk),
    .clk1_out   (hdmi_5x_clk),
    .reset      (1'b0)
);

PLL_HDMI_AUDIO u_audio_pll(
    .refclk     (clk),
    .reset      (1'b0),
    .extlock    (audio_pll_lock),
    .clk0_out   (),
    .clk1_out   (),
    .clk2_out   (audio_mclk)
);

always @(posedge ext_mem_clk or posedge rst_all) begin
    if (rst_all)
        frame_write_toggle_mem <= 1'b0;
    else if (frame_write_finish)
        frame_write_toggle_mem <= ~frame_write_toggle_mem;
end

wire        kd_next_tgl, kd_auto_tgl;
wire        kd_prev_tgl;

wire        img_sec_read;
wire [31:0] img_sec_read_addr;
wire [7:0]  img_rd_data;
wire        img_rd_dv;
wire        img_rd_end;

sd_card_bmp #(
    .CLK_FREQ_HZ       (100_000_000),
    .SCAN_START_SECTOR (32'd0),
    .SCAN_MAX_SECTOR   (32'd524287),

    .SCAN_TARGET_COUNT (6'd12)
) sd_card_bmp_m0(
    .clk               (sd_card_clk),
    .rst               (rst_all),
    .key_next          (1'b1),
    .key_auto          (1'b1),
    .disp_adopted      (disp_adopted),
    .next_tgl          (kd_next_tgl),
    .prev_tgl          (kd_prev_tgl),
    .play_mode         (menu_play_mode),
    .beat_evt          (beat_evt),
    .auto_period       (menu_period),
    .state_code        (state_code),
    .dbg_geom          (dbg_geom),
    .o_img_lvl         (img_lvl),
    .bmp_width         (16'd640),
    .bmp_height        (16'd480),
    .display_valid     (display_valid),

    .write_finish_toggle(frame_write_toggle_mem),
    .write_buf_idx     (write_buf_idx),
    .disp_buf_idx      (disp_buf_idx),

    .write_req         (sd_card_write_req),
    .write_req_ack     (sd_card_write_req_ack),
    .write_en          (sd_card_write_en),
    .write_data        (sd_card_write_data),
    .write_full        (sd_card_write_full),

    .sd_init_done      (sd_init_done),
    .sd_sec_read       (img_sec_read),
    .sd_sec_read_addr  (img_sec_read_addr),
    .sd_sec_read_data  (img_rd_data),
    .sd_sec_read_data_valid (img_rd_dv),
    .sd_sec_read_end   (img_rd_end),

    .dbg_bus           (dbg_bus)
);

wire        mus_sec_read;
wire [31:0] mus_sec_read_addr;

wire [7:0]  mus_rd_data;
wire        mus_rd_dv;
wire        mus_rd_end;

localparam [1:0] ARB_IDLE = 2'd0, ARB_RUN = 2'd1, ARB_SETTLE = 2'd2;

reg  [24:0] sd_wd;
wire        sd_wd_exp   = (sd_wd == 25'd29_999_999);
wire        sd_ctrl_rst = rst_all | sd_wd_exp;

reg  [1:0] arb_st;
reg        sd_owner;
reg  [3:0] arb_low;

wire       img_req   = img_sec_read;

wire       mus_req   = mus_sec_read & (music_on | wav_count_busy);
wire       arb_any   = img_req | mus_req;
wire       arb_grun  = (arb_st == ARB_RUN);
wire       arb_cur   = sd_owner ? mus_req : img_req;
wire       arb_aband = (arb_low == 4'd15) && !arb_cur;

wire       img_owns  = arb_grun & ~sd_owner;
wire       mus_owns  = arb_grun &  sd_owner;

assign img_rd_data = img_owns ? sd_sec_read_data : 8'd0;
assign img_rd_dv   = img_owns & sd_sec_read_data_valid;
assign img_rd_end  = img_owns & sd_sec_read_end & img_sec_read;
assign mus_rd_data = mus_owns ? sd_sec_read_data : 8'd0;
assign mus_rd_dv   = mus_owns & sd_sec_read_data_valid;
assign mus_rd_end  = mus_owns & sd_sec_read_end & mus_sec_read;

wire        sd_sec_read;
wire [31:0] sd_sec_read_addr;

assign sd_sec_read      = arb_grun && (sd_owner ? mus_req : img_req);
assign sd_sec_read_addr = sd_owner ? mus_sec_read_addr : img_sec_read_addr;

wire arb_pick_mus = (img_req & mus_req) ? ~sd_owner : mus_req;

always @(posedge sd_card_clk or posedge rst_all) begin
    if (rst_all) begin
        arb_st   <= ARB_IDLE;
        sd_owner <= 1'b0;
        arb_low  <= 4'd0;
    end else begin
        if (sd_wd_exp) begin
            arb_low <= 4'd0;
        end else if (arb_cur)  arb_low <= 4'd0;
        else if (arb_low != 4'd15) arb_low <= arb_low + 4'd1;

        if (sd_wd_exp) begin
            arb_st   <= ARB_IDLE;
            sd_owner <= 1'b0;
        end else case (arb_st)
            ARB_IDLE: if (arb_any) begin
                          arb_st   <= ARB_RUN;
                          sd_owner <= arb_pick_mus;
                      end
            ARB_RUN : if (sd_sec_read_end || arb_aband) arb_st <= ARB_SETTLE;
            default : if (arb_any) begin
                          arb_st   <= ARB_RUN;
                          sd_owner <= arb_pick_mus;
                      end else begin
                          arb_st   <= ARB_IDLE;
                      end
        endcase
    end
end

always @(posedge sd_card_clk or posedge rst_all) begin
    if (rst_all)        sd_wd <= 25'd0;
    else if (sd_wd_exp) sd_wd <= 25'd0;
    else if (arb_st == ARB_RUN && !sd_sec_read_end) sd_wd <= sd_wd + 25'd1;
    else                sd_wd <= 25'd0;
end

sd_card_top #(
    .SPI_LOW_SPEED_DIV  (248),
    .SPI_HIGH_SPEED_DIV (0)
) sd_card_top_m0 (
    .clk                    (sd_card_clk),
    .rst                    (sd_ctrl_rst),
    .SD_nCS                 (sd_ncs),
    .SD_DCLK                (sd_dclk),
    .SD_MOSI                (sd_mosi),
    .SD_MISO                (sd_miso),
    .sd_init_done           (sd_init_done),
    .sd_sec_read            (sd_sec_read),
    .sd_sec_read_addr       (sd_sec_read_addr),
    .sd_sec_read_data       (sd_sec_read_data),
    .sd_sec_read_data_valid (sd_sec_read_data_valid),
    .sd_sec_read_end        (sd_sec_read_end),
    .sd_sec_write           (1'b0),
    .sd_sec_write_addr      (32'd0),
    .sd_sec_write_data      (8'd0),
    .sd_sec_write_data_req  (),
    .sd_sec_write_end       ()
);

wire [3:0]  wav_state;
wire        wav_found;
wire [8:0]  wav_fifo_used;
wire [8:0]  wav_wptr, wav_rptr;
wire [7:0]  dbg_pop_k;
wire [8:0]  dbg_used;

wire        pcm_rd_req;
wire [15:0] pcm_data;
wire        pcm_empty;

wav_player #(
    .SCAN_FROM_SECTOR (32'd0),
    .SCAN_MAX_SECTOR  (32'd1048575)
) u_wav_player (
    .clk                    (sd_card_clk),
    .rst                    (rst_all),
    .sd_init_done           (sd_init_done),
    .sd_sec_read            (mus_sec_read),
    .sd_sec_read_addr       (mus_sec_read_addr),
    .sd_sec_read_data       (mus_rd_data),
    .sd_sec_read_data_valid (mus_rd_dv),
    .sd_sec_read_end        (mus_rd_end),
    .play_en                (music_on),
    .count_req              (menu_open),
    .count_busy             (wav_count_busy),
    .next_tgl               (menu_next_tgl),
    .prev_tgl               (menu_prev_tgl),
    .trk_now                (menu_trk_now),
    .trk_total              (menu_trk_total),
    .aclk                   (audio_mclk),
    .rd_req                 (pcm_rd_req),
    .rd_data                (pcm_data),
    .rd_empty               (pcm_empty),
    .state_code             (wav_state),
    .wav_found              (wav_found),
    .fifo_used              (wav_fifo_used),
    .dbg_wptr               (wav_wptr),
    .dbg_rptr               (wav_rptr),
    .dbg_pop_k_o            (dbg_pop_k),
    .dbg_used_o             (dbg_used),
    .dbg_data_ofs           (),
    .dbg_chn                (),
    .dbg_srate              ()
);

assign music_on = menu_music_on;

wire        stuck_valid = dbg_bus[55];

function [3:0] cnt_tens;
    input [5:0] c;
    begin

        cnt_tens = (c >= 6'd50) ? 4'd5 : (c >= 6'd40) ? 4'd4 : (c >= 6'd30) ? 4'd3 :
                   (c >= 6'd20) ? 4'd2 : (c >= 6'd10) ? 4'd1 : 4'd0;
    end
endfunction
function [3:0] cnt_ones;
    input [5:0] c;
    begin
        cnt_ones = c - (cnt_tens(c) * 6'd10);
    end
endfunction

reg  bri_t1, bri_t2, bri_t3;
reg  [11:0] amb_lat_v;
reg  bri_tgl_v;
reg  [8:0]  ao_show;

wire [13:0] ao_pk3 = {2'd0, amb_lat_v} + {2'd0, amb_lat_v} + {2'd0, amb_lat_v};
wire [8:0]  ao_pk9 = (ao_pk3[13:3] > 14'd511) ? 9'd511 : ao_pk3[13:3];

always @(posedge video_clk or posedge rst_all) begin
    if (rst_all) begin
        amb_lat_v <= 12'd2048;
        bri_tgl_v <= 1'b0;
    end else if ((vs_bd != vs) && vs) begin
        amb_lat_v <= amb_avg;
        bri_tgl_v <= ~bri_tgl_v;
    end
end

always @(posedge clk or posedge rst_all) begin
    if (rst_all) begin
        bri_t1  <= 1'b0; bri_t2 <= 1'b0; bri_t3 <= 1'b0;
        ao_show <= 9'd256;
    end else begin
        bri_t1 <= bri_tgl_v;
        bri_t2 <= bri_t1;
        bri_t3 <= bri_t2;
        if (bri_t2 != bri_t3)
            ao_show <= ao_pk9;
    end
end

function [3:0] amb_hund;
    input [9:0] v;
    begin
        amb_hund = (v >= 10'd900) ? 4'd9 : (v >= 10'd800) ? 4'd8 : (v >= 10'd700) ? 4'd7 :
                   (v >= 10'd600) ? 4'd6 : (v >= 10'd500) ? 4'd5 : (v >= 10'd400) ? 4'd4 :
                   (v >= 10'd300) ? 4'd3 : (v >= 10'd200) ? 4'd2 : (v >= 10'd100) ? 4'd1 : 4'd0;
    end
endfunction
function [9:0] amb_rem;
    input [9:0] v;
    begin
        amb_rem = v - amb_hund(v) * 10'd100;
    end
endfunction
function [3:0] amb_tens;
    input [9:0] v;
    reg   [9:0] r;
    begin
        r = amb_rem(v);
        amb_tens = (r >= 10'd90) ? 4'd9 : (r >= 10'd80) ? 4'd8 : (r >= 10'd70) ? 4'd7 :
                   (r >= 10'd60) ? 4'd6 : (r >= 10'd50) ? 4'd5 : (r >= 10'd40) ? 4'd4 :
                   (r >= 10'd30) ? 4'd3 : (r >= 10'd20) ? 4'd2 : (r >= 10'd10) ? 4'd1 : 4'd0;
    end
endfunction
function [3:0] amb_ones;
    input [9:0] v;
    begin
        amb_ones = amb_rem(v) - amb_tens(v) * 10'd10;
    end
endfunction
wire [9:0] ao_dec = {1'b0, ao_show};

wire [2:0]  disp_buf_used;
wire [8:0]  trans_fade;
wire [10:0] wipe_thr;

reg [2:0] disp_buf_idx_d;
always @(posedge video_clk or posedge rst_all) begin
    if (rst_all)
        disp_buf_idx_d <= 3'd0;
    else
        disp_buf_idx_d <= disp_buf_idx;
end
wire        swap_pulse  = (disp_buf_idx != disp_buf_idx_d);
assign trans_busy = (trans_fade != 9'd256);

wire [23:0] seg_raw = stuck_valid
                    ? {(dbg_bus[31] ? 4'hF : 4'hA), 3'b000, dbg_bus[54:50], 2'b00, dbg_bus[49:40]}
                    : {cnt_tens(dbg_bus[29:24]), cnt_ones(dbg_bus[29:24]),
                       4'h0,
                       amb_hund(ao_dec), amb_tens(ao_dec), amb_ones(ao_dec)};

wire [7:0]  pop_k  = dbg_pop_k;
wire [8:0]  used9  = dbg_used;
wire [5:0]  pop_k6 = (pop_k > 8'd59) ? 6'd59 : pop_k[5:0];
wire [5:0]  used6  = (used9 > 9'd59) ? 6'd59 : used9[5:0];

wire [23:0] seg_music = {cnt_tens(pop_k6),  cnt_ones(pop_k6),
                         cnt_tens(used6),   cnt_ones(used6),
                         4'd0,              wav_state};
wire [23:0] dbg_seg_mux = music_on ? seg_music : seg_raw;

reg [24:0] slow_cnt;
reg [23:0] seg_lat;
always @(posedge clk or posedge rst_all) begin
    if (rst_all) begin
        slow_cnt <= 25'd0;
        seg_lat  <= 24'd0;
    end else if (slow_cnt == 25'd19999999) begin
        slow_cnt <= 25'd0;
        seg_lat  <= dbg_seg_mux;
    end else begin
        slow_cnt <= slow_cnt + 25'd1;
    end
end

wire [23:0] dbg_seg = seg_lat;
wire [6:0]  seg_h2, seg_h1, seg_h0, seg_w2, seg_w1, seg_w0;

seg_decoder seg_dec_h2(.bin_data(dbg_seg[23:20]), .seg_data(seg_h2));
seg_decoder seg_dec_h1(.bin_data(dbg_seg[19:16]), .seg_data(seg_h1));
seg_decoder seg_dec_h0(.bin_data(dbg_seg[15:12]), .seg_data(seg_h0));
seg_decoder seg_dec_w2(.bin_data(dbg_seg[11:8]),  .seg_data(seg_w2));
seg_decoder seg_dec_w1(.bin_data(dbg_seg[7:4]),   .seg_data(seg_w1));
seg_decoder seg_dec_w0(.bin_data(dbg_seg[3:0]),   .seg_data(seg_w0));

seg_scan seg_scan_m0(
    .clk               (clk),
    .rst_n             (rst_n),
    .seg_sel           (seg_sel),
    .seg_data          (seg_data),
    .seg_data_0        ({1'b1,seg_h2}),
    .seg_data_1        ({1'b1,seg_h1}),
    .seg_data_2        ({1'b1,seg_h0}),
    .seg_data_3        ({1'b1,seg_w2}),
    .seg_data_4        ({1'b1,seg_w1}),
    .seg_data_5        ({1'b1,seg_w0})
);

video_timing_data video_timing_data_m0(
    .video_clk         (video_clk),
    .rst               (rst_all),
    .read_req          (video_read_req),
    .read_req_ack      (video_read_req_ack),
    .hs                (hs_0),
    .vs                (vs_0),
    .de                (de_0)
);

video_delay video_delay_m0(
    .video_clk         (video_clk),
    .rst               (rst_all),
    .read_en           (video_read_en),
    .read_data         (video_read_data[31:8]),
    .hs                (hs_0),
    .vs                (vs_0),
    .de                (de_0),
    .hs_r              (hs),
    .vs_r              (vs),
    .de_r              (de),
    .vout_data         (vout_data_raw)
);

wire [23:0] dbg_color;
assign dbg_color = (!dbg_bus[0])             ? 24'h0000FF :
                   (!dbg_bus[7])             ? 24'hFFFF00 :
                   (dbg_bus[6:4] == 3'd0)    ? 24'h00FFFF :
                   (dbg_bus[2])              ? 24'hFF8000 :
                                              24'hFF00FF;

assign vout_data = display_valid ? vout_data_raw : dbg_color;

wire [6:0]  osd_bright;
wire [6:0]  osd_contrast;

wire        menu_bri_en;
wire [6:0]  menu_bri;
wire        menu_con_en;
wire [6:0]  menu_con;
wire        menu_banner_on;
wire        menu_music_on;
wire        menu_status_on;
wire        menu_spec_on;
wire [1:0]  menu_vol_sel;

wire        kd_sw4_lvl, kd_sw4_up, kd_sw4_dn;
wire        kd_nav_up, kd_nav_down, kd_nav_ok, kd_nav_cancel;
wire        kd_hot_status, kd_hot_banner, kd_hot_spec;
wire        kd_hot_status_dn, kd_hot_banner_dn, kd_hot_spec_dn;

key_decode u_key_decode (
    .clk        (video_clk),
    .rst_n      (~rst_all),
    .sw1        (sw1),
    .sw2        (sw2),
    .sw3        (sw3),
    .sw4        (sw4),
    .key2       (key1),
    .key3       (key3),
    .key4       (key2),
    .sw4_lvl    (kd_sw4_lvl),
    .sw4_up     (kd_sw4_up),
    .sw4_dn     (kd_sw4_dn),
    .nav_up     (kd_nav_up),
    .nav_down   (kd_nav_down),
    .nav_ok     (kd_nav_ok),
    .nav_cancel (kd_nav_cancel),
    .hot_status (kd_hot_status),
    .hot_banner (kd_hot_banner),
    .hot_spec   (kd_hot_spec),
    .hot_status_dn(kd_hot_status_dn),
    .hot_banner_dn(kd_hot_banner_dn),
    .hot_spec_dn(kd_hot_spec_dn),
    .menu_open  (menu_open),
    .next_tgl   (kd_next_tgl),
    .prev_tgl   (kd_prev_tgl),
    .auto_tgl   (kd_auto_tgl)
);

reg kd_next_tgl_d, kd_auto_tgl_d;
always @(posedge video_clk or posedge rst_all) begin
    if (rst_all) begin
        kd_next_tgl_d <= 1'b0;
        kd_auto_tgl_d <= 1'b0;
    end else begin
        kd_next_tgl_d <= kd_next_tgl;
        kd_auto_tgl_d <= kd_auto_tgl;
    end
end
wire img_key_evt   = (kd_next_tgl != kd_next_tgl_d) | (kd_auto_tgl != kd_auto_tgl_d);

wire ext_music_off = 1'b0;

osd_ctrl u_osd_ctrl (
    .clk      (clk),
    .rst_n    (rst_n),
    .sw1      (sw1),
    .sw2      (sw2),
    .sw3      (sw3),
    .sw4      (1'b0),
    .menu_open(menu_open),
    .m_bri_en (menu_bri_en),
    .m_bri_val(menu_bri),
    .m_con_en (menu_con_en),
    .m_con_val(menu_con),
    .bright   (osd_bright),
    .contrast (osd_contrast)
);

wire       spec_bar_we;
wire [5:0] spec_bar_addr;
wire [5:0] spec_bar_val;

wire [15:0] av_env_fast;
wire [15:0] av_env_slow;
wire [1:0]  av_timbre;

audio_viz u_audio_viz (
    .clk         (video_clk),
    .rst_n       (~rst_all),
    .audio_valid (audio_valid),
    .audio_l     (audio_left_data),
    .audio_r     (audio_right_data),
    .bar_we      (spec_bar_we),
    .bar_addr    (spec_bar_addr),
    .bar_val     (spec_bar_val),
    .beat_evt    (beat_evt),
    .env_fast    (av_env_fast),
    .env_slow    (av_env_slow),
    .timbre_sel  (av_timbre)
);

assign disp_adopted = (disp_buf_used == disp_buf_idx);
wire [2:0] alt_idx;
trans_ctrl u_trans_ctrl (
    .clk         (video_clk),
    .rst_n       (~rst_all),
    .vs          (vs),
    .buf_idx_in  (disp_buf_idx),
    .swap_pulse  (swap_pulse),
    .buf_idx_out (disp_buf_used),
    .fade        (trans_fade),
    .alt_idx     (alt_idx),
    .cut_mode    (music_driving),
    .wipe_mode   (wipe_mode)
);

wire        buf_bank_alt = alt_idx[2];
wire [23:0] alt_a0 = buf_bank_alt ? BUF4_ADDR : BUF0_ADDR;
wire [23:0] alt_a1 = buf_bank_alt ? BUF5_ADDR : BUF1_ADDR;
wire [23:0] alt_a2 = buf_bank_alt ? 24'd0      : BUF2_ADDR;
wire [23:0] alt_a3 = buf_bank_alt ? 24'd0      : BUF3_ADDR;
reg  [23:0] alt_base;
always @(*) begin
    case (alt_idx[1:0])
        2'd0:    alt_base = alt_a0;
        2'd1:    alt_base = alt_a1;
        2'd2:    alt_base = alt_a2;
        default: alt_base = alt_a3;
    endcase
end

wire [10:0] alt_x = wipe_mode ? wipe_thr : 11'd640;

wire [23:0] vout_osd;
wire        de_osd;
wire        vs_osd;

reg [4:0] img_no_p1;
always @(posedge sd_card_clk or posedge rst_all) begin
    if (rst_all) img_no_p1 <= 5'd0;
    else         img_no_p1 <= dbg_bus[23:19] + 5'd1;
end

localparam EN_BREATH = 1'b1;
localparam EN_TIMBRE = 1'b1;

assign music_driving = (menu_play_mode == 2'd2) && menu_music_on;

assign music_hardcut = music_driving && (trans_sel_eff == 2'd0);

wire signed [16:0] env_d  = $signed({1'b0, av_env_fast}) - $signed({1'b0, av_env_slow});

wire signed [16:0] env_s6 = env_d >>> 2;
wire signed [16:0] env_s7 = env_d >>> 3;
wire signed [16:0] env_s9 = env_d >>> 5;
wire signed [16:0] env_s  = (img_lvl == 2'd0) ? env_s6 :
                            (img_lvl == 2'd2) ? env_s9 :
                                                env_s7;
wire signed [5:0]  env_c = (env_s >  17'sd10) ?  6'sd10 :
                           (env_s < -17'sd10) ? -6'sd10 : env_s[5:0];

reg  vs_bd_a;
reg  [5:0] amb_sync;
reg  [5:0] amb_use;

reg signed [5:0] breath_off;
always @(posedge video_clk or posedge rst_all) begin
    if (rst_all) begin
        vs_bd_a    <= 1'b0;
        vs_bd      <= 1'b0;
        amb_sync   <= 6'd20;
        amb_use    <= 6'd20;
        breath_off <= 6'sd0;
    end else begin
        vs_bd_a <= vs;
        vs_bd   <= vs;
        if ((vs_bd_a != vs) && vs) begin
            amb_sync <= amb_code;

            if      (!EN_AMBIENT)         amb_use <= 6'd20;
            else if (amb_sync > amb_use)  amb_use <= amb_use + 6'd1;
            else if (amb_sync < amb_use)  amb_use <= amb_use - 6'd1;
        end
        if ((vs_bd != vs) && vs)
            breath_off <= (EN_BREATH && music_driving) ? env_c : 6'sd0;
    end
end
wire signed [5:0] amb_off = $signed(amb_use) - 6'sd20;

/*
手动亮度 + 呼吸 + 环境光，三项各自补成 9 位有符号再相加。拼接表达式默认
无符号，只要有一个操作数没包 $signed()，整个和式就按无符号算，负的补偿会
变成一个大正数。
*/
wire signed [8:0] bright_s = $signed({2'b00, osd_bright})
                           + $signed({{3{breath_off[5]}}, breath_off})
                           + $signed({{3{amb_off[5]}},    amb_off});
always @(posedge video_clk or posedge rst_all) begin
    if (rst_all)
        osd_bright_use <= 7'd50;
    else if (bright_s < 9'sd0)
        osd_bright_use <= 7'd0;
    else if (bright_s > 9'sd99)
        osd_bright_use <= 7'd99;
    else
        osd_bright_use <= bright_s[6:0];
end

osd_insert u_osd_insert (
    .clk       (video_clk),
    .rst_n     (~rst_all),
    .de        (de),
    .vs        (vs),
    .rgb_in    (vout_data),
    .img_no    (img_no_p1),
    .img_total (dbg_bus[28:24]),
    .bright    (osd_bright_use),
    .timbre_sel(av_timbre),
    .timbre_en (EN_TIMBRE && music_driving),
    .contrast  (osd_contrast),
    .key_cnt   (dbg_bus[39:32]),
    .load_busy (img_load_busy_vis),
    .load_row  (dbg_geom[22:13]),
    .sw4_lvl   (kd_sw4_lvl),
    .sw4_up    (kd_sw4_up),
    .sw4_dn    (kd_sw4_dn),
    .nav_up    (kd_nav_up),
    .nav_down  (kd_nav_down),
    .nav_ok    (kd_nav_ok),
    .nav_cancel(kd_nav_cancel),
    .hot_status(kd_hot_status),
    .hot_banner(kd_hot_banner),
    .hot_spec  (kd_hot_spec),
    .hot_status_dn(kd_hot_status_dn),
    .hot_banner_dn(kd_hot_banner_dn),
    .hot_spec_dn(kd_hot_spec_dn),
    .menu_open (menu_open),
    .cur_bright(osd_bright),
    .cur_contrast(osd_contrast),
    .cur_auto  (dbg_bus[3]),
    .out_play_mode(menu_play_mode),
    .o_wipe_mode (wipe_mode),
    .o_wipe_thr  (wipe_thr),
    .o_trans_sel (trans_sel_eff),
    .auto_ext_tgl(kd_auto_tgl),
    .out_period(menu_period),
    .out_bri_en(menu_bri_en),
    .out_bri   (menu_bri),
    .out_con_en(menu_con_en),
    .out_con   (menu_con),
    .out_spec_on(menu_spec_on),
    .out_banner_on(menu_banner_on),
    .out_music_on(menu_music_on),
    .out_prev_tgl(menu_prev_tgl),
    .out_next_tgl(menu_next_tgl),
    .trk_now   (menu_trk_now),
    .trk_total (menu_trk_total),
    .ext_music_off(ext_music_off),
    .out_status_on(menu_status_on),
    .out_vol_sel(menu_vol_sel),
    .bar_we    (spec_bar_we),
    .bar_addr  (spec_bar_addr),
    .bar_val   (spec_bar_val),
    .fade_lvl  (trans_fade),
    .rgb_out   (vout_osd),
    .de_out    (de_osd),
    .vs_out    (vs_osd)
);

wire        buf_bank_rd = disp_buf_used[2];
wire        buf_bank_wr = write_buf_idx[2];

wire [23:0] rd_a0 = buf_bank_rd ? BUF4_ADDR : BUF0_ADDR;
wire [23:0] rd_a1 = buf_bank_rd ? BUF5_ADDR : BUF1_ADDR;
wire [23:0] rd_a2 = buf_bank_rd ? 24'd0      : BUF2_ADDR;
wire [23:0] rd_a3 = buf_bank_rd ? 24'd0      : BUF3_ADDR;

wire [23:0] wr_a0 = buf_bank_wr ? BUF4_ADDR : BUF0_ADDR;
wire [23:0] wr_a1 = buf_bank_wr ? BUF5_ADDR : BUF1_ADDR;
wire [23:0] wr_a2 = buf_bank_wr ? 24'd0      : BUF2_ADDR;
wire [23:0] wr_a3 = buf_bank_wr ? 24'd0      : BUF3_ADDR;

frame_read_write #(
    .WRITE_V_FLIP     (1),
    .FRAME_WIDTH      (640),
    .FRAME_HEIGHT     (480)
) frame_read_write_m0(
    .mem_clk           (ext_mem_clk),
    .rst               (rst_all),
    .Sdr_init_done     (Sdr_init_done),
    .Sdr_init_ref_vld  (Sdr_init_ref_vld),
    .Sdr_busy          (Sdr_busy),

    .App_rd_en         (App_rd_en),
    .App_rd_addr       (App_rd_addr),
    .Sdr_rd_en         (Sdr_rd_en),
    .Sdr_rd_dout       (Sdr_rd_dout),

    .read_clk          (video_clk),
    .read_req          (video_read_req),
    .read_req_ack      (video_read_req_ack),
    .read_finish       (),
    .read_addr_0       (rd_a0),
    .read_addr_1       (rd_a1),
    .read_addr_2       (rd_a2),
    .read_addr_3       (rd_a3),
    .read_addr_index   (disp_buf_used[1:0]),
    .read_len          (FRAME_PIXELS),
    .read_addr_alt     (alt_base),
    .read_alt_x        (alt_x),
    .read_en           (video_read_en),
    .read_data         (video_read_data),

    .App_wr_en         (App_wr_en),
    .App_wr_addr       (App_wr_addr),
    .App_wr_din        (App_wr_din),
    .App_wr_dm         (App_wr_dm),

    .write_clk         (sd_card_clk),
    .write_req         (sd_card_write_req),
    .write_req_ack     (sd_card_write_req_ack),
    .write_finish      (frame_write_finish),
    .write_addr_0      (wr_a0),
    .write_addr_1      (wr_a1),
    .write_addr_2      (wr_a2),
    .write_addr_3      (wr_a3),
    .write_addr_index  (write_buf_idx[1:0]),
    .write_len         (FRAME_PIXELS),
    .write_en          (sd_card_write_en),
    .write_data        (sd_card_write_data),
    .write_fifo_full   (sd_card_write_full)
);

sdram U3(
    .Clk               (ext_mem_clk),
    .Clk_sft           (ext_mem_clk_sft),
    .Rst               (rst_all),
    .Sdr_init_done     (Sdr_init_done),
    .Sdr_init_ref_vld  (Sdr_init_ref_vld),
    .Sdr_busy          (Sdr_busy),
    .App_wr_en         (App_wr_en),
    .App_wr_addr       (App_wr_addr),
    .App_wr_dm         (App_wr_dm),
    .App_wr_din        (App_wr_din),
    .App_rd_en         (App_rd_en),
    .App_rd_addr       (App_rd_addr),
    .Sdr_rd_en         (Sdr_rd_en),
    .Sdr_rd_dout       (Sdr_rd_dout)
);

reg audio_swap_tgl;
always @(posedge video_clk or posedge rst_all) begin
    if (rst_all)
        audio_swap_tgl <= 1'b0;
    else if (swap_pulse)
        audio_swap_tgl <= ~audio_swap_tgl;
end

localparam [1:0] AUDIO_LINK_MODE = 2'd0;
localparam [3:0] AUDIO_VOLUME    = 4'd8;

hdmi_audio_tone_i2s_64fs #(
    .PHASE_INC (32'd39370534),
    .AMP       (24'sd2000000)
) u_hdmi_audio_tone_i2s_64fs (
    .I_mclk      (audio_mclk),
    .I_rst       (rst_aud),

    .I_note_sel  (dbg_bus[21:19]),
    .I_trig_tgl  (audio_swap_tgl),
    .I_mode      (AUDIO_LINK_MODE),
    .I_vol       (AUDIO_VOLUME),

    .I_pcm       (pcm_data),
    .I_pcm_en    (music_on),
    .I_vol_sel   (menu_vol_sel),
    .O_pcm_rd    (pcm_rd_req),
    .O_i2s_BCLK  (audio_i2s_bclk),
    .O_i2s_LRCK  (audio_i2s_lrck),
    .O_i2s_DOUT  (audio_i2s_dout)
);

I2S_receiver u_I2S_receiver(
    .I_clk              (video_clk),
    .I_rst              (rst_all),
    .I_i2s_BCLK         (audio_i2s_bclk),
    .I_i2s_LRCK         (audio_i2s_lrck),
    .I_i2s_DOUT         (audio_i2s_dout),
    .O_audio_valid      (audio_valid),
    .O_audio_left_data  (audio_left_data),
    .O_audio_right_data (audio_right_data)
);

audio_arc_calculate #(
    .ACR_N         (6144)
) u_audio_arc_calculate (
    .I_clk         (video_clk),
    .I_rst         (rst_all),
    .I_audio_valid (audio_valid),
    .O_acr_valid   (acr_valid),
    .O_acr_cts     (acr_cts),
    .O_acr_n       (acr_n)
);

video_rgb_to_axis_640x480 u_video_rgb_to_axis_640x480(
    .I_clk         (video_clk),
    .I_rst         (rst_all),
    .I_vs          (vs_osd),
    .I_de          (de_osd),
    .I_rgb         (vout_osd),
    .O_video_user  (axis_s_user),
    .O_video_valid (axis_s_valid),
    .O_video_last  (axis_s_last),
    .O_video_data  (axis_s_data)
);

startup_pulse #(
    .CNT_MAX(20'd100000)
) u_startup_pulse (
    .I_clk   (video_clk),
    .I_rst   (rst_all),
    .O_pulse (edid_trig)
);

hdmi_1_4b_transmitter_core_wrapper #(
    .DEVICE                 ( "EG"       ),
    .HTOTAL                 ( 800        ),
    .HSA                    ( 96         ),
    .HFP                    ( 16         ),
    .HBP                    ( 48         ),
    .HACTIVE                ( 640        ),
    .VTOTAL                 ( 525        ),
    .VSA                    ( 2          ),
    .VFP                    ( 10         ),
    .VBP                    ( 33         ),
    .VACTIVE                ( 480        ),
    .VIDEO_VIC              ( 1          ),
    .VIDEO_TPG              ( "Disable"  ),
    .VIDEO_FORMAT           ( "RGB"      ),
    .AUDIO_SAMPLE_RATE      ( "48K"      ),
    .IIC_SCL_DIV            ( 250        )
) u_hdmi_1_4b_transmitter_core_wrapper(
    .I_pixel_clk        (video_clk),
    .I_rst              (rst_all),
    .I_edid_read_trig   (edid_trig),
    .O_edid_read_valid  (edid_valid),
    .O_edid_read_data   (edid_data),

    .I_axis_s_user      (axis_s_user),
    .I_axis_s_valid     (axis_s_valid),
    .I_axis_s_last      (axis_s_last),
    .I_axis_s_data      (axis_s_data),
    .O_axis_s_ready     (axis_s_ready),

    .I_audio_valid      (audio_valid),
    .I_audio_left_data  (audio_left_data),
    .I_audio_right_data (audio_right_data),
    .I_acr_valid        (acr_valid),
    .I_acr_cts          (acr_cts),
    .I_acr_n            (acr_n),

    .O_video_locked     (),
    .O_ddc_scl          (HDMI_DDC_SCL),
    .IO_ddc_sda         (HDMI_DDC_SDA),

    .O_ch0_tmds_data    (tmds_ch0_data),
    .O_ch1_tmds_data    (tmds_ch1_data),
    .O_ch2_tmds_data    (tmds_ch2_data),
    .O_clk_tmds_data    (tmds_clk_data)
);

hdmi_phy_wrapper #(
    .DEVICE ( "EG" )
) u_hdmi2phy_wrapper(
    .I_pixel_clk        (video_clk),
    .I_serial_clk       (hdmi_5x_clk),
    .I_rst              (rst_all),
    .I_tmds_channel_0   (tmds_ch0_data),
    .I_tmds_channel_1   (tmds_ch1_data),
    .I_tmds_channel_2   (tmds_ch2_data),
    .I_tmds_channel_clk (tmds_clk_data),
    .O_tmds_ch0_p       (HDMI_D0_P),
    .O_tmds_ch1_p       (HDMI_D1_P),
    .O_tmds_ch2_p       (HDMI_D2_P),
    .O_tmds_clk_p       (HDMI_CLK_P)
);

endmodule
