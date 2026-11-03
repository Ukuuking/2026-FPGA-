// 在视频像素流中插入 OSD 像素，保持 hs/vs/de 的时序与延迟不变。

module osd_insert (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        de,
    input  wire        vs,
    input  wire [23:0] rgb_in,

    input  wire [4:0]  img_no,
    input  wire [4:0]  img_total,
    input  wire [6:0]  bright,
    input  wire [6:0]  contrast,
    input  wire [7:0]  key_cnt,
    input  wire        load_busy,
    input  wire [9:0]  load_row,

    input  wire        sw4_lvl,
    input  wire        sw4_up,
    input  wire        sw4_dn,
    input  wire        nav_up,
    input  wire        nav_down,
    input  wire        nav_ok,
    input  wire        nav_cancel,
    input  wire        hot_status,
    input  wire        hot_banner,
    input  wire        hot_spec,
    input  wire        hot_status_dn,
    input  wire        hot_banner_dn,
    input  wire        hot_spec_dn,
    input  wire [6:0]  cur_bright,
    input  wire [6:0]  cur_contrast,
    input  wire        cur_auto,

    output wire        menu_open,
    output wire [1:0]  out_play_mode,
    output wire        o_wipe_mode,
    output wire [10:0] o_wipe_thr,
    output wire [1:0]  o_trans_sel,
    input  wire        auto_ext_tgl,
    output wire [1:0]  out_period,
    output wire        out_bri_en,
    output wire [6:0]  out_bri,
    output wire        out_con_en,
    output wire [6:0]  out_con,
    output wire        out_spec_on,
    output wire        out_music_on,
    output wire        out_prev_tgl,
    output wire        out_next_tgl,
    input  wire [3:0]  trk_now,
    input  wire [3:0]  trk_total,
    input  wire        ext_music_off,
    output wire        out_banner_on,
    output wire        out_status_on,
    output wire [1:0]  out_vol_sel,

    input  wire        bar_we,
    input  wire [5:0]  bar_addr,
    input  wire [5:0]  bar_val,

    input  wire [8:0]  fade_lvl,

    input  wire [1:0]  timbre_sel,
    input  wire        timbre_en,

    output wire [23:0] rgb_out,
    output wire        de_out,
    output wire        vs_out
);

    reg [10:0] px, py;
    reg        de_d;
    reg        vs_d;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            px   <= 11'd0;
            py   <= 11'd0;
            de_d <= 1'b0;
            vs_d <= 1'b0;
        end else begin
            de_d <= de;
            vs_d <= vs;

            if (de)
                px <= de_d ? (px + 11'd1) : 11'd0;
            else
                px <= 11'd0;

            if (vs_d != vs)
                py <= 11'd0;
            else if (!de && de_d)
                py <= (py >= 11'd479) ? 11'd0 : (py + 11'd1);
        end
    end

    reg [23:0] rgb_d1;
    reg        de_d1;
    reg        vs_d1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rgb_d1 <= 24'd0;
            de_d1  <= 1'b0;
            vs_d1  <= 1'b0;
        end else begin
            rgb_d1 <= rgb_in;
            de_d1  <= de;
            vs_d1  <= vs;
        end
    end

    wire        menu_on;
    wire [7:0]  menu_ch;
    wire [3:0]  menu_cursor;
    wire        ind_on;
    wire [1:0]  out_trans_sel;
    osd_menu u_osd_menu (
        .clk         (clk),
        .rst_n       (rst_n),
        .sw4_lvl     (sw4_lvl),
        .sw4_up      (sw4_up),
        .sw4_dn      (sw4_dn),
        .nav_up      (nav_up),
        .nav_down    (nav_down),
        .nav_ok      (nav_ok),
        .nav_cancel  (nav_cancel),
        .hot_status  (hot_status),
        .hot_banner  (hot_banner),
        .hot_spec    (hot_spec),
        .hot_status_dn(hot_status_dn),
        .hot_banner_dn(hot_banner_dn),
        .hot_spec_dn (hot_spec_dn),
        .cur_bright  (cur_bright),
        .cur_contrast(cur_contrast),
        .cur_auto    (cur_auto),
        .y           (py),
        .col         (px[9:4]),
        .menu_on     (menu_on),
        .menu_ch     (menu_ch),
        .menu_cursor (menu_cursor),
        .out_spec_on (out_spec_on),
        .out_banner_on(out_banner_on),
        .out_status_on(out_status_on),
        .out_trans_sel(out_trans_sel),
        .out_music_on(out_music_on),
        .out_prev_tgl(out_prev_tgl),
        .out_next_tgl(out_next_tgl),
        .trk_now     (trk_now),
        .trk_total   (trk_total),
        .ext_music_off(ext_music_off),
        .out_ind_on  (ind_on),
        .out_vol_sel (out_vol_sel),
        .out_play_mode(out_play_mode),
        .auto_ext_tgl(auto_ext_tgl),
        .out_period  (out_period),
        .out_bri_en  (out_bri_en),
        .out_bri     (out_bri),
        .out_con_en  (out_con_en),
        .out_con     (out_con),
        .out_open    (menu_open)
    );

    wire [23:0] osd_rgb;
    wire [2:0]  osd_a;
    wire        osd_de;

    wire [1:0] trans_sel = timbre_en ? 2'd0 : out_trans_sel;

    wire [13:0] wipe_x20 = {5'b0, fade_lvl} * 14'd20;
    wire [10:0] wipe_thr = wipe_x20[12:3];

    wire        wipe_on  = (trans_sel == 2'd2) && (wipe_thr != 11'd640);

    osd_layer u_osd_layer (
        .clk       (clk),
        .rst_n     (rst_n),
        .de        (de),
        .vs        (vs),
        .wipe_on   (wipe_on),
        .wipe_thr  (wipe_thr),
        .x         (px),
        .y         (py),
        .img_no    (img_no),
        .img_total (img_total),
        .bright    (bright),
        .contrast  (contrast),
        .play_mode (out_play_mode),
        .key_cnt   (key_cnt),
        .trk_now   (trk_now),
        .trk_total (trk_total),
        .load_busy (load_busy),
        .load_row  (load_row),
        .ind_on    (ind_on),
        .menu_on   (menu_on),
        .menu_ch   (menu_ch),
        .menu_cursor(menu_cursor),
        .banner_on (out_banner_on),
        .status_on (out_status_on),
        .spec_on   (out_spec_on),
        .bar_we    (bar_we),
        .bar_addr  (bar_addr),
        .bar_val   (bar_val),
        .osd_rgb   (osd_rgb),
        .osd_a     (osd_a),
        .osd_de    (osd_de)
    );

    wire [23:0] mix_rgb;
    wire        mix_de;

    assign o_wipe_mode  = (trans_sel == 2'd2);
    assign o_wipe_thr   = wipe_thr;
    assign o_trans_sel  = trans_sel;

    wire [8:0] fade_eff = (trans_sel == 2'd1) ? fade_lvl : 9'd256;

    osd_blend u_osd_blend (
        .clk      (clk),
        .rst_n    (rst_n),
        .de       (de_d1),
        .vid_rgb  (rgb_d1),
        .osd_rgb  (osd_rgb),
        .osd_a    (osd_a),
        .bright   (bright),
        .constr   (contrast),
        .fade     (fade_eff),
        .out_rgb  (mix_rgb),
        .out_de   (mix_de)
    );

    reg vs_d2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            vs_d2 <= 1'b0;
        else
            vs_d2 <= vs_d1;
    end

    assign rgb_out = mix_rgb;
    assign de_out  = mix_de;
    assign vs_out  = vs_d2;

endmodule
