// 十四项参数菜单。维护选中项、各图层开关与参数值，输出菜单文本与高亮区域。

module osd_menu (
    input  wire        clk,
    input  wire        rst_n,

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

    input  wire [10:0] y,
    input  wire [5:0]  col,
    output wire        menu_on,
    output wire [7:0]  menu_ch,
    output wire [3:0]  menu_cursor,

    output reg         out_spec_on,
    output reg         out_banner_on,
    output reg         out_status_on,
    output reg         out_ind_on,
    output reg  [1:0]  out_vol_sel,
    output reg  [1:0]  out_trans_sel,
    output reg         out_music_on,
    output reg         out_prev_tgl,
    output reg         out_next_tgl,
    input  wire [3:0]  trk_now,
    input  wire [3:0]  trk_total,
    input  wire        ext_music_off,

    output reg  [1:0]  out_play_mode,
    input  wire        auto_ext_tgl,
    output reg  [1:0]  out_period,
    output reg         out_bri_en,
    output reg  [6:0]  out_bri,
    output reg         out_con_en,
    output reg  [6:0]  out_con,

    output reg         out_open
);

    localparam [10:0] MENU_Y0 = 11'd128;
    localparam [10:0] MENU_Y1 = 11'd368;
    localparam [5:0]  MENU_C0 = 6'd10;
    localparam [5:0]  MENU_C1 = 6'd30;
    localparam [5:0]  MENU_NW = 6'd20;
    localparam [3:0]  R_LAST   = 4'd14;

    localparam [3:0]  R_SPEC     = 4'd1;
    localparam [3:0]  R_BANNER   = 4'd2;
    localparam [3:0]  R_STATUS   = 4'd3;
    localparam [3:0]  R_IND      = 4'd4;
    localparam [3:0]  R_PLAY     = 4'd5;
    localparam [3:0]  R_PERIOD   = 4'd6;
    localparam [3:0]  R_BRIGHT   = 4'd7;
    localparam [3:0]  R_CONTRAST = 4'd8;
    localparam [3:0]  R_TRANS    = 4'd9;
    localparam [3:0]  R_MUSIC    = 4'd10;
    localparam [3:0]  R_MSW      = 4'd11;
    localparam [3:0]  R_VOL      = 4'd12;
    localparam [3:0]  R_RESET    = 4'd13;
    localparam [3:0]  R_EXIT     = 4'd14;

    reg       open_r;
    reg [3:0] cursor;
    reg       auto_ext_d;
    wire      auto_ext_evt = auto_ext_tgl ^ auto_ext_d;

    wire [1:0] next_pmode = (out_play_mode == 2'd2) ? 2'd0 : (out_play_mode + 2'd1);

    function [1:0] trans_for_mode;
        input [1:0] m;
        begin
            trans_for_mode = (m == 2'd1) ? 2'd2 : 2'd1;
        end
    endfunction

    reg [3:0] tn_s0, tn_s1, tt_s0, tt_s1;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tn_s0 <= 4'd0; tn_s1 <= 4'd0;
            tt_s0 <= 4'd0; tt_s1 <= 4'd0;
        end else begin
            tn_s0 <= trk_now;    tn_s1 <= tn_s0;
            tt_s0 <= trk_total;  tt_s1 <= tt_s0;
        end
    end
    wire [3:0] tn_c = tn_s1;
    wire [3:0] tt_c = tt_s1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            open_r        <= 1'b0;
            cursor        <= R_SPEC;
            out_spec_on   <= 1'b1;
            out_banner_on <= 1'b1;
            out_status_on <= 1'b1;
            out_ind_on    <= 1'b1;
            out_vol_sel   <= 2'd3;
            out_trans_sel <= 2'd1;
            out_music_on  <= 1'b0;
            out_prev_tgl  <= 1'b0;
            out_next_tgl  <= 1'b0;
            out_play_mode <= 2'd0;
            out_period    <= 2'd0;
            out_bri_en    <= 1'b0;
            out_bri       <= 7'd50;
            out_con_en    <= 1'b0;
            out_con       <= 7'd50;
        end else begin

            if (sw4_up) begin
                open_r   <= 1'b1;
                cursor   <= R_SPEC;
            end else if (sw4_dn || (open_r && nav_cancel)) begin
                open_r   <= 1'b0;
            end else if (open_r && nav_ok && (cursor == R_EXIT)) begin
                open_r   <= 1'b0;
            end

            if (open_r) begin
                if (nav_down)
                    cursor <= (cursor >= R_EXIT) ? R_SPEC : (cursor + 4'd1);
                else if (nav_up)
                    cursor <= (cursor <= R_SPEC) ? R_EXIT : (cursor - 4'd1);
            end

            if (hot_spec)       out_spec_on   <= 1'b1;
            if (hot_spec_dn)    out_spec_on   <= 1'b0;
            if (hot_banner)     out_banner_on <= 1'b1;
            if (hot_banner_dn)  out_banner_on <= 1'b0;
            if (hot_status)     out_status_on <= 1'b1;
            if (hot_status_dn)  out_status_on <= 1'b0;

            if (ext_music_off) begin
                out_music_on <= 1'b0;
                if (out_play_mode == 2'd2) begin
                    out_play_mode <= 2'd0;
                    out_trans_sel <= trans_for_mode(2'd0);
                end
            end

            auto_ext_d <= auto_ext_tgl;
            if (auto_ext_evt) begin
                out_play_mode <= next_pmode;
                out_trans_sel <= trans_for_mode(next_pmode);

                if (out_play_mode == 2'd1) out_music_on <= 1'b1;
            end

            if (open_r && nav_ok && (cursor != R_EXIT)) begin
                case (cursor)
                    R_SPEC:     out_spec_on   <= ~out_spec_on;
                    R_BANNER:   out_banner_on <= ~out_banner_on;
                    R_STATUS:   out_status_on <= ~out_status_on;
                    R_IND:      out_ind_on    <= ~out_ind_on;
                    R_TRANS:    out_trans_sel <= (out_trans_sel == 2'd2) ? 2'd0
                                                              : (out_trans_sel + 2'd1);

                    R_MUSIC: begin
                        out_music_on <= ~out_music_on;
                        if (out_music_on && (out_play_mode == 2'd2)) begin
                            out_play_mode <= 2'd0;
                            out_trans_sel <= trans_for_mode(2'd0);
                        end
                    end
                    R_MSW:      out_next_tgl  <= ~out_next_tgl;
                    R_VOL:      out_vol_sel   <= out_vol_sel + 2'd1;
                    R_IND:      out_ind_on    <= ~out_ind_on;
                    R_PLAY: begin
                        out_play_mode <= next_pmode;
                        out_trans_sel <= trans_for_mode(next_pmode);

                        if (out_play_mode == 2'd1) out_music_on <= 1'b1;
                    end
                    R_PERIOD:   out_period    <= out_period + 2'd1;
                    R_BRIGHT: begin

                        out_bri    <= (cur_bright >= 7'd90) ? 7'd30 : (cur_bright + 7'd10);
                        out_bri_en <= ~out_bri_en;
                    end
                    R_CONTRAST: begin
                        out_con    <= (cur_contrast >= 7'd90) ? 7'd30 : (cur_contrast + 7'd10);
                        out_con_en <= ~out_con_en;
                    end
                    R_RESET: begin

                        out_spec_on   <= 1'b1;
                        out_banner_on <= 1'b1;
                        out_status_on <= 1'b1;
                        out_ind_on    <= 1'b1;
                        out_vol_sel   <= 2'd3;

                        out_trans_sel <= trans_for_mode((out_play_mode == 2'd2) ? 2'd0 : out_play_mode);
                        out_music_on  <= 1'b0;

                        if (out_play_mode == 2'd2) out_play_mode <= 2'd0;
                        out_period    <= 2'd0;
                        out_bri       <= 7'd50;
                        out_bri_en    <= ~out_bri_en;
                        out_con       <= 7'd50;
                        out_con_en    <= ~out_con_en;
                    end
                    default: ;
                endcase
            end

            out_open <= open_r;
        end
    end

    reg [6:0] bri_d, con_d;
    reg       auto_d;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bri_d <= 7'd50; con_d <= 7'd50; auto_d <= 1'b0;
        end else begin
            bri_d <= cur_bright; con_d <= cur_contrast; auto_d <= cur_auto;
        end
    end

    function [3:0] dec_tens;
        input [6:0] v;
        begin
            dec_tens = (v >= 7'd90) ? 4'd9 : (v >= 7'd80) ? 4'd8 : (v >= 7'd70) ? 4'd7 :
                       (v >= 7'd60) ? 4'd6 : (v >= 7'd50) ? 4'd5 : (v >= 7'd40) ? 4'd4 :
                       (v >= 7'd30) ? 4'd3 : (v >= 7'd20) ? 4'd2 : (v >= 7'd10) ? 4'd1 : 4'd0;
        end
    endfunction
    function [3:0] dec_ones;
        input [6:0] v;
        begin

            dec_ones = v - ({dec_tens(v), 3'b000} + {dec_tens(v), 1'b0});
        end
    endfunction

    reg [7:0] menu_buf [0:511];

    `define TXT_U5B89 8'd95
    `define TXT_U8DEF 8'd96
    `define TXT_U79D1 8'd97
    `define TXT_U6280 8'd98
    `define TXT_U591A 8'd99
    `define TXT_U5A92 8'd100
    `define TXT_U4F53 8'd101
    `define TXT_U64AD 8'd102
    `define TXT_U653E 8'd103
    `define TXT_U7CFB 8'd104
    `define TXT_U7EDF 8'd105
    `define TXT_U5E27 8'd106
    `define TXT_U7F13 8'd107
    `define TXT_U5B58 8'd108
    `define TXT_U56FE 8'd109
    `define TXT_U5C42 8'd110
    `define TXT_U53E0 8'd111
    `define TXT_U52A0 8'd112
    `define TXT_U5B57 8'd113
    `define TXT_U5E55 8'd114
    `define TXT_U6548 8'd115
    `define TXT_U679C 8'd116
    `define TXT_U8F6C 8'd117
    `define TXT_U573A 8'd118
    `define TXT_U7F29 8'd119
    `define TXT_U97F3 8'd120
    `define TXT_U9891 8'd121
    `define TXT_U53EF 8'd122
    `define TXT_U89C6 8'd123
    `define TXT_U5316 8'd124
    `define TXT_U53C2 8'd125
    `define TXT_U6570 8'd126
    `define TXT_U4EAE 8'd127
    `define TXT_U5EA6 8'd128
    `define TXT_U5BF9 8'd129
    `define TXT_U6BD4 8'd130
    `define TXT_U79D2 8'd131
    `define TXT_U81EA 8'd132
    `define TXT_U52A8 8'd133
    `define TXT_U8F6E 8'd134
    `define TXT_U505C 8'd135
    `define TXT_U6B62 8'd136
    `define TXT_U673A 8'd137
    `define TXT_U53F7 8'd138
    `define TXT_U5E74 8'd139
    `define TXT_U6708 8'd140
    `define TXT_U65E5 8'd141
    `define TXT_U65F6 8'd142
    `define TXT_U4E2A 8'd143
    `define TXT_U5F20 8'd144
    `define TXT_U5F00 8'd145
    `define TXT_U5173 8'd146
    `define TXT_U8C31 8'd147
    `define TXT_U6761 8'd148
    `define TXT_U72B6 8'd149
    `define TXT_U6001 8'd150
    `define TXT_U4FE1 8'd151
    `define TXT_U606F 8'd152
    `define TXT_U884C 8'd153
    `define TXT_U6A21 8'd154
    `define TXT_U5F0F 8'd155
    `define TXT_U95F4 8'd156
    `define TXT_U9694 8'd157
    `define TXT_U65B9 8'd158
    `define TXT_U6062 8'd159
    `define TXT_U590D 8'd160
    `define TXT_U9ED8 8'd161
    `define TXT_U8BA4 8'd162
    `define TXT_U9000 8'd163
    `define TXT_U51FA 8'd164
    `define TXT_U83DC 8'd165
    `define TXT_U5355 8'd166
    `define TXT_U624B 8'd167
    `define TXT_U95ED 8'd168
    `define TXT_U6DE1 8'd169
    `define TXT_U5165 8'd170
    `define TXT_U659C 8'd171
    `define TXT_U5411 8'd172
    `define TXT_U64E6 8'd173
    `define TXT_U9664 8'd174
    `define TXT_U4E50 8'd175
    `define TXT_U4E0A 8'd176
    `define TXT_U4E0B 8'd177
    `define TXT_U9996 8'd178
    `define TXT_U4E00 8'd179
    `define TXT_U5207 8'd180
    `define TXT_U6362 8'd181
    `define TXT_U5DE6 8'd182
    `define TXT_U6ED1 8'd183
    `define TXT_U8F7D 8'd184
    `define TXT_U6307 8'd185
    `define TXT_U793A 8'd186
    `define TXT_U91CF 8'd187

    localparam [159:0] M0  = 160'h00000000000000006869A5A60000000000000000;
    localparam [159:0] M1  = 160'h00799394000000000000001A0000000000000000;
    localparam [159:0] M2  = 160'h00717294000000000000001A0000000000000000;
    localparam [159:0] M3  = 160'h00959694000000000000001A0000000000000000;
    localparam [159:0] M4  = 160'h0070B8B9BA0000000000001A0000000000000000;
    localparam [159:0] M5  = 160'h0066679A9B0000000000001A0000000000000000;
    localparam [159:0] M6  = 160'h0086669C9D0000000000001A0000000000000000;
    localparam [159:0] M7  = 160'h007F8000000000000000001A0000000000000000;
    localparam [159:0] M8  = 160'h00818280000000000000001A0000000000000000;
    localparam [159:0] M9  = 160'h0075769E9B0000000000001A0000000000000000;
    localparam [159:0] M10 = 160'h0078AF66670000000000001A0000000000000000;
    localparam [159:0] M11 = 160'h0078AFB4B50000000000001A0000000000000000;
    localparam [159:0] M12 = 160'h0078BB00000000000000001A0000000000000000;
    localparam [159:0] M13 = 160'h009FA0A1A2000000000000000000000000000000;
    localparam [159:0] M14 = 160'h00A3A4A5A6000000000000000000000000000000;
    localparam [159:0] M15 = 160'h0;

    reg        ds_open;
    reg [3:0]  ds_cursor;
    reg        ds_spec, ds_banner, ds_status, ds_ind;
    reg        ds_auto;
    reg [1:0]  ds_vol;
    reg [1:0]  ds_trans;
    reg        ds_music;
    reg [3:0]  ds_tn, ds_tt;
    reg [1:0]  ds_period;
    reg [6:0]  ds_bri, ds_con;

    function [7:0] row_char;
        input [3:0] rw;
        input [4:0] cl;
        reg [159:0] row;
        reg [7:0]   bidx;
        begin
            case (rw)
                4'd0:  row = M0;
                4'd1:  row = M1;
                4'd2:  row = M2;
                4'd3:  row = M3;
                4'd4:  row = M4;
                4'd5:  row = M5;
                4'd6:  row = M6;
                4'd7:  row = M7;
                4'd8:  row = M8;
                4'd9:  row = M9;
                4'd10: row = M10;
                4'd11: row = M11;
                4'd12: row = M12;
                4'd13: row = M13;
                4'd14: row = M14;
                default: row = M15;

            endcase
            bidx = ({4'b0, 5'd19} - {3'b0, cl}) << 3;
            row_char = row[bidx +: 8];
        end
    endfunction

    function [7:0] dyn_char;
        input [3:0] rw;
        input [4:0] cl;
        reg [7:0] g;
        reg [7:0] c;
        reg       onv;
        begin
            g = 8'h00;
            c = 8'h00;
            if (cl == 5'd0) begin
                c = ((ds_cursor == rw) && ds_open) ? ">" : " ";
            end else case (rw)

                4'd1, 4'd2, 4'd3, 4'd4: begin
                    onv = (rw == 4'd1) ? ds_spec   :
                          (rw == 4'd2) ? ds_banner :
                          (rw == 4'd3) ? ds_status : ds_ind;
                    if (cl == 5'd13) g = onv ? `TXT_U5F00 : `TXT_U5173;
                end

                4'd5: begin
                    if (cl == 5'd13)
                        g = (out_play_mode == 2'd0) ? `TXT_U624B :
                            (out_play_mode == 2'd1) ? `TXT_U81EA :
                                                      `TXT_U97F3;
                    else if (cl == 5'd14)
                        g = (out_play_mode == 2'd2) ? `TXT_U4E50 :
                                                      `TXT_U52A8;
                end

                4'd6: case (cl)
                    5'd13: c = (ds_period == 2'd3) ? "1" : (ds_period == 2'd2) ? "5" :
                                (ds_period == 2'd1) ? "2" : "1";
                    5'd14: begin
                        if (ds_period == 2'd3) c = "0";
                        else                   g = `TXT_U79D2;
                    end
                    5'd15: if (ds_period == 2'd3) g = `TXT_U79D2;
                    default: ;
                endcase

                4'd7: case (cl)
                    5'd13: c = 8'h30 + {4'b0, dec_tens(ds_bri)};
                    5'd14: c = 8'h30 + {4'b0, dec_ones(ds_bri)};
                    default: ;
                endcase
                4'd8: case (cl)
                    5'd13: c = 8'h30 + {4'b0, dec_tens(ds_con)};
                    5'd14: c = 8'h30 + {4'b0, dec_ones(ds_con)};
                    default: ;
                endcase

                4'd10: if (cl == 5'd13) g = ds_music ? `TXT_U5F00 : `TXT_U5173;

                4'd12: begin
                    if (cl == 5'd13)
                        c = (ds_vol == 2'd0) ? "2" :
                            (ds_vol == 2'd1) ? "5" :
                            (ds_vol == 2'd2) ? "7" : "1";
                    else if (cl == 5'd14)
                        c = (ds_vol == 2'd3) ? "0" : "5";
                    else if (cl == 5'd15)
                        if (ds_vol == 2'd3) c = "0";
                end

                4'd11: begin
                    if (cl == 5'd13)      c = 8'h30 + {4'b0, ds_tn};
                    else if (cl == 5'd14) c = "/";
                    else if (cl == 5'd15) c = 8'h30 + {4'b0, ds_tt};
                end

                4'd9: case (ds_trans)
                    2'd0: begin
                        if (cl == 5'd13)      g = `TXT_U5173;
                        else if (cl == 5'd14) g = `TXT_U95ED;
                    end
                    2'd1: begin
                        if (cl == 5'd13 || cl == 5'd15) g = `TXT_U6DE1;
                        else if (cl == 5'd14)          g = `TXT_U5165;
                        else if (cl == 5'd16)          g = `TXT_U51FA;
                    end
                    default: begin
                        if (cl == 5'd13)      g = `TXT_U5411;
                        else if (cl == 5'd14) g = `TXT_U5DE6;
                        else if (cl == 5'd15) g = `TXT_U6ED1;
                        else if (cl == 5'd16) g = `TXT_U52A8;
                    end
                endcase
                default: ;
            endcase

            dyn_char = (g != 8'h00) ? g : ((c == 8'h00) ? 8'h00 : (c - 8'h20));
        end
    endfunction

    function [7:0] cell_char;
        input [3:0] rw;
        input [4:0] cl;
        reg [7:0] d;
        begin
            d = dyn_char(rw, cl);

            cell_char = (d == 8'h00) ? row_char(rw, cl) : d;
        end
    endfunction

    wire [40:0] disp_now = {open_r, cursor, auto_d, out_play_mode, out_period, bri_d, con_d,
                            out_spec_on, out_banner_on, out_status_on, out_ind_on,
                            out_trans_sel, out_music_on, out_vol_sel, tn_c, tt_c};
    reg  [40:0] disp_snap;

    reg [4:0]  wr_col;
    reg [3:0]  wr_row;
    reg        wr_run;
    wire [8:0] wr_row24 = ({5'b0, wr_row} << 4) + ({6'b0, wr_row} << 3);
    wire [8:0] wr_addr = wr_row24 + {4'b0, wr_col};

    wire chg = (disp_now != disp_snap);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_row    <= 4'd0;
            wr_col    <= 5'd0;
            wr_run    <= 1'b0;
            disp_snap <= 41'd0;
            ds_open <= 1'b0; ds_cursor <= R_SPEC; ds_auto <= 1'b0; ds_period <= 2'd0;
            ds_bri <= 7'd50; ds_con <= 7'd50;
            ds_spec <= 1'b1; ds_banner <= 1'b1; ds_status <= 1'b1;
            ds_ind <= 1'b1; ds_vol <= 2'd3; ds_trans <= 2'd2; ds_music <= 1'b0;
            ds_tn <= 4'd0; ds_tt <= 4'd0;
        end else if (!wr_run) begin
            if (chg) begin

                ds_open   <= open_r;
                ds_cursor <= cursor;
                ds_auto   <= auto_d;
                ds_period <= out_period;
                ds_bri    <= bri_d;
                ds_con    <= con_d;
                ds_spec   <= out_spec_on;
                ds_banner <= out_banner_on;
                ds_status <= out_status_on;
                ds_ind    <= out_ind_on;
                ds_vol    <= out_vol_sel;
                ds_trans  <= out_trans_sel;
                ds_music  <= out_music_on;
                ds_tn     <= tn_c;
                ds_tt     <= tt_c;
                wr_row    <= 4'd0;
                wr_col    <= 5'd0;
                wr_run    <= 1'b1;
            end
        end else begin
            menu_buf[wr_addr] <= cell_char(wr_row, wr_col);
            if (wr_col == MENU_NW - 6'd1) begin
                wr_col <= 5'd0;
                if (wr_row == R_LAST) begin
                    wr_run    <= 1'b0;
                    disp_snap <= disp_now;
                end else begin
                    wr_row <= wr_row + 4'd1;
                end
            end else begin
                wr_col <= wr_col + 5'd1;
            end
        end
    end

    wire        in_box = (y >= MENU_Y0) && (y < MENU_Y1) &&
                         (col >= MENU_C0) && (col < MENU_C1);
    wire [3:0]  m_row  = y[7:4] - 4'd8;
    wire [4:0]  m_col  = col - MENU_C0[4:0];
    wire [8:0]  m_row24 = ({5'b0, m_row} << 4) + ({6'b0, m_row} << 3);
    wire [8:0]  m_idx  = m_row24 + {4'b0, m_col};

    assign menu_on     = open_r && in_box;
    assign menu_ch     = (open_r && in_box) ? menu_buf[m_idx] : 8'd0;
    assign menu_cursor = cursor;

endmodule
