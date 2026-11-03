/*
OSD 图层合成。字幕、状态行、频谱柱、菜单各自产生使能与颜色，按优先级争用
同一个像素位置，高优先级覆盖低优先级。

字幕按像素步进滚动，滚动量取整数像素。
*/

module osd_layer (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        de,
    input  wire        vs,
    input  wire [10:0] x,
    input  wire [10:0] y,

    input  wire [4:0]  img_no,
    input  wire [4:0]  img_total,
    input  wire [6:0]  bright,
    input  wire [6:0]  contrast,
    input  wire [1:0]  play_mode,
    input  wire [7:0]  key_cnt,
    input  wire [3:0]  trk_now,
    input  wire [3:0]  trk_total,
    input  wire        load_busy,
    input  wire [9:0]  load_row,
    input  wire        ind_on,

    input  wire        menu_on,
    input  wire [7:0]  menu_ch,
    input  wire [3:0]  menu_cursor,
    input  wire        banner_on,
    input  wire        status_on,
    input  wire        spec_on,

    input  wire        bar_we,
    input  wire [5:0]  bar_addr,
    input  wire [5:0]  bar_val,

    input  wire        wipe_on,
    input  wire [10:0] wipe_thr,

    output reg  [23:0] osd_rgb,
    output reg  [2:0]  osd_a,
    output reg         osd_de
);

    localparam [10:0] Y_BANNER   = 11'd8;
    localparam [10:0] Y_STATUS   = 11'd430;
    localparam [10:0] Y_SPEC_TOP = 11'd448;
    localparam [5:0]  BAR_MAX_H  = 6'd32;
    localparam        N_BARS     = 32;
    localparam [3:0]  BAR_W      = 4'd7;  // 32 柱 7 像素，加 10 像素栏距正好 544

    reg [5:0] bar_ram [0:N_BARS-1];

    always @(posedge clk) begin
        if (bar_we && (bar_addr < N_BARS))
            bar_ram[bar_addr] <= (bar_val > BAR_MAX_H) ? BAR_MAX_H : bar_val;
    end

    localparam [6:0]  BANNER_LEN = 7'd64;

    reg        vs_d;
    reg [9:0]  scroll_pix;

    wire frame_tick = (vs_d != vs) && (vs == 1'b1);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            vs_d       <= 1'b0;
            scroll_pix <= 10'd0;
        end else begin
            vs_d <= vs;
            if (frame_tick)
                scroll_pix <= scroll_pix + 10'd1;  // 每像素推进一格，不是每次换字符
        end
    end

    function [7:0] ascii_idx;
        input [7:0] ch;
        begin
            ascii_idx = ch - 8'h20;
        end
    endfunction

    function [3:0] dec_tens;
        input [6:0] v;
        begin
            if      (v < 7'd10) dec_tens = 4'd0;
            else if (v < 7'd20) dec_tens = 4'd1;
            else if (v < 7'd30) dec_tens = 4'd2;
            else if (v < 7'd40) dec_tens = 4'd3;
            else if (v < 7'd50) dec_tens = 4'd4;
            else if (v < 7'd60) dec_tens = 4'd5;
            else if (v < 7'd70) dec_tens = 4'd6;
            else if (v < 7'd80) dec_tens = 4'd7;
            else if (v < 7'd90) dec_tens = 4'd8;
            else                dec_tens = 4'd9;
        end
    endfunction

    function [3:0] dec_ones;
        input [6:0] v;
        reg [6:0] t;
        begin
            t = {3'd0, dec_tens(v)} * 7'd10;
            dec_ones = v - t;
        end
    endfunction

    function [7:0] digit_idx;
        input [3:0] d;
        begin
            digit_idx = 7'd16 + {3'd0, d};
        end
    endfunction

    function [7:0] banner_char;
        input [5:0] pos;
        begin
            case (pos)

                6'd0:  banner_char = ascii_idx(8'h20);
                6'd1:  banner_char = ascii_idx(8'h20);
                6'd2:  banner_char = 7'd95;
                6'd3:  banner_char = 7'd96;
                6'd4:  banner_char = 7'd97;
                6'd5:  banner_char = 7'd98;
                6'd6:  banner_char = ascii_idx(8'h20);
                6'd7:  banner_char = ascii_idx("H");
                6'd8:  banner_char = ascii_idx("X");
                6'd9:  banner_char = ascii_idx("4");
                6'd10: banner_char = ascii_idx("S");
                6'd11: banner_char = ascii_idx("2");
                6'd12: banner_char = ascii_idx("0");
                6'd13: banner_char = ascii_idx("C");
                6'd14: banner_char = ascii_idx(8'h20);
                6'd15: banner_char = ascii_idx("H");
                6'd16: banner_char = ascii_idx("D");
                6'd17: banner_char = ascii_idx("M");
                6'd18: banner_char = ascii_idx("I");
                6'd19: banner_char = ascii_idx(8'h20);
                6'd20: banner_char = 7'd99;
                6'd21: banner_char = 7'd100;
                6'd22: banner_char = 7'd101;
                6'd23: banner_char = 7'd102;
                6'd24: banner_char = 7'd103;
                6'd25: banner_char = 7'd104;
                6'd26: banner_char = 7'd105;
                6'd27: banner_char = ascii_idx(8'h20);
                6'd28: banner_char = ascii_idx(8'h20);

                6'd29: banner_char = ascii_idx("A");
                6'd30: banner_char = ascii_idx("N");
                6'd31: banner_char = ascii_idx("L");
                6'd32: banner_char = ascii_idx("O");
                6'd33: banner_char = ascii_idx("G");
                6'd34: banner_char = ascii_idx("I");
                6'd35: banner_char = ascii_idx("C");
                6'd36: banner_char = ascii_idx(8'h20);
                6'd37: banner_char = ascii_idx("E");
                6'd38: banner_char = ascii_idx("G");
                6'd39: banner_char = ascii_idx("4");
                6'd40: banner_char = ascii_idx("S");
                6'd41: banner_char = ascii_idx("2");
                6'd42: banner_char = ascii_idx("0");
                6'd43: banner_char = ascii_idx(8'h20);
                6'd44: banner_char = ascii_idx(8'h20);
                6'd45: banner_char = ascii_idx("H");
                6'd46: banner_char = ascii_idx("D");
                6'd47: banner_char = ascii_idx("M");
                6'd48: banner_char = ascii_idx("I");
                6'd49: banner_char = ascii_idx(8'h20);
                6'd50: banner_char = ascii_idx("M");
                6'd51: banner_char = ascii_idx("E");
                6'd52: banner_char = ascii_idx("D");
                6'd53: banner_char = ascii_idx("I");
                6'd54: banner_char = ascii_idx("A");
                6'd55: banner_char = ascii_idx(8'h20);
                6'd56: banner_char = ascii_idx("P");
                6'd57: banner_char = ascii_idx("L");
                6'd58: banner_char = ascii_idx("A");
                6'd59: banner_char = ascii_idx("Y");
                6'd60: banner_char = ascii_idx("E");
                6'd61: banner_char = ascii_idx("R");
                6'd62: banner_char = ascii_idx(8'h20);
                6'd63: banner_char = ascii_idx(8'h20);
                default: banner_char = ascii_idx(8'h20);
            endcase
        end
    endfunction

    reg [4:0] img_no_r, img_total_r;
    reg [7:0] key_cnt_r;
    reg [3:0] trk_now_r, trk_total_r;
    reg [1:0] play_mode_r;
    reg [6:0]  bright_r, contrast_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            img_no_r    <= 5'd0;
            img_total_r <= 5'd0;
            key_cnt_r   <= 8'd0;
            trk_now_r   <= 4'd0;
            trk_total_r <= 4'd0;
            play_mode_r <= 2'd0;
            bright_r    <= 7'd0;
            contrast_r  <= 7'd0;
        end else begin
            img_no_r    <= img_no;
            img_total_r <= img_total;
            key_cnt_r   <= key_cnt;
            trk_now_r   <= trk_now;
            trk_total_r <= trk_total;
            play_mode_r <= play_mode;
            bright_r    <= bright;
            contrast_r  <= contrast;
        end
    end

    function [7:0] status_char;
        input [5:0] col;
        begin
            case (col)
                6'd0:  status_char = ascii_idx("I");
                6'd1:  status_char = ascii_idx("M");
                6'd2:  status_char = ascii_idx("G");
                6'd3:  status_char = ascii_idx(":");
                6'd4:  status_char = digit_idx(dec_tens(img_no_r));
                6'd5:  status_char = digit_idx(dec_ones(img_no_r));
                6'd6:  status_char = ascii_idx("/");
                6'd7:  status_char = digit_idx(dec_tens(img_total_r));
                6'd8:  status_char = digit_idx(dec_ones(img_total_r));
                6'd9:  status_char = ascii_idx(8'h20);
                6'd10: status_char = ascii_idx("M");
                6'd11: status_char = ascii_idx("U");
                6'd12: status_char = ascii_idx("S");
                6'd13: status_char = ascii_idx("I");
                6'd14: status_char = ascii_idx("C");
                6'd15: status_char = ascii_idx(":");
                6'd16: status_char = digit_idx(trk_now_r);
                6'd17: status_char = ascii_idx("/");
                6'd18: status_char = digit_idx(trk_total_r);
                6'd19: status_char = ascii_idx(8'h20);
                6'd20: status_char = ascii_idx("B");
                6'd21: status_char = ascii_idx("R");
                6'd22: status_char = ascii_idx("I");
                6'd23: status_char = ascii_idx(":");
                6'd24: status_char = digit_idx(dec_tens(bright_r));
                6'd25: status_char = digit_idx(dec_ones(bright_r));
                6'd26: status_char = ascii_idx(8'h20);
                6'd27: status_char = ascii_idx("C");
                6'd28: status_char = ascii_idx("O");
                6'd29: status_char = ascii_idx("N");
                6'd30: status_char = ascii_idx(":");
                6'd31: status_char = digit_idx(dec_tens(contrast_r));
                6'd32: status_char = digit_idx(dec_ones(contrast_r));
                6'd33: status_char = ascii_idx(8'h20);
                6'd34: status_char = ascii_idx("M");
                6'd35: status_char = ascii_idx("O");
                6'd36: status_char = ascii_idx("D");
                6'd37: status_char = ascii_idx("E");
                6'd38: status_char = ascii_idx(":");
                6'd39: status_char = digit_idx({2'b00, play_mode_r});
                default: status_char = ascii_idx(8'h20);
            endcase
        end
    endfunction

    wire in_banner = banner_on && (y >= Y_BANNER) && (y < Y_BANNER + 11'd16);
    wire in_status = status_on && (y >= Y_STATUS) && (y < Y_STATUS + 11'd16);
    wire in_spec   = spec_on && (y >= Y_SPEC_TOP) && (y < Y_SPEC_TOP + 11'd32);

    wire in_pbar = load_busy &&
                   (x >= 11'd583) && (x <= 11'd617) &&
                   (y >= 11'd365) && (y <= 11'd399);

    wire [5:0]  char_col = x[9:4];
    wire [3:0]  sub_y    = y[3:0];

    wire [9:0]  banner_xpix = {char_col, x[3:0]} + scroll_pix;
    wire [5:0]  banner_pos  = banner_xpix[9:4];
    wire [3:0]  banner_sx   = banner_xpix[3:0];

    wire [7:0]  rom_char = menu_on   ? menu_ch                    :
                           in_banner ? banner_char(banner_pos)     :
                           in_status ? status_char(char_col)       : 8'd0;

    wire [3:0]  rom_row = menu_on   ? y[3:0]              :
                          in_banner ? (y - Y_BANNER)      :
                                      (y - Y_STATUS);

    wire [11:0] rom_addr = {rom_char, rom_row};
    wire [47:0] rom_data;

    osd_font_rom u_font (
        .clk  (clk),
        .addr (rom_addr),
        .dout (rom_data)
    );

    reg        de_d, in_banner_d, in_status_d, in_spec_d, in_pbar_d, in_menu_d;
    reg [3:0]  sub_x_d;
    reg [10:0] x_d1;
    reg [10:0] y_d;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            de_d        <= 1'b0;
            in_banner_d <= 1'b0;
            in_status_d <= 1'b0;
            in_menu_d   <= 1'b0;
            in_spec_d   <= 1'b0;
            sub_x_d     <= 4'd0;
            x_d1        <= 11'd0;
            y_d         <= 11'd0;
        end else begin
            de_d        <= de;
            in_banner_d <= in_banner;
            in_status_d <= in_status;
            in_menu_d   <= menu_on;
            in_spec_d   <= in_spec;
            in_pbar_d   <= in_pbar;
            sub_x_d     <= in_banner ? banner_sx : x[3:0];
            x_d1        <= x;
            y_d         <= y;
        end
    end

    wire [5:0] lx = x_d1[5:0] - 6'd7;
    wire [5:0] ly = y_d[6:0]  - 7'd109;

    wire d0 = ((lx == 6'd33) && (ly >= 6'd16) && (ly <= 6'd18)) ||
              ((ly == 6'd17) && (lx >= 6'd32) && (lx <= 6'd34));
    wire d1 = ((lx == 6'd28) && (ly >= 6'd27) && (ly <= 6'd29)) ||
              ((ly == 6'd28) && (lx >= 6'd27) && (lx <= 6'd29));
    wire d2 = ((lx == 6'd17) && (ly >= 6'd32) && (ly <= 6'd34)) ||
              ((ly == 6'd33) && (lx >= 6'd16) && (lx <= 6'd18));
    wire d3 = ((lx == 6'd6)  && (ly >= 6'd27) && (ly <= 6'd29)) ||
              ((ly == 6'd28) && (lx >= 6'd5)  && (lx <= 6'd7));
    wire d4 = ((lx == 6'd1)  && (ly >= 6'd16) && (ly <= 6'd18)) ||
              ((ly == 6'd17) && (lx >= 6'd0)  && (lx <= 6'd2));
    wire d5 = ((lx == 6'd6)  && (ly >= 6'd5)  && (ly <= 6'd7)) ||
              ((ly == 6'd6)  && (lx >= 6'd5)  && (lx <= 6'd7));
    wire d6 = ((lx == 6'd17) && (ly >= 6'd0)  && (ly <= 6'd2)) ||
              ((ly == 6'd1)  && (lx >= 6'd16) && (lx <= 6'd18));
    wire d7 = ((lx == 6'd28) && (ly >= 6'd5)  && (ly <= 6'd7)) ||
              ((ly == 6'd6)  && (lx >= 6'd27) && (lx <= 6'd29));

    wire [2:0] dot_cur  = load_row[8:6];
    wire       dots_any = d0 | d1 | d2 | d3 | d4 | d5 | d6 | d7;

    wire dot_hit_cur = (dot_cur == 3'd0 && d0) || (dot_cur == 3'd1 && d1) ||
                       (dot_cur == 3'd2 && d2) || (dot_cur == 3'd3 && d3) ||
                       (dot_cur == 3'd4 && d4) || (dot_cur == 3'd5 && d5) ||
                       (dot_cur == 3'd6 && d6) || (dot_cur == 3'd7 && d7);

    wire dot_on = ind_on && in_pbar_d && dots_any;

    wire [5:0] aa_sh = 6'd45 - {sub_x_d, 1'b0} - {2'b0, sub_x_d};
    wire [2:0] text_aa = (in_banner_d || in_status_d || in_menu_d)
                       ? rom_data[aa_sh +: 3] : 3'd0;
    wire       text_on = (text_aa != 3'd0);

    wire        wipe_cover = wipe_on && de_d && ({1'b0, x_d1} >= {1'b0, wipe_thr});

    wire [11:0] wipe_delta = {1'b0, x_d1} - {1'b0, wipe_thr};
    wire        wipe_edge  = wipe_cover && (wipe_delta < 12'd2);

    wire [8:0]  bar_n    = x_d1[9:1];
    wire [15:0] bar_mn   = bar_n * 8'd205;
    wire [5:0]  bar_cell = bar_mn[15:10];
    wire [4:0]  bar_idx  = bar_cell[5:1];
    wire        bar_odd  = bar_cell[0];
    wire [9:0]  bar_bs   = bar_cell * 4'd10;
    wire [3:0]  bar_off  = x_d1[9:0] - bar_bs;
    wire [4:0]  bar_idx1 = (bar_idx == 5'd31) ? 5'd31 : (bar_idx + 5'd1);
    wire [5:0]  bar_h0   = bar_ram[bar_idx];
    wire [5:0]  bar_h1   = bar_ram[bar_idx1];

    wire [6:0]  bar_hsum = {1'b0, bar_h0} + {1'b0, bar_h1};
    wire [5:0]  bar_h    = bar_odd ? bar_hsum[6:1] : bar_h0;
    wire        bar_col  = (bar_off < BAR_W);

    wire [5:0]  bar_h_dr = (bar_h == 6'd0) ? 6'd1 : bar_h;

    wire [4:0]  spec_off = y_d - Y_SPEC_TOP;
    wire [5:0]  bar_rel  = BAR_MAX_H - {1'b0, spec_off};

    wire bar_on     = in_spec_d && bar_col && (bar_rel <= bar_h_dr);
    wire bar_top    = in_spec_d && bar_col && (bar_rel == bar_h_dr + 6'd1);

    function [23:0] bar_color;
        input [5:0] idx;
        begin
            case (idx)
                6'd0:  bar_color = 24'hF04242;
                6'd1:  bar_color = 24'hF05542;
                6'd2:  bar_color = 24'hF07C42;
                6'd3:  bar_color = 24'hF08F42;
                6'd4:  bar_color = 24'hF0A242;
                6'd5:  bar_color = 24'hF0B642;
                6'd6:  bar_color = 24'hF0DC42;
                6'd7:  bar_color = 24'hECF042;
                6'd8:  bar_color = 24'hD9F042;
                6'd9:  bar_color = 24'hC5F042;
                6'd10: bar_color = 24'h9EF042;
                6'd11: bar_color = 24'h8BF042;
                6'd12: bar_color = 24'h78F042;
                6'd13: bar_color = 24'h64F042;
                6'd14: bar_color = 24'h42F04E;
                6'd15: bar_color = 24'h42F062;
                6'd16: bar_color = 24'h42F075;
                6'd17: bar_color = 24'h42F089;
                6'd18: bar_color = 24'h42F0AF;
                6'd19: bar_color = 24'h42F0C3;
                6'd20: bar_color = 24'h42F0D6;
                6'd21: bar_color = 24'h42F0E9;
                6'd22: bar_color = 24'h42D9F0;
                6'd23: bar_color = 24'h42C5F0;
                6'd24: bar_color = 24'h42B2F0;
                6'd25: bar_color = 24'h429EF0;
                6'd26: bar_color = 24'h4278F0;
                6'd27: bar_color = 24'h4264F0;
                6'd28: bar_color = 24'h4F51F0;
                6'd29: bar_color = 24'h6242F0;
                6'd30: bar_color = 24'h8942F0;
                default: bar_color = 24'h9C42F0;
            endcase
        end
    endfunction

    wire [23:0] spec_color = bar_color(bar_idx);

    wire [3:0]  menu_row_dt = y_d[7:4] - 4'd8;
    wire        menu_cur_d  = (menu_row_dt == menu_cursor);

    localparam [3:0] MENU_BAND_ROW = 4'd14;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            osd_rgb <= 24'd0;
            osd_a   <= 3'd0;
            osd_de  <= 1'b0;
        end else begin
            osd_de <= de_d;

            if (wipe_edge && !in_menu_d) begin

                osd_rgb <= 24'h7AEDFA;
                osd_a   <= 3'd4;
            end else if (in_menu_d) begin

                if ((menu_row_dt == 4'd0) || (menu_row_dt == MENU_BAND_ROW)) begin
                    osd_rgb <= text_on ? 24'hFFFFFF : 24'h34343F;
                    osd_a   <= text_on ? text_aa : 3'd4;
                end else if (menu_cur_d) begin
                    osd_rgb <= text_on ? 24'h1A1A1A : 24'hE8B04B;
                    osd_a   <= text_on ? text_aa : 3'd4;
                end else begin
                    osd_rgb <= text_on ? 24'hDCDCE6 : 24'h15151A;
                    osd_a   <= text_on ? text_aa : 3'd4;
                end
            end else if (in_banner_d) begin

                if (text_on) begin
                    osd_rgb <= 24'hFFFFFF;
                    osd_a   <= text_aa;
                end else begin
                    osd_rgb <= 24'h000000;
                    osd_a   <= 3'd2;
                end
            end else if (in_status_d) begin
                if (text_on) begin
                    osd_rgb <= 24'h00FF00;
                    osd_a   <= text_aa;
                end else begin
                    osd_rgb <= 24'h000000;
                    osd_a   <= 3'd2;
                end
            end else if (dot_on) begin

                if (dot_hit_cur) begin
                    osd_a <= 3'd3;
                end else begin
                    osd_a <= 3'd1;
                end
                osd_rgb <= 24'h00FF00;
            end else if (in_spec_d) begin

                if (bar_on) begin
                    osd_rgb <= spec_color;
                    osd_a   <= 3'd3;
                end else if (bar_top) begin
                    osd_rgb <= 24'hFFFFFF;
                    osd_a   <= 3'd4;
                end else begin
                    osd_rgb <= 24'h000000;
                    osd_a   <= 3'd2;
                end
            end else begin
                osd_rgb <= 24'd0;
                osd_a   <= 3'd0;
            end
        end
    end

endmodule
