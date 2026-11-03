/*
音频频谱分析。33 路一阶低通并联构成 32 个频段，每路取绝对值后做快慢两个
包络，快包络用于起音检测，快慢之差驱动整屏明暗呼吸。

band_mag 不超过 4095，左移 12 位后送对数运算，超过会让乘法器溢出。
*/

module audio_viz (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        audio_valid,
    input  wire [23:0] audio_l,
    input  wire [23:0] audio_r,

    output reg         bar_we,
    output reg  [5:0]  bar_addr,
    output reg  [5:0]  bar_val,

    output reg         beat_evt,

    output reg  [15:0] env_fast,
    output reg  [15:0] env_slow,
    output reg  [1:0]  timbre_sel
);

    localparam integer N_LP   = 33;
    localparam integer N_BAND = 32;
    localparam [5:0]   BAR_MAX = 6'd32;
    localparam integer SMP_W  = 12;

    function [7:0] lp_coef;
        input [5:0] i;
        begin
            case (i)
                6'd0:  lp_coef = 8'd2;
                6'd1:  lp_coef = 8'd3;
                6'd2:  lp_coef = 8'd4;
                6'd3:  lp_coef = 8'd5;
                6'd4:  lp_coef = 8'd6;
                6'd5:  lp_coef = 8'd7;
                6'd6:  lp_coef = 8'd8;
                6'd7:  lp_coef = 8'd9;
                6'd8:  lp_coef = 8'd10;
                6'd9:  lp_coef = 8'd11;
                6'd10: lp_coef = 8'd12;
                6'd11: lp_coef = 8'd13;
                6'd12: lp_coef = 8'd15;
                6'd13: lp_coef = 8'd18;
                6'd14: lp_coef = 8'd21;
                6'd15: lp_coef = 8'd25;
                6'd16: lp_coef = 8'd29;
                6'd17: lp_coef = 8'd34;
                6'd18: lp_coef = 8'd40;
                6'd19: lp_coef = 8'd46;
                6'd20: lp_coef = 8'd54;
                6'd21: lp_coef = 8'd63;
                6'd22: lp_coef = 8'd73;
                6'd23: lp_coef = 8'd84;
                6'd24: lp_coef = 8'd96;
                6'd25: lp_coef = 8'd109;
                6'd26: lp_coef = 8'd124;
                6'd27: lp_coef = 8'd139;
                6'd28: lp_coef = 8'd155;
                6'd29: lp_coef = 8'd171;
                6'd30: lp_coef = 8'd186;
                6'd31: lp_coef = 8'd201;
                default: lp_coef = 8'd215;
            endcase
        end
    endfunction

    reg signed [SMP_W-1:0] lp [0:N_LP-1];
    reg signed [SMP_W-1:0] x16;

    reg [5:0]  st_cnt;
    reg        phase;
    reg        run;

    wire signed [SMP_W-1:0] lp_cur = lp[st_cnt];

    wire signed [SMP_W+8:0] lp_mul = ($signed(x16) - $signed(lp_cur)) * $signed({1'b0, lp_coef(st_cnt)});

    wire signed [SMP_W-1:0] lp_a = lp[st_cnt];
    wire signed [SMP_W-1:0] lp_b = lp[(st_cnt == 6'd32) ? 6'd32 : (st_cnt + 6'd1)];
    wire signed [SMP_W:0]   band_diff = lp_a - lp_b;
    wire [SMP_W:0] band_mag = band_diff[SMP_W] ? (~band_diff + 1'b1) : band_diff;

    reg [23:0] peak_run;
    reg [23:0] loud_q;
    reg [23:0] ef_q;

    wire [23:0] band_q   = {band_mag[11:0], 12'd0};  // band_mag 限 12 位，再高会乘法溢出
    wire [23:0] peak_att = (band_q > peak_run) ? band_q : peak_run;
    wire [SMP_W:0] pk16  = peak_att[23:12];

    reg [13:0] beat_gap;

    // 起音判定：快包络足够大、距上次起音超过间隔、且峰值高出包络 1/8 以上。
    wire [23:0] beat_thr = ef_q + (ef_q >> 3);

    wire        beat_ok  = (ef_q >= 24'd32768) &&
                           (beat_gap >= 14'd8000) &&
                           (peak_att > beat_thr);

    reg [5:0]  bar_hold [0:N_BAND-1];
    reg [8:0]  hold_div;

    function [4:0] msb_pos;
        input [SMP_W:0] v;
        begin
            if      (v[12]) msb_pos = 5'd12;
            else if (v[11]) msb_pos = 5'd11;
            else if (v[10]) msb_pos = 5'd10;
            else if (v[9])  msb_pos = 5'd9;
            else if (v[8])  msb_pos = 5'd8;
            else if (v[7])  msb_pos = 5'd7;
            else if (v[6])  msb_pos = 5'd6;
            else if (v[5])  msb_pos = 5'd5;
            else if (v[4])  msb_pos = 5'd4;
            else if (v[3])  msb_pos = 5'd3;
            else if (v[2])  msb_pos = 5'd2;
            else if (v[1])  msb_pos = 5'd1;
            else            msb_pos = 5'd0;
        end
    endfunction

    wire [4:0]  peak_msb = msb_pos(norm_pk);
    wire [7:0]  peak_m8  = (peak_msb >= 5'd7) ? (norm_pk >> (peak_msb - 5'd7))
                                              : (norm_pk << (5'd7 - peak_msb));

    wire [4:0]  loud_msb = msb_pos(loud_q[23:12]);

    wire signed [5:0] oct_d   = $signed({1'b0, peak_msb}) - $signed({1'b0, loud_msb});
    wire signed [5:0] oct_raw = (loud_q < 24'd1024) ? 6'sd0 : oct_d;

    wire [2:0]  oct_idx = (oct_raw >  6'sd3) ? 3'd6 :
                          (oct_raw < -6'sd3) ? 3'd0 :
                          (oct_raw + 6'sd3);

    function [5:0] h_tab;
        input [2:0] i;
        begin
            case (i)
                3'd0:    h_tab = 6'd9;
                3'd1:    h_tab = 6'd14;
                3'd2:    h_tab = 6'd19;
                3'd3:    h_tab = 6'd25;
                3'd4:    h_tab = 6'd30;
                3'd5:    h_tab = 6'd32;
                default: h_tab = 6'd32;
            endcase
        end
    endfunction

    wire [5:0]  h_top = h_tab(oct_idx);

    reg  [5:0]  h_top_r;

    function [5:0] recip;
        input [3:0] idx;
        begin
            case (idx)
                4'd0:  recip = 6'd31;
                4'd1:  recip = 6'd29;
                4'd2:  recip = 6'd28;
                4'd3:  recip = 6'd26;
                4'd4:  recip = 6'd25;
                4'd5:  recip = 6'd24;
                4'd6:  recip = 6'd23;
                4'd7:  recip = 6'd22;
                4'd8:  recip = 6'd21;
                4'd9:  recip = 6'd20;
                4'd10: recip = 6'd19;
                4'd11: recip = 6'd19;
                4'd12: recip = 6'd18;
                4'd13: recip = 6'd17;
                4'd14: recip = 6'd17;
                default: recip = 6'd16;
            endcase
        end
    endfunction

    wire [3:0]  recip_idx = peak_m8[7:3] - 4'd16;
    wire [5:0]  rcp       = recip(recip_idx);

    reg [4:0]  norm_msb;
    reg [5:0]  norm_rcp;
    reg        norm_okr;
    reg [SMP_W:0] norm_pk;

    wire [16:0] ratio_mul = band_mag * norm_rcp;
    wire [16:0] ratio_raw = ratio_mul >> norm_msb;

    wire [16:0] ratio_max = {11'd0, BAR_MAX};
    wire [5:0]  ratio_l   = (ratio_raw > ratio_max) ? BAR_MAX : ratio_raw[5:0];

    wire [5:0]  ratio_c32 = norm_okr ? ratio_l : 6'd0;
    wire [11:0] scale_mul = ratio_c32 * h_top_r;
    wire [5:0]  ratio_c   = scale_mul[10:5];
    wire        hold_tick = (hold_div == 9'd159);

    reg [5:0]  hold_old;
    reg [5:0]  idx_reg;
    reg [5:0]  ratio_reg;
    reg        p2sub;
    reg        p2_last;

    reg  [5:0]  lo_samp;
    reg  [5:0]  hi_samp;
    reg  [14:0] timb_hold;

    wire [6:0] timb_lo2  = {1'b0, lo_samp} <<< 1;
    wire [6:0] timb_hi   = {1'b0, hi_samp};
    wire [6:0] timb_lo15 = timb_lo2 + (timb_lo2 >>> 1);
    wire [6:0] timb_hi15 = timb_hi  + (timb_hi  >>> 1);
    wire [1:0] timb_new  = (timb_lo2 > timb_hi15) ? 2'd0 :
                           (timb_hi  > timb_lo15) ? 2'd2 :
                                                    2'd1;

    wire [2:0] dec_sh2    = (idx_reg < 6'd10) ? 3'd4 :
                            (idx_reg < 6'd22) ? 3'd3 :
                                                3'd2;
    wire [5:0] hold_dec2  = (hold_old == 6'd0) ? 6'd0
                                               : ((hold_old >> dec_sh2) + 6'd1);
    wire [5:0] hold_next2 = (ratio_reg >= hold_old) ? ratio_reg
                            : (hold_tick ? hold_dec2 : hold_old);

    integer k;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (k = 0; k < N_LP; k = k + 1)
                lp[k] <= 16'sd0;
            x16      <= 16'sd0;
            st_cnt   <= 6'd0;
            phase    <= 1'b0;
            run      <= 1'b0;
            peak_run <= 24'd0;
            loud_q   <= 24'd0;
            norm_msb <= 5'd0;
            norm_rcp <= 6'd0;
            norm_okr <= 1'b0;
            norm_pk  <= 13'd0;
            h_top_r  <= 6'd0;
            bar_we   <= 1'b0;
            bar_addr <= 6'd0;
            bar_val  <= 6'd0;
            hold_div <= 9'd0;
            hold_old <= 6'd0;
            idx_reg  <= 6'd0;
            ratio_reg<= 6'd0;
            p2sub    <= 1'b0;
            p2_last  <= 1'b0;
            for (k = 0; k < N_BAND; k = k + 1)
                bar_hold[k] <= 6'd0;
            beat_evt <= 1'b0;
            beat_gap <= 14'd0;

            ef_q       <= 24'd0;
            env_fast   <= 16'd0;
            env_slow   <= 16'd0;
            lo_samp    <= 6'd0;
            hi_samp    <= 6'd0;
            timb_hold  <= 15'd0;
            timbre_sel <= 2'd1;
        end else begin
            bar_we <= 1'b0;
            beat_evt <= 1'b0;

            if (!run && audio_valid) begin

                if (beat_gap != 14'd16383) beat_gap <= beat_gap + 14'd1;

                x16    <= ({audio_l[23], audio_l[23:12]} + {audio_r[23], audio_r[23:12]}) >>> 1;
                st_cnt <= 6'd0;
                phase  <= 1'b0;
                run    <= 1'b1;
                peak_run <= 24'd0;

            end else if (run) begin
                if (!phase) begin

                    lp[st_cnt] <= lp[st_cnt] + (lp_mul >>> 8);

                    if (st_cnt == N_LP - 1) begin
                        st_cnt <= 6'd0;
                        phase  <= 1'b1;
                    end else begin
                        st_cnt <= st_cnt + 6'd1;
                    end
                end else begin

                    peak_run <= peak_att;
                    if (!p2sub) begin

                        hold_old  <= bar_hold[st_cnt];
                        idx_reg   <= st_cnt;
                        ratio_reg <= ratio_c;

                        if (st_cnt == 6'd2)  lo_samp <= ratio_c;
                        if (st_cnt == 6'd24) hi_samp <= ratio_c;
                        if (st_cnt == N_BAND - 1) begin

                            norm_msb <= peak_msb;
                            norm_rcp <= rcp;

                            norm_okr <= (norm_pk >= 13'd2);
                            norm_pk  <= pk16;
                            h_top_r  <= h_top;

                            if (loud_q < 24'd1024)
                                loud_q <= peak_att;
                            else if (peak_att > loud_q)
                                loud_q <= loud_q + ((peak_att - loud_q) >> 18);
                            else
                                loud_q <= loud_q - ((loud_q - peak_att) >> 18);
                            hold_div <= hold_tick ? 9'd0 : (hold_div + 9'd1);
                            p2_last  <= 1'b1;

                            if (beat_ok) begin
                                beat_evt <= 1'b1;
                                beat_gap <= 14'd0;
                            end

                            if (peak_att > ef_q)
                                ef_q <= ef_q + ((peak_att - ef_q) >> 12);
                            else
                                ef_q <= ef_q - ((ef_q - peak_att) >> 12);
                            env_fast <= ef_q[23:12];
                            env_slow <= loud_q[23:12];

                            if (timb_hold == 15'd0) begin
                                timbre_sel <= timb_new;
                                timb_hold  <= 15'd24000;
                            end else begin
                                timb_hold  <= timb_hold - 15'd1;
                            end
                        end
                        p2sub <= 1'b1;
                    end else begin

                        bar_we            <= 1'b1;
                        bar_addr          <= idx_reg;
                        bar_val           <= hold_next2;
                        bar_hold[idx_reg] <= hold_next2;
                        p2sub <= 1'b0;

                        if (p2_last) begin
                            p2_last <= 1'b0;
                            st_cnt  <= 6'd0;
                            phase   <= 1'b0;
                            run     <= 1'b0;
                        end else begin
                            st_cnt <= st_cnt + 6'd1;
                        end
                    end
                end
            end
        end
    end

endmodule
