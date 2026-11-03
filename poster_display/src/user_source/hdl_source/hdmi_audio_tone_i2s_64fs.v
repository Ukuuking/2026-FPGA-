// I2S 发送，位时钟为采样率的 64 倍，数据在时钟下降沿更新。

module hdmi_audio_tone_i2s_64fs #(
    parameter [31:0] PHASE_INC = 32'd39370534,
    parameter signed [23:0] AMP = 24'sd2000000,
    parameter [15:0] NOTE_HOLD_FRAMES = 16'd24000,
    parameter [15:0] NOTIFY_FRAMES    = 16'd9600
)(
    input  wire I_mclk,
    input  wire I_rst,

    input  wire [2:0] I_note_sel,
    input  wire       I_trig_tgl,
    input  wire [1:0] I_mode,
    input  wire [3:0] I_vol,

    input  wire [15:0] I_pcm,
    input  wire        I_pcm_en,
    input  wire [1:0]  I_vol_sel,
    output reg         O_pcm_rd,
    output reg  O_i2s_BCLK,
    output reg  O_i2s_LRCK,
    output reg  O_i2s_DOUT
);

function [31:0] note_inc_lut;
    input [2:0] idx;
    begin
        case (idx)
            3'd0: note_inc_lut = 32'd23409862;
            3'd1: note_inc_lut = 32'd26276681;
            3'd2: note_inc_lut = 32'd29494578;
            3'd3: note_inc_lut = 32'd31248410;
            3'd4: note_inc_lut = 32'd35075155;
            3'd5: note_inc_lut = 32'd39370534;
            3'd6: note_inc_lut = 32'd44191930;
            default: note_inc_lut = 32'd46819716;
        endcase
    end
endfunction

reg [1:0]  mode_s1, mode_s2;
reg [3:0]  vol_s1,  vol_s2;
reg [2:0]  note_s1, note_s2;
reg        trig_s1, trig_s2, trig_s3;

wire [1:0] mode      = mode_s2;
wire [3:0] vol       = vol_s2;
wire       trig_edge = trig_s2 ^ trig_s3;

localparam [9:0] ENV_MAX  = 10'd512;
localparam [9:0] ENV_STEP = 10'd2;

reg [15:0] hold_cnt;
reg [9:0]  env;

wire legacy_mode = (mode == 2'd3);
wire want_on     = (mode == 2'd2) || legacy_mode || ((mode == 2'd1) && (hold_cnt != 16'd0));
wire env_up      = want_on  && (env != ENV_MAX);
wire env_dn      = !want_on && (env != 10'd0);

reg  [5:0]  S_bit_cnt;
reg  [63:0] S_shift_reg;
reg  [31:0] S_phase_acc;
reg signed [23:0] S_sample_word;
reg signed [23:0] S_sample_next;
reg  [2:0]  S_note_idx;
reg [15:0]  S_note_frame_cnt;

wire [31:0] W_note_inc = note_inc_lut(S_note_idx);

wire signed [23:0] pcm_24 = {I_pcm, 8'b0};

reg  [1:0] volsel_s1, volsel_s2;
wire [1:0] vol_sel = volsel_s2;

wire signed [23:0] pcm_v25 = pcm_24 >>> 3;
wire signed [23:0] pcm_v50 = pcm_24 >>> 2;
wire signed [23:0] pcm_v75 = (pcm_24 >>> 2) + (pcm_24 >>> 3);
wire signed [23:0] pcm_v100= pcm_24 >>> 1;
wire signed [23:0] pcm_lvl = (vol_sel == 2'd0) ? pcm_v25  :
                             (vol_sel == 2'd1) ? pcm_v50  :
                             (vol_sel == 2'd2) ? pcm_v75  : pcm_v100;

wire signed [27:0] amp_x_vol = AMP * {{24{1'b0}}, vol};
wire signed [23:0] amp_v     = amp_x_vol[27:4];

wire signed [33:0] amp_x_env = amp_v * {{24{1'b0}}, env};
wire signed [23:0] amp_env   = amp_x_env[32:9];

always @(posedge I_mclk or posedge I_rst) begin
    if (I_rst) begin
        O_i2s_BCLK      <= 1'b0;
        O_i2s_LRCK      <= 1'b0;
        O_i2s_DOUT      <= 1'b0;
        S_bit_cnt       <= 6'd0;
        S_shift_reg     <= 64'd0;
        S_phase_acc     <= 32'd0;
        S_sample_word   <= 24'sd0;
        S_sample_next   <= 24'sd0;
        S_note_idx      <= 3'd0;
        S_note_frame_cnt<= 16'd0;
        mode_s1 <= 2'd0; mode_s2 <= 2'd0;
        vol_s1  <= 4'd0; vol_s2  <= 4'd0;
        volsel_s1 <= 2'd0; volsel_s2 <= 2'd0;
        note_s1 <= 3'd0; note_s2 <= 3'd0;
        trig_s1 <= 1'b0; trig_s2 <= 1'b0; trig_s3 <= 1'b0;
        hold_cnt <= 16'd0;
        env      <= 10'd0;
    end
    else begin

        mode_s1 <= I_mode;     mode_s2 <= mode_s1;
        vol_s1  <= I_vol;      vol_s2  <= vol_s1;
        volsel_s1 <= I_vol_sel; volsel_s2 <= volsel_s1;
        note_s1 <= I_note_sel; note_s2 <= note_s1;
        trig_s1 <= I_trig_tgl; trig_s2 <= trig_s1; trig_s3 <= trig_s2;

        O_i2s_BCLK <= ~O_i2s_BCLK;

        if (O_i2s_BCLK == 1'b1) begin
            O_i2s_DOUT  <= S_shift_reg[63];
            S_shift_reg <= {S_shift_reg[62:0], 1'b0};

            if (S_bit_cnt == 6'd63) begin
                S_bit_cnt <= 6'd0;

                if (O_i2s_LRCK == 1'b0) begin

                    O_i2s_LRCK  <= 1'b1;
                    S_shift_reg <= {S_sample_word[23:0], 40'd0};
                end
                else begin

                    O_i2s_LRCK  <= 1'b0;

                    if (env_up)      env <= env + ENV_STEP;
                    else if (env_dn) env <= env - ENV_STEP;

                    if (mode == 2'd1) begin
                        if (trig_edge)              hold_cnt <= NOTIFY_FRAMES;
                        else if (hold_cnt != 16'd0) hold_cnt <= hold_cnt - 16'd1;
                    end
                    else begin
                        hold_cnt <= 16'd0;
                    end

                    if (legacy_mode) begin

                        if (S_note_frame_cnt == NOTE_HOLD_FRAMES - 16'd1) begin
                            S_note_frame_cnt <= 16'd0;
                            S_note_idx <= (S_note_idx == 3'd7) ? 3'd0
                                                              : (S_note_idx + 3'd1);
                        end
                        else begin
                            S_note_frame_cnt <= S_note_frame_cnt + 16'd1;
                        end
                    end
                    else begin

                        S_note_frame_cnt <= 16'd0;
                        S_note_idx <= (mode == 2'd1) ? 3'd7 : note_s2;
                    end

                    if (I_pcm_en) begin

                        S_sample_next <= pcm_lvl;
                        S_phase_acc   <= 32'd0;

                        O_pcm_rd      <= 1'b1;
                    end else begin
                        O_pcm_rd <= 1'b0;

                        S_phase_acc <= S_phase_acc + W_note_inc;
                        if (S_phase_acc[31])
                            S_sample_next <= amp_env;
                        else
                            S_sample_next <= -amp_env;
                    end

                    S_sample_word <= S_sample_next;
                    S_shift_reg   <= {S_sample_next[23:0], 40'd0};
                end
            end
            else begin
                S_bit_cnt <= S_bit_cnt + 6'd1;
            end
        end
    end
end

endmodule
