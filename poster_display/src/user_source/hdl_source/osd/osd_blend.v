// 单层 OSD 与视频混合。按 alpha 线性插值，alpha 为 0 时直通视频，为 4 时全取 OSD 色。

module osd_blend (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        de,
    input  wire [23:0] vid_rgb,
    input  wire [23:0] osd_rgb,
    input  wire [2:0]  osd_a,

    input  wire [6:0]  bright,
    input  wire [6:0]  constr,
    input  wire [8:0]  fade,

    output reg  [23:0] out_rgb,
    output reg         out_de
);

    wire signed [7:0] c_delta = $signed({1'b0, constr}) - 8'sd50;

    wire signed [8:0] b_off = ($signed({1'b0, bright}) - 9'sd50) <<< 1;

    function [7:0] blend_channel;
        input [7:0] base;
        input [7:0] ov;
        input [2:0] a;
        input signed [7:0]  cd;
        input signed [8:0]  bo;
        reg [9:0]  b3;
        reg [9:0]  o3;
        reg [9:0]  s1;
        reg [9:0]  s3;
        reg signed [12:0]   m;
        reg signed [13:0]   t;
        begin

            b3 = {2'b0, base} + {2'b0, base} + {2'b0, base};
            o3 = {2'b0, ov}   + {2'b0, ov}   + {2'b0, ov};
            s1 = b3 + {2'b0, ov};
            s3 = {2'b0, base} + o3;
            // alpha 五档：0 / 1/4 / 1/2 / 3/4 / 1，分母统一取 4
            case (a)
                3'd0:    m = $signed({5'b0, base});
                3'd1:    m = $signed({3'b0, s1}) >>> 2;
                3'd2:    m = ($signed({5'b0, base}) + $signed({5'b0, ov})) >>> 1;
                3'd3:    m = $signed({3'b0, s3}) >>> 2;
                default: m = $signed({5'b0, ov});
            endcase

            // 对比度绕中点 128 缩放：(m-128)*cd 再右移 6 位，cd 取 -50~+50
            t = m + ((($signed({m[7], m[7:0]}) - 13'sd128) * cd) >>> 6);

            t = t + bo;

            if (t < 14'sd0)
                blend_channel = 8'd0;
            else if (t > 14'sd255)
                blend_channel = 8'd255;
            else
                blend_channel = t[7:0];
        end
    endfunction

    wire [7:0] ch_r = blend_channel(vid_rgb[23:16], osd_rgb[23:16], osd_a, c_delta, b_off);
    wire [7:0] ch_g = blend_channel(vid_rgb[15:8],  osd_rgb[15:8],  osd_a, c_delta, b_off);
    wire [7:0] ch_b = blend_channel(vid_rgb[7:0],   osd_rgb[7:0],   osd_a, c_delta, b_off);

    wire [16:0] fade_r = ch_r * fade;
    wire [16:0] fade_g = ch_g * fade;
    wire [16:0] fade_b = ch_b * fade;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_rgb <= 24'd0;
            out_de  <= 1'b0;
        end else begin
            out_de  <= de;

            out_rgb <= { fade_r[15:8], fade_g[15:8], fade_b[15:8] };
        end
    end

endmodule
