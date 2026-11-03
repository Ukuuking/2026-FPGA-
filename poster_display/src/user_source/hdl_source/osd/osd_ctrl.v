// OSD 总控。汇总各图层的使能、位置与内容参数，产生统一的像素输出。

module osd_ctrl (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        sw1,
    input  wire        sw2,
    input  wire        sw3,
    input  wire        sw4,

    input  wire        m_bri_en,
    input  wire [6:0]  m_bri_val,
    input  wire        m_con_en,
    input  wire [6:0]  m_con_val,

    input  wire        menu_open,

    output reg  [6:0]  bright,
    output reg  [6:0]  contrast
);

    localparam [6:0]  PARAM_DEF  = 7'd50;
    localparam [6:0]  PARAM_STEP = 7'd10;
    localparam [6:0]  PARAM_MAX  = 7'd99;

    localparam [17:0] DEB_CYC  = 18'd262_143;

    reg [1:0] s1, s2, s3, s4;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s1 <= 2'b00; s2 <= 2'b00; s3 <= 2'b00; s4 <= 2'b00;
        end else begin
            s1 <= {s1[0], sw1};
            s2 <= {s2[0], sw2};
            s3 <= {s3[0], sw3};
            s4 <= {s4[0], sw4};
        end
    end

    reg [17:0] d1, d2, d3, d4;
    reg        l1, l2, l3, l4;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            d1 <= 18'd0; d2 <= 18'd0; d3 <= 18'd0; d4 <= 18'd0;
            l1 <= 1'b0;  l2 <= 1'b0;  l3 <= 1'b0;  l4 <= 1'b0;
        end else begin
            if (s1[1] != l1) begin
                if (d1 >= DEB_CYC) begin l1 <= s1[1]; d1 <= 18'd0; end
                else                    d1 <= d1 + 18'd1;
            end else begin
                d1 <= 18'd0;
            end

            if (s2[1] != l2) begin
                if (d2 >= DEB_CYC) begin l2 <= s2[1]; d2 <= 18'd0; end
                else                    d2 <= d2 + 18'd1;
            end else begin
                d2 <= 18'd0;
            end

            if (s3[1] != l3) begin
                if (d3 >= DEB_CYC) begin l3 <= s3[1]; d3 <= 18'd0; end
                else                    d3 <= d3 + 18'd1;
            end else begin
                d3 <= 18'd0;
            end

            if (s4[1] != l4) begin
                if (d4 >= DEB_CYC) begin l4 <= s4[1]; d4 <= 18'd0; end
                else                    d4 <= d4 + 18'd1;
            end else begin
                d4 <= 18'd0;
            end
        end
    end

    reg l1_d, l2_d, l3_d, l4_d;
    wire up1 = l1 & ~l1_d;
    wire up2 = l2 & ~l2_d;
    wire up3 = l3 & ~l3_d;
    wire up4 = l4 & ~l4_d;

    wire step1 = up1 & ~menu_open;
    wire step2 = up2 & ~menu_open;
    wire step3 = up3 & ~menu_open;
    wire step4 = up4 & ~menu_open;

    reg [2:0]  mb_s, mc_s;
    reg [6:0]  mv_bri_s1, mv_bri_s2, mv_con_s1, mv_con_s2;
    reg [1:0]  mb_cap, mc_cap;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mb_s <= 3'b000; mc_s <= 3'b000;
            mv_bri_s1 <= 7'd50; mv_bri_s2 <= 7'd50;
            mv_con_s1 <= 7'd50; mv_con_s2 <= 7'd50;
            mb_cap <= 2'b00; mc_cap <= 2'b00;
        end else begin
            mb_s <= {mb_s[1:0], m_bri_en};
            mc_s <= {mc_s[1:0], m_con_en};
            mv_bri_s1 <= m_bri_val;  mv_bri_s2 <= mv_bri_s1;
            mv_con_s1 <= m_con_val;  mv_con_s2 <= mv_con_s1;
            mb_cap <= {mb_cap[0], (mb_s[2] ^ mb_s[1])};
            mc_cap <= {mc_cap[0], (mc_s[2] ^ mc_s[1])};
        end
    end
    wire menu_set_bri = mb_cap[1];
    wire menu_set_con = mc_cap[1];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            l1_d     <= 1'b0;  l2_d <= 1'b0;  l3_d <= 1'b0;  l4_d <= 1'b0;
            bright   <= PARAM_DEF;
            contrast <= PARAM_DEF;
        end else begin
            l1_d <= l1; l2_d <= l2; l3_d <= l3; l4_d <= l4;

            if (menu_set_bri)
                bright <= (mv_bri_s2 > PARAM_MAX) ? PARAM_MAX : mv_bri_s2;
            else if (step1)
                bright <= (bright > PARAM_MAX - PARAM_STEP) ? PARAM_MAX : (bright + PARAM_STEP);
            else if (step2)
                bright <= (bright < PARAM_STEP) ? 7'd0 : (bright - PARAM_STEP);

            if (menu_set_con)
                contrast <= (mv_con_s2 > PARAM_MAX) ? PARAM_MAX : mv_con_s2;
            else if (step3)
                contrast <= (contrast > PARAM_MAX - PARAM_STEP) ? PARAM_MAX : (contrast + PARAM_STEP);
            else if (step4)
                contrast <= (contrast < PARAM_STEP) ? 7'd0 : (contrast - PARAM_STEP);
        end
    end

endmodule
