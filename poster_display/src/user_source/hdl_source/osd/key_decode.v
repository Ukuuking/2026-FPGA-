// 按键与拨码解码。区分电平、上升沿与下降沿，按菜单开关状态分派按键语义；长按用于取消。

module key_decode (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        sw1,
    input  wire        sw2,
    input  wire        sw3,
    input  wire        sw4,
    input  wire        key2,
    input  wire        key3,
    input  wire        key4,

    output wire        sw4_lvl,
    output wire        sw4_up,
    output wire        sw4_dn,

    output wire        nav_up,
    output wire        nav_down,
    output wire        nav_ok,
    output wire        nav_cancel,

    output wire        hot_status,
    output wire        hot_banner,
    output wire        hot_spec,
    output wire        hot_status_dn,
    output wire        hot_banner_dn,
    output wire        hot_spec_dn,

    input  wire        menu_open,
    output reg         next_tgl,
    output reg         prev_tgl,
    output reg         auto_tgl
);

    reg [1:0] s1, s2, s3, s4, k2, k3, k4;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s1 <= 2'b00; s2 <= 2'b00; s3 <= 2'b00; s4 <= 2'b00;
            k2 <= 2'b11; k3 <= 2'b11; k4 <= 2'b11;
        end else begin
            s1 <= {s1[0], sw1};
            s2 <= {s2[0], sw2};
            s3 <= {s3[0], sw3};
            s4 <= {s4[0], sw4};
            k2 <= {k2[0], key2};
            k3 <= {k3[0], key3};
            k4 <= {k4[0], key4};
        end
    end

    localparam [19:0] DEB = 20'd375_000;

    reg [19:0] c1, c2, c3, c4, e2, e3, e4;
    reg        l1, l2, l3, l4;
    reg        m2, m3, m4;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            c1 <= 20'd0; c2 <= 20'd0; c3 <= 20'd0; c4 <= 20'd0;
            e2 <= 20'd0; e3 <= 20'd0; e4 <= 20'd0;
            l1 <= 1'b0;  l2 <= 1'b0;  l3 <= 1'b0;  l4 <= 1'b0;
            m2 <= 1'b1;  m3 <= 1'b1;  m4 <= 1'b1;
        end else begin
            if (s1[1] != l1) begin if (c1 == DEB) l1 <= s1[1]; else c1 <= c1 + 20'd1; end else c1 <= 20'd0;
            if (s2[1] != l2) begin if (c2 == DEB) l2 <= s2[1]; else c2 <= c2 + 20'd1; end else c2 <= 20'd0;
            if (s3[1] != l3) begin if (c3 == DEB) l3 <= s3[1]; else c3 <= c3 + 20'd1; end else c3 <= 20'd0;
            if (s4[1] != l4) begin if (c4 == DEB) l4 <= s4[1]; else c4 <= c4 + 20'd1; end else c4 <= 20'd0;
            if (k2[1] != m2) begin if (e2 == DEB) m2 <= k2[1]; else e2 <= e2 + 20'd1; end else e2 <= 20'd0;
            if (k3[1] != m3) begin if (e3 == DEB) m3 <= k3[1]; else e3 <= e3 + 20'd1; end else e3 <= 20'd0;
            if (k4[1] != m4) begin if (e4 == DEB) m4 <= k4[1]; else e4 <= e4 + 20'd1; end else e4 <= 20'd0;
        end
    end

    reg l1_d, l2_d, l3_d, l4_d, m2_d, m3_d, m4_d;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            l1_d <= 1'b0; l2_d <= 1'b0; l3_d <= 1'b0; l4_d <= 1'b0;
            m2_d <= 1'b1; m3_d <= 1'b1; m4_d <= 1'b1;
        end else begin
            l1_d <= l1; l2_d <= l2; l3_d <= l3; l4_d <= l4;
            m2_d <= m2; m3_d <= m3; m4_d <= m4;
        end
    end

    wire k2_press = ~m2 & m2_d;
    wire k3_press = ~m3 & m3_d;
    wire k4_rel   = m4 & ~m4_d;

    assign sw4_lvl    = l4;
    assign sw4_up     = l4 & ~l4_d;
    assign sw4_dn     = ~l4 & l4_d;

    assign nav_up     = k2_press;
    assign nav_down   = k3_press;
    assign hot_status = l1 & ~l1_d;
    assign hot_banner = l2 & ~l2_d;
    assign hot_spec   = l3 & ~l3_d;
    assign hot_status_dn = ~l1 & l1_d;
    assign hot_banner_dn = ~l2 & l2_d;
    assign hot_spec_dn   = ~l3 & l3_d;
    localparam [23:0] LONG_LIM = 24'd12_500_000;

    reg [23:0] h4;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)             h4 <= 24'd0;
        else if (m4)            h4 <= 24'd0;
        else if (h4 < LONG_LIM) h4 <= h4 + 24'd1;
    end
    wire k4_long = ~m4 & (h4 == LONG_LIM);

    reg k4_long_used;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)          k4_long_used <= 1'b0;
        else if (m4)         k4_long_used <= 1'b0;
        else if (k4_long)    k4_long_used <= 1'b1;
    end
    wire k4_short = k4_rel & ~k4_long_used;

    assign nav_ok     = k4_short;

    assign nav_cancel = 1'b0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            next_tgl <= 1'b0;
            prev_tgl <= 1'b0;
            auto_tgl <= 1'b0;
        end else begin
            if (!menu_open && k2_press)   next_tgl <= ~next_tgl;
            if (!menu_open && k4_short)   prev_tgl <= ~prev_tgl;
            if (!menu_open && k3_press)   auto_tgl <= ~auto_tgl;
        end
    end

endmodule
