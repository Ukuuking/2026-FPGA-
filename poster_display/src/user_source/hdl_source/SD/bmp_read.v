/*
TF 卡 BMP 扫描、解析与像素读取。

扫描按簇步进，每簇只探第一个扇区 (BMP 起始扇区必在簇边界)，记下起始扇区；
加载时读文件头取几何参数，再逐行读出像素，等比缩放居中后写入帧缓存。

可识别条件："BM" 魔数，24 bit，compression=0，宽 8~10922，高 1~8192，
pixel_offset >= 54 且文件长度大于 pixel_offset，起始簇在 FAT 中已分配。

宽高上限的来源：行内偏移计数器 rel_byte 是 16 位有符号数，要求 3*W <= 32767，
所以 W <= 10922。

两个方向缩放算法不同：垂直缩小走面积平均，水平缩小走点采样，放大时两个方向
都走双线性。系数用 Q16 定点，s = min(640/源宽，480/源高)，尺寸向下取整后居中，
图外补黑。
*/

module bmp_read(
    input                       clk,
    input                       rst,
    output                      ready,

    input                       scan_start,
    input  [31:0]               scan_start_sector,
    input  [31:0]               scan_max_sector,
    input  [5:0]                scan_target_count,

    input                       scan_pause,
    input                       scan_resume,
    output reg                  scan_done,
    output reg                  scan_found_valid,
    output reg [31:0]           scan_found_sector,
    output reg [5:0]            scan_found_total,

    input                       load_start,
    input  [31:0]               load_sector,

    input                       sd_init_done,
    output reg [4:0]            state_code,
    input  [15:0]               bmp_width,
    input  [15:0]               bmp_height,

    output [23:0]               dbg_geom,

    output [15:0]               o_osd_src_w,
    output [15:0]               o_osd_src_h,
    output [10:0]               o_osd_disp_w,
    output [10:0]               o_osd_disp_h,

    output reg                  write_req,
    input                       write_req_ack,

    output reg                  sd_sec_read,
    output reg [31:0]           sd_sec_read_addr,
    input  [7:0]                sd_sec_read_data,
    input                       sd_sec_read_data_valid,
    input                       sd_sec_read_end,

    output reg                  bmp_data_wr_en,
    output reg [23:0]           bmp_data,

    input                       write_full,

    input                       stall_abort,
    output reg                  load_fail,
    output                      row_timeout
);

localparam ST_IDLE      = 5'd0;
localparam ST_SCAN      = 5'd1;
localparam ST_LOAD_HDR  = 5'd2;
localparam ST_ACK       = 5'd3;
localparam ST_GEOM      = 5'd4;
localparam ST_ROW_PREP  = 5'd5;
localparam ST_ROW_BLACK = 5'd6;
localparam ST_ROW_LEAD  = 5'd7;
localparam ST_ROW_BODY  = 5'd8;
localparam ST_ROW_TAIL  = 5'd9;
localparam ST_ROW_RANGE = 5'd10;
localparam ST_ROW_NEXT  = 5'd11;
localparam ST_ROW_BASE2 = 5'd13;
localparam ST_CHK_FAT   = 5'd14;
localparam ST_CHK_CAP   = 5'd15;
localparam ST_ROW_DIV   = 5'd16;
localparam ST_ROW_CLR   = 5'd17;
localparam ST_ROW_EMIT  = 5'd18;
localparam ST_HDIV      = 5'd19;
localparam ST_RESAMP    = 5'd20;

/*
行级超时 200ms @100MHz。SD 卡偶尔有几十毫秒的读取延迟尖峰，门槛设短会把
慢行当成死行，输出半条累加结果；真死锁由上层心跳看门狗中止整帧。
*/
localparam [24:0] ROW_TO_LIMIT = 25'd19_999_999;
reg [24:0] row_to_cnt;
reg        row_to;
localparam ST_ROW_BASE  = 5'd12;

localparam [15:0] MIN_SRC_W = 16'd8;
localparam [15:0] MAX_SRC_W = 16'd10922;
localparam [15:0] MAX_SRC_H = 16'd8192;

reg [4:0]  state;
reg [9:0]  rd_cnt;

reg [7:0]  header_0;
reg [7:0]  header_1;
reg [31:0] file_len;
reg [31:0] pixel_offset;
reg [31:0] width;
reg [31:0] height;
reg [15:0] bit_count;
reg [31:0] compression;

reg [31:0] scan_sector;

/*
提前一拍算好 scan_sector+1。若在地址寄存器赋值处直接做 32 位加法，进位链会
串到 sd_card_clk 的关键路径上。
*/
reg [31:0] scan_sector_p1;
reg [31:0] load_sector_latched;

reg        fs_checked;
reg        fs_ok;
reg        fs_pass;
reg [31:0] fs_fat_lba;
reg [31:0] fs_data_lba;
reg [3:0]  fs_spc_log2;

reg [9:0]  fs_cnt;
reg [31:0] fs_win;
reg [15:0] fs_bps, fs_rsvd, fs_rootent, fs_fatsz16;
reg [7:0]  fs_spc, fs_nfat;
reg [31:0] fs_fatsz32, fs_mbr_lba;
reg [15:0] fs_mbr_lo;
reg [15:0] fs_fatsz_lo;

reg        cand_ok;
reg [9:0]  chk_cnt;
reg [2:0]  chk_got;
reg [31:0] chk_entry;
reg [31:0] chk_clu;
reg [8:0]  chk_byte_ofs;
reg        chk_is_probe;

reg        fs_settle;
reg [31:0] fs_prod;
reg        chk_ph;
reg [31:0] chk_diff;

function [3:0] log2_u8;
    input [7:0] v;
    begin
        log2_u8 = v[7] ? 4'd7 : v[6] ? 4'd6 : v[5] ? 4'd5 : v[4] ? 4'd4 :
                  v[3] ? 4'd3 : v[2] ? 4'd2 : v[1] ? 4'd1 : 4'd0;
    end
endfunction

wire [31:0] fs_fatsz_sel   = (fs_fatsz32 != 32'd0) ? fs_fatsz32 : {16'd0, fs_fatsz16};
wire [31:0] fs_part_base   = fs_pass ? fs_mbr_lba : 32'd0;

wire [31:0] fs_prod_w      = (fs_nfat == 8'd1) ? fs_fatsz_sel :
                             (fs_nfat == 8'd2) ? {fs_fatsz_sel[30:0], 1'b0} :
                             (fs_nfat == 8'd3) ? ({fs_fatsz_sel[30:0], 1'b0} + fs_fatsz_sel) :
                             (fs_nfat == 8'd4) ? {fs_fatsz_sel[29:0], 2'b00} : 32'd0;
wire        fs_nfat_ok    = (fs_nfat >= 8'd1) && (fs_nfat <= 8'd4);

wire        fs_looks_like_boot = (fs_bps == 16'd512) &&
                                 (fs_spc != 8'd0) && ((fs_spc & (fs_spc - 8'd1)) == 8'd0) &&
                                 (fs_nfat_ok) && (fs_rsvd != 16'd0) &&
                                 (fs_rootent == 16'd0) &&
                                 (fs_fatsz_sel != 32'd0);
wire        fs_mbr_ok      = (!fs_pass) && (fs_mbr_lba != 32'd0) &&
                             (fs_mbr_lba < 32'd67_108_864);

wire        chk_entry_alloc = (chk_entry[31:8] != 24'd0) || (chk_entry[3:0] != 4'd0);

wire [31:0] chk_fat_sec    = fs_fat_lba + ((chk_clu * 32'd4) >> 9);
wire [31:0] chk_addr       = chk_is_probe ? (fs_pass ? fs_mbr_lba : 32'd0)
                                          : chk_fat_sec;

reg probe_boot_ok_r;
reg probe_mbr_ok_r;
always @(posedge clk or posedge rst) begin
    if (rst) begin
        probe_boot_ok_r <= 1'b0;
        probe_mbr_ok_r  <= 1'b0;
    end else if (sd_sec_read_data_valid && chk_is_probe && (fs_cnt >= 10'd42)) begin
        probe_boot_ok_r <= fs_looks_like_boot;
        probe_mbr_ok_r  <= fs_mbr_ok;
    end
end

wire [15:0] hdr_w_comb = width[15:0];
wire [31:0] hdr_h_abs  = height[31] ? (~height + 32'd1) : height;
wire [15:0] hdr_h_comb = hdr_h_abs[15:0];
wire        bottom_up_comb = (height[31] == 1'b0) && (height != 32'd0);

reg [15:0] hdr_w_r;
reg [15:0] hdr_h_r;
reg        bottom_up_r;
reg [31:0] file_sec_cnt_r;
reg        hdr_size_ok;
reg        hdr_len_ok;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        hdr_w_r        <= 16'd0;
        hdr_h_r        <= 16'd0;
        bottom_up_r    <= 1'b1;
        file_sec_cnt_r <= 32'd1;
        hdr_size_ok    <= 1'b0;
        hdr_len_ok     <= 1'b0;
    end else begin
        hdr_w_r        <= hdr_w_comb;
        hdr_h_r        <= hdr_h_comb;
        bottom_up_r    <= bottom_up_comb;
        file_sec_cnt_r <= (file_len == 32'd0) ? 32'd1 : ((file_len + 32'd511) >> 9);
        hdr_size_ok    <= (hdr_w_comb >= MIN_SRC_W) && (hdr_w_comb <= MAX_SRC_W) &&
                          (hdr_h_comb >= 16'd1)     && (hdr_h_comb <= MAX_SRC_H);
        hdr_len_ok     <= (pixel_offset >= 32'd54) && (file_len > pixel_offset);
    end
end

wire [15:0] hdr_w     = hdr_w_r;
wire [15:0] hdr_h     = hdr_h_r;
wire        bottom_up = bottom_up_r;

wire [31:0] stride_w    = ((hdr_w * 32'd3) + 32'd3) & 32'hFFFF_FFFC;
wire [31:0] row_bytes_w = hdr_w * 32'd3;

reg [15:0] stride_r;
reg [15:0] row_bytes_r;

wire header_match = (header_0 == "B") &&
                    (header_1 == "M") &&
                    (bit_count   == 16'd24) &&
                    (compression == 32'd0) &&
                    hdr_size_ok &&
                    hdr_len_ok;

reg hdr_match_r;
always @(posedge clk or posedge rst) begin
    if (rst)
        hdr_match_r <= 1'b0;
    else if (sd_sec_read_end)
        hdr_match_r <= 1'b0;
    else if (sd_sec_read_data_valid && (rd_cnt >= 10'd40) &&
             ((state == ST_SCAN) || (state == ST_LOAD_HDR)))
        hdr_match_r <= header_match;
end

reg scan_past_data;

wire [31:0] file_sector_count = file_sec_cnt_r;

wire [31:0] fs_spc_w      = 32'd1 << fs_spc_log2;
wire [31:0] scan_step      = (fs_checked && fs_ok) ? fs_spc_w : 32'd1;
wire [31:0] next_scan_sector = scan_sector + scan_step;

assign ready = (state == ST_IDLE);

always @(posedge clk or posedge rst) begin
    if (rst)
        rd_cnt <= 10'd0;
    else if (sd_sec_read_data_valid)
        rd_cnt <= rd_cnt + 10'd1;
    else if (sd_sec_read_end)
        rd_cnt <= 10'd0;
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        header_0     <= 8'd0;
        header_1     <= 8'd0;
        file_len     <= 32'd0;
        pixel_offset <= 32'd54;
        width        <= 32'd0;
        height       <= 32'd0;
        bit_count    <= 16'd0;
        compression  <= 32'd0;
    end else if (((state == ST_SCAN) || (state == ST_LOAD_HDR)) && sd_sec_read_data_valid) begin
        case (rd_cnt)
            10'd0  : header_0 <= sd_sec_read_data;
            10'd1  : header_1 <= sd_sec_read_data;

            10'd2  : file_len[7:0]    <= sd_sec_read_data;
            10'd3  : file_len[15:8]   <= sd_sec_read_data;
            10'd4  : file_len[23:16]  <= sd_sec_read_data;
            10'd5  : file_len[31:24]  <= sd_sec_read_data;

            10'd10 : pixel_offset[7:0]   <= sd_sec_read_data;
            10'd11 : pixel_offset[15:8]  <= sd_sec_read_data;
            10'd12 : pixel_offset[23:16] <= sd_sec_read_data;
            10'd13 : pixel_offset[31:24] <= sd_sec_read_data;

            10'd18 : width[7:0]    <= sd_sec_read_data;
            10'd19 : width[15:8]   <= sd_sec_read_data;
            10'd20 : width[23:16]  <= sd_sec_read_data;
            10'd21 : width[31:24]  <= sd_sec_read_data;

            10'd22 : height[7:0]   <= sd_sec_read_data;
            10'd23 : height[15:8]  <= sd_sec_read_data;
            10'd24 : height[23:16] <= sd_sec_read_data;
            10'd25 : height[31:24] <= sd_sec_read_data;

            10'd28 : bit_count[7:0]  <= sd_sec_read_data;
            10'd29 : bit_count[15:8] <= sd_sec_read_data;

            10'd30 : compression[7:0]   <= sd_sec_read_data;
            10'd31 : compression[15:8]  <= sd_sec_read_data;
            10'd32 : compression[23:16] <= sd_sec_read_data;
            10'd33 : compression[31:24] <= sd_sec_read_data;
            default: ;
        endcase
    end
end

reg [10:0] disp_w, disp_h;
reg [9:0]  off_x,  off_y;

wire [10:0] tgt_w = bmp_width[10:0];
wire [10:0] tgt_h = bmp_height[10:0];

reg [31:0] geom_dir_r;
reg [31:0] geom_opp_r;
reg [31:0] geom_cand_r;
reg [2:0]  geom_ph;
reg        geom_conv_r;
reg        narrow;

reg [9:0]  r_row;

assign dbg_geom = {1'b0, r_row, 8'd0, state_code};

assign o_osd_src_w  = hdr_w_r;
assign o_osd_src_h  = hdr_h_r;
assign o_osd_disp_w = disp_w;
assign o_osd_disp_h = disp_h;
reg [31:0] vrem;
reg [15:0] f_map;

reg [31:0] row_base;
reg [31:0] row_prod;
reg [15:0] row_f;
wire [31:0] row_base_nxt = pixel_offset + row_prod;

reg  [10:0] r_in_start_r;
reg  [10:0] r_in_end_r;
wire        row_inside = (r_row >= r_in_start_r) && (r_row < r_in_end_r);

reg [10:0] x_col;
reg [15:0] c_stream;
reg [15:0] c_hold;
reg        c_valid;
reg [15:0] c_req;
reg [31:0] hrem;
reg        inc_pend;
reg [1:0]  pix_phase;
reg [23:0] pix_acc;
reg [23:0] pix_hold;
reg        sec_pend;

wire [23:0] pix_merge = {sd_sec_read_data, pix_acc[15:0]};

reg [74:0] acc_ram [0:639];
reg [74:0] acc_rd;
reg [9:0]  acc_addr;
reg        acc_we;
reg [9:0]  acc_wa;
reg [74:0] acc_wd;

reg [23:0] line_buf [0:1023];
reg        lb_we;
reg [9:0]  lb_wa;
reg [23:0] lb_wd;
reg [9:0]  lb_ra;
reg [23:0] lb_rd;
reg [23:0] rs_pl, rs_pr;
reg [2:0]  rs_ph;
reg        rs_wrap;

reg [2:0]  rmw_st;
reg        rmw_go;
reg [23:0] apix;
reg [9:0]  acol;
reg [24:0] prod_r, prod_g, prod_b;

wire       rmw_idle = (rmw_st == 3'd0) && !rmw_go;

reg [15:0] v_lo, v_hi;
reg [15:0] rows_left;
reg [15:0] f_row;
reg [15:0] cnt_v;
reg [16:0] r16;

reg  [15:0] v_frac;
reg  [15:0] w_num;
reg  [16:0] w16;
wire        v_bi = (hdr_h < {5'b0, disp_h});

wire [15:0] div_den = v_bi ? {5'b0, disp_h} : cnt_v;

reg  [16:0] r16h;
reg  [23:0] pix_prev;
reg  [15:0] p_idx;
reg  [23:0] tpix;
reg  [16:0] twgt;
reg         hph;
reg  [17:0] apw;
wire        h_bi = (hdr_w < {5'b0, disp_w});

wire [10:0] hrem_c  = hrem[10:0];
wire [16:0] wh1_nxt = hrem_c * r16h;
wire [33:0] apwb_f  = wh1_r * w16;
wire        apwb_ovf= (apwb_f[33:16] > {1'b0, w16});
reg  [16:0] wh1_r;
reg  [17:0] apw_b;
reg  [17:0] apw_a;
reg  [1:0]  wq;
reg  [31:0] hrem_d, c_req_d;

wire        h2_comb = h_bi && ((c_req + 16'd1) < hdr_w);

wire        mag_fill = H_MAG_BUF && h_bi && (state == ST_ROW_BODY);

wire        h2_hi    = ((c_req + 16'd1) < hdr_w);

localparam  H2_EN = 1'b0;

localparam  H_MAG_BUF = 1'b1;
wire        h2_use  = H2_EN && h2_comb;
reg         h2_r;

always @(posedge clk or posedge rst) begin
    if (rst)
        h2_r <= 1'b0;
    else
        h2_r <= h2_use;
end

wire        pair_ok = (p_idx == c_req);
wire        pair_ge = (p_idx >= c_req);
wire        acc_go  = hph ? 1'b1
                    : (h2_r ? ((wq == 2'd2) && pair_ge)
                            : (c_hold >= c_req));

always @(posedge clk or posedge rst) begin
    if (rst) begin
        wh1_r   <= 17'd0;
        apw_a   <= 18'd0;
        apw_b   <= 18'd0;
        wq      <= 2'd0;
        hrem_d  <= 32'd0;
        c_req_d <= 32'd0;
    end else begin
        wh1_r   <= wh1_nxt;
        apw_b   <= apwb_ovf ? {1'b0, w16} : apwb_f[33:16];
        apw_a   <= apwb_ovf ? 18'd0       : ({1'b0, w16} - apwb_f[33:16]);
        hrem_d  <= hrem;
        c_req_d <= c_req;

        if ((hrem != hrem_d) || (c_req != c_req_d))
            wq <= 2'd0;
        else if (wq != 2'd2)
            wq <= wq + 2'd1;
    end
end

reg  [4:0]  hd_i;
reg  [16:0] hd_rem, hd_quo;
wire        hd_dbit  = (hd_i == 5'd1);
wire [16:0] hd_shift = {hd_rem[15:0], hd_dbit};
wire [16:0] hd_cext  = {1'b0, disp_w};
wire        hd_ge    = (hd_shift >= hd_cext);
reg [4:0]  dv_i;
reg [16:0] dv_rem, dv_quo;
reg [9:0]  clr_i;
reg [1:0]  emit_ph;
reg [1:0]  ecnt_ph;
reg [23:0] pix_out;
reg [10:0] ecnt;

localparam [8:0] WR_GAP = 9'd96;
reg [8:0]  wr_gap;

wire       wr_ok = (wr_gap == 9'd0) && !write_full;

wire [24:0] acc_r = acc_rd[24:0];
wire [24:0] acc_g = acc_rd[49:25];
wire [24:0] acc_b = acc_rd[74:50];

wire [7:0]  norm_r = acc_r[24] ? 8'hFF : acc_r[23:16];
wire [7:0]  norm_g = acc_g[24] ? 8'hFF : acc_g[23:16];
wire [7:0]  norm_b = acc_b[24] ? 8'hFF : acc_b[23:16];

wire        dv_dbit  = (dv_i == 5'd1);
wire [16:0] dv_shift = {dv_rem[15:0], dv_dbit};
wire [16:0] dv_cext  = {1'b0, div_den};
wire        dv_ge    = (dv_shift >= dv_cext);

reg [10:0] bcnt;
wire [10:0] right_cnt = tgt_w - {1'b0, off_x} - disp_w;

wire row_to_exp = (row_to_cnt >= ROW_TO_LIMIT);
wire row_active = (state == ST_ROW_BODY) || (state == ST_ROW_TAIL);

assign row_timeout = row_to;

reg [1:0] hdr_try;

reg signed [15:0] rel_byte;
wire in_row = (rel_byte >= 16'sd0) && (rel_byte < $signed({1'b0, row_bytes_r}));

always @(posedge clk or posedge rst) begin
    if (rst)
        row_to_cnt <= 25'd0;
    else if ((state == ST_ROW_PREP) || (state == ST_ROW_BASE) || (state == ST_ROW_CLR))
        row_to_cnt <= 25'd0;
    else if (row_active && !row_to_exp)
        row_to_cnt <= row_to_cnt + 25'd1;
end

always @(posedge clk) begin

    if ((state == ST_ROW_CLR) && (clr_i != 10'd640))
        acc_ram[clr_i] <= 75'd0;
    else if (acc_we)
        acc_ram[acc_wa] <= acc_wd;

    acc_rd <= acc_ram[acc_addr];

    if (lb_we)
        line_buf[lb_wa] <= lb_wd;
    lb_rd <= line_buf[lb_ra];
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        rmw_st <= 3'd0;
        acc_we <= 1'b0;
        acc_wa <= 10'd0;
        acc_wd <= 75'd0;
        prod_r <= 25'd0;
        prod_g <= 25'd0;
        prod_b <= 25'd0;
    end else begin
        acc_we <= 1'b0;
        case (rmw_st)
            3'd0: begin
                if (rmw_go)
                    rmw_st <= 3'd1;
            end
            3'd1: begin

                prod_r <= apix[23:16] * apw[16:0];
                prod_g <= apix[15:8]  * apw[16:0];
                prod_b <= apix[7:0]   * apw[16:0];
                rmw_st <= 3'd2;
            end
            3'd2: begin
                acc_we <= 1'b1;
                acc_wa <= acol;
                acc_wd <= {acc_b + prod_b, acc_g + prod_g, acc_r + prod_r};
                rmw_st <= 3'd3;
            end
            3'd3: rmw_st <= 3'd0;
            default: rmw_st <= 3'd0;
        endcase
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        state               <= ST_IDLE;
        sd_sec_read         <= 1'b0;
        sd_sec_read_addr    <= 32'd0;
        write_req           <= 1'b0;
        scan_done           <= 1'b0;
        scan_found_valid    <= 1'b0;
        scan_found_sector   <= 32'd0;
        scan_found_total    <= 3'd0;
        scan_sector         <= 32'd0;
        load_sector_latched <= 32'd0;

        fs_checked   <= 1'b0;
        fs_ok        <= 1'b0;
        fs_pass      <= 1'b0;
        fs_fat_lba   <= 32'd0;
        fs_data_lba  <= 32'd0;
        fs_spc_log2  <= 4'd0;
        fs_cnt       <= 10'd0;
        fs_win       <= 32'd0;
        fs_bps       <= 16'd0;
        fs_rsvd      <= 16'd0;
        fs_rootent   <= 16'd0;
        fs_fatsz16   <= 16'd0;
        fs_spc       <= 8'd0;
        fs_nfat      <= 8'd0;
        fs_fatsz32   <= 32'd0;
        fs_mbr_lba   <= 32'd0;
        fs_mbr_lo    <= 16'd0;
        fs_fatsz_lo  <= 16'd0;
        cand_ok      <= 1'b0;
        chk_cnt      <= 10'd0;
        chk_got      <= 3'd0;
        chk_entry    <= 32'd0;
        chk_clu      <= 32'd0;
        chk_byte_ofs <= 9'd0;
        chk_is_probe <= 1'b0;
        fs_settle    <= 1'b0;
        fs_prod      <= 32'd0;
        chk_ph       <= 1'b0;
        chk_diff     <= 32'd0;
        scan_past_data <= 1'b0;

        disp_w    <= 11'd0;
        disp_h    <= 11'd0;
        off_x     <= 10'd0;
        off_y     <= 10'd0;
        r_in_start_r <= 11'd0;
        r_in_end_r   <= 11'd0;
        geom_ph   <= 3'd0;
        geom_dir_r  <= 32'd0;
        geom_opp_r  <= 32'd0;
        geom_cand_r <= 32'd0;
        geom_conv_r <= 1'b0;
        narrow      <= 1'b0;
        stride_r    <= 16'd0;
        row_bytes_r <= 16'd0;

        r_row     <= 10'd0;
        vrem      <= 32'd0;
        f_map     <= 16'd0;
        row_base  <= 32'd0;
        row_prod  <= 32'd0;
        row_f     <= 16'd0;
        rel_byte  <= 16'sd0;

        x_col     <= 11'd0;
        c_stream  <= 16'd0;
        c_hold    <= 16'hFFFF;
        c_valid   <= 1'b0;
        c_req     <= 16'd0;
        hrem      <= 32'd0;
        inc_pend  <= 1'b0;
        pix_phase <= 2'd0;
        pix_acc   <= 24'd0;
        pix_hold  <= 24'd0;
        sec_pend  <= 1'b0;
        bcnt      <= 11'd0;

        row_to     <= 1'b0;
        state_code <= 5'd0;

        v_lo      <= 16'd0;
        v_hi      <= 16'd0;
        rows_left <= 16'd0;
        f_row     <= 16'd0;
        cnt_v     <= 16'd1;
        r16       <= 17'd65536;
        v_frac    <= 16'd0;
        w_num     <= 16'd1;
        w16       <= 17'd65536;
        r16h      <= 17'd65536;
        pix_prev  <= 24'd0;
        p_idx     <= 16'hFFFF;
        tpix      <= 24'd0;
        twgt      <= 17'd0;
        rs_ph     <= 3'd0;
        rs_wrap   <= 1'b0;
        rs_pl     <= 24'd0;
        rs_pr     <= 24'd0;
        lb_ra     <= 10'd0;
        lb_wa     <= 10'd0;
        lb_wd     <= 24'd0;
        hph       <= 1'b0;
        apw       <= 18'd65536;
        dv_i      <= 5'd0;
        dv_rem    <= 17'd0;
        dv_quo    <= 17'd0;
        clr_i     <= 10'd0;
        emit_ph   <= 2'd0;
        ecnt_ph   <= 2'd0;
        wr_gap    <= 9'd0;
        ecnt      <= 11'd0;
        acc_addr  <= 10'd0;
        apix      <= 24'd0;
        acol      <= 10'd0;
        rmw_go    <= 1'b0;

        bmp_data_wr_en <= 1'b0;
        bmp_data       <= 24'd0;
    end else if (!sd_init_done) begin

        state               <= ST_IDLE;
        sd_sec_read         <= 1'b0;
        sd_sec_read_addr    <= 32'd0;
        write_req           <= 1'b0;
        scan_done           <= 1'b0;
        scan_found_valid    <= 1'b0;
        scan_found_sector   <= 32'd0;
        scan_found_total    <= 3'd0;
        scan_sector         <= 32'd0;
        load_sector_latched <= 32'd0;
        bmp_data_wr_en      <= 1'b0;
        load_fail           <= 1'b0;
        hdr_try             <= 2'd0;
    end else begin
        scan_found_valid <= 1'b0;
        bmp_data_wr_en   <= 1'b0;
        rmw_go           <= 1'b0;
        lb_we            <= 1'b0;
        load_fail        <= 1'b0;

        if (wr_gap != 9'd0)
            wr_gap <= wr_gap - 9'd1;

        scan_past_data <= scan_past_data | (fs_checked && (scan_sector >= fs_data_lba));

        if (stall_abort) begin
            state       <= ST_IDLE;
            sd_sec_read <= 1'b0;
            write_req   <= 1'b0;
        end else
        case (state)

        ST_IDLE: begin
            sd_sec_read <= 1'b0;
            write_req   <= 1'b0;

            if (scan_start) begin
                scan_done        <= 1'b0;
                scan_found_total <= 3'd0;
                scan_sector      <= scan_start_sector;

                fs_checked       <= 1'b0;
                fs_ok            <= 1'b0;
                fs_pass          <= 1'b0;
                fs_cnt           <= 10'd0;
                chk_cnt          <= 10'd0;
                chk_got          <= 3'd0;
                chk_entry        <= 32'd0;
                chk_is_probe     <= 1'b1;
                cand_ok          <= 1'b0;
                fs_settle        <= 1'b0;
                chk_ph           <= 1'b0;
                chk_diff         <= 32'd0;
                scan_past_data   <= 1'b0;
                sd_sec_read_addr <= 32'd0;
                state            <= ST_CHK_CAP;
            end else if (scan_resume) begin
                /*
                必须把读地址指回扫描点。加载图片时 sd_sec_read_addr 被改写成
                图片数据区，只恢复 state 会让下一个探针读到数据中间，随后
                scan_sector 前进一簇，该簇上的下一张图被永久跳过。
                */
                sd_sec_read_addr <= scan_sector;
                state            <= ST_SCAN;
            end else if (load_start) begin
                load_sector_latched <= load_sector;
                sd_sec_read_addr    <= load_sector;
                hdr_try             <= 2'd0;
                state               <= ST_LOAD_HDR;
            end
        end

        ST_SCAN: begin
            sd_sec_read <= 1'b1;

            if (sd_sec_read_end) begin

                sd_sec_read <= 1'b0;
                if (scan_pause) begin
                    state <= ST_IDLE;
                end else

                if (hdr_match_r && (cand_ok || !fs_ok || !fs_checked)) begin
                    cand_ok           <= 1'b0;
                    scan_found_valid  <= 1'b1;
                    scan_found_sector <= scan_sector;
                    scan_found_total  <= scan_found_total + 3'd1;

                    if ((scan_found_total + 3'd1 >= scan_target_count) ||
                        (next_scan_sector > scan_max_sector)) begin
                        scan_done        <= 1'b1;
                        state            <= ST_IDLE;
                        sd_sec_read_addr <= next_scan_sector;
                        scan_sector      <= next_scan_sector;
                    end else begin

                        sd_sec_read_addr <= next_scan_sector;
                        scan_sector      <= next_scan_sector;
                    end
                end else if (hdr_match_r && !cand_ok && fs_ok && fs_checked &&
                             scan_past_data) begin

                    state <= ST_CHK_FAT;
                end else begin
                    cand_ok <= 1'b0;
                    if (scan_sector >= scan_max_sector) begin
                        scan_done <= 1'b1;
                        state     <= ST_IDLE;
                    end else begin
                        sd_sec_read_addr <= next_scan_sector;
                        scan_sector      <= next_scan_sector;
                    end
                end
            end
        end

        ST_CHK_FAT: begin
            if (fs_settle) begin

                fs_data_lba      <= fs_fat_lba + fs_prod;
                fs_settle        <= 1'b0;
                scan_sector      <= fs_fat_lba + fs_prod;
                sd_sec_read_addr <= fs_fat_lba + fs_prod;
                state            <= ST_SCAN;
            end else if (!chk_ph) begin
                chk_diff <= scan_sector - fs_data_lba;
                chk_ph   <= 1'b1;
            end else begin

                chk_clu      <= (chk_diff >> fs_spc_log2) + 32'd2;
                chk_is_probe <= 1'b0;
                chk_cnt      <= 10'd0;
                chk_got      <= 3'd0;
                chk_entry    <= 32'd0;
                chk_ph       <= 1'b0;
                state        <= ST_CHK_CAP;
            end
        end

        ST_CHK_CAP: begin
            sd_sec_read      <= 1'b1;
            sd_sec_read_addr <= chk_addr;

            chk_byte_ofs     <= {chk_clu[6:0], 2'b00};

            if (sd_sec_read_data_valid) begin
                if (chk_is_probe) begin

                    fs_win <= {fs_win[23:0], sd_sec_read_data};
                    case (fs_cnt)
                        10'd12:  fs_bps     <= {sd_sec_read_data, fs_win[7:0]};
                        10'd13:  fs_spc     <= sd_sec_read_data;
                        10'd15:  fs_rsvd    <= {sd_sec_read_data, fs_win[7:0]};
                        10'd16:  fs_nfat    <= sd_sec_read_data;
                        10'd18:  fs_rootent <= {sd_sec_read_data, fs_win[7:0]};
                        10'd23:  fs_fatsz16 <= {sd_sec_read_data, fs_win[7:0]};

                        10'd37:  fs_fatsz_lo <= {sd_sec_read_data, fs_win[7:0]};
                        10'd39:  fs_fatsz32 <= {{sd_sec_read_data, fs_win[7:0]}, fs_fatsz_lo};

                        10'd455: fs_mbr_lo  <= {sd_sec_read_data, fs_win[7:0]};
                        10'd457: if (!fs_pass) fs_mbr_lba <= {{sd_sec_read_data, fs_win[7:0]}, fs_mbr_lo};
                        default: ;
                    endcase
                    fs_cnt <= fs_cnt + 10'd1;
                end else begin

                    if (chk_cnt == chk_byte_ofs) begin
                        chk_entry <= {24'd0, sd_sec_read_data};
                        chk_got   <= 3'd1;
                    end else if ((chk_got != 3'd0) && (chk_got < 3'd4)) begin
                        chk_entry <= {chk_entry[23:0], sd_sec_read_data};
                        chk_got   <= chk_got + 3'd1;
                    end
                    chk_cnt <= chk_cnt + 10'd1;
                end
            end

            if (sd_sec_read_end) begin
                sd_sec_read <= 1'b0;

                if (chk_is_probe) begin
                    fs_checked <= 1'b1;
                    if (probe_boot_ok_r) begin

                        fs_fat_lba  <= fs_part_base + {16'd0, fs_rsvd};
                        fs_prod     <= fs_prod_w;
                        fs_spc_log2 <= log2_u8(fs_spc);
                        fs_ok       <= 1'b1;
                        fs_settle   <= 1'b1;
                        state       <= ST_CHK_FAT;
                    end else if (probe_mbr_ok_r) begin

                        fs_pass  <= 1'b1;
                        fs_cnt   <= 10'd0;
                        sd_sec_read_addr <= fs_mbr_lba;
                        state    <= ST_CHK_CAP;
                    end else begin

                        fs_ok            <= 1'b0;
                        sd_sec_read_addr <= scan_start_sector;
                        state            <= ST_SCAN;
                    end
                end else begin
                    if (chk_entry_alloc) begin

                        cand_ok          <= 1'b1;
                        sd_sec_read_addr <= scan_sector;
                        state            <= ST_SCAN;
                    end else begin

                        cand_ok          <= 1'b0;
                        scan_sector      <= next_scan_sector;
                        sd_sec_read_addr <= next_scan_sector;
                        state            <= ST_SCAN;
                    end
                end
            end
        end

        ST_LOAD_HDR: begin
            sd_sec_read <= 1'b1;

            if (sd_sec_read_end) begin
                sd_sec_read <= 1'b0;

                if (hdr_match_r) begin

                    write_req      <= 1'b1;
                    stride_r       <= stride_w[15:0];
                    row_bytes_r    <= row_bytes_w[15:0];
                    row_to         <= 1'b0;
                    sd_sec_read_addr <= load_sector_latched;
                    state          <= ST_ACK;
                end else begin

                    if (hdr_try < 2'd2) begin
                        hdr_try <= hdr_try + 2'd1;
                        state   <= ST_LOAD_HDR;
                    end else begin
                        load_fail <= 1'b1;
                        state     <= ST_IDLE;
                    end
                end
            end
        end

        ST_ACK: begin
            if (write_req_ack) begin
                write_req <= 1'b0;
                geom_ph   <= 3'd0;
                state     <= ST_GEOM;
            end
        end

        ST_GEOM: begin
            case (geom_ph)
                3'd0: begin

                    geom_dir_r <= hdr_w * tgt_h;
                    geom_opp_r <= hdr_h * tgt_w;
                    geom_ph    <= 3'd1;
                end
                3'd1: begin

                    narrow  <= (geom_dir_r <= geom_opp_r);
                    disp_w  <= tgt_w;
                    disp_h  <= tgt_h;
                    geom_ph <= 3'd2;
                end
                3'd2: begin

                    if (narrow)
                        geom_cand_r <= disp_w * hdr_h;
                    else
                        geom_cand_r <= disp_h * hdr_w;
                    geom_ph <= 3'd3;
                end
                3'd3: begin

                    if (narrow)
                        geom_conv_r <= (geom_cand_r <= geom_dir_r) || (disp_w <= 11'd1);
                    else
                        geom_conv_r <= (geom_cand_r <= geom_opp_r) || (disp_h <= 11'd1);
                    geom_ph <= 3'd4;
                end
                default: begin
                    if (geom_conv_r) begin
                        off_x     <= (tgt_w - disp_w) >> 1;
                        off_y     <= (tgt_h - disp_h) >> 1;

                        r_in_start_r <= tgt_h - ((tgt_h - disp_h) >> 1) - disp_h;
                        r_in_end_r   <= tgt_h - ((tgt_h - disp_h) >> 1);
                        r_row     <= 10'd0;
                        vrem      <= 32'd0;
                        f_map     <= 16'd0;

                        state     <= h_bi ? ST_HDIV : ST_ROW_PREP;
                    end else if (narrow) begin
                        disp_w  <= disp_w - 11'd1;
                        geom_ph <= 3'd2;
                    end else begin
                        disp_h  <= disp_h - 11'd1;
                        geom_ph <= 3'd2;
                    end
                end
            endcase
        end

        ST_ROW_PREP: begin
            sd_sec_read <= 1'b0;

            if (!row_inside) begin

                bcnt  <= tgt_w;
                state <= ST_ROW_BLACK;
            end else begin

                v_frac <= vrem[15:0];
                v_lo  <= f_map;
                v_hi  <= f_map;
                vrem  <= vrem + hdr_h;
                state <= ST_ROW_RANGE;
            end
        end

        ST_ROW_RANGE: begin
            if (vrem >= disp_h) begin
                vrem <= vrem - disp_h;
                v_hi <= v_hi + 16'd1;
            end else begin

                cnt_v     <= (v_hi == v_lo) ? 16'd1 : (v_hi - v_lo);

                rows_left <= v_bi
                           ? ((v_lo + 16'd1 < hdr_h) ? 16'd1 : 16'd0)
                           : ((v_hi == v_lo) ? 16'd0 : (v_hi - v_lo - 16'd1));
                dv_i      <= 5'd0;
                state     <= ST_ROW_DIV;
            end
        end

        ST_ROW_DIV: begin
            if (dv_i == 5'd0) begin
                dv_rem <= 17'd0;
                dv_quo <= 17'd0;
                dv_i   <= 5'd1;
            end else begin
                if (dv_ge) begin
                    dv_rem <= dv_shift - dv_cext;
                    dv_quo <= {dv_quo[15:0], 1'b1};
                end else begin
                    dv_rem <= dv_shift;
                    dv_quo <= {dv_quo[15:0], 1'b0};
                end
                if (dv_i == 5'd17) begin

                    r16   <= {dv_quo[15:0], dv_ge};
                    state <= ST_ROW_CLR;
                end else begin
                    dv_i <= dv_i + 5'd1;
                end
            end
        end

        ST_HDIV: begin
            if (hd_i == 5'd0) begin
                hd_rem <= 17'd0;
                hd_quo <= 17'd0;
                hd_i   <= 5'd1;
            end else begin
                if (hd_ge) begin
                    hd_rem <= hd_shift - hd_cext;
                    hd_quo <= {hd_quo[15:0], 1'b1};
                end else begin
                    hd_rem <= hd_shift;
                    hd_quo <= {hd_quo[15:0], 1'b0};
                end
                if (hd_i == 5'd17) begin
                    r16h  <= {hd_quo[15:0], hd_ge};
                    state <= ST_ROW_PREP;
                end else begin
                    hd_i <= hd_i + 5'd1;
                end
            end
        end

        ST_ROW_CLR: begin
            if (clr_i != 10'd640) begin
                clr_i <= clr_i + 10'd1;
            end else if (rmw_idle) begin
                clr_i <= 10'd0;
                f_row <= v_lo;
                row_f <= bottom_up ? v_lo : (hdr_h - 16'd1 - v_lo);

                w_num <= v_bi
                       ? ((v_lo + 16'd1 < hdr_h) ? {5'b0, disp_h} - v_frac
                                                 : {5'b0, disp_h})
                       : 16'd1;
                state <= ST_ROW_BASE;
            end
        end

        ST_ROW_BASE: begin
            row_prod <= row_f * stride_r;

            w16      <= w_num * r16;
            state    <= ST_ROW_BASE2;
        end

        ST_ROW_BASE2: begin
            row_base <= row_base_nxt;

            rel_byte <= 16'sd0 - $signed({7'b0, row_base_nxt[8:0]});

            x_col     <= 11'd0;
            c_stream  <= 16'd0;
            c_hold    <= 16'hFFFF;
            c_valid   <= 1'b0;
            c_req     <= 16'd0;
            hrem      <= 32'd0;
                inc_pend  <= 1'b0;
            pix_phase <= 2'd0;
            pix_acc   <= 24'd0;
            sec_pend  <= 1'b0;

            p_idx     <= 16'hFFFF;
            tpix      <= 24'd0;
            twgt      <= 17'd0;
            hph       <= 1'b0;
            bcnt      <= 11'd0;
            state     <= ST_ROW_LEAD;
        end

        ST_ROW_BLACK: begin
            if (bcnt != 11'd0) begin
                if (wr_ok) begin
                    bmp_data_wr_en <= 1'b1;
                    bmp_data       <= 24'h000000;
                    bcnt           <= bcnt - 11'd1;
                    wr_gap         <= WR_GAP;
                end
            end else begin
                state <= ST_ROW_NEXT;
            end
        end

        ST_ROW_LEAD: begin
            if (bcnt != 11'd0) begin
                if (wr_ok) begin
                    bmp_data_wr_en <= 1'b1;
                    bmp_data       <= 24'h000000;
                    bcnt           <= bcnt - 11'd1;
                    wr_gap         <= WR_GAP;
                end
            end else begin

                sd_sec_read_addr <= load_sector_latched + (row_base >> 9);
                sd_sec_read      <= 1'b1;
                state            <= ST_ROW_BODY;
            end
        end

        ST_ROW_BODY: begin

            if (sd_sec_read_data_valid)
                rel_byte <= rel_byte + 16'sd1;

            if (sd_sec_read_data_valid && in_row) begin
                case (pix_phase)
                    2'd0: begin
                        pix_acc[7:0] <= sd_sec_read_data;
                        pix_phase    <= 2'd1;
                    end
                    2'd1: begin
                        pix_acc[15:8] <= sd_sec_read_data;
                        pix_phase     <= 2'd2;
                    end
                    default: begin

                        pix_prev  <= pix_hold;
                        pix_hold  <= pix_merge;

                        p_idx     <= c_stream - 16'd1;
                        pix_phase <= 2'd0;
                        c_hold    <= c_stream;
                        c_valid   <= 1'b1;
                        c_stream  <= c_stream + 16'd1;

                        if (mag_fill) begin
                            lb_we <= 1'b1;
                            lb_wa <= c_stream[9:0];
                            lb_wd <= pix_merge;
                        end
                    end
                endcase
            end

            if (sd_sec_read_end) begin
                sd_sec_read <= 1'b0;
                sec_pend    <= 1'b1;
            end else if (!sd_sec_read_data_valid && sec_pend && (x_col < disp_w)) begin
                sd_sec_read_addr <= sd_sec_read_addr + 32'd1;
                sd_sec_read      <= 1'b1;
                sec_pend         <= 1'b0;
            end else if (!sd_sec_read_data_valid && inc_pend) begin

                hrem  <= hrem - disp_w;
                c_req <= c_req + 16'd1;
                if ((hrem - disp_w) < disp_w)
                    inc_pend <= 1'b0;
            end else if (mag_fill && (c_stream >= hdr_w)) begin

                lb_ra  <= 10'd0;
                rs_ph  <= 3'd0;
                state  <= ST_RESAMP;
            end else if (!mag_fill && !sd_sec_read_data_valid && rmw_idle && acc_go &&
                         (x_col < disp_w) && c_valid) begin

                if (h2_r && pair_ok && !hph) begin

                    apix     <= pix_prev;
                    tpix     <= pix_hold;
                    twgt     <= apw_b;
                    acol     <= x_col[9:0];
                    acc_addr <= x_col[9:0];
                    apw      <= apw_a;
                    rmw_go   <= 1'b1;
                    hph      <= 1'b1;
                end else begin

                    apix     <= hph ? tpix : pix_hold;
                    acol     <= x_col[9:0];
                    acc_addr <= x_col[9:0];
                    apw      <= hph ? {1'b0, twgt} : {1'b0, w16};
                    rmw_go   <= 1'b1;
                    hph      <= 1'b0;
                    x_col    <= x_col + 11'd1;

                    hrem     <= hrem + hdr_w;
                    if ((hrem + hdr_w) >= disp_w)
                        inc_pend <= 1'b1;
                end
            end

            if ((x_col == disp_w) || row_to_exp) begin
                if (row_to_exp)
                    row_to <= 1'b1;
                bcnt  <= 11'd0;
                state <= ST_ROW_TAIL;
            end
        end

        ST_RESAMP: begin

            if (sd_sec_read_end) begin
                sd_sec_read <= 1'b0;
                sec_pend    <= 1'b1;
            end

            if (row_to_exp) begin
                row_to <= 1'b1;
                state  <= ST_ROW_TAIL;
            end else case (rs_ph)
                3'd0: begin
                    if (x_col >= disp_w) begin
                        state <= ST_ROW_TAIL;
                    end else begin
                        rs_ph <= 3'd1;
                    end
                end
                3'd1: begin
                    rs_pl <= lb_rd;
                    lb_ra <= c_req[9:0] + 10'd1;

                    rs_wrap <= ((hrem[15:0] + hdr_w) >= disp_w);
                    rs_ph <= 3'd2;
                end
                3'd2: begin
                    rs_pr    <= lb_rd;
                    apix     <= rs_pl;
                    acol     <= x_col[9:0];
                    acc_addr <= x_col[9:0];
                    apw      <= h2_hi ? apw_a : {1'b0, w16};
                    rmw_go   <= 1'b1;
                    rs_ph    <= 3'd3;
                end
                3'd3: begin

                    if (rmw_idle) begin
                        apix     <= rs_pr;
                        acol     <= x_col[9:0];
                        acc_addr <= x_col[9:0];
                        apw      <= h2_hi ? apw_b : 18'd0;
                        rmw_go   <= 1'b1;
                        rs_ph    <= 3'd4;
                    end
                end
                default: begin
                    if (rmw_idle) begin
                        x_col <= x_col + 11'd1;

                        if (rs_wrap) begin
                            hrem  <= hrem + hdr_w - disp_w;
                            c_req <= c_req + 16'd1;
                            lb_ra <= c_req[9:0] + 10'd1;
                        end else begin
                            hrem  <= hrem + hdr_w;
                            lb_ra <= c_req[9:0];
                        end
                        rs_ph <= 3'd0;
                    end
                end
            endcase
        end

        ST_ROW_TAIL: begin

            if (sd_sec_read_end)
                sd_sec_read <= 1'b0;

            if (!sd_sec_read || sd_sec_read_end || row_to_exp) begin

                if (row_to_exp)
                    row_to <= 1'b1;
                if (rows_left != 16'd0) begin

                    rows_left <= rows_left - 16'd1;
                    f_row     <= f_row + 16'd1;
                    row_f     <= bottom_up ? (f_row + 16'd1)
                                           : (hdr_h - 16'd1 - (f_row + 16'd1));

                    w_num     <= v_bi ? v_frac : 16'd1;
                    state     <= ST_ROW_BASE;
                end else begin
                    f_map   <= v_hi;
                    bcnt    <= {1'b0, off_x};
                    ecnt    <= 11'd0;
                    emit_ph <= 2'd0;
                    ecnt_ph <= 2'd0;
                    state   <= ST_ROW_EMIT;
                end
            end
        end

        ST_ROW_EMIT: begin
            case (emit_ph)
                2'd0: begin
                    if (bcnt != 11'd0) begin
                        if (wr_ok) begin
                            bmp_data_wr_en <= 1'b1;
                            bmp_data       <= 24'h000000;
                            bcnt           <= bcnt - 11'd1;
                            wr_gap         <= WR_GAP;
                        end
                    end else begin
                        emit_ph <= 2'd1;
                        ecnt_ph <= 2'd0;
                    end
                end
                2'd1: begin
                    case (ecnt_ph)
                        2'd0: begin
                            if (ecnt < {1'b0, disp_w}) begin
                                if (wr_ok) begin
                                    acc_addr <= ecnt[9:0];
                                    ecnt_ph  <= 2'd1;
                                end
                            end else begin
                                bcnt    <= right_cnt;
                                emit_ph <= 2'd2;
                            end
                        end
                        2'd1: ecnt_ph <= 2'd2;
                        2'd2: begin
                            pix_out  <= {norm_r, norm_g, norm_b};
                            ecnt_ph  <= 2'd3;
                        end
                        default: begin
                            if (wr_ok) begin
                                bmp_data_wr_en <= 1'b1;
                                bmp_data       <= pix_out;
                                ecnt           <= ecnt + 11'd1;
                                wr_gap         <= WR_GAP;
                                ecnt_ph        <= 2'd0;
                            end
                        end
                    endcase
                end
                default: begin
                    if (bcnt != 11'd0) begin
                        if (wr_ok) begin
                            bmp_data_wr_en <= 1'b1;
                            bmp_data       <= 24'h000000;
                            bcnt           <= bcnt - 11'd1;
                            wr_gap         <= WR_GAP;
                        end
                    end else begin
                        state <= ST_ROW_NEXT;
                    end
                end
            endcase
        end

        ST_ROW_NEXT: begin
            if (r_row == (tgt_h - 11'd1)) begin
                state <= ST_IDLE;
            end else begin
                r_row <= r_row + 10'd1;
                state <= ST_ROW_PREP;
            end
        end

        default: begin
            state       <= ST_IDLE;
            sd_sec_read <= 1'b0;
            write_req   <= 1'b0;
        end
        endcase

        state_code <= state;
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) scan_sector_p1 <= 32'd1;
    else     scan_sector_p1 <= scan_sector + 32'd1;
end

endmodule
