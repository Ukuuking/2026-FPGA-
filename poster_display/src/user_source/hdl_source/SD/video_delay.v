// 视频行缓冲延迟，把像素推迟 DATA_WIDTH 拍，与同步信号对齐。

module video_delay
#(
	parameter DATA_WIDTH = 24
)
(
	input                       video_clk,
	input                       rst,
	output                      read_en,
	input[DATA_WIDTH - 1:0]     read_data,
	input                      hs,
	input                      vs,
	input                      de,

	output                      hs_r,
	output                      vs_r,
	output                      de_r,
	output[DATA_WIDTH - 1:0]    vout_data

);
reg [20:0] hs_d;
reg [20:0] vs_d;
reg [20:0] de_d;
reg[DATA_WIDTH - 1:0]  vout_data_r;

assign read_en = de_d[18];
assign hs_r = hs_d[20];
assign vs_r = vs_d[20];
assign de_r = de_d[20];
assign vout_data = vout_data_r;
always@(posedge video_clk or posedge rst)
begin
	if(rst == 1'b1)
		vout_data_r <= {DATA_WIDTH{1'b0}};
	else if(de_d[19])
		vout_data_r <= read_data;
	else
		vout_data_r <= {DATA_WIDTH{1'b0}};
end
always @(posedge video_clk or posedge rst)begin
	if(rst)begin
    	hs_d <= 20'b0;
        vs_d <= 20'b0;
        de_d <= 20'b0;
    end
	else begin

    	hs_d <= {hs_d[19:0],hs};
        vs_d <= {vs_d[19:0],vs};
        de_d <= {de_d[19:0],de};
    end
end
endmodule
