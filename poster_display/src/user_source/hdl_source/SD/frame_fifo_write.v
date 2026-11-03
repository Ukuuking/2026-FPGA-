// 写入侧帧 FIFO。写满必须停写，否则字被静默丢弃，整帧错位。

`timescale 1ns/1ps
module frame_fifo_write
#
(
	parameter MEM_DATA_BITS          = 32,
	parameter ADDR_BITS              = 21,
	parameter BURST_BITS             = 9,
	parameter BURST_SIZE             = 128,
	parameter WRITE_V_FLIP           = 1,
	parameter FRAME_WIDTH            = 640,
	parameter FRAME_HEIGHT           = 480
)
(
	input                            rst,
	input                            mem_clk,
	input							 Sdr_init_done,
	input							 Sdr_init_ref_vld,
    input							 Sdr_busy,
	input							 App_rd_busy,
    output							 O_wr_busy,

	output 							 App_wr_en,
	output  [ADDR_BITS - 1:0]	 	App_wr_addr,

	input                            write_req,
	output reg                       write_req_ack,
	output                           write_finish,
	input[ADDR_BITS - 1:0]           write_addr_0,
	input[ADDR_BITS - 1:0]           write_addr_1,
	input[ADDR_BITS - 1:0]           write_addr_2,
	input[ADDR_BITS - 1:0]           write_addr_3,
	input[1:0]                       write_addr_index,
	input[ADDR_BITS - 1:0]           write_len,
	output reg                       fifo_aclr,
	input[9:0]                      rdusedw
);
localparam ONE                       = 256'd1;
localparam ZERO                      = 256'd0;

localparam [ADDR_BITS - 1:0] VFLIP_FIRST_ADDR_OFFSET = (FRAME_HEIGHT - 1) * FRAME_WIDTH;
localparam [ADDR_BITS - 1:0] VFLIP_LINE_JUMP         = (FRAME_WIDTH * 2) - 1;
localparam [15:0]            VFLIP_LINE_LAST_X       = FRAME_WIDTH - 1;

localparam S_IDLE                    = 0;
localparam S_ACK                     = 1;
localparam S_CHECK_FIFO              = 2;
localparam S_WRITE_BURST             = 3;
localparam S_WRITE_BURST_END         = 4;
localparam S_END                     = 5;

reg                                 write_req_d0;
reg                                 write_req_d1;
reg                                 write_req_d2;
reg[ADDR_BITS - 1:0]                write_len_d0;
reg[ADDR_BITS - 1:0]                write_len_d1;
reg[ADDR_BITS - 1:0]                write_len_latch;
reg[ADDR_BITS - 1:0]                write_cnt;
reg[1:0]                            write_addr_index_d0;
reg[1:0]                            write_addr_index_d1;
reg[3:0]                            state;
reg [ADDR_BITS - 1:0]	 App_wr_addr_r;
reg [15:0]                            wr_x;

reg [BURST_BITS - 1:0]				burst_cnt;
wire								wr_burst_finish;

reg App_wr_en_r;
reg App_wr_en_d0;

wire into_burst;

reg  [9:0] rdusedw_r;
always @(posedge mem_clk or posedge rst)
begin
	if(rst == 1'b1)
		rdusedw_r <= 10'd0;
	else
		rdusedw_r <= rdusedw;
end

assign into_burst = (((write_len_latch <= (rdusedw_r + write_cnt))||rdusedw_r > BURST_SIZE) && ~App_rd_busy);
assign App_wr_addr = {App_wr_addr_r[ADDR_BITS - 1:0]};

assign O_wr_busy = (state == S_WRITE_BURST || (state == S_CHECK_FIFO && into_burst));
assign wr_burst_finish = (burst_cnt >= BURST_SIZE);
assign write_finish = (state == S_END) ? 1'b1 : 1'b0;
assign App_wr_en = App_wr_en_d0;

always@(posedge mem_clk or posedge rst)
begin
	if(rst == 1'b1)
	begin
		write_req_d0    <=  1'b0;
		write_req_d1    <=  1'b0;
		write_req_d2    <=  1'b0;
		write_len_d0    <=  ZERO[ADDR_BITS - 1:0];
		write_len_d1    <=  ZERO[ADDR_BITS - 1:0];
		write_addr_index_d0    <=  2'b00;
		write_addr_index_d1    <=  2'b00;
	end
	else
	begin
		write_req_d0    <=  write_req;
		write_req_d1    <=  write_req_d0;
		write_req_d2    <=  write_req_d1;
		write_len_d0    <=  write_len;
		write_len_d1    <=  write_len_d0;
		write_addr_index_d0 <= write_addr_index;
		write_addr_index_d1 <= write_addr_index_d0;
	end
end
always @(posedge mem_clk or posedge rst)
begin
	if(rst == 1'b1)
	begin
		burst_cnt <= ZERO[BURST_BITS - 1:0];
		App_wr_addr_r <= ZERO[ADDR_BITS - 1:0];
		App_wr_en_d0 <= 1'b0;
		wr_x <= 16'd0;
	end
	else begin

		if(state == S_CHECK_FIFO)
			burst_cnt <= ZERO[BURST_BITS - 1:0];
		else if(App_wr_en)
			burst_cnt <= burst_cnt + 1'b1;
		else
			burst_cnt <= burst_cnt;

		if(state == S_ACK)
			begin
				wr_x <= 16'd0;
				if(write_addr_index_d1 == 2'd0)
					App_wr_addr_r <= WRITE_V_FLIP ? (write_addr_0 + VFLIP_FIRST_ADDR_OFFSET) : write_addr_0;
				else if(write_addr_index_d1 == 2'd1)
					App_wr_addr_r <= WRITE_V_FLIP ? (write_addr_1 + VFLIP_FIRST_ADDR_OFFSET) : write_addr_1;
				else if(write_addr_index_d1 == 2'd2)
					App_wr_addr_r <= WRITE_V_FLIP ? (write_addr_2 + VFLIP_FIRST_ADDR_OFFSET) : write_addr_2;
				else if(write_addr_index_d1 == 2'd3)
					App_wr_addr_r <= WRITE_V_FLIP ? (write_addr_3 + VFLIP_FIRST_ADDR_OFFSET) : write_addr_3;
			end
		else if(App_wr_en)
			begin
				if(WRITE_V_FLIP)
				begin
					if(wr_x == VFLIP_LINE_LAST_X)
					begin
						wr_x <= 16'd0;
						App_wr_addr_r <= App_wr_addr_r - VFLIP_LINE_JUMP;
					end
					else
					begin
						wr_x <= wr_x + 1'b1;
						App_wr_addr_r <= App_wr_addr_r + 1'b1;
					end
				end
				else
					App_wr_addr_r <= App_wr_addr_r + 1'b1;
			end
		else
			App_wr_addr_r <= App_wr_addr_r;

		if(App_wr_en_r && burst_cnt + App_wr_en < BURST_SIZE && (burst_cnt + write_cnt + App_wr_en < write_len_latch))begin
            App_wr_en_d0 <= 1'b1;
        end
		else
			App_wr_en_d0 <= 1'b0;

	end
end
always@(posedge mem_clk or posedge rst)
begin
	if(rst == 1'b1)
	begin
		state <= S_IDLE;
		write_len_latch <= ZERO[ADDR_BITS - 1:0];

		App_wr_en_r <= 1'b0;

		write_cnt <= ZERO[ADDR_BITS - 1:0];
		fifo_aclr <= 1'b0;
		write_req_ack <= 1'b0;

	end
	else
		case(state)

			S_IDLE:
			begin
				if(write_req_d2 == 1'b1 && Sdr_init_done)
				begin
					state <= S_ACK;
				end
				write_req_ack <= 1'b0;
			end

			S_ACK:
			begin

				if(write_req_d2 == 1'b0)
				begin
					state <= S_CHECK_FIFO;
					fifo_aclr <= 1'b0;
					write_req_ack <= 1'b0;
				end
				else
				begin

					write_req_ack <= 1'b1;

					fifo_aclr <= 1'b1;

					write_len_latch <= write_len_d1;
				end

				write_cnt <= ZERO[ADDR_BITS - 1:0];
			end
			S_CHECK_FIFO:
			begin

				if(write_req_d2 == 1'b1)
				begin
					state <= S_ACK;
				end

				else if(into_burst)
				begin
					state <= S_WRITE_BURST;

					App_wr_en_r <= 1'b1;
				end
			end

			S_WRITE_BURST:
			begin

				if(wr_burst_finish == 1'b1)
				begin
					App_wr_en_r <= 1'b0;
					state <= S_WRITE_BURST_END;

					write_cnt <= write_cnt + BURST_SIZE[ADDR_BITS - 1:0];

				end
			end
			S_WRITE_BURST_END:
			begin

				if(write_req_d2 == 1'b1)
				begin
					state <= S_ACK;
				end

				else if(write_cnt < write_len_latch)
				begin
					state <= S_CHECK_FIFO;
				end
				else
				begin
					state <= S_END;
				end
			end
			S_END:
			begin
				state <= S_IDLE;
			end
			default:
				state <= S_IDLE;
		endcase
end
endmodule
