// 转场特效控制。换图时按设定模式输出进度，供 OSD 层做淡入淡出或推移。

module trans_ctrl #(

    parameter [8:0] STEP_DN  = 9'd48,
    parameter [8:0] STEP_UP  = 9'd24,
    parameter [8:0] WIPE_DN  = 9'd8,
    parameter [8:0] WIPE_UP  = 9'd6
)(
    input  wire       clk,
    input  wire       rst_n,

    input  wire       vs,
    input  wire [2:0] buf_idx_in,
    input  wire       swap_pulse,

    input  wire       cut_mode,

    input  wire       wipe_mode,

    output reg  [2:0] buf_idx_out,
    output reg  [8:0] fade,
    output reg  [2:0] alt_idx
);

    localparam [1:0] S_UP   = 2'd0;
    localparam [1:0] S_DOWN = 2'd1;
    localparam [1:0] S_HOLD = 2'd2;

    reg [1:0] state;
    reg       pending;
    reg [2:0] target;
    reg       hold_ph;
    reg       vs_d;

    wire [8:0] step_dn = wipe_mode ? WIPE_DN : STEP_DN;
    wire [8:0] step_up = wipe_mode ? WIPE_UP : STEP_UP;

    wire frame_tick = (vs_d != vs) && (vs == 1'b1);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            vs_d        <= 1'b0;
            state       <= S_UP;
            pending     <= 1'b0;
            target      <= 3'd0;
            hold_ph     <= 1'b0;
            buf_idx_out <= 3'd0;
            fade        <= 9'd0;
            alt_idx     <= 3'd0;
        end else begin
            vs_d <= vs;

            if (cut_mode) begin
                pending <= 1'b0;
                hold_ph <= 1'b0;
                state   <= S_UP;
                fade    <= 9'd256;
                if (swap_pulse && (buf_idx_in != buf_idx_out))
                    buf_idx_out <= buf_idx_in;
            end else begin

            if (swap_pulse && (buf_idx_in != buf_idx_out)) begin
                pending <= 1'b1;
                target  <= buf_idx_in;
                alt_idx <= buf_idx_in;
            end

            case (state)

                S_UP: begin
                    if (pending) begin
                        state   <= S_DOWN;
                    end else if (frame_tick) begin
                        if (fade + step_up >= 9'd256)
                            fade <= 9'd256;
                        else
                            fade <= fade + step_up;
                    end
                end

                S_DOWN: begin
                    if (frame_tick) begin
                        if (fade <= step_dn) begin
                            fade     <= 9'd0;
                            hold_ph  <= 1'b0;
                            state    <= S_HOLD;
                        end else begin
                            fade <= fade - step_dn;
                        end
                    end
                end

                S_HOLD: begin
                    if (frame_tick) begin
                        if (!hold_ph) begin

                            buf_idx_out <= target;
                            pending     <= 1'b0;
                            if (wipe_mode) begin

                                fade  <= 9'd256;
                                state <= S_UP;
                            end else begin
                                hold_ph <= 1'b1;
                            end
                        end else begin

                            hold_ph <= 1'b0;
                            state   <= S_UP;
                        end
                    end
                end

                default: state <= S_UP;
            endcase
            end
        end
    end

endmodule
