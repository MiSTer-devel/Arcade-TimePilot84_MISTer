//============================================================================
// 
//  Time Pilot '84 sound PCB model
//  Copyright (C) 2020, 2021 Ace, ElectronAsh & Enforcer
//
//  Permission is hereby granted, free of charge, to any person obtaining a
//  copy of this software and associated documentation files (the "Software"),
//  to deal in the Software without restriction, including without limitation
//  the rights to use, copy, modify, merge, publish, distribute, sublicense,
//  and/or sell copies of the Software, and to permit persons to whom the 
//  Software is furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in
//  all copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
//  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
//  DEALINGS IN THE SOFTWARE.
//
//============================================================================

module TimePilot84_SND
(
	input                reset,
	input                clk_49m,         //Actual frequency: 49.152MHz
	input         [15:0] dip_sw,
	input          [1:0] coin,                     //0 = coin 1, 1 = coin 2
	input          [1:0] start_buttons,            //0 = Player 1, 1 = Player 2
	input          [3:0] p1_joystick, p2_joystick, //0 = up, 1 = down, 2 = left, 3 = right
	input          [2:0] p1_buttons,
	input          [1:0] p2_buttons,
	input                btn_service,
	input                cpubrd_A5, cpubrd_A6,
	input                cs_controls_dip1, cs_dip2,
	input                irq_trigger, cs_sounddata,
	input          [7:0] cpubrd_Din,
	
	output         [7:0] controls_dip,
	output signed [15:0] sound,
	
	//This input serves to select different fractional dividers to acheive 3.579545MHz for the Z80 and 1.789772MHz for the
	//SN76489s depending on whether Time Pilot '84 runs with original or underclocked timings to normalize sync frequencies
	input                underclock,
	
	input                ep6_cs_i,
	input         [24:0] ioctl_addr,
	input          [7:0] ioctl_data,
	input                ioctl_wr
	//The sound board contains a video passthrough but video will instead be tapped
	//straight from the CPU board implementation (this passthrough is redundant for
	//an FPGA implementation)
);

//------------------------------------------------------- Signal outputs -------------------------------------------------------//

//Multiplex controls and DIP switches to be output to CPU board
assign controls_dip = cs_controls_dip1 ? controls_dip1:
                      cs_dip2          ? dip_sw[15:8]:
                      8'hFF;

//------------------------------------------------------- Clock division -------------------------------------------------------//

//Generate clock enables for sound data and IRQ logic, and DC offset removal
reg [8:0] div = 9'd0;
always_ff @(posedge clk_49m) begin
	div <= div + 9'd1;
end
wire cen_3m = !div[3:0];
wire cen_dcrm = !div;

//Generate 3.579545MHz clock enable for Z80, 1.789772MHz clock enable for SN76489s, clock enable for Z80 timer
//(uses Jotego's fractional clock divider from JTFRAME)
wire [9:0] sound_cen_n = underclock ? 10'd62 : 10'd60;
wire [9:0] sound_cen_m = underclock ? 10'd843 : 10'd824;
wire cen_3m58, cen_1m79, cen_timer;
jtframe_frac_cen #(11) sound_cen
(
	.clk(clk_49m),
	.n(sound_cen_n),
	.m(sound_cen_m),
	.cen({cen_timer, 8'bZZZZZZZZ, cen_1m79, cen_3m58})
);

//------------------------------------------------------------ CPU -------------------------------------------------------------//

//Sound CPU (Zilog Z80 - uses T80s version of the T80 soft core)
wire [15:0] sound_A;
wire [7:0] sound_Dout;
wire n_m1, n_mreq, n_iorq, n_rd, n_wr, n_rfsh;
T80s A9
(
	.RESET_n(reset),
	.CLK(clk_49m),
	.CEN(cen_3m58),
	.INT_n(n_irq),
	.M1_n(n_m1),
	.MREQ_n(n_mreq),
	.IORQ_n(n_iorq),
	.RD_n(n_rd),
	.WR_n(n_wr),
	.RFSH_n(n_rfsh),
	.A(sound_A),
	.DI(sound_Din),
	.DO(sound_Dout)
);
//Address decoding for Z80
wire n_rw = n_rd & n_wr;
wire cs_soundrom = (~n_rw & ~n_mreq & n_rfsh & (sound_A[15:13] == 3'b000));
wire cs_soundram = (~n_rw & ~n_mreq & n_rfsh & (sound_A[15:13] == 3'b010));
wire cs_sound = (~n_rw & ~n_mreq & n_rfsh & (sound_A[15:13] == 3'b011));
wire cs_timer = (~n_rw & ~n_mreq & n_rfsh & (sound_A[15:13] == 3'b100));
wire cs_lpf = (~n_rw & ~n_mreq & n_rfsh & (sound_A[15:13] == 3'b101));
wire cs_snlatch = (~n_rw & ~n_mreq & n_rfsh & (sound_A[15:13] == 3'b110) & (sound_A[2:0] == 3'b000));
wire cs_sn0 = (~n_rw & ~n_mreq & n_rfsh & (sound_A[15:13] == 3'b110) & (sound_A[2:0] == 3'b001));
wire cs_sn2 = (~n_rw & ~n_mreq & n_rfsh & (sound_A[15:13] == 3'b110) & (sound_A[2:0] == 3'b011));
wire cs_sn3 = (~n_rw & ~n_mreq & n_rfsh & (sound_A[15:13] == 3'b110) & (sound_A[2:0] == 3'b100));
//Multiplex data input to Z80
wire [7:0] sound_Din = cs_soundrom          ? eprom6_D:
                       (cs_soundram & n_wr) ? sndram_D:
                       cs_sound             ? sound_D:
                       cs_timer             ? {4'hF, timer}:
                       8'hFF;

//Sound ROM
wire [7:0] eprom6_D;
eprom_6 A6
(
	.ADDR(sound_A[12:0]),
	.CLK(clk_49m),
	.DATA(eprom6_D),
	.ADDR_DL(ioctl_addr),
	.CLK_DL(clk_49m),
	.DATA_IN(ioctl_data),
	.CS_DL(ep6_cs_i),
	.WR(ioctl_wr)
);

//Z80 RAM (lower 4 bits)
wire [7:0] sndram_D;
spram #(4, 10) A2
(
	.clk(clk_49m),
	.we(cs_soundram & ~n_wr),
	.addr(sound_A[9:0]),
	.data(sound_Dout[3:0]),
	.q(sndram_D[3:0])
);

//Z80 RAM (upper 4 bits)
spram #(4, 10) A3
(
	.clk(clk_49m),
	.we(cs_soundram & ~n_wr),
	.addr(sound_A[9:0]),
	.data(sound_Dout[7:4]),
	.q(sndram_D[7:4])
);

//Latch sound data coming in from CPU board
reg [7:0] sound_D = 8'd0;
always_ff @(posedge clk_49m) begin
	if(cen_3m && cs_sounddata)
		sound_D <= cpubrd_Din;
end

//Generate Z80 interrupts
wire irq_clr = (~reset | ~(n_iorq | n_m1));
reg n_irq = 1;
always_ff @(posedge clk_49m or posedge irq_clr) begin
	if(irq_clr)
		n_irq <= 1;
	else if(cen_3m && irq_trigger)
		n_irq <= 0;
end

//Z80 timer
reg [3:0] timer = 4'd0;
always_ff @(posedge clk_49m) begin
	if(cen_timer)
		timer <= timer + 4'd1;
end

//--------------------------------------------------- Controls & DIP switches --------------------------------------------------//

//Multiplex player inputs and DIP switch bank 1
wire [7:0] controls_dip1 = ({cpubrd_A6, cpubrd_A5} == 2'b00) ? {3'b111, start_buttons, btn_service, coin}:
                           ({cpubrd_A6, cpubrd_A5} == 2'b01) ? {1'b1, p1_buttons, p1_joystick[1:0], p1_joystick[3:2]}:
                           ({cpubrd_A6, cpubrd_A5} == 2'b10) ? {2'b11, p2_buttons, p2_joystick[1:0], p2_joystick[3:2]}:
                           ({cpubrd_A6, cpubrd_A5} == 2'b11) ? dip_sw[7:0]:
                           8'hFF;

//--------------------------------------------------------- Sound chips --------------------------------------------------------//

//Generate chip enables for all SN76489s
wire n_sn0_ce = (~cs_sn0 & sn0_ready);
wire n_sn2_ce = (~cs_sn2 & sn2_ready);
wire n_sn3_ce = (~cs_sn3 & sn3_ready);

//Latch data from Z80 to SN76489s
reg [7:0] sn_D = 8'd0;
always_ff @(posedge clk_49m) begin
	if(cen_3m58 && cs_snlatch)
		sn_D <= sound_Dout;
end

//Sound chip 1 (Texas Instruments SN76489 - uses Arnim Laeuger's SN76489 implementation with bugfixes)
wire [7:0] sn0_raw;
wire sn0_ready;
sn76489_top E5
(
	.clock_i(clk_49m),
	.clock_en_i(cen_1m79),
	.res_n_i(reset),
	.ce_n_i(n_sn0_ce),
	.we_n_i(sn0_ready),
	.ready_o(sn0_ready),
	.d_i(sn_D),
	.aout_o(sn0_raw)
);

//Sound chip 2 (Texas Instruments SN76489 - uses Arnim Laeuger's SN76489 implementation with bugfixes)
wire [7:0] sn2_raw;
wire sn2_ready;
sn76489_top E6
(
	.clock_i(clk_49m),
	.clock_en_i(cen_1m79),
	.res_n_i(reset),
	.ce_n_i(n_sn2_ce),
	.we_n_i(sn2_ready),
	.ready_o(sn2_ready),
	.d_i(sn_D),
	.aout_o(sn2_raw)
);

//Sound chip 3 (Texas Instruments SN76489 - uses Arnim Laeuger's SN76489 implementation with bugfixes)
wire [7:0] sn3_raw;
wire sn3_ready;
sn76489_top E7
(
	.clock_i(clk_49m),
	.clock_en_i(cen_1m79),
	.res_n_i(reset),
	.ce_n_i(n_sn3_ce),
	.we_n_i(sn3_ready),
	.ready_o(sn3_ready),
	.d_i(sn_D),
	.aout_o(sn3_raw)
);

//----------------------------------------------------- Final audio output -----------------------------------------------------//

//Apply gain and remove DC offset from SN76489s (uses jt49_dcrm2 from JT49 by Jotego for DC offset removal)
wire [15:0] sn0_gain = sn0_raw * 16'd176;
wire [15:0] sn2_gain = sn2_raw * 16'd176;
wire [15:0] sn3_gain = sn3_raw * 16'd176;
wire signed [15:0] sn0_dcrm, sn2_dcrm, sn3_dcrm;
jt49_dcrm2 #(16) dcrm_sn0
(
	.clk(clk_49m),
	.cen(cen_dcrm),
	.rst(~reset),
	.din(sn0_gain),
	.dout(sn0_dcrm)
);
jt49_dcrm2 #(16) dcrm_sn2
(
	.clk(clk_49m),
	.cen(cen_dcrm),
	.rst(~reset),
	.din(sn2_gain),
	.dout(sn2_dcrm)
);
jt49_dcrm2 #(16) dcrm_sn3
(
	.clk(clk_49m),
	.cen(cen_dcrm),
	.rst(~reset),
	.din(sn3_gain),
	.dout(sn3_dcrm)
);

//Time Pilot '84's SN76489s contain selectable low-pass filters with the following cutoff frequencies:
//3386.28Hz, 723.43Hz, 596.09Hz
//Model this here (the PCB handles this via a 74HC4066 switching IC located at C6)
wire signed [15:0] sn2_filt, sn3_filt;
wire signed [15:0] sn0_light, sn0_med, sn0_heavy;
wire signed [15:0] sn0_sound, sn1_sound, sn2_sound, sn3_sound;
tp84_lpf_light sn0_lpf_light
(
	.clk(clk_49m),
	.reset(~reset),
	.in(sn0_dcrm),
	.out(sn0_light)
);

tp84_lpf_medium sn0_lpf_medium
(
	.clk(clk_49m),
	.reset(~reset),
	.in(sn0_dcrm),
	.out(sn0_med)
);

tp84_lpf_heavy sn0_lpf_heavy
(
	.clk(clk_49m),
	.reset(~reset),
	.in(sn0_dcrm),
	.out(sn0_heavy)
);

tp84_lpf_light sn2_lpf
(
	.clk(clk_49m),
	.reset(~reset),
	.in(sn2_dcrm),
	.out(sn2_filt)
);

tp84_lpf_light sn3_lpf
(
	.clk(clk_49m),
	.reset(~reset),
	.in(sn3_dcrm),
	.out(sn3_filt)
);

//Latch low-pass filter control lines
reg [1:0] sn0_filter = 2'd0;
reg sn2_filter = 0;
reg sn3_filter = 0;
always_ff @(posedge clk_49m) begin
	if(cen_3m58 && cs_lpf) begin
		sn3_filter <= sound_A[8];
		sn2_filter <= sound_A[7];
		sn0_filter <= sound_A[4:3];
	end
end

//Apply audio filtering based on the state of the low-pass filter controls
always_comb begin
	case(sn0_filter)
		2'b00: sn0_sound = sn0_dcrm;
		2'b01: sn0_sound = sn0_light;
		2'b10: sn0_sound = sn0_med;
		2'b11: sn0_sound = sn0_heavy;
	endcase
end

assign sn2_sound = sn2_filter ? sn2_filt : sn2_dcrm;
assign sn3_sound = sn3_filter ? sn3_filt : sn3_dcrm;

//Mix all SN76489s and apply an anti-aliasing low-pass filter to prevent ringing noises when using low-pass filtering via
//the MiSTer OSD at 48KHz (this game also has variable low-pass filtering based on how loud the PCB's volume dial is set and
//will be modeled externally)
wire signed [15:0] sn76489_mix = (sn0_sound + sn2_sound + sn3_sound);
tp84_lpf aalpf
(
	.clk(clk_49m),
	.reset(~reset),
	.in(sn76489_mix),
	.out(sound)
);

endmodule
