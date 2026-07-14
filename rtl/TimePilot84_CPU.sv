//============================================================================
// 
//  Time Pilot '84 main PCB model
//  Copyright (C) 2020, 2021 Ace
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

//Module declaration, I/O ports
module TimePilot84_CPU
(
	input         reset,
	input         clk_49m, //Actual frequency: 49.152MHz
	output  [3:0] red, green, blue, //12-bit RGB, 4 bits per color
	output        video_hsync, video_vsync, video_csync, //CSync not needed for MISTer
	output        video_hblank, video_vblank,
	output        ce_pix,
	
	input   [7:0] controls_dip,
	output  [7:0] cpubrd_Dout,
	output        cpubrd_A5, cpubrd_A6,
	output        cs_sounddata, irq_trigger,
	output        cs_dip2, cs_controls_dip1,
	
	input         is_set3, //Flag to remap primary CPU address space for Time Pilot '84 Set 3
	
	//Screen centering (alters HSync, VSync and VBlank timing in the Konami 082 to reposition the video output)
	input   [3:0] h_center, v_center,
	
	input         ep1_cs_i,
	input         ep2_cs_i,
	input         ep3_cs_i,
	input         ep4_cs_i,
	input         ep5_cs_i,
	input         ep7_cs_i,
	input         ep8_cs_i,
	input         ep9_cs_i,
	input         ep10_cs_i,
	input         ep11_cs_i,
	input         ep12_cs_i,
	input         cp1_cs_i,
	input         cp2_cs_i,
	input         cp3_cs_i,
	input         tl_cs_i,
	input         sl_cs_i,
	input  [24:0] ioctl_addr,
	input   [7:0] ioctl_data,
	input         ioctl_wr,

	input         pause,

	input  [15:0] hs_address,
	input   [7:0] hs_data_in,
	output  [7:0] hs_data_out,
	input         hs_write,
	input         hs_access
);

//------------------------------------------------------- Signal outputs -------------------------------------------------------//

//Assign active high HBlank and VBlank outputs
assign video_hblank = hblk;
assign video_vblank = vblk;

//Output pixel clock enable
assign ce_pix = cen_6m;

//Output select lines for player inputs and DIP switches to sound board
assign cs_controls_dip1 = is_set3 ? ((mA[15:4] == 12'h1A0 | mA[15:4] == 12'h1A2 | mA[15:4] == 12'h1A4 | mA[15:4] == 12'h1A6) & m_rw):
                                    ((mA[15:4] == 12'h280 | mA[15:4] == 12'h282 | mA[15:4] == 12'h284 | mA[15:4] == 12'h286) & m_rw);
assign cs_dip2 = is_set3 ? ((mA[15:8] == 8'h1C) & m_rw) : ((mA[15:8] == 8'h30) & m_rw);

//Output primary MC6809E address lines A5 and A6 to sound board
assign cpubrd_A5 = mA[5];
assign cpubrd_A6 = mA[6];

//Assign CPU board data output to sound board
assign cpubrd_Dout = mD_out;

//Generate and output chip select for latching sound data to sound CPU
assign cs_sounddata = is_set3 ? ((mA[15:4] == 12'h1E8) & ~m_rw) : ((mA[15:8] == 8'h3A) & ~m_rw);

//Generate sound IRQ trigger
reg sound_irq = 1;
always_ff @(posedge clk_49m) begin
	if(cen_3m) begin
		if(cs_soundirq)
			sound_irq <= 1;
		else
			sound_irq <= 0;
	end
end
assign irq_trigger = sound_irq;

//------------------------------------------------------- Clock division -------------------------------------------------------//

//Generate 6.144MHz, 3.072MHz and 1.576MHz clock enables
reg [4:0] div = 5'd0;
always_ff @(posedge clk_49m) begin
	div <= div + 5'd1;
end
reg [2:0] n_div = 3'd0;
always_ff @(negedge clk_49m) begin
	n_div <= n_div + 3'd1;
end
wire cen_6m = !div[2:0];
wire n_cen_6m = !n_div;
wire cen_3m = !div[3:0];
wire cen_1m5 = !div;

//Generate E clock enables for MC6809Es (code adapted from Sorgelig's phase generator used in the MiSTer Vectrex core)
reg mE = 0;
reg mQ = 0;
reg sE = 0;
reg sQ = 0;
always_ff @(posedge clk_49m) begin
	reg [1:0] clk_phase =0;
	mE <= 0;
	mQ <= 0;
	sE <= 0;
	sQ <= 0;
	if(cen_6m) begin
		clk_phase <= clk_phase + 1'd1;
		case(clk_phase)
			2'b00: sE <= 1;
			2'b01: mQ <= 1;
			2'b10: mE <= 1;
			2'b11: sQ <= 1;
		endcase
	end
end

//------------------------------------------------------------ CPUs ------------------------------------------------------------//

//Primary CPU - Motorola MC6809E (uses synchronous version of Greg Miller's cycle-accurate MC6809E made by Sorgelig with an
//additional bugfix by Arnim Laeuger and Jotego)
wire [15:0] mA;
wire [7:0] mD_out;
wire m_rw;
mc6809is u12G
(
	.CLK(clk_49m),
	.fallE_en(mE),
	.fallQ_en(mQ),
	.D(mD_in),
	.DOut(mD_out),
	.ADDR(mA),
	.RnW(m_rw),
	.nIRQ(mirq),
	.nFIRQ(1),
	.nNMI(1),
	.nHALT(pause), 
	.nRESET(reset),
	.nDMABREQ(1)
);
//Address decoding for primary MC6809E (Time Pilot '84 (Set 3) has everything but its ROMs relocated to different parts of the
//primary MC6809E's address space - reconfigure this based on the state of the is_set3 flag)
wire cs_vram = is_set3 ? (mA[15:11] == 5'b00000) : (mA[15:11] == 5'b01000);
wire cs_cram = is_set3 ? (mA[15:11] == 5'b00001) : (mA[15:11] == 5'b01001);
wire cs_msharedram = is_set3 ? (mA[15:11] == 5'b00010) : (mA[15:11] == 5'b01010);
wire cs_mainlatch = is_set3 ? ((mA[15:8] == 8'h1C) & ~m_rw) : ((mA[15:8] == 8'h30) & ~m_rw);
wire cs_xscrollreg = is_set3 ? ((mA[15:8] == 8'h1F) & ~m_rw) : ((mA[15:8] == 8'h3C) & ~m_rw);
wire cs_yscrollreg = is_set3 ? ((mA[15:4] == 12'h1F8) & ~m_rw) : ((mA[15:8] == 8'h3E) & ~m_rw);
wire cs_colorlatch = is_set3 ? ((mA[15:8] == 8'h1A) & ~m_rw) : ((mA[15:8] == 8'h28) & ~m_rw);
wire cs_soundirq = is_set3 ? ((mA[15:8] == 8'h1E) & ~m_rw) : ((mA[15:8] == 8'h38) & ~m_rw);
wire cs_rom1 = (mA[15:13] == 3'b100);
wire cs_rom2 = (mA[15:13] == 3'b101);
wire cs_rom3 = (mA[15:13] == 3'b110);
wire cs_rom4 = (mA[15:13] == 3'b111);
//Multiplex data inputs to primary MC6809E
wire [7:0] mD_in = (cs_controls_dip1 | cs_dip2) ? controls_dip:
                   (cs_vram & m_rw)             ? vram_Dout:
                   (cs_cram & m_rw)             ? cram_Dout:
                   (cs_msharedram & m_rw)       ? m_sharedram_D:
                   cs_rom1                      ? eprom1_D:
                   cs_rom2                      ? eprom2_D:
                   cs_rom3                      ? eprom3_D:
                   cs_rom4                      ? eprom4_D:
                   8'hFF;

//Primary CPU ROMs (there is a 5th ROM socket on the original PCB at 6J, but is unpopulated)
//ROM 1/4
wire [7:0] eprom1_D;
eprom_1 u7J
(
	.ADDR(mA[12:0]),
	.CLK(clk_49m),
	.DATA(eprom1_D),
	.ADDR_DL(ioctl_addr),
	.CLK_DL(clk_49m),
	.DATA_IN(ioctl_data),
	.CS_DL(ep1_cs_i),
	.WR(ioctl_wr)
);
//ROM 2/4
wire [7:0] eprom2_D;
eprom_2 u8J
(
	.ADDR(mA[12:0]),
	.CLK(clk_49m),
	.DATA(eprom2_D),
	.ADDR_DL(ioctl_addr),
	.CLK_DL(clk_49m),
	.DATA_IN(ioctl_data),
	.CS_DL(ep2_cs_i),
	.WR(ioctl_wr)
);
//ROM 3/4
wire [7:0] eprom3_D;
eprom_3 u9J
(
	.ADDR(mA[12:0]),
	.CLK(clk_49m),
	.DATA(eprom3_D),
	.ADDR_DL(ioctl_addr),
	.CLK_DL(clk_49m),
	.DATA_IN(ioctl_data),
	.CS_DL(ep3_cs_i),
	.WR(ioctl_wr)
);
//ROM 4/4
wire [7:0] eprom4_D;
eprom_4 u10J
(
	.ADDR(mA[12:0]),
	.CLK(clk_49m),
	.DATA(eprom4_D),
	.ADDR_DL(ioctl_addr),
	.CLK_DL(clk_49m),
	.DATA_IN(ioctl_data),
	.CS_DL(ep4_cs_i),
	.WR(ioctl_wr)
);

//Generate H/V flip signals and primary MC6809E IRQ mask from main latch
reg vflip, hflip, mirq_mask;
always_ff @(posedge clk_49m) begin
	if(!reset) begin
		vflip <= 0;
		hflip <= 0;
		mirq_mask <= 0;
	end
	else if(cen_3m) begin
		if(cs_mainlatch)
			case(mA[2:0])
				3'b000: mirq_mask <= mD_out[0];
				3'b100: vflip <= mD_out[0];
				3'b101: hflip <= mD_out[0];
				default:;
		endcase
	end
end

//Color latch
reg [4:0] color_latch;
always_ff @(posedge clk_49m) begin
	if(!reset)
		color_latch <= 5'd0;
	else if(cen_3m && cs_colorlatch)
		color_latch <= mD_out[4:0];
end

//Generate VBlank IRQ for primary MC6809E
reg mirq = 1;
always_ff @(posedge clk_49m) begin
	if(cen_6m) begin
		if(!mirq_mask)
			mirq <= 1;
		else if(vblank_irq_en)
			mirq <= 0;
	end
end

//Secondary CPU - Motorola MC6809E (uses synchronous version of Greg Miller's cycle-accurate MC6809E made by Sorgelig with an
//additional bugfix by Arnim Laeuger and Jotego)
wire [15:0] sA;
wire [7:0] sD_out;
wire s_rw;
mc6809is u12E
(
	.CLK(clk_49m),
	.fallE_en(sE),
	.fallQ_en(sQ),
	.D(sD_in),
	.DOut(sD_out),
	.ADDR(sA),
	.RnW(s_rw),
	.nIRQ(sirq),
	.nFIRQ(1),
	.nNMI(1),
	.nHALT(1), 
	.nRESET(reset),
	.nDMABREQ(1)
);
//Address decoding for secondary MC6809E
wire cs_beam = (sA[15:13] == 3'b001);
wire cs_sirqmask = ((sA[15:13] == 3'b010) & ~s_rw);
wire cs_ssharedram = (sA[15:11] == 5'b10000);
wire cs_spriteram = (sA[15:13] == 3'b011);
wire cs_rom5 = (sA[15:13] == 3'b111);
//Multiplex data inputs to secondary MC6809E
wire [7:0] sD_in = cs_rom5                          ? eprom5_D:
                   (cs_ssharedram & s_rw)           ? s_sharedram_D:
                   (cs_spriteram1 & ~spriteram1_we) ? spriteram_D[15:8]:
                   (cs_spriteram0 & ~spriteram0_we) ? spriteram_D[7:0]:
                   cs_beam                          ? vcnt_lat:
                   8'hFF;

//Secondary CPU ROM
wire [7:0] eprom5_D;
eprom_5 u10D
(
	.ADDR(sA[12:0]),
	.CLK(clk_49m),
	.DATA(eprom5_D),
	.ADDR_DL(ioctl_addr),
	.CLK_DL(clk_49m),
	.DATA_IN(ioctl_data),
	.CS_DL(ep5_cs_i),
	.WR(ioctl_wr)
);

//Generate IRQ mask for secondary MC6809E
reg sirq_mask;
always_ff @(posedge clk_49m) begin
	if(!reset)
		sirq_mask <= 0;
	else if(cen_3m && cs_sirqmask)
		sirq_mask <= sD_out[0];
end

//Generate VBlank IRQ for secondary MC6809E
reg sirq = 1;
always_ff @(posedge clk_49m) begin
	if(cen_6m) begin
		if(!sirq_mask)
			sirq <= 1;
		else if(vblank_irq_en)
			sirq <= 0;
	end
end

//Shared RAM for the two MC6809E CPUs
wire [7:0] m_sharedram_D, s_sharedram_D;

// Hiscore mux
wire [10:0] u9F_addr = hs_access ? hs_address[10:0] : sA[10:0];
wire [7:0] u9F_din = hs_access ? hs_data_in : sD_out;
wire u9F_wren = hs_access ? hs_write : (cs_ssharedram & ~s_rw);

wire [7:0] u9F_dout;
assign s_sharedram_D = hs_access ? 8'h00 : u9F_dout;
assign hs_data_out = hs_access ? u9F_dout : 8'h00;

dpram_dc #(.widthad_a(11)) u9F
(
	.clock_a(clk_49m),
	.wren_a(cs_msharedram & ~m_rw),
	.address_a(mA[10:0]),
	.data_a(mD_out),
	.q_a(m_sharedram_D),
	
	.clock_b(clk_49m),
	.wren_b(u9F_wren),
	.address_b(u9F_addr),
	.data_b(u9F_din),
	.q_b(u9F_dout)
);

//-------------------------------------------------------- Video timing --------------------------------------------------------//

//Konami 082 custom chip - responsible for all video timings
wire vblk, vblank_irq_en;
wire [8:0] h_cnt;
wire [7:0] v_cnt;
k082 u6A
(
	.reset(1),
	.clk(clk_49m),
	.cen(cen_6m),
	.h_center(h_center),
	.v_center(v_center),
	.n_vsync(video_vsync),
	.sync(video_csync),
	.n_hsync(video_hsync),
	.vblk(vblk),
	.vblk_irq_en(vblank_irq_en),
	.h1(h_cnt[0]),
	.h2(h_cnt[1]),
	.h4(h_cnt[2]),
	.h8(h_cnt[3]),
	.h16(h_cnt[4]),
	.h32(h_cnt[5]),
	.h64(h_cnt[6]),
	.h128(h_cnt[7]),
	.n_h256(h_cnt[8]),
	.v1(v_cnt[0]),
	.v2(v_cnt[1]),
	.v4(v_cnt[2]),
	.v8(v_cnt[3]),
	.v16(v_cnt[4]),
	.v32(v_cnt[5]),
	.v64(v_cnt[6]),
	.v128(v_cnt[7])
);

//Latch vertical counter from 082 custom chip on the rising edge of horizontal counter bit 8
reg [7:0] vcnt_lat = 8'd0;
reg old_n_h256;
always_ff @(posedge clk_49m) begin
	old_n_h256 <= h_cnt[8];
	if(!old_n_h256 && h_cnt[8])
		vcnt_lat <= v_cnt;
end

//Latch least significant bit of horizontal counter (to be used for sprite RAM logic)
reg h1d;
always_ff @(posedge clk_49m) begin
	if(n_cen_6m)
		h1d <= h_cnt[0];
end

//XOR horizontal counter bits [7:2] with HFLIP flag
wire [7:2] hcnt_x = h_cnt[7:2] ^ {6{hflip}};

//XOR latched vertical counter bits with VFLIP flag
wire [7:0] vcnt_x = vcnt_lat ^ {8{vflip}};

//--------------------------------------------------------- Tile layer ---------------------------------------------------------//

//Latch X and Y scroll registers
reg [7:0] xscroll_reg = 8'd0;
reg [7:0] yscroll_reg = 8'd0;
always_ff @(posedge clk_49m) begin
	if(cen_3m) begin
		if(cs_xscrollreg)
			xscroll_reg <= mD_out;
		if(cs_yscrollreg)
			yscroll_reg <= mD_out;
	end
end
wire [7:0] L = n_scroll_en ? 8'hFF : yscroll_reg;
wire [7:2] J = n_scroll_en ? 6'b111111 : xscroll_reg[7:2];
wire shf1 = n_scroll_en ? 1'b1 : xscroll_reg[1];
wire shf0 = n_scroll_en ? 1'b1 : xscroll_reg[0];

//Generate tilemap horizontal and vertical position by summing X and Y scroll registers with their respective horizontal
//and vertical counter bits
wire [7:2] ha = n_scroll_en + hcnt_x + J;
wire [7:0] va = n_scroll_en + vcnt_x + L;

//Generate addresses for VRAM and color RAM
wire [10:0] vram_cram_A = {n_scroll_en, va[7:3], ha[7:3]};

//VRAM
wire [7:0] vram_D, vram_Dout;
dpram_dc #(.widthad_a(11)) u5F
(
	.clock_a(clk_49m),
	.wren_a(cs_vram & ~m_rw),
	.address_a(mA[10:0]),
	.data_a(mD_out),
	.q_a(vram_Dout),
	
	.clock_b(clk_49m),
	.address_b(vram_cram_A),
	.q_b(vram_D)
);

//Color RAM
wire [7:0] cram_D, cram_Dout;
dpram_dc #(.widthad_a(11)) u4F
(
	.clock_a(clk_49m),
	.wren_a(cs_cram & ~m_rw),
	.address_a(mA[10:0]),
	.data_a(mD_out),
	.q_a(cram_Dout),
	
	.clock_b(clk_49m),
	.address_b(vram_cram_A),
	.q_b(cram_D)
);

//Latch and shift tilemap code and attributes from VRAM and color RAM
//Time Pilot '84 has two tilemap layers - shift data out at 1/4 of the pixel clock on the rising edge to handle both layers
reg [15:0] tile_code = 16'd0;
reg [15:0] tile_attrib = 16'd0;
always_ff @(posedge clk_49m) begin
	if(cen_1m5) begin
		tile_code <= {vram_D, tile_code[15:8]};
		tile_attrib <= {cram_D, tile_attrib[15:8]};
	end
end

//Latch tile color information and flip signal for tilemap 083 custom chip every 4 pixels
wire tile_flip = tile_hflip ^ ~hflip;
reg tile_083_flip = 0;
reg [3:0] tile_color = 4'd0;
always_ff @(posedge clk_49m) begin
	if(cen_6m) begin
		if(h_cnt[1:0] == 2'b11) begin
			tile_083_flip <= tile_flip;
			tile_color <= tile_attrib[3:0];
		end
		else begin
			tile_083_flip <= tile_083_flip;
			tile_color <= tile_color;
		end
	end
end

//Generate lower 4 address lines of tilemap ROMs when the lower 2 bits of the horizontal counter are both set to 1
reg va1l, va2l, va4l, ha2l;
always_ff @(posedge clk_49m) begin
	if(cen_6m && h_cnt[1:0] == 2'b11) begin
		va1l <= va[0];
		va2l <= va[1];
		va4l <= va[2];
		ha2l <= ha[2];
	end
end

//Address tilemap ROMs and assign tile flip attributes
assign tilerom_A[12] = tile_attrib[4];
assign tilerom_A[11:4] = tile_code[7:0];
assign tilerom_A[3:0] = {ha2l ^ tile_hflip, va4l ^ tile_vflip, va2l ^ tile_vflip, va1l ^ tile_vflip};
wire tilerom_sel = tile_attrib[5];
wire tile_hflip = tile_attrib[6];
wire tile_vflip = tile_attrib[7];

//Tilemap ROMs
wire [12:0] tilerom_A;
//ROM 1/2
wire [7:0] eprom7_D;
eprom_7 u2J
(
	.ADDR(tilerom_A),
	.CLK(clk_49m),
	.DATA(eprom7_D),
	.ADDR_DL(ioctl_addr),
	.CLK_DL(clk_49m),
	.DATA_IN(ioctl_data),
	.CS_DL(ep7_cs_i),
	.WR(ioctl_wr)
);
//ROM 2/2
wire [7:0] eprom8_D;
eprom_8 u1J
(
	.ADDR(tilerom_A),
	.CLK(clk_49m),
	.DATA(eprom8_D),
	.ADDR_DL(ioctl_addr),
	.CLK_DL(clk_49m),
	.DATA_IN(ioctl_data),
	.CS_DL(ep8_cs_i),
	.WR(ioctl_wr)
);

//Multiplex tilemap ROM data outputs
wire [7:0] tilerom_D = tilerom_sel ? eprom8_D : eprom7_D;

//Konami 083 custom chip 1/2 - this one shifts the pixel data from tilemap ROMs
k083 u1G
(
	.CK(clk_49m),
	.CEN(cen_6m),
	.LOAD(h_cnt[1:0] == 2'b11),
	.FLIP(tile_083_flip),
	.DB0i(tilerom_D),
	.DSH0(tile_lut_A[1:0])
);

//Tilemap lookup PROM
wire [7:0] tile_lut_A;
assign tile_lut_A[7:6] = color_latch[4:3];
assign tile_lut_A[5:2] = tile_color;
wire [3:0] tile_lut_D;
tile_lut_prom u1F
(
	.ADDR(tile_lut_A),
	.CLK(clk_49m),
	.DATA(tile_lut_D),
	.ADDR_DL(ioctl_addr),
	.CLK_DL(clk_49m),
	.DATA_IN(ioctl_data),
	.CS_DL(tl_cs_i),
	.WR(ioctl_wr)
);

//Shift data out of tilemap lookup PROM at every rising edge of the pixel clock
reg [15:0] tilemap_Dshift;
always_ff @(posedge clk_49m) begin
	if(cen_6m)
		tilemap_Dshift <= {tilemap_Dshift[11:0], tile_lut_D};
end
wire [3:0] S = tilemap_Dshift[3:0];
wire [3:0] SS = tilemap_Dshift[7:4];
wire [3:0] SH = tilemap_Dshift[11:8];
wire [3:0] SF = tilemap_Dshift[15:12];

//Signal to the video hardware when to draw the HUD - these signals are active when the horizontal counter is above 504 and below
//138 for the bottom HUD, and above 507 and below 284 for the top HUD.
wire bottom_hud_en = (h_cnt > 504 || h_cnt < 138);
wire top_hud_en = (h_cnt > 507 || h_cnt < 284);

//Signal to the video hardware when to draw the playfield and allow scrolling - this signal is active when the horizontal counter
//is between 272 and 495
wire n_scroll_en = ~(h_cnt > 271 && h_cnt < 496);

//Generate tilemap data select lines
wire tile_sel0 = (~(shf0_l | top_hud_en) | bottom_hud_en);
wire tile_sel1 = (~(shf1_l | top_hud_en) | bottom_hud_en);

//Multiplex tile data
wire [3:0] tile_D = ({tile_sel1, tile_sel0} == 2'b00) ? SF:
                    ({tile_sel1, tile_sel0} == 2'b01) ? SH:
                    ({tile_sel1, tile_sel0} == 2'b10) ? SS:
                    ({tile_sel1, tile_sel0} == 2'b11) ? S:
                    4'h0;

//-------------------------------------------------------- Sprite layer --------------------------------------------------------//

//Sprite RAM read/write decoding logic
wire spriteram_en = (~h_cnt[1] & cs_spriteram);
wire spriteram_gate = (spriteram_wr | s_rw);
wire cs_spriteram0 = (spriteram_en & sA[1] & spriteram_gate);
wire cs_spriteram1 = (spriteram_en & ~sA[1] & spriteram_gate);
wire spriteram_wr = ~(h1d | s_rw);
wire spriteram0_we = (spriteram_en & sA[1] & spriteram_wr);
wire spriteram1_we = (spriteram_en & ~sA[1] & spriteram_wr);

//Sprite RAM
wire [15:0] spriteram_D;
wire [9:0] spriteram_A;
assign spriteram_A[9:8] = {(h_cnt[1] | sA[10]), (h_cnt[1] | sA[9])};
assign spriteram_A[7:0] = h_cnt[1] ? {2'b11, h_cnt[7], (h_cnt[8] ^ h_cnt[7]), h_cnt[6:4], h_cnt[2]} : {sA[8:2], sA[0]};
//Bank 0 (lower 4 bits)
spram #(4, 10) u9B
(
	.clk(clk_49m),
	.we(cs_spriteram0 & spriteram0_we),
	.addr(spriteram_A),
	.data(sD_out[3:0]),
	.q(spriteram_D[3:0])
);
//Bank 0 (upper 4 bits)
spram #(4, 10) u8B
(
	.clk(clk_49m),
	.we(cs_spriteram0 & spriteram0_we),
	.addr(spriteram_A),
	.data(sD_out[7:4]),
	.q(spriteram_D[7:4])
);
//Bank 1 (lower 4 bits)
spram #(4, 10) u9C
(
	.clk(clk_49m),
	.we(cs_spriteram1 & spriteram1_we),
	.addr(spriteram_A),
	.data(sD_out[3:0]),
	.q(spriteram_D[11:8])
);
//Bank 1 (upper 4 bits)
spram #(4, 10) u8C
(
	.clk(clk_49m),
	.we(cs_spriteram1 & spriteram1_we),
	.addr(spriteram_A),
	.data(sD_out[7:4]),
	.q(spriteram_D[15:12])
);

//Konami 503 custom chip - generates sprite addresses for lower half of sprite ROMs, sprite line buffer control, enable for
//sprite write and sprite flip for 083 custom chip.
wire cs_linebuffer, sprite_flip, n_cara, n_ocoll;
k503 u11A
(
	.CLK(clk_49m),
	.OB(spriteram_D[7:0]),
	.VCNT(vcnt_lat),
	.H4(h_cnt[2]),
	.H8(h_cnt[3]),
	.LD(h_cnt[1:0] != 2'b11),
	.OCS(cs_linebuffer),
	.OFLP(sprite_flip),
	.ODAT(n_cara),
	.OCOL(n_ocoll),
	.R(spriterom_A[5:0])
);

//Latch sprite code from sprite RAM bank 1 every 8 pixels
reg [7:0] sprite_code = 8'd0;
always_ff @(posedge clk_49m) begin
	if(cen_6m) begin
		if(h_cnt[3:0] == 4'b0111)
			sprite_code <= spriteram_D[15:8];
		else
			sprite_code <= sprite_code;
	end
end

//Assign sprite code to address the upper 7 bits of the sprite ROMs
assign spriterom_A[12:6] = sprite_code[6:0];

//Sprite ROMs
wire [12:0] spriterom_A;
//ROM 1/4
wire [7:0] eprom9_D;
eprom_9 u12A
(
	.ADDR(spriterom_A),
	.CLK(clk_49m),
	.DATA(eprom9_D),
	.ADDR_DL(ioctl_addr),
	.CLK_DL(clk_49m),
	.DATA_IN(ioctl_data),
	.CS_DL(ep9_cs_i),
	.WR(ioctl_wr)
);
//ROM 2/4
wire [7:0] eprom10_D;
eprom_10 u13A
(
	.ADDR(spriterom_A),
	.CLK(clk_49m),
	.DATA(eprom10_D),
	.ADDR_DL(ioctl_addr),
	.CLK_DL(clk_49m),
	.DATA_IN(ioctl_data),
	.CS_DL(ep10_cs_i),
	.WR(ioctl_wr)
);
//ROM 3/4
wire [7:0] eprom11_D;
eprom_11 u14A
(
	.ADDR(spriterom_A),
	.CLK(clk_49m),
	.DATA(eprom11_D),
	.ADDR_DL(ioctl_addr),
	.CLK_DL(clk_49m),
	.DATA_IN(ioctl_data),
	.CS_DL(ep11_cs_i),
	.WR(ioctl_wr)
);
//ROM 4/4
wire [7:0] eprom12_D;
eprom_12 u15A
(
	.ADDR(spriterom_A),
	.CLK(clk_49m),
	.DATA(eprom12_D),
	.ADDR_DL(ioctl_addr),
	.CLK_DL(clk_49m),
	.DATA_IN(ioctl_data),
	.CS_DL(ep12_cs_i),
	.WR(ioctl_wr)
);

//Multiplex sprite ROM data outputs based on the state of the most significant bit of the sprite code
wire spriterom_sel = sprite_code[7];
wire [15:0] spriterom_D = spriterom_sel ? {eprom12_D, eprom10_D} : {eprom11_D, eprom9_D};

//Konami 083 custom chip 2/2 - shifts the pixel data from sprite ROMs
k083 u16A
(
	.CK(clk_49m),
	.CEN(cen_6m),
	.LOAD(h_cnt[1:0] == 2'b11),
	.FLIP(sprite_k083_flip),
	.DB0i(spriterom_D[7:0]),
	.DB1i(spriterom_D[15:8]),
	.DSH0(sprite_lut_A[1:0]),
	.DSH1(sprite_lut_A[3:2])
);

//Latch sprite color information, enable for sprite line buffer, XORed SHFx signals, sprite flip flag at every
//12 pixels
wire shf0_flip = (shf0 ^ ~hflip);
wire shf1_flip = (shf1 ^ ~hflip);
reg [3:0] sprite_color = 4'd0;
reg shf0_l, shf1_l, sprite_lbuff_en, sprite_k083_flip;
always_ff @(posedge clk_49m) begin
	if(cen_6m) begin
		if(h_cnt[3:0] == 4'b1011) begin
			sprite_color <= spriteram_D[3:0];
			shf0_l <= shf0_flip;
			shf1_l <= shf1_flip;
			sprite_lbuff_en <= cs_linebuffer;
			sprite_k083_flip <= sprite_flip;
		end
		else begin
			sprite_color <= sprite_color;
			shf0_l <= shf0_l;
			shf1_l <= shf1_l;
			sprite_lbuff_en <= sprite_lbuff_en;
			sprite_k083_flip <= sprite_k083_flip;
		end
	end
end

//Assign sprite color information to the upper 4 bits of the sprite lookup PROM
assign sprite_lut_A[7:4] = sprite_color;

//Sprite lookup PROM
wire [7:0] sprite_lut_A;
wire [3:0] sprite_lut_D;
sprite_lut_prom u16C
(
	.ADDR(sprite_lut_A),
	.CLK(clk_49m),
	.DATA(sprite_lut_D),
	.ADDR_DL(ioctl_addr),
	.CLK_DL(clk_49m),
	.DATA_IN(ioctl_data),
	.CS_DL(sl_cs_i),
	.WR(ioctl_wr)
);

//Konami 502 custom chip, responsible for generating sprites (sits between sprite ROMs and the sprite line buffer)
wire [7:0] sprite_lbuff_Do;
wire [4:0] sprite_D;
wire sprite_lbuff_sel, sprite_lbuff_dec0, sprite_lbuff_dec1;
k502 u15D
(
	.RESET(1),
	.CK1(clk_49m),
	.CK2(clk_49m),
	.CEN(cen_6m),
	.LDO(h_cnt[2:0] != 3'b111),
	.H2(h_cnt[1]),
	.H256(h_cnt[8]),
	.SPAL(sprite_lut_D),
	.SPLBi({sprite_lbuff1_D, sprite_lbuff0_D}),
	.SPLBo(sprite_lbuff_Do),
	.OSEL(sprite_lbuff_sel),
	.OLD(sprite_lbuff_dec1),
	.OCLR(sprite_lbuff_dec0),
	.COL(sprite_D)
);

//----------------------------------------------------- Sprite line buffer -----------------------------------------------------//

//Generate load and clear signals for counters generating addresses to sprite line buffer
reg sprite_lbuff0_ld, sprite_lbuff1_ld, sprite_lbuff0_clr, sprite_lbuff1_clr;
always_ff @(posedge clk_49m) begin
	if(h_cnt[1:0] == 2'b11) begin
		if(sprite_lbuff_dec0 && !sprite_lbuff_dec1) begin
			sprite_lbuff0_clr <= 0;
			sprite_lbuff1_clr <= 1;
		end
		else if(!sprite_lbuff_dec0 && sprite_lbuff_dec1) begin
			sprite_lbuff0_clr <= 1;
			sprite_lbuff1_clr <= 0;
		end
		else begin
			sprite_lbuff0_clr <= 0;
			sprite_lbuff1_clr <= 0;
		end
	end
	else begin
		sprite_lbuff0_clr <= 0;
		sprite_lbuff1_clr <= 0;
	end
	if(h_cnt[3:0] == 4'b1011) begin
		if(!sprite_lbuff_dec1) begin
			sprite_lbuff0_ld <= 1;
			sprite_lbuff1_ld <= 0;
		end
		else begin
			sprite_lbuff0_ld <= 0;
			sprite_lbuff1_ld <= 1;
		end
	end
	else begin
		sprite_lbuff0_ld <= 0;
		sprite_lbuff1_ld <= 0;
	end
end

//Generate addresses for sprite line buffer
//Bank 0, lower 4 bits
reg [3:0] linebuffer0_l = 4'd0;
always_ff @(posedge clk_49m) begin
	if(cen_6m) begin
		if(sprite_lbuff0_clr)
			linebuffer0_l <= 4'd0;
		else
			if(sprite_lbuff0_ld)
				linebuffer0_l <= spriteram_D[11:8];
			else
				linebuffer0_l <= linebuffer0_l + 4'd1;
	end
end
//Bank 0, upper 4 bits
reg [3:0] linebuffer0_h = 4'd0;
always_ff @(posedge clk_49m) begin
	if(cen_6m) begin
		if(sprite_lbuff0_clr)
			linebuffer0_h <= 4'd0;
		else
			if(sprite_lbuff0_ld)
				linebuffer0_h <= spriteram_D[15:12];
			else if(linebuffer0_l == 4'hF)
				linebuffer0_h <= linebuffer0_h + 4'd1;
	end
end
wire [7:0] sprite_lbuff0_A = {linebuffer0_h, linebuffer0_l};
//Bank 1, lower 4 bits
reg [3:0] linebuffer1_l = 4'd0;
always_ff @(posedge clk_49m) begin
	if(cen_6m) begin
		if(sprite_lbuff1_clr)
			linebuffer1_l <= 4'd0;
		else
			if(sprite_lbuff1_ld)
				linebuffer1_l <= spriteram_D[11:8];
			else
				linebuffer1_l <= linebuffer1_l + 4'd1;
	end
end
//Bank 1, upper 4 bits
reg [3:0] linebuffer1_h = 4'd0;
always_ff @(posedge clk_49m) begin
	if(cen_6m) begin
		if(sprite_lbuff1_clr)
			linebuffer1_h <= 4'd0;
		else
			if(sprite_lbuff1_ld)
				linebuffer1_h <= spriteram_D[15:12];
			else if(linebuffer1_l == 4'hF)
				linebuffer1_h <= linebuffer1_h + 4'd1;
	end
end
wire [7:0] sprite_lbuff1_A = {linebuffer1_h, linebuffer1_l};

//Generate chip select signals for sprite line buffer
wire cs_sprite_lbuff0 = ~(sprite_lbuff_en & sprite_lbuff_sel);
wire cs_sprite_lbuff1 = ~(sprite_lbuff_en & ~sprite_lbuff_sel);

//Sprite line buffer bank 0
wire [3:0] sprite_lbuff0_D;
spram #(4, 8) u13D
(
	.clk(clk_49m),
	.we(cen_6m & cs_sprite_lbuff0),
	.addr(sprite_lbuff0_A),
	.data(sprite_lbuff_Do[3:0]),
	.q(sprite_lbuff0_D)
);

//Sprite line buffer bank 1
wire [3:0] sprite_lbuff1_D;
spram #(4, 8) u14D
(
	.clk(clk_49m),
	.we(cen_6m & cs_sprite_lbuff1),
	.addr(sprite_lbuff1_A),
	.data(sprite_lbuff_Do[7:4]),
	.q(sprite_lbuff1_D)
);

//----------------------------------------------------- Final video output -----------------------------------------------------//

//Generate HBlank (active high) while the horizontal counter is between 139 and 269
wire hblk = (h_cnt > 137 && h_cnt < 269);

//Multiplex tile and sprite data
wire tile_sprite_sel = ((bottom_hud_en | top_hud_en) | sprite_D[4]);
wire [3:0] tile_sprite_D = tile_sprite_sel ? tile_D[3:0] : sprite_D[3:0];

//Latch pixel data for color PROMs
reg [4:0] pixel_D;
always_ff @(posedge clk_49m) begin
	if(cen_6m)
		pixel_D <= {tile_sprite_sel, tile_sprite_D};
end

//Color PROMs
wire [7:0] color_A = {pixel_D[4], color_latch[2:0], pixel_D[3:0]};
//Red
wire [3:0] prom_red;
color_prom_1 u2C
(
	.ADDR(color_A),
	.CLK(clk_49m),
	.DATA(prom_red),
	.ADDR_DL(ioctl_addr),
	.CLK_DL(clk_49m),
	.DATA_IN(ioctl_data),
	.CS_DL(cp1_cs_i),
	.WR(ioctl_wr)
);
//Green
wire [3:0] prom_green;
color_prom_2 u2D
(
	.ADDR(color_A),
	.CLK(clk_49m),
	.DATA(prom_green),
	.ADDR_DL(ioctl_addr),
	.CLK_DL(clk_49m),
	.DATA_IN(ioctl_data),
	.CS_DL(cp2_cs_i),
	.WR(ioctl_wr)
);
//Blue
wire [3:0] prom_blue;
color_prom_3 u1E
(
	.ADDR(color_A),
	.CLK(clk_49m),
	.DATA(prom_blue),
	.ADDR_DL(ioctl_addr),
	.CLK_DL(clk_49m),
	.DATA_IN(ioctl_data),
	.CS_DL(cp3_cs_i),
	.WR(ioctl_wr)
);

//Output video signal from color PROMs, otherwise output black if in HBlank or VBlank
assign red = (hblk | vblk) ? 4'h0 : prom_red;
assign green = (hblk | vblk) ? 4'h0 : prom_green;
assign blue = (hblk | vblk) ? 4'h0 : prom_blue;

endmodule
