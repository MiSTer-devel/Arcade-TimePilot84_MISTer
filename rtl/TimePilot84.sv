//============================================================================
// 
//  Time Pilot '84 top-level module
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
module TimePilot84
(
	input                reset,
	input                clk_49m,                  //Actual frequency: 49.152MHz
	input          [1:0] coin,                     //0 = coin 1, 1 = coin 2
	input          [1:0] start_buttons,            //0 = Player 1, 1 = Player 2
	input          [3:0] p1_joystick, p2_joystick, //0 = up, 1 = down, 2 = left, 3 = right
	input          [2:0] p1_buttons,               //0 = shot, 1 = missile, 2 = spare
	input          [1:0] p2_buttons,               //0 = shot, 1 = missile
	input                btn_service,
	input         [15:0] dip_sw,
	output               video_hsync, video_vsync, video_csync,
	output               video_hblank, video_vblank,
	output               ce_pix,
	output         [3:0] video_r, video_g, video_b,
	output signed [15:0] sound,
	
	input                is_set3, //Flag to remap primary CPU address space for Time Pilot '84 (Set 3)
	
	//Screen centering (alters HSync, VSync and VBlank timing in the Konami 082 to reposition the video output)
	input          [3:0] h_center, v_center,
	
	input         [24:0] ioctl_addr,
	input          [7:0] ioctl_data,
	input                ioctl_wr,

	input                pause,
	
	//This input serves to select different fractional dividers to acheive 3.579545MHz for the Z80 and 1.789772MHz for the
	//SN76489s depending on whether Time Pilot '84 runs with original or underclocked timings to normalize sync frequencies
	input                underclock,

	input         [15:0] hs_address,
	input          [7:0] hs_data_in,
	output         [7:0] hs_data_out,
	input                hs_write,
	input                hs_access
);

//Linking signals between PCBs
wire A5, A6, irq_trigger, cs_sounddata, cs_controls_dip1, cs_dip2;
wire [7:0] controls_dip, cpubrd_D;

//ROM loader signals for MISTer (loads ROMs from SD card)
wire ep1_cs_i, ep2_cs_i, ep3_cs_i, ep4_cs_i, ep5_cs_i, ep6_cs_i, ep7_cs_i, ep8_cs_i,
     ep9_cs_i, ep10_cs_i, ep11_cs_i, ep12_cs_i;
wire cp1_cs_i, cp2_cs_i, cp3_cs_i, tl_cs_i, sl_cs_i;

//MiSTer data write selector
selector DLSEL
(
	.ioctl_addr(ioctl_addr),
	.ep1_cs(ep1_cs_i),
	.ep2_cs(ep2_cs_i),
	.ep3_cs(ep3_cs_i),
	.ep4_cs(ep4_cs_i),
	.ep5_cs(ep5_cs_i),
	.ep6_cs(ep6_cs_i),
	.ep7_cs(ep7_cs_i),
	.ep8_cs(ep8_cs_i),
	.ep9_cs(ep9_cs_i),
	.ep10_cs(ep10_cs_i),
	.ep11_cs(ep11_cs_i),
	.ep12_cs(ep12_cs_i),
	.cp1_cs(cp1_cs_i),
	.cp2_cs(cp2_cs_i),
	.cp3_cs(cp3_cs_i),
	.tl_cs(tl_cs_i),
	.sl_cs(sl_cs_i)
);

//Instantiate main PCB
TimePilot84_CPU main_pcb
(
	.reset(reset),
	.clk_49m(clk_49m),
	.red(video_r),
	.green(video_g),
	.blue(video_b),
	.video_hsync(video_hsync),
	.video_vsync(video_vsync),
	.video_csync(video_csync),
	.video_hblank(video_hblank),
	.video_vblank(video_vblank),
	.ce_pix(ce_pix),
	
	.h_center(h_center),
	.v_center(v_center),
	
	.controls_dip(controls_dip),
	.cpubrd_Dout(cpubrd_D),
	.cpubrd_A5(A5),
	.cpubrd_A6(A6),
	.cs_sounddata(cs_sounddata),
	.irq_trigger(irq_trigger),
	.cs_dip2(cs_dip2),
	.cs_controls_dip1(cs_controls_dip1),
	
	.is_set3(is_set3),
	
	.ep1_cs_i(ep1_cs_i),
	.ep2_cs_i(ep2_cs_i),
	.ep3_cs_i(ep3_cs_i),
	.ep4_cs_i(ep4_cs_i),
	.ep5_cs_i(ep5_cs_i),
	.ep7_cs_i(ep7_cs_i),
	.ep8_cs_i(ep8_cs_i),
	.ep9_cs_i(ep9_cs_i),
	.ep10_cs_i(ep10_cs_i),
	.ep11_cs_i(ep11_cs_i),
	.ep12_cs_i(ep12_cs_i),
	.cp1_cs_i(cp1_cs_i),
	.cp2_cs_i(cp2_cs_i),
	.cp3_cs_i(cp3_cs_i),
	.tl_cs_i(tl_cs_i),
	.sl_cs_i(sl_cs_i),
	.ioctl_addr(ioctl_addr),
	.ioctl_wr(ioctl_wr),
	.ioctl_data(ioctl_data),
	.pause(pause),
	.hs_address(hs_address),
	.hs_data_in(hs_data_in),
	.hs_data_out(hs_data_out),
	.hs_write(hs_write),
	.hs_access(hs_access)
);

//Instantiate sound PCB
TimePilot84_SND sound_pcb
(
	.reset(reset),
	.clk_49m(clk_49m),
	.irq_trigger(irq_trigger),
	.cs_sounddata(cs_sounddata),
	.dip_sw(dip_sw),
	.coin(coin),
	.start_buttons(start_buttons),
	.p1_joystick(p1_joystick),
	.p2_joystick(p2_joystick),
	.p1_buttons(p1_buttons),
	.p2_buttons(p2_buttons),
	.btn_service(btn_service),
	
	.cs_controls_dip1(cs_controls_dip1),
	.cs_dip2(cs_dip2),
	.cpubrd_A5(A5),
	.cpubrd_A6(A6),
	.cpubrd_Din(cpubrd_D),	
	.controls_dip(controls_dip),
	.sound(sound),
	
	.underclock(underclock),
	
	.ep6_cs_i(ep6_cs_i),
	.ioctl_addr(ioctl_addr),
	.ioctl_wr(ioctl_wr),
	.ioctl_data(ioctl_data)
);

endmodule
