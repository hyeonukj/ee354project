`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// VGA verilog template
// Author:  Da Cheng
//////////////////////////////////////////////////////////////////////////////////
module vga_demo(ClkPort, vga_h_sync, vga_v_sync, vga_r, vga_g, vga_b, Sw0, Sw1, btnU, btnR, btnL, btnD, btnC,
	St_ce_bar, St_rp_bar, Mt_ce_bar, Mt_St_oe_bar, Mt_St_we_bar,
	An0, An1, An2, An3, Ca, Cb, Cc, Cd, Ce, Cf, Cg, Dp,
	LD0, LD1, LD2, LD3, LD4, LD5, LD6, LD7);
	input ClkPort, Sw0, Sw1, btnU, btnL, btnR, btnD, btnC;
	output St_ce_bar, St_rp_bar, Mt_ce_bar, Mt_St_oe_bar, Mt_St_we_bar;
	output vga_h_sync, vga_v_sync, vga_r, vga_g, vga_b;
	output An0, An1, An2, An3, Ca, Cb, Cc, Cd, Ce, Cf, Cg, Dp;
	output LD0, LD1, LD2, LD3, LD4, LD5, LD6, LD7;
	reg vga_r, vga_g, vga_b;
	
	//////////////////////////////////////////////////////////////////////////////////////////
	
	/*  LOCAL SIGNALS */
	wire	reset, start, ClkPort, board_clk, clk, button_clk;
	
	BUF BUF1 (board_clk, ClkPort); 	
	BUF BUF2 (reset, Sw0);
	BUF BUF3 (start, Sw1);
	
	reg [27:0]	DIV_CLK;
	always @ (posedge board_clk, posedge reset)  
	begin : CLOCK_DIVIDER
      if (reset)
			DIV_CLK <= 0;
      else
			DIV_CLK <= DIV_CLK + 1'b1;
	end	

	assign	button_clk = DIV_CLK[18];
	assign	clk = DIV_CLK[1];
	assign  note_clk = DIV_CLK[25];
	assign 	{St_ce_bar, St_rp_bar, Mt_ce_bar, Mt_St_oe_bar, Mt_St_we_bar} = {5'b11111};
	
	wire inDisplayArea;
	wire [9:0] CounterX;
	wire [9:0] CounterY;

	hvsync_generator syncgen(.clk(clk), .reset(reset),.vga_h_sync(vga_h_sync), .vga_v_sync(vga_v_sync), .inDisplayArea(inDisplayArea), .CounterX(CounterX), .CounterY(CounterY));
	
	/////////////////////////////////////////////////////////////////
	///////////////		VGA control starts here		/////////////////
	/////////////////////////////////////////////////////////////////
	reg [9:0] position [0:14];
	reg [2:0] notes [0:202];
	reg flag [0:14];
	reg scoreFlag [0:2];
	initial begin
	notes[0] = 3'b000;
	notes[1] = 3'b001;
	notes[2] = 3'b010;
	notes[3] = 3'b100;
	notes[4] = 3'b001;
	notes[5] = 3'b010;
	notes[6] = 3'b100;
	notes[7] = 3'b001;
	notes[8] = 3'b010;
	notes[9] = 3'b100;
	notes[10] = 3'b010;
	notes[11] = 3'b100;
	notes[12] = 3'b010;
	notes[13] = 3'b001;
	notes[14] = 3'b001;
	notes[15] = 3'b100;
	notes[16] = 3'b000;
	notes[17] = 3'b000;
	notes[18] = 3'b100;
	notes[19] = 3'b000;
	notes[20] = 3'b010;
	notes[21] = 3'b000;
	notes[22] = 3'b001;
	notes[23] = 3'b010;
	notes[24] = 3'b100;
	notes[25] = 3'b010;
	notes[26] = 3'b000;
	notes[27] = 3'b001;
	notes[28] = 3'b000;
	notes[29] = 3'b010;
	notes[30] = 3'b001;
	notes[31] = 3'b010;
	notes[32] = 3'b000;
	notes[33] = 3'b100;
	notes[34] = 3'b000;
	notes[35] = 3'b001;
	notes[36] = 3'b000;
	notes[37] = 3'b001;
	notes[38] = 3'b000;
	notes[39] = 3'b000;
	notes[40] = 3'b000;
	notes[41] = 3'b010;
	notes[42] = 3'b000;
	notes[43] = 3'b100;
	notes[44] = 3'b010;
	notes[45] = 3'b010;
	notes[46] = 3'b001;
	notes[47] = 3'b000;
	notes[48] = 3'b100;
	notes[49] = 3'b000;
	notes[50] = 3'b000;
	notes[51] = 3'b100;
	notes[52] = 3'b100;
	notes[53] = 3'b001;
	notes[54] = 3'b001;
	notes[55] = 3'b010;
	notes[56] = 3'b010;
	notes[57] = 3'b000;
	notes[58] = 3'b001;
	notes[59] = 3'b000;
	notes[60] = 3'b100;
	notes[61] = 3'b010;
	notes[62] = 3'b010;
	notes[63] = 3'b000;
	notes[64] = 3'b000;
	notes[65] = 3'b010;
	notes[66] = 3'b100;
	notes[67] = 3'b010;
	notes[68] = 3'b001;
	notes[69] = 3'b010;
	notes[70] = 3'b000;
	notes[71] = 3'b100;
	notes[72] = 3'b001;
	notes[73] = 3'b010;
	notes[74] = 3'b001;
	notes[75] = 3'b000;
	notes[76] = 3'b000;
	notes[77] = 3'b001;
	notes[78] = 3'b010;
	notes[79] = 3'b001;
	notes[80] = 3'b000;
	notes[81] = 3'b000;
	notes[82] = 3'b000;
	notes[83] = 3'b010;
	notes[84] = 3'b100;
	notes[85] = 3'b000;
	notes[86] = 3'b000;
	notes[87] = 3'b001;
	notes[88] = 3'b100;
	notes[89] = 3'b000;
	notes[90] = 3'b010;
	notes[91] = 3'b000;
	notes[92] = 3'b001;
	notes[93] = 3'b010;
	notes[94] = 3'b000;
	notes[95] = 3'b000;
	notes[96] = 3'b100;
	notes[97] = 3'b010;
	notes[98] = 3'b100;
	notes[99] = 3'b001;
	notes[100] = 3'b000;
	notes[101] = 3'b100;
	notes[102] = 3'b100;
	notes[103] = 3'b001;
	notes[104] = 3'b001;
	notes[105] = 3'b010;
	notes[106] = 3'b010;
	notes[107] = 3'b000;
	notes[108] = 3'b010;
	notes[109] = 3'b000;
	notes[110] = 3'b010;
	notes[111] = 3'b010;
	notes[112] = 3'b000;
	notes[113] = 3'b001;
	notes[114] = 3'b010;
	notes[115] = 3'b100;
	notes[116] = 3'b010;
	notes[117] = 3'b000;
	notes[118] = 3'b001;
	notes[119] = 3'b000;
	notes[120] = 3'b010;
	notes[121] = 3'b001;
	notes[122] = 3'b010;
	notes[123] = 3'b000;
	notes[124] = 3'b101;
	notes[125] = 3'b000;
	notes[126] = 3'b101;
	notes[127] = 3'b000;
	notes[128] = 3'b100;
	notes[129] = 3'b000;
	notes[130] = 3'b000;
	notes[131] = 3'b000;
	notes[132] = 3'b000;
	notes[133] = 3'b000;
	notes[134] = 3'b000;
	notes[135] = 3'b000;
	notes[136] = 3'b000;
	notes[137] = 3'b000;
	notes[138] = 3'b111;
	notes[139] = 3'b100;
	notes[140] = 3'b110;
	notes[141] = 3'b100;
	notes[142] = 3'b111;
	notes[143] = 3'b000;
	notes[144] = 3'b000;
	notes[145] = 3'b000;
	notes[146] = 3'b000;
	notes[147] = 3'b000;
	notes[148] = 3'b000;
	notes[149] = 3'b111;
	notes[150] = 3'b000;
	notes[151] = 3'b100;
	notes[152] = 3'b000;
	notes[153] = 3'b110;
	notes[154] = 3'b000;
	notes[155] = 3'b100;
	notes[156] = 3'b000;
	notes[157] = 3'b111;
	notes[158] = 3'b000;
	notes[159] = 3'b000;
	notes[160] = 3'b000;
	notes[161] = 3'b000;
	notes[162] = 3'b000;
	notes[163] = 3'b000;
	notes[164] = 3'b111;
	notes[165] = 3'b000;
	notes[166] = 3'b001;
	notes[167] = 3'b000;
	notes[168] = 3'b011;
	notes[169] = 3'b000;
	notes[170] = 3'b001;
	notes[171] = 3'b000;
	notes[172] = 3'b111;
	notes[173] = 3'b000;
	notes[174] = 3'b000;
	notes[175] = 3'b000;
	notes[176] = 3'b000;
	notes[177] = 3'b000;
	notes[178] = 3'b000;
	notes[179] = 3'b111;
	notes[180] = 3'b000;
	notes[181] = 3'b001;
	notes[182] = 3'b000;
	notes[183] = 3'b111;
	notes[184] = 3'b000;
	notes[185] = 3'b100;
	notes[186] = 3'b000;
	notes[187] = 3'b111;
	notes[188] = 3'b000;
	notes[189] = 3'b000;
	notes[190] = 3'b000;
	notes[191] = 3'b000;
	notes[192] = 3'b000;
	notes[193] = 3'b000;
	notes[194] = 3'b001;
	notes[195] = 3'b000;
	notes[196] = 3'b001;
	notes[197] = 3'b000;
	notes[198] = 3'b111;
	notes[199] = 3'b000;
	notes[200] = 3'b011;
	notes[201] = 3'b000;
	notes[202] = 3'b001;
		
	for (i = 0; i < 15; i = i+1)
		begin
			position[i] = 0;
			flag[i] = 0;	
		end
	
	end
	
	
	reg [8:0] counter;
	reg [5:0] posCount;
	reg [2:0] state;	
	reg [8:0] redBoxPos;
	reg [8:0] greenBoxPos;
	reg [8:0] blueBoxPos;
	reg [8:0] boxPos;
	reg [8:0] redTest;
	reg [5:0] scoreCounter[0:2];
	reg tempFlag;
	reg hitFlag[0:2];
	reg [15:0] score;
	wire R, G, B;
	integer i;
	
	assign	sys_clk = board_clk;
	
	localparam
	INITIAL = 3'b001,
	PLAY    = 3'b010,
	DONE    = 3'b100;

	always @(posedge DIV_CLK[19])
		begin
			if (reset)
				begin
					state <= INITIAL;
					posCount <= 0;
					counter <= 0;
					
					for(i=0;i<3;i=i+1)
						begin
							scoreFlag[i]<=0;
							scoreCounter[i]<=0;
						end
					for (i = 0; i < 15; i = i+1)
						begin
							position[i] <= 0;
							flag[i] <= 0;
						end
				end
			else
				begin
					case (state)
					INITIAL:
						begin							
							if (start == 1) 
								state <= PLAY;
								
							score <= 0;
						end
					PLAY:
						begin
							// when done displaying notes, go to DONE state
							// use 225 to make sure all notes off of screen
							if (counter >= 225)
								state <= DONE;
							
							// check if button is pressed in order to increment score
							if(btnL)
								begin
									redBoxPos <= 450;
								
								for (i = 0; i < 5; i = i+1)
									begin
										if ((position[i] >= 430) && (position[i] <= 470))
											begin
												if(scoreFlag[0]==0)
												begin
													score <= score + 1;
													scoreFlag[0]<=1;
												end
											end											
									end
								end
							else
								begin
									redBoxPos <= 0;
									scoreCounter[0]<=scoreCounter[0]+1;
									if(scoreCounter[0]==10)
										begin
											scoreFlag[0]<=0;
											scoreCounter[0]<=0;
										end
								end
								
							if(btnC)
								begin
									greenBoxPos <= 450;
									for (i = 5; i < 10; i = i+1)
										begin
											if ((position[i] >= 430) && (position[i] <= 470))
												begin
													if(scoreFlag[1]==0)
													begin
														score <= score + 1;
														scoreFlag[1]<=1;
													end
											end
										end
								end
							else
								begin
									greenBoxPos <= 0;
									scoreCounter[1]<=scoreCounter[1]+1;
									if(scoreCounter[1]==10)
										begin
											scoreFlag[1]<=0;
											scoreCounter[1]<=0;
										end
								end	
								
							if(btnR)
								begin
									blueBoxPos <= 450;
									for (i = 10; i < 15; i = i+1)
										begin
											if ((position[i] >= 420) && (position[i] <= 440))
												begin
											if(scoreFlag[2]==0)
											begin
												score <= score + 1;
												scoreFlag[2]<=1;
											end
										end
										end
								end
							else
								begin
									blueBoxPos <= 0;
									scoreCounter[2]<=scoreCounter[2]+1;
									if(scoreCounter[2]==10)
										begin
											scoreFlag[2]<=0;
											scoreCounter[2]<=0;
										end
								end
							// for moving the blocks down the screen 
							for (i = 0; i < 15; i = i+1)
								begin
									if (flag[i] == 1)
										position[i] <= position[i] + 1;				
									
									if (position[i] > 490)
										begin		
											position[i] <= 0;
											flag[i] <= 0;
										end
								end
						
							posCount <= posCount + 1;
															
							// posCount is the interval between the current note and the next note on the screen
							// increments continuously, goes back to 0 after overflow
							if(posCount == 0)
								begin
									counter <= counter + 1;
									
									// Red
									if (notes[counter][2] == 1)
										begin
											if ((flag[0] != 1) && (flag[1] != 1) && (flag[2] != 1)&& (flag[3] != 1)&& (flag[4] != 1)) // 000
												flag[0] <= 1;
											else if((flag[4]==1)&&(flag[0]!=1))
												flag[0] <=1;
											else if ((flag[1] != 1) && (flag[0] == 1))		// 001
												flag[1] <= 1;
											else if ((flag[1]==1)&& (flag[2] != 1))		// 011
												flag[2] <= 1;
											else if ((flag[2] == 1)&& (flag[3] != 1))		// 011
												flag[3] <= 1;
											else if ((flag[3] == 1)&& (flag[4] != 1))		// 011
												flag[4] <= 1;
										end
									// Green
									if (notes[counter][1] == 1)
										begin
											if ((flag[5] != 1) && (flag[6] != 1) && (flag[7] != 1)&& (flag[8] != 1)&& (flag[9] != 1)) // 000
												flag[5] <= 1;
											else if((flag[9]==1)&&(flag[5]!=1))
												flag[5] <=1;
											else if ((flag[6] != 1) && (flag[5] == 1))		// 001
												flag[6] <= 1;
											else if ((flag[6]==1)&& (flag[7] != 1))		// 011
												flag[7] <= 1;
											else if ((flag[7] == 1)&& (flag[8] != 1))		// 011
												flag[8] <= 1;
											else if ((flag[8] == 1)&& (flag[9] != 1))		// 011
												flag[9] <= 1;
										end
										
									// Blue
									if (notes[counter][0] == 1)
										begin
											if ((flag[10] != 1) && (flag[11] != 1) && (flag[12] != 1)&& (flag[13] != 1)&& (flag[14] != 1)) // 000
												flag[10] <= 1;
											else if((flag[14]==1)&&(flag[10]!=1))
												flag[0] <=1;
											else if ((flag[11] != 1) && (flag[10] == 1))		// 001
												flag[11] <= 1;
											else if ((flag[11]==1)&& (flag[12] != 1))		// 011
												flag[12] <= 1;
											else if ((flag[12] == 1)&& (flag[13] != 1))		// 011
												flag[13] <= 1;
											else if ((flag[13] == 1)&& (flag[14] != 1))		// 011
												flag[14] <= 1;
										end
								end
						end
					DONE:
						begin
							if (btnU)
								state <= PLAY;
							else 
								begin								
									score <= 0;
									posCount <= 0;
									counter <= 0;
								end
						end
					endcase
				end
		end

	
	always @(posedge DIV_CLK[1])
	begin
		if (tempFlag==0)
			begin
				boxPos<=450;
				tempFlag<=1;
			end
		else
			begin
				boxPos<=410;
				tempFlag<=0;
			end
			
	end
	
	
	
	wire box = CounterX>=0 && CounterX<=640 && CounterY<=boxPos && CounterY>=410; 
	wire r_hit = CounterX>=0 && CounterX<=199 && CounterY<=(redBoxPos) && CounterY>=(410);
	wire g_hit = CounterX>=220 && CounterX<=419 && CounterY<=(greenBoxPos) && CounterY>=(410);
	wire b_hit = CounterX>=440 && CounterX<=639 && CounterY<=(blueBoxPos) && CounterY>=(410);

	
	wire R4 = CounterX>=0 && CounterX<=199 && CounterY<=(position[4]+20) && CounterY>=(position[4]-20);
	wire R3 = CounterX>=0 && CounterX<=199 && CounterY<=(position[3]+20) && CounterY>=(position[3]-20);
	wire R2 = CounterX>=0 && CounterX<=199 && CounterY<=(position[2]+20) && CounterY>=(position[2]-20);
	wire R1 = CounterX>=0 && CounterX<=199 && CounterY<=(position[1]+20) && CounterY>=(position[1]-20);
	wire Red = (CounterX>=0 && CounterX<=199 && CounterY<=(position[0]+20) && CounterY>=(position[0]-20)) || R1 || R2 || R3 || R4 || r_hit || g_hit || b_hit;


	wire G9 = CounterX>=220 && CounterX<=419 && CounterY<=(position[9]+20) && CounterY>=(position[9]-20);
	wire G8 = CounterX>=220 && CounterX<=419 && CounterY<=(position[8]+20) && CounterY>=(position[8]-20);	
	wire G7 = CounterX>=220 && CounterX<=419 && CounterY<=(position[7]+20) && CounterY>=(position[7]-20);
	wire G6 = CounterX>=220 && CounterX<=419 && CounterY<=(position[6]+20) && CounterY>=(position[6]-20);
	wire Green = (CounterX>=220 && CounterX<=419 && CounterY<=(position[5]+20) && CounterY>=(position[5]-20)) || G6 || G7 || G8 || G9 || box ||r_hit || g_hit || b_hit;

	wire B14 = CounterX>=440 && CounterX<=639 && CounterY<=(position[14]+20) && CounterY>=(position[14]-20);
	wire B13 = CounterX>=440 && CounterX<=639 && CounterY<=(position[13]+20) && CounterY>=(position[13]-20);
	wire B12 = CounterX>=440 && CounterX<=639 && CounterY<=(position[12]+20) && CounterY>=(position[12]-20);
	wire B11 = CounterX>=440 && CounterX<=639 && CounterY<=(position[11]+20) && CounterY>=(position[11]-20);
	wire Blue = CounterX>=440 && CounterX<=639 && CounterY<=(position[10]+20) && CounterY>=(position[10]-20) || B11 || B12 || B13 || B14 || box;
	
	always @(posedge clk)
	begin
		vga_r <= Red & inDisplayArea;
		vga_g <= Green & inDisplayArea;
		vga_b <= Blue & inDisplayArea;
	end
	
	/////////////////////////////////////////////////////////////////
	//////////////  	  VGA control ends here 	 ///////////////////
	/////////////////////////////////////////////////////////////////
	
	/////////////////////////////////////////////////////////////////
	//////////////  	  LD control starts here 	 ///////////////////
	/////////////////////////////////////////////////////////////////
	`define QI 			2'b00
	`define QGAME_1 	2'b01
	`define QGAME_2 	2'b10
	`define QDONE 		2'b11
	
	reg [3:0] p2_score; 
	reg [3:0] p1_score;
	wire LD0, LD1, LD2, LD3, LD4, LD5, LD6, LD7;
	
	assign LD0 = (p1_score == 4'b1010);
	assign LD1 = (p2_score == 4'b1010);
	
	assign LD2 = start;
	assign LD4 = reset;
	
	assign LD3 = 1'b0;//(state == `QI);
	assign LD5 = 1'b0;	
	assign LD6 = 1'b0;
	assign LD7 = 1'b0;
	
	/////////////////////////////////////////////////////////////////
	//////////////  	  LD control ends here 	 	////////////////////
	/////////////////////////////////////////////////////////////////
	
	/////////////////////////////////////////////////////////////////
	//////////////  	  SSD control starts here 	 ///////////////////
	/////////////////////////////////////////////////////////////////
	reg 	[3:0]	SSD;
	wire 	[3:0]	SSD0, SSD1, SSD2, SSD3;
	wire 	[1:0] ssdscan_clk;
	// wire ones, tens, hundreds, thousands;
	
	// assign ones = score % 10;
	// assign tens = (score % 100 - ones)/10;
	// assign hundreds = (score % 1000 - tens*10 - ones)/100;
	
	assign SSD3 = score[15:12];
	assign SSD2 = score[11:8];
	assign SSD1 = score[7:4];
	assign SSD0 = score[3:0];
	
	// need a scan clk for the seven segment display 
	// 191Hz (50MHz / 2^18) works well
	assign ssdscan_clk = DIV_CLK[19:18];	
	assign An0	= !(~(ssdscan_clk[1]) && ~(ssdscan_clk[0]));  // when ssdscan_clk = 00
	assign An1	= !(~(ssdscan_clk[1]) &&  (ssdscan_clk[0]));  // when ssdscan_clk = 01
	assign An2	= !( (ssdscan_clk[1]) && ~(ssdscan_clk[0]));  // when ssdscan_clk = 10
	assign An3	= !( (ssdscan_clk[1]) &&  (ssdscan_clk[0]));  // when ssdscan_clk = 11
	
	always @ (ssdscan_clk, SSD0, SSD1, SSD2, SSD3)
	begin : SSD_SCAN_OUT
		case (ssdscan_clk) 
			2'b00:
					SSD = SSD0;
			2'b01:
					SSD = SSD1;
			2'b10:
					SSD = SSD2;
			2'b11:
					SSD = SSD3;
		endcase 
	end	

	// and finally convert SSD_num to ssd
	reg [6:0]  SSD_CATHODES;
	assign {Ca, Cb, Cc, Cd, Ce, Cf, Cg, Dp} = {SSD_CATHODES, 1'b1};
	// Following is Hex-to-SSD conversion
	always @ (SSD) 
	begin : HEX_TO_SSD
		case (SSD)		
			4'b0000: SSD_CATHODES = 7'b0000001 ; //0
			4'b0001: SSD_CATHODES = 7'b1001111 ; //1
			4'b0010: SSD_CATHODES = 7'b0010010 ; //2
			4'b0011: SSD_CATHODES = 7'b0000110 ; //3
			4'b0100: SSD_CATHODES = 7'b1001100 ; //4
			4'b0101: SSD_CATHODES = 7'b0100100 ; //5
			4'b0110: SSD_CATHODES = 7'b0100000 ; //6
			4'b0111: SSD_CATHODES = 7'b0001111 ; //7
			4'b1000: SSD_CATHODES = 7'b0000000 ; //8
			4'b1001: SSD_CATHODES = 7'b0000100 ; //9
			4'b1010: SSD_CATHODES = 7'b0001000 ; //10 or A
			4'b1011: SSD_CATHODES = 7'b1100000 ; //b
			4'b1100: SSD_CATHODES = 7'b0110001 ; //C
			4'b1101: SSD_CATHODES = 7'b1000010 ; //d
			4'b1110: SSD_CATHODES = 7'b0110000 ; //E
			4'b1111: SSD_CATHODES = 7'b0111000 ; //F 
			default: SSD_CATHODES = 7'bXXXXXXX ; // default is not needed as we covered all cases
		endcase
	end
	
	/////////////////////////////////////////////////////////////////
	//////////////  	  SSD control ends here 	 ///////////////////
	/////////////////////////////////////////////////////////////////
endmodule
