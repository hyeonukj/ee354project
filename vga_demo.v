`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// VGA verilog template
// Author:  Da Cheng
//////////////////////////////////////////////////////////////////////////////////
module vga_demo(ClkPort, vga_h_sync, vga_v_sync, vga_r, vga_g, vga_b, Sw0, Sw1, btnU,btnL, btnD,
	St_ce_bar, St_rp_bar, Mt_ce_bar, Mt_St_oe_bar, Mt_St_we_bar,
	An0, An1, An2, An3, Ca, Cb, Cc, Cd, Ce, Cf, Cg, Dp,
	LD0, LD1, LD2, LD3, LD4, LD5, LD6, LD7);
	input ClkPort, Sw0, btnU, btnL,btnD, Sw0, Sw1;
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
	reg [2:0] notes [0:22];
	reg [8:0] counter;
	reg [5:0] posCount;
	reg [2:0] state;	
	reg flag [0:14];
	wire R, G, B;
	integer i;
	
	localparam
	INITIAL = 3'b001,
	PLAY    = 3'b010,
	DONE    = 3'b100;
	
	initial begin
	notes[0] = 3'b000;
	notes[1] = 3'b001;
	notes[2] = 3'b010;
	notes[3] = 3'b100;
	notes[4] = 3'b001;
	notes[5] = 3'b010;
	notes[6] = 3'b100;
	notes[7] = 3'b111;
	notes[8] = 3'b010;
	notes[9] = 3'b101;
	notes[10] = 3'b010;
	notes[11] = 3'b101;
	notes[12] = 3'b010;
	notes[13] = 3'b101;
	notes[14] = 3'b011;
	notes[15] = 3'b111;
	notes[16] = 3'b010;
	notes[17] = 3'b101;
	notes[18] = 3'b010;
	notes[19] = 3'b101;
	notes[20] = 3'b010;
	notes[21] = 3'b101;
	notes[22] = 3'b011;
		
	for (i = 0; i < 15; i = i+1)
		begin
			position[i] = 0;
			flag[i] = 0;	
		end
	
	end
			
	always @(posedge DIV_CLK[19])
		begin
			if (reset)
				begin
					state <= INITIAL;
					posCount <= 0;
					counter <= 0;
					for (i = 0; i < 15; i = i+1)
						begin
							position[i] <= 0;
							flag[i] <= 0;
						end
				end
			else if (btnD)
				begin
					for (i = 0; i < 15; i = i+1)
						position[i] <= position[i];
				end
			else
				begin
					case (state)
					INITIAL:
						begin
							if (start == 1) 
								state <= PLAY;
						end
					PLAY:
						begin
							if (btnU)
								state <= INITIAL;
							
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
					endcase
				end
		end
	
	reg [8:0] boxPos;
	reg tempFlag;
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
	
	wire R4 = CounterX>=0 && CounterX<=199 && CounterY<=(position[4]+20) && CounterY>=(position[4]-20);
	wire R3 = CounterX>=0 && CounterX<=199 && CounterY<=(position[3]+20) && CounterY>=(position[3]-20);
	wire R2 = CounterX>=0 && CounterX<=199 && CounterY<=(position[2]+20) && CounterY>=(position[2]-20);
	wire R1 = CounterX>=0 && CounterX<=199 && CounterY<=(position[1]+20) && CounterY>=(position[1]-20);
	wire Red = (CounterX>=0 && CounterX<=199 && CounterY<=(position[0]+20) && CounterY>=(position[0]-20)) || R1 || R2 || R3 || R4 || box;

	wire G9 = CounterX>=220 && CounterX<=419 && CounterY<=(position[9]+20) && CounterY>=(position[9]-20);
	wire G8 = CounterX>=220 && CounterX<=419 && CounterY<=(position[8]+20) && CounterY>=(position[8]-20);	
	wire G7 = CounterX>=220 && CounterX<=419 && CounterY<=(position[7]+20) && CounterY>=(position[7]-20);
	wire G6 = CounterX>=220 && CounterX<=419 && CounterY<=(position[6]+20) && CounterY>=(position[6]-20);
	wire Green = (CounterX>=220 && CounterX<=419 && CounterY<=(position[5]+20) && CounterY>=(position[5]-20)) || G6 || G7 || G8 || G9 || box;

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
	
	assign SSD3 = 4'b1111;
	assign SSD2 = 4'b1111;
	assign SSD1 = 4'b1111;
	assign SSD0 = position[0][3:0];
	
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
			4'b1111: SSD_CATHODES = 7'b1111111 ; //Nothing 
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
			default: SSD_CATHODES = 7'bXXXXXXX ; // default is not needed as we covered all cases
		endcase
	end
	
	/////////////////////////////////////////////////////////////////
	//////////////  	  SSD control ends here 	 ///////////////////
	/////////////////////////////////////////////////////////////////
endmodule
