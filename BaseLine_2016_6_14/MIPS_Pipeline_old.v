
module MIPS_Pipeline(
		// control interface
		clk, 
		rst_n,
		
		//I cache interface
		ICACHE_addr,
		ICACHE_rdata,
		ICACHE_ren,
		ICACHE_wen,
		ICACHE_wdata,
		ICACHE_stall,
	
		//D cache interface
		DCACHE_addr,
		DCACHE_rdata,
		DCACHE_ren,
		DCACHE_wen,
		DCACHE_wdata,
		DCACHE_stall
	);
	
	input clk;
	input rst_n; 
	
	output [29:0] ICACHE_addr;
	input [31:0] ICACHE_rdata;
	output ICACHE_ren;
	output ICACHE_wen;
	output [31:0] ICACHE_wdata;
	input ICACHE_stall;

	output [29:0] DCACHE_addr;
	input [31:0] DCACHE_rdata;
	output DCACHE_ren;
	output DCACHE_wen;
	output [31:0] DCACHE_wdata;
	input DCACHE_stall;
	
	//control output
	parameter R_TYPE = 6'b000000;
	wire j, jal, jr, branch_signal, branch;
	
	wire memtoreg;
	wire regwrite;
	wire memread;
	wire memwrite;
	wire ALUsrc;
	wire [3:0] ALUop;
	wire RegDst;
	
	wire memtoreg_b;
	wire regwrite_b;
	wire memread_b;
	wire memwrite_b;
	wire ALUsrc_b;
	wire [3:0] ALUop_b;
	wire RegDst_b;
	
	//hazard output
	wire bubble;
	wire redo;
	wire nope;
	
	
	reg [29:0] PC_in;
	wire [29:0] PC_next;
	wire [29:0] PC_add4;
	
	wire [29:0] jump_addr;
	wire [29:0] jump_or_PC_add4;
	
	wire [31:0] instruction_next;
	reg [31:0] instruction;
	
	//-----------------Control---------------------//
	
	assign memtoreg = (op==6'b100011)? 1'b1: 1'b0;
	assign regwrite = (op==6'b000100 || op==6'b000010 || ((op==R_TYPE)&&(funct == 6'b001000))||(op==6'b101011))? 1'b0:1'b1;
	assign memread = memtoreg;
	assign memwrite = (op==6'b101011)? 1'b1: 1'b0;
	assign ALUsrc = (op==R_TYPE || op==6'b000010 || op==6'b000011)?1'b0:1'b1;
	assign RegDst = (op==R_TYPE)?1'b1:1'b0;
	
	assign memtoreg_b = (bubble)? 0: memtoreg;
	assign regwrite_b = (bubble)? 0: regwrite;
	assign memread_b =  (bubble)? 0: memread;
	assign memwrite_b = (bubble)? 0: memwrite
	assign ALUsrc_b =   (bubble)? 0: ALUsrc;
	assign RegDst_b =   (bubble)? 0: RegDst;
	
	assign jr = (op == R_TYPE && (funct == 6'b001000 || funct == 6'b001001)) 1'b1:1'b0;
	assign jal = ((op == 6'b000011)||(op == R_TYPE && funct == 6'b001001)) 1'b1:1'b0;
	assign j = (op == 6'b000010)? 1'b1: 1'b0;
	assign branch_signal = (op == 6'b000100)? 1'b1: 1'b0;
	assign branch = branch_signal & zero;
	always(*)begin
		case(op)
			R_TYPE:
				case(funct)
					6'b100000: assign ALUop = 0;
					6'b100010: assign ALUop = 1;
					6'b100100: assign ALUop = 2;
					6'b100101: assign ALUop = 3;
					6'b100110: assign ALUop = 4;
					6'b000000: assign ALUop = 5;
					6'b000011: assign ALUop = 6;
					6'b000010: assign ALUop = 7;
					6'b101010: assign ALUop = 8;
					6'b011011: assign ALUop = 9;
					default: assign ALUop = 0;
				endcase
			6'b001000: assign ALUop = 0;
			6'b001100: assign ALUop = 2;
			6'b001101: assign ALUop = 3;
			6'b001110: assign ALUop = 4;
			6'b001010: assign ALUop = 8;
			6'b000100: assign ALUop = 1; //BEQ not sure if essential
			default: assign ALUop = 0;
		endcase
	end
	
	//-----------------Hazard---------------------//
	
	assign redo = (DCACHE_stall || ICACHE_stall)? 1'b1: 1'b0;
	assign nope = (j || jr || branch || jal)?1'b1: 1'b0;
	assign bubble = (memread_EX && ((rt_EX == rs)||(rt_EX == rt)))? 1'b1: 1'b0;
	
	assign ICACHE_ren = 1'b1;
	assign ICACHE_wen = 1'b0;
	assign ICACHE_wdata = 0;
	
	assign PC_add4 = PC_in + 2'b100;
	assign jump_or_PC_add4 = (j||jal||jr)? jump_addr: PC_add4;
	assign PC_next = (bubble || redo)? PC_in: jump_or_PC_add4;
	assign instruction_next = (bubble&&!nope)?instruction:
							(!bubble&&nope)?32'b0:
							ICACHE_rdata;
	assign ICACHE_addr = PC_in;
	
	always @(posedge clk)
		begin 
			if(~rst_n) PC_in<=0;
			else begin 
				PC_in <= PC_next;
				instruction <= instruction_next;
			end
		end

endmodule