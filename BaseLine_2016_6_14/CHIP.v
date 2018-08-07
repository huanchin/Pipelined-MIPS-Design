// Top module of your design, you cannot modify this module!!
module CHIP (	mem_read_D,
				mem_write_D,
				mem_addr_D,
				mem_rdata_D,
				mem_wdata_D,
				mem_ready_D,
				mem_read_I,
				mem_write_I,
				mem_addr_I,
				mem_rdata_I,
				mem_wdata_I,
				mem_ready_I,
				clk,
				rst_n,
				DCACHE_addr,
				DCACHE_wdata,
				DCACHE_wen
				);

output			mem_read_I;
output			mem_write_I;
output	[31:4]	mem_addr_I;
input	[127:0]	mem_rdata_I;
output	[127:0]	mem_wdata_I;
input			mem_ready_I;

output			mem_read_D;
output			mem_write_D;
output	[31:4]	mem_addr_D;
input	[127:0]	mem_rdata_D;
output	[127:0]	mem_wdata_D;
input			mem_ready_D;

input			clk, rst_n;

// For testbench check
output	[29:0]	DCACHE_addr;
output	[31:0]	DCACHE_wdata;
output			DCACHE_wen;

// wire declaration

wire [29:0] ICACHE_addr;
wire [31:0] ICACHE_rdata;
wire ICACHE_ren;
wire ICACHE_wen;
wire [31:0] ICACHE_wdata;
wire ICACHE_stall;

wire [29:0] DCACHE_addr;
wire [31:0] DCACHE_rdata;
wire DCACHE_ren;
wire DCACHE_wen;
wire [31:0] DCACHE_wdata;
wire DCACHE_stall;
	
//=========================================
	// Note that the overall design of your MIPS includes:
	// 1. pipelined MIPS processor
	// 2. data cache
	// 3. instruction cache


	MIPS_Pipeline i_MIPS(
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
	
	cache D_cache(
		clk,
		~rst_n,
		
		DCACHE_ren,
		DCACHE_wen,
		DCACHE_addr,
		DCACHE_rdata,
		DCACHE_wdata,
		DCACHE_stall,
		
		mem_read_D,
		mem_write_D,
		mem_addr_D,
		mem_rdata_D,
		mem_wdata_D,
		mem_ready_D
	);

	cache I_cache(
		clk,
		~rst_n,
		
		ICACHE_ren,
		ICACHE_wen,
		ICACHE_addr,
		ICACHE_rdata,
		ICACHE_wdata,
		ICACHE_stall,
		
		mem_read_I,
		mem_write_I,
		mem_addr_I,
		mem_rdata_I,
		mem_wdata_I,
		mem_ready_I
	);



endmodule


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

    output reg [29:0] DCACHE_addr;
    input [31:0] DCACHE_rdata;
    output reg DCACHE_ren;
    output reg DCACHE_wen;
    output reg [31:0] DCACHE_wdata;
    input DCACHE_stall;

//------------wire/reg---------------  
    parameter R_TYPE = 6'b000000;
	integer i;
	wire j, jal, jr, branch_signal, branch;
	
	wire memtoreg;
	wire regwrite;
	wire memread;
	wire memwrite;
	wire ALUsrc;
	reg [3:0] ALUop;
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
  //-----------------Forwarding--------------------//
	reg [4:0] MEM_WB_reg_rd;
	reg [4:0] EX_MEM_reg_rd;
	reg [2:0] ForwardA;
	reg [2:0] ForwardB;
	
	
	reg [31:0] PC_in;
	wire [31:0] PC_next;
	wire [31:0] PC_4;
	wire [31:0] PC_4_ID_w;
	
	reg [31:0] jump_addr;
	wire [31:0] jump_or_PC_add4;
	wire [31:0] jump_out;
	wire [31:0] branch_out;
	
	wire [31:0] instruction_next;
	reg [31:0] instruction;
	wire [31:0] instruction_w;
	
	reg [4:0] rtrd_prev;
	reg [4:0] rtrd;
	reg [3:0] ALUop_EX;
	
	reg [31:0] RegMemory[0:31];
	reg memtoreg_EX;
	reg regwrite_EX;
	reg jal_EX;
	reg jal_ex;
	reg memwrite_EX;
	reg memread_EX;
	
	reg memwrite_ex;
	reg memread_ex;
	
	//EX
	reg ALUsrc_EX;
	reg ALUsrc_ex;
	//ReadData
	reg [31:0] ReadData1_EX;
	reg [31:0] ReadData2_EX;
	reg [31:0] sign_ext_out_EX;
	wire [31:0] sign_ext_out;
	reg [31:0] PC_4_EX;
	reg [4:0] rs_EX;
	reg [4:0] rtrd_EX;
	
	reg [3:0]ALUop_ex;
	reg memtoreg_ex;
	reg regwrite_ex;
	reg [31:0] ReadData1_ex;
	reg [31:0] ReadData2_ex;
	reg [31:0] sign_ext_out_ex;
	reg [31:0] PC_4_ex;
	reg [4:0] rs_ex;
	reg [4:0] rtrd_ex;
	
	
  	reg memtoreg_MEM;
	reg regwrite_MEM;
	reg jal_MEM;

	//M
	reg memwrite_MEM;
	reg memread_MEM;

	reg [31:0] ALU_out_MEM;
	reg [31:0] Mem_writedata;
 
	reg [31:0] PC_4_MEM;

	reg memtoreg_MEM_w;
	reg regwrite_MEM_w;
	reg jal_MEM_w;
	reg memwrite_MEM_w;
	reg memread_MEM_w;
	reg [31:0] ALU_out_MEM_w;
	reg [31:0] Mem_writedata_w;
	reg [4:0] EX_MEM_reg_rd_w;
	reg [31:0] PC_4_MEM_w;
  	reg [4:0] EX_reg_rd;
  	
	//WB
 
 
 

	//M
 
 

	reg [31:0] ReadMem_MEM;
	reg [31:0] writedata_MEM;
  //WB
	reg memtoreg_WB;
	reg regwrite_WB;
	reg jal_WB;

	reg [31:0] ReadMem_WB;	
	reg [31:0] ALU_out_WB;
	reg [4:0] MEM_WB_reg_rd_no_jal;
	reg [31:0] PC_4_WB;

	reg memtoreg_WB_w;
	reg regwrite_WB_w;
	reg jal_WB_w;
	reg [31:0] ReadMem_WB_w;
	reg [31:0] ALU_out_WB_w;
	reg [4:0] MEM_WB_reg_rd_w;
	reg [31:0] PC_4_WB_w;
  	reg [31:0] ALU_input1;
	reg [31:0] ALU_input2;
	reg [31:0] ALU_input2_no_ALUsrc;
 
	reg [31:0] write_data_1;
	
	reg [31:0] WriteData;
 //////////////////////////ID////////////////////////////////////////////////////// 
  	wire [5:0] op;
	wire [4:0] rs;
	wire [4:0] rt;
	wire [4:0] rd;
	wire [15:0] imm;
	wire [25:0] addr;
	wire [5:0] funct;
	
	reg [31:0] ReadData1;
	reg [31:0] ReadData2;
	reg [4:0] WriteReg;
	reg [31:0] ALU_out_EX;
	reg [31:0] PC_4_ID;
	
	reg zero;
	wire [27:0] shiftl_out_2;
	wire [31:0] shiftl_out;
	
	assign op = instruction[31:26];
	assign rs = instruction[25:21];
	assign rt = instruction[20:16];
	assign rd = instruction[15:11];
	assign imm = instruction[15:0];
	assign addr = instruction[25:0];
	assign funct = instruction[5:0];
//---------------------Hou------------------------//

	
	
	//-----------------Control---------------------//
	
	assign memtoreg = (op==6'b100011)? 1'b1: 1'b0;
	assign regwrite = (op==6'b000100 || op==6'b000010 || ((op==R_TYPE)&&(funct == 6'b001000))||(op==6'b101011) || instruction==0)? 1'b0:1'b1;
	assign memread = memtoreg;
	assign memwrite = (op==6'b101011)? 1'b1: 1'b0;
	assign ALUsrc = (op==R_TYPE || op==6'b000010 || op==6'b000011)?1'b0:1'b1;
	assign RegDst = (op==R_TYPE)?1'b1:1'b0;
	
	assign memtoreg_b = (bubble)? 0: memtoreg;
	assign regwrite_b = (bubble)? 0: regwrite;
	assign memread_b =  (bubble)? 0: memread;
  	assign memwrite_b = (bubble)? 0: memwrite;
	assign ALUop_b =    (bubble)? 0: ALUop;
	assign ALUsrc_b =   (bubble)? 0: ALUsrc;
	assign RegDst_b =   (bubble)? 0: RegDst;
	
	assign jr = (op == R_TYPE && (funct == 6'b001000 || funct == 6'b001001))? 1'b1:1'b0;
	assign jal = ((op == 6'b000011)||(op == R_TYPE && funct == 6'b001001))? 1'b1:1'b0;
	assign j = (op == 6'b000010)? 1'b1: 1'b0;
	assign branch_signal = (op == 6'b000100)? 1'b1: 1'b0;
	assign branch = branch_signal & zero;
	always@(*)begin
		case(op)
			R_TYPE:
				case(funct)
					6'b100000: ALUop = 0;
					6'b100010: ALUop = 1;
					6'b100100: ALUop = 2;
					6'b100101: ALUop = 3;
					6'b100110: ALUop = 4;
					6'b000000: ALUop = 5;
					6'b000011: ALUop = 6;
					6'b000010: ALUop = 7;
					6'b101010: ALUop = 8;
					6'b011011: ALUop = 9;
					default: ALUop = 0;
				endcase
			6'b001000: ALUop = 0;
			6'b001100: ALUop = 2;
			6'b001101: ALUop = 3;
			6'b001110: ALUop = 4;
			6'b001010: ALUop = 8;
			6'b000100: ALUop = 1; //BEQ not sure if essential
			default: ALUop = 0;
		endcase
	end
	
	//-----------------Hazard---------------------//
	
	assign redo = (DCACHE_stall || ICACHE_stall)? 1'b1: 1'b0;
	assign nope = (j || jr || branch || jal)?1'b1: 1'b0;
	assign bubble = (memread_EX && ((EX_reg_rd == rs)||(EX_reg_rd == rt)))? 1'b1: 1'b0;
	
	assign ICACHE_ren = 1'b1;
	assign ICACHE_wen = 1'b0;
	assign ICACHE_wdata = 0;
	
  	//-------------------IF---------------------//
	assign PC_4 = PC_in + 3'b100;
	assign jump_or_PC_add4 = (j||jal||jr||branch)? jump_addr: PC_4;
    assign PC_next = (bubble || redo)? PC_in: jump_or_PC_add4;
	assign instruction_next = (bubble&&!nope)?instruction:
							(!bubble&&nope)?32'b0:
							ICACHE_rdata;
	assign instruction_w = (redo)? instruction: instruction_next;
    assign ICACHE_addr = PC_in [31:2];
	assign  PC_4_ID_w = (redo)? PC_4_ID: PC_4;
	
	always @(posedge clk or negedge rst_n)
		begin 
          	if(~rst_n) begin
            	PC_in<=0;
              	instruction <= 0;
				PC_4_ID <= 0;
          	end
			else begin 
				PC_in <= PC_next;
				instruction <= instruction_w;
				PC_4_ID <= PC_4_ID_w;
			end
		end
//----------------------huanchin----------------------------------

//WB
always@(*) begin
	WriteReg = MEM_WB_reg_rd;
    WriteData = write_data_1;
end    
//--------------ID--------------------------------

//--------register---------------------
      
always@(*)begin
  case(ForwardA)
    2'b00: ReadData1 = RegMemory[rs];
    2'b01:	ReadData1 = ALU_out_EX;
    2'b10:	ReadData1 = writedata_MEM;
    2'b11:	ReadData1 = WriteData;
  endcase
  case(ForwardB)
    2'b00: ReadData2 = RegMemory[rt];
    2'b01:	ReadData2 = ALU_out_EX;
    2'b10:	ReadData2 = writedata_MEM;
    2'b11:	ReadData2 = WriteData;
  endcase
end  
//--------others--------------------------------
assign  sign_ext_out = {{16{imm[15]}},imm[15:0]}; //sign_extend
assign  shiftl_out = sign_ext_out << 2; // shift left 2
assign  branch_out = PC_4_ID + shiftl_out; //branch adder
assign  shiftl_out_2 = {addr,2'b00}; // shift left 2
assign  jump_out = {PC_4_ID[31:28],shiftl_out_2[27:0]}; // 

always@(*)begin

  
//--------comparator--------------------
    if(ReadData1==ReadData2)
        zero = 1;
    else 
        zero = 0;
//--------jump_addr MUX-----------------    
    if(jr)
        jump_addr = ReadData1;
    else if (branch)
        jump_addr = branch_out;
    else
        jump_addr = jump_out;
    
end
//----------control signal ID/EX-------------
always@(*)begin
	if(redo) begin
     //--------WB--------------
    memtoreg_ex = memtoreg_EX;
 	regwrite_ex = regwrite_EX;
	jal_ex = jal_EX;
  	//--------MEM-------------
    memread_ex = memread_EX;
    memwrite_ex = memwrite_EX;
  	//--------EX-------------
    ALUsrc_ex = ALUsrc_EX;
    ALUop_ex = ALUop_EX;
  //-------------------------------------
    ReadData1_ex = ReadData1_EX;
    ReadData2_ex = ReadData2_EX;
	
    
    PC_4_ex = PC_4_EX;
    rs_ex = rs_EX;
    rtrd_ex = EX_reg_rd;
    sign_ext_out_ex = sign_ext_out_EX;
	end
  
	else begin
     //--------WB--------------
    memtoreg_ex = memtoreg_b;
 	regwrite_ex = regwrite_b;
	jal_ex = jal;
  	//--------MEM-------------
    memread_ex = memread_b;
    memwrite_ex = memwrite_b;
  	//--------EX-------------
    ALUsrc_ex = ALUsrc_b;
    ALUop_ex = ALUop_b;
  //-------------------------------------
    ReadData1_ex = ReadData1;
    ReadData2_ex = ReadData2;
    
    PC_4_ex = PC_4_ID;
    rs_ex = rs;
    rtrd_ex = rtrd;
    sign_ext_out_ex = sign_ext_out;
	end
end 

always @(posedge clk or negedge rst_n)begin

//--------memory register---------------------
    
    if (~rst_n)begin
        for(i=0;i<32;i=i+1)
            RegMemory[i] <= 0;
		
		RegMemory[WriteReg] <= 0;
//----------register------------------------------   
		memtoreg_EX <= 0;
		regwrite_EX <= 0;
		jal_EX <= 0;
  	//--------MEM-------------
		memread_EX <= 0;
		memwrite_EX <= 0;
  	//--------EX-------------
		ALUsrc_EX <= 0;
		ALUop_EX <= 0;
  //	-------------------------------------
		ReadData1_EX <= 0;
		ReadData2_EX <= 0;
    
		PC_4_EX <= 0;
		rs_EX <= 0;
		EX_reg_rd <= 0;
		sign_ext_out_EX <= 0;
    end
    else
    begin
        if (regwrite_WB)begin
			RegMemory[WriteReg] <= WriteData;
        end
//----------register------------------------------   
		memtoreg_EX <= memtoreg_ex;
		regwrite_EX <= regwrite_ex;
		jal_EX <= jal_ex;
  	//--------MEM-------------
		memread_EX <= memread_ex;
		memwrite_EX <= memwrite_ex;
  	//--------EX-------------
		ALUsrc_EX <= ALUsrc_ex;
		ALUop_EX <= ALUop_ex;
  //	-------------------------------------
		ReadData1_EX <= ReadData1_ex;
		ReadData2_EX <= ReadData2_ex;
    
		PC_4_EX <= PC_4_ex;
		rs_EX <= rs_ex;
		EX_reg_rd <= rtrd_ex;
		sign_ext_out_EX <= sign_ext_out_ex;
    end 
//---------rtrd--------------------------------------------------
end
  	always@(*)
        if (RegDst)
            rtrd_prev = rd;
		else
          	rtrd_prev = rt;
  	always@(*)
        if (jal)
            rtrd = 5'b11111;
		else
          	rtrd = rtrd_prev;    
    
    
//-------------------------SHIH-----------------------------------

//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                         EX                                                       //
//////////////////////////////////////////////////////////////////////////////////////////////////////



//-----------------EX_reg_rd----------------------//


//-----------------EX_reg_rd---------------------//




//-----------------Forwarding--------------------//
always@(*)
begin
    if(regwrite_EX & (EX_reg_rd == rs))
        ForwardA = 2'd1;
    else if(regwrite_MEM & (EX_MEM_reg_rd == rs) )
        ForwardA = 2'd2;
    else if(regwrite_WB & (WriteReg == rs))
        ForwardA = 2'd3;
    else
        ForwardA = 2'd0;
end

always@(*)
begin
    if(regwrite_EX & (EX_reg_rd == rt))
        ForwardB = 2'd1;
    else if(regwrite_MEM & (EX_MEM_reg_rd == rt) )
        ForwardB = 2'd2;
    else if(regwrite_WB & (WriteReg == rt))
        ForwardB = 2'd3;
    else
        ForwardB = 2'd0;
end
//-----------------Forwarding--------------------//


//-----------------ALU input---------------------//


always@(*)
begin
    ALU_input1 = ReadData1_EX;
	ALU_input2_no_ALUsrc = ReadData2_EX;
end



always@(*)
begin
    if(ALUsrc_EX)
        ALU_input2 = sign_ext_out_EX;
    else
        ALU_input2 = ALU_input2_no_ALUsrc;
end
//-----------------ALU input---------------------//


//-----------------ALU Compute-------------------//

always@(*)
begin
    case(ALUop_EX)
    4'd0:  ALU_out_EX = ALU_input1 + ALU_input2;
    4'd1:  ALU_out_EX = ALU_input1 - ALU_input2;
    4'd2:  ALU_out_EX = ALU_input1 & ALU_input2;
    4'd3:  ALU_out_EX = ALU_input1 | ALU_input2;
    4'd4:  ALU_out_EX = ALU_input1 ^ ALU_input2;
    4'd5:  ALU_out_EX = ALU_input1 << ALU_input2;
    4'd6:  ALU_out_EX = ALU_input1 >>> ALU_input2;
    4'd7:  ALU_out_EX = ALU_input1 >> ALU_input2;
    4'd8:	begin
      		if(ALU_input1 < ALU_input2)
            	ALU_out_EX = 1;
            else
                ALU_out_EX = 0;
    	end
    default: ALU_out_EX = ~(ALU_input1 | ALU_input2);
      
    endcase
end
//-----------------ALU Compute-------------------//


//-----------------EX_MEM------------------------//
//WB


always@(*)
begin
  	if(redo)
    begin
        memtoreg_MEM_w      = memtoreg_MEM;
        regwrite_MEM_w      = regwrite_MEM;
        jal_MEM_w           = jal_MEM;
        memwrite_MEM_w      = memwrite_MEM;
        memread_MEM_w       = memread_MEM;
        ALU_out_MEM_w       = ALU_out_MEM;
        Mem_writedata_w     = Mem_writedata;
        EX_MEM_reg_rd_w     = EX_MEM_reg_rd;
        PC_4_MEM_w          = PC_4_MEM;
    end
    else
    begin
        memtoreg_MEM_w      = memtoreg_EX;
        regwrite_MEM_w      = regwrite_EX;
        jal_MEM_w           = jal_EX;
        memwrite_MEM_w      = memwrite_EX;
        memread_MEM_w       = memread_EX;
        ALU_out_MEM_w       = ALU_out_EX;
        Mem_writedata_w     = ALU_input2_no_ALUsrc;
        EX_MEM_reg_rd_w     = EX_reg_rd;
        PC_4_MEM_w          = PC_4_EX;
    end
end

always@(posedge clk or negedge rst_n)
begin
	if(~rst_n) begin
		memtoreg_MEM    <= 0;
		regwrite_MEM    <= 0;
		jal_MEM         <= 0;
		memwrite_MEM    <= 0;
		memread_MEM     <= 0;
		ALU_out_MEM     <= 0;
		Mem_writedata   <= 0;
		EX_MEM_reg_rd   <= 0;
		PC_4_MEM        <= 0;
	end
	else begin
		memtoreg_MEM    <= memtoreg_MEM_w;
		regwrite_MEM    <= regwrite_MEM_w;
		jal_MEM         <= jal_MEM_w;
		memwrite_MEM    <= memwrite_MEM_w;
		memread_MEM     <= memread_MEM_w;
		ALU_out_MEM     <= ALU_out_MEM_w;
		Mem_writedata   <= Mem_writedata_w;
		EX_MEM_reg_rd   <= EX_MEM_reg_rd_w;
		PC_4_MEM        <= PC_4_MEM_w;
	end
end
//-----------------EX_MEM------------------------//

//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                         EX                                                       //
//////////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                         MEM                                                      //
//////////////////////////////////////////////////////////////////////////////////////////////////////


//-----------------CACHE-------------------------//


always@(*)
begin
  	DCACHE_addr = ALU_out_MEM [31:2];
    ReadMem_MEM = DCACHE_rdata;
    DCACHE_ren = memread_MEM;
    DCACHE_wen = memwrite_MEM;
    DCACHE_wdata = Mem_writedata;
end

//-----------------CACHE-------------------------//

always@(*)
begin
	if(jal_MEM)
		writedata_MEM = PC_4_MEM;
	else if (memtoreg_MEM)
		writedata_MEM = ReadMem_MEM;
	else
		writedata_MEM = ALU_out_MEM;
end


//-----------------MEM_WB------------------------//



always@(*)
begin
  	if(redo)
    begin
        memtoreg_WB_w               = memtoreg_WB;
        regwrite_WB_w               = regwrite_WB;
        jal_WB_w                    = jal_WB;
        ReadMem_WB_w                = write_data_1;
        ALU_out_WB_w                = ALU_out_WB;
        MEM_WB_reg_rd_w             = MEM_WB_reg_rd;
        PC_4_WB_w                   = PC_4_WB;
    end
    else
    begin
        memtoreg_WB_w               = memtoreg_MEM;
        regwrite_WB_w               = regwrite_MEM;
        jal_WB_w                    = jal_MEM;
        ReadMem_WB_w                = writedata_MEM;
        ALU_out_WB_w                = ALU_out_MEM;
        MEM_WB_reg_rd_w      = EX_MEM_reg_rd;
        PC_4_WB_w                   = PC_4_MEM;
    end
end

always@(posedge clk or negedge rst_n)
begin
	if(~rst_n)begin
		memtoreg_WB             <= 0;
		regwrite_WB             <= 0;
		jal_WB                  <= 0;
		ReadMem_WB              <= 0;
		ALU_out_WB              <= 0;
		MEM_WB_reg_rd           <= 0;
		PC_4_WB                 <= 0;
	end
	else begin
		memtoreg_WB             <= memtoreg_WB_w;
		regwrite_WB             <= regwrite_WB_w;
		jal_WB                  <= jal_WB_w;
		write_data_1              <= ReadMem_WB_w;
		ALU_out_WB              <= ALU_out_WB_w;
		MEM_WB_reg_rd           <= MEM_WB_reg_rd_w;
		PC_4_WB                 <= PC_4_WB_w;
	end
end

//-----------------MEM_WB------------------------//

//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                         MEM                                                      //
//////////////////////////////////////////////////////////////////////////////////////////////////////

//-------------------------SHIH-----------------------------------

endmodule



module cache(
    clk,
    proc_reset,
    proc_read,
    proc_write,
    proc_addr,
    proc_rdata,
    proc_wdata,
    proc_stall,
    mem_read,
    mem_write,
    mem_addr,
    mem_rdata,
    mem_wdata,
    mem_ready
);
    
//==== input/output definition ============================
    input          clk;
    // processor interface
    input          proc_reset;
    input          proc_read, proc_write;
    input   [29:0] proc_addr;
    input   [31:0] proc_wdata;
    output         proc_stall;
    output  [31:0] proc_rdata;
    // memory interface
    input  [127:0] mem_rdata;
    input          mem_ready;
    output         mem_read, mem_write;
    output  [27:0] mem_addr;
    output [127:0] mem_wdata;
    
//==== wire/reg definition ================================

reg [127:0] mem_wdata;
reg [31:0] Cache_w [0:31];
reg [31:0] Cache_r [0:31];
reg [31:0] proc_rdata;
reg [27:0] mem_addr;
reg [24:0] tag_w[0:7], tag_r[0:7];
reg [1:0] current_state, next_state;
reg v_w[0:7], v_r[0:7], d_w[0:7], d_r[0:7], hit, dirty, proc_stall, mem_read, mem_write;
integer i, j, k;
    
//==== combinational circuit ==============================

//FSM
//00:Read 01:Write 10:ReadMem 11:WriteBack
always@(*)
begin
    case(current_state)
    2'b10:
    begin
        if(mem_ready)
            next_state = 2'b00;
        else
            next_state = 2'b10;
    end
    2'b11:
    begin
        if(mem_ready)
            next_state = 2'b10;
        else
            next_state = 2'b11;
    end
    default:
    begin
        if((~proc_read) & (~proc_write))
            next_state = 2'b00;
        else if(~hit)
        begin
            if(dirty)
                next_state = 2'b11;
            else
                next_state = 2'b10;
        end
        else
            next_state = 2'b00;
    end
    endcase
end

// Cache
always@(*)
begin
    case(current_state)
    2'b00:
    if(proc_write)
    begin
        for(j = 0; j < 8; j = j + 1)
        begin
            v_w[j] = v_r[j];
            d_w[j] = d_r[j];
            tag_w[j] = tag_r[j];
        end
        for(j = 0; j < 32; j = j + 1)
            Cache_w[j] = Cache_r[j];
        if(hit)
        case(proc_addr[4:0])
        5'd0: begin
            Cache_w[0] = proc_wdata;
            d_w[0] = 1'b1;
            end
        5'd1: begin
            Cache_w[1] = proc_wdata;
            d_w[0] = 1'b1;
            end
        5'd2: begin
            Cache_w[2] = proc_wdata;
            d_w[0] = 1'b1;
            end
        5'd3: begin
            Cache_w[3] = proc_wdata;
            d_w[0] = 1'b1;
            end
        5'd4: begin
            Cache_w[4] = proc_wdata;
            d_w[1] = 1'b1;
            end
        5'd5: begin
            Cache_w[5] = proc_wdata;
            d_w[1] = 1'b1;
            end
        5'd6: begin
            Cache_w[6] = proc_wdata;
            d_w[1] = 1'b1;
            end
        5'd7: begin
            Cache_w[7] = proc_wdata;
            d_w[1] = 1'b1;
            end
        5'd8: begin
            Cache_w[8] = proc_wdata;
            d_w[2] = 1'b1;
            end
        5'd9: begin
            Cache_w[9] = proc_wdata;
            d_w[2] = 1'b1;
            end
        5'd10: begin
            Cache_w[10] = proc_wdata;
            d_w[2] = 1'b1;
            end
        5'd11: begin
            Cache_w[11] = proc_wdata;
            d_w[2] = 1'b1;
            end
        5'd12: begin
            Cache_w[12] = proc_wdata;
            d_w[3] = 1'b1;
            end
        5'd13: begin
            Cache_w[13] = proc_wdata;
            d_w[3] = 1'b1;
            end
        5'd14: begin
            Cache_w[14] = proc_wdata;
            d_w[3] = 1'b1;
            end
        5'd15: begin
            Cache_w[15] = proc_wdata;
            d_w[3] = 1'b1;
            end
        5'd16: begin
            Cache_w[16] = proc_wdata;
            d_w[4] = 1'b1;
            end
        5'd17: begin
            Cache_w[17] = proc_wdata;
            d_w[4] = 1'b1;
            end
        5'd18: begin
            Cache_w[18] = proc_wdata;
            d_w[4] = 1'b1;
            end
        5'd19: begin
            Cache_w[19] = proc_wdata;
            d_w[4] = 1'b1;
            end
        5'd20: begin
            Cache_w[20] = proc_wdata;
            d_w[5] = 1'b1;
            end
        5'd21: begin
            Cache_w[21] = proc_wdata;
            d_w[5] = 1'b1;
            end
        5'd22: begin
            Cache_w[22] = proc_wdata;
            d_w[5] = 1'b1;
            end
        5'd23: begin
            Cache_w[23] = proc_wdata;
            d_w[5] = 1'b1;
            end
        5'd24: begin
            Cache_w[24] = proc_wdata;
            d_w[6] = 1'b1;
            end
        5'd25: begin
            Cache_w[25] = proc_wdata;
            d_w[6] = 1'b1;
            end
        5'd26: begin
            Cache_w[26] = proc_wdata;
            d_w[6] = 1'b1;
            end
        5'd27: begin
            Cache_w[27] = proc_wdata;
            d_w[6] = 1'b1;
            end
        5'd28: begin
            Cache_w[28] = proc_wdata;
            d_w[7] = 1'b1;
            end
        5'd29: begin
            Cache_w[29] = proc_wdata;
            d_w[7] = 1'b1;
            end
        5'd30: begin
            Cache_w[30] = proc_wdata;
            d_w[7] = 1'b1;
            end
        default: begin
            Cache_w[31] = proc_wdata;
            d_w[7] = 1'b1;
            end
        endcase
    end
    else
    begin
        for(j = 0; j < 8; j = j + 1)
        begin
            v_w[j] = v_r[j];
            d_w[j] = d_r[j];            
            tag_w[j] = tag_r[j];
        end
        for(j = 0; j < 32; j = j + 1)
            Cache_w[j] = Cache_r[j];
    end
    2'b10:
    begin
        for(j = 0; j < 8; j = j + 1)
        begin
            v_w[j] = v_r[j];
            d_w[j] = d_r[j];            
            tag_w[j] = tag_r[j];
        end
        for(j = 0; j < 32; j = j + 1)
            Cache_w[j] = Cache_r[j];
        case(mem_addr[2:0])
        3'd0:begin
            {Cache_w[3], Cache_w[2], Cache_w[1], Cache_w[0]} = mem_rdata;
            tag_w[0] = mem_addr[27:3];
            v_w[0] = 1;
            d_w[0] = 0;
            end
        3'd1:begin
            {Cache_w[7], Cache_w[6], Cache_w[5], Cache_w[4]} = mem_rdata;
            tag_w[1] = mem_addr[27:3];
            v_w[1] = 1;
            d_w[1] = 0;
            end
        3'd2:begin
            {Cache_w[11], Cache_w[10], Cache_w[9], Cache_w[8]} = mem_rdata;
            tag_w[2] = mem_addr[27:3];
            v_w[2] = 1;
            d_w[2] = 0;
            end
        3'd3:begin
            {Cache_w[15], Cache_w[14], Cache_w[13], Cache_w[12]} = mem_rdata;
            tag_w[3] = mem_addr[27:3];
            v_w[3] = 1;
            d_w[3] = 0;
            end
        3'd4:begin
            {Cache_w[19], Cache_w[18], Cache_w[17], Cache_w[16]} = mem_rdata;
            tag_w[4] = mem_addr[27:3];
            v_w[4] = 1;
            d_w[4] = 0;
            end
        3'd5:begin
            {Cache_w[23], Cache_w[22], Cache_w[21], Cache_w[20]} = mem_rdata;
            tag_w[5] = mem_addr[27:3];
            v_w[5] = 1;
            d_w[5] = 0;
            end
        3'd6:begin
            {Cache_w[27], Cache_w[26], Cache_w[25], Cache_w[24]} = mem_rdata;
            tag_w[6] = mem_addr[27:3];
            v_w[6] = 1;
            d_w[6] = 0;
            end
        default:begin
            {Cache_w[31], Cache_w[30], Cache_w[29], Cache_w[28]} = mem_rdata;
            tag_w[7] = mem_addr[27:3];
            v_w[7] = 1;
            d_w[7] = 0;
            end
        endcase
    end
    default:
    begin
        for(j = 0; j < 8; j = j + 1)
        begin
            v_w[j] = v_r[j];
            d_w[j] = d_r[j];            
            tag_w[j] = tag_r[j];
        end
        for(j = 0; j < 32; j = j + 1)
            Cache_w[j] = Cache_r[j];
    end
    endcase
end

//signals
always@(*)
begin
    case(proc_addr[4:2])
    3'd0: begin
        hit = v_r[0] & (proc_addr[29:5] == tag_r[0]);
        dirty = d_r[0];
        end
    3'd1: begin
        hit = v_r[1] & (proc_addr[29:5] == tag_r[1]);
        dirty = d_r[1];
        end
    3'd2: begin
        hit = v_r[2] & (proc_addr[29:5] == tag_r[2]);
        dirty = d_r[2];
        end
    3'd3: begin
        hit = v_r[3] & (proc_addr[29:5] == tag_r[3]);
        dirty = d_r[3];
        end
    3'd4: begin
        hit = v_r[4] & (proc_addr[29:5] == tag_r[4]);
        dirty = d_r[4];
        end
    3'd5: begin
        hit = v_r[5] & (proc_addr[29:5] == tag_r[5]);
        dirty = d_r[5];
        end
    3'd6: begin
        hit = v_r[6] & (proc_addr[29:5] == tag_r[6]);
        dirty = d_r[6];
        end
    default: begin
        hit = v_r[7] & (proc_addr[29:5] == tag_r[7]);
        dirty = d_r[7];
        end
    endcase
end

//output
always@(*)
begin
    if(~current_state[1])
        if((~proc_read) & (~proc_write))
            proc_stall = 1'b0;
        else if(hit)
            proc_stall = 1'b0;
        else
            proc_stall = 1'b1;
    else
        proc_stall = 1'b1;

    case(proc_addr[4:0])
    5'd0: proc_rdata = Cache_r[0];
    5'd1: proc_rdata = Cache_r[1];
    5'd2: proc_rdata = Cache_r[2];
    5'd3: proc_rdata = Cache_r[3];
    5'd4: proc_rdata = Cache_r[4];
    5'd5: proc_rdata = Cache_r[5];
    5'd6: proc_rdata = Cache_r[6];
    5'd7: proc_rdata = Cache_r[7];
    5'd8: proc_rdata = Cache_r[8];
    5'd9: proc_rdata = Cache_r[9];
    5'd10: proc_rdata = Cache_r[10];
    5'd11: proc_rdata = Cache_r[11];
    5'd12: proc_rdata = Cache_r[12];
    5'd13: proc_rdata = Cache_r[13];
    5'd14: proc_rdata = Cache_r[14];
    5'd15: proc_rdata = Cache_r[15];
    5'd16: proc_rdata = Cache_r[16];
    5'd17: proc_rdata = Cache_r[17];
    5'd18: proc_rdata = Cache_r[18];
    5'd19: proc_rdata = Cache_r[19];
    5'd20: proc_rdata = Cache_r[20];
    5'd21: proc_rdata = Cache_r[21];
    5'd22: proc_rdata = Cache_r[22];
    5'd23: proc_rdata = Cache_r[23];
    5'd24: proc_rdata = Cache_r[24];
    5'd25: proc_rdata = Cache_r[25];
    5'd26: proc_rdata = Cache_r[26];
    5'd27: proc_rdata = Cache_r[27];
    5'd28: proc_rdata = Cache_r[28];
    5'd29: proc_rdata = Cache_r[29];
    5'd30: proc_rdata = Cache_r[30];
    default: proc_rdata = Cache_r[31];
    endcase
    if(next_state == 2'b10)
        mem_read = 1'b1;
    else
        mem_read = 1'b0;
    if(next_state == 2'b11)
        mem_write = 1'b1;
    else
        mem_write = 1'b0;

    if(next_state == 2'b11)
        case(proc_addr[4:2])
        3'd0: mem_addr = {tag_r[0], 3'd0};
        3'd1: mem_addr = {tag_r[1], 3'd1};
        3'd2: mem_addr = {tag_r[2], 3'd2};
        3'd3: mem_addr = {tag_r[3], 3'd3};
        3'd4: mem_addr = {tag_r[4], 3'd4};
        3'd5: mem_addr = {tag_r[5], 3'd5};
        3'd6: mem_addr = {tag_r[6], 3'd6};
        default: mem_addr = {tag_r[7], 3'd7};
        endcase 
    else 
        mem_addr = proc_addr[29:2];
    
    case(proc_addr[4:2])
    3'd0: mem_wdata = {Cache_r[3], Cache_r[2], Cache_r[1], Cache_r[0]};
    3'd1: mem_wdata = {Cache_r[7], Cache_r[6], Cache_r[5], Cache_r[4]};
    3'd2: mem_wdata = {Cache_r[11], Cache_r[10], Cache_r[9], Cache_r[8]};
    3'd3: mem_wdata = {Cache_r[15], Cache_r[14], Cache_r[13], Cache_r[12]};
    3'd4: mem_wdata = {Cache_r[19], Cache_r[18], Cache_r[17], Cache_r[16]};
    3'd5: mem_wdata = {Cache_r[23], Cache_r[22], Cache_r[21], Cache_r[20]};
    3'd6: mem_wdata = {Cache_r[27], Cache_r[26], Cache_r[25], Cache_r[24]};
    default: mem_wdata = {Cache_r[31], Cache_r[30], Cache_r[29], Cache_r[28]};
    endcase
end


//==== sequential circuit =================================
always@( posedge clk or posedge proc_reset ) begin
    if( proc_reset ) begin 
        current_state = 2'b00;
        for(i = 0; i < 8; i = i + 1)
        begin
            v_r[i] = 0;
            d_r[i] = 0;
        end
    end
    else begin
        current_state = next_state;
        for(i = 0; i < 8; i = i + 1)
        begin
            v_r[i] = v_w[i];
            d_r[i] = d_w[i];
        end
    end    
end
always@( posedge clk )
begin
    for(k = 0; k < 8; k = k + 1)
    begin
        tag_r[k] = tag_w[k];
    end
    for(k = 0; k < 32; k = k + 1)
        Cache_r[k] = Cache_w[k];
end
endmodule

