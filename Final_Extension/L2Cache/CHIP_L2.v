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




endmodule


