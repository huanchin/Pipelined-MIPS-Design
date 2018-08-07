Final_tb.v		        	testbench for project, use it when the pipelined datapath is debugged
slow_memory		same spec as HW4, but not the same

I_mem			Istructionary memory. machine code , series and sort         ( with assembly description )
D_mem			Data memory. initial data for slow memory

TestBed.v			expected write operation to data memory (DMEM) of finaltest
					
CHIP.v			Top module of your design, including MIPS_Pipeline, I_cache, and D_cache	
			You cannot moidy this module!!			