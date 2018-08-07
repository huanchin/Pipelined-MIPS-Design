#You may modified the clock constraints 
#or add more constraints for your design
####################################################
set cycle  6        
create_clock -period $cycle [get_ports clk]

####################################################



#The following are design spec. for synthesis
#You can NOT modify this seciton except for the I/O delay
#####################################################
set_clock_uncertainty  0.1  [all_clocks]
set_clock_latency      0.5  [all_clocks]
set_input_delay  2.5     -clock clk [remove_from_collection [all_inputs] [get_ports clk]]
set_output_delay 2.5     -clock clk [all_outputs] 
set_load         1     [all_outputs]
set_drive        1     [all_inputs]

set_operating_conditions -min_library fast -min fast -max_library slow -max slow
set_wire_load_model -name tsmc13_wl10 -library slow               
#####################################################


#Compile and save files
#You may modified setting of compile 
#####################################################
compile
write_sdf -version 2.1 CHIP.sdf
write -format verilog -hier -output CHIP.vg
write -format ddc     -hier -output CHIP.ddc  
#####################################################                    
