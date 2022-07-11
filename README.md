# Simple-RISC-Machine
A simple RISC machine with 8 registers, 16-bit instruction decoder, datapath controller, memory, 4 operation ALU, shifter, and overflow detection.

Written in verilog, with the instruction files for the testbench written in ARM Assembly and assembled using sas to the instruction files data.txt

TO IMPLEMENT: You will need to install the most recent edition of modelsim and quartus

Create a new modelsim project on your machine, and add all of the files in this repository to the project Compile, and simulate work.top_tb and ensure that all the tests pass Create a new quartus project with the top file, and include top.v, alu.v, cpu.v, shifter.v, regfile.v, and datapath.v. Be sure to configure the hardware settings as appropriate for the hardware you are implementing it on Compile on quartus Download to your device, and all the functionality of the machine should work!
