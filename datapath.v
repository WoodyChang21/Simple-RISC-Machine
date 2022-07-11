module datapath(clk, readnum, vsel, loada, loadb, shift, asel, bsel, ALUop, loadc, loads, writenum, write, mdata, sximm8, sximm5, PC, status_out, datapath_out);
    input clk, loada, loadb, asel, bsel, loadc, loads, write;
    input [2:0] readnum; 
    input [1:0] shift;
    input [1:0] ALUop;
    input [1:0] vsel; //lab6: changed vsel from 1 bit to 2 bit
    input [2:0] writenum;
    input [15:0] sximm8, sximm5; //lab6
    input [7:0] PC; //lab6
    output [2:0] status_out;
    output [15:0] datapath_out;

    input [15:0]mdata;
    wire [7:0] PC;

    wire [15:0] data_out, register_a_out, register_b_out, Ain, Bin, ALUout, sout, register_c_out; 
    wire [2:0] status_register_input;
    reg [15:0] data_in;

    
    //assign data_in = vsel ? datapath_in : datapath_out; //if vsel is true, data_in = datapath_in
    always @(*) begin  // Extending mux to 4-bits
        case (vsel)
            2'b00: data_in = mdata; //mdata;
            2'b01: data_in = sximm8;
            2'b10: data_in = {8'b0,PC};
            2'b11: data_in = datapath_out;
        endcase
    end
   
    regfile REGFILE(data_in,writenum,write,readnum,clk,data_out); //register file

    vDFFE register_a(clk, loada, data_out, register_a_out); //load register A
    vDFFE register_b(clk, loadb, data_out, register_b_out); //load register B

    assign Ain = asel ? 16'd0 : register_a_out; //if asel is true, alu input A = register A output. if asel is false, alu input A = 0

    shifter shift_b(register_b_out,shift,sout); // output of register b goes into shifter

    assign Bin = bsel ? sximm5 : sout; // If bsel = 1'b1, Bin = sout, otherwise Bin = sximm8
    
    ALU alu(Ain, Bin, ALUop, ALUout, status_register_input); // preforms arithmetic logic operation, also sets the status flags

    vDFFE #(3) status_reg (clk, loads, status_register_input, status_out); // stores value of status if loads is 1'b1
    vDFFE register_c(clk, loadc, ALUout, register_c_out); //load register C

    assign datapath_out = register_c_out;
    
endmodule


