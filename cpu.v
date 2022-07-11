//states
`define wait_state 4'd1
`define decode_state 5'd2
`define get_A_state 5'd3
`define get_B_state 5'd4
`define add_state 5'd5  
`define write_reg_state 5'd6
`define write_imm_state 5'd7
`define cmp_state 5'd8
`define and_state 5'd9
`define mvn_state 5'd10
`define mov_reg_state 5'd11
`define get_B_reg_state 5'd12

//ldr
`define LDR0 5'd13
`define LDR1 5'd14
`define LDR2 5'd15
`define LDR3 5'd16

//str
`define STR0 5'd17
`define STR1 5'd18
`define STR2 5'd19
`define STR3 5'd20
`define STR4 5'd21
`define STR5 5'd22

`define IF1 5'd23
`define IF2 5'd24
`define UpdatePC 5'd25
`define halt 5'd26

//opcodes
`define mov_opcode 3'b110
`define alu_opcode 3'b101


//op
`define mov_imm_op 2'b10
`define mov_reg_op 2'b00

`define alu_add_op 2'b00
`define alu_cmp_op 2'b01
`define alu_and_op 2'b10
`define alu_mvn_op 2'b11

`define mem_op 2'b00

//str,ldr
`define mem_ldr 3'b011
`define mem_str 3'b100

//mem_cd
`define MREAD 2'b01
`define MWRITE 2'b10
`define MNONE 2'b00



module cpu(clk,dp_mdata,reset,s,load,in,out,N,V,Z,w,mem_cmd,mem_addr);

    input clk, reset, s, load;
    input [15:0] dp_mdata,in;
    output [15:0] out;
    output N, V, Z, w;
    output [8:0] mem_addr;
    output [1:0] mem_cmd; //state_machine output and also CPU output 

    //IR 
    wire [1:0] op, ALUop, shift;
    wire [2:0] nsel;
    wire [15:0] sximm5, sximm8;
    wire [2:0] opcode, readnum, writenum;
    wire [15:0] ir_out; //instruction register out

    wire [15:0] dp_sximm5, dp_sximm8;
    wire [2:0] dp_nsel, dp_status_out;
    wire [1:0] dp_vsel, dp_shift; 
    wire dp_write, dp_loada, dp_loadb, dp_asel, dp_bsel, dp_loadc, dp_loads;
    wire [15:0] dp_mdata,dp_datapath_out;
    
    wire dp_load_pc; //output of state_machine
    wire dp_reset_pc; //output of state_machine
    wire [8:0] next_pc;
    wire [8:0] PC;
    wire [7:0] datapath_pc;

    wire dp_load_addr; //output of state_machine
    wire dp_addr_sel; //output of state_machine
    wire [8:0] data_address_out;

    assign datapath_pc = 8'b00000000;
    assign next_pc = dp_reset_pc? 9'b000000000: PC + 1'b1;
    vDFFE #(9) program_counter (clk,load_pc,next_pc,PC);
    vDFFE #(9) data_address (clk,load_addr,dp_datapath_out[8:0],data_address_out);
    assign mem_addr = dp_addr_sel? PC: data_address_out;


    vDFFE #(16) instruction_register (clk, load, dp_mdata, ir_out);

    InstructionDecoder ID(ir_out, opcode, op, dp_nsel, ALUop, sximm5, sximm8, shift, readnum, writenum);

    StateMachine FSM(clk, s, w, reset, opcode, op, dp_nsel, dp_vsel,dp_write, dp_loada, dp_loadb, dp_shift, 
                    dp_asel, dp_bsel, dp_loadc, dp_loads,dp_load_pc,dp_reset_pc,dp_load_addr,dp_addr_sel,mem_cmd,load_ir);
                    //(clk, s, w, reset, opcode, op, nsel, vsel, write, loada, loadb, shift, asel, bsel, loadc, loads, load_pc, reset_pc, load_addr, addr_sel, mem_cd, load_ir)


    datapath DP(clk, readnum, dp_vsel, dp_loada, dp_loadb, shift, dp_asel, dp_bsel, ALUop, dp_loadc, dp_loads, 
                    writenum, dp_write, dp_mdata, sximm8, sximm5, datapath_pc, dp_status_out, dp_datapath_out);
            //(clk, readnum, vsel, loada, loadb, shift, asel, bsel, ALUop, loadc, loads, writenum, write, mdata,
            //  sximm8, sximm5, PC, status_out, datapath_out)

    assign N = dp_status_out[2];
    assign V = dp_status_out[1];
    assign Z = dp_status_out[0];

    assign out = dp_datapath_out;

endmodule

//Decodes struction from Instruction register into signals for statemachine and datapath
module InstructionDecoder(instruction, opcode, op, nsel, ALUop, sximm5, sximm8, shift, readnum, writenum);
    input [15:0] instruction;
    input [2:0] nsel;

    output [1:0] op, ALUop, shift;
    output [15:0] sximm5, sximm8;
    output [2:0] opcode;
    output reg [2:0] readnum, writenum;

    assign ALUop = instruction[12:11];
    assign opcode = instruction[15:13];
    assign op = instruction[12:11];
    assign sximm5 = { {11{instruction[4]}} , instruction[4:0] };
    assign sximm8 = { {8{instruction[7]}} , instruction[7:0] };
    assign shift = instruction[4:3];
    
    always @(*) begin 
        case(nsel)
            3'b001: {readnum, writenum} = {2{instruction[2:0]}};  //Rm  
            3'b010: {readnum, writenum} = {2{instruction[7:5]}}; //Rd
            3'b100: {readnum, writenum} = {2{instruction[10:8]}}; //Rn
            default: {readnum, writenum} = {6{1'bx}};
        endcase
    end
endmodule  

//state machine controller for datapath
module StateMachine(clk, s, w, reset, opcode, op, nsel, vsel, write, loada, loadb, shift, asel, bsel, loadc, loads, load_pc, reset_pc, load_addr, addr_sel, mem_cd, load_ir);
    input clk;
    input s;
    input reset;
    input [2:0] opcode;
    input [1:0] op;
    output reg [2:0] nsel;
    output reg [1:0] vsel, shift; 
    output reg write, loada, loadb, asel, bsel, loadc, loads, w;     

    output reg load_pc, reset_pc, load_addr, addr_sel, load_ir;
    output reg [1:0] mem_cd;

    reg [4:0] state;

    //combinational logic block for state transitions
    always @(posedge clk) begin 
        if (reset) begin
			state = `wait_state; // if reset is 1, go to wait state 
		end else begin 
			casex({state, opcode, op})
                // wait state if s = 0, otherwise go to decode state 
				{`wait_state, 3'bxxx, 2'bxx} : state = (s ? `IF1 : `wait_state);
                
                {`IF1,3'bxxx,2'bxx} : state = `IF2;
                {`IF2,3'bxxx,2'bxx} : state = `UpdatePC;

                {`UpdatePC,3'bxxx,2'bxx} : state = `decode_state;

                {`decode_state, `alu_opcode, `alu_add_op} : state = `get_A_state;
                {`decode_state, `alu_opcode, `alu_cmp_op} : state = `get_A_state;
                {`decode_state, `alu_opcode, `alu_and_op} : state = `get_A_state;
                {`decode_state, `alu_opcode, `alu_mvn_op} : state = `get_B_state;
                {`decode_state, `mem_ldr, `mem_op} : state = `LDR0;
                {`decode_state, `mem_str, `mem_op} : state = `STR0;
                {`decode_state, 3'b111,2'bxx} : state = `halt;
                
                //move imm into register
                {`decode_state, `mov_opcode, `mov_imm_op}: state = `write_imm_state;
                //move reg 
                {`decode_state, `mov_opcode, `mov_reg_op}: state = `get_B_reg_state;
                
                {`write_imm_state, 3'bxxx, 2'bxx}: state = `wait_state;

                //move datapath_out into register
                {`get_B_reg_state, 3'bxxx, `alu_add_op}: state = `mov_reg_state;
                {`mov_reg_state, 3'bxxx, 2'bxx}: state = `write_reg_state;

                //mem operation
                {`LDR0,3'bxxx,2'bxx}: state = `LDR1;
                {`LDR1,3'bxxx,2'bxx}: state = `LDR2;
                {`LDR2,3'bxxx,2'bxx}: state = `LDR3;
                {`LDR3,3'bxxx,2'bxx}: state = `wait_state;

                {`STR0,5'bxxxxx}: state = `STR1;
                {`STR1,5'bxxxxx}: state = `STR2;
                {`STR2,5'bxxxxx}: state = `STR3;
                {`STR3,5'bxxxxx}: state = `STR4;
                {`STR4,5'bxxxxx}: state = `STR5;
                {`STR5,5'bxxxxx}: state = `wait_state;

                {`get_A_state, 3'bxxx, 2'bxx}: state = `get_B_state;
                //alu add operation
                {`get_B_state, 3'bxxx, `alu_add_op}: state = `add_state;
                {`add_state, 3'bxxx, 2'bxx}: state = `write_reg_state;

                //alu cmp operation 
                {`get_B_state, 3'bxxx, `alu_cmp_op}: state = `cmp_state;
                {`cmp_state, 3'bxxx, 2'bxx}: state = `wait_state;
                
                //alu and operation 
                {`get_B_state, 3'bxxx, `alu_and_op}: state = `and_state;
                {`and_state, 3'bxxx, 2'bxx}: state = `write_reg_state;

                //alu mvn operation
                {`get_B_state, 3'bxxx, `alu_mvn_op}: state = `mvn_state; //selects Rn
                {`mvn_state, 3'bxxx, 2'bxx}: state = `write_reg_state;

                //move to wait after write                
                {`write_reg_state, 3'bxxx, 2'bxx}: state = `wait_state;


                default: state = 4'b1111; //should never reach
            endcase
        end
    end  
    
    //asynchronous logic block for state outputs
    always @(*) begin
        case(state)

        //    output reg load_pc, reset_pc, load_addr, addr_sel;
        //        output reg [1:0] mem_cd;

            `wait_state: {vsel, nsel, write, loada, loadb, asel, bsel, loadc, loads, w, shift, load_pc, reset_pc, load_addr, addr_sel, mem_cd} = {
                        2'b00,          //vsel
                        3'b000,         //nsel
                        1'b0,           //write
                        1'b0,           //loada
                        1'b0,           //loadb
                        1'b0,           //asel          
                        1'b0,           //bsel
                        1'b0,           //loadc
                        1'b0,           //loads
                        1'b1,           //w                 w = 1
                        2'b00,          //shift  
                        1'b1,           //load_pc = 1
                        1'b1,           //reset_pc = 1
                        1'b0,           //load_addr
                        1'b0,           //addr_sel
                        2'b00,           //mem_cd
                        1'b0            //load_ir
                    };

            `IF1: {vsel, nsel, write, loada, loadb, asel, bsel, loadc, loads, w, shift, load_pc, reset_pc, load_addr, addr_sel, mem_cd} = {
                //set addr_sel = 1, mem_cd = MREAD
                        2'b00,          //vsel
                        3'b000,         //nsel
                        1'b0,           //write
                        1'b0,           //loada
                        1'b0,           //loadb
                        1'b0,           //asel          
                        1'b0,           //bsel
                        1'b0,           //loadc
                        1'b0,           //loads
                        1'b0,           //w               
                        2'b00,          //shift  
                        1'b0,           //load_pc
                        1'b0,           //reset_pc
                        1'b0,           //load_addr
                        1'b1,           //addr_sel      addr_sel = 1
                        `MREAD,         //mem_cd        MREAD
                        1'b0            //load_ir
            };
            `IF2: {vsel, nsel, write, loada, loadb, asel, bsel, loadc, loads, w, shift, load_pc, reset_pc, load_addr, addr_sel, mem_cd, load_ir} = {
                //set addr_sel = 1, load_ir = 1, mem_cd = MREAD
                        2'b00,          //vsel
                        3'b000,         //nsel
                        1'b0,           //write
                        1'b0,           //loada
                        1'b0,           //loadb
                        1'b0,           //asel          
                        1'b0,           //bsel
                        1'b0,           //loadc
                        1'b0,           //loads
                        1'b0,           //w               
                        2'b00,          //shift  
                        1'b0,           //load_pc
                        1'b0,           //reset_pc    
                        1'b0,           //load_addr
                        1'b1,           //addr_sel      addr_sel = 1
                        `MREAD,         //mem_cd        MREAD
                        1'b1            //load_ir       load_ir = 1
            };

            `decode_state: {vsel, nsel, write, loada, loadb, asel, bsel, loadc, loads, w, shift, load_pc, reset_pc, load_addr, addr_sel, mem_cd, load_ir} = { 
                        2'b00,          //vsel
                        3'b000,         //nsel
                        1'b0,           //write
                        1'b0,           //loada
                        1'b0,           //loadb
                        1'b0,           //asel
                        1'b0,           //bsel
                        1'b0,           //loadc
                        1'b0,           //loads
                        1'b0,           //w
                        2'b00,           //shift 
                        1'b0,           //load_pc
                        1'b0,           //reset_pc
                        1'b0,           //load_addr
                        1'b0,           //addr_sel
                        `MNONE,          //mem_cd
                        1'b0            //load_ir
                    };
            `halt: {vsel, nsel, write, loada, loadb, asel, bsel, loadc, loads, w, shift, load_pc, reset_pc, load_addr, addr_sel, mem_cd, load_ir} = { 
                        2'b00,          //vsel
                        3'b000,         //nsel
                        1'b0,           //write
                        1'b0,           //loada
                        1'b0,           //loadb
                        1'b0,           //asel
                        1'b0,           //bsel
                        1'b0,           //loadc
                        1'b0,           //loads
                        1'b0,           //w
                        2'b00,           //shift 
                        1'b0,           //load_pc
                        1'b0,           //reset_pc
                        1'b0,           //load_addr
                        1'b0,           //addr_sel
                        `MNONE,          //mem_cd
                        1'b0            //load_ir
                    };
            `get_A_state: {vsel, nsel, write, loada, loadb, asel, bsel, loadc, loads, w, shift, load_pc, reset_pc, load_addr, addr_sel, mem_cd, load_ir} = { 
                        2'b00,          //vsel
                        3'b100,         //nsel          nsel = 100 (selects Rn)
                        1'b0,           //write
                        1'b1,           //loada         loada = 1
                        1'b0,           //loadb
                        1'b0,           //asel
                        1'b0,           //bsel
                        1'b0,           //loadc
                        1'b0,           //loads
                        1'b0,           //w
                        2'b00,           //shift 
                        1'b0,           //load_pc
                        1'b0,           //reset_pc    
                        1'b0,           //load_addr
                        1'b0,           //addr_sel     
                        `MNONE,         //mem_cd        
                        1'b0           //load_ir 
                    };
            `get_B_state:  {vsel, nsel, write, loada, loadb, asel, bsel, loadc, loads, w, shift, load_pc, reset_pc, load_addr, addr_sel, mem_cd, load_ir} = { 
                        2'b00,          //vsel
                        3'b001,         //nsel          nsel = 001 //Rm
                        1'b0,           //write
                        1'b0,           //loada
                        1'b1,           //loadb         loadb = 1
                        1'b0,           //asel
                        1'b0,           //bsel
                        1'b0,           //loadc
                        1'b0,           //loads
                        1'b0,           //w
                        2'b00,          //shift 
                        1'b0,           //load_pc
                        1'b0,           //reset_pc    
                        1'b0,           //load_addr
                        1'b0,           //addr_sel     
                        `MNONE,         //mem_cd        
                        1'b0           //load_ir  
                    };
                    
            `get_B_reg_state: {vsel, nsel, write, loada, loadb, asel, bsel, loadc, loads, w, shift, load_pc, reset_pc, load_addr, addr_sel, mem_cd, load_ir} = { 
                        2'b00,          //vsel
                        3'b001,         //nsel          nsel = 010
                        1'b0,           //write
                        1'b0,           //loada
                        1'b1,           //loadb         loadb = 1
                        1'b0,           //asel
                        1'b0,           //bsel
                        1'b0,           //loadc
                        1'b0,           //loads
                        1'b0,           //w
                        2'b00,          //shift 
                        1'b0,           //load_pc
                        1'b0,           //reset_pc    
                        1'b0,           //load_addr
                        1'b0,           //addr_sel     
                        `MNONE,         //mem_cd        
                        1'b0           //load_ir  
                    };
            `mov_reg_state:  {vsel, nsel, write, loada, loadb, asel, bsel, loadc, loads, w, shift, load_pc, reset_pc, load_addr, addr_sel, mem_cd, load_ir} = { 
                        2'b00,          //vsel
                        3'b000,         //nsel
                        1'b0,           //write
                        1'b0,           //loada         
                        1'b0,           //loadb
                        1'b1,           //asel          asel = 1
                        1'b0,           //bsel          bsel = 0
                        1'b1,           //loadc         loadc = 1
                        1'b0,           //loads
                        1'b0,           //w
                        2'b00,          //shift          shift = 00
                        1'b0,           //load_pc
                        1'b0,           //reset_pc    
                        1'b0,           //load_addr
                        1'b0,           //addr_sel     
                        `MNONE,         //mem_cd        
                        1'b0           //load_ir 
                    };
            `add_state:  {vsel, nsel, write, loada, loadb, asel, bsel, loadc, loads, w, shift, load_pc, reset_pc, load_addr, addr_sel, mem_cd, load_ir} = { 
                        2'b00,          //vsel
                        3'b000,         //nsel
                        1'b0,           //write
                        1'b0,           //loada         
                        1'b0,           //loadb
                        1'b0,           //asel          asel = 0
                        1'b0,           //bsel          bsel = 0
                        1'b1,           //loadc         loadc = 1
                        1'b0,           //loads
                        1'b0,           //w
                        2'b00,          //shift          shift = 00
                        1'b0,           //load_pc
                        1'b0,           //reset_pc    
                        1'b0,           //load_addr
                        1'b0,           //addr_sel     
                        `MNONE,         //mem_cd        
                        1'b0           //load_ir 
                    };
            `cmp_state:  {vsel, nsel, write, loada, loadb, asel, bsel, loadc, loads, w, shift, load_pc, reset_pc, load_addr, addr_sel, mem_cd, load_ir} = { 
                        2'b00,          //vsel
                        3'b000,         //nsel
                        1'b0,           //write
                        1'b0,           //loada         
                        1'b0,           //loadb
                        1'b0,           //asel          asel = 0
                        1'b0,           //bsel          bsel = 0
                        1'b0,           //loadc         loadc = 0
                        1'b1,           //loads         loads = 1
                        1'b0,           //w
                        2'b00,          //shift          shift = 00
                        1'b0,           //load_pc
                        1'b0,           //reset_pc    
                        1'b0,           //load_addr
                        1'b0,           //addr_sel     
                        `MNONE,         //mem_cd        
                        1'b0           //load_ir 
                    };   
            `and_state:  {vsel, nsel, write, loada, loadb, asel, bsel, loadc, loads, w, shift, load_pc, reset_pc, load_addr, addr_sel, mem_cd, load_ir} = { 
                        2'b00,          //vsel
                        3'b000,         //nsel
                        1'b0,           //write
                        1'b0,           //loada         
                        1'b0,           //loadb
                        1'b0,           //asel          asel = 0
                        1'b0,           //bsel          bsel = 0
                        1'b1,           //loadc         loadc = 0
                        1'b0,           //loads         loads = 1
                        1'b0,           //w
                        2'b00,          //shift          shift = 00
                        1'b0,           //load_pc
                        1'b0,           //reset_pc    
                        1'b0,           //load_addr
                        1'b0,           //addr_sel     
                        `MNONE,         //mem_cd        
                        1'b0           //load_ir 
                    };      
            `mvn_state:  {vsel, nsel, write, loada, loadb, asel, bsel, loadc, loads, w, shift, load_pc, reset_pc, load_addr, addr_sel, mem_cd, load_ir} = { 
                        2'b00,          //vsel
                        3'b000,         //nsel
                        1'b0,           //write
                        1'b0,           //loada         
                        1'b0,           //loadb
                        1'b1,           //asel          asel = 0
                        1'b0,           //bsel          bsel = 0
                        1'b1,           //loadc         loadc = 1
                        1'b0,           //loads         loads = 1
                        1'b0,           //w
                        2'b00,          //shift          shift = 00
                        1'b0,           //load_pc
                        1'b0,           //reset_pc    
                        1'b0,           //load_addr
                        1'b0,           //addr_sel     
                        `MNONE,         //mem_cd        
                        1'b0           //load_ir 
                    };  
            `write_reg_state: {vsel, nsel, write, loada, loadb, asel, bsel, loadc, loads, w, shift, load_pc, reset_pc, load_addr, addr_sel, mem_cd, load_ir} = { 
                        2'b11,          //vsel          vsel = 11 (setting datapath out to register file in)
                        3'b010,         //nsel          nsel = 010
                        1'b1,           //write         write = 1
                        1'b0,           //loada
                        1'b0,           //loadb
                        1'b0,           //asel
                        1'b0,           //bsel
                        1'b0,           //loadc
                        1'b0,           //loads
                        1'b0,           //w
                        2'b00,          //shift 
                        1'b0,           //load_pc
                        1'b0,           //reset_pc    
                        1'b0,           //load_addr
                        1'b0,           //addr_sel     
                        `MNONE,         //mem_cd        
                        1'b0           //load_ir 
                    };
            `write_imm_state: {vsel, nsel, write, loada, loadb, asel, bsel, loadc, loads, w, shift, load_pc, reset_pc, load_addr, addr_sel, mem_cd, load_ir} = { 
                        2'b01,          //vsel          vsel = 01 (setting imm8 to register file in)
                        3'b100,         //nsel          nsel = 10 (Rn)
                        1'b1,           //write         write = 1
                        1'b0,           //loada
                        1'b0,           //loadb
                        1'b0,           //asel
                        1'b0,           //bsel
                        1'b0,           //loadc
                        1'b0,           //loads
                        1'b0,           //w
                        2'b00,          //shift 
                        1'b0,           //load_pc
                        1'b0,           //reset_pc    
                        1'b0,           //load_addr
                        1'b0,           //addr_sel     
                        `MNONE,         //mem_cd        
                        1'b0           //load_ir 
                    };
            `LDR0: {vsel, nsel, write, loada, loadb, asel, bsel, loadc, loads, w, shift, load_pc, reset_pc, load_addr, addr_sel, mem_cd, load_ir} = { 
                        2'b00,          //vsel
                        3'b100,         //nsel          nsel = 100 (Rn)
                        1'b0,           //write         write = 0
                        1'b1,           //loada         loada = 1
                        1'b0,           //loadb
                        1'b0,           //asel          
                        1'b0,           //bsel
                        1'b0,           //loadc
                        1'b0,           //loads
                        1'b0,           //w               
                        2'b00,          //shift  
                        1'b0,           //load_pc
                        1'b0,           //reset_pc    
                        1'b0,           //load_addr
                        1'b0,           //addr_sel      
                        `MNONE,         //mem_cd        MNONE
                        1'b0            //load_ir       
                    };
            `LDR1: {vsel, nsel, write, loada, loadb, asel, bsel, loadc, loads, w, shift, load_pc, reset_pc, load_addr, addr_sel, mem_cd, load_ir} = { 
                        2'b00,          //vsel
                        3'b000,         //nsel
                        1'b0,           //write
                        1'b0,           //loada
                        1'b0,           //loadb
                        1'b0,           //asel          
                        1'b1,           //bsel          bsel = 1 (sximm5)
                        1'b1,           //loadc         loadc = 1
                        1'b0,           //loads
                        1'b0,           //w               
                        2'b00,          //shift  
                        1'b0,           //load_pc
                        1'b0,           //reset_pc    
                        1'b0,           //load_addr
                        1'b0,           //addr_sel      
                        `MNONE,         //mem_cd        MNONE
                        1'b0            //load_ir       
                    };
            `LDR2: {vsel, nsel, write, loada, loadb, asel, bsel, loadc, loads, w, shift, load_pc, reset_pc, load_addr, addr_sel, mem_cd, load_ir} = { 
                        2'b00,          //vsel
                        3'b000,         //nsel
                        1'b0,           //write
                        1'b0,           //loada
                        1'b0,           //loadb
                        1'b0,           //asel          
                        1'b0,           //bsel
                        1'b0,           //loadc
                        1'b0,           //loads
                        1'b0,           //w               
                        2'b00,          //shift  
                        1'b0,           //load_pc
                        1'b0,           //reset_pc    
                        1'b1,           //load_addr     load_addr = 1
                        1'b0,           //addr_sel      addr_sel = 0
                        `MREAD,         //mem_cd        MREAD
                        1'b0            //load_ir       
                    };
            `LDR3: {vsel, nsel, write, loada, loadb, asel, bsel, loadc, loads, w, shift, load_pc, reset_pc, load_addr, addr_sel, mem_cd, load_ir} = { 
                //write back 
                        2'b00,          //vsel         vsel = 00 (mdata)
                        3'b010,         //nsel         nsel = 010 (rd)
                        1'b0,           //write        write = 1
                        1'b0,           //loada
                        1'b0,           //loadb
                        1'b0,           //asel          
                        1'b0,           //bsel
                        1'b0,           //loadc
                        1'b0,           //loads
                        1'b0,           //w               
                        2'b00,          //shift  
                        1'b0,           //load_pc
                        1'b0,           //reset_pc    
                        1'b0,           //load_addr
                        1'b0,           //addr_sel     
                        `MNONE,         //mem_cd        
                        1'b0           //load_ir       
                    };
            `STR0: {vsel, nsel, write, loada, loadb, asel, bsel, loadc, loads, w, shift, load_pc, reset_pc, load_addr, addr_sel, mem_cd, load_ir} = { 
                        2'b00,          //vsel
                        3'b100,         //nsel = 100
                        1'b0,           //write
                        1'b1,           //loada = 1
                        1'b0,           //loadb
                        1'b0,           //asel
                        1'b0,           //bsel
                        1'b0,           //loadc
                        1'b0,           //loads
                        1'b0,           //w
                        2'b00,           //shift 
                        1'b0,           //load_pc
                        1'b0,           //reset_pc
                        1'b0,           //load_addr
                        1'b0,           //addr_sel
                        `MNONE,          //mem_cd
                        1'b0            //load_ir
                    };
            `STR1: {vsel, nsel, write, loada, loadb, asel, bsel, loadc, loads, w, shift, load_pc, reset_pc, load_addr, addr_sel, mem_cd, load_ir} = { 
                        2'b00,          //vsel
                        3'b000,         //nsel
                        1'b0,           //write
                        1'b0,           //loada
                        1'b0,           //loadb
                        1'b1,           //asel = 1
                        1'b1,           //bsel = 1
                        1'b0,           //loadc
                        1'b0,           //loads
                        1'b0,           //w
                        2'b00,           //shift 
                        1'b0,           //load_pc
                        1'b0,           //reset_pc
                        1'b0,           //load_addr
                        1'b0,           //addr_sel
                        `MNONE,          //mem_cd
                        1'b0            //load_ir
                    };
            `STR2: {vsel, nsel, write, loada, loadb, asel, bsel, loadc, loads, w, shift, load_pc, reset_pc, load_addr, addr_sel, mem_cd, load_ir} = { 
                        2'b00,          //vsel
                        3'b000,         //nsel
                        1'b0,           //write
                        1'b0,           //loada
                        1'b0,           //loadb
                        1'b0,           //asel
                        1'b0,           //bsel
                        1'b0,           //loadc
                        1'b0,           //loads
                        1'b0,           //w
                        2'b00,           //shift 
                        1'b0,           //load_pc
                        1'b0,           //reset_pc
                        1'b1,           //load_addr = 1
                        1'b0,           //addr_sel
                        `MWRITE,          //mem_cd = MWRITE
                        1'b0            //load_ir
                    };
            `STR3: {vsel, nsel, write, loada, loadb, asel, bsel, loadc, loads, w, shift, load_pc, reset_pc, load_addr, addr_sel, mem_cd, load_ir} = { 
                        2'b00,          //vsel
                        3'b000,         //nsel 010
                        1'b0,           //write
                        1'b0,           //loada
                        1'b1,           //loadb =1
                        1'b0,           //asel
                        1'b0,           //bsel
                        1'b0,           //loadc
                        1'b0,           //loads
                        1'b0,           //w
                        2'b00,           //shift 
                        1'b0,           //load_pc
                        1'b0,           //reset_pc
                        1'b0,           //load_addr
                        1'b0,           //addr_sel
                        `MNONE,          //mem_cd
                        1'b0            //load_ir
                    };
            `STR4: {vsel, nsel, write, loada, loadb, asel, bsel, loadc, loads, w, shift, load_pc, reset_pc, load_addr, addr_sel, mem_cd, load_ir} = { 
                        2'b00,          //vsel
                        3'b000,         //nsel
                        1'b0,           //write
                        1'b0,           //loada
                        1'b0,           //loadb
                        1'b1,           //asel = 1
                        1'b0,           //bsel
                        1'b1,           //loadc = 1
                        1'b0,           //loads
                        1'b0,           //w
                        2'b00,           //shift 
                        1'b0,           //load_pc
                        1'b0,           //reset_pc
                        1'b0,           //load_addr
                        1'b0,           //addr_sel
                        `MNONE,          //mem_cd
                        1'b0            //load_ir
                    };
            `STR5: {vsel, nsel, write, loada, loadb, asel, bsel, loadc, loads, w, shift, load_pc, reset_pc, load_addr, addr_sel, mem_cd, load_ir} = { 
                        2'b00,          //vsel
                        3'b000,         //nsel
                        1'b0,           //write
                        1'b0,           //loada
                        1'b0,           //loadb
                        1'b0,           //asel
                        1'b0,           //bsel
                        1'b0,           //loadc
                        1'b0,           //loads
                        1'b0,           //w
                        2'b00,           //shift 
                        1'b0,           //load_pc
                        1'b0,           //reset_pc
                        1'b0,           //load_addr
                        1'b0,           //addr_sel
                        `MWRITE,          //mem_cd = mwrite
                        1'b0            //load_ir
                    };
            default: {vsel, nsel, write, loada, loadb, asel, bsel, loadc, loads, w, shift, load_pc, reset_pc, load_addr, addr_sel, mem_cd, load_ir} = {22{1'bx}};
        endcase
    
    
    end
endmodule

//decodes instruction into signals




