module regfile(data_in,writenum,write,readnum,clk,data_out);
    input   [15:0] data_in;
    input   [2:0] writenum, readnum;
    input   write, clk;
    output  [15:0] data_out;
    
    reg [15:0] data_out;
    
    wire [7:0] writenum_decoded, and_out, readnum_decoded;
    wire [15:0] R0, R1, R2, R3, R4, R5, R6, R7;
    

    three_eight_dec writedec (writenum, writenum_decoded); //double check syntax

    assign and_out = {write & writenum_decoded[7], 
                       write & writenum_decoded[6],
                       write & writenum_decoded[5],
                       write & writenum_decoded[4],
                       write & writenum_decoded[3],
                       write & writenum_decoded[2],
                       write & writenum_decoded[1],
                       write & writenum_decoded[0]
                    };

    vDFFE register0 (clk, and_out[0], data_in, R0);
    vDFFE register1 (clk, and_out[1], data_in, R1);
    vDFFE register2 (clk, and_out[2], data_in, R2);
    vDFFE register3 (clk, and_out[3], data_in, R3);
    vDFFE register4 (clk, and_out[4], data_in, R4);
    vDFFE register5 (clk, and_out[5], data_in, R5);
    vDFFE register6 (clk, and_out[6], data_in, R6);
    vDFFE register7 (clk, and_out[7], data_in, R7);

    three_eight_dec readdec (readnum, readnum_decoded); //double check syntax

    always @(*) begin 
        case(readnum_decoded)
            8'b00000001: data_out = R0; 
            8'b00000010: data_out = R1;
            8'b00000100: data_out = R2; 
            8'b00001000: data_out = R3; 
            8'b00010000: data_out = R4; 
            8'b00100000: data_out = R5;
            8'b01000000: data_out = R6; 
            8'b10000000: data_out = R7; 
            default: data_out = {(16){1'bx}};
        endcase
    end
endmodule

//converts 3 bit binary to 8 bit one hot code
module three_eight_dec(in, out);
    input [2:0] in;
    output [7:0] out;
    reg [7:0] out;

    always @(*) begin
            case (in) //converting 3 bit binary to 2^3 = 8 bit one hot code 
                3'd0: out = 8'b00000001; 
                3'd1: out = 8'b00000010; 
                3'd2: out = 8'b00000100; 
                3'd3: out = 8'b00001000;
                3'd4: out = 8'b00010000;
                3'd5: out = 8'b00100000;
                3'd6: out = 8'b01000000;
                3'd7: out = 8'b10000000;
                default: out = {8{1'bx}};
            endcase
    end
endmodule

//Verilog for n-bit D Flip flop with load enable: TAKEN FROM lab5 SS
module vDFFE(clk, en, in, out);
    parameter n = 16;  // width
    input  clk, en;
    input  [n-1:0] in ;
    output [n-1:0] out ;
    reg    [n-1:0] out ;
    wire   [n-1:0] next_out ;

    assign next_out = en ? in : out;

    always @(posedge clk)
        out = next_out;  
endmodule
