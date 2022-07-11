module ALU(Ain,Bin,ALUop,out,status);
    input [15:0] Ain, Bin;
    input [1:0] ALUop;
    output [15:0] out;
    reg [15:0] out;
    output [2:0] status;
    wire overflow;
    wire z_flag;
    wire [15:0] addsub_output;
    reg sub;

    AddSub addsub(Ain, Bin, sub, addsub_output, overflow);

    //sub determines whether add/sub unit subtracts or adds
    //out is the output of the addsub module
    always @(*) begin  
        case(ALUop) 
            2'b00: {sub, out} = {1'b0, Ain + Bin};
            2'b01: {sub, out} = {1'b1, Ain - Bin};
            2'b10: {sub, out} = {1'b0, Ain & Bin};  //no sub for AND operation
            2'b11: {sub, out} = {1'b0, ~Bin};       //no sub for NOT operation
            default: {sub, out} = {17{1'bx}};
        endcase        
    end

    assign z_flag = (out == 16'd0) ? 1'b1 : 1'b0;
    //status output = { N, V, Z } 
    assign status = {out[15], overflow, z_flag};
endmodule

//CODE TAKEN FROM DALLY p. 221 Figure 10.14
//add/subtract unit with overflow detection
module AddSub(a, b, sub, s, ovf) ;
    input [15:0] a,b;
    input sub ; //1 if subtracting, 0 if adding
    output [15:0] s;
    output ovf;
    wire c1, c2;
    assign ovf = c1 ^ c2; //XOR to see if signs match

    // add non sign bits
    assign {c1, s[14:0]} = a[14:0] + (b[14:0] ^ {15{sub}}) + sub;
    // add sign bits
    assign {c2, s[15]} = a[15] + (b[15] ^ sub) + c1;
endmodule




    