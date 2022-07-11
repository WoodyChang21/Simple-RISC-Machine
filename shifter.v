module shifter(in,shift,sout);
    input   [15:0] in;
    input   [1:0] shift;
    output  [15:0] sout;

    reg [15:0] sout;
    reg tempbm;

    always @(*) begin
        tempbm = in[15]; // saves the MSB of input in case of shift = 2'b11 operation

        case(shift)      // preform specific operation based on value of shift
            2'b00: sout = in; 
            2'b01: sout = in << 1;  // multiply by 2
            2'b10: sout = in >> 1;  // divide by 2
            2'b11: sout = in >> 1; // shifts right by 1 however 15th bit (MSB) is 0 
            default: sout = {16{1'bx}};
        endcase
        
        if (shift == 2'b11)  // Set MSB of sout to B[15] 
            sout[15] = tempbm;
    end
endmodule