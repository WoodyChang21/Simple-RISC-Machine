`define MREAD 2'b01
`define MWRITE 2'b10
`define MNONE 2'b00

module lab7_top(KEY,SW,LEDR,HEX0,HEX1,HEX2,HEX3,HEX4,HEX5);
input [3:0] KEY;
input [9:0] SW;
output [9:0] LEDR;
output [6:0] HEX0, HEX1, HEX2, HEX3, HEX4, HEX5;


wire [1:0] mem_cmd;
wire [8:0] mem_addr;
wire [15:0] read_data,write_data;
wire [15:0] dout;
wire enable;
reg msel,mread,mwrite;
wire write;

cpu CPU(.clk      (~KEY[0]),
        .dp_mdata (read_data),
             .reset    (~KEY[1]), 
             .s        (~KEY[2]),
             .load     (~KEY[3]),
             .in       (read_data),
             .out      (write_data),
             .Z        (Z),
             .N        (N),
             .V        (V),
             .w        (LEDR[9]),
             .mem_cmd  (mem_cmd),
             .mem_addr (mem_addr)  );
//(clk,dp_mdata,reset,s,load,in,out,N,V,Z,w,mem_cmd,mem_addr);

always@(*) begin
    msel = mem_addr[8] & 1'b0;
    mread = (mem_cmd[0] & 1'b1) & (mem_cmd[1] & 1'b0);
    mwrite = (mem_cmd[0] & 1'b0) & (mem_cmd[1] & 1'b1);
end

assign enable = msel & mread;
assign write = mwrite & msel; //in write mode and writing into effective adress 
assign read_data = enable? dout: {16{1'bz}};

sseg H0(write_data[3:0],   HEX0);
sseg H1(write_data[7:4],   HEX1);
sseg H2(write_data[11:8],  HEX2);
sseg H3(write_data[15:12], HEX3);


RAM #(16,8,"data.txt") MEM(.clk           (~KEY[0]),
                           .read_address  (mem_addr[7:0]),
                           .write_address (mem_addr[7:0]),
                           .write         (write),
                           .din           (write_data),
                           .dout          (dout)   );
LED ledr (.clk         (~KEY[0]),
        .mem_cmd     (mem_cmd),
        .mem_addr    (mem_addr),
        .write_data  (write_data[7:0]),
        .LEDR        (LEDR[7:0])  );

switch stch (.mem_cmd     (mem_cmd),
          .mem_addr    (mem_addr),
          .SW          (SW[7:0]),
          .read_data   (read_data)  );

endmodule




module RAM(clk,read_address,write_address,write,din,dout);
    parameter data_width = 32;
    parameter addr_width = 4;
    parameter filename = "data.txt";

    input clk; 
    input [addr_width-1:0] read_address, write_address;
    input write;
    input [data_width-1:0] din;
    output reg [data_width-1:0] dout;

    reg [data_width-1:0] mem [2**addr_width-1:0];

    initial $readmemb(filename, mem);

    always @(posedge clk) begin
        if (write)
        mem[write_address] <= din;
        dout <= mem[read_address];
    end

endmodule

module sseg(in,segs);
  input [3:0] in;
  output [6:0] segs;
  reg [6:0] segs;

  always @(*) begin
    case (in)
        4'b0000: segs = 7'b1000000;
        4'b0001: segs = 7'b1111001;
     4'b0010: segs = 7'b0100100;
     4'b0011: segs = 7'b0110000;
     4'b0100: segs = 7'b0011001;
     4'b0101: segs = 7'b0010010;
     4'b0110: segs = 7'b0000010;
     4'b0111: segs = 7'b1111000;
     4'b1000: segs = 7'b0000000;
     4'b1001: segs = 7'b0011000;
     4'b1010: segs = 7'b0001000;
     4'b1011: segs = 7'b0000011;
     4'b1100: segs = 7'b1000110;
     4'b1101: segs = 7'b0100001;
     4'b1110: segs = 7'b0000110;
     4'b1111: segs = 7'b0001110;
     default: segs = 7'b1111111;
     endcase
    end
endmodule

module switch (SW,mem_cmd,mem_addr,read_data);
    input [7:0] SW;
    input [1:0] mem_cmd;
    input [8:0] mem_addr;

    output [15:0] read_data;
    reg tri_load;

    wire [7:0] low; //lower 8 bits
    wire [7:0] high; // higher 8 bits
    
    always@(*) begin
        if((mem_cmd == `MREAD)& (mem_addr == 9'h140)) begin
            tri_load = 1'b1;
        end
        else begin
            tri_load = 1'b0;
        end
    end

    assign low = tri_load? SW[7:0]: {8{1'bz}};
    assign high = tri_load? 8'h00: {8{1'bz}};

    assign read_data = {high,low};
endmodule

module LED (clk,write_data,mem_cmd,mem_addr,LEDR);
    input [7:0] write_data;
    input [1:0] mem_cmd;
    input [8:0] mem_addr;
    input clk;

    reg load;

    output [7:0] LEDR;

    always@(*) begin
        if ((mem_cmd == `MWRITE) & (mem_addr == 9'h100)) begin
            load = 1'b1;
        end
        else begin
            load = 1'b0;
        end
    end

    vDFFE #(8) LEDRR (clk,load,write_data,LEDR);
endmodule

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

