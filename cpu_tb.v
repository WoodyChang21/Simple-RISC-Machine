module cpu_tb();
  reg clk, reset, s, load;
  reg [15:0] in;
  wire [15:0] out;
  wire N,V,Z,w;

  reg err;

  wire [8:0] PC;
  wire [4:0] present_state;
  wire [8:0] mem_addr;
  wire [1:0] mem_cmd;

  cpu DUT2(clk,dp_mdata,reset,s,load,in,out,N,V,Z,w,mem_cmd,mem_addr);

  initial begin
    clk = 0; #5;
    forever begin
      clk = 1; #5;
      clk = 0; #5;
    end
  end

  initial begin
    // Test case 1 for MOV, MOV SH, ADD
    err = 0;
    reset = 1; s = 0; load = 0; in = 16'b0;
    #10;
    reset = 0; 
    #10;

    in = 16'b1101000000000111;
    load = 1;
    #10;
    load = 0;
    s = 1;
    #10
    s = 0;
    @(posedge w); // wait for w to go high again
    #10;
    $display("Register 0 contains %b", cpu_tb.DUT2.DP.REGFILE.R0);
    if (cpu_tb.DUT2.DP.REGFILE.R0 !== 16'h7) begin
      err = 1;
      $display("FAILED: MOV R0, #7");
      $stop;
    end

    in = 16'b1100000000101000;
    load = 1;
    #10;
    load = 0;
    s = 1;
    #10
    s = 0;
    @(posedge w); // wait for w to go high again
    #10;
    $display("Register 1 contains %b", cpu_tb.DUT2.DP.REGFILE.R1);
    if (cpu_tb.DUT2.DP.REGFILE.R1 !== 16'hE) begin
      err = 1;
      $display("FAILED: MOV R1, LSL#1");
      $stop;
    end

    in = 16'b1010000101001000;
    load = 1;
    #10;
    load = 0;
    s = 1;
    #10
    s = 0;
    @(posedge w); // wait for w to go high again
    #10;
    $display("Register 2 contains %b", cpu_tb.DUT2.DP.REGFILE.R2);
    if (cpu_tb.DUT2.DP.REGFILE.R2 !== 16'h1C) begin
      err = 1;
      $display("FAILED: ADD R2, R1, R0, LSL#1");
      $stop;
    end
    if (~err) $display("Test case 1 passed");

    // Test case 2 for MOV, CMP
    in = 16'b1101001100000111;
    load = 1;
    #10;
    load = 0;
    s = 1;
    #10
    s = 0;
    @(posedge w); // wait for w to go high again
    #10;
    $display("Register 3 contains %b", cpu_tb.DUT2.DP.REGFILE.R3);
    if (cpu_tb.DUT2.DP.REGFILE.R3 !== 16'h7) begin
      err = 1;
      $display("FAILED: MOV R3, #7");
      $stop;
    end

    in = 16'b1010101010101000;
    load = 1;
    #10;
    load = 0;
    s = 1;
    #10
    s = 0;
    @(posedge w); // wait for w to go high again
    #10;
    $display("Register 5 contatins %b", cpu_tb.DUT2.DP.REGFILE.R5);
    if (cpu_tb.DUT2.DP.REGFILE.R5 !== 16'hE) begin
      err = 1;
      $display("FAILED: CMP R2, R0, LSL#1");
      $stop;
    end
    $display("stat_out is %b", cpu_tb.DUT2.stat_out);
    if (cpu_tb.DUT2.stat_out !== 3'b000) begin
      err = 1;
      $display("FAILED: CMP R2, R0, LSL#1");
      $stop;
    end

    in = 16'b1010100011000001;
    load = 1;
    #10;
    load = 0;
    s = 1;
    #10
    s = 0;
    @(posedge w); // wait for w to go high again
    #10;
    $display("Register 6 contatins %b", cpu_tb.DUT2.DP.REGFILE.R6);
    if (cpu_tb.DUT2.DP.REGFILE.R6 !== 16'b1111111111111001) begin
      err = 1;
      $display("FAILED: CMP R0, R1");
      $stop;
    end
    $display("stat_out is %b", cpu_tb.DUT2.stat_out);
    if (cpu_tb.DUT2.stat_out !== 3'b010) begin
      err = 1;
      $display("FAILED: CMP R2, R0");
      $stop;
    end

    in = 16'b1010101110000000;
    load = 1;
    #10;
    load = 0;
    s = 1;
    #10
    s = 0;
    @(posedge w); // wait for w to go high again
    #10;
    $display("Register 4 contatins %b", cpu_tb.DUT2.DP.REGFILE.R4);
    if (cpu_tb.DUT2.DP.REGFILE.R4 !== 16'h0) begin
      err = 1;
      $display("FAILED: CMP R3, R0");
      $stop;
    end
    $display("stat_out is %b", cpu_tb.DUT2.stat_out);
    if (cpu_tb.DUT2.stat_out !== 3'b001) begin
      err = 1;
      $display("FAILED: CMP R3, R0");
      $stop;
    end
    if (~err) $display("Test case 2 passed");

   // Test case 3 for MVN, AND
    in = 16'b1011100011100110;
    load = 1;
    #10;
    load = 0;
    s = 1;
    #10
    s = 0;
    @(posedge w); // wait for w to go high again
    #10;
    $display("Register 7 contains %b", cpu_tb.DUT2.DP.REGFILE.R7);
    if (cpu_tb.DUT2.DP.REGFILE.R7 !== 16'h6) begin
      err = 1;
      $display("FAILED: MVN R7, R6");
      $stop;
    end

    in = 16'b1011011011101101;
    load = 1;
    #10;
    load = 0;
    s = 1;
    #10
    s = 0;
    @(posedge w); // wait for w to go high again
    #10;
    $display("Register 7 contains %b", cpu_tb.DUT2.DP.REGFILE.R7);
    if (cpu_tb.DUT2.DP.REGFILE.R7 !== 16'h18) begin
      err = 1;
      $display("FAILED: AND R6, R5");
      $stop;
    end
    if (~err) $display("Test case 3 passed");

    // Overflow case test for CMP
    in = 16'b1101000000000001;
    load = 1;
    #10;
    load = 0;
    s = 1;
    #10
    s = 0;
    @(posedge w); // wait for w to go high again
    #10;
    $display("Register 0 contains %b", cpu_tb.DUT2.DP.REGFILE.R0);
    if (cpu_tb.DUT2.DP.REGFILE.R0 !== 16'b0000000000000001) begin
      err = 1;
      $display("FAILED: MOV R0");
      $stop;
    end

    in = 16'b1100000000011000;
    load = 1;
    #10;
    load = 0;
    s = 1;
    #10
    s = 0;
    @(posedge w); // wait for w to go high again
    #10;
    $display("Register 0 contains %b", cpu_tb.DUT2.DP.REGFILE.R0);
    if (cpu_tb.DUT2.DP.REGFILE.R0 !== 16'b1000000000000000) begin
      err = 1;
      $display("FAILED: MOV R0, R0, ASR#1");
      $stop;
    end

    in = 16'b1100000000110000;
    load = 1;
    #10;
    load = 0;
    s = 1;
    #10
    s = 0;
    @(posedge w); // wait for w to go high again
    #10;
    $display("Register 1 contains %b", cpu_tb.DUT2.DP.REGFILE.R1);
    if (cpu_tb.DUT2.DP.REGFILE.R1 !== 16'b0100000000000000) begin
      err = 1;
      $display("FAILED: MOV R1, R0, LSR#1");
      $stop;
    end

    in = 16'b1011100001000001;
    load = 1;
    #10;
    load = 0;
    s = 1;
    #10
    s = 0;
    @(posedge w); // wait for w to go high again
    #10;
    $display("Register 2 contains %b", cpu_tb.DUT2.DP.REGFILE.R2);
    if (cpu_tb.DUT2.DP.REGFILE.R2 !== 16'b1011111111111111) begin
      err = 1;
      $display("FAILED: MVN R2, R1");
      $stop;
    end

    in = 16'b1010100101100010;
    load = 1;
    #10;
    load = 0;
    s = 1;
    #10
    s = 0;
    @(posedge w); // wait for w to go high again
    #10;
    $display("Register 3 contains %b", cpu_tb.DUT2.DP.REGFILE.R3);
    if (cpu_tb.DUT2.DP.REGFILE.R3 !== 16'b1000000000000001) begin
      err = 1;
      $display("FAILED: CMP R1, R2");
      $stop;
    end
    $display("stat_out is %b", cpu_tb.DUT2.stat_out);
    if (cpu_tb.DUT2.DP.alu.stat !== 3'b110) begin
      err = 1;
      $display("FAILED: CMP R1, R2");
      $stop;
    end
    if (~err) $display("Test case 4 passed");

   // Test case 5 for MVN, AND, ADD
    in = 16'b1011100010000001;
    load = 1;
    #10;
    load = 0;
    s = 1;
    #10
    s = 0;
    @(posedge w); // wait for w to go high again
    #10;
    $display("Register 4 contains %b", cpu_tb.DUT2.DP.REGFILE.R4);
    if (cpu_tb.DUT2.DP.REGFILE.R4 !== 16'b1011111111111111) begin
      err = 1;
      $display("FAILED: MVN R4, R1");
      $stop;
    end

    in = 16'b1011010010100001;
    load = 1;
    #10;
    load = 0;
    s = 1;
    #10
    s = 0;
    @(posedge w); // wait for w to go high again
    #10;
    $display("Register 5 contains %b", cpu_tb.DUT2.DP.REGFILE.R5);
    if (cpu_tb.DUT2.DP.REGFILE.R5 !== 16'b0000000000000000) begin
      err = 1;
      $display("FAILED: AND R5, R4, R1");
      $stop;
    end

    in = 16'b1010010011000001;
    load = 1;
    #10;
    load = 0;
    s = 1;
    #10
    s = 0;
    @(posedge w); // wait for w to go high again
    #10;
    $display("Register 6 contains %b", cpu_tb.DUT2.DP.REGFILE.R6);
    if (cpu_tb.DUT2.DP.REGFILE.R6 !== 16'b1111111111111111) begin
      err = 1;
      $display("FAILED: ADD R6, R4, R1");
      $stop;
    end
    if (~err) $display("Test case 5 passed");

    // Test Case 6 for MVN, ADD, AND
    in = 16'b1011100011100110;
    load = 1;
    #10;
    load = 0;
    s = 1;
    #10
    s = 0;
    @(posedge w); // wait for w to go high again
    #10;
    $display("Register 4 contains %b", cpu_tb.DUT2.DP.REGFILE.R7);
    if (cpu_tb.DUT2.DP.REGFILE.R7 !== 16'b0000000000000000) begin
      err = 1;
      $display("FAILED: MVN R7, R6");
      $stop;
    end

    in = 16'b1011011000000011;
    load = 1;
    #10;
    load = 0;
    s = 1;
    #10
    s = 0;
    @(posedge w); // wait for w to go high again
    #10;
    $display("Register 5 contains %b", cpu_tb.DUT2.DP.REGFILE.R0);
    if (cpu_tb.DUT2.DP.REGFILE.R0 !== 16'b1000000000000001) begin
      err = 1;
      $display("FAILED: AND R0, R6, R3");
      $stop;
    end

    in = 16'b1010001001000001;
    load = 1;
    #10;
    load = 0;
    s = 1;
    #10
    s = 0;
    @(posedge w); // wait for w to go high again
    #10;
    $display("Register 6 contains %b", cpu_tb.DUT2.DP.REGFILE.R2);
    if (cpu_tb.DUT2.DP.REGFILE.R2 !== 16'b1111111111111111) begin
      err = 1;
      $display("FAILED: ADD R2, R2, R1");
      $stop;
    end
    if (~err) $display("Test case 6 passed");
    // Display test passed if successful
    if (~err) $display("All tests passed");
    $stop;
  end
endmodule
