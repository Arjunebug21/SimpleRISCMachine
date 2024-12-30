module cpu_tb();
reg clk, reset, load;
reg [15:0] read_data;
reg err = 0;

wire [15:0] out;
wire N, V, Z;
wire [1:0] mem_cmd;
wire [8:0] mem_addr;


cpu DUT(.clk(clk), .reset(reset), .out(out), .N(N), .V(V), .Z(Z), .mem_cmd(mem_cmd), .mem_addr(mem_addr), .read_data(read_data));

task Auto;

input [15:0] instruction_here;

read_data = instruction_here;

#5;


endtask

// task ADD;
//     input [15:0] instr_here;
//     input [15:0] expected_data_out;
//     input expected_z, expected_v, expected_n;
//     in = instr_here;
//     load = 1;
//     #10;
//     load = 0;
//     s = 1;
//     #10;
//     s = 0;

//     if (out !== expected_data_out && (Z !== expected_z || V !== expected_v || N !== expected_n)) begin
//         $display("**ERROR** output is %b and should be %b, also *Error* Z, V, or N aren't right. Z: %b should be %b, V: %b should be %b, N: %b, should be %b", out, expected_data_out, Z, expected_z, V, expected_v, N, expected_n);
//         err = 1;
//     end else if (Z !== expected_z || V !== expected_v || N !== expected_n)begin
//         err = 1;
//         $display("*Error* Z, V, or N aren't right. Z: %b should be %b, V: %b should be %b, N: %b, should be %b", Z, expected_z, V, expected_v, N, expected_n);
//     end else begin
//         $display("SUCCESS! output change to %b works", out);
//     end
// endtask

initial begin
clk = 0; #5;
forever begin
    clk = 1; #5;
    clk = 0; #5;
end
end

initial begin
    reset = 1; 
    #10;
    reset = 0;
    #5;
    Auto(16'b1101000000000111);  // this means, take the absolute number 7 and store it in R0
    #30;
    Auto(16'b1101000100000010); //Put 2 in R1
    #100;
    Auto(16'b1010000001000001); //Add R0 and R1 and put it in R2
    #100;
    Auto(16'b1100000001100010); //Move what's in R2 (9) into R3
    #1000;
    $stop;

    // $display("Value in R0 is: %b, should be %b", DUT.DP.REGFILE.R0, 16'b111);
    // if (DUT.DP.REGFILE.R0 !== 16'b111) begin
    //     err = 1;
    // end

    // #30;
    // if (DUT.DP.REGFILE.R1 != 16'b10) begin
    //     err = 1;
    //     $display("Moving 2 into R1 didn't work at line 77");
    // end else begin
    //     $display("Moving 2 into R1 works!");
    // end
    // //$display("Value in R0 is: %b", DUT.DP.REGFILE.R0);
    // Auto(16'b1101011111111111); //Check that sign extend works - Put biggest number in R7
    // #50;
    // if (DUT.DP.REGFILE.R7 !== {16{1'b1}}) begin
    //     err = 1;
    //     $display("Shift didn't work correctly");
    // end
    // #30;
    // //$display("Value in R0 is: %b", DUT.DP.REGFILE.R0);
    // Auto(16'b1100000001100000); //Move whats in R0 (7) into R3
    // #105;
    // reset = 0;
    // //$display("Value in R0 is: %b", DUT.DP.REGFILE.R0);
    // Auto(16'b1100000010000001); //Move what's in R1 into R4
    // #105;
    // //$display("Value in R0 is: %b", DUT.DP.REGFILE.R0);
    // Auto(16'b1101011000000010); //Put 2 in R6
    // #100;
    // //$display("Value in R0 is: %b", DUT.DP.REGFILE.R0);
    // Auto(16'b1100000000000110); //Move whats in R0 (7) into R3
    // #105;
    // if (DUT.DP.REGFILE.R3 !== 16'b111) begin
    //     err = 1;
    //     $display("R3 does not have 7 it has %b", DUT.DP.REGFILE.R3);
    // end else begin
    //     $display("YIPPEE R3!");
    // end
    // Auto(16'b1010000100000110); //R0 = 2 + 2 = 4; 
    // #200;
    // if (DUT.DP.REGFILE.R0 !== 16'b100) begin
    // $display("Value in R0 is: %b and should be %b", DUT.DP.REGFILE.R0, 16'b100);
    // err = 1;
    // end else begin
    // $display("Yippee, R0 has 4!");
    // end
    // if (out !== 16'b100) begin
    //     $display("Gate level simulation of out when adding 2+2 is incorrect");
    //     err = 1;
    // end else begin
    //     $display("Gate Level Simulation of out works when adding 2+2");
    // end
    // Auto(16'b1101000011111101); //Mov -3 to R0;
    // #30;
    // Auto(16'b1101000100000011); //Mov 3 to R1;
    // #50;
    // Auto(16'b1010000001000001); //Add R0 and R1 should = 0;
    // #200;
    // if (DUT.DP.REGFILE.R2 !== 16'b0) begin
    //     err = 1;
    //     $display("Addition of 3 and -3 is incorrect");
    // end
    // Auto(16'b1010000101000001); //Add R1 and R1, R2 should = 6;
    // #200;
    // if (DUT.DP.REGFILE.R2 !== 16'b110) begin
    //     err = 1;
    //     $display("Addition of R1 and R1 is incorrect. R2 = %b and should be 6", DUT.DP.REGFILE.R2);
    // end else begin
    //     $display("Addition of R1 with itself worked");
    // end
    // Auto(16'b1100000001100001); //Mov what's in R1 into R3 so Move 3 to R3
    // #105;
    // if (DUT.DP.REGFILE.R3 != 16'b11) begin
    //     err = 1;
    //     $display("Move from Reg to Reg didn't work - R3 isn't 3");
    // end else begin
    //     $display("YIPPEE Reg 3 has 3 inside");
    // end
    // Auto(16'b1010100100000011); //CMP R1 (3) and R3 (3). Z should be on
    // #200;
    // if (Z !== 1'b1) begin
    //     err = 1;
    //     $display("Z is not 1 when it should be Comparing 3 and 3");
    // end else begin
    //     $display("CMP works with 3 and 3 Z is on");
    // end
    // Auto(16'b1010101000000001); //CMP R2(6) and R1(3) Z = 0
    // #200;
    // if (Z !== 1'b0) begin
    //     err = 1;
    //     $display("Z is not 0 when it should be Comparing 6 and 3");
    // end else begin
    //     $display("CMP works with 6 and 3, Z is off");
    // end
    // Auto(16'b1010101000001001); //CMP R2(6) and R1 shifted by 1 (3*2). Z = 1
    // #100;
    // if (Z !== 1'b1 || V !== 1'b0 || N !== 1'b0) begin
    //     err = 1;
    //     $display("Z is not 1 when it should be Comparing 6 and 3 shifted by 1");
    // end else begin
    //     $display("CMP works with 6 and 3 shifted by 1, Z is on, V is off, N is off");
    // end
    // Auto(16'b1101000011111111); //Move 11111111 into R0
    // #50;
    // if (DUT.DP.REGFILE.R0 !== {16{1'b1}}) begin
    //     err = 1;
    //     $display("R0 Doesn't have all 1's when it should");
    // end else begin
    //     $display("YIPPEE Reg 0 has all 1's inside");
    // end
    // Auto(16'b1101000111111111); //Move 11111111 into R1
    // #50;
    // if (DUT.DP.REGFILE.R1 !== {16{1'b1}}) begin
    //     err = 1;
    //     $display("R1 Doesn't have all 1's when it should");
    // end else begin
    //     $display("YIPPEE Reg 1 has all 1's inside");
    // end
    // Auto(16'b1011000001000001); //AND R0 and R1 should pass 1111111111111111
    // #200;
    // if (DUT.DP.REGFILE.R2 !== {16{1'b1}}) begin
    //     err = 1;
    //     $display("R2 Doesn't have all 1's when it should");
    // end else begin
    //     $display("YIPPEE When using AND Reg 2 has all 1's inside");
    // end   
    // Auto(16'b1011001100000100); //R0: AND R3 and R4 should be 16'b10
    // #200;
    // if (DUT.DP.REGFILE.R0 !== 16'b10) begin
    //     err = 1;
    //     $display("R0 When using AND is incorrect");
    // end else begin
    //     $display("YIPPEE Reg 0 when using AND is correct");
    // end
    // Auto(16'b1011000000101100); //Put into R1: R0 & (R4 shifted by 1) should be all 0
    // #200;
    // if (DUT.DP.REGFILE.R1 !== 16'b0) begin
    //     err = 1;
    //     $display("R1 when using AND is incorrect");
    // end else begin
    //     $display("HOORAY!!! R1 when using AND is correct!");
    // end
    // Auto(16'b1011100000000001); //Put NOT R1(All zeros) in R0
    // #200;
    // if (DUT.DP.REGFILE.R0 !== {16{1'b1}}) begin
    //     err = 1;
    //     $display("R0 should have all 1's and doesn't when using MVN");
    // end else begin
    //     $display("OH WONDROUS DAY! R0 has all 1's using MVN!");
    // end
    // Auto(16'b1011100010110011); //Put NOT(R3 shifted right by 1) in R5
    // #200;
    // if (DUT.DP.REGFILE.R5 !== 16'b1111111111111110) begin
    //     err = 1;
    //     $display("R5 is incorrect - Check please");
    // end else begin
    //     $display("OH BY JOVE! R5 has the correct value when using MVN on a shifted value");
    // end
    // Auto(16'b1011100010100101); //Put NOT(R5) into R5
    // #200;
    // if (DUT.DP.REGFILE.R5 !== 16'b1) begin
    //     err = 1;
    //     $display("Using NOT with same destination and Rm is incorrect - Check R5");
    // end else begin
    //     $display("HUZZAH! R5 has the correct value in it when using MVN on itself!");
    // end
    // $stop;
end 
endmodule