module ALU_tb();
    reg [15:0] Ain, Bin;
    reg [1:0] ALUop;
    wire [15:0] out;
    wire Z;
    reg err = 0;

    ALU DUT (.Ain(Ain), .Bin(Bin), .ALUop(ALUop), .out(out), .Z(Z));

    task alu_checker;

    input [15:0] Asig, Bsig;
    input [1:0] ALUopSig;
    input [15:0] expected_output;
    input expected_z;

    Ain = Asig;
    Bin = Bsig;
    ALUop = ALUopSig; 
    #5;

    if (out !== expected_output && Z !== expected_z) begin
        $display("**ERROR** nothing is right... (out, Z) is %b, %b and should be %b, %b", out, Z, expected_output, expected_z);
        err = 1;
    end else if (out !== expected_output) begin
        $display("*ERROR* output is incorrect but Z is correct. Out is %b and should be %b", out, expected_output);
    end else if (Z !== expected_z) begin
        $display("*ERROR* Z is incorrect but output is correct. Z is %b and should be %b", Z, expected_z);
    end else begin
        $display("SUCCESS! output and Z are correct! (out, Z) = %b, %b", out, Z);
    end
    endtask

    initial begin
        alu_checker(16'b1, 16'b1, 2'b00, 16'b10, 1'b0); // 1+1 = 2 Z = 0
        alu_checker(16'b111, 16'b1, 2'b00, 16'b1000, 1'b0); //7+1 = 8 Z = 0
        alu_checker(16'b111, 16'b1, 2'b01, 16'b110, 1'b0); //7-1 = 6 Z = 0
        alu_checker(16'b1, 16'b1, 2'b10, 16'b1, 1'b0); //1&1 = 1
        alu_checker(16'b0, 16'b1, 2'b10, 16'b0, 1'b1); //0&1 = 0 Z = 1
        alu_checker(16'b1101, 16'b0101, 2'b10, 16'b0101, 1'b0); //1101&0101 = 0101 Z = 0
        alu_checker(16'b1000000000000001, 16'b1000000000000000, 2'b10, 16'b1000000000000000, 1'b0); //Result should be 1000000000000000, Z = 0;
        alu_checker(16'b0, 16'b1, 2'b11, 16'b1111111111111110, 1'b0); 
        alu_checker(16'b111, 16'b11101, 2'b11, 16'b1111111111100010, 1'b0); //~11101 = 00010, Z = 0
        alu_checker(16'b111, 16'b1111111111111111, 2'b11, 16'b0, 1'b1); //~1111111111111111 = 0, Z = 1
        alu_checker(16'b111010101, 16'b0000000000000000, 2'b11, 16'b1111111111111111, 1'b0); //~0000000000000000 = 1111111111111111, Z = 0
    end


endmodule
