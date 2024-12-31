module datapath_tb();
reg clk;
reg write, loada, loadb, asel, bsel, loadc, loads;
reg [2:0] readnum, writenum;
reg [1:0] shift, ALUop, vsel; //vsel changed here
reg err = 0;

//Stage 1 changes:
reg [15:0] mdata;
reg [15:0] sximm8;
reg [7:0] PC;
reg [15:0] sximm5;

wire [15:0] datapath_out;

//More output changes:
wire N,V,Z;

datapath DUT(.clk(clk), .mdata({16'b0}), .sximm8(sximm8), .sximm5(sximm5), .PC({8'b0}), .write(write), .vsel(vsel), .loada(loada), .loadb(loadb), .asel(asel), .bsel(bsel), .loadc(loadc), .loads(loads), .writenum(writenum), .readnum(readnum), .shift(shift), .ALUop(ALUop), .datapath_out(datapath_out), .N(N), .V(V), .Z(Z));

task mov;

    input [15:0] data_in;
    input [2:0] writenum_in;

    vsel = 2'b10;
    #2;
    sximm8 = data_in;
    #2;
    write = 1'b1;
    #2;
    writenum = writenum_in;
    #2;

    clk = 1;
    #5;
    clk = 0;

endtask

task load_into_a;

    input [2:0] reg2read;

    readnum = reg2read;
    write = 1'b0;
    loada = 1'b1;
    #5;
    clk = 1;
    #5;
    clk = 0;
    loada = 1'b0;

endtask

task load_into_b;

    input [2:0] reg3read;

    readnum = reg3read;
    write = 1'b0;
    loadb = 1'b1;
    #5;
    clk = 1;
    #5;
    clk = 0;
    loadb = 1'b0;
    
endtask

task arithmetic;

    input [1:0] shift2;
    input [1:0] expression;
    input [15:0] expected_data_out;
    input asel_in, bsel_in;
    input expected_z, expected_n, expected_v;
    #5;
    asel = asel_in;
    bsel = bsel_in; 
    shift = shift2;
    #5;
    ALUop = expression;
    #5;
    loadc = 1;
    loads = 1;

    #5;
    clk = 1;
    #5;
    clk = 0;
    #5;

    if (datapath_out !== expected_data_out && (Z !== expected_z || V !== expected_v || N !== expected_n)) begin
        $display("**ERROR** output is %b and should be %b, also *Error* Z, V, or N aren't right. Z: %b should be %b, V: %b should be %b, N: %b, should be %b", datapath_out, expected_data_out, Z, expected_z, V, expected_v, N, expected_n);
        err = 1;
    end else if (Z !== expected_z || V !== expected_v || N !== expected_n)begin
        err = 1;
        $display("*Error* Z, V, or N aren't right. Z: %b should be %b, V: %b should be %b, N: %b, should be %b", Z, expected_z, V, expected_v, N, expected_n);
    end else begin
        $display("SUCCESS! output change to %b works", datapath_out);
    end

endtask

task load_back;

    input [2:0] write_into;
    vsel = 2'b00;
    write = 1;

    writenum = write_into;
    #5;
    clk = 1;
    #5;
    clk = 0;


endtask

initial begin
    mov(16'b111, 3'b000); //Operations from Lab Manual, Minimum
    mov(16'b10, 3'b001);
    load_into_b(3'b0);
    load_into_a(3'b1);
    arithmetic(2'b01, 2'b00, 16'b10000, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0);
    load_back(3'b010);

    mov(16'b101010, 3'b011); //Operations from Lab 5 Powerpoint
    mov(16'b1101, 3'b101); //MOV R5, #13
    load_into_b(3'b011); //Load R3 into B
    load_into_a(3'b101); //Load R5 into A
    arithmetic(2'b00, 2'b00, 16'b110111, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0); //Add R5 and R3
    load_back(3'b010); //Completes ADD R2, R5, R3
    load_into_b(3'b011); //Put 42 into R3
    arithmetic(2'b00, 2'b00, 16'b101010, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0); //Move 42 into LoadC
    load_back(3'b111); //Load R3 into R7

    //Checking N V and Z values
    mov (16'b0100111000100000, 3'b000); //Add 20,000 to 20,000 Causes Overflow and Negative to be active
    mov(16'b0100111000100000, 3'b001);
    load_into_b(3'b0);
    load_into_a(3'b1);
    arithmetic(2'b00, 2'b00, 16'b1100111001000000, 1'b0, 1'b0, 1'b0, 1'b1, 1'b1);
    load_back(3'b010);
end

endmodule
