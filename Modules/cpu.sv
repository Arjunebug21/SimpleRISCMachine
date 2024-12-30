module cpu(clk,reset,out,N,V,Z, mem_cmd, mem_addr, read_data);
input clk, reset;
output [15:0] out;
output N, V, Z;

//New lab7 vars
wire load_pc, reset_pc, load_ir, addr_sel;
output [1:0] mem_cmd; //Memory command output is 2 bits wide
output [8:0] mem_addr; //Memory address output is 9 bits wide
input [15:0] read_data; //Read data input is 16 bits wide and goes to instruction register

//Instruction Register
wire [15:0] instr_wire;
wire load_addr;
load_enable_cpu instruction_reg(.in(read_data), .load(load_ir), .clk(clk), .out(instr_wire)); //Changed in to read_data and load to load_ir

//Instruction decoder

//To FSM
wire [2:0] opcode;
wire [1:0] op;
//From FSM
wire [1:0] nsel;
//To datapath
wire [15:0] sximm5_num;
wire [15:0] sximm8_num;
wire [1:0] shift_num;
wire [2:0] read_reg;
wire [2:0] write_reg;
wire write, loada, loadb, asel, bsel, loadc, loads;
wire [1:0] vsel;



i_decoder instruction_decoder(.in(instr_wire), .nsel(nsel), .opcode(opcode), .op(op), .sximm5(sximm5_num), .sximm8(sximm8_num), .shift(shift_num), .readnum(read_reg), .writenum(write_reg));

FSM_controller controlla(.reset(reset), .clk(clk), .opcode(opcode), .op(op),
                        .nsel(nsel), .loada(loada), .loadb(loadb), 
                        .asel(asel), .bsel(bsel), .loads(loads), 
                        .loadc(loadc), .write(write), .vsel(vsel), 
                        .load_pc(load_pc), .reset_pc(reset_pc), 
                        .load_ir(load_ir), .addr_sel(addr_sel), .mem_cmd(mem_cmd), .load_addr(load_addr));

datapath DP(.clk(clk), .readnum(read_reg), .vsel(vsel), .mdata(read_data), 
            .sximm8(sximm8_num), .sximm5(sximm5_num), .PC(8'b0), .loada(loada), 
            .loadb(loadb), .shift(shift_num), .asel(asel), .bsel(bsel), 
            .ALUop(op), .loadc(loadc), .loads(loads), .writenum(write_reg), 
            .write(write), .N(N), .V(V), .Z(Z), .datapath_out(out));

//New from Lab 7
wire [8:0] PC, next_pc;
wire [8:0] curr_data_addr;

program_counter PC_reg(.in(next_pc), .load(load_pc), .clk(clk), .out(PC)); //PC itself
assign next_pc = reset_pc ? 9'b0: (PC + 9'b1); //Reset PC MUX and Adder

//addr_sel MUX
assign mem_addr = addr_sel ? PC : curr_data_addr;

//Data address register
load_enable_data_addr ld_en_data_address(.in(out[8:0]), .load(load_addr), .clk(clk), .out(curr_data_addr));


endmodule

module program_counter(in, load, clk, out);
    input [8:0] in;
    input load, clk;
    output reg [8:0] out;
    wire [8:0] next_output;

    assign next_output = load ? in: out;

    always_ff @(posedge clk) begin
        out <= next_output;
    end

endmodule

module load_enable_data_addr(in, load, clk, out);
input [8:0] in;
input load, clk;
output reg [8:0] out;
wire [8:0] next_output;

assign next_output = load ? in: out;

always_ff @(posedge clk) begin
    out <= next_output;
end
endmodule

module load_enable_cpu(in, load, clk, out);
input [15:0] in;
input load, clk;
output reg [15:0] out;
wire [15:0] next_output;

assign next_output = load ? in: out;

always_ff @(posedge clk) begin
    out <= next_output;
end

endmodule

module i_decoder(in, nsel, opcode, op, sximm5, sximm8, shift, readnum, writenum);
input [15:0] in;
input [1:0] nsel; //From FSM
output [2:0] opcode;
output [1:0] op; //op goes to FSM and datapath for ALU
output [15:0] sximm8, sximm5;
output [2:0] readnum, writenum;
output [1:0] shift;

sign_extend5 s5(in[4:0], sximm5); //Sign extend for sximm5 and 8
sign_extend8 s8(in[7:0], sximm8);

//Mux for which registers to use
reg [2:0] reg_num;
wire [2:0] Rn, Rd, Rm;
assign Rn = in[10:8];
assign Rd = in[7:5];
assign Rm = in[2:0];
always_comb begin
    case (nsel)
    2'b00: reg_num = Rn; //If nsel is 0 choose Rn
    2'b01: reg_num = Rd; //If nsel is 1 choose Rd
    2'b10: reg_num = Rm; //if nsel is 2 choose Rm
    default: reg_num = 3'bx;
    endcase
end

assign readnum = reg_num;
assign writenum = reg_num;
//Done Mux

//Shift
assign shift = in[4:3];
//Done Shift

//To FSM
assign opcode = in[15:13];
assign op = in[12:11];
//Done to FSM
endmodule

module sign_extend5(in, out);
input [4:0] in;
output reg [15:0] out;

assign out = {{11{in[4]}}, in};

endmodule

module sign_extend8(in, out);
input [7:0] in;
output reg [15:0] out;

assign out = {{8{in[7]}}, in};
endmodule

module FSM_controller(reset, clk, opcode, op, nsel, loada, loadb, asel, bsel, loads, loadc, write, vsel, load_pc, reset_pc, load_ir, addr_sel, mem_cmd, load_addr);
input reset, clk;
input [2:0] opcode;
input [1:0] op;
output reg [1:0] nsel, vsel;
output reg loada, loadb, loadc, loads, asel, bsel, write;


//New from Lab 7
output reg load_pc, reset_pc, load_ir, addr_sel, load_addr;
output reg [1:0] mem_cmd;


reg [4:0] present_state, next_state, next_state_reset;

`define RST 5'b00000
`define IF1 5'b10001
`define IF2 5'b10010
`define UpdatePC 5'b10011
`define DECODE 5'b00001
`define MOV 5'b00010
`define MOV_imm 5'b00011
`define MOV_reg1 5'b00100
`define MOV_reg2 5'b00101
`define MOV_reg3 5'b00111
`define ALU 5'b01000
`define ADD1 5'b01001
`define ADD2 5'b01010
`define ADD3 5'b01011
`define ADD4 5'b01111
`define CMP 5'b10000
`define LDR2 5'b10100 
`define LDR3 5'b10101 
`define LDR4 5'b10110 
`define STR3 5'b10111 
`define STR4 5'b11000 
`define STR5 5'b11001 
`define HALT 5'b11010 
`define LDR5 5'b11011 
`define STR6 5'b11100 


//Mem commands
`define MNONE 2'b00
`define MREAD 2'b01
`define MWRITE 2'b10

//Mux for reset
assign next_state_reset = reset ? `RST : next_state;

my_vDFF #(5) flip_flop(.clk(clk), .in(next_state_reset), .out(present_state));

always_comb begin
    write = 0;
    loada = 0;
    loadb = 0;
    loadc = 0;
    loads = 0;
    asel = 0;
    bsel = 0;
    load_pc = 0;
    load_ir = 0;
    reset_pc = 0;
    addr_sel = 0;
    nsel = 2'b00;
    vsel = 2'b00;
    next_state = `RST;
    mem_cmd = `MREAD;
    load_addr = 0;
    case (present_state)
    `RST: begin //Haven't changed the states yet
        write = 0;
        loada = 0;
        loadb = 0;
        loadc = 0;
        loads = 0;
        load_pc = 1;
        load_ir = 0;
        reset_pc = 1;
        addr_sel = 0;
        load_addr = 0;
        next_state = `IF1;
    end
    `IF1: begin
        loada = 0;
        loadb = 0;
        loadc = 0;
        loads = 0;
        write = 0;
        load_pc = 0;
        reset_pc = 0;
        addr_sel = 1;
        load_addr = 0;
        mem_cmd = `MREAD; //Do I need to change this eventually?
        next_state = `IF2;
    end
    `IF2: begin
        addr_sel = 1;
        load_ir = 1;
        mem_cmd = `MREAD; //Do I need to change this eventually?
        next_state = `UpdatePC; 
    end
    `UpdatePC: begin
        load_ir = 0;
        load_pc = 1;
        next_state = `DECODE;
    end
    `DECODE: begin //Change this portion when adding new instructions
        if (opcode === 3'b110)
        next_state = `MOV;
        else if (opcode === 3'b111)
        next_state = `HALT;
        else
        next_state = `ALU;
    end
    `HALT: begin
        load_pc = 0;
        next_state = `HALT;
    end
    `MOV: begin //Change this when you add a second MOV instruction
        if (op === 2'b10)
            next_state = `MOV_imm;
        else
            next_state = `MOV_reg1;
    end
    `MOV_imm: begin //Mov immediate 8 state
        vsel = 2'b10;
        nsel = 2'b00;
        write = 1;
        loada = 0;
        loadb = 0;
        loadc = 0;
        loads = 0;
        next_state = `IF1;
    end
    `MOV_reg1: begin //Start of Moving from Reg to reg
        write = 0;
        nsel = 2'b10;
        loada = 0;
        loadb = 1;
        loadc = 0;
        loads = 0;
        next_state = `MOV_reg2;
    end
    `MOV_reg2: begin
        loada = 0;
        loadb = 0;
        asel = 1;
        bsel = 0;
        loadc = 1;
        next_state = `MOV_reg3;
    end
    `MOV_reg3: begin
        loadc = 0;
        loads = 0;
        nsel = 2'b01;
        write = 1;
        vsel = 2'b00;
        next_state = `IF1;
    end
    `ALU: begin
        if (opcode === 3'b011 || opcode === 3'b100)
        next_state = `ADD2;
        else
        next_state = `ADD1; //I'm using the same states for many ops
    end
    `ADD1: begin
        nsel = 2'b10;
        loadb = 1;
        loada = 0;
        loadc = 0;
        loads = 0;
        write = 0;
        if (op === 2'b11)
        next_state = `ADD3;
        else
        next_state = `ADD2;
    end
    `ADD2: begin
        nsel = 2'b00;
        loadc = 0;
        loads = 0;
        loadb = 0;
        loada = 1;
        if (opcode === 3'b011 || opcode === 3'b100)
        next_state = `LDR2;
        else if (op === 2'b01)
        next_state = `CMP;
        else if (op === 2'b00 || op === 2'b10)
        next_state = `ADD3;
    end
    `ADD3: begin
        loada = 0;
        loadb = 0;
        asel = 0;
        bsel = 0;
        loadc = 1;
        next_state = `ADD4;
    end
    `ADD4: begin
        loadc = 0;
        vsel = 2'b00;
        write = 1;
        nsel = 2'b01;
        next_state = `IF1;
    end
    `CMP: begin
        asel = 0;
        bsel = 0;
        loadc = 0;
        loads = 1;
        next_state = `IF1;
    end
    `LDR2: begin
        loada = 1'b0;
        loadb = 1'b0;
        loads = 1'b0;
        bsel = 1'b1;
        asel = 1'b0;
        loadc = 1'b1;
        if (opcode === 3'b100)
        next_state = `STR3;
        else
        next_state = `LDR3;
    end
    `LDR3: begin
        load_addr = 1;
        addr_sel = 0;
        mem_cmd = `MREAD;
        next_state = `LDR4;
    end
    `LDR4: begin
        next_state = `LDR5;
    end
    `LDR5: begin
        vsel = 2'b11;
        nsel = 2'b01;
        write = 1'b1;
        loada = 0;
        loadb = 0;
        loadc = 0;
        loads = 0;
        next_state = `IF1;
    end
    `STR3: begin
        load_addr = 1;
        loadc = 0;
        next_state = `STR4;
    end
    `STR4: begin
        load_addr = 0;
        write = 0;
        nsel = 2'b01;
        loadb = 1;
        loada = 0;
        loadc = 0;
        loads = 0;
        next_state = `STR5;
    end
    `STR5: begin
        asel = 1;
        bsel = 0;
        loadc = 1;
        loadb = 0;
        next_state = `STR6;
    end
    `STR6: begin
        addr_sel = 0;
        mem_cmd = `MWRITE;
        loadc = 0;
        load_addr = 1;
        next_state = `IF1;
    end
    default: next_state = 16'bx;
    endcase
end

endmodule

module my_vDFF(clk, in, out) ;
  parameter n = 1;  // width
  input clk ;
  input [n-1:0] in ;
  output [n-1:0] out ;
  reg [n-1:0] out ;

  always_ff @(posedge clk)
    out = in ;
endmodule 
