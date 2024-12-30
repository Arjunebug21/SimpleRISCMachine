module datapath(clk, readnum, vsel, mdata, sximm8, sximm5, PC, loada, loadb, shift, asel, bsel, ALUop, loadc, loads, writenum, write, N, V, Z, datapath_out);
input clk;
//input [15:0] datapath_in;
input write, loada, loadb, asel, bsel, loadc, loads;
input [2:0] readnum, writenum;
input [1:0] shift, ALUop;

//New Inputs from Lab 6
input [1:0] vsel;
input [15:0] mdata;
input [15:0] sximm8;
input [7:0] PC;
input [15:0] sximm5;

//New outputs
output Z, N, V;

output [15:0] datapath_out;


reg [15:0] reg_data_in;
wire [15:0] reg_data_out;

always_comb begin
    case(vsel)
    2'b11: reg_data_in = mdata;
    2'b10: reg_data_in = sximm8;
    2'b01: reg_data_in = {8'b0, PC};
    2'b00: reg_data_in = datapath_out;
    endcase
end

regfile REGFILE (.data_in(reg_data_in), .writenum(writenum), .write(write), .readnum(readnum), .clk(clk), .data_out(reg_data_out));

wire [15:0] loada_out, loadb_out;

load_en loadA(.in(reg_data_out), .load(loada), .clk(clk), .out(loada_out));
load_en loadB(.in(reg_data_out), .load(loadb), .clk(clk), .out(loadb_out));

wire [15:0] sout;

shifter SHIFT(.in(loadb_out), .shift(shift), .sout(sout));

wire [15:0] Ain, Bin;

assign Ain = asel ? 16'b0: loada_out;
assign Bin = bsel ? sximm5 : sout;

wire [15:0] ALUout;
wire Z_o, N_o, V_o;

ALU a_l_u (.Ain(Ain), .Bin(Bin), .ALUop(ALUop), .out(ALUout), .N_sig(N_o), .V_sig(V_o), .Z_sig(Z_o));

load_enNVZ status(.in({N_o, V_o, Z_o}), .load(loads), .clk(clk), .out({N, V, Z}));

load_en loadC(.in(ALUout), .load(loadc), .clk(clk), .out(datapath_out));

endmodule

module load_en(in, load, clk, out);
input [15:0] in;
input load, clk;
output reg [15:0] out;
wire [15:0] next_output;

assign next_output = load ? in: out;

always_ff @(posedge clk) begin
    out <= next_output;
end

endmodule

module load_enNVZ(in, load, clk, out);
input [2:0] in;
input load, clk;
output reg [2:0] out;
reg [2:0] next_output;

assign next_output = load ? in: out;

always_ff @(posedge clk) begin
    out <= next_output;
end

endmodule
